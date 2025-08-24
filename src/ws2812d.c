/*
 * ws2812d.c - rpi-ws2812d WS2812 LED control daemon using shared memory
 *
 * SPDX-License-Identifier: GPL-3.0
 *
 * Copyright (C) 2025 KaliAssistant
 * Author: KaliAssistant
 *
 * This file is part of the rpi-ws2812d project and is licensed under the GNU
 * General Public License v3.0 or later.
 *
 * rpi-ws2812d is a lightweight userspace daemon designed to control WS2812 addressable RGB LEDs
 * on Raspberry Pi using shared memory. It enables external applications to update LED data
 * by writing to a shared memory region, allowing for flexible and fast integration in
 * embedded or headless systems.
 *
 * Features:
 *   - Controls WS2812/NeoPixel LEDs on Raspberry Pi using the rpi_ws281x library.
 *   - Reads RGB LED color data from shared memory to allow external updates.
 *   - Supports power control of LEDs via configurable GPIO power pin.
 *   - Configurable LED count, data pin, power pin, and polling delay via config file.
 *   - Prevents multiple instances by PID lock file.
 *   - Supports daemon mode for background operation.
 *   - Uses Linux GPIO character device API for power pin control.
 *
 * Project GitHub: https://github.com/KaliAssistant/rpi-ws2812d
 *
 * This project includes components under the following licenses:
 *   - inih: BSD-3-Clause (https://github.com/benhoyt/inih)
 *   - rpi_ws281x: BSD-2-Clause (https://github.com/jgarff/rpi_ws281x)
 *
 * See LICENSE for more details.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <inttypes.h>
#include <signal.h>
#include <errno.h>
#include <sys/mman.h>
#include <sys/file.h>
#include <linux/gpio.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <getopt.h>
#include "ini.h"
#include "utils.h"
#include "xstrconv.h"
//#include "version.h"
#include "config.h"

#define SHM_FILE            "/led_rgb"
#define LOCK_FILE           "/var/run/ws2812.lock"
#define WS2812D_CONF_FILE   "/etc/ws2812d/ws2812d.conf"
#define POWER_GPIOCHIP      "/dev/gpiochip0"
#define SPI_DEV             "/dev/spidev0.0"

#define LED_COUNT 1

#define DEFAULT_CONF_PWR_GPIO     16
#define DEFAULT_CONF_DELAY_US     2000
#define DEFAULT_CONF_SPI_FREQ     2400000

static int led_count = LED_COUNT;
static int is_daemon = 0;
static char *ws2812d_conf_file = WS2812D_CONF_FILE;

typedef struct ws2812d_conf {
    int power_pin;
    uint32_t leds_count;
    uint32_t spi_freq;
    uint32_t delay_us;
}ws2812d_conf;

static int lock_fd = -1;
static int shm_fd = -1;
static uint8_t *shm_ptr = NULL;
static uint8_t *last_shm_ptr = NULL;
static uint8_t *spi_buf_ptr = NULL;
static int gpio_fd = -1;
static int gpio_line_fd = -1;
static int spi_fd = -1;

static ws2812d_conf ws2812d_default_conf = {
    .power_pin = DEFAULT_CONF_PWR_GPIO,
    .spi_freq = DEFAULT_CONF_SPI_FREQ, 
    .delay_us = DEFAULT_CONF_DELAY_US,
};

static const int available_pwr_gpio[] = {
     2,  3,  4,  5,  6,  7,  8,  9, 
    11, 14, 15, 16, 17, 20, 22, 23,
    24, 25, 26, 27,
};

// Lookup table: 1 WS2812 bit -> 3 SPI bits (encoded as a byte for convenience)
static const uint8_t ws2812_lookup[2] = { 0b100, 0b110 };

// Precompute 256 entries (3 SPI bytes per WS2812 byte)
static uint8_t ws2812_table[256][3];

static void build_table() {
    for (int val = 0; val < 256; val++) {
        uint32_t out = 0;
        for (int bit = 7; bit >= 0; --bit) {
            uint8_t b = (val >> bit) & 1;
            uint8_t pat = ws2812_lookup[b];
            out = (out << 3) | pat;
        }
        ws2812_table[val][0] = (out >> 16) & 0xFF;
        ws2812_table[val][1] = (out >> 8) & 0xFF;
        ws2812_table[val][2] = out & 0xFF;
    }
}

static int encode_leds(const uint8_t *grb, uint8_t *_spibuf, size_t _led_count) {
    if (!grb) {
        errno = EFAULT;
        perror("data.leds_encoder.got_null_grb_data");
        return -1;
    }
    if (!_spibuf) {
        errno = EFAULT;
        perror("data.leds_encoder.got_null_spibuf");
        return -1;
    }
    
    for (size_t i = 0; i < _led_count; i++) {
        memcpy(_spibuf, ws2812_table[grb[i*3 + 1]], 3); _spibuf += 3; // R
        memcpy(_spibuf, ws2812_table[grb[i*3 + 0]], 3); _spibuf += 3; // G
        memcpy(_spibuf, ws2812_table[grb[i*3 + 2]], 3); _spibuf += 3; // B
    }
    return 0;
}


static int conf_handler(void *user, const char *section, const char *name, const char *value) {
    ws2812d_conf *config = (ws2812d_conf *)user;
    #define CONF_MATCH(s, n) strcmp(section, s) == 0 && strcmp(name, n) == 0
    if (CONF_MATCH("gpio", "power_pin")) {
        return xstr2i32(value, 10, &config->power_pin);
    } else if (CONF_MATCH("spi", "spi_freq")) {
        return xstr2u32(value, 10, &config->spi_freq);
    } else if (CONF_MATCH("user", "leds_count")) {
        return xstr2u32(value, 10, &config->leds_count);
    } else if (CONF_MATCH("user", "delay_us")) {
        return xstr2u32(value, 10, &config->delay_us);
    } else {
        return 0;
    }
    return 1;
}

static int conf_checker(const ws2812d_conf *conf) {
    if (!conf) {
        errno = EFAULT;
        perror("conf.ini_checker.got_null_conf");
        abort();
    }
    
    if (!int_in_list(conf->power_pin, available_pwr_gpio, sizeof(available_pwr_gpio)/sizeof(int))) {
        fprintf(stderr, "conf.ini_checker.invalid_config: invalid power pin: %d\n", conf->power_pin);
        return -1;
    }

    led_count = conf->leds_count;
    
    return 0;
}

static void cleanup()
{
    if (last_shm_ptr) free(last_shm_ptr);
    if (spi_buf_ptr) free(spi_buf_ptr);

    if (spi_fd >= 0)
        close(spi_fd);

    // Turn off power pin
    if (gpio_line_fd >= 0) {
        struct gpiohandle_data data = { .values[0] = 0 };
        ioctl(gpio_line_fd, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data);
        close(gpio_line_fd);
    }

    if (gpio_fd >= 0)
        close(gpio_fd);

    if (shm_ptr)
        munmap(shm_ptr, led_count * 3);
    if (shm_fd >= 0) {
        close(shm_fd);
        shm_unlink(SHM_FILE);
    }
    if (lock_fd >= 0) {
        flock(lock_fd, LOCK_UN);
        close(lock_fd);
    }
}

static void signal_handler(int signum)
{
    (void)signum;
    cleanup();
    exit(0);
}

static int setup_spi_dev() {
    spi_fd = open(SPI_DEV, O_RDWR);
    if (spi_fd < 0) {
        perror("spi.spidev.cannot_open_device");
        return -1;
    }

    const uint8_t mode = SPI_MODE_0;
    const uint8_t bits = 8;
    const uint32_t speed = ws2812d_default_conf.spi_freq;
    
    if (ioctl(spi_fd, SPI_IOC_WR_MODE, &mode) < 0 ||
        ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 ||
        ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
        perror("spi.spidev.configure_failed");
        close(spi_fd);
        return -1;
    }
    
    return 0;
}


static int setup_gpio_power()
{
    gpio_fd = open(POWER_GPIOCHIP, O_RDONLY);
    if (gpio_fd < 0) {
        perror("gpio.power_pin.cannot_open_gpiochip");
        return -1;
    }

    struct gpiohandle_request req = {0};
    req.lineoffsets[0] = ws2812d_default_conf.power_pin;
    req.flags = GPIOHANDLE_REQUEST_OUTPUT;
    req.default_values[0] = 1;  // turn power on
    req.lines = 1;

    if (ioctl(gpio_fd, GPIO_GET_LINEHANDLE_IOCTL, &req) < 0) {
        perror("gpio.power_pin.get_linehandle_ioctl_failed");
        return -1;
    }

    gpio_line_fd = req.fd;
    return 0;
}


static void usage(const char *prog_name) {
    if (!prog_name) return;
    fprintf(stderr, "ws2812d - rpi-ws2812d WS281x RGB NeoPixel daemon for raspberry pi\n\n");
    fprintf(stderr, "Usage: %s -c <config file> [-Dhv]\n\n", prog_name);
    fprintf(stderr, "-c :\t<ws2812d.conf>, ws2812d config file, default is '/etc/ws2812d/ws2812d.conf'\n");
    fprintf(stderr, "-D :\trun as daemon mode (SysVinit)\n");
    fprintf(stderr, "-h :\tshow this help\n");
    fprintf(stderr, "-v :\tshow version\n\n");
    fprintf(stderr, "Version %s By KaliAssistant\n", VERSION);
    fprintf(stderr, "Github: https://github.com/KaliAssistant/rpi-ws2812d.git\n");
    return;
}

int main(int argc, char **argv) {
    
    int opt;
    while ((opt = getopt(argc, argv, "c:Dhv")) != -1) {
        switch (opt) {
            case 'h':
                usage(argv[0]);
                return 1;
            case 'v':
                fprintf(stdout, "%s\n", VERSION);
                return 0;
            case 'c':
                ws2812d_conf_file = optarg;
                break;
            case 'D':
                is_daemon = 1;
                break;
            case '?':
                fprintf(stderr, "See '%s -h' for help.\n", argv[0]);
                return 1;
            default:
                errno = EFAULT;
                perror("main.getopt.got_impossible_default");
                abort();
        }
    }
    
    for (int i = optind; i < argc; i++) {
        fprintf(stderr, "main.getopt.got_non_option_warning: got non-option argument '%s'.\n", argv[i]);
    }

    int ini_parse_err = ini_parse(ws2812d_conf_file, conf_handler, &ws2812d_default_conf);
    if ( ini_parse_err < 0) {
        perror("main.conf_parse.cannot_load_conf");
        return 1;
    } else if (ini_parse_err) {
        fprintf(stderr, "main.conf_parse.bad_conf_file: bad config file (first error on line %d).\n", ini_parse_err);
        return 1;
    }

    if (conf_checker(&ws2812d_default_conf) < 0) {
        fprintf(stderr, "main.conf_parse.conf_checker_error: invalid configuration.\n");
        return 1;
    }

    
    lock_fd = open(LOCK_FILE, O_CREAT | O_RDWR, 0644);

    if (lock_fd < 0) {
        perror("main.process.cannot_open_lock_file");
        return 1;
    }

    if (flock(lock_fd, LOCK_EX | LOCK_NB) < 0) {
        if (errno == EWOULDBLOCK) {
            fprintf(stderr, "main.process.flock_error: another instance is already running.\n");
            return 1;
        } else {
            perror("main.process.flock_error");
            return 1;
        }
    }

    if (is_daemon) {
        pid_t pid = fork();
        if (pid < 0) {
            perror("main.process.daemon_fork_failed");
            return 1;
        }
        if (pid > 0) return 0;
        if (setsid() < 0) {
            perror("main.process.daemon_setsid_failed");
            return 1;
        }
        close(0); close(1); close(2);
        open("/dev/null", O_RDONLY);
        open("/dev/null", O_WRONLY);
        open("/dev/null", O_RDWR);
        chdir("/");
    }
    ftruncate(lock_fd, 0);
    dprintf(lock_fd, "%d\n", getpid());

    build_table();

    


    if (setup_gpio_power() < 0) {
        fprintf(stderr, "main.process.setup_gpio_power: cannot setup power_gpio.\n");
        return 1;
    }

    if (setup_spi_dev() < 0) {
        fprintf(stderr, "main.process.setup_spi_dev: cannot setup spidev.\n");
        return 1;
    }

    shm_fd = shm_open(SHM_FILE, O_RDWR | O_CREAT, 0666);
    if (shm_fd < 0) {
        perror("main.process.cannot_open_shm_file");
        cleanup();
        return 1;
    }
    ftruncate(shm_fd, led_count * 3); // or whatever size you actually need
    shm_ptr = mmap(NULL, led_count * 3, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (shm_ptr == MAP_FAILED) {
        perror("main.process.mmap_failed");
        cleanup();
        return 1;
    }

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    last_shm_ptr = malloc(led_count * 3 * sizeof(uint8_t));
    spi_buf_ptr = malloc(led_count * 9 * sizeof(uint8_t));
    
    memset(last_shm_ptr, 0xFF, led_count * 3 * sizeof(uint8_t));

    while (1) {
        if (memcmp(shm_ptr, last_shm_ptr, led_count * 3) != 0) {
            memcpy(last_shm_ptr, shm_ptr, led_count * 3);
            encode_leds(shm_ptr, spi_buf_ptr, led_count);
            ssize_t ret = write(spi_fd, spi_buf_ptr, led_count * 9);
            if (ret < 0) {
                perror("main.process.spi_write_failed");
                break;
            }
        }
        usleep(ws2812d_default_conf.delay_us);
    }
    cleanup();
    return 0;
}
