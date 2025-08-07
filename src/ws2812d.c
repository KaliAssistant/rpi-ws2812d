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
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <getopt.h>
#include <ws2811/ws2811.h>
#include "ini.h"
#include "utils.h"
//#include "version.h"
#include "config.h"

#define SHM_FILE "/led_rgb"
#define LOCK_FILE "/var/run/ws2812.lock"
#define WS2812D_CONF_FILE "/etc/ws2812d/ws2812d.conf"
#define POWER_GPIOCHIP "/dev/gpiochip0"

#define WS2812_STRIP_TYPE WS2811_STRIP_GRB
#define LED_COUNT 1
#define DMA 10

#define DEFAULT_CONF_PWR_GPIO 16
#define DEFAULT_CONF_DATA_GPIO 21
#define DEFAULT_CONF_DELAY_US 2000

#define TARGET_FREQ WS2811_TARGET_FREQ

//static volatile int keep_running = 1;

static int led_count = LED_COUNT;
static int is_daemon = 0;
static char *ws2812d_conf_file = WS2812D_CONF_FILE;

static ws2811_t ledstring =
{
    .freq = TARGET_FREQ,
    .dmanum = DMA,
    .channel =
    {
        [0] = {
            .gpionum = DEFAULT_CONF_DATA_GPIO,
            .count = LED_COUNT,
            .invert = 0,
            .strip_type = WS2812_STRIP_TYPE,
            .brightness = 255,
        },
        [1] = {
            .gpionum = 0,
            .count = 0,
            .invert = 0,
            .brightness = 0,
        },
    },
};

typedef struct ws2812d_conf {
    int power_pin;
    int data_pin;
    uintmax_t delay_us;
}ws2812d_conf;

static int lock_fd = -1;
static int shm_fd = -1;
static uint8_t *shm_ptr = NULL;
static uint8_t *last_shm_ptr = NULL;
static int gpio_fd = -1;
static int gpio_line_fd = -1;

static ws2812d_conf ws2812d_default_conf = {
    .power_pin = DEFAULT_CONF_PWR_GPIO,
    .data_pin = DEFAULT_CONF_DATA_GPIO,
    .delay_us = DEFAULT_CONF_DELAY_US,
};

static const int available_data_gpio[] = {
    10, 12, 13, 18, 19, 21,
};

static const int available_pwr_gpio[] = {
     2,  3,  4,  5,  6,  7,  8,  9, 
    11, 14, 15, 16, 17, 20, 22, 23,
    24, 25, 26, 27,
};

static int conf_handler(void *user, const char *section, const char *name, const char *value) {
    ws2812d_conf *config = (ws2812d_conf *)user;
    #define CONF_MATCH(s, n) strcmp(section, s) == 0 && strcmp(name, n) == 0
    if (CONF_MATCH("gpio", "power_pin")) {
        config->power_pin = atoi(value);
    } else if (CONF_MATCH("gpio", "data_pin")) {
        config->data_pin = atoi(value);
    } else if (CONF_MATCH("user", "delay_us")) {
        return xstr2umax(value, 10, &config->delay_us);
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
    if (!int_in_list(conf->data_pin, available_data_gpio, sizeof(available_data_gpio)/sizeof(int))) {
        fprintf(stderr, "conf.ini_checker.invalid_config: invalid data pin: %d\n", conf->data_pin);
        return -1;
    }

    return 0;
}


static void conf_apply(ws2811_t *_led_string, const ws2812d_conf *conf) {
    if (!_led_string) {
        errno = EFAULT;
        perror("conf.ini_apply.got_null_ledstring");
        abort();
    }
    if (!conf) {
        errno = EFAULT;
        perror("conf.ini_apply.got_null_conf");
        abort();
    }

    _led_string->channel[0].gpionum = conf->data_pin;
    return;
}


static void cleanup()
{
    if (last_shm_ptr) free(last_shm_ptr);
    // Turn off LEDs
    memset(ledstring.channel[0].leds, 0, led_count * sizeof(ws2811_led_t));
    ws2811_render(&ledstring);
    ws2811_fini(&ledstring);

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
        perror("main.conf_prarse.cannot_load_conf");
        return 1;
    } else if (ini_parse_err) {
        fprintf(stderr, "main.conf_prarse.bad_conf_file: bad config file (first error on line %d).\n", ini_parse_err);
        return 1;
    }

    if (conf_checker(&ws2812d_default_conf) < 0) {
        fprintf(stderr, "main.conf_prarse.conf_checker_error: invalid configuration.\n");
        return 1;
    }

    conf_apply(&ledstring, &ws2812d_default_conf);


    

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



    if (setup_gpio_power() < 0) {
        fprintf(stderr, "main.process.setup_gpio_power: cannot setup power_gpio.\n");
        return 1;
    }

    if (ws2811_init(&ledstring) != WS2811_SUCCESS) {
        fprintf(stderr, "main.process.ws2811_init: cannot init ws2811.\n");
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
    memset(last_shm_ptr, 0xFF, led_count * 3 * sizeof(uint8_t));

    while (1) {
        if (memcmp(shm_ptr, last_shm_ptr, led_count * 3) != 0) {
            memcpy(last_shm_ptr, shm_ptr, led_count * 3);
            for (int i = 0; i < LED_COUNT; ++i) {
                int r = shm_ptr[i * 3 + 0];
                int g = shm_ptr[i * 3 + 1];
                int b = shm_ptr[i * 3 + 2];
                ledstring.channel[0].leds[i] = (r << 16) | (g << 8) | b;
            }
            ws2811_render(&ledstring);
        }
        usleep(ws2812d_default_conf.delay_us);
    }
    cleanup();
    return 0;
}
