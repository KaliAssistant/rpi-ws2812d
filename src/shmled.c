/*
 * shmled.c - rpi-ws2812d WS2812 LED control daemon using shared memory
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
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <signal.h>
#include <unistd.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>
#include <getopt.h>
#include <math.h>
#include "utils.h"
//#include "version.h"
#include "config.h"

#define SHM_FILE "/led_rgb"
#define SHM_SIZE 3

static uint8_t *shm_ptr = NULL;
static int shm_fd = -1;

static uint8_t color_r, color_g, color_b;
static int loop = 0;
static int blink = 0;
static int fade = 0;
static int rainbow = 0;
static uintmax_t delay_us = 500000;
static const char *input_file = NULL;

static int setup_shm_writer(void) {
    shm_fd = shm_open(SHM_FILE, O_RDWR, 0);
    if (shm_fd < 0) {
        perror("setup.shm.cannot_open_shm_file");
        return -1;
    }

    shm_ptr = mmap(NULL, SHM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (shm_ptr == MAP_FAILED) {
        perror("setup.shm.mmap_failed");
        close(shm_fd);
        return -1;
    }

    return 0;
}

static int write_rgb(uint8_t r, uint8_t g, uint8_t b) {
    if (!shm_ptr) {
        errno = EFAULT;
        return -1;
    }
    //printf("write_rgb: R=%u G=%u B=%u\n", r, g, b);
    shm_ptr[0] = r;
    shm_ptr[1] = g;
    shm_ptr[2] = b;
    msync(shm_ptr, SHM_SIZE, MS_SYNC);
    return 0;
}

static void cleanup(void) {
    if (shm_ptr) munmap((void *)shm_ptr, SHM_SIZE);
    if (shm_fd >= 0) close(shm_fd);
}

static void handle_signal(int sig) {
    //fprintf(stderr, "Caught signal %d, exiting...\n", sig); // for debug
    (void)sig;
    cleanup();
    exit(1);
}

static void return_to_cleanup(int ret) {
    cleanup();
    exit(ret);
}

static int do_blink() {
    while (1) {
        if (write_rgb(color_r, color_g, color_b) < 0) {
            perror("main.do_blink.write_rgb_failed");
            return 1;
        }
        usleep(delay_us);
        if (write_rgb(0, 0, 0) < 0) {
            perror("main.do_blink.write_rgb_failed");
            return 1;
        }
        usleep(delay_us);
    }
}

static int do_fade() {
    while (1) {
        for (int i = 0; i <= 255; ++i) {
            if (write_rgb((color_r * i) / 255, (color_g * i) / 255, (color_b * i) /255) < 0) {
                perror("main.do_fade.write_rgb_failed");
                return 1;
            }
            usleep(delay_us / 256);
        }
        for (int i = 255; i >= 0; --i) {
            if (write_rgb((color_r * i) / 255, (color_g * i) / 255, (color_b * i) /255) < 0) {
                perror("main.do_fade.write_rgb_failed");
                return 1;
            }
            usleep(delay_us / 256);
        }
    }
}

static int do_rainbow() {
    float h = 0;
    while (1) {
        for (h = 0; h < 360; h += 2) {
            uint8_t r, g, b;
            hsv2rgb(h, 1.0, 1.0, &r, &g, &b);
            if (write_rgb(r, g, b) < 0) {
                perror("main.do_rainbow.write_rgb_failed");
                return 1;
            }
            usleep(delay_us);
        }
    }
}

static int do_file_playback(bool loop) {
    while (1) {
        FILE *f = fopen(input_file, "rb");
        if (!f) {
            perror("main.do_file_playback.cannot_open_input_file");
            return 1;
        }

        uint8_t buf[3];
        while (fread(buf, 1, 3, f) == 3) {
            if (write_rgb(buf[0], buf[1], buf[2]) < 0) {
                perror("main.do_file_playback.write_rgb_failed");
                fclose(f);
                return 1;
            }
            usleep(delay_us);
        }

        fclose(f);
        if (!loop) break;
    }
    
    return 0;
}

static void usage(const char *prog_name) {
    if (!prog_name) return;
    fprintf(stderr, "shmLED - rpi-ws2812d WS281x RGB NeoPixel daemon for raspberry pi\n\n");
    fprintf(stderr, "Usage: %s -c #RRGGBB -i <rgb bin file input> [-l -i|-b|-d|-r] [-s µs]\n\n", prog_name);
    fprintf(stderr, "-c :\t#RRGGBB, rgb hex code\n");
    fprintf(stderr, "-i :\t<input.bin>, rgb bin file input, format: \\xRR\\xGG\\xBB ...\n\n");
    fprintf(stderr, "-l :\tloop mode:\n");
    fprintf(stderr, "    -b :\tblink\n");
    fprintf(stderr, "    -d :\tfade\n");
    fprintf(stderr, "    -r :\trainbow\n");
    fprintf(stderr, "    -i :\tsame as rgb file input, but loop mode\n\n");
    fprintf(stderr, "-s :\tdelay µs per frame\n");
    fprintf(stderr, "-h :\tshow this help\n\n");
    fprintf(stderr, "Version %s By KaliAssistant\n", VERSION);
    fprintf(stderr, "Github: https://github.com/KaliAssistant/rpi-ws2812d.git\n");
    return;
}

int main(int argc, char **argv) {
    int opt;
    while ((opt = getopt(argc, argv, "c:i:lbdhrs:")) != -1) {
        switch (opt) {
            case 'c':
                if (!xhexstr2rgb(optarg, &color_r, &color_g, &color_b)) {
                    fprintf(stderr, "Invalid color format: %s\n", optarg);
                    return 1;
                }
                break;
            case 'h': usage(argv[0]); return 0;
            case 'l': loop = 1; break;
            case 'b': blink = 1; break;
            case 'd': fade = 1; break;
            case 'r': rainbow = 1; break;
            case 's': 
                if (!xstr2umax(optarg, 10, &delay_us)) {
                    perror("main.optarg.cannot_parse_delay_us");
                    return 1;
                }
                break;
            case 'i': input_file = optarg; break;
            case '?': 
                fprintf(stderr, "See '%s -h' for help.\n", argv[0]);
                return 1;
            default:
                errno = EFAULT;
                perror("main.getopt.got_impossible_default");
                abort();
        }
    }

    signal(SIGINT, handle_signal);
    signal(SIGBUS, handle_signal);
    signal(SIGSEGV, handle_signal);
    signal(SIGTERM, handle_signal);

    if (setup_shm_writer() < 0) return 1;

    if (!loop) {
        // One-time color set
        if (input_file) return_to_cleanup(do_file_playback(0));
        
        // default set color to 0, 0, 0
        if (write_rgb(color_r, color_g, color_b) < 0) {
            perror("main.otpcolor.write_rgb_failed");
            return_to_cleanup(1);
        }
        return_to_cleanup(0);
    }

    // Loop mode
    if (input_file) return_to_cleanup(do_file_playback(1));
    
    if (blink) return_to_cleanup(do_blink());
    if (fade) return_to_cleanup(do_fade());
    if (rainbow) return_to_cleanup(do_rainbow());

    fprintf(stderr, "Loop mode enabled, but no -b, -d, -r or -i <input> specified\n");
    return 1;

}
