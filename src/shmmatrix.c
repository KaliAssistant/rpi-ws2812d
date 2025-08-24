/*
 * shmmatrix.c - Shared memory RGB matrix writer for rpi-ws2812d WS2812 daemon
 *
 * SPDX-License-Identifier: GPL-3.0
 *
 * Copyright (C) 2025 KaliAssistant
 * Author: KaliAssistant
 *
 * This file is part of the rpi-ws2812d project and is licensed under the GNU
 * General Public License v3.0 or later.
 *
 * shmmatrix is a playback utility that writes RGB frame data into shared
 * memory for WS2812 LED matrices driven by the rpi-ws2812d daemon.
 *
 * Features:
 *   - Outputs RGB888 frame data (3 bytes per LED) to the shared memory region.
 *   - Plays back binary RGB frame sequences stored in a file.
 *   - Supports one-shot playback or continuous loop mode.
 *   - Configurable delay (in microseconds) between frames.
 *   - Handles UNIX signals for clean shutdown and shared memory unmapping.
 *
 * Typical usage:
 *   shmmatrix -c64 -i video.bin -s33333
 *   (Plays an 8x8 RGB matrix animation with ~30 FPS)
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
#include "xstrconv.h"
//#include "version.h"
#include "config.h"

#define DEFAULT_LED_COUNT 1

#define SHM_FILE "/led_rgb"

static uint8_t *shm_ptr = NULL;
static uint8_t *file_buf_ptr = NULL;

static int shm_fd = -1;

static uint32_t leds_count = 1;
static int loop = 0;

static uint32_t delay_us = 500000;
static const char *input_file = NULL;

static int setup_shm_writer(void) {
    shm_fd = shm_open(SHM_FILE, O_RDWR, 0);
    if (shm_fd < 0) {
        perror("setup.shm.cannot_open_shm_file");
        return -1;
    }

    shm_ptr = mmap(NULL, leds_count * 3, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (shm_ptr == MAP_FAILED) {
        perror("setup.shm.mmap_failed");
        close(shm_fd);
        return -1;
    }

    return 0;
}

static int write_matrix(uint8_t *_shm_ptr, uint8_t *matrix_ptr, size_t _led_count) {
    if (!_shm_ptr || !matrix_ptr) {
        errno = EFAULT;
        return -1;
    }
    memcpy(_shm_ptr, matrix_ptr, _led_count * 3);
    msync(shm_ptr, _led_count * 3, MS_SYNC);
    return 0;
}

static void cleanup(void) {
    if (shm_ptr) if (shm_ptr) munmap((void *)shm_ptr, leds_count * 3);
    if (shm_fd >= 0) close(shm_fd);
    if (file_buf_ptr) free(file_buf_ptr);
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


static int do_file_playback(bool loop) {
    while (1) {
        FILE *f = fopen(input_file, "rb");
        if (!f) {
            perror("main.do_file_playback.cannot_open_input_file");
            return 1;
        }

        while (fread(file_buf_ptr, 1, leds_count * 3, f) == leds_count * 3) {
            if (write_matrix(shm_ptr, file_buf_ptr, leds_count) < 0) {
                perror("main.do_file_playback.write_matrix_failed");
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
    fprintf(stderr, "shmMatrix - Shared memory RGB matrix writer for rpi-ws2812d WS2812 daemon\n\n");
    fprintf(stderr, "Usage: %s -c <leds count> -i <rgb matrix bin file input> [-l -i] [-s µs]\n\n", prog_name);
    fprintf(stderr, "-c :\t<count>, leds count\n");
    fprintf(stderr, "-i :\t<input.bin>, matrix rgb bin file input, format: \\xRR\\xGG\\xBB ...\n\n");
    fprintf(stderr, "-l :\tloop mode:\n");
    fprintf(stderr, "    -i :\tsame as matrix rgb file input, but loop mode\n\n");
    fprintf(stderr, "-s :\tdelay µs per frame\n");
    fprintf(stderr, "-h :\tshow this help\n\n");
    fprintf(stderr, "Version %s By KaliAssistant\n", VERSION);
    fprintf(stderr, "Github: https://github.com/KaliAssistant/rpi-ws2812d.git\n");
    return;
}

int main(int argc, char **argv) {
    int opt;
    while ((opt = getopt(argc, argv, "c:i:lhs:")) != -1) {
        switch (opt) {
            case 'c':
                if (!xstr2u32(optarg, 10, &leds_count)) {
                    fprintf(stderr, "Invalid leds count: %s\n", optarg);
                    return 1;
                }
                break;
            case 'h': usage(argv[0]); return 0;
            case 'l': loop = 1; break;
            case 's': 
                if (!xstr2u32(optarg, 10, &delay_us)) {
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

    file_buf_ptr = calloc(leds_count * 3, sizeof(uint8_t));

    if (!loop) {
        if (input_file) return_to_cleanup(do_file_playback(0));
        
        if (write_matrix(shm_ptr, file_buf_ptr, leds_count) < 0) {
            perror("main.otpcolor.write_matrix_failed");
            return_to_cleanup(1);
        }
        return_to_cleanup(0);
    }

    if (input_file) return_to_cleanup(do_file_playback(1));

    fprintf(stderr, "Loop mode enabled, but no -i <input> specified\n");
    return 1;
}
