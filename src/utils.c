/*
 * utils.c - rpi-ws2812d WS2812 LED control daemon using shared memory
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
#include <stdbool.h>
#include <inttypes.h>
#include <errno.h>
#include <ctype.h>
#include <string.h>
#include <math.h>


bool xstr2umax(const char *str, int base, uintmax_t *val) {
    errno = 0;
    char *endptr;
    *val = strtoumax(str, &endptr, base);
    
    if (*endptr != '\0' || str == endptr) {
        errno = EINVAL;
        return false;
    }
    if (errno == ERANGE) return false;
    return true;
}

bool xhexstr2rgb(const char *hex_str, uint8_t *r, uint8_t *g, uint8_t *b) {
    int hex_str_len = strlen(hex_str);
    int hex_str_seek = 0;
    if (hex_str_len == 6) {
        for (int i = 0; i < 6; i++) {
            if (!isxdigit(hex_str[i])) {
                errno = EINVAL;
                return false;
            }
        }
    } else if (hex_str_len == 7) {
        hex_str_seek ++;
        if (hex_str[0] != '#') return false;
        for (int i = 1; i < 7; i++) {
            if (!isxdigit(hex_str[i])) {
                errno = EINVAL;
                return false;
            }
        }
    } else {
        errno = EINVAL;
        return false;
    }
    
    sscanf(hex_str + hex_str_seek, "%2hhx%2hhx%2hhx", r, g, b);
    return true;
}

bool int_in_list(int value, const int *list, size_t len) {
    for (size_t i = 0; i < len; i++) {
        if (list[i] == value)
            return true;
    }
    return false;
}

bool str_in_list(const char *s, const char * const list[], size_t len) {
    for (size_t i = 0; i < len; i++) {
        if (strcmp(s, list[i]) == 0)
            return true;
    }
    return false;
}

void hsv2rgb(float h, float s, float v, uint8_t *r, uint8_t *g, uint8_t *b) {
    float c = v * s;
    float x = c * (1 - fabs(fmodf(h / 60.0, 2) - 1));
    float m = v - c;
    float r_, g_, b_;

    if (h < 60)       { r_ = c; g_ = x; b_ = 0; }
    else if (h < 120) { r_ = x; g_ = c; b_ = 0; }
    else if (h < 180) { r_ = 0; g_ = c; b_ = x; }
    else if (h < 240) { r_ = 0; g_ = x; b_ = c; }
    else if (h < 300) { r_ = x; g_ = 0; b_ = c; }
    else              { r_ = c; g_ = 0; b_ = x; }

    *r = (r_ + m) * 255;
    *g = (g_ + m) * 255;
    *b = (b_ + m) * 255;
}
