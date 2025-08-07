/*
 * utils.h - rpi-ws2812d WS2812 LED control daemon using shared memory
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

#ifndef UTILS_H
#define UTILS_H

#include <inttypes.h>

bool xstr2umax(const char *str, int base, uintmax_t *val);
bool xhexstr2rgb(const char *hex_str, uint8_t *r, uint8_t *g, uint8_t *b);
bool int_in_list(int value, const int *list, size_t len);
bool str_in_list(const char *s, const char * const list[], size_t len);
void hsv2rgb(float h, float s, float v, uint8_t *r, uint8_t *g, uint8_t *b);

#endif
