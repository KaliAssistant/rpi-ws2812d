/*
 * utils.h - rpi-ws2812d common utility functions
 *
 * SPDX-License-Identifier: GPL-3.0
 *
 * Copyright (C) 2025 KaliAssistant
 * Author: KaliAssistant
 *
 * This file is part of the rpi-ws2812d project and is licensed under the GNU
 * General Public License v3.0 or later.
 *
 * This file implements helper functions shared across rpi-ws2812d programs,
 * providing consistent, safe parsing and list-checking utilities, as well as
 * color space conversion.
 *
 * Functions:
 *   - xstr2umax(): Convert string to uintmax_t with validation.
 *   - xhexstr2rgb(): Convert RGB hex string to separate R, G, B components.
 *   - int_in_list(): Check if an integer is in a given integer list.
 *   - str_in_list(): Check if a string is in a given string list.
 *   - hsv2rgb(): Convert HSV color to RGB color.
 *
 * These functions are designed for strict input validation, error handling,
 * and color manipulation to support robust LED control and configuration parsing.
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

/**
 * Convert a string representing an unsigned integer in a given base
 * to a uintmax_t value.
 *
 * @param str   The input string to convert.
 * @param base  The numeric base to interpret the string (e.g., 10, 16).
 * @param val   Pointer to store the converted uintmax_t value.
 * @return      true if conversion succeeded and string fully parsed, false otherwise.
 */
bool xstr2umax(const char *str, int base, uintmax_t *val);

/**
 * Convert a hex color string to RGB components.
 *
 * Accepts strings like "#RRGGBB" or "RRGGBB".
 *
 * @param hex_str   The hex color string.
 * @param r         Pointer to store red component (0-255).
 * @param g         Pointer to store green component (0-255).
 * @param b         Pointer to store blue component (0-255).
 * @return          true if the input string is valid and conversion succeeded,
 *                  false if the string is invalid.
 */
bool xhexstr2rgb(const char *hex_str, uint8_t *r, uint8_t *g, uint8_t *b);

/**
 * Check if an integer value is present in a list of integers.
 *
 * @param value The integer value to check.
 * @param list  Pointer to an array of integers.
 * @param len   Number of elements in the list.
 * @return      true if value is found in the list, false otherwise.
 */
bool int_in_list(int value, const int *list, size_t len);

/**
 * Check if a string is present in a list of strings.
 *
 * @param s     The string to check.
 * @param list  Array of string pointers.
 * @param len   Number of strings in the list.
 * @return      true if the string is found, false otherwise.
 */
bool str_in_list(const char *s, const char * const list[], size_t len);

/**
 * Convert HSV color representation to RGB.
 *
 * Hue (h) in degrees [0, 360), saturation (s) and value (v) in [0, 1].
 * Output r, g, b components are in [0, 255].
 *
 * @param h Hue angle in degrees (0-360).
 * @param s Saturation (0.0-1.0).
 * @param v Value/Brightness (0.0-1.0).
 * @param r Pointer to output red component.
 * @param g Pointer to output green component.
 * @param b Pointer to output blue component.
 */
void hsv2rgb(float h, float s, float v, uint8_t *r, uint8_t *g, uint8_t *b);

#endif /* UTILS_H */
