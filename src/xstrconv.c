/*
 * xstrconv.c - Extended string-to-number conversion helpers
 *
 * SPDX-License-Identifier: GPL-3.0
 *
 * Copyright (C) 2025 KaliAssistant
 * Author: KaliAssistant
 *
 * This file is part of the rpi-ws2812d project and is licensed under the GNU
 * General Public License v3.0 or later.
 *
 * xstrconv provides safe wrappers for converting C strings to numeric types.
 * Unlike direct use of strtol/strtoul/etc., these helpers:
 *   - Reject NULL pointers.
 *   - Ensure the entire string is consumed (no trailing garbage).
 *   - Validate against the target type’s min/max range.
 *   - Set errno to indicate error (EINVAL, ERANGE).
 *   - Return a bool for success/failure instead of relying only on errno.
 *
 * Supported conversions:
 *   Unsigned:  uint8_t, uint16_t, uint32_t, uint64_t, uintmax_t
 *   Signed:    int8_t, int16_t, int32_t, int64_t, intmax_t
 *   Floating:  float, double, long double
 *
 * Example:
 *   uint32_t val;
 *   if (xstr2u32("1234", 10, &val)) {
 *       // success, val == 1234
 *   } else {
 *       perror("xstr2u32 failed");
 *   }
 *
 * Project GitHub: https://github.com/KaliAssistant/rpi-ws2812d
 *
 * See LICENSE for more details.
 */


#include <stdio.h>
#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdlib.h>
#include <limits.h> // For min/max constants
//#include "xstrconv.h"

bool xstr2u8(const char *str, int base, uint8_t *val) {
    if (!str || !val) {
        errno = EINVAL;
        return false;
    }
    errno = 0;
    char *endptr;
    uintmax_t tmp = strtoumax(str, &endptr, base);
    if (str == endptr || *endptr != '\0') {
        errno = EINVAL;
        return false;
    }
    if (errno == ERANGE || tmp > UINT8_MAX) {
        errno = ERANGE;
        return false;
    }
    *val = (uint8_t)tmp;
    return true;
}

bool xstr2u16(const char *str, int base, uint16_t *val) {
    if (!str || !val) {
        errno = EINVAL;
        return false;
    }
    errno = 0;
    char *endptr;
    uintmax_t tmp = strtoumax(str, &endptr, base);
    if (str == endptr || *endptr != '\0') {
        errno = EINVAL;
        return false;
    }
    if (errno == ERANGE || tmp > UINT16_MAX) {
        errno = ERANGE;
        return false;
    }
    *val = (uint16_t)tmp;
    return true;
}

bool xstr2u32(const char *str, int base, uint32_t *val) {
    if (!str || !val) {
        errno = EINVAL;
        return false;
    }
    errno = 0;
    char *endptr;
    uintmax_t tmp = strtoumax(str, &endptr, base);
    if (str == endptr || *endptr != '\0') {
        errno = EINVAL;
        return false;
    }
    if (errno == ERANGE || tmp > UINT32_MAX) {
        errno = ERANGE;
        return false;
    }
    *val = (uint32_t)tmp;
    return true;
}

bool xstr2u64(const char *str, int base, uint64_t *val) {
    if (!str || !val) {
        errno = EINVAL;
        return false;
    }
    errno = 0;
    char *endptr;
    uintmax_t tmp = strtoumax(str, &endptr, base);
    if (str == endptr || *endptr != '\0') {
        errno = EINVAL;
        return false;
    }
    if (errno == ERANGE || tmp > UINT64_MAX) {
        errno = ERANGE;
        return false;
    }
    *val = (uint64_t)tmp;
    return true;
}

bool xstr2umax(const char *str, int base, uintmax_t *val) {
    if (!str || !val) {
        errno = EINVAL;
        return false;
    }
    errno = 0;
    char *endptr;
    uintmax_t tmp = strtoumax(str, &endptr, base);
    if (str == endptr || *endptr != '\0') {
        errno = EINVAL;
        return false;
    }
    if (errno == ERANGE) {
        return false;
    }
    *val = tmp;
    return true;
}

bool xstr2i8(const char *str, int base, int8_t *val) {
    if (!str || !val) {
        errno = EINVAL;
        return false;
    }
    errno = 0;
    char *endptr;
    intmax_t tmp = strtoimax(str, &endptr, base);
    if (str == endptr || *endptr != '\0') {
        errno = EINVAL;
        return false;
    }
    if (errno == ERANGE || tmp < INT8_MIN || tmp > INT8_MAX) {
        errno = ERANGE;
        return false;
    }
    *val = (int8_t)tmp;
    return true;
}

bool xstr2i16(const char *str, int base, int16_t *val) {
    if (!str || !val) {
        errno = EINVAL;
        return false;
    }
    errno = 0;
    char *endptr;
    intmax_t tmp = strtoimax(str, &endptr, base);
    if (str == endptr || *endptr != '\0') {
        errno = EINVAL;
        return false;
    }
    if (errno == ERANGE || tmp < INT16_MIN || tmp > INT16_MAX) {
        errno = ERANGE;
        return false;
    }
    *val = (int16_t)tmp;
    return true;
}

bool xstr2i32(const char *str, int base, int32_t *val) {
    if (!str || !val) {
        errno = EINVAL;
        return false;
    }
    errno = 0;
    char *endptr;
    intmax_t tmp = strtoimax(str, &endptr, base);
    if (str == endptr || *endptr != '\0') {
        errno = EINVAL;
        return false;
    }
    if (errno == ERANGE || tmp < INT32_MIN || tmp > INT32_MAX) {
        errno = ERANGE;
        return false;
    }
    *val = (int32_t)tmp;
    return true;
}

bool xstr2i64(const char *str, int base, int64_t *val) {
    if (!str || !val) {
        errno = EINVAL;
        return false;
    }
    errno = 0;
    char *endptr;
    intmax_t tmp = strtoimax(str, &endptr, base);
    if (str == endptr || *endptr != '\0') {
        errno = EINVAL;
        return false;
    }
    if (errno == ERANGE || tmp < INT64_MIN || tmp > INT64_MAX) {
        errno = ERANGE;
        return false;
    }
    *val = (int64_t)tmp;
    return true;
}

bool xstr2imax(const char *str, int base, intmax_t *val) {
    if (!str || !val) {
        errno = EINVAL;
        return false;
    }
    errno = 0;
    char *endptr;
    intmax_t tmp = strtoimax(str, &endptr, base);
    if (str == endptr || *endptr != '\0') {
        errno = EINVAL;
        return false;
    }
    if (errno == ERANGE) {
        return false;
    }
    *val = tmp;
    return true;
}

bool xstr2flt(const char *str, float *val) {
    if (!str || !val) {
        errno = EINVAL;
        return false;
    }
    errno = 0;
    char *endptr;
    float tmp = strtof(str, &endptr);
    if (str == endptr || *endptr != '\0') {
        errno = EINVAL;
        return false;
    }
    if (errno == ERANGE) {
        return false;
    }
    *val = tmp;
    return true;  
}

bool xstr2dbl(const char *str, double *val) {
    if (!str || !val) {
        errno = EINVAL;
        return false;
    }
    errno = 0;
    char *endptr;
    double tmp = strtod(str, &endptr);
    if (str == endptr || *endptr != '\0') {
        errno = EINVAL;
        return false;
    }
    if (errno == ERANGE) {
        return false;
    }
    *val = tmp;
    return true;
}

bool xstr2ldbl(const char *str, long double *val) {
    if (!str || !val) {
        errno = EINVAL;
        return false;
    }
    errno = 0;
    char *endptr;
    long double tmp = strtold(str, &endptr);
    if (str == endptr || *endptr != '\0') {
        errno = EINVAL;
        return false;
    }
    if (errno == ERANGE) {
        return false;
    }
    *val = tmp;
    return true;
}

