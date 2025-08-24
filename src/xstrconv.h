/*
 * xstrconv.h - Extended string-to-number conversion helpers
 *
 * SPDX-License-Identifier: GPL-3.0
 *
 * Copyright (C) 2025 KaliAssistant
 * Author: KaliAssistant
 *
 * This file is part of the rpi-ws2812d project and is licensed under the GNU
 * General Public License v3.0 or later.
 *
 * Provides safe wrappers for converting strings into various numeric types.
 * Each function:
 *   - Returns true on success, false on error.
 *   - Sets errno appropriately (EINVAL, ERANGE).
 *   - Checks that the entire string is consumed.
 *   - Validates against the target type’s min/max range.
 *
 * Supported types:
 *   Unsigned:  uint8_t, uint16_t, uint32_t, uint64_t, uintmax_t
 *   Signed:    int8_t, int16_t, int32_t, int64_t, intmax_t
 *   Floating:  float, double, long double
 */

#ifndef XSTRCONV_UTILS_H
#define XSTRCONV_UTILS_H

#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdlib.h>
#include <limits.h>  /* For min/max constants */

#ifdef __cplusplus
extern "C" {
#endif

/* Unsigned integer conversions */
bool xstr2u8   (const char *str, int base, uint8_t   *val);
bool xstr2u16  (const char *str, int base, uint16_t  *val);
bool xstr2u32  (const char *str, int base, uint32_t  *val);
bool xstr2u64  (const char *str, int base, uint64_t  *val);
bool xstr2umax (const char *str, int base, uintmax_t *val);

/* Signed integer conversions */
bool xstr2i8   (const char *str, int base, int8_t   *val);
bool xstr2i16  (const char *str, int base, int16_t  *val);
bool xstr2i32  (const char *str, int base, int32_t  *val);
bool xstr2i64  (const char *str, int base, int64_t  *val);
bool xstr2imax (const char *str, int base, intmax_t *val);

/* Floating-point conversions */
bool xstr2flt  (const char *str, float        *val);
bool xstr2dbl  (const char *str, double       *val);
bool xstr2ldbl (const char *str, long double  *val);

#ifdef __cplusplus
}
#endif

#endif /* XSTRCONV_UTILS_H */

