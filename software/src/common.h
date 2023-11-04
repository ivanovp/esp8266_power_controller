/**
 * @file        common.h
 * @brief       Common definitions
 * @author      Copyright (C) Peter Ivanov, 2011, 2012, 2013, 2014, 2021, 2023
 *
 * Created      2011-01-19 11:48:53
 * Last modify: 2021-02-16 17:51:49 ivanovp {Time-stamp}
 * Licence:     GPL
 */

#ifndef INCLUDE_COMMON_H
#define INCLUDE_COMMON_H

#include <stdint.h>
#include "config.h"

#define VERSION_MAJOR    1
#define VERSION_MINOR    1
#define VERSION_REVISION 1

#define MAX(x, y) (x > y ? x : y)
#define MIN(x, y) (x < y ? x : y)

#ifndef FALSE
#define FALSE   (0)
#endif
#ifndef TRUE
#define TRUE    (1)
#endif

#define CHR_EOS                     '\0'    /* End of string */
#define CHR_SPACE                   ' '
#define CHR_CR                      '\r'    /* Carriage return */
#define CHR_LF                      '\n'    /* Line feed */
#define CHR_TAB                     '\t'    /* Tabulator */
#define IS_WHITESPACE(c)    ((c) == CHR_SPACE || (c) == CHR_CR || (c) == CHR_LF || (c) == CHR_TAB)

#if DEBUG
#define debugprintf(...)            Serial.printf(__VA_ARGS__)
#else
#define debugprintf(...)
#endif
#define verboseprintf(...)          if (config.verbose) Serial.printf(__VA_ARGS__)
#define errorprintf(...)            Serial.printf("ERROR: "); serial.printf(__VA_ARGS__)

// mark parameters not used
#define UNUSED                      __attribute__((unused))

#if !DISABLE_TRACE
// TRACE output simplified, can be deactivated here
#define TRACE(...)                  Serial.printf(__VA_ARGS__)

#define ERROR(...)                  Serial.printf(__VA_ARGS__)

// TODO: add file output as well!
#define FILE_TRACE(...)             Serial.printf(__VA_ARGS__)
#else
#define TRACE(...)
#define ERROR(...)

#define FILE_TRACE(...)
#endif

/* One minute in milliseconds unit */
#define SEC_TO_MS(seconds)          ((seconds) * 1000u)
#define ONE_MIN_IN_MS               SEC_TO_MS(60)

#define XSTR(x)                     #x
#define TOSTR(x)                    XSTR(x)

typedef char bool_t;

#endif /* INCLUDE_COMMON_H */

