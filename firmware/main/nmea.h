/*
 * NMEA 0183 sentence construction.
 *
 * Pure C, no ESP-IDF dependencies, so the same code is compiled into the host
 * unit tests under tests/.
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

/*
 * Wrap a sentence body into a complete NMEA line.
 *
 * `body` is everything between '$' and '*', for example
 *   "GNGGA,120000.00,3100.000000,N,12100.000000,E,4,22,0.7,3.5,M,0.0,M,1.0,0000"
 *
 * The function prepends '$', appends the XOR checksum as two upper-case hex
 * digits and terminates the line with CRLF.  Returns the number of bytes
 * written (excluding the terminating NUL) or -1 when `out` is too small.
 */
int nmea_sentence(char *out, size_t out_len, const char *body);

/*
 * Format a latitude/longitude for NMEA: "ddmm.mmmmmm" (latitude) or
 * "dddmm.mmmmmm" (longitude) plus the hemisphere letter.  Both buffers are
 * always NUL terminated; `hemi` receives 'N'/'S' or 'E'/'W'.
 */
void nmea_fmt_lat(double deg, char *val, size_t val_len, char *hemi);
void nmea_fmt_lon(double deg, char *val, size_t val_len, char *hemi);

/* XOR checksum of a sentence body (without '$' and '*'). */
uint8_t nmea_checksum(const char *body);
