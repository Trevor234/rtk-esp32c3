/*
 * RTCM 3.x input framing.
 *
 * The flight controller forwards CORS/NTRIP corrections to the GPS port, so the
 * simulator has to look like a receiver that consumes them.  Full RTCM decoding
 * (observation messages, ephemerides) is deliberately out of scope: what the
 * bench needs is to prove that corrections arrive, how many messages, of which
 * type, and how large.  That is enough to distinguish "NTRIP link dead" from
 * "RTCM arrives but the module ignores it".
 *
 * Pure C so tests/ can exercise the framing without an ESP32.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define RTCM_MAX_TYPES 32

typedef struct {
    uint16_t msg_type;
    uint16_t len;         /* payload length in bytes */
    uint8_t  crc_ok;      /* 1 when the 24-bit CRC matched */
} rtcm_msg_info_t;

typedef struct {
    /* stream state machine */
    uint32_t preamble_count;   /* 0xD3 bytes seen */
    uint32_t frame_count;      /* complete frames extracted */
    uint32_t crc_errors;       /* frames whose CRC failed */
    uint32_t overflow_errors;  /* payload length out of range */
    uint32_t total_bytes;      /* bytes pushed in */

    /* most recent frame */
    uint16_t last_msg_type;
    uint16_t last_len;

    /* per-type counters for the current session */
    uint16_t types[RTCM_MAX_TYPES];
    uint32_t type_counts[RTCM_MAX_TYPES];
    uint8_t  type_used;

    /* millisecond timestamp of the last accepted frame */
    uint32_t last_frame_ms;

    /* rolling buffer state (not for external use) */
    uint8_t  hdr[3];
    uint8_t  hdr_len;
    uint16_t need;
    uint16_t got;
    uint8_t  payload[1024 + 3];
    uint32_t crc;
} rtcm_parser_t;

void rtcm_init(rtcm_parser_t *p);

/*
 * Feed received bytes.  `now_ms` stamps the last accepted frame.  Calls
 * `on_frame` for every complete frame whose length is plausible, whether or not
 * its CRC matched (a CRC failure is counted, not silently dropped, because a
 * mismatched CRC on every frame usually means the RTCM stream is truncated).
 */
void rtcm_push(rtcm_parser_t *p, const uint8_t *data, size_t len, uint32_t now_ms,
               void (*on_frame)(rtcm_parser_t *p, const rtcm_msg_info_t *info, void *ctx),
               void *ctx);

/* CRC-24Q as used by RTCM 3.x.  `_cont` lets a caller carry the running value
   across discontiguous buffers (the frame header and its payload). */
uint32_t rtcm_crc24q(const uint8_t *data, size_t len);
uint32_t rtcm_crc24q_cont(uint32_t crc, const uint8_t *data, size_t len);

/* Describe a message type in a human-readable way. */
const char *rtcm_type_name(uint16_t type);
