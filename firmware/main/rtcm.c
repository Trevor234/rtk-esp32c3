#include "rtcm.h"

#include <stdio.h>
#include <string.h>

/* RTCM 3.x frame: 0xD3, 6 reserved bits + 10 bit length, payload, CRC-24Q. */
#define RTCM_MAX_PAYLOAD 1023

static uint32_t crc24q_byte(uint32_t crc, uint8_t b)
{
    crc ^= (uint32_t)b << 16;
    for (int i = 0; i < 8; ++i) {
        crc <<= 1;
        if (crc & 0x1000000u) {
            crc ^= 0x1864CFBu;
        }
    }
    return crc & 0xFFFFFFu;
}

uint32_t rtcm_crc24q(const uint8_t *data, size_t len)
{
    return rtcm_crc24q_cont(0, data, len);
}

uint32_t rtcm_crc24q_cont(uint32_t crc, const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; ++i) {
        crc = crc24q_byte(crc, data[i]);
    }
    return crc & 0xFFFFFFu;
}


static uint16_t type_from_payload(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 4) | ((uint16_t)p[1] >> 4));
}

const char *rtcm_type_name(uint16_t type)
{
    switch (type) {
    case 1001: return "GPS L1 code";
    case 1002: return "GPS L1 code+phase";
    case 1003: return "GPS L1/L2";
    case 1004: return "GPS L1/L2 code+phase";
    case 1005: return "Station coords (ECEF)";
    case 1006: return "Station coords+height";
    case 1007: return "Antenna descriptor";
    case 1012: return "GLONASS L1/L2";
    case 1019: return "GPS ephemeris";
    case 1020: return "GLONASS ephemeris";
    case 1033: return "Receiver/antenna info";
    case 1042: return "BDS ephemeris";
    case 1046: return "Galileo ephemeris";
    case 1071: return "GPS MSM1";
    case 1072: return "GPS MSM2";
    case 1074: return "GPS MSM4";
    case 1075: return "GPS MSM5";
    case 1076: return "GPS MSM6";
    case 1077: return "GPS MSM7";
    case 1084: return "GLONASS MSM4";
    case 1087: return "GLONASS MSM7";
    case 1094: return "Galileo MSM4";
    case 1097: return "Galileo MSM7";
    case 1124: return "BDS MSM4";
    case 1127: return "BDS MSM7";
    case 1230: return "GLONASS code-phase bias";
    default:   return "other";
    }
}

void rtcm_init(rtcm_parser_t *p)
{
    memset(p, 0, sizeof(*p));
    p->hdr_len = 0;
    p->need = 0;
    p->got = 0;
}

static void note_type(rtcm_parser_t *p, uint16_t type)
{
    for (uint8_t i = 0; i < p->type_used; ++i) {
        if (p->types[i] == type) {
            p->type_counts[i]++;
            return;
        }
    }
    if (p->type_used < RTCM_MAX_TYPES) {
        p->types[p->type_used] = type;
        p->type_counts[p->type_used] = 1;
        p->type_used++;
    }
}

static void emit_frame(rtcm_parser_t *p,
                       void (*on_frame)(rtcm_parser_t *, const rtcm_msg_info_t *, void *),
                       void *ctx, uint32_t now_ms)
{
    const uint16_t type = type_from_payload(p->payload);
    /* The CRC covers the 3 header bytes followed by the payload; the header and
       payload live in separate buffers, so the running value is carried across
       both rather than hashing past the end of the header array. */
    const uint32_t crc_calc = rtcm_crc24q_cont(rtcm_crc24q(p->hdr, 3), p->payload, p->need);
    const uint32_t crc_rx = ((uint32_t)p->payload[p->need] << 16) |
                            ((uint32_t)p->payload[p->need + 1] << 8) |
                            (uint32_t)p->payload[p->need + 2];
    const uint8_t ok = (crc_calc == crc_rx) ? 1u : 0u;

    p->frame_count++;
    p->last_msg_type = type;
    p->last_len = p->need;
    if (ok) {
        p->last_frame_ms = now_ms;
        note_type(p, type);
    } else {
        p->crc_errors++;
    }

    if (on_frame) {
        rtcm_msg_info_t info = { .msg_type = type, .len = p->need, .crc_ok = ok };
        on_frame(p, &info, ctx);
    }
}

void rtcm_push(rtcm_parser_t *p, const uint8_t *data, size_t len, uint32_t now_ms,
               void (*on_frame)(rtcm_parser_t *, const rtcm_msg_info_t *, void *),
               void *ctx)
{
    p->total_bytes += (uint32_t)len;

    for (size_t i = 0; i < len; ++i) {
        const uint8_t b = data[i];

        if (p->need != 0) {
            /* collecting payload + 3 CRC bytes */
            p->payload[p->got++] = b;
            if (p->got == (uint16_t)(p->need + 3)) {
                emit_frame(p, on_frame, ctx, now_ms);
                p->need = 0;
                p->got = 0;
                p->hdr_len = 0;
            }
            continue;
        }

        if (p->hdr_len < 3) {
            p->hdr[p->hdr_len++] = b;
            if (p->hdr_len == 1 && b != 0xD3) {
                p->preamble_count += 0;
                p->hdr_len = 0;
                continue;
            }
            if (p->hdr_len == 3) {
                const uint16_t payload_len =
                    (uint16_t)(((uint16_t)(p->hdr[1] & 0x03u) << 8) | p->hdr[2]);
                p->preamble_count++;
                if (payload_len == 0 || payload_len > RTCM_MAX_PAYLOAD) {
                    p->overflow_errors++;
                    p->hdr_len = 0;
                    continue;
                }
                p->need = payload_len;
                p->got = 0;
            }
            continue;
        }
    }
}
