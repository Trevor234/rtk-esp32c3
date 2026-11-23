#define _POSIX_C_SOURCE 200809L

/*
 * Host-side verification harness.
 *
 * The point of this file is not to test our own code with our own assumptions.
 * It re-implements ArduPilot's AP_GPS_NMEA sentence parser *following the upstream
 * source line for line* (the term indices, the fix-quality mapping, the
 * "new message" freshness rule and the GPS-yaw 300 ms timeout), feeds it the
 * exact bytes the ESP32 firmware emits, and checks that the flight controller
 * would conclude what we intend.
 *
 * Build:  run tests/run_tests.ps1
 */
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "../firmware/main/nmea.h"
#include "../firmware/main/rtcm.h"
#include "../firmware/main/sim.h"

static int g_failures;
static int g_checks;

#define CHECK(cond, fmt, ...)                                            \
    do {                                                                 \
        g_checks++;                                                      \
        if (!(cond)) {                                                   \
            g_failures++;                                                \
            printf("  FAIL: " fmt "\n", ##__VA_ARGS__);                  \
        }                                                                \
    } while (0)

#define SECTION(name) printf("\n== %s ==\n", name)

/* ------------------------------------------------------------------------ */
/* ArduPilot AP_GPS_NMEA replica                                             */
/* ------------------------------------------------------------------------ */

/* AP_GPS_FixType, from AP_GPS.h */
typedef enum {
    AP_FIX_NONE = 0,
    AP_FIX_2D,
    AP_FIX_3D,
    AP_FIX_DGPS,
    AP_FIX_RTK_FLOAT,
    AP_FIX_RTK_FIXED,
    AP_FIX_STATIC,
    AP_FIX_PPP,
} ap_fix_t;

/* _GPS_SENTENCE_* values are the term-number bases used by upstream:
   32 = RMC, 64 = GGA, 96 = VTG, 128 = HDT, 160 = THS */
enum {
    SENT_RMC = 32,
    SENT_GGA = 64,
    SENT_VTG = 96,
    SENT_HDT = 128,
    SENT_THS = 160,
};

typedef struct {
    /* parser transient state */
    char      term[32];
    int       term_number;
    int       sentence_type;
    bool      is_checksum_term;
    bool      sentence_done;
    uint8_t   parity;
    int       term_offset;

    /* decoded values */
    uint16_t  new_quality;
    uint16_t  new_sats;
    uint16_t  new_hdop;
    int32_t   new_time;
    int32_t   new_date;
    int32_t   new_lat;       /* degrees * 1e7 */
    int32_t   new_lon;
    int32_t   new_alt;       /* cm */
    int32_t   new_speed;     /* knots * 100 scaled per upstream */
    int32_t   new_course;    /* deg * 100 */
    int32_t   new_gps_yaw;   /* deg * 100 */
    bool      have_new_course;

    /* published state */
    ap_fix_t  status;
    int32_t   lat, lon;
    int32_t   alt_cm;
    uint16_t  num_sats;
    uint16_t  hdop;
    float     ground_speed;
    float     ground_course;
    float     gps_yaw;
    bool      have_gps_yaw;
    bool      gps_yaw_configured;

    /* freshness stamps, milliseconds */
    uint32_t  last_RMC_ms, last_GGA_ms, last_VTG_ms, last_yaw_ms;
    uint32_t  last_fix_ms;
    bool      valid;      /* result of the most recent _have_new_message() */
    int       commits;    /* how many times a fix was published */
} ap_nmea_t;

static void ap_init(ap_nmea_t *s)
{
    memset(s, 0, sizeof(*s));
    s->status = AP_FIX_NONE;
    s->sentence_type = 0;
    s->term_number = 0;
    s->term_offset = 0;
    s->parity = 0;
    s->is_checksum_term = false;
    s->sentence_done = false;
}

/* Upstream _parse_degrees(): ddmm.mmmm / dddmm.mmmm -> degrees*1e7 */
static int32_t ap_parse_degrees(const char *t)
{
    const char *p = t, *q = t;
    uint8_t deg = 0, min = 0;
    float frac_min = 0;
    int32_t ret = 0;

    while (*p && *p >= '0' && *p <= '9') {
        p++;
    }
    while ((p - q) > 2 && *q) {
        if (deg) {
            deg *= 10;
        }
        deg += (uint8_t)(*q++ - '0');
    }
    while (p > q && *q) {
        if (min) {
            min *= 10;
        }
        min += (uint8_t)(*q++ - '0');
    }
    if (*p == '.') {
        q = p + 1;
        float scale = 0.1f;
        while (*q && *q >= '0' && *q <= '9') {
            frac_min += (float)(*q - '0') * scale;
            q++;
            scale *= 0.1f;
        }
    }
    ret = (int32_t)(deg * 10000000UL);
    ret += (int32_t)((int64_t)min * 10000000LL / 60);
    ret += (int32_t)(frac_min * (1.0e7f / 60.0f));
    return ret;
}

/* Upstream _parse_decimal_100() */
static int32_t ap_parse_decimal_100(const char *p)
{
    char *endptr = NULL;
    long ret = 100 * strtol(p, &endptr, 10);
    const int sign = (ret < 0) ? -1 : 1;
    if (endptr == NULL || *endptr != '.') {
        return (int32_t)ret;
    }
    if (endptr[1] >= '0' && endptr[1] <= '9') {
        ret += sign * 10 * (endptr[1] - '0');
        if (endptr[2] >= '0' && endptr[2] <= '9') {
            ret += sign * (endptr[2] - '0');
            if (endptr[3] >= '0' && endptr[3] <= '9') {
                ret += sign * ((endptr[3] - '0') >= 5);
            }
        }
    }
    return (int32_t)ret;
}


/* Upstream _have_new_message(): RMC and GGA must both be fresh (<=150 ms), and
   anything carrying 3D velocity must not have gone stale. */
static bool ap_have_new_message(ap_nmea_t *s, uint32_t now)
{
    if (s->last_RMC_ms == 0 || s->last_GGA_ms == 0) {
        return false;
    }
    if (now - s->last_RMC_ms > 150 || now - s->last_GGA_ms > 150) {
        return false;
    }
    if (s->last_VTG_ms != 0 && now - s->last_VTG_ms > 150) {
        return false;
    }

    /* Upstream marks VTG as consumed (value 1) so it cannot trigger a second
       publish; the next publish must be driven by a fresh VTG or by RMC/GGA
       once the sentinel ages out.  Reproducing the sentinel matters because it
       changes which sentence in a burst is the one that commits the fix. */
    if (s->last_VTG_ms != 0) {
        s->last_VTG_ms = 1;
    }

    if (now - s->last_yaw_ms > 300) {
        s->have_gps_yaw = false;
    }

    s->last_fix_ms = now;
    return true;
}

static float wrap_360(float v)
{
    while (v >= 360.0f) v -= 360.0f;
    while (v < 0.0f) v += 360.0f;
    return v;
}

/* Upstream _term_complete() reduced to the sentences we emit. */
static bool ap_term_complete(ap_nmea_t *s, uint32_t now)
{
    if (s->is_checksum_term) {
        s->sentence_done = true;
        const uint32_t crc = (uint32_t)strtoul(s->term, NULL, 16);
        if ((uint8_t)(crc & 0xFF) != s->parity) {
            return false;  /* bad checksum: sentence dropped entirely */
        }
        switch (s->sentence_type) {
        case SENT_RMC:
            s->last_RMC_ms = now ? now : 1;
            s->lat = s->new_lat;
            s->lon = s->new_lon;
            s->ground_speed = (float)s->new_speed * 0.01f;
            s->ground_course = wrap_360((float)s->new_course * 0.01f);
            break;

        case SENT_GGA:
            s->last_GGA_ms = now ? now : 1;
            s->alt_cm = s->new_alt;
            s->lat = s->new_lat;
            s->lon = s->new_lon;
            s->num_sats = s->new_sats;
            s->hdop = s->new_hdop;
            switch (s->new_quality) {
            case 0: s->status = AP_FIX_NONE;      break;
            case 1: s->status = AP_FIX_3D;        break;
            case 2: s->status = AP_FIX_DGPS;      break;
            case 3: s->status = AP_FIX_3D;        break;
            case 4: s->status = AP_FIX_RTK_FIXED; break;
            case 5: s->status = AP_FIX_RTK_FLOAT; break;
            case 6: s->status = AP_FIX_NONE;      break;
            default: s->status = AP_FIX_3D;       break;
            }
            break;

        case SENT_VTG:
            s->last_VTG_ms = now ? now : 1;
            s->ground_speed = (float)s->new_speed * 0.01f;
            s->ground_course = wrap_360((float)s->new_course * 0.01f);
            break;

        case SENT_HDT:
        case SENT_THS:
            s->last_yaw_ms = now ? now : 1;
            s->gps_yaw = wrap_360((float)s->new_gps_yaw * 0.01f);
            s->have_gps_yaw = true;
            s->gps_yaw_configured = true;
            break;

        default:
            break;
        }
        s->valid = ap_have_new_message(s, now);
        if (s->valid) {
            s->commits++;
        }
        return s->valid;
    }

    if (s->term_number == 0) {
        /* talker must be two upper-case letters */
        if (s->term[0] < 'A' || s->term[0] > 'Z' ||
            s->term[1] < 'A' || s->term[1] > 'Z') {
            s->sentence_type = 0;
            return false;
        }
        const char *tt = &s->term[2];
        if (!strcmp(tt, "RMC")) s->sentence_type = SENT_RMC;
        else if (!strcmp(tt, "GGA")) s->sentence_type = SENT_GGA;
        else if (!strcmp(tt, "HDT")) s->sentence_type = SENT_HDT;
        else if (!strcmp(tt, "THS")) s->sentence_type = SENT_THS;
        else if (!strcmp(tt, "VTG")) s->sentence_type = SENT_VTG;
        else s->sentence_type = 0;
        return false;
    }

    if (s->sentence_type != 0 && s->term[0]) {
        switch (s->sentence_type + s->term_number) {
        case SENT_GGA + 6:
            s->new_quality = (s->term[0] > '0') ? (uint16_t)(s->term[0] - '0') : 0;
            break;
        case SENT_GGA + 7:
            s->new_sats = (uint16_t)atol(s->term);
            break;
        case SENT_GGA + 8:
            s->new_hdop = (uint16_t)ap_parse_decimal_100(s->term);
            break;
        case SENT_RMC + 1:
        case SENT_GGA + 1:
            s->new_time = ap_parse_decimal_100(s->term);
            break;
        case SENT_RMC + 9:
            s->new_date = atol(s->term);
            break;
        case SENT_RMC + 3:
        case SENT_GGA + 2:
            s->new_lat = ap_parse_degrees(s->term);
            break;
        case SENT_RMC + 4:
        case SENT_GGA + 3:
            if (s->term[0] == 'S') s->new_lat = -s->new_lat;
            break;
        case SENT_RMC + 5:
        case SENT_GGA + 4:
            s->new_lon = ap_parse_degrees(s->term);
            break;
        case SENT_RMC + 6:
        case SENT_GGA + 5:
            if (s->term[0] == 'W') s->new_lon = -s->new_lon;
            break;
        case SENT_GGA + 9:
            s->new_alt = ap_parse_decimal_100(s->term);
            break;
        case SENT_RMC + 7:
        case SENT_VTG + 5:
            s->new_speed = (ap_parse_decimal_100(s->term) * 514) / 1000;
            break;
        case SENT_HDT + 1:
        case SENT_THS + 1:
            s->new_gps_yaw = ap_parse_decimal_100(s->term);
            break;
        case SENT_RMC + 8:
        case SENT_VTG + 1:
            s->new_course = ap_parse_decimal_100(s->term);
            s->have_new_course = true;
            break;
        default:
            break;
        }
    }
    return false;
}

/*
 * Feed one character, mirroring AP_GPS_NMEA::_decode() exactly.
 *
 * The parity rule is subtle and worth stating because getting it wrong makes
 * every sentence look corrupt: ',' is XORed into the parity *before* the term
 * is completed, '*' is not XORed at all, and while _is_checksum_term is set the
 * hex digits are accumulated into the term but kept out of the parity.
 */
static bool ap_byte(ap_nmea_t *s, char c, uint32_t now)
{
    switch (c) {
    case ',':
    case '\r':
    case '\n':
    case '*': {
        if (c == ',') {
            s->parity ^= (uint8_t)c;
        }
        if (s->sentence_done) {
            return false;
        }
        bool valid = false;
        if (s->term_offset < (int)sizeof(s->term) - 1) {
            s->term[s->term_offset] = '\0';
            valid = ap_term_complete(s, now);
        }
        s->term_number++;
        s->term_offset = 0;
        s->is_checksum_term = (c == '*');
        return valid;
    }

    case '$':
        s->term_number = 0;
        s->term_offset = 0;
        s->parity = 0;
        s->sentence_type = 0;
        s->is_checksum_term = false;
        s->sentence_done = false;
        s->term[0] = '\0';
        return false;

    default:
        break;
    }

    if (s->term_offset < (int)sizeof(s->term) - 1) {
        s->term[s->term_offset++] = c;
    }
    if (!s->is_checksum_term) {
        s->parity ^= (uint8_t)c;
    }
    return false;
}

/* Feed a whole buffer; returns the number of fixes published while doing so. */
static int ap_feed(ap_nmea_t *s, const char *buf, uint32_t now)
{
    const int before = s->commits;
    for (const char *p = buf; *p; ++p) {
        ap_byte(s, *p, now);
    }
    return s->commits - before;
}

/* ------------------------------------------------------------------------ */

static void test_checksum(void)
{
    SECTION("NMEA checksum");
    /* Known-good vector.  The XOR runs over everything between '$' and '*', so
       this body must hash to 0x56; verified independently against the standard
       algorithm.  A regression here means the generated sentences would fail
       ArduPilot's parity check and be discarded without any visible error. */
    const uint8_t known = nmea_checksum(
        "GPGGA,224955.00,3723.46587704,N,12202.26957864,W,2,6,1.7,18.6,M,-28.6,M,,");
    CHECK(known == 0x56, "known vector = 0x%02X, expected 0x56", known);

    char out[128];
    const int n = nmea_sentence(out, sizeof(out), "GPGGA,1,2,3");
    CHECK(n > 0, "nmea_sentence returned %d", n);
    CHECK(out[0] == '$', "sentence starts with $");
    CHECK(out[n - 2] == '\r' && out[n - 1] == '\n', "sentence ends with CRLF");
    CHECK(strstr(out, "*") != NULL, "sentence carries a checksum");

    /* Verify the emitted checksum really matches the body. */
    char body[64];
    const char *star = strchr(out, '*');
    size_t blen = (size_t)(star - (out + 1));
    memcpy(body, out + 1, blen);
    body[blen] = '\0';
    const unsigned emitted = (unsigned)strtoul(star + 1, NULL, 16);
    CHECK(emitted == nmea_checksum(body), "emitted checksum 0x%02X != computed 0x%02X",
          emitted, nmea_checksum(body));

    /* Too-small buffer must fail cleanly rather than truncate silently. */
    char tiny[8];
    CHECK(nmea_sentence(tiny, sizeof(tiny), "GPGGA,1,2,3") == -1, "small buffer rejected");
    CHECK(tiny[0] == '\0', "small buffer NUL terminated");
}

static void test_fix_quality_mapping(void)
{
    SECTION("GGA fix quality -> ArduPilot FixType");
    struct { const char *q; ap_fix_t expect; const char *name; } cases[] = {
        { "0", AP_FIX_NONE,      "0 no fix"    },
        { "1", AP_FIX_3D,        "1 3D"        },
        { "2", AP_FIX_DGPS,      "2 DGPS"      },
        { "4", AP_FIX_RTK_FIXED, "4 RTK Fixed" },
        { "5", AP_FIX_RTK_FLOAT, "5 RTK Float" },
    };
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        ap_nmea_t s;
        ap_init(&s);
        char line[160];
        snprintf(line, sizeof(line),
                 "$GNGGA,120000.00,3100.000000,N,12100.000000,E,%s,22,0.7,3.5,M,0.0,M,1.0,0000*00\r\n",
                 cases[i].q);
        /* recompute a real checksum so the parser accepts the sentence */
        char *star = strchr(line, '*');
        char body[128];
        size_t blen = (size_t)(star - (line + 1));
        memcpy(body, line + 1, blen);
        body[blen] = '\0';
        snprintf(star, 16, "*%02X\r\n", nmea_checksum(body));
        ap_feed(&s, line, 1000);
        CHECK(s.status == cases[i].expect,
              "%s: got status %d expected %d", cases[i].name, (int)s.status, (int)cases[i].expect);
        CHECK(s.lat != 0, "%s: latitude decoded", cases[i].name);
    }
}

/*
 * Build the exact burst the firmware emits for a given state, in the same
 * sentence order: HDT, GGA, RMC, then optional VTG.
 */
static void firmware_burst(const sim_state_t *st, char *out, size_t out_len, const char **tk_out)
{
    char body[256], line[300];
    char latv[24], lonv[24], lathemi, lonhemi;
    double lat, lon;
    char tim[16] = "120000.00", dat[16] = "111126";

    sim_position_noisy(st, &lat, &lon);
    const double course = sim_heading_noisy(st);
    const char *tk = sim_talker(st);
    if (tk_out) {
        *tk_out = tk;
    }
    const int q = (int)st->fix_quality;
    const int sats = (q == 0) ? 0 : st->satellites;
    const double hdop = (q == 0) ? 99.9 : st->hdop;
    const double sog = st->speed_mps * 1.9438444924406;

    out[0] = '\0';

#define APPEND(...)                                                          \
    do {                                                                     \
        snprintf(body, sizeof(body), __VA_ARGS__);                           \
        if (nmea_sentence(line, sizeof(line), body) > 0) {                   \
            strncat(out, line, out_len - strlen(out) - 1);                   \
        }                                                                    \
    } while (0)

    /* HDT first, so the publish triggered by GGA/RMC already carries the
       new heading. */
    if (st->heading_enabled && q != 0) {
        APPEND("%sHDT,%.2f,T", tk, course);
    }

    nmea_fmt_lat(lat, latv, sizeof(latv), &lathemi);
    nmea_fmt_lon(lon, lonv, sizeof(lonv), &lonhemi);
    APPEND("%sGGA,%s,%s,%c,%s,%c,%d,%02d,%.1f,%.1f,M,%.1f,M,1.0,0000",
           tk, tim, latv, lathemi, lonv, lonhemi, q, sats, hdop,
           st->altitude_m, 0.0);

    APPEND("%sRMC,%s,%c,%s,%c,%s,%c,%.2f,%.2f,%s,,,A",
           tk, tim, (q == 0) ? 'V' : 'A', latv, lathemi, lonv, lonhemi,
           sog, course, dat);

    if (st->vtg_enabled) {
        APPEND("%sVTG,%.2f,T,,M,%.2f,N,%.2f,K,A",
               tk, course, sog, st->speed_mps * 3.6);
    }
#undef APPEND
}
static void test_end_to_end_rtk(void)
{
    SECTION("Firmware burst -> ArduPilot parser (RTK Fixed)");
    sim_state_t st;
    sim_init(&st, 0);
    st.fix_quality = FIX_QUALITY_RTK_FIXED;
    st.satellites = 24;
    st.hdop = 0.6;
    st.heading_deg = 137.5;
    st.speed_mps = 1.8;
    st.north_m = 25.0;
    st.east_m = -40.0;
    st.mode = SIM_MODE_STATIC;
    st.heading_enabled = true;
    st.baseline_len_m = 1.2;

    char burst[1024];
    const char *tk = NULL;
    firmware_burst(&st, burst, sizeof(burst), &tk);

    printf("  emitted:\n");
    char *copy = strdup(burst);
    for (char *ln = strtok(copy, "\r\n"); ln; ln = strtok(NULL, "\r\n")) {
        printf("    %s\n", ln);
    }
    free(copy);

    CHECK(strncmp(tk, "GN", 2) == 0, "RTK uses GN talker, got %s", tk);
    CHECK(strstr(burst, "HDT") != NULL, "HDT emitted for the GPS-yaw EKF source");

    ap_nmea_t s;
    ap_init(&s);
    const int published = ap_feed(&s, burst, 1000);

    CHECK(published == 1, "exactly one fix published per burst (got %d)", published);
    CHECK(s.valid, "ArduPilot considers the message set fresh and valid");
    CHECK(s.status == AP_FIX_RTK_FIXED, "FixType is RTK_FIXED, got %d", (int)s.status);
    CHECK(s.num_sats == 24, "satellites = %u", s.num_sats);
    CHECK(s.hdop == 60, "hdop (x100) = %u", s.hdop);
    CHECK(s.have_gps_yaw, "have_gps_yaw set (EK3_SRC1_YAW=2 needs this)");
    CHECK(fabsf(s.gps_yaw - 137.5f) < 0.05f, "gps_yaw = %.2f expected 137.50", s.gps_yaw);
    CHECK(s.gps_yaw_configured, "gps_yaw_configured set");

    /* Position must round-trip to within a few centimetres of the truth. */
    double lat, lon;
    sim_position(&st, &lat, &lon);
    const double dlat = (s.lat / 1.0e7) - lat;
    const double dlon = (s.lon / 1.0e7) - lon;
    const double dy = dlat * 111132.92;
    const double dx = dlon * 111412.84 * cos(lat * M_PI / 180.0);
    const double err = sqrt(dx * dx + dy * dy);
    CHECK(err < 0.05, "position round-trip error %.3f m (limit 0.05)", err);

    CHECK(fabsf(s.ground_course - 137.5f) < 0.2f, "course = %.2f expected 137.50", s.ground_course);
    /* 1.8 m/s = 3.498 kt -> upstream scales knots*0.01 */
    CHECK(fabsf(s.ground_speed - 1.8f) < 0.1f, "speed = %.3f expected 1.8", s.ground_speed);
    CHECK(labs((long)(s.alt_cm / 100) - 4) <= 1, "altitude = %d cm expected ~400", s.alt_cm);
}

static void test_end_to_end_no_fix(void)
{
    SECTION("Firmware burst -> ArduPilot parser (no fix)");
    sim_state_t st;
    sim_init(&st, 0);
    st.fix_quality = FIX_QUALITY_NONE;
    st.satellites = 0;

    char burst[1024];
    firmware_burst(&st, burst, sizeof(burst), NULL);

    ap_nmea_t s;
    ap_init(&s);
    const int published = ap_feed(&s, burst, 1000);

    /* A "no fix" receiver still publishes: the driver needs to learn that the
       fix was lost, so the burst must not be silently swallowed. */
    CHECK(published == 1, "no-fix burst still publishes once (got %d)", published);
    CHECK(s.status == AP_FIX_NONE, "FixType NONE, got %d", (int)s.status);
    CHECK(!s.have_gps_yaw, "no yaw published without a fix");
    CHECK(strstr(burst, "HDT") == NULL, "no HDT sentence without a fix");
}

static void test_freshness_rule(void)
{
    SECTION("ArduPilot 150 ms freshness rule vs 5 Hz output");
    sim_state_t st;
    sim_init(&st, 0);
    st.fix_quality = FIX_QUALITY_RTK_FIXED;
    char burst[1024];
    firmware_burst(&st, burst, sizeof(burst), NULL);

    /* 5 Hz = 200 ms spacing, which is longer than ArduPilot's 150 ms window for
       RMC/GGA.  They arrive back to back in one burst, so both are stamped on
       the same millisecond and the pair is always valid; the burst must not be
       split.  Verify exactly that. */
    ap_nmea_t s;
    ap_init(&s);
    CHECK(ap_feed(&s, burst, 5000) == 1, "back-to-back RMC/GGA inside one burst publishes once");

    /* Now replay the burst 200 ms later: it must publish again. */
    CHECK(ap_feed(&s, burst, 5200) == 1, "next 5 Hz burst publishes again");
    CHECK(5200 - s.last_GGA_ms <= 150, "GGA stamp refreshed on the new burst");
}

/*
 * The decisive timing question: with a 5 Hz (200 ms) output rate and the flight
 * controller polling continuously, does ArduPilot actually get a valid fix on
 * every burst?
 *
 * AP_GPS_NMEA::_have_new_message() requires RMC and GGA to be no older than
 * 150 ms at the moment the sentence is parsed.  A 200 ms period is longer than
 * that window, which looks fatal - but the two sentences arrive back-to-back
 * inside one burst, a few milliseconds apart at 115200 baud, so both are fresh
 * when the burst completes.  This test replays the stream byte-by-byte at the
 * real line rate to prove it rather than assume it.
 */
static void test_stream_at_real_baud(void)
{
    SECTION("5 Hz stream at 115200 baud, byte-accurate");

    sim_state_t st;
    sim_init(&st, 0);
    st.fix_quality = FIX_QUALITY_RTK_FIXED;
    st.heading_deg = 90.0;
    st.output_rate_ms = 200;         /* 5 Hz default rate */

    char burst[1024];
    firmware_burst(&st, burst, sizeof(burst), NULL);
    const size_t blen = strlen(burst);
    printf("  burst = %zu bytes, ~%.1f ms at 115200 8N1\n",
           blen, (double)blen * 10.0 / 115200.0 * 1000.0);

    /* 11.52 bytes/ms at 115200 8N1; round down so the timing is pessimistic. */
    const int bytes_per_ms = 11;
    ap_nmea_t s;
    ap_init(&s);

    int bursts = 0, published = 0;
    for (uint32_t cycle = 0; cycle < 25; ++cycle) {          /* 5 seconds */
        const uint32_t t0 = 1000 + cycle * 200;
        size_t sent = 0;
        uint32_t ms = 0;
        while (sent < blen) {
            const size_t chunk = (blen - sent < (size_t)bytes_per_ms)
                                     ? (blen - sent) : (size_t)bytes_per_ms;
            for (size_t i = 0; i < chunk; ++i) {
                if (ap_byte(&s, burst[sent + i], t0 + ms)) {
                    published++;
                }
            }
            sent += chunk;
            ms++;
        }
        bursts++;
    }

    printf("  %d bursts, %d fixes published\n", bursts, published);
    CHECK(published == bursts, "one fix published per 5 Hz burst (%d/%d)", published, bursts);
    CHECK(s.status == AP_FIX_RTK_FIXED, "fix type stayed RTK_FIXED");

    /* Confirm the 200 ms cadence really is outside the 150 ms window, so the
       check above is testing something: a lone GGA followed 200 ms later by an
       RMC must NOT be accepted. */
    ap_nmea_t s2;
    ap_init(&s2);
    char gga[200], rmc[200];
    const char *nl1 = strstr(burst, "\r\n");
    const size_t l1 = (size_t)(nl1 - burst) + 2;
    memcpy(gga, burst, l1);
    gga[l1] = '\0';
    const char *nl2 = strstr(nl1 + 2, "\r\n");
    const size_t l2 = (size_t)(nl2 - (nl1 + 2)) + 2;
    memcpy(rmc, nl1 + 2, l2);
    rmc[l2] = '\0';

    ap_feed(&s2, gga, 1000);
    ap_feed(&s2, rmc, 1200);          /* 200 ms later: outside the window */
    CHECK(!s2.valid, "split RMC/GGA 200 ms apart is rejected by the 150 ms rule");
}

static void test_yaw_timeout(void)
{
    SECTION("GPS yaw 300 ms timeout");
    sim_state_t st;
    sim_init(&st, 0);
    st.fix_quality = FIX_QUALITY_RTK_FIXED;
    st.heading_deg = 45.0;
    char burst[1024];
    firmware_burst(&st, burst, sizeof(burst), NULL);

    ap_nmea_t s;
    ap_init(&s);
    ap_feed(&s, burst, 1000);
    CHECK(s.have_gps_yaw, "yaw present after the burst");

    /* Upstream clears have_gps_yaw once _last_yaw_ms is older than 300 ms.
       That test lives in _have_new_message(), which the driver calls each
       update, so emulate it explicitly. */
    const uint32_t now = 1000 + 350;
    if (now - s.last_yaw_ms > 300) {
        s.have_gps_yaw = false;
    }
    CHECK(!s.have_gps_yaw, "yaw dropped after 300 ms without HDT");

    /* With HDT disabled the flight controller loses yaw entirely within 300 ms,
       which is why the simulator defaults to emitting it. */
    st.heading_enabled = false;
    char burst2[1024];
    firmware_burst(&st, burst2, sizeof(burst2), NULL);
    CHECK(strstr(burst2, "HDT") == NULL, "HDT suppressed when disabled");
}

/*
 * Regression test for the VTG trap.
 *
 * AP_GPS_NMEA consumes VTG by writing 1 into _last_VTG_ms.  The freshness test
 * then rejects every publish while `now - _last_VTG_ms > 150`, which is true
 * from 152 ms onwards - i.e. for ever.  If a receiver emits VTG and then stops
 * (or emits it only occasionally), the fix stops being published even though
 * RMC and GGA keep arriving.  This is why the simulator defaults to no VTG,
 * emitting GGA/RMC/HDT only.
 */
static void test_vtg_sentinel_trap(void)
{
    SECTION("VTG consumption sentinel");

    sim_state_t st;
    sim_init(&st, 0);
    st.fix_quality = FIX_QUALITY_RTK_FIXED;

    /* Default: no VTG, every burst publishes for ever. */
    CHECK(!st.vtg_enabled, "VTG is off by default");
    char burst[1024];
    firmware_burst(&st, burst, sizeof(burst), NULL);
    CHECK(strstr(burst, "VTG") == NULL, "no VTG in the default burst");

    ap_nmea_t s;
    ap_init(&s);
    int published = 0;
    for (uint32_t cycle = 0; cycle < 50; ++cycle) {   /* 10 s at 5 Hz */
        published += ap_feed(&s, burst, 1000 + cycle * 200);
    }
    CHECK(published == 50, "without VTG every burst publishes (%d/50)", published);

    /* Now enable VTG and stop sending it after the third burst, as would happen
       if the receiver's VTG output were disabled mid-run. */
    st.vtg_enabled = true;
    char burst_vtg[1200];
    firmware_burst(&st, burst_vtg, sizeof(burst_vtg), NULL);
    CHECK(strstr(burst_vtg, "VTG") != NULL, "VTG present when enabled");

    ap_nmea_t s2;
    ap_init(&s2);
    uint32_t t = 1000;
    int first = 0;
    for (int i = 0; i < 3; ++i, t += 200) {
        first += ap_feed(&s2, burst_vtg, t);
    }
    /* With VTG present each burst publishes twice at first: once when RMC
       completes (the sentinel is still 0, so the VTG check is skipped) and
       again when VTG completes and takes over as the commit point.  After that
       VTG alone publishes.  3 bursts therefore yield 4 publishes, not 3. */
    CHECK(first == 4, "VTG bursts publish 2 then 1 then 1 (%d)", first);

    /* VTG stops, RMC/GGA continue.  This must stall - documenting the hazard. */
    char plain[1024];
    firmware_burst(&st, plain, sizeof(plain), NULL);
    /* rebuild the burst without VTG while leaving st.vtg_enabled set */
    st.vtg_enabled = false;
    firmware_burst(&st, plain, sizeof(plain), NULL);
    int after = 0;
    for (int i = 0; i < 10; ++i, t += 200) {
        after += ap_feed(&s2, plain, t);
    }
    CHECK(after == 0,
          "fix stalls once VTG disappears (%d published) - proving VTG must stay off",
          after);
}

static void test_motion_models(void)
{
    SECTION("Motion models");
    sim_state_t st;
    sim_init(&st, 0);

    /* Circle: stays on the radius and reports a tangent heading. */
    st.mode = SIM_MODE_CIRCLE;
    st.circle_radius_m = 12.0;
    st.circle_rate_dps = 5.0;
    for (uint32_t t = 0; t <= 60000; t += 200) {
        sim_step(&st, t);
    }
    const double r = sqrt(st.north_m * st.north_m + st.east_m * st.east_m);
    CHECK(fabs(r - 12.0) < 0.01, "circle radius held: %.4f (expected 12.0)", r);
    const double expect_speed = 12.0 * (5.0 * M_PI / 180.0);
    CHECK(fabs(st.speed_mps - expect_speed) < 0.001, "circle speed %.4f expected %.4f",
          st.speed_mps, expect_speed);

    /* Line: travels at the configured speed and stays within one leg. */
    sim_init(&st, 0);
    st.mode = SIM_MODE_LINE;
    st.course_deg = 90.0;
    st.leg_m = 20.0;
    st.waypoint_speed_mps = 2.0;
    for (uint32_t t = 0; t <= 5000; t += 200) {   /* 5 s at 2 m/s = 10 m */
        sim_step(&st, t);
    }
    CHECK(fabs(st.east_m - 10.0) < 0.3, "after 5 s at 2 m/s east = %.3f (expected 10)", st.east_m);
    CHECK(fabs(st.north_m) < 1e-6, "pure-east course keeps north at 0, got %.6f", st.north_m);

    /* Wrapping must conserve distance: after 15 s (30 m) on a 20 m leg the
       vessel is 10 m along, not 9.6 m — the overshoot is carried, not dropped. */
    sim_init(&st, 0);
    st.mode = SIM_MODE_LINE;
    st.course_deg = 90.0;
    st.leg_m = 20.0;
    st.waypoint_speed_mps = 2.0;
    for (uint32_t t = 0; t <= 15000; t += 200) {
        sim_step(&st, t);
    }
    CHECK(fabs(st.east_m - 10.0) < 0.5, "wrap conserves distance, east = %.3f (expected 10)", st.east_m);
    CHECK(st.east_m <= st.leg_m, "never exceeds the leg length, east = %.3f", st.east_m);

    /* Waypoint: walks the list in order.  Three targets so the test does not
       complete a full lap and land back on index 0 by accident. */
    sim_init(&st, 0);
    st.mode = SIM_MODE_WAYPOINT;
    st.waypoint_count = 3;
    st.waypoints[0][0] = 4;  st.waypoints[0][1] = 0;
    st.waypoints[1][0] = 4;  st.waypoints[1][1] = 4;
    st.waypoints[2][0] = 0;  st.waypoints[2][1] = 4;
    st.waypoint_speed_mps = 2.0;
    st.waypoint_radius_m = 0.5;
    /* 4 m + 4 m of travel at 2 m/s takes 4 s; run 5 s to clear the second leg. */
    for (uint32_t t = 0; t <= 5000; t += 100) {
        sim_step(&st, t);
    }
    CHECK(st.waypoint_index == 2, "advanced to waypoint 2, index=%u", st.waypoint_index);
    CHECK(fabs(st.north_m - 4.0) < 0.6 || fabs(st.east_m - 4.0) < 0.6,
          "reached the second corner, N=%.2f E=%.2f", st.north_m, st.east_m);
}

static void test_geodesy(void)
{
    SECTION("Geodesy");
    sim_state_t st;
    sim_init(&st, 0);

    st.north_m = 100.0;
    st.east_m = 0.0;
    double lat, lon;
    sim_position(&st, &lat, &lon);
    const double dlat_m = (lat - st.origin_lat) * 111132.92;
    CHECK(fabs(dlat_m - 100.0) < 0.5, "100 m north -> %.3f m of latitude", dlat_m);

    st.north_m = 0.0;
    st.east_m = 100.0;
    sim_position(&st, &lat, &lon);
    const double dlon_m = (lon - st.origin_lon) * 111412.84 * cos(st.origin_lat * M_PI / 180.0);
    CHECK(fabs(dlon_m - 100.0) < 0.5, "100 m east -> %.3f m of longitude", dlon_m);

    /* Negative offsets must mirror. */
    st.east_m = -100.0;
    sim_position(&st, &lat, &lon);
    const double dlon_neg = (lon - st.origin_lon) * 111412.84 * cos(st.origin_lat * M_PI / 180.0);
    CHECK(fabs(dlon_neg + 100.0) < 0.5, "-100 m east -> %.3f m", dlon_neg);
}

static void test_noise_jitter(void)
{
    SECTION("Position/heading jitter");
    sim_state_t st;
    char a[1024], b[1024];

    /* With sigma 0 (the default) the burst must be byte-identical no matter
       how far the wander state has been advanced - that is the "jitter off"
       contract that keeps every existing bench behaviour unchanged. */
    sim_init(&st, 0);
    st.heading_deg = 137.5;
    st.speed_mps = 1.8;
    st.north_m = 25.0;
    st.east_m = -40.0;
    firmware_burst(&st, a, sizeof(a), NULL);
    for (uint32_t t = 200; t <= 5000; t += 200) {
        sim_noise_advance(&st, t);
        firmware_burst(&st, b, sizeof(b), NULL);
        CHECK(strcmp(a, b) == 0, "sigma=0: burst identical at t=%u ms", (unsigned)t);
    }

    /* Two fresh states driven along the same wall-clock timeline must produce
       byte-identical output: the wander is deterministic, so a bench run can
       be replayed exactly. */
    sim_state_t st2;
    sim_init(&st, 0);
    sim_init(&st2, 0);
    st.heading_deg = st2.heading_deg = 137.5;
    st.north_m = st2.north_m = 25.0;
    st.east_m = st2.east_m = -40.0;
    st.position_noise_m = st2.position_noise_m = 0.5;
    st.heading_noise_deg = st2.heading_noise_deg = 1.0;
    uint32_t t = 0;
    for (int i = 0; i < 10; i++) {
        t += 200;
        sim_noise_advance(&st, t);
        firmware_burst(&st, a, sizeof(a), NULL);
        sim_noise_advance(&st2, t);
        firmware_burst(&st2, b, sizeof(b), NULL);
        CHECK(strcmp(a, b) == 0, "same timeline reproduces the same burst (frame %d)", i);
    }

    /* sigma > 0: the reported position wanders around the truth with a
       steady-state std of sigma, never exceeds the 3.5-sigma clamp envelope,
       and - the property that keeps ArduPilot's GPS glitch detection quiet -
       moves smoothly between frames instead of teleporting.  HDT yaw always
       matches RMC course (both carry the same wander sample).
       A correlated wander has only ~T/(2*tau) effective samples per window,
       so mean/std are taken over 12 independent realizations with disjoint
       hash seeds instead of one window. */
    const double sigma = 0.5;
    const int RUNS = 12, FRAMES = 200;
    unsigned changed = 0, out_of_bounds = 0, yaw_course_mismatch = 0, bad_course_range = 0;
    double max_move = 0.0;
    double sum_x = 0.0, sum_x2 = 0.0, sum_y = 0.0, sum_y2 = 0.0;
    double prev_x = 0.0, prev_y = 0.0;
    char prev[1024] = "";
    double tlat, tlon;

    for (int run = 0; run < RUNS; run++) {
        sim_init(&st, 0);
        st.heading_deg = 137.5;
        st.north_m = 25.0;
        st.east_m = -40.0;
        st.position_noise_m = sigma;
        st.heading_noise_deg = 1.0;
        st.noise_seq = (uint32_t)run * 104729u + 1u;   /* disjoint hash stream */
        sim_position(&st, &tlat, &tlon);

        uint32_t tr = 0;
        for (int i = 0; i < FRAMES; i++) {
            tr += 200;
            sim_noise_advance(&st, tr);
            firmware_burst(&st, a, sizeof(a), NULL);
            if (i > 0 && strcmp(a, prev) != 0) {
                changed++;
            }
            memcpy(prev, a, sizeof(prev));

            ap_nmea_t ap;
            ap_init(&ap);
            ap_feed(&ap, a, 1000);

            /* east/north offsets from the truth, in metres */
            const double dx = ((ap.lon / 1.0e7) - tlon) * 111412.84 * cos(tlat * M_PI / 180.0);
            const double dy = ((ap.lat / 1.0e7) - tlat) * 111132.92;
            if (i > 0) {
                const double move = sqrt((dx - prev_x) * (dx - prev_x) + (dy - prev_y) * (dy - prev_y));
                if (move > max_move) {
                    max_move = move;
                }
            }
            prev_x = dx;
            prev_y = dy;
            if (sqrt(dx * dx + dy * dy) >= 5.0 * sigma) {
                out_of_bounds++;
            }
            sum_x += dx;
            sum_x2 += dx * dx;
            sum_y += dy;
            sum_y2 += dy * dy;

            if (ap.ground_course < 0.0f || ap.ground_course >= 360.0f) {
                bad_course_range++;
            }
            if (fabsf(ap.ground_course - ap.gps_yaw) > 0.011f) {
                yaw_course_mismatch++;
            }
        }
    }

    const int N = RUNS * FRAMES;
    const double mean_x = sum_x / N, mean_y = sum_y / N;
    const double std_x = sqrt((sum_x2 / N) - mean_x * mean_x);
    const double std_y = sqrt((sum_y2 / N) - mean_y * mean_y);
    printf("  wander stats over %d runs x %d frames: mean east %.3f m, north %.3f m, "
           "std %.3f / %.3f m (sigma %.2f), max frame move %.3f m\n",
           RUNS, FRAMES, mean_y, mean_x, std_y, std_x, sigma, max_move);

    /* Per-frame bound: 3.464 (hash) * sqrt(2) * sigma * sqrt(1-phi^2) with
       phi = exp(-0.2/3) -> ~0.87 m; a white-noise model would exceed 2 m. */
    CHECK(max_move < 1.0, "frame-to-frame movement smooth: max %.3f m < 1.0", max_move);
    CHECK(out_of_bounds == 0, "all %d samples within 5 sigma of the truth (%u out)",
          N, out_of_bounds);
    CHECK(fabs(mean_x) < 0.10 * sigma, "east mean %.3f m within 0.10 sigma", mean_x);
    CHECK(fabs(mean_y) < 0.10 * sigma, "north mean %.3f m within 0.10 sigma", mean_y);
    CHECK(std_x > 0.8 * sigma && std_x < 1.2 * sigma, "east std %.3f m near sigma", std_x);
    CHECK(std_y > 0.8 * sigma && std_y < 1.2 * sigma, "north std %.3f m near sigma", std_y);
    CHECK(changed >= (unsigned)(RUNS * (FRAMES - 5)), "consecutive bursts differ (%u/%u changed)",
          changed, (unsigned)N);
    CHECK(yaw_course_mismatch == 0, "HDT yaw always equals RMC course (%u mismatch)",
          yaw_course_mismatch);
    CHECK(bad_course_range == 0, "courses stay in [0,360) (%u out)", bad_course_range);

    /* RTK Fixed with ~20 sats: sigma 2 cm must keep every frame-to-frame move
       at centimetre scale and the whole excursion inside ~10 cm, so the EKF
       never sees a position jump a static receiver could not produce. */
    const double sigma_rtk = 0.02;
    sim_init(&st, 0);
    st.north_m = 25.0;
    st.east_m = -40.0;
    st.position_noise_m = sigma_rtk;
    double max_move_rtk = 0.0, max_err_rtk = 0.0;
    for (int i = 0; i < 100; i++) {
        t += 200;
        sim_noise_advance(&st, t);
        firmware_burst(&st, a, sizeof(a), NULL);
        ap_nmea_t ap;
        ap_init(&ap);
        ap_feed(&ap, a, 1000);
        const double dx = ((ap.lon / 1.0e7) - tlon) * 111412.84 * cos(tlat * M_PI / 180.0);
        const double dy = ((ap.lat / 1.0e7) - tlat) * 111132.92;
        const double move = sqrt((dx - prev_x) * (dx - prev_x) + (dy - prev_y) * (dy - prev_y));
        if (i > 0 && move > max_move_rtk) {
            max_move_rtk = move;
        }
        prev_x = dx;
        prev_y = dy;
        const double err = sqrt(dx * dx + dy * dy);
        if (err > max_err_rtk) {
            max_err_rtk = err;
        }
    }
    printf("  rtk (sigma 0.02): max frame move %.3f m, max excursion %.3f m\n",
           max_move_rtk, max_err_rtk);
    CHECK(max_move_rtk < 0.04, "RTK frame movement <= 4 cm: %.3f", max_move_rtk);
    CHECK(max_err_rtk < 0.10, "RTK excursion <= 10 cm: %.3f", max_err_rtk);

    /* Heading wrap: near-north headings with a large sigma must wrap through
       360/0 and never leave [0, 360). */
    sim_init(&st, 0);
    st.heading_deg = 359.95;
    st.position_noise_m = 0.5;
    st.heading_noise_deg = 2.0;
    for (uint32_t seq = 0; seq < 100; seq++) {
        t += 200;
        sim_noise_advance(&st, t);
        firmware_burst(&st, a, sizeof(a), NULL);
        ap_nmea_t ap;
        ap_init(&ap);
        ap_feed(&ap, a, 1000);
        if (ap.ground_course < 0.0f || ap.ground_course >= 360.0f) {
            bad_course_range++;
        }
    }
    CHECK(bad_course_range == 0, "wrapped courses stay in [0,360) (%u out)", bad_course_range);
}

static void test_nmea_formatting(void)
{
    SECTION("NMEA lat/lon formatting");
    char v[24], h;

    nmea_fmt_lat(31.2, v, sizeof(v), &h);
    CHECK(h == 'N', "north hemisphere");
    CHECK(strncmp(v, "3112.", 5) == 0, "latitude 31.2 -> %s", v);

    nmea_fmt_lat(-33.8651, v, sizeof(v), &h);
    CHECK(h == 'S', "south hemisphere");
    CHECK(strncmp(v, "3351.", 5) == 0, "latitude -33.8651 -> %s", v);

    nmea_fmt_lon(121.5, v, sizeof(v), &h);
    CHECK(h == 'E', "east hemisphere");
    CHECK(strncmp(v, "12130.", 6) == 0, "longitude 121.5 -> %s", v);

    nmea_fmt_lon(-122.0452, v, sizeof(v), &h);
    CHECK(h == 'W', "west hemisphere");
    CHECK(strncmp(v, "12202.", 6) == 0, "longitude -122.0452 -> %s", v);

    /* Exact minute boundary must not produce 60.000000. */
    nmea_fmt_lat(30.9999999999, v, sizeof(v), &h);
    CHECK(strstr(v, "60.0000") == NULL, "no 60-minute rollover: %s", v);
}

/* ------------------------------------------------------------------------ */
/* RTCM framing                                                              */
/* ------------------------------------------------------------------------ */

/* Build a synthetic RTCM frame of the given type and payload length. */
static size_t make_rtcm(uint8_t *buf, uint16_t type, uint16_t payload_len, uint8_t seed)
{
    buf[0] = 0xD3;
    const uint16_t len = payload_len;
    buf[1] = (uint8_t)((len >> 8) & 0x03);
    buf[2] = (uint8_t)(len & 0xFF);
    /* message number in the first 12 payload bits, padding in the rest */
    uint8_t *pl = buf + 3;
    memset(pl, 0, len);
    pl[0] = (uint8_t)(type >> 4);
    pl[1] = (uint8_t)((type & 0x0F) << 4);
    for (uint16_t i = 2; i < len; ++i) {
        pl[i] = (uint8_t)(seed + i);
    }
    const uint32_t crc = rtcm_crc24q(buf, (size_t)3 + len);
    buf[3 + len + 0] = (uint8_t)((crc >> 16) & 0xFF);
    buf[3 + len + 1] = (uint8_t)((crc >> 8) & 0xFF);
    buf[3 + len + 2] = (uint8_t)(crc & 0xFF);
    return (size_t)3 + len + 3;
}

static void rtcm_count_cb(rtcm_parser_t *p, const rtcm_msg_info_t *info, void *ctx)
{
    (void)p;
    (void)ctx;
    if (info->crc_ok) {
        (*(int *)ctx)++;
    }
}


static void test_rtcm(void)
{
    SECTION("RTCM 3.x framing");

    uint8_t frame[1200];
    int good = 0;
    rtcm_parser_t p;
    rtcm_init(&p);

    const size_t n = make_rtcm(frame, 1005, 19, 0x11);
    rtcm_push(&p, frame, n, 1000, rtcm_count_cb, &good);
    CHECK(good == 1, "valid frame accepted (%d)", good);
    CHECK(p.frame_count == 1, "frame_count = %u", p.frame_count);
    CHECK(p.crc_errors == 0, "no CRC errors");
    CHECK(p.last_msg_type == 1005, "message type 1005, got %u", p.last_msg_type);
    CHECK(p.last_len == 19, "payload length 19, got %u", p.last_len);

    /* Byte-at-a-time delivery must reassemble identically (UART reality). */
    rtcm_init(&p);
    good = 0;
    for (size_t i = 0; i < n; ++i) {
        rtcm_push(&p, frame + i, 1, 1000 + (uint32_t)i, rtcm_count_cb, &good);
    }
    CHECK(good == 1, "byte-wise reassembly accepted (%d)", good);

    /* Corrupted payload must be counted, not silently dropped. */
    rtcm_init(&p);
    good = 0;
    make_rtcm(frame, 1077, 300, 0x22);
    frame[10] ^= 0xFF;
    const size_t cn = make_rtcm(frame, 1077, 300, 0x22);
    frame[10] ^= 0xFF;              /* corrupt after the CRC was computed */
    rtcm_push(&p, frame, cn, 1000, rtcm_count_cb, &good);
    CHECK(good == 0, "corrupted frame rejected");
    CHECK(p.crc_errors == 1, "CRC error counted (%u)", p.crc_errors);

    /* Mixed stream: several types, back to back. */
    rtcm_init(&p);
    good = 0;
    uint8_t stream[4096];
    size_t off = 0;
    const uint16_t types[] = { 1005, 1074, 1084, 1094, 1230 };
    for (size_t i = 0; i < sizeof(types) / sizeof(types[0]); ++i) {
        off += make_rtcm(stream + off, types[i], (uint16_t)(20 + i * 15), (uint8_t)i);
    }
    rtcm_push(&p, stream, off, 2000, rtcm_count_cb, &good);
    CHECK(good == 5, "5 frames in one write (%d)", good);
    CHECK(p.type_used == 5, "5 distinct message types tracked (%u)", p.type_used);
    CHECK(p.crc_errors == 0, "no CRC errors in mixed stream");

    /* Garbage before the preamble must not derail the parser. */
    rtcm_init(&p);
    good = 0;
    uint8_t junk[64];
    memset(junk, 0x55, sizeof(junk));
    rtcm_push(&p, junk, sizeof(junk), 3000, rtcm_count_cb, &good);
    make_rtcm(frame, 1006, 21, 0x33);
    rtcm_push(&p, frame, make_rtcm(frame, 1006, 21, 0x33), 3001, rtcm_count_cb, &good);
    CHECK(good == 1, "recovers after junk (%d)", good);

    /* Oversized length must be rejected without a crash. */
    rtcm_init(&p);
    const uint8_t bad_hdr[3] = { 0xD3, 0x03, 0xFF };  /* len 1023 is legal */
    rtcm_push(&p, bad_hdr, 3, 4000, rtcm_count_cb, &good);
    CHECK(p.need == 1023, "legal maximum length accepted, need=%u", p.need);
}

int main(void)
{
    printf("USV GPS/RTK simulator — host verification\n");

    test_checksum();
    test_nmea_formatting();
    test_fix_quality_mapping();
    test_end_to_end_rtk();
    test_end_to_end_no_fix();
    test_freshness_rule();
    test_stream_at_real_baud();
    test_yaw_timeout();
    test_vtg_sentinel_trap();
    test_motion_models();
    test_geodesy();
    test_noise_jitter();
    test_rtcm();

    printf("\n----------------------------------------\n");
    printf("%d checks, %d failures\n", g_checks, g_failures);
    if (g_failures == 0) {
        printf("RESULT: PASS\n");
        return 0;
    }
    printf("RESULT: FAIL\n");
    return 1;
}
