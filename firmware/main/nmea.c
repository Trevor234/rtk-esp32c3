#include "nmea.h"

#include <stdio.h>
#include <string.h>

uint8_t nmea_checksum(const char *body)
{
    uint8_t ck = 0;
    for (const char *p = body; *p; ++p) {
        ck ^= (uint8_t)*p;
    }
    return ck;
}

int nmea_sentence(char *out, size_t out_len, const char *body)
{
    const uint8_t ck = nmea_checksum(body);
    /* '$' + body + '*' + 2 hex + CRLF + NUL */
    const size_t need = strlen(body) + 7;
    if (out_len < need) {
        if (out_len > 0) {
            out[0] = '\0';
        }
        return -1;
    }
    const int n = snprintf(out, out_len, "$%s*%02X\r\n", body, (unsigned)ck);
    if (n < 0 || (size_t)n >= out_len) {
        if (out_len > 0) {
            out[0] = '\0';
        }
        return -1;
    }
    return n;
}

void nmea_fmt_lat(double deg, char *val, size_t val_len, char *hemi)
{
    *hemi = (deg < 0.0) ? 'S' : 'N';
    double a = (deg < 0.0) ? -deg : deg;
    int d = (int)a;
    double m = (a - (double)d) * 60.0;
    /* Guard against 59.999999 rounding up to 60. */
    if (m >= 59.9999995) {
        m = 0.0;
        d += 1;
    }
    snprintf(val, val_len, "%02d%09.6f", d, m);
}

void nmea_fmt_lon(double deg, char *val, size_t val_len, char *hemi)
{
    *hemi = (deg < 0.0) ? 'W' : 'E';
    double a = (deg < 0.0) ? -deg : deg;
    int d = (int)a;
    double m = (a - (double)d) * 60.0;
    if (m >= 59.9999995) {
        m = 0.0;
        d += 1;
    }
    snprintf(val, val_len, "%03d%09.6f", d, m);
}
