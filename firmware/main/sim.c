#include "sim.h"

#include <math.h>
#include <string.h>

/* M_PI is not guaranteed by strict C11 (and MinGW omits it), so carry our own. */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Metres per degree at the equator, and the latitude shrink factor. */
#define M_PER_DEG_LAT 111132.92
#define M_PER_DEG_LON_EQ 111412.84
#define EARTH_MEAN_R 6371008.8

static double deg2rad(double d) { return d * (M_PI / 180.0); }
static double rad2deg(double r) { return r * (180.0 / M_PI); }

static double wrap360(double d)
{
    d = fmod(d, 360.0);
    if (d < 0.0) {
        d += 360.0;
    }
    return d;
}

/* Deterministic white noise in [-1, 1] from a counter: keeps host tests stable
   and avoids pulling in a PRNG just for jitter. */
static double noise01(uint32_t k)
{
    uint32_t x = k * 2654435761u;
    x ^= x >> 15;
    x *= 2246822519u;
    x ^= x >> 13;
    return ((double)(x & 0xFFFFu) / 65535.0) * 2.0 - 1.0;
}

/* Unit-variance Gaussian-like sample from the same hash: four independent
   uniforms summed (Irwin-Hall) then scaled from std 2/sqrt(3) to 1.  Bounded
   at +/-3.46 sigma, which keeps a runaway sigma from producing absurd jumps.
   Distinct salt offsets give independent streams for lat / lon / heading. */
#define NOISE_SALT_LON     7919u
#define NOISE_SALT_HEADING 15485863u

static double noise_gauss(uint32_t k)
{
    const double acc = noise01(k)
                     + noise01(k ^ 0x9E3779B9u)
                     + noise01(k ^ 0x85EBCA6Bu)
                     + noise01(k ^ 0xC2B2AE35u);
    return acc * 0.8660254037844386;
}

/* Decorrelation time constants of the wander.  Real RTK positions drift
   slowly (multipath-dominated); per-frame teleporting would trip ArduPilot's
   GPS glitch detection and the EKF failsafe instead of looking like noise. */
#define NOISE_TAU_POS_S  3.0
#define NOISE_TAU_HDG_S  2.0

/* The wander is clamped to +-3.5 sigma so a long run can never accumulate a
   biased excursion beyond what the receiver would plausibly report. */
#define NOISE_CLAMP_SIGMA 3.5

/* Advance the correlated noise state to the given wall clock.  The firmware
   calls this once per burst before snapshotting; sigma = 0 keeps the state
   pinned at zero, so "jitter off" stays byte-identical. */
void sim_noise_advance(sim_state_t *s, uint32_t now_ms)
{
    if (now_ms <= s->noise_last_ms) {
        return;                 /* no time has passed: keep the last sample */
    }
    double dt = (double)(now_ms - s->noise_last_ms) / 1000.0;
    s->noise_last_ms = now_ms;
    /* Guard against a long stall producing one huge jump, as sim_step does. */
    if (dt > 1.0) {
        dt = 1.0;
    }
    s->noise_seq++;

    /* First-order Gauss-Markov: x' = phi*x + sigma*sqrt(1-phi^2)*w with unit
       white w, so the steady-state std is exactly sigma while the per-frame
       step stays at sigma*sqrt(2*dt/tau) - mm to cm at RTK settings. */
    const double phi_p = exp(-dt / NOISE_TAU_POS_S);
    const double g_p = s->position_noise_m * sqrt(1.0 - phi_p * phi_p);
    s->noise_n_m = phi_p * s->noise_n_m + g_p * noise_gauss(s->noise_seq);
    s->noise_e_m = phi_p * s->noise_e_m + g_p * noise_gauss(s->noise_seq + NOISE_SALT_LON);
    const double lim_p = NOISE_CLAMP_SIGMA * s->position_noise_m;
    if (s->noise_n_m > lim_p)  s->noise_n_m = lim_p;
    if (s->noise_n_m < -lim_p) s->noise_n_m = -lim_p;
    if (s->noise_e_m > lim_p)  s->noise_e_m = lim_p;
    if (s->noise_e_m < -lim_p) s->noise_e_m = -lim_p;

    const double phi_h = exp(-dt / NOISE_TAU_HDG_S);
    const double g_h = s->heading_noise_deg * sqrt(1.0 - phi_h * phi_h);
    s->noise_hdg_deg += g_h * noise_gauss(s->noise_seq + NOISE_SALT_HEADING);
    const double lim_h = NOISE_CLAMP_SIGMA * s->heading_noise_deg;
    if (s->noise_hdg_deg > lim_h)  s->noise_hdg_deg = lim_h;
    if (s->noise_hdg_deg < -lim_h) s->noise_hdg_deg = -lim_h;
}

void sim_init(sim_state_t *s, uint32_t now_ms)
{
    memset(s, 0, sizeof(*s));

    /* Default origin coordinates (can be overridden from web UI or REST API). */
    s->origin_lat = 31.00000000;
    s->origin_lon = 121.00000000;
    s->origin_alt = 4.0;

    s->north_m = 0.0;
    s->east_m = 0.0;
    s->heading_deg = 0.0;
    s->speed_mps = 0.0;
    s->altitude_m = s->origin_alt;
    s->mode = SIM_MODE_STATIC;

    s->circle_radius_m = 10.0;
    s->circle_rate_dps = 6.0;
    s->course_deg = 0.0;
    s->leg_m = 40.0;
    s->line_progress_m = 0.0;

    s->waypoint_count = 0;
    s->waypoint_index = 0;
    s->waypoint_radius_m = 2.0;
    s->waypoint_speed_mps = 1.5;

    s->fix_quality = FIX_QUALITY_STD;
    s->satellites = 18;
    s->hdop = 0.9;
    s->rtk_engine_ready = false;
    s->heading_noise_deg = 0.0;
    s->position_noise_m = 0.0;
    s->output_rate_ms = 200;
    s->baseline_len_m = 1.2;
    s->heading_enabled = true;
    /* Standard configuration emits GGA/RMC/HDT only. */
    s->vtg_enabled = false;

    s->start_ms = now_ms;
    s->last_step_ms = now_ms;
    s->steps = 0;
    s->noise_seq = 0;
    s->noise_n_m = 0.0;
    s->noise_e_m = 0.0;
    s->noise_hdg_deg = 0.0;
    s->noise_last_ms = now_ms;
}

void sim_step(sim_state_t *s, uint32_t now_ms)
{
    if (now_ms == s->last_step_ms && s->steps > 0) {
        return;
    }
    double dt = (s->steps == 0) ? 0.0 : (double)(now_ms - s->last_step_ms) / 1000.0;
    /* Guard against a long stall (e.g. serial monitor) producing a huge jump. */
    if (dt < 0.0 || dt > 1.0) {
        dt = (dt < 0.0) ? 0.0 : 1.0;
    }
    s->last_step_ms = now_ms;
    s->steps++;

    switch (s->mode) {
    case SIM_MODE_STATIC:
        s->speed_mps = 0.0;
        break;

    case SIM_MODE_CIRCLE: {
        const double r = (s->circle_radius_m > 0.5) ? s->circle_radius_m : 0.5;
        /* advance the arc angle; keep the vessel on the circle */
        const double dtheta = deg2rad(s->circle_rate_dps) * dt;
        double ang = atan2(s->north_m, s->east_m) + dtheta;
        s->east_m = r * cos(ang);
        s->north_m = r * sin(ang);
        /* heading is tangent to the circle, counter-clockwise */
        s->heading_deg = wrap360(rad2deg(ang) + 90.0);
        s->speed_mps = r * deg2rad(s->circle_rate_dps);
        break;
    }

    case SIM_MODE_LINE: {
        s->speed_mps = (s->waypoint_speed_mps > 0.0) ? s->waypoint_speed_mps : 0.0;
        s->line_progress_m += s->speed_mps * dt;
        if (s->leg_m > 0.0) {
            /* Wrap with fmod, not by zeroing: dropping the overshoot would make
               the simulated vessel fall behind by one step at every lap and
               skew any speed measurement taken from the position track. */
            while (s->line_progress_m >= s->leg_m) {
                s->line_progress_m -= s->leg_m;
            }
        }
        const double c = deg2rad(s->course_deg);
        s->east_m = s->line_progress_m * sin(c);
        s->north_m = s->line_progress_m * cos(c);
        s->heading_deg = wrap360(s->course_deg);
        break;
    }

    case SIM_MODE_WAYPOINT: {
        if (s->waypoint_count == 0) {
            s->speed_mps = 0.0;
            break;
        }
        const double tgt_n = (double)s->waypoints[s->waypoint_index][0];
        const double tgt_e = (double)s->waypoints[s->waypoint_index][1];
        double dn = tgt_n - s->north_m;
        double de = tgt_e - s->east_m;
        const double dist = sqrt(dn * dn + de * de);
        const double speed = (s->waypoint_speed_mps > 0.0) ? s->waypoint_speed_mps : 1.0;
        s->speed_mps = speed;

        if (dist <= s->waypoint_radius_m || dist < 1e-6) {
            s->waypoint_index = (uint8_t)((s->waypoint_index + 1) % s->waypoint_count);
            dn = (double)s->waypoints[s->waypoint_index][0] - s->north_m;
            de = (double)s->waypoints[s->waypoint_index][1] - s->east_m;
        }
        s->heading_deg = wrap360(rad2deg(atan2(de, dn)));

        const double step = speed * dt;
        const double d = sqrt(dn * dn + de * de);
        if (d > 1e-6) {
            const double k = (step < d) ? step : d;
            s->north_m += dn / d * k;
            s->east_m += de / d * k;
        }
        break;
    }
    }

    /* Keep the simulated vessel within a sane radius of the origin so a
       forgotten long-running circle does not produce absurd coordinates. */
    const double lim = 5000.0;
    if (s->north_m > lim) s->north_m = lim;
    if (s->north_m < -lim) s->north_m = -lim;
    if (s->east_m > lim) s->east_m = lim;
    if (s->east_m < -lim) s->east_m = -lim;

    s->altitude_m = s->origin_alt;
}

void sim_position(const sim_state_t *s, double *lat, double *lon)
{
    const double lat_rad = deg2rad(s->origin_lat);
    const double m_per_deg_lon = M_PER_DEG_LON_EQ * cos(lat_rad);
    *lat = s->origin_lat + s->north_m / M_PER_DEG_LAT;
    *lon = s->origin_lon + (m_per_deg_lon > 1.0 ? s->east_m / m_per_deg_lon : 0.0);
}

void sim_position_noisy(const sim_state_t *s, double *lat, double *lon)
{
    /* The correlated wander (advanced by sim_noise_advance) is part of the
       reported antenna position: convert ENU truth plus the noise state. */
    const double lat_rad = deg2rad(s->origin_lat);
    const double m_per_deg_lon = M_PER_DEG_LON_EQ * cos(lat_rad);
    *lat = s->origin_lat + (s->north_m + s->noise_n_m) / M_PER_DEG_LAT;
    *lon = s->origin_lon + (m_per_deg_lon > 1.0 ? (s->east_m + s->noise_e_m) / m_per_deg_lon : 0.0);
}

double sim_heading_noisy(const sim_state_t *s)
{
    return wrap360(s->heading_deg + s->noise_hdg_deg);
}

double sim_yaw_accuracy_deg(const sim_state_t *s)
{
    /* Typical dual-antenna figures: 0.2 deg RMS at a 1 m baseline, 20 cm minimum. */
    double base = (s->baseline_len_m > 0.2) ? s->baseline_len_m : 0.2;
    return 0.2 * (1.0 / base) + s->heading_noise_deg;
}

double sim_h_accuracy_m(const sim_state_t *s)
{
    switch (s->fix_quality) {
    case FIX_QUALITY_RTK_FIXED: return 0.01;
    case FIX_QUALITY_RTK_FLOAT: return 0.20;
    case FIX_QUALITY_DGPS:      return 0.50;
    case FIX_QUALITY_STD:       return 1.50;
    case FIX_QUALITY_NONE:
    default:                    return 0.0;
    }
}

const char *sim_talker(const sim_state_t *s)
{
    /* A real multi-constellation receiver switches to GN once it is using more
       than GPS; mirror that so the flight controller sees realistic sentences. */
    return (s->fix_quality == FIX_QUALITY_RTK_FIXED ||
            s->fix_quality == FIX_QUALITY_RTK_FLOAT) ? "GN" : "GP";
}

void sim_on_rtcm_applied(sim_state_t *s, uint32_t now_ms)
{
    (void)now_ms;
    s->rtk_engine_ready = true;
    if (s->fix_quality < FIX_QUALITY_RTK_FLOAT) {
        s->fix_quality = FIX_QUALITY_RTK_FLOAT;
    }
    if (s->satellites < 20) {
        s->satellites = 24;
    }
    if (s->hdop > 0.6) {
        s->hdop = 0.6;
    }
}

void sim_on_rtcm_timeout(sim_state_t *s)
{
    s->rtk_engine_ready = false;
    if (s->fix_quality == FIX_QUALITY_RTK_FIXED ||
        s->fix_quality == FIX_QUALITY_RTK_FLOAT) {
        s->fix_quality = FIX_QUALITY_STD;
        s->hdop = 0.9;
    }
}
