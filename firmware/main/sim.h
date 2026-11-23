/*
 * GPS/RTK module state machine: simulated motion, fix quality and the shared
 * configuration that both the UART output and the web UI read.
 *
 * Pure C so host tests can drive it without FreeRTOS.  Caller supplies the wall
 * clock in milliseconds.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* NMEA fix quality reported in GGA field 6. */
typedef enum {
    FIX_QUALITY_NONE  = 0,   /* no fix */
    FIX_QUALITY_STD   = 1,   /* standalone, ~1.5 m */
    FIX_QUALITY_DGPS  = 2,   /* differential, ~0.5 m */
    FIX_QUALITY_RTK_FLOAT = 5,
    FIX_QUALITY_RTK_FIXED = 4,
} sim_fix_quality_t;

/* How the simulated vessel moves. */
typedef enum {
    SIM_MODE_STATIC = 0,  /* hold the current position */
    SIM_MODE_CIRCLE,      /* constant-radius circle, for path-follower benches */
    SIM_MODE_LINE,        /* straight line at `course_deg`, wraps at `leg_m` */
    SIM_MODE_WAYPOINT,    /* walk a list of local ENU offsets in order */
} sim_mode_t;

#define SIM_MAX_WAYPOINTS 32

typedef struct {
    /* -------- geodetic origin (the "harbour") -------- */
    double origin_lat;
    double origin_lon;
    double origin_alt;      /* ellipsoid-ish altitude in metres */

    /* -------- current simulated state -------- */
    double north_m;         /* local ENU offset from origin */
    double east_m;
    double heading_deg;     /* true heading, 0..360 */
    double speed_mps;
    double altitude_m;
    sim_mode_t mode;

    /* -------- simulated motion parameters -------- */
    double circle_radius_m;
    double circle_rate_dps;   /* how fast it drives round the circle */
    double course_deg;        /* used by SIM_MODE_LINE */
    double leg_m;             /* wrap length for SIM_MODE_LINE */
    double line_progress_m;

    int32_t waypoints[SIM_MAX_WAYPOINTS][2];  /* [north, east] in metres */
    uint8_t waypoint_count;
    uint8_t waypoint_index;
    double waypoint_radius_m;
    double waypoint_speed_mps;

    /* -------- receiver configuration -------- */
    sim_fix_quality_t fix_quality;
    uint8_t satellites;
    double hdop;
    bool    rtk_engine_ready;   /* corrections are considered applied */
    double  heading_noise_deg;  /* heading wander sigma, degrees (0 = off) */
    double  position_noise_m;   /* horizontal wander sigma per axis, metres (0 = off) */
    uint16_t output_rate_ms;    /* NMEA period, default 200 -> 5 Hz */
    double  baseline_len_m;     /* dual-antenna baseline, feeds yaw accuracy */
    bool    heading_enabled;    /* emit $G?HDT (required by EK3_SRC1_YAW=2) */
    bool    vtg_enabled;        /* emit $G?VTG; off by default, see below */

    /* -------- housekeeping -------- */
    uint32_t start_ms;
    uint32_t last_step_ms;
    uint32_t steps;             /* number of sim_step() calls */
    uint32_t noise_seq;         /* per-burst counter seeding the noise samples */

    /* Correlated noise state (first-order Gauss-Markov), advanced once per
       burst by sim_noise_advance().  Steady-state std = the sigma below. */
    double  noise_n_m;          /* north wander, metres */
    double  noise_e_m;          /* east wander, metres */
    double  noise_hdg_deg;      /* heading wander, degrees */
    uint32_t noise_last_ms;     /* wall clock of the last noise advance */
} sim_state_t;

void sim_init(sim_state_t *s, uint32_t now_ms);

/* Advance the simulated motion.  Idempotent within one millisecond. */
void sim_step(sim_state_t *s, uint32_t now_ms);

/* Absolute geodetic position of the simulated antenna (no noise). */
void sim_position(const sim_state_t *s, double *lat, double *lon);

/* Advance the correlated noise state to the given wall clock (milliseconds).
   Call once per emitted burst, before snapshotting the state for output.
   Steady-state std equals the configured sigmas; per-frame movement stays at
   sigma*sqrt(2*dt/tau), i.e. millimetres at RTK settings. */
void sim_noise_advance(sim_state_t *s, uint32_t now_ms);

/* Position including the small injected wander, as the receiver would report. */
void sim_position_noisy(const sim_state_t *s, double *lat, double *lon);

/* Heading including the small injected wander, wrapped to [0, 360).
   One sample per burst: HDT, RMC COG and VTG all carry the same value so the
   flight controller never sees them disagree. */
double sim_heading_noisy(const sim_state_t *s);

/* RTK-typical yaw accuracy for the configured baseline, in degrees (1 sigma). */
double sim_yaw_accuracy_deg(const sim_state_t *s);

/* Horizontal accuracy implied by the current fix quality, in metres. */
double sim_h_accuracy_m(const sim_state_t *s);

/* The NMEA talker prefix to use: GN when RTK, GP otherwise. */
const char *sim_talker(const sim_state_t *s);

/* Record that RTCM corrections were applied; promotes the fix quality. */
void sim_on_rtcm_applied(sim_state_t *s, uint32_t now_ms);

/* Drop back to standalone after corrections stop arriving. */
void sim_on_rtcm_timeout(sim_state_t *s);
