/*
 * Shared declarations for the GPS simulator's web layer.
 *
 * State is owned by main.c; the web layer only reads/writes it through these
 * accessors so that the two translation units never disagree about layout.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "rtcm.h"
#include "sim.h"

/* Network link bookkeeping, published to the UI. */
typedef struct {
    uint32_t baud;
    uint8_t  mon_enabled;
    uint8_t  wifi_connected;
    char     sta_ssid[33];
    char     ap_ssid[33];
    char     ip[16];
} link_state_t;

/* Mutex around the shared sim state (implemented in main.c). */
void gps_sim_state_lock(void);
void gps_sim_state_unlock(void);

sim_state_t   *gps_sim_state(void);
rtcm_parser_t *gps_sim_rtcm(void);
link_state_t  *gps_sim_link(void);

uint32_t gps_sim_last_rtcm_ms(void);
uint32_t gps_sim_rtcm_bytes(void);

/* Apply a new GPS UART baud rate (implemented in main.c). */
void gps_sim_set_baud(uint32_t baud);

/* Start the HTTP server on port 80. */
void web_start(uint32_t stack_size);