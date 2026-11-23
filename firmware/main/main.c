/*
 * USV GPS/RTK module simulator — ESP32-C3.
 *
 * Emulates a dual-antenna RTK receiver as seen from an ArduPilot flight controller's GPS port:
 *
 *   NMEA out : $GNGGA / $GNRMC / $GPVTG / $GNHDT at the configured rate
 *   RTCM in  : CORS corrections forwarded by the flight controller
 *   Status   : three GPIOs mirroring RTK_STAT / PVT_STAT / ERR_STAT
 *
 * Wiring (ESP32-C3 SuperMini / devkit, 3V3 logic):
 *   GPIO21 (U0TXD) -> FC GPS RX      (NMEA to the flight controller)
 *   GPIO20 (U0RXD) <- FC GPS TX      (RTCM from the flight controller)
 *   GPIO8  -> RTK_STAT   (high on RTK Fixed)
 *   GPIO9  -> PVT_STAT   (high with any valid fix)
 *   GPIO10 -> ERR_STAT   (high when the receiver would report an error)
 *
 * UART0 carries the GPS link; UART1 is an optional mirror for a USB-TTL probe so
 * the bench operator can watch the same sentences.  The console lives on the
 * USB Serial/JTAG peripheral (see sdkconfig.defaults) so neither is stolen.
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#include "nmea.h"
#include "rtcm.h"
#include "sim.h"
#include "web.h"

static const char *TAG = "gps_sim";

/* ------------------------------------------------------------------ config */

#define GPS_UART            UART_NUM_0
#define GPS_TX_GPIO         21
#define GPS_RX_GPIO         20
#define MON_UART            UART_NUM_1
#define MON_TX_GPIO         (-1)   /* transmit-only mirror; no RX pin */

/* Status outputs for RTK / PVT / Error indicators.
   Deliberately NOT on GPIO8/9/10:
     GPIO8  is the blue LED on most C3 boards (and a strapping pin)
     GPIO9  is the BOOT strapping pin
     GPIO10 is free but sits next to them, so the whole block moved for
            a clean, contiguous wiring harness. */
#define PIN_RTK_STAT        GPIO_NUM_4
#define PIN_PVT_STAT        GPIO_NUM_5
#define PIN_ERR_STAT        GPIO_NUM_6

/* Blue LED: pulses once per NMEA burst so you can see the data leaving. */
#define LED_GPIO            CONFIG_USV_GPS_LED_GPIO
#define LED_PULSE_MS        CONFIG_USV_GPS_LED_PULSE_MS
#if CONFIG_USV_GPS_LED_ACTIVE_LOW
#define LED_ON              0
#define LED_OFF             1
#else
#define LED_ON              1
#define LED_OFF             0
#endif

#define GPS_BAUD_DEFAULT    115200
#define UART_RX_BUF         4096
#define RTCM_TASK_STACK     4096
#define NMEA_TASK_STACK     4096
#define WEB_TASK_STACK      8192

/* How long without a usable correction before the fix degrades. */
#define RTCM_TIMEOUT_MS     10000

/* ------------------------------------------------------------------- state */


static sim_state_t      g_sim;
static rtcm_parser_t    g_rtcm;
static link_state_t     g_link;
static SemaphoreHandle_t g_sim_lock;

/* Last time an RTCM frame with a good CRC arrived. */
static volatile uint32_t g_last_rtcm_ms;
/* Number of RTCM bytes applied since boot (diagnostics). */
static volatile uint32_t g_rtcm_bytes;

/* ----------------------------------------------------------- small helpers */

static inline uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

/* LED helpers live further down, next to the status outputs they accompany. */
static void led_init(void);
static void led_pulse(void);

void gps_sim_state_lock(void)   { xSemaphoreTake(g_sim_lock, portMAX_DELAY); }
void gps_sim_state_unlock(void) { xSemaphoreGive(g_sim_lock); }

sim_state_t *gps_sim_state(void) { return &g_sim; }
rtcm_parser_t *gps_sim_rtcm(void) { return &g_rtcm; }
link_state_t *gps_sim_link(void) { return &g_link; }
uint32_t gps_sim_last_rtcm_ms(void) { return g_last_rtcm_ms; }
uint32_t gps_sim_rtcm_bytes(void) { return g_rtcm_bytes; }

/* --------------------------------------------------------------- NMEA output */

/* Emit one sentence on both the GPS UART and the optional monitor UART. */
static void emit(const char *line)
{
    const size_t n = strlen(line);
    if (n == 0) {
        return;
    }
    uart_write_bytes(GPS_UART, line, n);
    if (g_link.mon_enabled) {
        uart_write_bytes(MON_UART, line, n);
    }
}

/* HHMMSS.ss from UTC.  The simulator keeps UTC, so the flight controller sees a
   plausible GPS date/time and can set its clock from it. */
static void utc_stamp(char *buf, size_t len)
{
    time_t t = time(NULL);
    struct tm tm_utc;
    gmtime_r(&t, &tm_utc);
    snprintf(buf, len, "%02d%02d%02d.%02d",
             tm_utc.tm_hour, tm_utc.tm_min, tm_utc.tm_sec, 0);
}

static void utc_date(char *buf, size_t len)
{
    time_t t = time(NULL);
    struct tm tm_utc;
    gmtime_r(&t, &tm_utc);
    snprintf(buf, len, "%02d%02d%02d",
             tm_utc.tm_mday, tm_utc.tm_mon + 1, (tm_utc.tm_year + 1900) % 100);
}

static void send_nmea_burst(void)
{
    char body[256];
    char line[300];
    char latv[24], lonv[24], lathemi, lonhemi;
    char tim[16], dat[16];
    double lat, lon;

    gps_sim_state_lock();
    /* Evolve the correlated wander once per burst: the position/heading noise
       state moves smoothly towards its next sample instead of teleporting. */
    sim_noise_advance(&g_sim, now_ms());
    sim_state_t snapshot = g_sim;
    gps_sim_state_unlock();

    sim_position_noisy(&snapshot, &lat, &lon);
    const char *tk = sim_talker(&snapshot);
    const int q = (int)snapshot.fix_quality;
    const int sats = (q == FIX_QUALITY_NONE) ? 0 : snapshot.satellites;
    const double hdop = (q == FIX_QUALITY_NONE) ? 99.9 : snapshot.hdop;
    const double speed_mps = (q == FIX_QUALITY_NONE) ? 0.0 : snapshot.speed_mps;
    /* One jittered heading per burst feeds HDT, RMC COG and VTG alike, so the
       yaw the EKF stores from HDT always matches the course it commits from
       RMC. */
    const double course = sim_heading_noisy(&snapshot);
    const double alt = snapshot.altitude_m;

    utc_stamp(tim, sizeof(tim));
    utc_date(dat, sizeof(dat));

    /* Yaw first.
     *
     * AP_GPS_NMEA publishes a fix every time a sentence completes and the
     * freshness rule passes, so a four-sentence burst is published four times
     * unless the sentences are ordered around the commit point.  HDT carries no
     * timestamp of its own beyond the yaw timeout, so emitting it before GGA
     * and RMC means the single publish that GGA/RMC generate already carries
     * the new heading.  Sending HDT last instead produces a second publish -
     * and, worse, a publish whose yaw is one cycle stale. */
    if (snapshot.heading_enabled && snapshot.fix_quality != FIX_QUALITY_NONE) {
        snprintf(body, sizeof(body), "%sHDT,%.2f,T", sim_talker(&snapshot), course);
        if (nmea_sentence(line, sizeof(line), body) > 0) {
            emit(line);
        }
    }

    /* ---- GGA: position, fix quality, satellites, HDOP, altitude ----
       ArduPilot reads the quality indicator into AP_GPS_FixType:
       4 = RTK Fixed, 5 = RTK Float, 1 = 3D fix.  The field must not be blank
       for a fix to be usable. */
    nmea_fmt_lat(lat, latv, sizeof(latv), &lathemi);
    nmea_fmt_lon(lon, lonv, sizeof(lonv), &lonhemi);
    snprintf(body, sizeof(body),
             "%sGGA,%s,%s,%c,%s,%c,%d,%02d,%.1f,%.1f,M,%.1f,M,1.0,0000",
             tk, tim, latv, lathemi, lonv, lonhemi, q, sats, hdop, alt, 0.0);
    if (nmea_sentence(line, sizeof(line), body) > 0) {
        emit(line);
    }

    /* ---- RMC: date/time, position, speed over ground, course ----
       RMC is the last sentence of the burst: it is the one that publishes the
       fix, so it must come after everything it should carry. */
    const double sog_knots = speed_mps * 1.9438444924406;
    snprintf(body, sizeof(body),
             "%sRMC,%s,%c,%s,%c,%s,%c,%.2f,%.2f,%s,,,A",
             tk, tim,
             (q == FIX_QUALITY_NONE) ? 'V' : 'A',
             latv, lathemi, lonv, lonhemi,
             sog_knots, course, dat);
    if (nmea_sentence(line, sizeof(line), body) > 0) {
        emit(line);
    }

    /* ---- VTG: course and speed over ground (optional) ----
       Deliberately off by default.  AP_GPS_NMEA marks VTG as consumed by
       setting its timestamp to 1, and its freshness test then rejects the RMC
       publish while `now - 1 > 150` - that is, for ever.  Emitting VTG moves
       the commit point from RMC to VTG, so if VTG ever stops the fix stops
       with it.  RMC already carries speed and course, and standard configurations send
       no VTG at all. */
    if (snapshot.vtg_enabled) {
        snprintf(body, sizeof(body),
                 "%sVTG,%.2f,T,,M,%.2f,N,%.2f,K,A",
                 tk, course, sog_knots, speed_mps * 3.6);
        if (nmea_sentence(line, sizeof(line), body) > 0) {
            emit(line);
        }
    }
}
static void send_rtcm_status(void)
{
    char body[256];
    char line[300];
    const uint32_t since = now_ms() - g_last_rtcm_ms;
    const int age = (g_last_rtcm_ms == 0) ? -1 : (int)(since > 99999 ? 99999 : since);
    const uint16_t type = g_rtcm.last_msg_type;

    /* Custom receiver status sentence.  Not part of NMEA 0183 proper, so
       ArduPilot's NMEA driver ignores it ('P' is not an A-Z talker pair), but a
       terminal on the monitor port can read the RTCM state directly. */
    snprintf(body, sizeof(body),
             "PUSV,RTCM,%u,%u,%u,%u,%d,%u",
             (unsigned)g_rtcm.frame_count, (unsigned)g_rtcm.crc_errors,
             (unsigned)g_rtcm_bytes, (unsigned)type, age, (unsigned)g_sim.fix_quality);
    if (nmea_sentence(line, sizeof(line), body) > 0) {
        emit(line);
    }
}

/* ------------------------------------------------------------- tasks: NMEA */

static void nmea_task(void *arg)
{
    (void)arg;
    TickType_t last = xTaskGetTickCount();
    uint32_t status_div = 0;

    for (;;) {
        gps_sim_state_lock();
        const uint16_t period = g_sim.output_rate_ms ? g_sim.output_rate_ms : 200;
        gps_sim_state_unlock();

        send_nmea_burst();

        /* One blink per burst: the LED tracks the actual output rate. */
        led_pulse();

        /* receiver status sentence at ~1 Hz regardless of the fix rate */
        if (++status_div >= (uint32_t)(1000 / (period ? period : 200))) {
            status_div = 0;
            send_rtcm_status();
        }

        /* Clear the pulse even at very long output periods, so the LED cannot
           stay lit between bursts. */
        vTaskDelayUntil(&last, pdMS_TO_TICKS(period));
    }
}

/* ------------------------------------------------------------- tasks: RTCM */

static void on_rtcm_frame(rtcm_parser_t *p, const rtcm_msg_info_t *info, void *ctx)
{
    (void)ctx;
    if (!info->crc_ok) {
        return;
    }
    /* Any CRC-valid frame means the correction link is alive. */
    g_last_rtcm_ms = now_ms();
    gps_sim_state_lock();
    sim_on_rtcm_applied(&g_sim, g_last_rtcm_ms);
    gps_sim_state_unlock();
}

static void rtcm_task(void *arg)
{
    (void)arg;
    uint8_t buf[512];
    uint32_t last_check = 0;

    for (;;) {
        const int n = uart_read_bytes(GPS_UART, buf, sizeof(buf), pdMS_TO_TICKS(100));
        const uint32_t t = now_ms();
        if (n > 0) {
            g_rtcm_bytes += (uint32_t)n;
            rtcm_push(&g_rtcm, buf, (size_t)n, t, on_rtcm_frame, NULL);
        }

        /* Degrade the fix when corrections stop, like a real receiver does. */
        if (t - last_check > 1000) {
            last_check = t;
            if (g_last_rtcm_ms != 0 && (t - g_last_rtcm_ms) > RTCM_TIMEOUT_MS) {
                gps_sim_state_lock();
                sim_on_rtcm_timeout(&g_sim);
                gps_sim_state_unlock();
            }
        }
    }
}

/* ------------------------------------------------------------ blue LED */

/*
 * The LED is pulsed from the NMEA task once per emitted burst, so the blink
 * rate *is* the output rate: a glance at the board shows whether sentences are
 * still going out and how fast.
 *
 * Turning the LED off is done with a one-shot esp_timer rather than inline code.
 * The NMEA task sleeps for the whole output period, so any "check whether the
 * pulse has expired" logic placed after that sleep runs only just before the
 * next pulse - the LED would light at the start of one burst and be switched off
 * at the start of the next, appearing solidly on instead of flashing. The timer
 * fires mid-period and gives a genuinely separate flash.
 */
static esp_timer_handle_t g_led_timer;

static void led_off_cb(void *arg)
{
    (void)arg;
#if CONFIG_USV_GPS_LED_GPIO >= 0
    gpio_set_level(CONFIG_USV_GPS_LED_GPIO, LED_OFF);
#endif
}

static void led_init(void)
{
#if CONFIG_USV_GPS_LED_GPIO >= 0
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << CONFIG_USV_GPS_LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&cfg));
    gpio_set_level(CONFIG_USV_GPS_LED_GPIO, LED_OFF);

    const esp_timer_create_args_t args = {
        .callback = &led_off_cb,
        .name = "led_off",
    };
    ESP_ERROR_CHECK(esp_timer_create(&args, &g_led_timer));

    /* Three quick flashes at startup, then idle.  This makes the wiring and the
       polarity obvious at a glance: a lit-then-dark triple flash means the
       board matches CONFIG_USV_GPS_LED_ACTIVE_LOW; a pin that never lights (or
       one that stays lit until the first burst) means the polarity option needs
       flipping. */
    for (int i = 0; i < 3; ++i) {
        gpio_set_level(CONFIG_USV_GPS_LED_GPIO, LED_ON);
        vTaskDelay(pdMS_TO_TICKS(120));
        gpio_set_level(CONFIG_USV_GPS_LED_GPIO, LED_OFF);
        vTaskDelay(pdMS_TO_TICKS(120));
    }
#endif
}

/* Called once per burst, right after the sentences are written. */
static void led_pulse(void)
{
#if CONFIG_USV_GPS_LED_GPIO >= 0
    gpio_set_level(CONFIG_USV_GPS_LED_GPIO, LED_ON);
    if (g_led_timer) {
        /* Restart so a short output period cannot leave the LED latched on. */
        esp_timer_stop(g_led_timer);
        esp_timer_start_once(g_led_timer, (uint64_t)LED_PULSE_MS * 1000ULL);
    }
#endif
}
/* --------------------------------------------------------- status GPIOs */

static void status_gpio_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << PIN_RTK_STAT) | (1ULL << PIN_PVT_STAT) | (1ULL << PIN_ERR_STAT),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&cfg));
    gpio_set_level(PIN_RTK_STAT, 0);
    gpio_set_level(PIN_PVT_STAT, 0);
    gpio_set_level(PIN_ERR_STAT, 0);
}

static void status_gpio_task(void *arg)
{
    (void)arg;
    for (;;) {
        gps_sim_state_lock();
        const sim_fix_quality_t q = g_sim.fix_quality;
        const bool rtk_ready = g_sim.rtk_engine_ready;
        gps_sim_state_unlock();

        gpio_set_level(PIN_RTK_STAT, (q == FIX_QUALITY_RTK_FIXED && rtk_ready) ? 1 : 0);
        gpio_set_level(PIN_PVT_STAT, (q != FIX_QUALITY_NONE) ? 1 : 0);
        gpio_set_level(PIN_ERR_STAT, 0);

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

/* ------------------------------------------------------------------- WiFi */

/* A target AP that is absent makes the STA rescan every channel on each
   retry; that off-channel time is what breaks softAP client association and
   DHCP on the phone.  So the join is retried a bounded number of times and
   then abandoned, leaving the radio to the softAP alone until reboot. */
#define STA_JOIN_MAX_ATTEMPTS 8

static uint8_t s_sta_attempts;

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    link_state_t *lk = gps_sim_link();

    if (base == WIFI_EVENT) {
        if (id == WIFI_EVENT_STA_START) {
            s_sta_attempts = 0;
            esp_wifi_connect();
        } else if (id == WIFI_EVENT_STA_DISCONNECTED) {
            const wifi_event_sta_disconnected_t *d = (wifi_event_sta_disconnected_t *)data;
            lk->wifi_connected = 0;
            lk->ip[0] = '\0';
            if (s_sta_attempts < STA_JOIN_MAX_ATTEMPTS) {
                s_sta_attempts++;
                ESP_LOGW(TAG, "STA '%s' join attempt %u/%u failed (reason %d), retrying",
                         CONFIG_USV_GPS_WIFI_SSID, (unsigned)s_sta_attempts,
                         (unsigned)STA_JOIN_MAX_ATTEMPTS, d ? d->reason : -1);
                esp_wifi_connect();
            } else if (s_sta_attempts == STA_JOIN_MAX_ATTEMPTS) {
                s_sta_attempts++;
                ESP_LOGW(TAG, "STA join abandoned; softAP-only mode until reboot");
            }
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)data;
        snprintf(lk->ip, sizeof(lk->ip), IPSTR, IP2STR(&ev->ip_info.ip));
        lk->wifi_connected = 1;
        s_sta_attempts = 0;
        ESP_LOGI(TAG, "WiFi connected, IP %s", lk->ip);
    } else if (base == IP_EVENT && id == IP_EVENT_ASSIGNED_IP_TO_CLIENT) {
        /* Visibly confirming DHCP served a softAP client makes WiFi side
           problems diagnosable from the serial console alone. */
        ip_event_assigned_ip_to_client_t *ev = (ip_event_assigned_ip_to_client_t *)data;
        ESP_LOGI(TAG, "softAP client got IP " IPSTR, IP2STR(&ev->ip));
    }
}

static void wifi_start(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_sta();
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_ASSIGNED_IP_TO_CLIENT, &wifi_event_handler, NULL, NULL));

    /* Credentials come from Kconfig so a bench unit can be provisioned by
       editing sdkconfig.defaults and rebuilding, with no code change. */
    wifi_config_t sta = {0};
    const char *ssid = CONFIG_USV_GPS_WIFI_SSID;
    const char *pwd = CONFIG_USV_GPS_WIFI_PASSWORD;
    snprintf((char *)sta.sta.ssid, sizeof(sta.sta.ssid), "%s", ssid);
    snprintf((char *)sta.sta.password, sizeof(sta.sta.password), "%s", pwd);
    sta.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    wifi_config_t ap = {0};
    snprintf((char *)ap.ap.ssid, sizeof(ap.ap.ssid), "%s", CONFIG_USV_GPS_AP_SSID);
    ap.ap.ssid_len = (uint8_t)strlen((char *)ap.ap.ssid);
    snprintf((char *)ap.ap.password, sizeof(ap.ap.password), "%s", CONFIG_USV_GPS_AP_PASSWORD);
    ap.ap.max_connection = 4;
    ap.ap.authmode = WIFI_AUTH_WPA2_PSK;
    ap.ap.channel = 6;      /* country code cc=01 restricts 2G channels to 1-11 */

    snprintf(g_link.sta_ssid, sizeof(g_link.sta_ssid), "%s", ssid);
    snprintf(g_link.ap_ssid, sizeof(g_link.ap_ssid), "%s", CONFIG_USV_GPS_AP_SSID);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap));
    ESP_ERROR_CHECK(esp_wifi_start());

    /* NOTE: do NOT call esp_wifi_set_country() here.  An explicit CN policy,
       although accepted by the driver, muted the softAP's beacon TX entirely
       on this bench - the radio received fine but no scan ever heard the
       SSID.  The world-safe default ("01", channels 1-11) beacons correctly;
       just keep the AP on a channel inside it (this build uses ch6). */

    ESP_LOGI(TAG, "softAP '%s' up, joining '%s'", CONFIG_USV_GPS_AP_SSID, ssid);
}

/* ------------------------------------------------------------------- UART */

static void uart_setup(void)
{
    /* GPS link: bidirectional, NMEA out and RTCM in on the same pins. */
    uart_config_t cfg = {
        .baud_rate = GPS_BAUD_DEFAULT,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(GPS_UART, UART_RX_BUF, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(GPS_UART, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(GPS_UART, GPS_TX_GPIO, GPS_RX_GPIO,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    /* Monitor mirror: transmit only, no driver needed beyond a pin mux. */
    ESP_ERROR_CHECK(uart_driver_install(MON_UART, 256, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(MON_UART, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(MON_UART, CONFIG_USV_GPS_MON_TX_GPIO, UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    g_link.baud = GPS_BAUD_DEFAULT;
    g_link.mon_enabled = 1;
}

void gps_sim_set_baud(uint32_t baud)
{
    if (baud != 9600 && baud != 19200 && baud != 38400 && baud != 57600 &&
        baud != 115200 && baud != 230400 && baud != 460800 && baud != 921600) {
        return;
    }
    uart_wait_tx_done(GPS_UART, pdMS_TO_TICKS(200));
    uart_set_baudrate(GPS_UART, baud);
    uart_set_baudrate(MON_UART, baud);
    g_link.baud = baud;
    ESP_LOGI(TAG, "GPS UART baud -> %u", (unsigned)baud);
}

/* ------------------------------------------------------------------- main */

/* ------------------------------------------------------------ self-test */

/*
 * Loopback self-check.
 *
 * Enabling the UART's internal loopback ties TX to RX inside the peripheral, so
 * the bytes the NMEA task writes come straight back.  We let it run for two
 * seconds, then verify that well-formed sentences with correct checksums
 * arrived and that both GGA and RMC are present.  This proves the framing, the
 * checksum arithmetic, the talker selection and the pin muxing without needing
 * a flight controller, a logic analyser or a working Wi-Fi link.
 *
 * Loopback is switched off again before the RTCM task starts, so normal
 * operation is unaffected.
 */
/*
 * Push a synthetic RTCM 3.x frame through the UART loopback and confirm the
 * production parser accepts it.  Returns 1 on success.
 */
static int self_test_rtcm(void)
{
    uint8_t frame[64];
    const uint16_t type = 1005;
    const uint16_t plen = 19;

    frame[0] = 0xD3;
    frame[1] = (uint8_t)((plen >> 8) & 0x03);
    frame[2] = (uint8_t)(plen & 0xFF);
    uint8_t *pl = frame + 3;
    memset(pl, 0, plen);
    pl[0] = (uint8_t)(type >> 4);
    pl[1] = (uint8_t)((type & 0x0F) << 4);
    for (uint16_t i = 2; i < plen; ++i) {
        pl[i] = (uint8_t)(0x40 + i);
    }
    const uint32_t crc = rtcm_crc24q(frame, (size_t)3 + plen);
    frame[3 + plen + 0] = (uint8_t)((crc >> 16) & 0xFF);
    frame[3 + plen + 1] = (uint8_t)((crc >> 8) & 0xFF);
    frame[3 + plen + 2] = (uint8_t)(crc & 0xFF);
    const size_t total = (size_t)3 + plen + 3;

    ESP_ERROR_CHECK(uart_set_loop_back(GPS_UART, true));
    uart_flush_input(GPS_UART);
    uart_write_bytes(GPS_UART, (const char *)frame, total);
    vTaskDelay(pdMS_TO_TICKS(60));

    uint8_t rb[128];
    const int n = uart_read_bytes(GPS_UART, rb, sizeof(rb), pdMS_TO_TICKS(120));
    ESP_ERROR_CHECK(uart_set_loop_back(GPS_UART, false));
    uart_flush_input(GPS_UART);

    if (n <= 0) {
        ESP_LOGE(TAG, "self-test: RTCM loopback read nothing");
        return 0;
    }

    rtcm_parser_t probe;
    rtcm_init(&probe);
    rtcm_push(&probe, rb, (size_t)n, now_ms(), NULL, NULL);

    /* rtcm_push counts CRC failures internally; a good frame leaves it at 0. */
    const bool ok = (probe.frame_count == 1 && probe.crc_errors == 0 &&
                     probe.last_msg_type == type);
    ESP_LOGI(TAG, "self-test: RTCM %u bytes -> frames=%u crc_err=%u type=%u",
             (unsigned)n, (unsigned)probe.frame_count,
             (unsigned)probe.crc_errors, (unsigned)probe.last_msg_type);
    if (!ok) {
        ESP_LOGE(TAG, "self-test: RTCM framing failed");
        return 0;
    }
    return 1;
}

static void self_test_run(void)
{
    ESP_LOGI(TAG, "self-test: UART%d internal loopback, 2 s", GPS_UART);
    ESP_ERROR_CHECK(uart_set_loop_back(GPS_UART, true));
    uart_flush_input(GPS_UART);

    char buf[1024];
    size_t used = 0;
    const uint32_t deadline = now_ms() + 2000;

    while ((int32_t)(deadline - now_ms()) > 0 && used < sizeof(buf) - 1) {
        const int n = uart_read_bytes(GPS_UART, (uint8_t *)buf + used,
                                      sizeof(buf) - 1 - used, pdMS_TO_TICKS(100));
        if (n > 0) {
            used += (size_t)n;
        }
    }
    buf[used] = '\0';

    ESP_ERROR_CHECK(uart_set_loop_back(GPS_UART, false));
    uart_flush_input(GPS_UART);

    /* Walk the captured stream line by line, verifying the XOR checksum. */
    int sentences = 0, bad = 0, gga = 0, rmc = 0;
    bool echoed_hdt = false;
    char *line_start = buf;
    while (line_start && *line_start) {
        char *eol = strchr(line_start, '\n');
        if (eol) {
            *eol = '\0';
        }
        char *star = strchr(line_start, '*');
        if (line_start[0] == '$' && star && (star - line_start) > 1) {
            char body[256];
            const size_t blen = (size_t)(star - (line_start + 1));
            if (blen < sizeof(body)) {
                memcpy(body, line_start + 1, blen);
                body[blen] = '\0';
                const unsigned want = (unsigned)strtoul(star + 1, NULL, 16);
                if ((unsigned)nmea_checksum(body) == (want & 0xFF)) {
                    sentences++;
                    if (strstr(body, "GGA")) {
                        gga++;
                        if (gga == 1) {
                            /* print one full sentence so the operator can see
                               the real bytes the flight controller will read */
                            ESP_LOGI(TAG, "self-test: %s", line_start);
                        }
                    }
                    if (strstr(body, "RMC")) {
                        rmc++;
                        if (rmc == 1) {
                            ESP_LOGI(TAG, "self-test: %s", line_start);
                        }
                    }
                    if (strncmp(body + 2, "HDT,", 4) == 0 && !echoed_hdt) {
                        echoed_hdt = true;
                        ESP_LOGI(TAG, "self-test: %s", line_start);
                    }
                } else {
                    bad++;
                }
            }
        }
        line_start = eol ? eol + 1 : NULL;
    }

    ESP_LOGI(TAG, "self-test: %d valid sentences, %d bad checksums, "
                  "GGA=%d RMC=%d", sentences, bad, gga, rmc);
    /* ---- RTCM receive path ----
       The corrections side had a real bug that only a byte-level test caught
       (the CRC was computed across the 3-byte header buffer).  Exercise the
       same path on hardware: build a frame, push it through the loopback, and
       let the production parser read it back. */
    int rtcm_ok = self_test_rtcm();

    const bool nmea_ok = (sentences > 0 && bad == 0 && gga > 0 && rmc > 0);
    if (nmea_ok && rtcm_ok) {
        ESP_LOGI(TAG, "self-test: PASS");
    } else {
        ESP_LOGE(TAG, "self-test: FAIL - nmea_ok=%d rtcm_ok=%d", (int)nmea_ok, rtcm_ok);
    }
}

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    g_sim_lock = xSemaphoreCreateMutex();
    configASSERT(g_sim_lock);

    sim_init(&g_sim, now_ms());
    rtcm_init(&g_rtcm);
    memset(&g_link, 0, sizeof(g_link));

    /* Seed the clock so NMEA time/date fields look sane before SNTP could run. */
    setenv("TZ", "UTC0", 1);
    tzset();

    led_init();
    status_gpio_init();
    uart_setup();
    wifi_start();

    ESP_LOGI(TAG, "USV GPS/RTK simulator starting");
    ESP_LOGI(TAG, "  GPS UART%d  TX=GPIO%d RX=GPIO%d @ %d 8N1",
             GPS_UART, GPS_TX_GPIO, GPS_RX_GPIO, GPS_BAUD_DEFAULT);
    ESP_LOGI(TAG, "  monitor UART%d TX=GPIO%d", MON_UART, CONFIG_USV_GPS_MON_TX_GPIO);
    ESP_LOGI(TAG, "  origin %.7f, %.7f  (override via http://<ip>/api/set)",
             g_sim.origin_lat, g_sim.origin_lon);

    xTaskCreate(nmea_task, "nmea", NMEA_TASK_STACK, NULL, 6, NULL);

    /* Run the loopback check once the NMEA task is producing sentences, but
       before the RTCM task starts consuming the port. */
    self_test_run();

    xTaskCreate(rtcm_task, "rtcm", RTCM_TASK_STACK, NULL, 7, NULL);
    xTaskCreate(status_gpio_task, "status", 3072, NULL, 3, NULL);
    web_start(WEB_TASK_STACK);
}
