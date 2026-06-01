/*
 * =============================================================================
 * PA1010D GPS Module Driver — NMEA 0183 Parser
 * Vehicle Emergency Response System — Zephyr RTOS
 *
 * Parses $GPRMC and $GPGGA sentences from GPS UART.
 * Extracts: latitude, longitude, speed (knots→km/h), altitude, fix validity,
 *           satellite count, UTC time and date.
 * =============================================================================
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/device.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "sensors/gps.h"
#include "vers_types.h"

LOG_MODULE_REGISTER(gps_drv, LOG_LEVEL_DBG);

/* ─── UART device ───────────────────────────────────────────────────────── */

static const struct device *gps_uart;

/* ─── NMEA rx buffer ────────────────────────────────────────────────────── */

#define NMEA_MAX_LEN  100
static char  nmea_buf[NMEA_MAX_LEN];
static int   nmea_len = 0;

/* ─── Last valid fix ────────────────────────────────────────────────────── */

static gps_fix_t last_fix = { .fix_valid = false };
static K_MUTEX_DEFINE(fix_mutex);

/* ─── Coordinate conversion: DDDMM.MMMM → decimal degrees ─────────────── */

static double nmea_to_deg(const char *raw)
{
    double val = atof(raw);
    int deg = (int)(val / 100);
    double min = val - (deg * 100);
    return deg + min / 60.0;
}

/* ─── NMEA checksum validation ──────────────────────────────────────────── */

static bool nmea_checksum_ok(const char *sentence)
{
    const char *p = sentence + 1; /* skip '$' */
    uint8_t calc = 0;
    while (*p && *p != '*') calc ^= (uint8_t)*p++;
    if (*p != '*') return false;
    uint8_t expected = (uint8_t)strtol(p + 1, NULL, 16);
    return calc == expected;
}

/* ─── Field extractor ───────────────────────────────────────────────────── */

static char *nmea_field(char *sentence, int idx)
{
    char *p = sentence;
    int  count = 0;
    while (*p) {
        if (*p == ',') {
            count++;
            if (count == idx) return p + 1;
        }
        p++;
    }
    return NULL;
}

/* ─── Parse $GPRMC ──────────────────────────────────────────────────────── */

static void parse_gprmc(char *line)
{
    /* $GPRMC,HHMMSS.ss,A,LLLL.LL,a,YYYYY.YY,a,x.x,x.x,DDMMYY,,,*hh */
    if (!nmea_checksum_ok(line)) return;

    char *f[13] = {0};
    char tmp[NMEA_MAX_LEN];
    strncpy(tmp, line, NMEA_MAX_LEN - 1);

    char *tok = strtok(tmp, ",");
    for (int i = 0; tok && i < 13; i++, tok = strtok(NULL, ",")) {
        f[i] = tok;
    }

    if (!f[2] || f[2][0] != 'A') return; /* Not a valid fix */

    k_mutex_lock(&fix_mutex, K_FOREVER);

    last_fix.fix_valid  = true;
    last_fix.latitude   = nmea_to_deg(f[3]);
    if (f[4] && f[4][0] == 'S') last_fix.latitude  = -last_fix.latitude;
    last_fix.longitude  = nmea_to_deg(f[5]);
    if (f[6] && f[6][0] == 'W') last_fix.longitude = -last_fix.longitude;
    last_fix.speed_kmh  = (float)atof(f[7]) * 1.852f; /* knots → km/h */
    last_fix.timestamp  = k_uptime_get();

    /* UTC time: HHMMSS.ss */
    if (f[1] && strlen(f[1]) >= 6) {
        snprintf(last_fix.utc_time, sizeof(last_fix.utc_time),
                 "%c%c:%c%c:%c%c",
                 f[1][0], f[1][1], f[1][2],
                 f[1][3], f[1][4], f[1][5]);
    }
    /* UTC date: DDMMYY */
    if (f[9] && strlen(f[9]) >= 6) {
        snprintf(last_fix.utc_date, sizeof(last_fix.utc_date),
                 "%c%c/%c%c/%c%c",
                 f[9][0], f[9][1], f[9][2],
                 f[9][3], f[9][4], f[9][5]);
    }

    k_mutex_unlock(&fix_mutex);

    LOG_DBG("GPS GPRMC: %.6f,%.6f spd=%.1f",
            last_fix.latitude, last_fix.longitude,
            (double)last_fix.speed_kmh);
}

/* ─── Parse $GPGGA ──────────────────────────────────────────────────────── */

static void parse_gpgga(char *line)
{
    /* $GPGGA,HHMMSS.ss,LLLL.LL,a,YYYYY.YY,a,x,xx,x.x,x.x,M,,,,*hh */
    if (!nmea_checksum_ok(line)) return;

    char tmp[NMEA_MAX_LEN];
    strncpy(tmp, line, NMEA_MAX_LEN - 1);

    char *f[15] = {0};
    char *tok = strtok(tmp, ",");
    for (int i = 0; tok && i < 15; i++, tok = strtok(NULL, ",")) {
        f[i] = tok;
    }

    k_mutex_lock(&fix_mutex, K_FOREVER);
    if (f[7])  last_fix.satellites = (uint8_t)atoi(f[7]);
    if (f[9])  last_fix.altitude   = (float)atof(f[9]);
    k_mutex_unlock(&fix_mutex);
}

/* ─── UART ISR callback ─────────────────────────────────────────────────── */

static void gps_uart_cb(const struct device *dev, void *user_data)
{
    ARG_UNUSED(user_data);

    while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
        if (!uart_irq_rx_ready(dev)) break;

        uint8_t c;
        if (uart_fifo_read(dev, &c, 1) != 1) break;

        if (c == '$') {
            nmea_len = 0;
        }

        if (nmea_len < NMEA_MAX_LEN - 1) {
            nmea_buf[nmea_len++] = (char)c;
        }

        if (c == '\n') {
            nmea_buf[nmea_len] = '\0';
            if (strncmp(nmea_buf, "$GPRMC", 6) == 0) {
                parse_gprmc(nmea_buf);
            } else if (strncmp(nmea_buf, "$GPGGA", 6) == 0) {
                parse_gpgga(nmea_buf);
            }
            nmea_len = 0;
        }
    }
}

/* ─── Init ──────────────────────────────────────────────────────────────── */

int gps_init(void)
{
    gps_uart = DEVICE_DT_GET(DT_NODELABEL(uart1));
    if (!device_is_ready(gps_uart)) {
        LOG_ERR("GPS UART not ready");
        return -ENODEV;
    }

    uart_irq_callback_set(gps_uart, gps_uart_cb);
    uart_irq_rx_enable(gps_uart);

    LOG_INF("GPS initialised — NMEA parser active on UART1 @ 9600 baud");
    return 0;
}

/* ─── Read (blocking, max 1.1s for GPRMC period) ────────────────────────── */

int gps_read(gps_fix_t *fix)
{
    if (!fix) return -EINVAL;

    k_mutex_lock(&fix_mutex, K_FOREVER);
    memcpy(fix, &last_fix, sizeof(gps_fix_t));
    k_mutex_unlock(&fix_mutex);

    return fix->fix_valid ? 0 : -EAGAIN;
}

const gps_fix_t *gps_last_fix(void)
{
    return &last_fix;
}
