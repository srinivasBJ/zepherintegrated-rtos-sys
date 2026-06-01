/*
 * =============================================================================
 * SIM800L GSM Modem Driver — AT Command Interface
 * Vehicle Emergency Response System — Zephyr RTOS
 *
 * Modem: SIMCom SIM800L
 *   - UART: 115200 baud, 8N1
 *   - SMS text mode (PDU mode also supported)
 *   - Network: GSM 850/900/1800/1900 MHz quad-band
 *
 * AT command reference: SIM800 Series_AT Command Manual_V1.12
 * =============================================================================
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/device.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "comm/gsm.h"
#include "vers_types.h"

LOG_MODULE_REGISTER(gsm_drv, LOG_LEVEL_INF);

/* ─── UART device ───────────────────────────────────────────────────────── */

static const struct device *gsm_uart;

/* ─── AT response buffer ────────────────────────────────────────────────── */

#define AT_BUF_SIZE  256
static char at_rx_buf[AT_BUF_SIZE];
static int  at_rx_len = 0;

K_SEM_DEFINE(at_resp_sem, 0, 1);

/* ─── UART RX callback ──────────────────────────────────────────────────── */

static void gsm_uart_cb(const struct device *dev, void *user_data)
{
    ARG_UNUSED(user_data);

    while (uart_irq_update(dev) && uart_irq_rx_ready(dev)) {
        uint8_t c;
        if (uart_fifo_read(dev, &c, 1) != 1) break;

        if (at_rx_len < AT_BUF_SIZE - 1) {
            at_rx_buf[at_rx_len++] = (char)c;
            at_rx_buf[at_rx_len]   = '\0';
        }

        /* Signal on \n — caller checks for "OK" / "ERROR" */
        if (c == '\n') {
            k_sem_give(&at_resp_sem);
        }
    }
}

/* ─── Send AT command, wait for expected response ────────────────────────── */

static int at_cmd(const char *cmd, const char *expect_ok,
                  unsigned int timeout_ms)
{
    /* Flush rx buffer */
    at_rx_len = 0;
    memset(at_rx_buf, 0, AT_BUF_SIZE);

    /* Send command + CR/LF */
    for (const char *p = cmd; *p; p++) {
        uart_poll_out(gsm_uart, (unsigned char)*p);
    }
    uart_poll_out(gsm_uart, '\r');
    uart_poll_out(gsm_uart, '\n');

    LOG_DBG("[GSM] >> %s", cmd);

    /* Wait for response */
    int64_t deadline = k_uptime_get() + timeout_ms;
    while (k_uptime_get() < deadline) {
        k_sem_take(&at_resp_sem, K_MSEC(100));
        if (strstr(at_rx_buf, expect_ok)) {
            LOG_DBG("[GSM] << OK (%s)", at_rx_buf);
            return 0;
        }
        if (strstr(at_rx_buf, "ERROR")) {
            LOG_WRN("[GSM] << ERROR response to: %s", cmd);
            return -EIO;
        }
    }

    LOG_WRN("[GSM] Timeout waiting for '%s' after: %s", expect_ok, cmd);
    return -ETIMEDOUT;
}

/* ─── Init ──────────────────────────────────────────────────────────────── */

int gsm_init(void)
{
    gsm_uart = DEVICE_DT_GET(DT_NODELABEL(uart2));
    if (!device_is_ready(gsm_uart)) {
        LOG_ERR("GSM UART not ready");
        return -ENODEV;
    }

    uart_irq_callback_set(gsm_uart, gsm_uart_cb);
    uart_irq_rx_enable(gsm_uart);

    LOG_INF("[GSM] Waking SIM800L...");
    k_msleep(1000);

    /* Basic AT handshake */
    int ret = at_cmd("AT", "OK", 3000);
    if (ret) {
        LOG_ERR("[GSM] Modem not responding");
        return ret;
    }

    /* Echo off */
    at_cmd("ATE0", "OK", 1000);

    /* Wait for network registration */
    for (int attempts = 0; attempts < 20; attempts++) {
        at_cmd("AT+CREG?", "OK", 2000);
        if (strstr(at_rx_buf, "+CREG: 0,1") ||
            strstr(at_rx_buf, "+CREG: 0,5")) {
            LOG_INF("[GSM] Network registered");
            break;
        }
        LOG_DBG("[GSM] Waiting for network... (%d/20)", attempts + 1);
        k_msleep(2000);
    }

    /* SMS text mode */
    at_cmd("AT+CMGF=1", "OK", 1000);

    /* Show timestamp in messages */
    at_cmd("AT+CSDH=1", "OK", 1000);

    LOG_INF("[GSM] SIM800L initialised — SMS text mode enabled");
    return 0;
}

/* ─── Signal quality ────────────────────────────────────────────────────── */

int gsm_signal_quality(void)
{
    at_cmd("AT+CSQ", "OK", 2000);
    /* +CSQ: <rssi>,<ber>   rssi: 0-31, 99=unknown */
    char *p = strstr(at_rx_buf, "+CSQ: ");
    if (!p) return INT_MIN;
    int rssi = atoi(p + 6);
    if (rssi == 99) return INT_MIN;
    return -113 + (rssi * 2); /* convert to dBm */
}

/* ─── Network registration ──────────────────────────────────────────────── */

bool gsm_is_registered(void)
{
    at_cmd("AT+CREG?", "OK", 2000);
    return strstr(at_rx_buf, "+CREG: 0,1") != NULL ||
           strstr(at_rx_buf, "+CREG: 0,5") != NULL;
}

/* ─── Send SMS ──────────────────────────────────────────────────────────── */

int gsm_send_sms(const char *number, const emergency_event_t *event)
{
    if (!number || !event) return -EINVAL;

    /* Build SMS body */
    char msg[160];
    const char *type_str[] = {
        "CRASH", "HR_LOW", "HR_HIGH", "NO_GPS", "ROLLOVER", "SOS"
    };
    const char *t = (event->type <= ALERT_MANUAL) ?
                    type_str[event->type] : "UNKNOWN";

    snprintf(msg, sizeof(msg),
        "VERS ALERT[%s] id=%u dev=%s "
        "GPS=%.5f,%.5f spd=%.0fkmh "
        "G=%.1f HR=%dbpm SpO2=%d%%",
        t, event->event_id, event->device_id,
        event->gps.latitude, event->gps.longitude,
        (double)event->gps.speed_kmh,
        (double)event->accel.magnitude_g,
        event->hr.bpm, event->hr.spo2);

    LOG_INF("[GSM] Sending SMS to %s: %s", number, msg);

    /* AT+CMGS="<number>" */
    char at_cmd_buf[64];
    snprintf(at_cmd_buf, sizeof(at_cmd_buf), "AT+CMGS=\"%s\"", number);
    int ret = at_cmd(at_cmd_buf, ">", 5000);
    if (ret) return ret;

    /* Send message body + Ctrl-Z */
    for (const char *p = msg; *p; p++) {
        uart_poll_out(gsm_uart, (unsigned char)*p);
    }
    uart_poll_out(gsm_uart, 0x1A); /* Ctrl-Z = send */

    ret = at_cmd("", "+CMGS:", 10000);
    if (ret == 0) {
        LOG_INF("[GSM] SMS delivered OK");
    }
    return ret;
}
