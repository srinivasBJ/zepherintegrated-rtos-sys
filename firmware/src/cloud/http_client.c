/*
 * =============================================================================
 * Cloud HTTP Client
 * Vehicle Emergency Response System — Zephyr RTOS
 *
 * Sends emergency events to the FastAPI backend as JSON over HTTP/1.1.
 * Uses Zephyr's built-in HTTP client library (CONFIG_HTTP_CLIENT=y).
 *
 * Endpoint: POST http://<CLOUD_HOST>:<CLOUD_PORT><CLOUD_PATH>
 * =============================================================================
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/http/client.h>
#include <string.h>
#include <stdio.h>

#include "cloud/http_client.h"
#include "vers_types.h"

LOG_MODULE_REGISTER(http_client, LOG_LEVEL_INF);

/* ─── Config (set via CMake -D flags) ──────────────────────────────────── */

#ifndef CLOUD_HOST
#define CLOUD_HOST "127.0.0.1"
#endif
#ifndef CLOUD_PORT
#define CLOUD_PORT 8000
#endif
#ifndef CLOUD_PATH
#define CLOUD_PATH "/api/event"
#endif

#define HTTP_TIMEOUT_MS 10000
#define JSON_BUF_SIZE   512
#define RECV_BUF_SIZE   512

/* ─── JSON builder ──────────────────────────────────────────────────────── */

static const char *alert_type_str(alert_type_t t)
{
    switch (t) {
    case ALERT_CRASH:    return "CRASH";
    case ALERT_HR_LOW:   return "HR_LOW";
    case ALERT_HR_HIGH:  return "HR_HIGH";
    case ALERT_NO_GPS:   return "NO_GPS";
    case ALERT_ROLLOVER: return "ROLLOVER";
    case ALERT_MANUAL:   return "MANUAL_SOS";
    default:             return "UNKNOWN";
    }
}

static int build_json(const emergency_event_t *event, char *out, size_t max)
{
    return snprintf(out, max,
        "{"
          "\"device_id\":\"%s\","
          "\"event_id\":%u,"
          "\"alert_type\":\"%s\","
          "\"accel\":{"
            "\"x_g\":%.3f,\"y_g\":%.3f,\"z_g\":%.3f,"
            "\"magnitude_g\":%.3f"
          "},"
          "\"heart_rate\":{"
            "\"bpm\":%u,\"spo2\":%u,\"valid\":%s"
          "},"
          "\"gps\":{"
            "\"latitude\":%.6f,\"longitude\":%.6f,"
            "\"speed_kmh\":%.1f,\"altitude\":%.1f,"
            "\"satellites\":%u,\"fix_valid\":%s,"
            "\"utc_time\":\"%s\",\"utc_date\":\"%s\""
          "},"
          "\"firmware_ts\":%lld"
        "}",
        event->device_id,
        event->event_id,
        alert_type_str(event->type),
        (double)event->accel.x_g,
        (double)event->accel.y_g,
        (double)event->accel.z_g,
        (double)event->accel.magnitude_g,
        event->hr.bpm,
        event->hr.spo2,
        event->hr.valid ? "true" : "false",
        event->gps.latitude,
        event->gps.longitude,
        (double)event->gps.speed_kmh,
        (double)event->gps.altitude,
        event->gps.satellites,
        event->gps.fix_valid ? "true" : "false",
        event->gps.utc_time,
        event->gps.utc_date,
        (long long)k_uptime_get()
    );
}

/* ─── HTTP response callback ────────────────────────────────────────────── */

static uint8_t recv_buf[RECV_BUF_SIZE];
static int http_status_code = 0;

static void http_response_cb(struct http_response *rsp,
                              enum http_final_call final_data,
                              void *user_data)
{
    ARG_UNUSED(user_data);
    if (final_data == HTTP_DATA_FINAL) {
        http_status_code = rsp->http_status_code;
        LOG_INF("[HTTP] Response: %d — %.*s",
                http_status_code,
                (int)MIN(rsp->data_len, 80),
                rsp->recv_buf);
    }
}

/* ─── POST emergency event ──────────────────────────────────────────────── */

int http_post_emergency(const emergency_event_t *event)
{
    if (!event) return -EINVAL;

    /* Build JSON body */
    static char json_body[JSON_BUF_SIZE];
    int json_len = build_json(event, json_body, sizeof(json_body));
    if (json_len < 0 || json_len >= (int)sizeof(json_body)) {
        LOG_ERR("[HTTP] JSON buffer overflow (%d bytes)", json_len);
        return -ENOMEM;
    }

    /* Build Content-Length header */
    static char content_len_hdr[32];
    snprintf(content_len_hdr, sizeof(content_len_hdr),
             "Content-Length: %d\r\n", json_len);

    /* Resolve host */
    struct zsock_addrinfo hints = {
        .ai_family   = AF_INET,
        .ai_socktype = SOCK_STREAM,
    };
    struct zsock_addrinfo *addr;
    char port_str[8];
    snprintf(port_str, sizeof(port_str), "%d", CLOUD_PORT);

    int ret = zsock_getaddrinfo(CLOUD_HOST, port_str, &hints, &addr);
    if (ret) {
        LOG_ERR("[HTTP] DNS resolve failed: %d", ret);
        return -ENETUNREACH;
    }

    /* Open TCP socket */
    int sock = zsock_socket(addr->ai_family, addr->ai_socktype, 0);
    if (sock < 0) {
        LOG_ERR("[HTTP] Socket create failed: %d", sock);
        zsock_freeaddrinfo(addr);
        return sock;
    }

    ret = zsock_connect(sock, addr->ai_addr, addr->ai_addrlen);
    zsock_freeaddrinfo(addr);
    if (ret) {
        LOG_ERR("[HTTP] Connect to %s:%d failed: %d", CLOUD_HOST, CLOUD_PORT, ret);
        zsock_close(sock);
        return ret;
    }

    /* Prepare HTTP request */
    struct http_request req = {
        .method        = HTTP_POST,
        .url           = CLOUD_PATH,
        .host          = CLOUD_HOST,
        .protocol      = "HTTP/1.1",
        .payload       = json_body,
        .payload_len   = (uint32_t)json_len,
        .header_fields = (const char *[]){
            "Content-Type: application/json\r\n",
            content_len_hdr,
            NULL
        },
        .response      = http_response_cb,
        .recv_buf      = recv_buf,
        .recv_buf_len  = sizeof(recv_buf),
    };

    http_status_code = 0;
    ret = http_client_req(sock, &req, HTTP_TIMEOUT_MS, NULL);
    zsock_close(sock);

    if (ret < 0) {
        LOG_ERR("[HTTP] Request failed: %d", ret);
        return ret;
    }

    if (http_status_code < 200 || http_status_code >= 300) {
        LOG_ERR("[HTTP] Server returned %d", http_status_code);
        return -EIO;
    }

    LOG_INF("[HTTP] Event #%u posted successfully (%d bytes, HTTP %d)",
            event->event_id, json_len, http_status_code);
    return 0;
}
