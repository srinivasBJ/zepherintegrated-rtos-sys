/*
 * =============================================================================
 * Decision Engine — Crash & Vital-Sign State Machine
 * Vehicle Emergency Response System — Zephyr RTOS
 *
 * State transitions:
 *
 *   INIT ──────────────────────────────────────────► NORMAL
 *   NORMAL ──[crash OR vital alert]──────────────── ► CRASH_ALERT
 *   CRASH_ALERT ──[2-second confirmation window]─── ► EMERGENCY
 *   CRASH_ALERT ──[false positive cleared]────────── ► NORMAL
 *   EMERGENCY ──[ACK received OR 5-minute timeout]── ► NORMAL
 *   ANY ──[manual SOS]──────────────────────────────► EMERGENCY
 *
 * The 2-second confirmation window prevents single-spike false positives
 * (e.g., hitting a pothole).
 * =============================================================================
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <stdio.h>

#include "decision/engine.h"
#include "sensors/accel.h"
#include "sensors/heartrate.h"
#include "vers_types.h"

LOG_MODULE_REGISTER(engine, LOG_LEVEL_INF);

/* ─── State ─────────────────────────────────────────────────────────────── */

static sys_state_t current_state = SYS_STATE_INIT;
static uint32_t    event_counter  = 0;
static int64_t     alert_start_ms = 0;
static bool        manual_sos     = false;

/* ─── Confirmation window ───────────────────────────────────────────────── */
#define CRASH_CONFIRM_MS     2000  /* ms sustained above threshold → EMERGENCY */
#define EMERGENCY_TIMEOUT_MS (5 * 60 * 1000)  /* 5 min auto-clear          */

/* ─── Device ID (from HW UUID or fixed during provisioning) ─────────────── */
static const char DEVICE_ID[] = "VERS-NRF52840-001";

/* ─── Init ──────────────────────────────────────────────────────────────── */

void engine_init(void)
{
    current_state = SYS_STATE_NORMAL;
    event_counter = 0;
    LOG_INF("[Engine] Initialised — state=NORMAL, crash_thresh=%.0fg",
            (double)CRASH_G_THRESHOLD);
}

/* ─── State getter ──────────────────────────────────────────────────────── */

sys_state_t engine_get_state(void)
{
    return current_state;
}

/* ─── Manual SOS trigger ────────────────────────────────────────────────── */

void engine_trigger_manual_sos(void)
{
    LOG_WRN("[Engine] Manual SOS triggered!");
    manual_sos = true;
}

/* ─── Build emergency payload ───────────────────────────────────────────── */

static void build_event(alert_type_t type,
                        const accel_sample_t *accel,
                        const hr_sample_t    *hr,
                        const gps_fix_t      *gps,
                        emergency_event_t    *out)
{
    memset(out, 0, sizeof(*out));
    out->type     = type;
    out->event_id = ++event_counter;
    strncpy(out->device_id, DEVICE_ID, sizeof(out->device_id) - 1);

    if (accel) memcpy(&out->accel, accel, sizeof(accel_sample_t));
    if (hr)    memcpy(&out->hr,    hr,    sizeof(hr_sample_t));
    if (gps)   memcpy(&out->gps,   gps,   sizeof(gps_fix_t));
}

/* ─── Main process function ─────────────────────────────────────────────── */

bool engine_process(const accel_sample_t *accel,
                    const hr_sample_t    *hr,
                    const gps_fix_t      *gps,
                    emergency_event_t    *event)
{
    int64_t now = k_uptime_get();

    /* ── Manual SOS bypasses all state logic ── */
    if (manual_sos) {
        manual_sos = false;
        current_state = SYS_STATE_EMERGENCY;
        build_event(ALERT_MANUAL, accel, hr, gps, event);
        LOG_WRN("[Engine] → EMERGENCY (MANUAL SOS) event_id=%u", event->event_id);
        return true;
    }

    switch (current_state) {

    /* ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ─ */
    case SYS_STATE_NORMAL: {

        bool crash_flag   = accel && accel_crash_detected(accel);
        bool hr_flag      = hr    && hr_vital_alert(hr);
        bool rollover     = accel && (accel_tilt_angle(accel) > ROLLOVER_THRESHOLD);

        if (crash_flag || hr_flag || rollover) {
            current_state  = SYS_STATE_CRASH_ALERT;
            alert_start_ms = now;
            LOG_WRN("[Engine] → CRASH_ALERT (crash=%d hr=%d rollover=%d) |G|=%.2f",
                    crash_flag, hr_flag, rollover,
                    accel ? (double)accel->magnitude_g : 0.0);
        }
        break;
    }

    /* ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ─ */
    case SYS_STATE_CRASH_ALERT: {

        bool still_crash   = accel && accel_crash_detected(accel);
        bool still_hr      = hr    && hr_vital_alert(hr);
        int64_t elapsed    = now - alert_start_ms;

        if (!still_crash && !still_hr) {
            /* Cleared before confirmation window — false positive */
            LOG_INF("[Engine] → NORMAL (false positive cleared after %llims)",
                    (long long)elapsed);
            current_state = SYS_STATE_NORMAL;
            break;
        }

        if (elapsed >= CRASH_CONFIRM_MS) {
            /* Sustained — raise emergency */
            current_state = SYS_STATE_EMERGENCY;

            alert_type_t type = ALERT_CRASH;
            if (accel && accel_tilt_angle(accel) > ROLLOVER_THRESHOLD) {
                type = ALERT_ROLLOVER;
            } else if (!still_crash && still_hr) {
                type = (hr && hr->bpm < HR_MIN_BPM) ? ALERT_HR_LOW : ALERT_HR_HIGH;
            } else if (!gps || !gps->fix_valid) {
                /* Crash with no GPS — flag separately */
                type = ALERT_NO_GPS;
            }

            build_event(type, accel, hr, gps, event);
            LOG_WRN("[Engine] → EMERGENCY type=%d event_id=%u after %llims",
                    type, event->event_id, (long long)elapsed);
            return true;
        }

        LOG_DBG("[Engine] CRASH_ALERT confirmation... %lld/%d ms",
                (long long)elapsed, CRASH_CONFIRM_MS);
        break;
    }

    /* ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ─ */
    case SYS_STATE_EMERGENCY: {
        /* Stay in emergency state for 5 min then auto-recover */
        if ((now - alert_start_ms) > EMERGENCY_TIMEOUT_MS) {
            LOG_INF("[Engine] → NORMAL (emergency timeout)");
            current_state = SYS_STATE_NORMAL;
        }
        break;
    }

    default:
        current_state = SYS_STATE_NORMAL;
        break;
    }

    return false;
}
