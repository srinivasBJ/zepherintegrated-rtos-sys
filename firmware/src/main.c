/*
 * =============================================================================
 * Vehicle Emergency Response System
 * main.c — System Entry Point & Thread Orchestration
 *
 * Zephyr RTOS version: 3.x
 * Target boards: native_posix (simulation), nrf52840dk_nrf52840 (hardware)
 *
 * Thread architecture:
 *   sensor_thread    (P=5)  — 50 ms poll: accel, HR, GPS → zbus channels
 *   decision_thread  (P=4)  — 100 ms: reads channels, runs crash detection
 *   comm_thread      (P=6)  — blocks on emergency_chan, sends HTTP/SMS
 *
 * ISR: ADXL345 INT1 → accel interrupt callback → wakes decision_thread early
 * =============================================================================
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/zbus/zbus.h>
#include <string.h>
#include <stdio.h>

#include "vers_types.h"
#include "sensors/accel.h"
#include "sensors/heartrate.h"
#include "sensors/gps.h"
#include "decision/engine.h"
#include "comm/gsm.h"
#include "cloud/http_client.h"

#ifdef CONFIG_BOARD_NATIVE_POSIX
#include "sim/sensor_sim.h"
#endif

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* ─────────────────────────────────────────────────────────────────────────── */
/*  Zbus channel definitions                                                   */
/* ─────────────────────────────────────────────────────────────────────────── */

ZBUS_CHAN_DEFINE(accel_chan,
    accel_sample_t,
    NULL,           /* validator */
    NULL,           /* user_data */
    ZBUS_OBSERVERS_EMPTY,
    ZBUS_MSG_INIT(.x_g = 0, .y_g = 0, .z_g = 0)
);

ZBUS_CHAN_DEFINE(hr_chan,
    hr_sample_t,
    NULL, NULL,
    ZBUS_OBSERVERS_EMPTY,
    ZBUS_MSG_INIT(.bpm = 0, .valid = false)
);

ZBUS_CHAN_DEFINE(gps_chan,
    gps_fix_t,
    NULL, NULL,
    ZBUS_OBSERVERS_EMPTY,
    ZBUS_MSG_INIT(.fix_valid = false)
);

ZBUS_CHAN_DEFINE(emergency_chan,
    emergency_event_t,
    NULL, NULL,
    ZBUS_OBSERVERS_EMPTY,
    ZBUS_MSG_INIT(.type = ALERT_CRASH)
);

/* ─────────────────────────────────────────────────────────────────────────── */
/*  Thread stacks                                                              */
/* ─────────────────────────────────────────────────────────────────────────── */

K_THREAD_STACK_DEFINE(sensor_stack,   SENSOR_STACK_SIZE);
K_THREAD_STACK_DEFINE(decision_stack, DECISION_STACK_SIZE);
K_THREAD_STACK_DEFINE(comm_stack,     COMM_STACK_SIZE);

static struct k_thread sensor_thread_data;
static struct k_thread decision_thread_data;
static struct k_thread comm_thread_data;

/* ─────────────────────────────────────────────────────────────────────────── */
/*  Thread semaphore: ISR can wake decision thread early                       */
/* ─────────────────────────────────────────────────────────────────────────── */

K_SEM_DEFINE(accel_isr_sem, 0, 1);

/* ─────────────────────────────────────────────────────────────────────────── */
/*  ADXL345 GPIO interrupt callback                                            */
/* ─────────────────────────────────────────────────────────────────────────── */

static const struct gpio_dt_spec accel_int =
    GPIO_DT_SPEC_GET(DT_ALIAS(accel_int), gpios);

static struct gpio_callback accel_gpio_cb;

static void accel_isr_handler(const struct device *dev,
                               struct gpio_callback *cb,
                               uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);
    /* Wake decision thread immediately on hardware interrupt */
    k_sem_give(&accel_isr_sem);
}

/* ─────────────────────────────────────────────────────────────────────────── */
/*  Thread A: Sensor Acquisition                                               */
/* ─────────────────────────────────────────────────────────────────────────── */

static void sensor_thread_fn(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

    LOG_INF("[Sensor] Thread started — poll interval %d ms", SENSOR_POLL_MS);

    accel_sample_t  accel = {0};
    hr_sample_t     hr    = {0};
    gps_fix_t       gps   = {0};

    while (1) {
        /* ── Accelerometer ── */
        if (accel_read(&accel) == 0) {
            zbus_chan_pub(&accel_chan, &accel, K_NO_WAIT);
            LOG_DBG("[Sensor] Accel: X=%.2f Y=%.2f Z=%.2f |G|=%.2f",
                    (double)accel.x_g, (double)accel.y_g,
                    (double)accel.z_g, (double)accel.magnitude_g);
        } else {
            LOG_WRN("[Sensor] Accelerometer read failed");
        }

        /* ── Heart Rate ── */
        if (hr_read(&hr) == 0) {
            zbus_chan_pub(&hr_chan, &hr, K_NO_WAIT);
            LOG_DBG("[Sensor] HR: %d BPM, SpO2: %d%%", hr.bpm, hr.spo2);
        }

        /* ── GPS ── */
        if (gps_read(&gps) == 0) {
            zbus_chan_pub(&gps_chan, &gps, K_NO_WAIT);
            LOG_DBG("[Sensor] GPS: %.6f,%.6f spd=%.1f km/h sats=%d",
                    gps.latitude, gps.longitude,
                    (double)gps.speed_kmh, gps.satellites);
        }

        k_sleep(K_MSEC(SENSOR_POLL_MS));
    }
}

/* ─────────────────────────────────────────────────────────────────────────── */
/*  Thread B: Decision Engine                                                  */
/* ─────────────────────────────────────────────────────────────────────────── */

static void decision_thread_fn(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

    LOG_INF("[Decision] Thread started — crash threshold %.0fg", (double)CRASH_G_THRESHOLD);

    engine_init();

    while (1) {
        /* Wait for periodic timeout OR hardware ISR wake */
        int sem_ret = k_sem_take(&accel_isr_sem, K_MSEC(DECISION_POLL_MS));
        if (sem_ret == 0) {
            LOG_DBG("[Decision] Woken by ADXL345 ISR");
        }

        /* Pull latest samples from zbus */
        accel_sample_t  accel = {0};
        hr_sample_t     hr    = {0};
        gps_fix_t       gps   = {0};

        zbus_chan_read(&accel_chan, &accel, K_NO_WAIT);
        zbus_chan_read(&hr_chan,    &hr,    K_NO_WAIT);
        zbus_chan_read(&gps_chan,   &gps,   K_NO_WAIT);

        /* Run the state machine */
        emergency_event_t event = {0};
        bool need_alert = engine_process(&accel, &hr, &gps, &event);

        if (need_alert) {
            LOG_WRN("[Decision] EMERGENCY detected — type=%d", event.type);
            zbus_chan_pub(&emergency_chan, &event, K_MSEC(100));
        }
    }
}

/* ─────────────────────────────────────────────────────────────────────────── */
/*  Thread C: Communication Manager                                            */
/* ─────────────────────────────────────────────────────────────────────────── */

static void comm_thread_fn(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

    LOG_INF("[Comm] Thread started — awaiting emergency events");

    gsm_init();

    while (1) {
        emergency_event_t event = {0};

        /* Block indefinitely until an event arrives */
        int ret = zbus_chan_read(&emergency_chan, &event, K_FOREVER);
        if (ret != 0) {
            LOG_ERR("[Comm] zbus read error: %d", ret);
            continue;
        }

        LOG_INF("[Comm] Processing emergency event #%u type=%d",
                event.event_id, event.type);

        /* Try HTTP first, fall back to SMS */
        ret = http_post_emergency(&event);
        if (ret != 0) {
            LOG_WRN("[Comm] HTTP POST failed (%d), falling back to SMS", ret);
            ret = gsm_send_sms(EMERGENCY_SMS_NUMBER, &event);
            if (ret != 0) {
                LOG_ERR("[Comm] SMS also failed (%d) — event lost!", ret);
            }
        } else {
            LOG_INF("[Comm] Emergency event #%u sent via HTTP", event.event_id);
        }
    }
}

/* ─────────────────────────────────────────────────────────────────────────── */
/*  main()                                                                     */
/* ─────────────────────────────────────────────────────────────────────────── */

int main(void)
{
    LOG_INF("╔══════════════════════════════════════════════╗");
    LOG_INF("║  Vehicle Emergency Response System v1.0.0   ║");
    LOG_INF("║  Zephyr RTOS — Build: " __DATE__ " " __TIME__ "   ║");
    LOG_INF("╚══════════════════════════════════════════════╝");

#ifdef CONFIG_BOARD_NATIVE_POSIX
    LOG_INF("[Main] Running on native_posix — sensor simulation active");
    sensor_sim_start();
#endif

    /* ── Peripheral initialisation ── */
    int ret;

    ret = accel_init();
    if (ret) { LOG_ERR("[Main] Accelerometer init failed: %d", ret); }

    ret = hr_init();
    if (ret) { LOG_ERR("[Main] Heart-rate sensor init failed: %d", ret); }

    ret = gps_init();
    if (ret) { LOG_ERR("[Main] GPS init failed: %d", ret); }

    /* ── GPIO interrupt from ADXL345 ── */
    if (!gpio_is_ready_dt(&accel_int)) {
        LOG_WRN("[Main] ACCEL_INT GPIO not ready — ISR disabled");
    } else {
        gpio_pin_configure_dt(&accel_int, GPIO_INPUT);
        gpio_init_callback(&accel_gpio_cb, accel_isr_handler,
                           BIT(accel_int.pin));
        gpio_add_callback(accel_int.port, &accel_gpio_cb);
        gpio_pin_interrupt_configure_dt(&accel_int, GPIO_INT_EDGE_RISING);
        LOG_INF("[Main] ADXL345 INT1 ISR registered on pin %d", accel_int.pin);
    }

    /* ── Spawn threads ── */
    k_thread_create(&sensor_thread_data, sensor_stack,
                    K_THREAD_STACK_SIZEOF(sensor_stack),
                    sensor_thread_fn, NULL, NULL, NULL,
                    SENSOR_THREAD_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&sensor_thread_data, "sensor");

    k_thread_create(&decision_thread_data, decision_stack,
                    K_THREAD_STACK_SIZEOF(decision_stack),
                    decision_thread_fn, NULL, NULL, NULL,
                    DECISION_THREAD_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&decision_thread_data, "decision");

    k_thread_create(&comm_thread_data, comm_stack,
                    K_THREAD_STACK_SIZEOF(comm_stack),
                    comm_thread_fn, NULL, NULL, NULL,
                    COMM_THREAD_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&comm_thread_data, "comm");

    LOG_INF("[Main] All threads started — system operational");

    /* Main thread becomes idle supervisor */
    while (1) {
        k_sleep(K_SECONDS(5));
        sys_state_t state = engine_get_state();
        LOG_INF("[Main] Heartbeat — sys_state=%d", state);
    }

    return 0;
}
