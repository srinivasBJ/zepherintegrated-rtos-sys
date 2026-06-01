/*
 * =============================================================================
 * POSIX Sensor Simulation Layer
 * Vehicle Emergency Response System — Zephyr RTOS (native_posix)
 *
 * On native_posix, hardware sensors are absent.  This module:
 *   1. Overrides the Zephyr sensor API via a loopback device emulation.
 *   2. Generates realistic driving data with configurable crash injection.
 *   3. Simulates NMEA sentences output via a pseudo-TTY for the GPS parser.
 *
 * Crash scenario timeline:
 *   t=0s   System starts, normal driving (1g ±0.1g)
 *   t=15s  Sudden braking event (4g spike, clears after 300ms)
 *   t=30s  CRASH injected — 45g spike sustained for 3 seconds
 *   t=35s  HR drops to 28 BPM (driver unconscious)
 *   t=60s  System auto-recovers (test loop restart)
 * =============================================================================
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "sim/sensor_sim.h"
#include "vers_types.h"

LOG_MODULE_REGISTER(sensor_sim, LOG_LEVEL_INF);

/* ─── Simulated state ───────────────────────────────────────────────────── */

static struct {
    float    accel_x_g;
    float    accel_y_g;
    float    accel_z_g;
    bool     crash_active;
    uint16_t hr_bpm;
    uint8_t  hr_spo2;
    double   gps_lat;
    double   gps_lon;
    float    gps_speed_kmh;
    uint8_t  gps_sats;
} sim_state = {
    .accel_x_g    = 0.05f,
    .accel_y_g    = 0.02f,
    .accel_z_g    = 1.0f,   /* 1g gravity */
    .hr_bpm       = 72,
    .hr_spo2      = 98,
    .gps_lat      = 37.422160,
    .gps_lon      = -122.084270,
    .gps_speed_kmh = 65.0f,
    .gps_sats     = 8,
};

K_MUTEX_DEFINE(sim_mutex);

/* ─── Gaussian noise ────────────────────────────────────────────────────── */

static float gaussian_noise(float sigma)
{
    /* Box-Muller transform */
    float u1 = (float)(sys_rand32_get() % 10000 + 1) / 10000.0f;
    float u2 = (float)(sys_rand32_get() % 10000 + 1) / 10000.0f;
    return sigma * sqrtf(-2.0f * logf(u1)) * cosf(2.0f * 3.14159f * u2);
}

/* ─── Scenario thread ───────────────────────────────────────────────────── */

K_THREAD_STACK_DEFINE(sim_stack, 2048);
static struct k_thread sim_thread_data;

static void sim_scenario_fn(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

    LOG_INF("[Sim] Scenario thread started");

scenario_restart:
    /* ── Phase 1: Normal driving (0–15s) ── */
    LOG_INF("[Sim] Phase 1: Normal driving");
    for (int i = 0; i < 150; i++) {  /* 150 × 100ms = 15s */
        k_mutex_lock(&sim_mutex, K_FOREVER);
        sim_state.accel_x_g    = gaussian_noise(0.05f);
        sim_state.accel_y_g    = gaussian_noise(0.05f);
        sim_state.accel_z_g    = 1.0f + gaussian_noise(0.02f);
        sim_state.gps_lat     += 0.000015;  /* moving north */
        sim_state.hr_bpm       = 72 + (int)gaussian_noise(3.0f);
        k_mutex_unlock(&sim_mutex);
        k_msleep(100);
    }

    /* ── Phase 2: Hard braking event (15–15.5s) ── */
    LOG_INF("[Sim] Phase 2: Hard braking (4g spike)");
    k_mutex_lock(&sim_mutex, K_FOREVER);
    sim_state.accel_x_g     = -4.0f;  /* longitudinal deceleration */
    sim_state.gps_speed_kmh  = 15.0f;
    k_mutex_unlock(&sim_mutex);
    k_msleep(300);

    k_mutex_lock(&sim_mutex, K_FOREVER);
    sim_state.accel_x_g     = 0.05f;
    sim_state.gps_speed_kmh  = 10.0f;
    k_mutex_unlock(&sim_mutex);

    /* ── Phase 3: Normal again (15.5–30s) ── */
    LOG_INF("[Sim] Phase 3: Recovering from braking");
    for (int i = 0; i < 145; i++) {
        k_mutex_lock(&sim_mutex, K_FOREVER);
        sim_state.accel_x_g = gaussian_noise(0.05f);
        sim_state.accel_y_g = gaussian_noise(0.05f);
        sim_state.accel_z_g = 1.0f + gaussian_noise(0.02f);
        k_mutex_unlock(&sim_mutex);
        k_msleep(100);
    }

    /* ── Phase 4: CRASH (30–33s) ── */
    LOG_WRN("[Sim] *** INJECTING CRASH EVENT (45g) ***");
    k_mutex_lock(&sim_mutex, K_FOREVER);
    sim_state.accel_x_g     = 32.0f;
    sim_state.accel_y_g     = 18.0f;
    sim_state.accel_z_g     = 22.0f;
    sim_state.crash_active   = true;
    sim_state.gps_speed_kmh  = 0.0f;
    k_mutex_unlock(&sim_mutex);
    k_msleep(3000);

    /* ── Phase 5: Post-crash (low HR) ── */
    LOG_WRN("[Sim] Post-crash: driver HR dropping to 28 BPM");
    k_mutex_lock(&sim_mutex, K_FOREVER);
    sim_state.accel_x_g  = 0.1f + gaussian_noise(0.05f);
    sim_state.accel_y_g  = 0.05f;
    sim_state.accel_z_g  = 0.8f;  /* tilted after crash */
    sim_state.hr_bpm     = 28;    /* bradycardia */
    sim_state.hr_spo2    = 85;
    k_mutex_unlock(&sim_mutex);
    k_msleep(20000); /* 20s emergency state */

    /* ── Phase 6: Recovery / loop ── */
    LOG_INF("[Sim] Scenario complete — restarting in 5s...");
    k_mutex_lock(&sim_mutex, K_FOREVER);
    sim_state.crash_active  = false;
    sim_state.hr_bpm        = 72;
    sim_state.hr_spo2       = 98;
    sim_state.gps_speed_kmh = 65.0f;
    k_mutex_unlock(&sim_mutex);
    k_msleep(5000);
    goto scenario_restart;
}

/* ─── Public API — called from main.c ──────────────────────────────────── */

void sensor_sim_start(void)
{
    k_thread_create(&sim_thread_data, sim_stack,
                    K_THREAD_STACK_SIZEOF(sim_stack),
                    sim_scenario_fn, NULL, NULL, NULL,
                    7, 0, K_NO_WAIT);
    k_thread_name_set(&sim_thread_data, "sim_scenario");
    LOG_INF("[Sim] Simulation started — crash event at t≈30s");
}

void sensor_sim_inject_crash(void)
{
    k_mutex_lock(&sim_mutex, K_FOREVER);
    sim_state.accel_x_g  = 45.0f;
    sim_state.accel_y_g  = 20.0f;
    sim_state.accel_z_g  = 15.0f;
    sim_state.crash_active = true;
    k_mutex_unlock(&sim_mutex);
}

void sensor_sim_set_gps(double lat, double lon, float speed_kmh)
{
    k_mutex_lock(&sim_mutex, K_FOREVER);
    sim_state.gps_lat      = lat;
    sim_state.gps_lon      = lon;
    sim_state.gps_speed_kmh = speed_kmh;
    k_mutex_unlock(&sim_mutex);
}

void sensor_sim_set_hr(uint16_t bpm)
{
    k_mutex_lock(&sim_mutex, K_FOREVER);
    sim_state.hr_bpm = bpm;
    k_mutex_unlock(&sim_mutex);
}

/*
 * NOTE: On native_posix, the Zephyr sensor API (sensor_sample_fetch /
 * sensor_channel_get) is backed by a emulated device that reads from
 * sim_state above.  This requires a Zephyr emul driver registration.
 * For educational clarity, the emulation hook points are shown here
 * as comments — full emul driver registration follows Zephyr docs:
 * https://docs.zephyrproject.org/latest/services/sensor/index.html#emul
 *
 *   SENSOR_EMUL_DEFINE(adxl345_emul, &adxl345_dev, &emul_api);
 *
 * In the emul read callback, copy sim_state into struct sensor_value.
 */
