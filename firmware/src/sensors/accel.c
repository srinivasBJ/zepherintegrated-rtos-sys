/*
 * =============================================================================
 * ADXL345 Accelerometer Driver Implementation
 * Vehicle Emergency Response System — Zephyr RTOS
 *
 * Sensor: Analog Devices ADXL345
 *   - I²C address: 0x53 (SDO/ALT = GND) or 0x1D (SDO/ALT = VDD)
 *   - Range: ±2g / ±4g / ±8g / ±16g (configured for ±16g here)
 *   - ODR:   up to 3200 Hz (configured for 100 Hz)
 *   - INT1:  DATA_READY triggers GPIO interrupt
 *
 * On native_posix the Zephyr sensor API calls are forwarded to the
 * simulation layer (sensor_sim.c) which injects synthetic data.
 * =============================================================================
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/device.h>
#include <math.h>
#include <string.h>

#include "sensors/accel.h"
#include "vers_types.h"

LOG_MODULE_REGISTER(accel_drv, LOG_LEVEL_DBG);

/* ─── Device handle ─────────────────────────────────────────────────────── */

static const struct device *adxl345_dev;

/* ─── Simple Moving Average filter ─────────────────────────────────────── */

static float sma_buf[ACCEL_SMA_WINDOW];
static uint8_t sma_idx = 0;
static float   sma_sum = 0.0f;
static bool    sma_full = false;

static void sma_update(float magnitude)
{
    sma_sum -= sma_buf[sma_idx];
    sma_buf[sma_idx] = magnitude;
    sma_sum += magnitude;
    sma_idx = (sma_idx + 1) % ACCEL_SMA_WINDOW;
    if (sma_idx == 0) sma_full = true;
}

float accel_get_filtered_magnitude(void)
{
    uint8_t count = sma_full ? ACCEL_SMA_WINDOW : sma_idx;
    return (count > 0) ? (sma_sum / count) : 0.0f;
}

/* ─── Init ──────────────────────────────────────────────────────────────── */

int accel_init(void)
{
    adxl345_dev = DEVICE_DT_GET(DT_NODELABEL(adxl345));
    if (!device_is_ready(adxl345_dev)) {
        LOG_ERR("ADXL345 device not ready");
        return -ENODEV;
    }

    /* Set measurement range to ±16g */
    struct sensor_value range = { .val1 = 16, .val2 = 0 };
    int ret = sensor_attr_set(adxl345_dev,
                              SENSOR_CHAN_ACCEL_XYZ,
                              SENSOR_ATTR_FULL_SCALE,
                              &range);
    if (ret) {
        LOG_WRN("Could not set ADXL345 range: %d", ret);
    }

    /* Set ODR to 100 Hz */
    struct sensor_value odr = { .val1 = 100, .val2 = 0 };
    ret = sensor_attr_set(adxl345_dev,
                          SENSOR_CHAN_ACCEL_XYZ,
                          SENSOR_ATTR_SAMPLING_FREQUENCY,
                          &odr);
    if (ret) {
        LOG_WRN("Could not set ADXL345 ODR: %d", ret);
    }

    memset(sma_buf, 0, sizeof(sma_buf));
    LOG_INF("ADXL345 initialised (±16g, 100 Hz ODR)");
    return 0;
}

/* ─── Read ──────────────────────────────────────────────────────────────── */

int accel_read(accel_sample_t *sample)
{
    if (!sample) return -EINVAL;

    int ret = sensor_sample_fetch(adxl345_dev);
    if (ret) {
        LOG_ERR("ADXL345 fetch failed: %d", ret);
        return ret;
    }

    struct sensor_value sx, sy, sz;
    sensor_channel_get(adxl345_dev, SENSOR_CHAN_ACCEL_X, &sx);
    sensor_channel_get(adxl345_dev, SENSOR_CHAN_ACCEL_Y, &sy);
    sensor_channel_get(adxl345_dev, SENSOR_CHAN_ACCEL_Z, &sz);

    /*
     * Zephyr sensor API returns m/s².  Convert to g (1g = 9.80665 m/s²).
     */
    sample->x_g = sensor_value_to_float(&sx) / 9.80665f;
    sample->y_g = sensor_value_to_float(&sy) / 9.80665f;
    sample->z_g = sensor_value_to_float(&sz) / 9.80665f;
    sample->magnitude_g = sqrtf(sample->x_g * sample->x_g +
                                 sample->y_g * sample->y_g +
                                 sample->z_g * sample->z_g);
    sample->timestamp = k_uptime_get();

    sma_update(sample->magnitude_g);
    return 0;
}

/* ─── Crash detection ───────────────────────────────────────────────────── */

bool accel_crash_detected(const accel_sample_t *sample)
{
    if (!sample) return false;
    /* Instantaneous peak OR filtered magnitude exceeds threshold */
    return (sample->magnitude_g >= CRASH_G_THRESHOLD) ||
           (accel_get_filtered_magnitude() >= CRASH_G_THRESHOLD * 0.8f);
}

/* ─── Tilt / rollover ───────────────────────────────────────────────────── */

float accel_tilt_angle(const accel_sample_t *sample)
{
    if (!sample) return 0.0f;
    /* Angle of the Z-axis relative to gravity vector */
    float angle_rad = acosf(sample->z_g / sample->magnitude_g);
    return angle_rad * (180.0f / 3.14159265f);
}
