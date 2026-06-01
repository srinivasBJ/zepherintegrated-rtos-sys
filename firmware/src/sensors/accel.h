/*
 * =============================================================================
 * ADXL345 Accelerometer Driver Header
 * Vehicle Emergency Response System — Zephyr RTOS
 * =============================================================================
 */

#ifndef ACCEL_H
#define ACCEL_H

#include "vers_types.h"

/**
 * @brief Initialise the ADXL345 over I²C.
 *
 * Configures measurement range (±16g), output data rate (100 Hz),
 * and enables the DATA_READY interrupt on INT1.
 *
 * @return 0 on success, negative errno on failure.
 */
int accel_init(void);

/**
 * @brief Read a single accelerometer sample.
 *
 * Fills *sample with the latest X/Y/Z values in g-force units and
 * computes the 3-axis magnitude.  Also updates the internal SMA filter.
 *
 * @param sample  Pointer to accel_sample_t to populate.
 * @return 0 on success, negative errno on failure.
 */
int accel_read(accel_sample_t *sample);

/**
 * @brief Return filtered (SMA) magnitude of the last N samples.
 */
float accel_get_filtered_magnitude(void);

/**
 * @brief Check whether the latest sample exceeds the crash threshold.
 *
 * @param sample  Raw sample to test.
 * @return true if |G| >= CRASH_G_THRESHOLD.
 */
bool accel_crash_detected(const accel_sample_t *sample);

/**
 * @brief Estimate tilt angle (degrees) from accelerometer data.
 *        Used for rollover detection.
 *
 * @param sample  Raw sample.
 * @return Tilt angle in degrees (0 = upright, 90 = on side).
 */
float accel_tilt_angle(const accel_sample_t *sample);

#endif /* ACCEL_H */
