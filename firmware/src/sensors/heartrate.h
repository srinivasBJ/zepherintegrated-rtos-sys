/*
 * MAX30102 Heart-Rate / SpO₂ Driver Header
 * Vehicle Emergency Response System — Zephyr RTOS
 */

#ifndef HEARTRATE_H
#define HEARTRATE_H

#include "vers_types.h"

/** Initialise MAX30102 over I²C (100Hz sample rate, IR+Red LEDs). */
int hr_init(void);

/** Read latest BPM and SpO₂ from MAX30102. */
int hr_read(hr_sample_t *sample);

/** Returns true if BPM is outside safe range [HR_MIN_BPM, HR_MAX_BPM]. */
bool hr_vital_alert(const hr_sample_t *sample);

#endif /* HEARTRATE_H */
