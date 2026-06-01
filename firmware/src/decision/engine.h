/*
 * Decision Engine Header — Crash State Machine
 * Vehicle Emergency Response System — Zephyr RTOS
 */

#ifndef ENGINE_H
#define ENGINE_H

#include "vers_types.h"

/** Initialise the decision engine and state machine. */
void engine_init(void);

/**
 * @brief Process latest sensor samples through the state machine.
 *
 * @param accel   Latest accelerometer sample.
 * @param hr      Latest heart-rate sample.
 * @param gps     Latest GPS fix.
 * @param event   Output emergency event (populated when returns true).
 * @return true if an emergency event should be dispatched.
 */
bool engine_process(const accel_sample_t *accel,
                    const hr_sample_t    *hr,
                    const gps_fix_t      *gps,
                    emergency_event_t    *event);

/** Return current system state. */
sys_state_t engine_get_state(void);

/** Force a manual SOS (e.g. panic button pressed). */
void engine_trigger_manual_sos(void);

#endif /* ENGINE_H */
