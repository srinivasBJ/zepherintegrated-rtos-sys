/*
 * PA1010D GPS Module Driver Header
 * Vehicle Emergency Response System — Zephyr RTOS
 */

#ifndef GPS_H
#define GPS_H

#include "vers_types.h"

/** Initialise UART for GPS (9600 baud NMEA). */
int gps_init(void);

/** Read latest GPS fix (blocks ≤ 1100ms for next $GPRMC sentence). */
int gps_read(gps_fix_t *fix);

/** Return the last known valid fix (non-blocking). */
const gps_fix_t *gps_last_fix(void);

#endif /* GPS_H */
