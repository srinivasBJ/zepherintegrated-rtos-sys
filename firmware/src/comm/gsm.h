/*
 * SIM800L GSM Modem Driver Header
 * Vehicle Emergency Response System — Zephyr RTOS
 */

#ifndef GSM_H
#define GSM_H

#include "vers_types.h"

/** Power-on and initialise SIM800L over UART2. */
int gsm_init(void);

/**
 * @brief Send SMS with emergency payload to the given number.
 *
 * @param number E.164 phone number string (e.g. "+11234567890").
 * @param event  Emergency event payload to encode.
 * @return 0 on success.
 */
int gsm_send_sms(const char *number, const emergency_event_t *event);

/**
 * @brief Get current signal quality (RSSI in dBm).
 * @return RSSI value, or INT_MIN on error.
 */
int gsm_signal_quality(void);

/** True if SIM800L reports network registration. */
bool gsm_is_registered(void);

#endif /* GSM_H */
