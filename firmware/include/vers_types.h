/*
 * =============================================================================
 * Vehicle Emergency Response System
 * Shared Types & Definitions
 * =============================================================================
 */

#ifndef VERS_TYPES_H
#define VERS_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/kernel.h>

/* ─────────────────────────────────────────────────────────────────────────── */
/*  System State Machine                                                       */
/* ─────────────────────────────────────────────────────────────────────────── */

typedef enum {
    SYS_STATE_INIT        = 0,  /**< Power-on, peripheral init               */
    SYS_STATE_NORMAL      = 1,  /**< Normal operation — polling sensors       */
    SYS_STATE_CRASH_ALERT = 2,  /**< Crash detected — gathering location/HR  */
    SYS_STATE_EMERGENCY   = 3,  /**< Alert sent — waiting for acknowledgement */
    SYS_STATE_FAULT       = 4,  /**< Unrecoverable hardware fault             */
} sys_state_t;

/* ─────────────────────────────────────────────────────────────────────────── */
/*  Accelerometer Data                                                         */
/* ─────────────────────────────────────────────────────────────────────────── */

typedef struct {
    float x_g;          /**< X-axis acceleration (g)                         */
    float y_g;          /**< Y-axis acceleration (g)                         */
    float z_g;          /**< Z-axis acceleration (g)                         */
    float magnitude_g;  /**< √(x²+y²+z²) — scalar magnitude                 */
    int64_t timestamp;  /**< k_uptime_get() at sample time (ms)              */
} accel_sample_t;

/* ─────────────────────────────────────────────────────────────────────────── */
/*  Heart-Rate / SpO₂ Data                                                     */
/* ─────────────────────────────────────────────────────────────────────────── */

typedef struct {
    uint16_t bpm;       /**< Beats per minute                                */
    uint8_t  spo2;      /**< Blood oxygen saturation (%)                     */
    bool     valid;     /**< Sensor reports valid contact                     */
    int64_t  timestamp;
} hr_sample_t;

/* ─────────────────────────────────────────────────────────────────────────── */
/*  GPS Fix Data                                                               */
/* ─────────────────────────────────────────────────────────────────────────── */

typedef struct {
    double   latitude;  /**< Decimal degrees (+ North)                       */
    double   longitude; /**< Decimal degrees (+ East)                        */
    float    speed_kmh; /**< Ground speed km/h                               */
    float    altitude;  /**< Altitude above MSL (metres)                     */
    uint8_t  satellites;/**< Number of satellites in view                    */
    bool     fix_valid; /**< True when NMEA reports a valid fix               */
    char     utc_time[10]; /**< "HH:MM:SS"                                  */
    char     utc_date[8];  /**< "DD/MM/YY"                                  */
    int64_t  timestamp;
} gps_fix_t;

/* ─────────────────────────────────────────────────────────────────────────── */
/*  Emergency Event (payload sent to cloud)                                    */
/* ─────────────────────────────────────────────────────────────────────────── */

typedef enum {
    ALERT_CRASH          = 0,
    ALERT_HR_LOW         = 1,
    ALERT_HR_HIGH        = 2,
    ALERT_NO_GPS         = 3,
    ALERT_ROLLOVER       = 4,
    ALERT_MANUAL         = 5,  /**< Driver pressed SOS button               */
} alert_type_t;

typedef struct {
    alert_type_t type;
    accel_sample_t accel;
    hr_sample_t    hr;
    gps_fix_t      gps;
    uint32_t       event_id;  /**< Monotonic counter                        */
    char           device_id[24]; /**< Hardware UUID                        */
} emergency_event_t;

/* ─────────────────────────────────────────────────────────────────────────── */
/*  Zbus channels                                                              */
/* ─────────────────────────────────────────────────────────────────────────── */

#include <zephyr/zbus/zbus.h>

ZBUS_CHAN_DECLARE(accel_chan);
ZBUS_CHAN_DECLARE(hr_chan);
ZBUS_CHAN_DECLARE(gps_chan);
ZBUS_CHAN_DECLARE(emergency_chan);

/* ─────────────────────────────────────────────────────────────────────────── */
/*  Configuration constants (overridable via CMake -D flags)                   */
/* ─────────────────────────────────────────────────────────────────────────── */

#ifndef CRASH_G_THRESHOLD
#define CRASH_G_THRESHOLD   30.0f   /* g-force crash threshold               */
#endif

#ifndef HR_MIN_BPM
#define HR_MIN_BPM          40U     /* below → bradycardia alert             */
#endif

#ifndef HR_MAX_BPM
#define HR_MAX_BPM          180U    /* above → tachycardia alert             */
#endif

#define SENSOR_POLL_MS      50      /* Thread A period (ms)                  */
#define DECISION_POLL_MS    100     /* Thread B period (ms)                  */
#define ACCEL_SMA_WINDOW    8       /* Simple moving-average window samples  */
#define ROLLOVER_THRESHOLD  45.0f   /* Degrees tilt for rollover detection   */

/* Thread priorities */
#define SENSOR_THREAD_PRIORITY      5
#define DECISION_THREAD_PRIORITY    4
#define COMM_THREAD_PRIORITY        6

/* Stack sizes */
#define SENSOR_STACK_SIZE   2048
#define DECISION_STACK_SIZE 2048
#define COMM_STACK_SIZE     4096

#endif /* VERS_TYPES_H */
