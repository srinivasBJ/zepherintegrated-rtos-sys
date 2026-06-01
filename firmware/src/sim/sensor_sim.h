/* POSIX Sensor Simulation Header */
#ifndef SENSOR_SIM_H
#define SENSOR_SIM_H

/** Start background thread that injects synthetic sensor data. */
void sensor_sim_start(void);

/** Inject a synthetic crash event (sets accel magnitude >> threshold). */
void sensor_sim_inject_crash(void);

/** Set simulated GPS position. */
void sensor_sim_set_gps(double lat, double lon, float speed_kmh);

/** Set simulated heart rate (BPM). */
void sensor_sim_set_hr(uint16_t bpm);

#endif /* SENSOR_SIM_H */
