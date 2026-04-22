/*
 * state.h
 *
 *  Created on: Apr 14, 2026
 *      Author: Sigmond
 */

#ifndef INC_STATE_H_
#define INC_STATE_H_

#define GRAVITY 9.80665f

typedef struct {
	// Time
	uint32_t t; // [ms] since boot

	// Power
	float batt_v; // [V]
	float batt_i; // [A]

    // Sensors
    float accel_ms2[3];
    float omega_rads[3];
    float mag_mgauss[3];
    float pres_hpa;

    // Body frame sensors
    float accel_b[3]; // [m/s/s] in body frame. *proper acceleration
    float omega_b[3]; // [rad/s] in body frame
    float mag_b[3]; // [mgauss] in body frame

    // State estimation
    float quat[4]; // body to inertial rotation already I think
    float accel_e[3]; // [m/s/s] in inertial frame
    float alt_agl; // [m] AGL with + up
    float vel_z; // [m] with + up

    // Control
    // TODO

    // Servo
    float servo_cmd; // [deg]
    float servo_fdbk; // [deg]
} state_t;

extern state_t state; // global instance

#endif /* INC_STATE_H_ */
