/*
 * state.h
 *
 *  Created on: Apr 14, 2026
 *      Author: Sigmond
 */

#ifndef INC_STATE_H_
#define INC_STATE_H_

#include <stdint.h>

#define GRAVITY 9.80665f

typedef struct {
	// Time
	uint32_t t; // [ms] since boot
	uint32_t launch_t; // [ms] launch detect
	uint32_t elapsed_t; // [ms] since launch detected

	// Launch detect
	uint32_t flags; // state_flag_t

	// Power
	float batt_v; // [V]
	float batt_i; // [A]

    // Sensors
    float accel_ms2[3];
    float omega_rads[3];
    float mag_mgauss[3];
//    float pres_hpa;
    float pres_pa;

    // Body frame sensors
    float accel_b[3]; // [m/s/s] in body frame. *proper acceleration
    float omega_b[3]; // [rad/s] in body frame
    float mag_b[3]; // [mgauss] in body frame

    // State estimation
    float quat[4]; // body to inertial rotation already I think
    float accel_e[3]; // [m/s/s] in inertial frame

    float p_ground; // [Pa]
    float h_agl_pres; // [m] from baro only
    float alt_agl; // [m] AGL with + up
    float vel_z; // [m] with + up

    // Control
    // TODO
    float predicted; // [m] predicted apogee
    float output; // 0 to 1 mapping to air brakes deployment range
    float p_contrib; // proportional term
    float i_contrib; // integral term

    // Servo
    float servo_cmd; // [deg]
    float servo_fdbk; // [deg]

    // Control panel overrides
    // enabled by flags
	uint8_t mode_override;
	float servo_cmd_override; // [deg]
} state_t;

extern state_t global_state; // global instance

// Flags
typedef enum {
	// Flight state
	FLAG_LAUNCHED = (1 << 0),
	FLAG_APOGEE = (1 << 1),
	// reserved bits thru 7

	// Control system
	FLAG_LOCKOUT_ACCEL = (1 << 8), // 1 if accel hasn't gone negative yet
	FLAG_LOCKOUT_ALTITUDE = (1 << 9), // 1 if altitude below threshold
	FLAG_LOCKOUT_ELAPSED = (1 << 10), // 1 if elapsed less than motor burn time
	FLAG_LOCKOUT_APOGEE = (1 << 11), // 1 if vertical velocity low
	FLAG_LOCKOUT_ATTITUDE = (1 << 12), // 1 if pointy end not up

	FLAG_CONTROL_ENABLED = (1 << 13), // 1 if no lockout is asserted
	// reserved thru 15

	// Peripherals and hardware status
	// reserved thru 23

	// Control panel overrides
	FLAG_USE_HIL_DATA = (1 << 24),
	FLAG_MODE_OVERRIDE_EN = (1 << 25),
	FLAG_SERVO_OVERRIDE_EN = (1 << 26)
	// to bit 31
} state_flag_t;

#define STATE_FLAG_SET(flag) (global_state.flags |= (flag)) // sets flag bit to 1
#define STATE_FLAG_CLEAR(flag) (global_state.flags &= ~(flag)) // sets flag bit to 0
#define STATE_FLAG_GET(flag) ((global_state.flags & (flag)) != 0) // gets flag bit
#define STATE_FLAG_WRITE(flag, value) ((value) ? STATE_FLAG_SET(flag) : STATE_FLAG_CLEAR(flag)) // writes flag bit to value

#endif /* INC_STATE_H_ */
