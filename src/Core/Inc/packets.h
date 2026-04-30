/*
 * packets.h
 *
 *  Created on: Apr 30, 2026
 *      Author: Sigmond
 */

#ifndef INC_PACKETS_H_
#define INC_PACKETS_H_

#pragma pack(push, 1) // so that there is no padding

// Telemetry
typedef struct {
	// Time
	uint32_t t; // [ms] since boot

	// Launch detect
	uint32_t is_launched;

	// Power
	float batt_v; // [V]
	float batt_i; // [A]

    // Sensors
    float pres_pa;

    // Body frame sensors
    float accel_b[3]; // [m/s/s] in body frame. *proper acceleration
    float omega_b[3]; // [rad/s] in body frame
    float mag_b[3]; // [mgauss] in body frame

    // State estimation
    float quat[4]; // body to inertial rotation already I think
    float accel_e[3]; // [m/s/s] in inertial frame

    float p_ground; // [Pa]
    float alt_agl; // [m] AGL with + up
    float vel_z; // [m] with + up

    // Control
    // TODO
    float output; // 0 to 1 mapping to air brakes deployment range

    // Servo
    float servo_cmd; // [deg]
    float servo_fdbk; // [deg]
} telemetry_packet_t;

// Flash
// for 2 packets per 256 byte page we have max 128 bytes = 32 floats
typedef struct {
	uint32_t t; // [ms] since boot 4 bytes
	float batt_v; // [V] 8
	float batt_i; // [A] 12
	float accel_b[3]; // [m/s/s] in body frame. *proper acceleration 24
	float omega_b[3]; // [rad/s] in body frame 36
	float mag_b[3]; // [mgauss] in body frame 48
    float quat[4]; // body to inertial rotation already I think 64
    float accel_e[3]; // [m/s/s] in inertial frame 76
    float p_ground; // [Pa] 80
    float alt_agl; // [m] AGL with + up 84
    float vel_z; // [m] with + up 88
    float output; // 0 to 1 mapping to air brakes deployment range 92
    float servo_cmd; // [deg] 96
    float servo_fdbk; // [deg] 100
    // 28 bytes = 7 floats remaining
} flash_packet_t;

// HIL
typedef struct {
	float pres_pa;
	float accel_ms2[3];
	float omega_rads[3];
	float mag_mgauss[3];
} hil_rx_packet_t;

typedef struct {
	float output;
} hil_tx_packet_t;

#pragma pack(pop)

#endif /* INC_PACKETS_H_ */
