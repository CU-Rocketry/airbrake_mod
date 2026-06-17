/*
 * packets.h
 *
 *  Created on: Apr 30, 2026
 *      Author: Sigmond
 */

#ifndef INC_PACKETS_H_
#define INC_PACKETS_H_

// Telemetry packet types
typedef enum {
	PKT_TYPE_TELEMETRY = 0x01, // telemetry data stream
	PKT_TYPE_LOG = 0x02, // log appears on telemetry
	PKT_TYPE_CMD = 0x03, // manual/override commands from control panel software

	PKT_TYPE_HIL_DATA  = 0x10, // simulink to air brakes sensor data (return is in telemetry)
} packet_type_t;

#pragma pack(push, 1) // so that there is no padding

// Telemetry
typedef struct {
	uint8_t pkt_type; // always 0x01 for telemetry

	// Time
	uint32_t t; // [ms] since boot

	// Bit flags
	uint32_t flags;

	// Power
	float batt_v; // [V]
	float batt_i; // [A]

    // Sensors
    float pres_pa;
    float accel_ms2[3]; // [m/s/s] in sensor frame
    float omega_rads[3]; // [rad/s] in sensor frame
    float mag_mgauss[3]; // [mgauss] in sensor frame

    // Body frame sensors
    float accel_b[3]; // [m/s/s] in body frame. *proper acceleration
    float omega_b[3]; // [rad/s] in body frame
    float mag_b[3]; // [mgauss] in body frame

    // State estimation
    float quat[4]; // body to inertial rotation already I think
    float accel_e[3]; // [m/s/s] in inertial frame

    float p_ground; // [Pa]
    float h_agl_pres; // [m]
    float alt_agl; // [m] AGL with + up
    float vel_z; // [m] with + up

    // Control
    // TODO
    float predicted;
    float output; // 0 to 1 mapping to air brakes deployment range
    float p_contrib; // proportional term
	float i_contrib; // integral term

    // Servo
    float servo_cmd; // [deg]
    float servo_fdbk; // [deg]
} telemetry_packet_t;

typedef struct {
    uint8_t pkt_type; // always 0x02 for log
    uint8_t lvl; // log_lvl_t 0 to 3 for debug, info, warning, error
    char message[126]; // there's extra space even in just the first 254 bytes then
} log_packet_t;

// Flash
// 1 frame per page = 256 bytes = 64 floats
typedef struct {
	uint32_t t; // [ms] since boot 4 bytes
	uint32_t elapsed_t; // [ms] since launch detected 8

	uint32_t flags; // 12

	float batt_v; // [V] 16
	float batt_i; // [A] 20

    // Sensors
    float accel_ms2[3]; // 32
    float omega_rads[3]; // 44
    float mag_mgauss[3]; // 56
//    float pres_hpa;
    float pres_pa; // 60

	float accel_b[3]; // [m/s/s] in body frame. *proper acceleration 72
	float omega_b[3]; // [rad/s] in body frame 84
	float mag_b[3]; // [mgauss] in body frame 96

    float quat[4]; // body to inertial rotation already I think 112
    float accel_e[3]; // [m/s/s] in inertial frame 124

    float p_ground; // [Pa] 128
    float alt_agl; // [m] AGL with + up 132
    float vel_z; // [m] with + up 136

    float predicted; // [m] 140
    float output; // 0 to 1 mapping to air brakes deployment range 144
    float p_contrib; // proportional term 148
	float i_contrib; // integral term 152

    float servo_cmd; // [deg] 156
    float servo_fdbk; // [deg] 160
    // 96 bytes = 24 floats remaining
} flash_packet_t;

// Command
typedef struct {
    uint8_t pkt_type; // always 0x03 for cmd

    // mask of flags to set and clear
    uint32_t state_flags_set;
    uint32_t state_flags_clear;

    // variables maybe paid attention to based on flags
    uint8_t mode; // mode to change to
    float servo_cmd; // [deg]
} command_packet_t;

// HIL
typedef struct {
	uint8_t pkt_type; // always 0x10 for HIL data

	float pres_pa;
	float accel_ms2[3];
	float omega_rads[3];
	float mag_mgauss[3];
} hil_packet_t;

#pragma pack(pop)

#endif /* INC_PACKETS_H_ */
