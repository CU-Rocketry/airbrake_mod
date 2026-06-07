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

typedef struct {
    uint8_t pkt_type; // always 0x02 for log
    uint8_t lvl; // log_lvl_t 0 to 3 for debug, info, warning, error
    char message[126]; // there's extra space even in just the first 254 bytes then
} log_packet_t;

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

// Command
typedef struct {
    uint8_t pkt_type; // always 0x03 for cmd

    uint8_t mode_en; // requests mode change (substitute for rotating selector)
    uint8_t mode; // mode to change to

    uint8_t launch_detect_en; // set to 1 to trigger launch detect

    uint8_t servo_cmd_en;
    float servo_cmd; // [deg]

    uint8_t use_hil_data; // set to 1 to enable HIL mode else 0 sources data from real sensors
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
