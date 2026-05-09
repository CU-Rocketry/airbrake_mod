/*
 * telemetry.h
 *
 *  Created on: Apr 19, 2026
 *      Author: Sigmond
 */

#ifndef INC_TELEMETRY_H_
#define INC_TELEMETRY_H_

#include <stdint.h>
#include "packets.h"
#include "cobs_uart.h"
#include "stm32h7xx_hal.h"

void telemetry_packet_build(const state_t *current_state, telemetry_packet_t *packet) {
	// Time
	packet->t = current_state->t;

	// Launch detect
	packet->is_launched = current_state->is_launched;

	// Power
	packet->batt_v = current_state->batt_v;
	packet->batt_i = current_state->batt_i;

	// Sensors
	packet->pres_pa = current_state->pres_pa;

	// Body frame sensors
	memcpy(packet->accel_b, current_state->accel_b, sizeof(packet->accel_b));
	memcpy(packet->omega_b, current_state->omega_b, sizeof(packet->omega_b));
	memcpy(packet->mag_b, current_state->mag_b, sizeof(packet->mag_b));

	// State estimation
	memcpy(packet->quat, current_state->quat, sizeof(packet->quat));
	memcpy(packet->accel_e, current_state->accel_e, sizeof(packet->accel_e));

	packet->p_ground = current_state->p_ground;
	packet->alt_agl = current_state->alt_agl;
	packet->vel_z = current_state->vel_z;

	// Control
	// TODO
	packet->output = current_state->output;

	// Servo
	packet->servo_cmd = current_state->servo_cmd;
	packet->servo_fdbk = current_state->servo_fdbk;
}

void telemetry_send(cobs_uart_t *port, telemetry_packet_t *packet) {
	cobs_uart_send(port, packet, sizeof(telemetry_packet_t));
}

void telemetry_log(cobs_uart_t *port, const char *format, ...) {
    log_packet_t log_pkt;
    log_pkt.pkt_type = PKT_TYPE_LOG;

    va_list args;
    va_start(args, format);
    vsnprintf(log_pkt.message, sizeof(log_pkt.message), format, args);
    va_end(args);

    // Exact length + 1 for pkt_type + 1 for null terminator
    uint16_t len = 1 + strlen(log_pkt.message) + 1;
    cobs_uart_send(port, &log_pkt, len);
}

#endif /* INC_TELEMETRY_H_ */
