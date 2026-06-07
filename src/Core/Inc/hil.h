/*
 * hil.h
 *
 *  Created on: Apr 30, 2026
 *      Author: Sigmond
 */

#ifndef INC_HIL_H_
#define INC_HIL_H_

#include <cobs.h>
#include <stdint.h>
#include <string.h>
#include "state.h"
#include "packets.h"

// Parse incoming HIL data and apply it to the state
static inline uint8_t hil_parse_rx(cobs_uart_t *port, state_t *current_state) {
	hil_packet_t rx_data;
	uint16_t decoded_len = cobs_decode(port->rx_buf, port->rx_idx, (uint8_t*)&rx_data);

	if (decoded_len == sizeof(hil_packet_t)) {
		current_state->pres_pa = rx_data.pres_pa;
		memcpy(current_state->accel_ms2, rx_data.accel_ms2, sizeof(rx_data.accel_ms2));
		memcpy(current_state->omega_rads, rx_data.omega_rads, sizeof(rx_data.omega_rads));
		memcpy(current_state->mag_mgauss, rx_data.mag_mgauss, sizeof(rx_data.mag_mgauss));
		return 1; // Successfully parsed
	}
	return 0; // Packet malformed or wrong size
}

// Send control output back to Simulink
//static inline void hil_send(cobs_uart_t *port, const state_t *current_state) {
//	hil_tx_packet_t tx_data;
//	tx_data.output = current_state->output;
//
//	cobs_uart_send(port, &tx_data, sizeof(hil_tx_packet_t));
//}

#endif /* INC_HIL_H_ */
