/*
 * telemetry.c
 *
 *  Created on: Jun 6, 2026
 *      Author: Sigmond
 */

#include "cobs.h"
#include <stdint.h>
#include "packets.h"
#include "stm32h7xx_hal.h"
#include <stdio.h> // vsnprintf
#include <stdarg.h> // va_list va_start va_end
#include "state.h" // state_t for telemetry packet
#include "telemetry.h"
#include <string.h> // memcpy
#include "main.h"

static telemetry_t *ctx;

void telemetry_init(telemetry_t *telemetry) {
	ctx = telemetry;
}

void telemetry_packet(const state_t *current_state) {
	telemetry_packet_t packet; // create packet to be sent

	packet.pkt_type = PKT_TYPE_TELEMETRY;

	// Time
	packet.t = current_state->t;

	// Launch detect
	packet.is_launched = current_state->is_launched;

	// Power
	packet.batt_v = current_state->batt_v;
	packet.batt_i = current_state->batt_i;

	// Sensors
	packet.pres_pa = current_state->pres_pa;
	memcpy(packet.accel_ms2, current_state->accel_ms2, sizeof(packet.accel_ms2));
	memcpy(packet.omega_rads, current_state->omega_rads, sizeof(packet.omega_rads));
	memcpy(packet.mag_mgauss, current_state->mag_mgauss, sizeof(packet.mag_mgauss));

	// Body frame sensors
	memcpy(packet.accel_b, current_state->accel_b, sizeof(packet.accel_b));
	memcpy(packet.omega_b, current_state->omega_b, sizeof(packet.omega_b));
	memcpy(packet.mag_b, current_state->mag_b, sizeof(packet.mag_b));

	// State estimation
	memcpy(packet.quat, current_state->quat, sizeof(packet.quat));
	memcpy(packet.accel_e, current_state->accel_e, sizeof(packet.accel_e));

	packet.p_ground = current_state->p_ground;
	packet.alt_agl = current_state->alt_agl;
	packet.vel_z = current_state->vel_z;

	// Control
	// TODO
	packet.output = current_state->output;

	// Servo
	packet.servo_cmd = current_state->servo_cmd;
	packet.servo_fdbk = current_state->servo_fdbk;

	telemetry_send(&packet, sizeof(telemetry_packet_t));
}

void telemetry_send(const void *data, uint16_t length) {
	if (ctx->handle->gState != HAL_UART_STATE_READY) { // if UART still transmitting something
		return; // don't send new data
	}

    uint16_t encoded_len = cobs_encode((const uint8_t*)data, length, ctx->tx_buf); // COBS encode

    ctx->tx_buf[encoded_len] = 0x00; // final character is null terminator

//    HAL_UART_Transmit_IT(ctx->handle, ctx->tx_buf, encoded_len + 1);
//    HAL_UART_Transmit(ctx->handle, ctx->tx_buf, encoded_len + 1, 1);
    HAL_UART_Transmit_DMA(ctx->handle, ctx->tx_buf, encoded_len + 1);
}

void telemetry_log(log_lvl_t lvl, const char *format, ...) {
    static log_packet_t log_pkt; // memory for packet
    memset(&log_pkt, 0, sizeof(log_packet_t)); // make it blank

    log_pkt.pkt_type = PKT_TYPE_LOG; // indicate that its a log packet
	log_pkt.lvl = (uint8_t)lvl; // convert level to uint8_t and apply it

    va_list args;
    va_start(args, format);
    vsnprintf(log_pkt.message, sizeof(log_pkt.message), format, args);
    va_end(args);

    // Exact length + 1 for pkt_type + 1 for null terminator
    uint16_t len = 1 + strlen(log_pkt.message) + 1;

    telemetry_send(&log_pkt, len);
}

// call when cobs_uart.rx_ready == 1
void telemetry_parse_rx(state_t *state) {
    uint8_t decoded_buf[256]; // this buffer should be good enough as long as neither HIL nor commands get too long
    uint16_t decoded_len = cobs_decode(ctx->rx_buf, ctx->rx_idx, decoded_buf);

    if (decoded_len > 0) {
        uint8_t pkt_type = decoded_buf[0];

        if (pkt_type == PKT_TYPE_CMD && decoded_len == sizeof(command_packet_t)) {
        	telemetry_log(LOG_LVL_DEBUG, "Received command packet\r\n");

			command_packet_t *cmd = (command_packet_t *)decoded_buf;

			// Enable/disable HIL mode
			if (cmd->use_hil_data != global_state.use_hil_data) {
				global_state.use_hil_data = cmd->use_hil_data;

				if (global_state.use_hil_data) {
					HAL_NVIC_DisableIRQ(BARO_INT_EXTI_IRQn);
					HAL_NVIC_DisableIRQ(IMU_INT1_EXTI_IRQn);
					HAL_NVIC_DisableIRQ(MAG_INT_EXTI_IRQn);
					telemetry_log(LOG_LVL_INFO, "HIL data enabled, sensors disabled\r\n");
				} else {
					HAL_NVIC_EnableIRQ(BARO_INT_EXTI_IRQn);
					HAL_NVIC_EnableIRQ(IMU_INT1_EXTI_IRQn);
					HAL_NVIC_EnableIRQ(MAG_INT_EXTI_IRQn);
					telemetry_log(LOG_LVL_INFO, "HIL data disabled, sensors enabled\r\n");
				}
			}

			// Map the rest of the commands
			global_state.mode_override_en = cmd->mode_en;
			global_state.mode_override = cmd->mode;

			global_state.servo_cmd_en = cmd->servo_cmd_en;
			global_state.servo_cmd_override = cmd->servo_cmd;

			if (cmd->launch_detect_en) {
				global_state.is_launched = 1;
			}
		}

		else if (pkt_type == PKT_TYPE_HIL_DATA && decoded_len == sizeof(hil_packet_t)) { // incoming packet has HIL data
			if (global_state.use_hil_data) { // if enabled in GUI then HIL data will be used in place of the sensors, which were disabled
				hil_packet_t *hil = (hil_packet_t *)decoded_buf;

				// overwrite state variables with HIL data
				global_state.pres_pa = hil->pres_pa;
				memcpy(global_state.accel_ms2, hil->accel_ms2, sizeof(global_state.accel_ms2));
				memcpy(global_state.omega_rads, hil->omega_rads, sizeof(global_state.omega_rads));
				memcpy(global_state.mag_mgauss, hil->mag_mgauss, sizeof(global_state.mag_mgauss));

				// Set sensor ready flags
				extern uint8_t imu_ready;
				extern uint8_t mag_ready;
				extern uint8_t baro_ready;
				imu_ready = 1;
				mag_ready = 1;
				baro_ready = 1;

			} else {
				telemetry_log(LOG_LVL_WARNING, "Received HIL data packet, but HIL mode disabled\r\n");
			}
		}
    }
}
