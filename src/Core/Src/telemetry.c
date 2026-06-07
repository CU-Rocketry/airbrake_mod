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

	// Setup struct fields for RX
	ctx->rx_buf_read_idx = 0;
	ctx->rx_pkt_len = 0;
	HAL_UART_Receive_DMA(ctx->handle, ctx->rx_buf, TELEMETRY_RX_BUF_SIZE);
}

void telemetry_packet(const state_t *current_state) {
	telemetry_packet_t packet; // create packet to be sent

	packet.pkt_type = PKT_TYPE_TELEMETRY;

	// Time
	packet.t = current_state->t;

	// All bit flags
	packet.flags = current_state->flags;

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

static void telemetry_handle_rx_packet(uint8_t *decoded_buf, uint16_t decoded_len, state_t *state) {
    uint8_t pkt_type = decoded_buf[0];

    if (pkt_type == PKT_TYPE_CMD && decoded_len == sizeof(command_packet_t)) {
        // telemetry_log(LOG_LVL_DEBUG, "Received command packet\r\n");

        command_packet_t *cmd = (command_packet_t *)decoded_buf;

        // Enable or disable HIL if needed
        if (cmd->use_hil_data != STATE_FLAG_GET(FLAG_USE_HIL_DATA)) { // if use_hil_data has changed
            STATE_FLAG_WRITE(FLAG_USE_HIL_DATA, cmd->use_hil_data); // write new value

            if (STATE_FLAG_GET(FLAG_USE_HIL_DATA)) { // if HIL data enabled
            	// Disable DRDY interrupts for all sensors
                HAL_NVIC_DisableIRQ(BARO_INT_EXTI_IRQn);
                HAL_NVIC_DisableIRQ(IMU_INT1_EXTI_IRQn);
                HAL_NVIC_DisableIRQ(MAG_INT_EXTI_IRQn);
                telemetry_log(LOG_LVL_INFO, "HIL data enabled, sensors disabled\r\n");
            } else { // HIL data disabled
            	// Enable DRDY interrupts for all sensors
                HAL_NVIC_EnableIRQ(BARO_INT_EXTI_IRQn);
                HAL_NVIC_EnableIRQ(IMU_INT1_EXTI_IRQn);
                HAL_NVIC_EnableIRQ(MAG_INT_EXTI_IRQn);
                telemetry_log(LOG_LVL_INFO, "HIL data disabled, sensors enabled\r\n");
            }
        }

        // Update mode override
        STATE_FLAG_WRITE(FLAG_MODE_OVERRIDE_EN, cmd->mode_en);
        global_state.mode_override = cmd->mode;

        // Update servo angle command
        STATE_FLAG_WRITE(FLAG_SERVO_OVERRIDE_EN, cmd->servo_cmd_en);
        global_state.servo_cmd_override = cmd->servo_cmd;

        if (cmd->launch_detect_en) {
            STATE_FLAG_SET(FLAG_LAUNCHED);
        }
    }
    else if (pkt_type == PKT_TYPE_HIL_DATA && decoded_len == sizeof(hil_packet_t)) {
        if (STATE_FLAG_GET(FLAG_USE_HIL_DATA)) {
            hil_packet_t *hil = (hil_packet_t *)decoded_buf;

            global_state.pres_pa = hil->pres_pa;
            memcpy(global_state.accel_ms2, hil->accel_ms2, sizeof(global_state.accel_ms2));
            memcpy(global_state.omega_rads, hil->omega_rads, sizeof(global_state.omega_rads));
            memcpy(global_state.mag_mgauss, hil->mag_mgauss, sizeof(global_state.mag_mgauss));

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

void telemetry_rx_poll(state_t *state) {
    uint16_t dma_write_idx = TELEMETRY_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(ctx->handle->hdmarx); // get DMA write pointer from hardware

    while (ctx->rx_buf_read_idx != dma_write_idx) { // loop until read pointer matches DMA write pointer

        uint8_t byte = ctx->rx_buf[ctx->rx_buf_read_idx]; // extract current byte
        ctx->rx_buf_read_idx = (ctx->rx_buf_read_idx + 1) % TELEMETRY_RX_BUF_SIZE; // increment read pointer and wrap if needed

        if (byte != 0x00) { // not delimiter, so normal byte in packet
        	if (ctx->rx_pkt_len < sizeof(ctx->rx_pkt_buf)) { // if packet doesn't yet fill buffer
				ctx->rx_pkt_buf[ctx->rx_pkt_len++] = byte; // write byte into packet buffer

			} else { // buffer full
				ctx->rx_pkt_len = 0; // overflow wraps, resetting packet length to zero and everything is messed up
				telemetry_log(LOG_LVL_ERROR, "Telemetry RX packet size overflow\r\n");
			}
        } else { // byte is COBS delimiter, handle end of packet
        	if (ctx->rx_pkt_len > 0) { // if there was data before the delimiter

				// Decode the packet
				uint8_t decoded_buf[256];
				uint16_t decoded_len = cobs_decode(ctx->rx_pkt_buf, ctx->rx_pkt_len, decoded_buf);

				if (decoded_len > 0) {
					telemetry_handle_rx_packet(decoded_buf, decoded_len, state);
				}

				ctx->rx_pkt_len = 0; // Reset length for next packet assembly
			} else {
				telemetry_log(LOG_LVL_ERROR, "Received COBS delimiter not preceded by packet\r\n");
			}
        }
    }
}
