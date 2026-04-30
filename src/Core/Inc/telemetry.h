/*
 * telemetry.h
 *
 *  Created on: Apr 19, 2026
 *      Author: Sigmond
 */

#ifndef INC_TELEMETRY_H_
#define INC_TELEMETRY_H_

#include <stdint.h>
#include "state.h"
#include "stm32h7xx_hal.h"

typedef struct {
    UART_HandleTypeDef *handle;
    uint8_t tx_buf[256]; // TX buffer for non blocking. 256/4=64 words space rn

    uint8_t rx_buf[256]; // incoming bytes after decoding
	uint16_t rx_idx; // write index, needs to be uint16_t so it goes up to 256
	uint8_t rx_byte; // buffer for UART write
	uint8_t rx_ready; // indicates COBS packet RX ready
} telemetry_t;

// see https://en.wikipedia.org/wiki/Consistent_Overhead_Byte_Stuffing
static uint16_t cobs_encode(const uint8_t * input, uint32_t length, uint8_t * output) {
	uint16_t r_idx = 0; // index in input
	uint16_t w_idx = 1; // index in output, starting at 1 for the first code
	uint16_t code_idx = 0; // where to store code when needed
	uint8_t code = 1; // distance to next zero byte

	while (r_idx < length) { // loop through input
		if (input[r_idx] == 0) { // if there is ever a zero
			output[code_idx] = code; // write accumulated distance to last code position
			code = 1; // reset distance back to 1 for next time
			code_idx = w_idx++; // save the new header position to be the write index, then move ahead of that index
			r_idx++; // move to next input

		} else { // if the byte is nonzero
			output[w_idx++] = input[r_idx++]; // copy it directly
			code++; // Increment the distance code

			if (code == 0xFF) { // if we reach max block size = 254 bytes
				output[code_idx] = code; // write code 0xFF to indicate wrapping
				code = 1; // reset distance back to 1
				code_idx = w_idx++; // save new header position like above
			}
		}
	}

	output[code_idx] = code; // write final distance
	return w_idx; // equals the length of encoded data
}

void telemetry_state(telemetry_t *ctx, state_t *state) {
	if (ctx->handle->gState != HAL_UART_STATE_READY) { // if UART still transmitting something
		return; // don't send new data
	}

    uint16_t encoded_len = cobs_encode((uint8_t*)state, sizeof(state_t), ctx->tx_buf); // COBS encode

    ctx->tx_buf[encoded_len] = 0x00; // final character is null terminator

//    HAL_UART_Transmit_IT(ctx->handle, ctx->tx_buf, encoded_len + 1);
//    HAL_UART_Transmit(ctx->handle, ctx->tx_buf, encoded_len + 1, 1);
    HAL_UART_Transmit_DMA(ctx->handle, ctx->tx_buf, encoded_len + 1);
}

typedef struct {
    float pres_pa;
    float accel_ms2[3];
    float omega_rads[3];
    float mag_mgauss[3];
} simulink_sensor_data_t;

typedef struct {
	float output;
} control_output_t;

// complement to cobs_encode
static uint16_t cobs_decode(const uint8_t *input, uint16_t length, uint8_t *output) {
	uint16_t r_idx = 0; // index in input
	uint16_t w_idx = 0; // index in output
	uint8_t code;
	uint8_t i;

	while (r_idx < length) {
		code = input[r_idx];

		if (r_idx + code > length && code != 1) {
			return 0; // packet malformed I guess
		}

		r_idx++;

		for (i = 1; i < code; i++) {
			output[w_idx++] = input[r_idx++];
		}

		if (code < 0xFF && r_idx < length) {
			output[w_idx++] = 0;
		}
	}
	return w_idx; // equals the length of decoded data
}

#endif /* INC_TELEMETRY_H_ */
