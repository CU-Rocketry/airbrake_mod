/*
 * cobs_uart.c
 *
 *  Created on: Jun 6, 2026
 *      Author: Sigmond
 */

#include "cobs.h"

// see https://en.wikipedia.org/wiki/Consistent_Overhead_Byte_Stuffing
uint16_t cobs_encode(const uint8_t *input, uint32_t length, uint8_t *output) {
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
// complement to cobs_encode for received data
uint16_t cobs_decode(const uint8_t *input, uint16_t length, uint8_t *output) {
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
