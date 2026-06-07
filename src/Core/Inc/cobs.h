/*
 * cobs_uart.h
 *
 *  Created on: Apr 30, 2026
 *      Author: Sigmond
 */

#ifndef INC_COBS_H_
#define INC_COBS_H_

#include <stdint.h>

uint16_t cobs_decode(const uint8_t *input, uint16_t length, uint8_t *output);
uint16_t cobs_encode(const uint8_t *input, uint32_t length, uint8_t *output);

#endif /* INC_COBS_H_ */
