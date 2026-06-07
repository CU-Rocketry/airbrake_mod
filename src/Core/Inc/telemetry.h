/*
 * telemetry.h
 *
 *  Created on: Apr 19, 2026
 *      Author: Sigmond
 */

#ifndef INC_TELEMETRY_H_
#define INC_TELEMETRY_H_

#include <cobs.h>
#include "state.h"
#include "packets.h"

typedef struct {
    UART_HandleTypeDef *handle;
    uint8_t tx_buf[256]; // TX buffer for non blocking. 256/4=64 words space rn

    uint8_t rx_buf[256]; // incoming bytes after decoding
	uint16_t rx_idx; // write index, needs to be uint16_t so it goes up to 256
	uint8_t rx_byte; // buffer for UART write
	uint8_t rx_ready; // indicates COBS packet RX ready
} telemetry_t;

typedef enum {
    LOG_LVL_DEBUG = 0,
    LOG_LVL_INFO = 1,
    LOG_LVL_WARNING = 2,
    LOG_LVL_ERROR = 3
} log_lvl_t;

void telemetry_init(telemetry_t *telemetry);
void telemetry_packet(const state_t *current_state);
void telemetry_send(const void *data, uint16_t length);
void telemetry_log(log_lvl_t lvl, const char *format, ...);
void telemetry_parse_rx(state_t *state);

#endif /* INC_TELEMETRY_H_ */
