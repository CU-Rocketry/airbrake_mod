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

#define TELEMETRY_RX_BUF_SIZE 1024 // four packets worth basically

typedef struct {
    UART_HandleTypeDef *handle;
    uint8_t tx_buf[256]; // TX buffer for non blocking. 256/4=64 words space rn

    uint8_t rx_buf[TELEMETRY_RX_BUF_SIZE]; // RX DMA buffer
    uint16_t rx_buf_read_idx; // how far we are in processing the buffer
    // write ptr is calculated from DMA hardware

    uint8_t rx_pkt_buf[256]; // Packet assembly buffer
    uint8_t rx_pkt_len; // where the next byte goes when assembling packet; equivalent to length of currently assmebled packet
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
void telemetry_rx_poll(state_t *state);

#endif /* INC_TELEMETRY_H_ */
