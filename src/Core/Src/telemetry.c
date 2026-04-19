/*
 * telemetry.c
 *
 *  Created on: Apr 19, 2026
 *      Author: Sigmond
 */

#include "telemetry.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

static UART_HandleTypeDef *telemetry_uart;

typedef struct {
    uint32_t type; // 1 for state, 2 for log
    state_t state;
    uint32_t checksum;
} telemetry_state_packet_t;

// Standard COBS encode function
// Returns the length of the encoded data
static size_t cobs_encode(const uint8_t *input, size_t length, uint8_t *output) {
    size_t read_index = 0;
    size_t write_index = 1;
    size_t code_index = 0;
    uint8_t code = 1;

    while (read_index < length) {
        if (input[read_index] == 0) {
            output[code_index] = code;
            code = 1;
            code_index = write_index++;
            read_index++;
        } else {
            output[write_index++] = input[read_index++];
            code++;
            if (code == 0xFF) {
                output[code_index] = code;
                code = 1;
                code_index = write_index++;
            }
        }
    }
    output[code_index] = code;
    return write_index;
}

// Simple 32-bit XOR checksum aligned to 4 bytes
static uint32_t calculate_checksum32(uint32_t *data, size_t word_count) {
    uint32_t sum = 0;
    for (size_t i = 0; i < word_count; i++) {
        sum ^= data[i];
    }
    return sum;
}

void telemetry_init(UART_HandleTypeDef *handle) {
    telemetry_uart = handle;
}

void telemetry_state(state_t *current_state) {
    telemetry_state_packet_t packet;
    packet.type = 1;
    memcpy(&packet.state, current_state, sizeof(state_t));

    // Checksum covers the type and the state payload
    size_t words_to_check = (sizeof(uint32_t) + sizeof(state_t)) / 4;
    packet.checksum = calculate_checksum32((uint32_t*)&packet, words_to_check);

    // COBS encoding buffer needs to be slightly larger than the raw packet
    // Max overhead is 1 byte per 254 bytes of data, plus the trailing 0x00 delimiter
    uint8_t encode_buffer[sizeof(telemetry_state_packet_t) + 4];

    size_t encoded_len = cobs_encode((uint8_t*)&packet, sizeof(telemetry_state_packet_t), encode_buffer);

    // Append the COBS delimiter
    encode_buffer[encoded_len] = 0x00; // final character is null terminator

    HAL_UART_Transmit(telemetry_uart, encode_buffer, encoded_len + 1, HAL_MAX_DELAY); // TODO its blocking
}
