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

void telemetry_init(UART_HandleTypeDef *handle);

void telemetry_state(state_t *state);

#endif /* INC_TELEMETRY_H_ */
