/*
 * mode_switch.h
 *
 *  Created on: Mar 31, 2026
 *      Author: Sigmond
 */

#ifndef INC_MODE_SWITCH_H_
#define INC_MODE_SWITCH_H_

#include "main.h"
#include <stdint.h>

typedef struct {
	uint8_t current;
	uint8_t raw_prev;
	uint8_t stable_cnt;
} mode_switch_t;

void mode_switch_init(mode_switch_t *ms);

// Updates mode switch driver internal state
// to be called in 100Hz loop
void mode_switch_update(mode_switch_t *ms);

// Returns the current debounced mode
uint8_t mode_switch_get(mode_switch_t *ms);

#endif /* INC_MODE_SWITCH_H_ */
