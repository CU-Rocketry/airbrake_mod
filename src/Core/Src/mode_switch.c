/*
 * mode_switch.c
 *
 *  Created on: Jun 16, 2026
 *      Author: Sigmond
 */

#include "mode_switch.h"

#define DEBOUNCE 100 // cycles where 1 cycle = 0.01s

// Private helper to read the physical switch
static uint8_t get_mode_switch_raw(void) {
	HAL_GPIO_WritePin(MODE_C_GPIO_Port, MODE_C_Pin, 1);

	uint8_t mode = HAL_GPIO_ReadPin(MODE_1_GPIO_Port, MODE_1_Pin);
	mode |= HAL_GPIO_ReadPin(MODE_2_GPIO_Port, MODE_2_Pin) << 1;
	mode |= HAL_GPIO_ReadPin(MODE_4_GPIO_Port, MODE_4_Pin) << 2;

	HAL_GPIO_WritePin(MODE_C_GPIO_Port, MODE_C_Pin, 0);

	return mode;
}

void mode_switch_init(mode_switch_t *ms) {
	ms->current = get_mode_switch_raw();
	ms->raw_prev = ms->current;
	ms->stable_cnt = 0;
}

// Updates mode switch driver internal state
// to be called in 100Hz loop
void mode_switch_update(mode_switch_t *ms) {
	uint8_t raw = get_mode_switch_raw();

	// Debounce the physical switch
	if (raw == ms->raw_prev) {
		if (ms->stable_cnt < DEBOUNCE) {
			ms->stable_cnt++;
		}
	} else {
		ms->stable_cnt = 0;
		ms->raw_prev = raw;
	}

	// Commit to the mode once stable
	if (ms->stable_cnt >= DEBOUNCE) {
		ms->current = raw;
	}
}

// Returns the current debounced mode
uint8_t mode_switch_get(mode_switch_t *ms) {
	return ms->current;
}
