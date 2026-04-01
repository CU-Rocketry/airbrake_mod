/*
 * mode_switch.h
 *
 *  Created on: Mar 31, 2026
 *      Author: Sigmond
 */

#ifndef INC_MODE_SWITCH_H_
#define INC_MODE_SWITCH_H_

#include "main.h"
#include <stdio.h>

uint8_t get_mode_switch(void) {
	HAL_GPIO_WritePin(MODE_C_GPIO_Port, MODE_C_Pin, 1);

	uint8_t mode = HAL_GPIO_ReadPin(MODE_1_GPIO_Port, MODE_1_Pin);
	mode |= HAL_GPIO_ReadPin(MODE_2_GPIO_Port, MODE_2_Pin) << 1;
	mode |= HAL_GPIO_ReadPin(MODE_4_GPIO_Port, MODE_4_Pin) << 2;

	HAL_GPIO_WritePin(MODE_C_GPIO_Port, MODE_C_Pin, 0);

	return mode;
}

#endif /* INC_MODE_SWITCH_H_ */
