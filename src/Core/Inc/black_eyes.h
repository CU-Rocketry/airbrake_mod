/*
 * black_eyes.h
 *
 *  Created on: Mar 31, 2026
 *      Author: Sigmond
 */

#ifndef INC_BLACK_EYES_H_
#define INC_BLACK_EYES_H_

#include "main.h"

void black_eye_set(uint8_t eye, uint8_t state) {
	if (eye == 0) {
		HAL_GPIO_WritePin(EYE_0_GPIO_Port, EYE_0_Pin, state);
	} else if (eye == 1) {
		HAL_GPIO_WritePin(EYE_1_GPIO_Port, EYE_1_Pin, state);
	}
}

#endif /* INC_BLACK_EYES_H_ */
