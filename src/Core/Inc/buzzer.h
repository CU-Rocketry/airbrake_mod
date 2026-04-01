/*
 * buzzer.h
 *
 *  Created on: Mar 31, 2026
 *      Author: Sigmond
 */

#ifndef INC_BUZZER_H_
#define INC_BUZZER_H_

typedef struct {
	TIM_HandleTypeDef* handle;
} buzzer_t;

void buzzer_set(buzzer_t* buzzer, uint8_t status) {
	uint32_t ccr;

	if (status == 0) {
		ccr = 0;
	}

	if (status == 1) {
		ccr = 8000;
	}

	// update duty cycles
	__HAL_TIM_SET_COMPARE(buzzer->handle, TIM_CHANNEL_1, ccr);
}

void buzzer_init(buzzer_t* buzzer) {
	buzzer_set(buzzer, 0); // turn channel off

	// start PWM
	HAL_TIM_PWM_Start(buzzer->handle, TIM_CHANNEL_1);
}

#endif /* INC_BUZZER_H_ */
