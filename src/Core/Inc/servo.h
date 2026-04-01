/*
 * servo.h
 *
 *  Created on: Mar 31, 2026
 *      Author: Sigmond
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_

typedef struct {
	TIM_HandleTypeDef* handle;
} servo_t;

// duty is the on time of the pulse (min 500 max 2500) [us]
void servo_set(servo_t* servo, uint32_t duty) {

	if (duty < 500) {
		duty = 500;
	} else if (duty > 2500) {
		duty = 2500;
	}

	// update duty cycles
	__HAL_TIM_SET_COMPARE(servo->handle, TIM_CHANNEL_1, duty);
}

void servo_init(servo_t* servo) {
	servo_set(servo, 1520); // middle position

	// start PWM
	HAL_TIM_PWM_Start(servo->handle, TIM_CHANNEL_1);
}


#endif /* INC_SERVO_H_ */
