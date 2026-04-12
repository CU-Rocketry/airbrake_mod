/*
 * servo.h
 *
 *  Created on: Mar 31, 2026
 *      Author: Sigmond
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_

typedef struct {
	// PWM
	TIM_HandleTypeDef* tim_handle;
	// Power enable
	GPIO_TypeDef* en_gpio_port;
	uint16_t en_pin;
	// Feedback
	ADC_HandleTypeDef* adc_handle;
	uint32_t fdbk_raw;
} servo_t;

// duty is the on time of the pulse (min 500 max 2500) [us]
void servo_set(servo_t* servo, uint32_t duty) {

	if (duty < 500) {
		duty = 500;
	} else if (duty > 2500) {
		duty = 2500;
	}

	// update duty cycles
	__HAL_TIM_SET_COMPARE(servo->tim_handle, TIM_CHANNEL_1, duty);
}

void servo_enable(servo_t* servo, uint8_t state) {
	HAL_GPIO_WritePin(servo->en_gpio_port, servo->en_pin, state);
}

void servo_init(servo_t* servo) {
	servo_enable(servo, 0);

	servo_set(servo, 1520); // middle position

	// start PWM
	HAL_TIM_PWM_Start(servo->tim_handle, TIM_CHANNEL_1);

	// start ADC
	HAL_ADC_Start_DMA(servo->adc_handle, &servo->fdbk_raw, 1);
}

float servo_get_angle(servo_t* servo) {
	// 300mV for 0 degrees to 3000mV for 180 degrees
	// needs to be mapped into the ADC 0 to 4096 range
	// 300mV = 372 counts and 3000mV = 3724 counts

	uint16_t raw = servo->fdbk_raw;

	// clamp values to prevent out-of-bounds angles due to noise
	if (raw < 372) {
		raw = 372;
	} else if (raw > 3724) {
		raw = 3724;
	}

	// map the range (raw - counts_min) * (angle_max / counts_range)
	float angle = (float)(raw - 372) * (180.0f / (3724.0f - 372.0f));

	return angle;
}

#endif /* INC_SERVO_H_ */
