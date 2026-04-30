/*
 * servo.h
 *
 *  Created on: Mar 31, 2026
 *      Author: Sigmond
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#include "state.h"

typedef struct {
	// PWM
	TIM_HandleTypeDef* tim_handle;
	// Power enable
	GPIO_TypeDef* en_gpio_port;
	uint16_t en_pin;
	// Feedback
	ADC_HandleTypeDef* adc_handle;
	volatile uint16_t* dma_buf;
	// Air brakes endpoints
	uint32_t duty_retracted;
	uint32_t duty_extended;
} servo_t;

// duty is the on time of the pulse (min 500 max 2500) [us]
void servo_set_duty(servo_t* servo, uint32_t duty) {

	global_state.servo_cmd = (float)(duty - 500) * (180.0f / (2500.0f - 500.0f)); // calculate servo cmd angle before clamping duty to expose issues

	// clamp duty to servo valid input
	if (duty < 500) {
		duty = 500;
	} else if (duty > 2500) {
		duty = 2500;
	}

	__HAL_TIM_SET_COMPARE(servo->tim_handle, TIM_CHANNEL_1, duty); // update PWM duty cycle
}

void servo_set_deployment(servo_t* servo, float deployment) {
	uint32_t duty = deployment * (float)(servo->duty_extended - servo->duty_retracted) + (float)(servo->duty_retracted);
	servo_set_duty(servo, duty);
}

// sig's mistake fixing
void servo_enable(servo_t* servo, uint8_t state) {
	if (state == 1) { // state 1
		// need to ramp up voltage
		for (uint32_t i = 0; i < 5; i++) {
			servo->en_gpio_port->BSRR = servo->en_pin; // turn on
			// about 100 ns here
			servo->en_gpio_port->BSRR = (uint32_t)servo->en_pin << 16U; // turn back off

			// delay to slowly discharge the capacitance
			for (volatile uint32_t d = 0; d < 5000; d++) {
				__NOP();
			}
		}

		servo->en_gpio_port->BSRR = servo->en_pin; // turn on

	} else { // state is 0
		servo->en_gpio_port->BSRR = (uint32_t)servo->en_pin << 16U; // turn off
	}
}

void servo_init(servo_t* servo) {
	servo_enable(servo, 0);

	servo_set_duty(servo, 1520); // middle position

	// start PWM
	HAL_TIM_PWM_Start(servo->tim_handle, TIM_CHANNEL_1);

	// start continuous ADC
	HAL_ADC_Start_DMA(servo->adc_handle, (uint32_t*)servo->dma_buf, 1);
}

float servo_get_angle(servo_t* servo) {
	// 300mV for 0 degrees to 3000mV for 180 degrees
	// needs to be mapped into the ADC 0 to 4096 range
	// 300mV = 372 counts and 3000mV = 3724 counts

//	HAL_ADC_Start(servo->adc_handle);
//	HAL_ADC_PollForConversion(servo->adc_handle, 1);
//	uint32_t raw = HAL_ADC_GetValue(servo->adc_handle);

//	SCB_InvalidateDCache_by_Addr((uint32_t*)servo->dma_buf, 2);
	uint32_t raw = servo->dma_buf[0];

	// clamp to known voltage range (300 to 3000 mV)
	if (raw < 1130) {
		raw = 1130;
	} else if (raw > 3195) {
		raw = 3195;
	}

	// map the range (raw - counts_min) * (angle_max / counts_range)
	global_state.servo_fdbk = 180.0f - (float)(raw - 1130) * (180.0f / (3195.0f - 1130.0f));

	return global_state.servo_fdbk;
}

#endif /* INC_SERVO_H_ */
