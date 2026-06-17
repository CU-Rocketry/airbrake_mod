/*
 * timing.h
 *
 *  Created on: Jun 13, 2026
 *      Author: Sigmond
 */

#ifndef INC_TIMING_H_
#define INC_TIMING_H_

#include "stm32h7xx_hal.h"

extern TIM_HandleTypeDef htim2; // hardcoded for now

// Calculates elapsed time in seconds and updates the tracker
static inline float get_dt(uint32_t *last_time, float fallback) {
    uint32_t now = __HAL_TIM_GET_COUNTER(&htim2);
    float dt = fallback;

    if (*last_time != 0) { // allows for initialization without a really big dt jump
        dt = (float)(now - *last_time) * 0.000001f; // now - last_time will handle wrapping fine as uints. it gets converted to float after then to microseconds
    }

    *last_time = now;
    return dt;
}

#endif /* INC_TIMING_H_ */
