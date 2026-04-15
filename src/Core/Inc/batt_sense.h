/*
 * batt_sense.h
 *
 *  Created on: Apr 15, 2026
 *      Author: Sigmond
 */

#ifndef INC_BATT_SENSE_H_
#define INC_BATT_SENSE_H_

typedef struct {
	ADC_HandleTypeDef* adc_handle;
	uint16_t vsense_raw;
	uint16_t isense_raw;
} batt_sense_t;

void batt_sense_get(batt_sense_t* batt_sense, float* out_v, float* out_i) {
    HAL_ADC_Start(batt_sense->adc_handle);

    if (HAL_ADC_PollForConversion(batt_sense->adc_handle, 1) == HAL_OK) {
        batt_sense->vsense_raw = HAL_ADC_GetValue(batt_sense->adc_handle);
    }

    if (HAL_ADC_PollForConversion(batt_sense->adc_handle, 1) == HAL_OK) {
        batt_sense->isense_raw = HAL_ADC_GetValue(batt_sense->adc_handle);
    }

    HAL_ADC_Stop(batt_sense->adc_handle);

    const float VREF = 3.3f;
    const float ADC_COUNTS_MAX = (float)(0xFFFF); // 16 bit
    const float VSENSE_RATIO = (82.0f + 47.0f) / 47.0f;
    const float SHUNT_RESISTANCE = 0.002f; // [ohm]
    const float SHUNT_GAIN = 200.0f; // [V/V]

    // (Raw / 4095) * 3.3 * (129 / 47)
    *out_v = ((float)batt_sense->vsense_raw / ADC_COUNTS_MAX) * VREF * VSENSE_RATIO;

    // (Raw / 4095) * 3.3 / (Shunt * Gain)
    // 0.002 * 200 = 0.4
    *out_i = (((float)batt_sense->isense_raw / ADC_COUNTS_MAX) * VREF) / (SHUNT_RESISTANCE * SHUNT_GAIN);
}



#endif /* INC_BATT_SENSE_H_ */
