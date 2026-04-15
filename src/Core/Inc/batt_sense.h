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
	volatile uint16_t dma_buf[2];
} batt_sense_t;

void batt_sense_get(batt_sense_t* batt_sense, float* out_v, float* out_i) {
    uint16_t vsense_raw = batt_sense->dma_buf[0];
    uint16_t isense_raw = batt_sense->dma_buf[1];

    const float VREF = 3.3f;
    const float ADC_COUNTS_MAX = (float)(0xFFFF); // 16 bit
    const float VSENSE_RATIO = (82.0f + 47.0f) / 47.0f;
    const float SHUNT_RESISTANCE = 0.002f; // [ohm]
    const float SHUNT_GAIN = 200.0f; // [V/V]

    // (Raw / 4095) * 3.3 * (129 / 47)
    *out_v = ((float)vsense_raw / ADC_COUNTS_MAX) * VREF * VSENSE_RATIO;

    // (Raw / 4095) * 3.3 / (Shunt * Gain)
    // 0.002 * 200 = 0.4
    *out_i = (((float)isense_raw / ADC_COUNTS_MAX) * VREF) / (SHUNT_RESISTANCE * SHUNT_GAIN);
}

void batt_sense_init(batt_sense_t* batt_sense) {
    HAL_ADC_Start_DMA(batt_sense->adc_handle, (uint32_t*)batt_sense->dma_buf, 2); // starts continuous conversion
}

#endif /* INC_BATT_SENSE_H_ */
