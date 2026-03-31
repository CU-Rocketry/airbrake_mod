/*
 * sensors.c
 *
 *  Created on: Mar 27, 2026
 *      Author: Sigmond
 */

#include "lps22df_reg.h"
#include "lsm6dsv80x_reg.h"
#include "iis2mdc_reg.h"

uint32_t pres_raw;
float pres_hpa;

float accel_ms2[3];
float omega_rads[3];
float mag_mgauss[3];

float get_accel_ms2(float *out) {
	out = accel_ms2;
}

float get_omega_rads(float *out) {
	out = omega_rads;
}

float get_mag_mgauss(float *out) {
	out = mag_mgauss;
}

float get_pres_hpa(float *out) {
	out = pres_hpa;
}

void lps22df_int_drdy_handler()
{
	// int_drdy = 1; // this was old flag to cause loop to read data

	// initialize the SPI transmission
	spi_tx_buf[0] = LPS22HB_PRESS_OUT_XL;
	spi_tx_buf[0] |= 0x80; // set MSB (0x80 = 1000 0000) to indicate read
	spi_tx_buf[1] = 0;
	spi_tx_buf[2] = 0;
	spi_tx_buf[3] = 0;

	HAL_GPIO_WritePin(BARO_NSS_GPIO_Port, BARO_NSS_Pin, 0); // assert chip select (active low)
	HAL_SPI_TransmitReceive_DMA(&hspi2, spi_tx_buf, spi_rx_buf, 4);
}

void lps22df_spi_callback()
{
	pres_raw = (uint32_t)(((uint32_t)spi_rx_buf[3] << 24) | ((uint32_t)spi_rx_buf[2] << 16) | ((uint32_t)spi_rx_buf[1] << 8));
	pres_hpa = lps22hb_from_lsb_to_hpa(pres_raw);
}

