/*
 * sensors.c
 *
 *  Created on: Mar 27, 2026
 *      Author: Sigmond
 */
#include "sensors.h"
#include "main.h"
#include "lps22df_reg.h"
#include "lsm6dsv80x_reg.h"
#include "iis2mdc_reg.h"
#include <string.h>
#include <stdio.h>
#include "state.h"
#include "telemetry.h"

#define MG_TO_MS2 0.009805f
//1 g=9.805 m/s^2; 1000 mG=1 g
//(1/1000)G*(9.805 m/s^2/1 G)--> 9.805/1000--> multiply by 0.009805
//123.0f

#define MILIDEGREE_TO_RADS 0.00001745f
//1/1000 degree=1 milidegree; pi/180 to convert from deg to rad
//multiply by pi/180,000

// SPI handles
extern SPI_HandleTypeDef hspi1; // Baro
extern SPI_HandleTypeDef hspi2; // IMU
extern SPI_HandleTypeDef hspi4; // Magnetometer

// ST drivers
stmdev_ctx_t lps22df_ctx;
stmdev_ctx_t lsm6dsv80x_ctx;
stmdev_ctx_t iis2mdc_ctx;

// Sensor readings in LSBs
uint32_t pres_raw;
int16_t accel_raw[3];
int16_t omega_raw[3];
int16_t mag_raw[3];

// New data available flags
uint8_t baro_ready = 0;
uint8_t imu_ready = 0;
uint8_t mag_ready = 0;

// SPI DMA buffers
// Size = 1 for register + N bytes
uint8_t baro_tx_buf[4], baro_rx_buf[4]; // XL, L, and H
uint8_t imu_tx_buf[13], imu_rx_buf[13]; // Gyro 2 bytes * 3 axes + Accel 2 bytes * 3 axes
uint8_t mag_tx_buf, mag_rx_buf[6]; // X, Y, Z where each channel is 16 bits = 2 bytes

void spi_nss(SPI_HandleTypeDef *handle, uint8_t level) {
	if (handle->Instance == SPI1) { // baro
		HAL_GPIO_WritePin(BARO_NSS_GPIO_Port, BARO_NSS_Pin, level);
	} else if (handle->Instance == SPI2) { // imu
		HAL_GPIO_WritePin(IMU_NSS_GPIO_Port, IMU_NSS_Pin, level);
	} else if (handle->Instance == SPI4) { // mag
		HAL_GPIO_WritePin(MAG_NSS_GPIO_Port, MAG_NSS_Pin, level);
	}
}

// ST driver platform functions
int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
	HAL_StatusTypeDef status = HAL_OK;

	spi_nss(handle, 0);
	status += HAL_SPI_Transmit(handle, &reg, 1, 1000);
	status += HAL_SPI_Transmit(handle, bufp, len, 1000);
	spi_nss(handle, 1);
	return status;
}

int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
	reg |= 0x80; // set MSB for read

	HAL_StatusTypeDef status = HAL_OK;

	spi_nss(handle, 0);
	status += HAL_SPI_Transmit(handle, &reg, 1, 1000);
	status += HAL_SPI_Receive(handle, bufp, len, 1000);
	spi_nss(handle, 1);
	return status;
}

void platform_delay(uint32_t millisec)
{
	HAL_Delay(millisec);
}

// DRDY handlers
// when data ready, init the DMA SPI transmission
void baro_int_drdy_handler()
{
	// initialize the SPI transmission
	memset(baro_tx_buf, 0, 4);
	baro_tx_buf[0] = LPS22DF_PRESS_OUT_XL | 0x80; // with MSB set for read
	spi_nss(lps22df_ctx.handle, 0); // assert baro CS
	HAL_SPI_TransmitReceive_DMA(lps22df_ctx.handle, baro_tx_buf, baro_rx_buf, 4);
}

void imu_int_drdy_handler() {
	memset(imu_tx_buf, 0, 13);
	imu_tx_buf[0] = LSM6DSV80X_OUTX_L_G | 0x80;
	spi_nss(lsm6dsv80x_ctx.handle, 0); // assert imu CS
	HAL_SPI_TransmitReceive_DMA(lsm6dsv80x_ctx.handle, imu_tx_buf, imu_rx_buf, 13);
}

void mag_int_drdy_handler() {
	mag_tx_buf = IIS2MDC_OUTX_L_REG | 0x80;
	spi_nss(iis2mdc_ctx.handle, 0); // assert mag CS
//	HAL_SPI_TransmitReceive_DMA(iis2mdc_ctx.handle, mag_tx_buf, mag_rx_buf, 7); // this doesn't work for half duplex SPI
	HAL_SPI_Transmit(&hspi4, &mag_tx_buf, 1, 1);
	HAL_SPI_Receive_DMA(&hspi4, mag_rx_buf, 6);
}

// SPI DMA done callbacks
// after data transfer process into units
void baro_spi_callback() {
	spi_nss(lps22df_ctx.handle, 1); // deassert baro CS
	pres_raw = (uint32_t)(((uint32_t)baro_rx_buf[3] << 24) | ((uint32_t)baro_rx_buf[2] << 16) | ((uint32_t)baro_rx_buf[1] << 8));
//	pres_hpa = lps22df_from_lsb_to_hPa(pres_raw);
//	state.pres_hpa = lps22df_from_lsb_to_hPa(pres_raw);
	global_state.pres_pa = lps22df_from_lsb_to_hPa(pres_raw) * 100.0f; // unit conversion, 1 hPa = 100 Pa;
	baro_ready = 1;
}

void imu_spi_callback() {
	spi_nss(lsm6dsv80x_ctx.handle, 1); // deassert imu CS
	// Pack registers into omega LSBs
	omega_raw[0] = (int16_t)(((int16_t)imu_rx_buf[2] << 8) | (int16_t)imu_rx_buf[1]);
	omega_raw[1] = (int16_t)(((int16_t)imu_rx_buf[4] << 8) | (int16_t)imu_rx_buf[3]);
	omega_raw[2] = (int16_t)(((int16_t)imu_rx_buf[6] << 8) | (int16_t)imu_rx_buf[5]);
	// Convert to scientific units
	global_state.omega_rads[0] = lsm6dsv80x_from_fs4000_to_mdps(omega_raw[0])*0.00001745f; //converted to rads
	global_state.omega_rads[1] = lsm6dsv80x_from_fs4000_to_mdps(omega_raw[1])*0.00001745f;
	global_state.omega_rads[2] = lsm6dsv80x_from_fs4000_to_mdps(omega_raw[2])*0.00001745f;
	// Pack registers into accel LSBs
	accel_raw[0] = (int16_t)(((int16_t)imu_rx_buf[8] << 8) | (int16_t)imu_rx_buf[7]);
	accel_raw[1] = (int16_t)(((int16_t)imu_rx_buf[10] << 8) | (int16_t)imu_rx_buf[9]);
	accel_raw[2] = (int16_t)(((int16_t)imu_rx_buf[12] << 8) | (int16_t)imu_rx_buf[11]);
	// Convert to scientific units
	global_state.accel_ms2[0] = lsm6dsv80x_from_fs16_to_mg(accel_raw[0])*0.009805f; // converted to m/s^2
	global_state.accel_ms2[1] = lsm6dsv80x_from_fs16_to_mg(accel_raw[1])*0.009805f;
	global_state.accel_ms2[2] = lsm6dsv80x_from_fs16_to_mg(accel_raw[2])*0.009805f;
	imu_ready = 1;
}


void mag_spi_callback() { // when DMA receive transfer is done (data is in memory of STM 32)
	spi_nss(iis2mdc_ctx.handle, 1); // deassert mag CS
	mag_raw[0] = (int16_t)(((int16_t)mag_rx_buf[1] << 8) | (int16_t)mag_rx_buf[0]);
	mag_raw[1] = (int16_t)(((int16_t)mag_rx_buf[3] << 8) | (int16_t)mag_rx_buf[2]);
	mag_raw[2] = (int16_t)(((int16_t)mag_rx_buf[5] << 8) | (int16_t)mag_rx_buf[4]);
	global_state.mag_mgauss[0] = iis2mdc_from_lsb_to_mgauss(mag_raw[0]);//mG
	global_state.mag_mgauss[1] = iis2mdc_from_lsb_to_mgauss(mag_raw[1]);
	global_state.mag_mgauss[2] = iis2mdc_from_lsb_to_mgauss(mag_raw[2]);
	mag_ready = 1;
}

void sensors_init() {
	HAL_Delay(100);
	baro_init();
	imu_init();
	mag_init();
}

void baro_init() {
	// Setup baro driver device context
	lps22df_ctx.write_reg = platform_write;
	lps22df_ctx.read_reg = platform_read;
	lps22df_ctx.mdelay = platform_delay;
	lps22df_ctx.handle = &hspi1;

	lps22df_pin_int_route_t int_route;
	lps22df_bus_mode_t bus_mode;
	lps22df_stat_t status;
	lps22df_id_t id;
	lps22df_md_t md;

	telemetry_log(LOG_LVL_DEBUG, "Baro init\r\n");

	lps22df_id_get(&lps22df_ctx, &id);
	if (id.whoami != LPS22DF_ID) {
		// printf("LPS22DF whoami failed: %u, expected %u\r\n", id.whoami, LPS22DF_ID);
		telemetry_log(LOG_LVL_ERROR, "LPS22DF whoami failed: %u, expected %u\r\n", id.whoami, LPS22DF_ID);
		while (1);
	}

	lps22df_init_set(&lps22df_ctx, LPS22DF_RESET);
	do {
		lps22df_status_get(&lps22df_ctx, &status);
	} while (status.sw_reset);

	bus_mode.filter = LPS22DF_FILTER_AUTO;
	bus_mode.interface = LPS22DF_SPI_4W;
	lps22df_bus_mode_set(&lps22df_ctx, &bus_mode);

	md.odr = LPS22DF_100Hz;
	md.avg = LPS22DF_64_AVG;
	md.lpf = LPS22DF_LPF_ODR_DIV_9;
	lps22df_mode_set(&lps22df_ctx, &md);

	// Enable DRDY routed on INT pin
	lps22df_pin_int_route_get(&lps22df_ctx, &int_route);
	int_route.drdy_pres = PROPERTY_ENABLE;
	lps22df_pin_int_route_set(&lps22df_ctx, &int_route);

//	printf("LPS22DF init complete\r\n");
	telemetry_log(LOG_LVL_DEBUG, "Baro init complete\r\n");
}

void imu_init() {
	// Setup IMU driver device context
	lsm6dsv80x_ctx.write_reg = platform_write;
	lsm6dsv80x_ctx.read_reg = platform_read;
	lsm6dsv80x_ctx.mdelay = platform_delay;
	lsm6dsv80x_ctx.handle = &hspi2;

	telemetry_log(LOG_LVL_DEBUG, "IMU init\r\n");

	/* Check device ID */
	uint8_t whoami;
	lsm6dsv80x_device_id_get(&lsm6dsv80x_ctx, &whoami);
	if (whoami != LSM6DSV80X_ID) {
//		printf("LSM6DSV80X whoami failed: %u, expected %u\r\n", whoami, LSM6DSV80X_ID);
		telemetry_log(LOG_LVL_ERROR, "LSM6DSV80X whoami failed: %u, expected %u\r\n", whoami, LSM6DSV80X_ID);
		while (1);
	}

	/* Perform device power-on-reset */
	lsm6dsv80x_sw_por(&lsm6dsv80x_ctx);

	/* Enable Block Data Update */
	lsm6dsv80x_block_data_update_set(&lsm6dsv80x_ctx, PROPERTY_ENABLE);

	/* Set Output Data Rate.
	* Selected data rate have to be equal or greater with respect
	* with MLC data rate.
	*/
	lsm6dsv80x_xl_setup(&lsm6dsv80x_ctx, LSM6DSV80X_ODR_AT_960Hz, LSM6DSV80X_XL_HIGH_PERFORMANCE_MD);
	lsm6dsv80x_hg_xl_data_rate_set(&lsm6dsv80x_ctx, LSM6DSV80X_HG_XL_ODR_AT_960Hz, 1);
	lsm6dsv80x_gy_setup(&lsm6dsv80x_ctx, LSM6DSV80X_ODR_AT_960Hz, LSM6DSV80X_GY_HIGH_PERFORMANCE_MD);

	/* Set full scale */
	lsm6dsv80x_xl_full_scale_set(&lsm6dsv80x_ctx, LSM6DSV80X_16g);
	lsm6dsv80x_hg_xl_full_scale_set(&lsm6dsv80x_ctx, LSM6DSV80X_80g);
	lsm6dsv80x_gy_full_scale_set(&lsm6dsv80x_ctx, LSM6DSV80X_4000dps);

	/* Configure filtering chain */
	lsm6dsv80x_filt_settling_mask_t filt_settling_mask;
	filt_settling_mask.drdy = PROPERTY_ENABLE;
	filt_settling_mask.irq_xl = PROPERTY_ENABLE;
	filt_settling_mask.irq_g = PROPERTY_ENABLE;
	lsm6dsv80x_filt_settling_mask_set(&lsm6dsv80x_ctx, filt_settling_mask);
	lsm6dsv80x_filt_gy_lp1_set(&lsm6dsv80x_ctx, PROPERTY_ENABLE);
	lsm6dsv80x_filt_gy_lp1_bandwidth_set(&lsm6dsv80x_ctx, LSM6DSV80X_GY_ULTRA_LIGHT);
	lsm6dsv80x_filt_xl_lp2_set(&lsm6dsv80x_ctx, PROPERTY_ENABLE);
	lsm6dsv80x_filt_xl_lp2_bandwidth_set(&lsm6dsv80x_ctx, LSM6DSV80X_XL_STRONG);

	// Setup DRDY interrupt
	lsm6dsv80x_pin_int_route_t pin_int_route;
	lsm6dsv80x_pin_int1_route_get(&lsm6dsv80x_ctx, &pin_int_route);
	pin_int_route.drdy_xl = PROPERTY_ENABLE;
	lsm6dsv80x_pin_int1_route_set(&lsm6dsv80x_ctx, &pin_int_route);

	telemetry_log(LOG_LVL_DEBUG, "IMU init complete\r\n");
}

void mag_init() {
	// Setup magnetometer device driver context
	iis2mdc_ctx.write_reg = platform_write;
	iis2mdc_ctx.read_reg = platform_read;
	iis2mdc_ctx.mdelay = platform_delay;
	iis2mdc_ctx.handle = &hspi4;

	telemetry_log(LOG_LVL_DEBUG, "Mag init\r\n");

	uint8_t whoami = 0;
	iis2mdc_device_id_get(&iis2mdc_ctx, &whoami);
	if (whoami != IIS2MDC_ID) {
		printf("IIS2MDC whoami failed: %u, expected %u\r\n", whoami, IIS2MDC_ID);
		telemetry_log(LOG_LVL_ERROR, "IIS2MDC whoami failed: %u, expected %u\r\n", whoami, IIS2MDC_ID);
		while (1);
	}

	/* Restore default configuration */
	iis2mdc_reset_set(&iis2mdc_ctx, PROPERTY_ENABLE);

	uint8_t rst;
	do {
	iis2mdc_reset_get(&iis2mdc_ctx, &rst);
	} while (rst);

	/* Enable Block Data Update */
	iis2mdc_block_data_update_set(&iis2mdc_ctx, PROPERTY_ENABLE);
	/* Set Output Data Rate */
	iis2mdc_data_rate_set(&iis2mdc_ctx, IIS2MDC_ODR_100Hz);
	/* Set / Reset sensor mode */
	iis2mdc_set_rst_mode_set(&iis2mdc_ctx, IIS2MDC_SENS_OFF_CANC_EVERY_ODR);
	/* Enable temperature compensation */
	iis2mdc_offset_temp_comp_set(&iis2mdc_ctx, PROPERTY_ENABLE);
	/* Set device in continuous mode */
	iis2mdc_operating_mode_set(&iis2mdc_ctx, IIS2MDC_CONTINUOUS_MODE);

	iis2mdc_drdy_on_pin_set(&iis2mdc_ctx, 1);

//	printf("IIS2MDC init complete\r\n");
	telemetry_log(LOG_LVL_DEBUG, "Mag init complete\r\n");
}

