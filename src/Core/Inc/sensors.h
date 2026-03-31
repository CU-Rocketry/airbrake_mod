/*
 * sensors.h
 *
 *  Created on: Mar 27, 2026
 *      Author: Sigmond
 */

#ifndef SRC_SENSORS_H_
#define SRC_SENSORS_H_

float get_accel_ms2(float *out);
float get_omega_rads(float *out);
float get_mag_mgauss(float *out);
float get_pres_hpa(float *out);

void lps22df_int_drdy_handler();
void lsm6dsv80x_int_drdy_handler();
void iis2mdc_int_drdy_handler();

#endif /* SRC_SENSORS_H_ */
