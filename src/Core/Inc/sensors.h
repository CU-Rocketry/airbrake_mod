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

#endif /* SRC_SENSORS_H_ */
