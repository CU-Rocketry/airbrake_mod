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
