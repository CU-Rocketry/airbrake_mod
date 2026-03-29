/*
 * sensors.c
 *
 *  Created on: Mar 27, 2026
 *      Author: Sigmond
 */

#include "lps22df_reg.h"

uint32_t pres_raw;
double pres_hpa;

double accel[3];
double omega[3];
double mag[3];

double get_pres_hpa(double *out) {
	out = pres_hpa;
}
