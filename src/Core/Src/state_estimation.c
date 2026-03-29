/*
 * state_estimation.c
 *
 *  Created on: Mar 27, 2026
 *      Author: Sigmond
 */

#include "arm_math.h"
#include "sensors.h"

void state_estimation(void) {

	arm_matrix_instance accel_b;
	status = get_accel_b(&accel_b);

}

arm_status get_accel_b(arm_matrix_instance *out)
{
	arm_status status;

	float accel[3];

	get_accel_ms2(&accel); // TODO

	static float32_t rot_imu_to_b_data[3*3] = {
	  0, 0, 1,
	  0, -1, 0,
	  1, 0, 0
	};

	arm_matrix_instance rot_imu_to_b, accel_row, accel_col, accel_b;

	arm_mat_init_f32(&rot_imu_to_b, 3, 3, rot_imu_to_b_data);
	arm_mat_init_f32(&accel_row, 1, 3, accel);

	status = arm_mat_trans_f32(&accel_row, &accel_col);
	status = arm_mat_mult_f32(&rot_imu_to_b, &accel_col, &accel_b);
}
