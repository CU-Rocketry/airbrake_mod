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

arm_status get_imu_b(arm_matrix_instance *out_accel, arm_matrix_instance *out_omega)
{
	arm_status status;

	float accel[3];

	get_accel_ms2(&accel); // TODO

	float omega[3];
	get_accel_rads(&omega); // TODO

	static float32_t rot_imu_to_b_data[3*3] = {
	  0, 0, 1,
	  0, -1, 0,
	  1, 0, 0
	};

	arm_matrix_instance rot_imu_to_b;
	arm_matrix_instance accel_row, accel_col, accel_b;

	arm_matrix_instance omega_row, omega_col, omega_b;

	arm_mat_init_f32(&rot_imu_to_b, 3, 3, rot_imu_to_b_data);
	arm_mat_init_f32(&accel_row, 1, 3, accel);
	arm_mat_init_f32(&omega_row, 1, 3, omega);


	status = arm_mat_trans_f32(&accel_row, &accel_col);
	status = arm_mat_mult_f32(&rot_imu_to_b, &accel_col, &accel_b);

	status = arm_mat_trans_f32(&omega_row, &omega_col);
	status = arm_mat_mult_f32(&rot_imu_to_b, &omega_col, &omega_b);

	arm_status get_mag_b(arm_matrix_instance *out_mag)
	{
		arm_status status;

		float mag[3];

		get_mag_mgauss(&mag); // TODO

		static float32_t mag_to_b_data[3*3] = {
		  0, 0, 1,
		  0, -1, 0,
		  -1, 0, 0
		};
		arm_matrix_instance rot_mag_to_b;
		arm_matrix_instance mag_row, mag_col, mag_b;



		arm_mat_init_f32(&mag_row, 1, 3, mag);


		status = arm_mat_trans_f32(&mag_row, &mag_col);
		status = arm_mat_mult_f32(&rot_mag_to_b, &mag_col, &mag_b);


}

