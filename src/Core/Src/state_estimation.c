/*
 * state_estimation.c
 *
 *  Created on: Mar 27, 2026
 *      Author: Sigmond
 */

#include "arm_math.h"
#include "sensors.h"
#include "state_estimation.h"
#include "MadgwickAHRS.h"

uint16_t last_sample_time;
uint16_t sample_time;
float dt;

extern uint8_t imu_ready;
extern uint8_t mag_ready;
extern uint8_t baro_ready;

void state_estimation(void) {

	arm_matrix_instance_f32 accel_b, omega_b;
	get_imu_b(&accel_b, &omega_b);

	if (sample_time < last_sample_time) {
		dt = (0xFFFF - last_sample_time + sample_time); // [us]
	} else {
		dt = (sample_time - last_sample_time); // [us]
	}
	dt *= 0.000001f;


	if (imu_ready)
	{
		if (mag_ready) {
			MadgwickAHRSupdate(omega_b.pData[0], omega_b.pData[1], omega_b.pData[2],
					accel_b.pData[0], accel_b.pData[1], accel_b.pData[2],
					accel_b.pData[0], accel_b.pData[1], accel_b.pData[2],
					dt); // TODO make it mag

		} else {
			MadgwickAHRSupdateIMU(omega_b.pData[0], omega_b.pData[1], omega_b.pData[2],
								 accel_b.pData[0], accel_b.pData[1], accel_b.pData[2],
								 dt);
		}

		// EKF prediction
	}

	if (baro_ready) {
		// EKF correction
	}

}

arm_status get_imu_b(arm_matrix_instance_f32 *out_accel, arm_matrix_instance_f32 *out_omega)
{
	arm_status status;

	float accel[3];

	get_accel_ms2(accel); // TODO

	float omega[3];
	get_omega_rads(omega); // TODO

	static float32_t rot_imu_to_b_data[3*3] = {
	  0, 0, 1,
	  0, -1, 0,
	  1, 0, 0
	};

	arm_matrix_instance_f32 rot_imu_to_b;
	arm_matrix_instance_f32 accel_row, accel_col, accel_b;

	arm_matrix_instance_f32 omega_row, omega_col, omega_b;

	arm_mat_init_f32(&rot_imu_to_b, 3, 3, rot_imu_to_b_data);
	arm_mat_init_f32(&accel_row, 1, 3, accel);
	arm_mat_init_f32(&omega_row, 1, 3, omega);


	status = arm_mat_trans_f32(&accel_row, &accel_col);
	status = arm_mat_mult_f32(&rot_imu_to_b, &accel_col, &accel_b);

	status = arm_mat_trans_f32(&omega_row, &omega_col);
	status = arm_mat_mult_f32(&rot_imu_to_b, &omega_col, &omega_b);

	return status;
}

arm_status get_mag_b(arm_matrix_instance_f32 *out_mag)
{
		arm_status status;

		float mag[3];

		get_mag_mgauss(mag); // TODO

		static float32_t mag_to_b_data[3*3] = {
		  0, 0, 1,
		  0, -1, 0,
		  -1, 0, 0
		};
		arm_matrix_instance_f32 rot_mag_to_b;
		arm_matrix_instance_f32 mag_row, mag_col, mag_b;


		arm_mat_init_f32(&rot_mag_to_b, 3, 3, mag_to_b_data);
		arm_mat_init_f32(&mag_row, 1, 3, mag);


		status = arm_mat_trans_f32(&mag_row, &mag_col);
		status = arm_mat_mult_f32(&rot_mag_to_b, &mag_col, &mag_b);

		return status;
}

