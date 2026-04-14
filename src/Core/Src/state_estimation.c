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

	arm_matrix_instance_f32 mag_b;
	get_mag_b(&mag_b);

//	if (sample_time < last_sample_time) {
//		dt = (0xFFFF - last_sample_time + sample_time); // [us]
//	} else {
//		dt = (sample_time - last_sample_time); // [us]
//	}
//	dt *= 0.000001f;
	dt = 0.002f; // 500 Hz, hardcoded for now


	if (imu_ready)
	{
		if (mag_ready) {
			MadgwickAHRSupdate(omega_b.pData[0], omega_b.pData[1], omega_b.pData[2],
					accel_b.pData[0], accel_b.pData[1], accel_b.pData[2],
					mag_b.pData[0], mag_b.pData[1], mag_b.pData[2],
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

	// Quaternion to euler angles for output
	float roll  = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));
	float pitch = asinf(2.0f * (q0 * q2 - q3 * q1));
	float yaw   = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));

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

//indexing starts at zero
//u cross v

//void quat_conj(arm_matrix_instance_f32 *in,arm_matrix_instance_f32 *out)
//{
//	float out_data[4] = {in->pData[0], -(in->pData[1]),-(in->pData[2]),-(in->pData[4])};
//	status = arm_mat_trans_f32(&mag_row, &mag_col);
//			status = arm_mat_mult_f32(&rot_mag_to_b, &mag_col, &mag_b);
//			arm_matrix_instance_f32 out_mat;
//			arm_mat_init_f32(&out_mat,4,1,out_data);
//				out=out_mat;
//			}
//void cross_prod(arm_matrix_instance_f32 *a,arm_matrix_instance_f32 *b, arm_matrix_instance_f32 *out)
//{
//	float a_data[3]=a->pData;
//	float b_data[3]=b->pData;
//	float out_data[3];
//	out_data[0]=(a_data[1]*b_data[2]-b_data[1]*a_data[2]);
//	out_data[1]=-(a_data[0]*b_data[2]-b_data[0]*a_data[2]);
//	out_data[2]=a_data[0]*b_data[1]-b_data[0]*a_data[1];
//
//	arm_matrix_instance_f32 out_mat;
//
//		arm_mat_init_f32(*out_mat,3,1,out_data);
//		out=out_mat;
//		//cross product
//}
//s=pData[0];
//pData=[scalar,i,j,k]
//scalar=s
//vector being rotated=pData[1],pData[2],pData[3];


//v= i,j,k components of original vector

//r=vector components of quaternion we're rotating about
//q=quaternion=[0,r]


//s=scalar


//v=3 dim vector

//v=pData[1],pData[2],pData[3];

//unit length quaternion:
//q=(r,s);




//q0=0,pData[1],pData[2],pData[3];
//unit quaternion is normalized/divided by its length
//magq=arm_sqrt_f32(q0);
//q=(1/magq)*q0;




//components)*quaternion vector components+(s^2-quat vector dotted w/ quat vector)*original vector components)
//+2s(quat vector comps cross original vector comps)


//rot_vector=arm_mult_f32(2(arm_dot_prod_f32(u,v)))+arm_mult_f32(s*s-arm_dot_prod_f32(q,q),v)+2*s*cross_prod(u,v);

