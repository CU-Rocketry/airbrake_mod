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
#include <stdio.h>
#include "state.h"

uint16_t last_sample_time;
uint16_t sample_time;
float dt;

extern uint8_t imu_ready;
extern uint8_t mag_ready;
extern uint8_t baro_ready;

float roll, pitch, yaw;

void state_estimation(void) {

//	float accel_b_data[3];
//	float omega_b_data[3];
//	arm_matrix_instance_f32 accel_b, omega_b;
//	arm_mat_init_f32(&accel_b, 3, 1, accel_b_data);
//	arm_mat_init_f32(&omega_b, 3, 1, omega_b_data);

	float accel_b[3];
	float omega_b[3];
	get_imu_b(accel_b, omega_b);

//	float mag_b_data[3];
//	arm_matrix_instance_f32 mag_b;
//	arm_mat_init_f32(&mag_b, 3, 1, mag_b_data);
	float mag_b[3] = {0,0,0};
//	get_mag_b(&mag_b);

	// TODO
//	if (sample_time < last_sample_time) {
//		dt = (0xFFFF - last_sample_time + sample_time); // [us]
//	} else {
//		dt = (sample_time - last_sample_time); // [us]
//	}
//	dt *= 0.000001f;
	dt = 0.002f; // 500 Hz, hardcoded for now


	if (imu_ready)
	{
		imu_ready = 0;
		if (mag_ready) {
			mag_ready = 0;
			MadgwickAHRSupdate(omega_b[0], omega_b[1], omega_b[2],
					accel_b[0], accel_b[1], accel_b[2],
					mag_b[0], mag_b[1], mag_b[2],
					dt);

		} else {
			MadgwickAHRSupdateIMU(omega_b[0], omega_b[1], omega_b[2],
								 accel_b[0], accel_b[1], accel_b[2],
								 dt);
		}

		// EKF prediction
	}

	if (baro_ready) {
		baro_ready = 0;
		// EKF correction
	}

	MadgwickQuaternionGet(state.quat);

//	// Quaternion to euler angles for output
//	state.roll  = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]));
//	state.pitch = asinf(2.0f * (q[0] * q[2] - q[3] * q[1]));
//	state.yaw   = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]));
//
//	// pi = 180 degrees
//	state.roll *= (180.0f / PI);
//	state.pitch *= (180.0f / PI);
//	state.yaw *= (180.0f / PI);
}

//void state_estimation_ea_get(float* out_roll, float* out_pitch, float* out_yaw) {
//	*out_roll = roll;
//	*out_pitch = pitch;
//	*out_yaw = yaw;
//}

// ax:9.818493,ay:0.019139,az:-0.531117,ax_b:-0.531117,ay_b:-0.019139,az_b:-0.019139

//arm_status get_imu_b(arm_matrix_instance_f32 *out_accel, arm_matrix_instance_f32 *out_omega)
//{
//	arm_status status;
//
////	float accel[3] = state.accel_ms2;
////	float omega[3] = state.omega_rads;
//	// this is wrong
//
//	static float32_t rot_imu_to_b_data[3*3] = {
//	  0, 0, 1,
//	  0, -1, 0,
//	  1, 0, 0
//	};
//
//	arm_matrix_instance_f32 rot_imu_to_b;
//	arm_mat_init_f32(&rot_imu_to_b, 3, 3, rot_imu_to_b_data);
//
//	float accel_col_data[3];
//	float omega_col_data[3];
//
//	arm_matrix_instance_f32 accel_row, accel_col;
//	arm_matrix_instance_f32 omega_row, omega_col;
//
//	arm_mat_init_f32(&accel_row, 1, 3, state.accel_ms2);
//	arm_mat_init_f32(&omega_row, 1, 3, state.omega_rads);
//
//	arm_mat_init_f32(&accel_col, 3, 1, accel_col_data);
//	arm_mat_init_f32(&omega_col, 3, 1, omega_col_data);
//
//	status = arm_mat_trans_f32(&accel_row, &accel_col);
//	status = arm_mat_mult_f32(&rot_imu_to_b, &accel_col, out_accel);
//
//	memcpy(state.accel_b, out_accel->pData, 3 * sizeof(float));
//
//	status = arm_mat_trans_f32(&omega_row, &omega_col);
//	status = arm_mat_mult_f32(&rot_imu_to_b, &omega_col, out_omega);
//
//	memcpy(state.omega_b, out_omega->pData, 3 * sizeof(float));
//
//	return status;
//}

void get_imu_b(float out_accel_b[3], float out_omega_b[3]) {
	out_accel_b[0] = state.accel_ms2[2]; // body +X is now sensor +Z
	out_accel_b[1] = -state.accel_ms2[1]; // body +Y is now sensor -Y
	out_accel_b[2] = state.accel_ms2[0]; // body +Z is now sensor +X

	out_omega_b[0] = state.omega_rads[2]; // body +X is now sensor +Z
	out_omega_b[1] = -state.omega_rads[1]; // body +Y is now sensor -Y
	out_omega_b[2] = state.omega_rads[0]; // body +Z is now sensor +X
}

//arm_status get_mag_b(arm_matrix_instance_f32 *out_mag)
//{
//		arm_status status;
//
////		float mag[3] = state.mag_mgauss;
//
//		static float32_t mag_to_b_data[3*3] = {
//		  0, 0, 1,
//		  0, -1, 0,
//		  -1, 0, 0
//		};
//		arm_matrix_instance_f32 rot_mag_to_b;
//		arm_matrix_instance_f32 mag_row, mag_col, mag_b;
//
//
//		arm_mat_init_f32(&rot_mag_to_b, 3, 3, mag_to_b_data);
//		arm_mat_init_f32(&mag_row, 1, 3, state.mag_mgauss);
//
//
//		status = arm_mat_trans_f32(&mag_row, &mag_col);
//		status = arm_mat_mult_f32(&rot_mag_to_b, &mag_col, &mag_b);
//
//		return status;
//}

void get_mag_b(float out_mag_b[3]) {
	out_mag_b[0] = state.mag_mgauss[2]; // body +X is now sensor +Z
	out_mag_b[1] = -state.mag_mgauss[1]; // body +Y is now sensor -Y
	out_mag_b[2] = -state.mag_mgauss[0]; // body +Z is now sensor +X
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




//q[0]=0,pData[1],pData[2],pData[3];
//unit quaternion is normalized/divided by its length
//magq=arm_sqrt_f32(q[0]);
//q=(1/magq)*q[0];




//components)*quaternion vector components+(s^2-quat vector dotted w/ quat vector)*original vector components)
//+2s(quat vector comps cross original vector comps)


//rot_vector=arm_mult_f32(2(arm_dot_prod_f32(u,v)))+arm_mult_f32(s*s-arm_dot_prod_f32(q,q),v)+2*s*cross_prod(u,v);

