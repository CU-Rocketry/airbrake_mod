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
					-accel_b[0], -accel_b[1], -accel_b[2],
					mag_b[0], mag_b[1], mag_b[2],
					dt);

		} else {
			MadgwickAHRSupdateIMU(omega_b[0], omega_b[1], omega_b[2],
								 -accel_b[0], -accel_b[1], -accel_b[2],
								 dt);
		}

		MadgwickQuaternionGet(state.quat); // output madgwick filter quaternion to state

		float q_star[4];
		quat_conj(state.quat, q_star); // get conjugate of madgwick output
		// now q_star is the body to earth rotation

		float accel_e[3];
		quat_rot(accel_b, q_star, accel_e); // rotate body accel by b to e rotation to get inertial acceleration

		accel_e[2] += 9.80665f; // get rid of gravity. TODO a constant maybe

		state.accel_e[0] = accel_e[0];
		state.accel_e[1] = accel_e[1];
		state.accel_e[2] = accel_e[2];

		// EKF prediction
	}

	if (baro_ready) {
		baro_ready = 0;
		// EKF correction
	}
}

void get_imu_b(float out_accel_b[3], float out_omega_b[3]) {
	out_accel_b[0] = state.accel_ms2[2]; // body +X is now sensor +Z
	out_accel_b[1] = -state.accel_ms2[1]; // body +Y is now sensor -Y
	out_accel_b[2] = state.accel_ms2[0]; // body +Z is now sensor +X

	out_omega_b[0] = state.omega_rads[2]; // body +X is now sensor +Z
	out_omega_b[1] = -state.omega_rads[1]; // body +Y is now sensor -Y
	out_omega_b[2] = state.omega_rads[0]; // body +Z is now sensor +X
}

void get_mag_b(float out_mag_b[3]) {
	out_mag_b[0] = state.mag_mgauss[2]; // body +X is now sensor +Z
	out_mag_b[1] = -state.mag_mgauss[1]; // body +Y is now sensor -Y
	out_mag_b[2] = -state.mag_mgauss[0]; // body +Z is now sensor +X
}

//indexing starts at zero
//u cross v


// q* = [q0, -q1, -q2, -q3] at least in the representation we have
void quat_conj(float q[4], float q_star[4])
{
	q_star[0] =  q[0];
	q_star[1] = -q[1];
	q_star[2] = -q[2];
	q_star[3] = -q[3];
}

// rotates a vector by quaternion
// see https://danceswithcode.net/engineeringnotes/quaternions/quaternions.html
void quat_rot(float v[3], float q[4], float v_out[3]) {
	v_out[0] = v[0] * (1.0f - 2.0f * (q[2]*q[2] + q[3]*q[3])) +
			   v[1] * (2.0f * (q[1]*q[2] - q[0]*q[3])) +
			   v[2] * (2.0f * (q[1]*q[3] + q[0]*q[2]));

	v_out[1] = v[0] * (2.0f * (q[1]*q[2] + q[0]*q[3])) +
			   v[1] * (1.0f - 2.0f * (q[1]*q[1] + q[3]*q[3])) +
			   v[2] * (2.0f * (q[2]*q[3] - q[0]*q[1]));

	v_out[2] = v[0] * (2.0f * (q[1]*q[3] - q[0]*q[2])) +
			   v[1] * (2.0f * (q[2]*q[3] + q[0]*q[1])) +
			   v[2] * (1.0f - 2.0f * (q[1]*q[1] + q[2]*q[2]));
}

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

