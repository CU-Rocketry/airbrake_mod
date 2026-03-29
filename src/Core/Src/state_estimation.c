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
	arm_matrix_instance omega_b;

	status = get_imu_b(&accel_b,&omega_b);

	arm_matrix_instance mag_b;
	status = get_mag_b(&mag_b);

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
	return arm_status;
}

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

	return arm_status;

}

void quat_conj(arm_matrix_instance *in,arm_matrix_instance *out)
{
	float out_data[4] = {in->pdata[0], -(in->pdata[1]),-(in->pdata[2]),-(in->pdata[4])};

	arm_matrix_instance out_mat;

	arm_mat_init_f32(&out_mat,4,1,out_data);
	out=out_mat;
}

void cross_prod(arm_matrix_instance *a,arm_matrix_instance *b, arm_matrix_instance *out)
{
	float a_data[3]=a->pdata;
	float b_data[3]=b->pdata;
	float out_data[3];
	out_data[0]=(a_data[1]*b_data[2]-b_data[1]*a_data[2]);
	out_data[1]=-(a_data[0]*b_data[2]-b_data[0]*a_data[2]);
	out_data[2]=a_data[0]*b_data[1]-b_data[0]*a_data[1];

	arm_matrix_instance out_mat;

		arm_mat_init_f32(*out_mat,3,1,out_data);
		out=out_mat;
		//cross product
}

//pdata=[scalar,i,j,k]
//scalar=s
//vector being rotated=pdata[1],pdata[2],pdata[3]

//v= i,j,k components of original vector

//r=vector components of quaternion we're rotating about
//q=quaternion=[0,r]


//s=scalar
s=pdata[0];

//v=3 dim vector

v=[pdata[1],pdata[2],pdata[3];]

//unit length quaternion:
q=(r,s)




q=[0,pdata[1],pdata[2],pdata[3]];


//new rotated vector:
rot_vector=q[1,2,3]+arm_scale_f32(2,q[1],q[2],q[3])*cross_prod()(cross_prod()+sv)

//rotated vector=


//2(quaternion vector components dotted with original vector
//components)*quaternion vector components+(s^2-quat vector dotted w/ quat vector)*original vector components)
//+2s(quat vector comps cross original vector comps)


rot_vector=arm_mult_f32(2(arm_dot_prod_f32(r,v)))+arm_mult_f32(s*s-arm_dot_prod_f32(r,r),v)+2s*cross_prod(r,v)


