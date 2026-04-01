/*
 * state_estimation.h
 *
 *  Created on: Mar 27, 2026
 *      Author: Sigmond
 */

#ifndef INC_STATE_ESTIMATION_H_
#define INC_STATE_ESTIMATION_H_

void state_estimation(void);
arm_status get_imu_b(arm_matrix_instance_f32 *out_accel, arm_matrix_instance_f32 *out_omega);

#endif /* INC_STATE_ESTIMATION_H_ */
