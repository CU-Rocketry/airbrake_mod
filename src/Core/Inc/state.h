/*
 * state.h
 *
 *  Created on: Apr 14, 2026
 *      Author: Sigmond
 */

#ifndef INC_STATE_H_
#define INC_STATE_H_

typedef struct {
    // Sensors
    float accel_ms2[3];
    float omega_rads[3];
    float mag_mgauss[3];
    float pres_hpa;

    // Body frame sensors
    float accel_b[3];
    float omega_b[3];
    float mag_b[3];

    // State estimation
    float quat[4];
//    float roll;
//    float pitch;
//    float yaw;

    // Control
    // TODO
} state_t;

extern state_t state; // global instance

#endif /* INC_STATE_H_ */
