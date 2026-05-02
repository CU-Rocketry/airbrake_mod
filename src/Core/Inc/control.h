/*
 * control.h
 *
 *  Created on: Apr 21, 2026
 *      Author: Sigmond
 */

#ifndef INC_CONTROL_H_
#define INC_CONTROL_H_

#include <stdint.h>

#define ROCKET_MASS_EMPTY 24.526f   // [kg]
#define ROCKET_CD 0.32f // Tunable drag coefficient
#define ROCKET_A_REF 0.013478f // [m^2] Reference area
// #define TARGET_APOGEE 3048.0f // [m] 10,000 ft
#define TARGET_APOGEE 850.0f // [m] for testbed


void control_update(float dt);
float predict_apogee(float alt_agl, float vel_z, float g, float Cd, float a_ref, float mass_empty);
uint8_t airbrakes_lockout(float accel_b_x, float alt_agl, float elapsed, float vel_z);
float pi_controller(float predicted, uint8_t enable, float Kp, float Ki, float target, float dt);

#endif /* INC_CONTROL_H_ */
