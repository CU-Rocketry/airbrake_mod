/*
 * control.c
 *
 *  Created on: Apr 21, 2026
 *      Author: Sigmond
 */


#include "control.h"
#include "state.h"
#include <math.h>

// Tuning parameters
static const float Kp = 0.03f;
static const float Ki = 0.00001f;

// predict apogee uses a projectile except with drag. mass is the rocket empty mass as it has already burned all fuel
float predict_apogee(float alt_agl, float vel_z, float g, float Cd, float a_ref, float mass_empty) {
    float rho = 1.225f * expf(-alt_agl / 9000.0f); // ISA density approximation

    if (vel_z > 0.0f) {
        float F_d = 0.5f * rho * (vel_z * vel_z) * Cd * a_ref; // 0.5bv^2 basically where drag coefficient b is rho, Cd, a_ref
        float a_d = F_d / mass_empty; // a = F/m

        // apogee = current + (v^2/2g) but with drag
        return alt_agl + ((vel_z * vel_z) / (2.0f * (g + a_d))); // gravity and drag will act downwards together
    } else {
        return alt_agl; // we're falling so current height is apogee
    }
}

uint8_t airbrakes_lockout(float accel_b_x, float alt_agl, float elapsed, float vel_z) {
    // persistent state variable
    static uint8_t vel_z_thresh_triggered = 0;

    const float ACCEL_B_X_THRESH = 0.0f; // [m/s^2] must be < to deploy
    const float ALT_AGL_THRESH = 1000.0f; // [m] AGL must be >= to deploy
    const float ELAPSED_THRESH = 4.0f; // [s] since launch detect, must be >=
    const float VEL_Z_THRESH = 75.0f; // [m/s] must be >=, to disable near apogee

    uint8_t airbrakes_enabled = 0;

    if ((accel_b_x < ACCEL_B_X_THRESH) &&
        (alt_agl >= ALT_AGL_THRESH) &&
        (elapsed >= ELAPSED_THRESH) &&
        (!vel_z_thresh_triggered)) {
        airbrakes_enabled = 1;
    }

    // to check vel_z vs the thresh, we should be in active control already
    if (airbrakes_enabled && (vel_z < VEL_Z_THRESH)) {
        vel_z_thresh_triggered = 1;
        airbrakes_enabled = 0;
        // printf("Triggered lockout\r\n"); // Optional debug
    }

    return airbrakes_enabled;
}

// [airbrakes, integral_error, error, error_raw, proportional, integral_error_raw, integral]
float pi_controller(float predicted, uint8_t enable, float Kp, float Ki, float target, float dt) {
    static float integral_error = 0.0f;

    if (enable == 0) {
        integral_error = 0.0f;
        return 0.0f; // disable air brakes deployment
    }

    // error simply difference to target apogee
    float error_raw = predicted - target;
    float error;

    if (error_raw > 0.0f) {
        error = error_raw;
    } else {
        // if undershooting, pretend its zero-- dont let controller try to fix, and hope for the best
        error = 0.0f;
        // reset integrator to react faster if overshooting later
        integral_error = 0.0f;
    }

    float proportional = Kp * error;

    // Note: Added dt multiplication here to make it a true time-based integrator
    float integral_error_raw = integral_error + (error * dt);

    // Cap integral contribution to 50% deployment
    float INTEGRAL_MAX = 0.5f / Ki;

    if (integral_error_raw > INTEGRAL_MAX) {
        integral_error = INTEGRAL_MAX;
    } else {
        integral_error = integral_error_raw;
    }

    float integral = Ki * integral_error;
    float output = proportional + integral;

    // clamp
    if (output > 1.0f) {
        output = 1.0f;
    } else if (output < 0.0f) {
        output = 0.0f;
    }

    return output;
}

void control_update(float dt) {
    // Calculate elapsed time since launch (requires launch_time to be tracked in state)
    // float elapsed_time = (state.t - state.launch_time) / 1000.0f;
    float elapsed_time = 5.0f; // TODO: Pull real elapsed time

    // 1. Predict
    float predicted_apogee = predict_apogee(state.alt_agl, state.vel_z, GRAVITY, ROCKET_CD, ROCKET_A_REF, ROCKET_MASS_EMPTY);

    // 2. Lockout
    // Assuming body X is your axial acceleration out of the IMU
    uint8_t brakes_enabled = airbrakes_lockout(state.accel_b[0], state.alt_agl, elapsed_time, state.vel_z);

    // 3. Control
    float brake_command = pi_controller(predicted_apogee, brakes_enabled, Kp, Ki, TARGET_APOGEE, dt);

    // 4. Actuate
    // Assuming servo takes a deployment percentage 0.0 to 1.0, map it to PWM limits
    // e.g., duty = duty_retracted + brake_command * (duty_extended - duty_retracted)
    state.servo_cmd = brake_command;
}
