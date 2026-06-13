/*
 * control.c
 *
 *  Created on: Apr 21, 2026
 *      Author: Sigmond
 */


#include "control.h"
#include "state.h"
#include <math.h>
//#include "telemetry.h"

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

void lockouts_init() {
	// Set all lockouts that won't be cleared until burnout
	STATE_FLAG_SET(FLAG_LOCKOUT_ACCEL);
	STATE_FLAG_SET(FLAG_LOCKOUT_ALTITUDE);
	STATE_FLAG_SET(FLAG_LOCKOUT_ELAPSED);

	STATE_FLAG_CLEAR(FLAG_LOCKOUT_ATTITUDE); // attitude should be good on the pad
	STATE_FLAG_CLEAR(FLAG_LOCKOUT_APOGEE); // apogee obv won't be reached yet either

	STATE_FLAG_CLEAR(FLAG_CONTROL_ENABLED); // whether we can deploy air brakes
}

void lockouts_check() {
    const float ACCEL_B_X_THRESH = 0.0f; // [m/s^2] must be < to deploy
    const float ALT_AGL_THRESH = 1000.0f; // [m] AGL must be >= to deploy
    const float ELAPSED_THRESH = 4.0f; // [s] since launch detect, must be >=
    const float VEL_Z_THRESH = 75.0f; // [m/s] must be >=, to disable near apogee

    if (global_state.accel_b[0] < ACCEL_B_X_THRESH)
    	STATE_FLAG_CLEAR(FLAG_LOCKOUT_ACCEL);

    if (global_state.alt_agl > ALT_AGL_THRESH)
        STATE_FLAG_CLEAR(FLAG_LOCKOUT_ALTITUDE);

    if (global_state.elapsed_t > ELAPSED_THRESH)
    	STATE_FLAG_CLEAR(FLAG_LOCKOUT_ELAPSED);

    // TODO
    // attitude threshold

    // to check if vertical velocity too low, we should already be in active control state
    if (STATE_FLAG_GET(FLAG_CONTROL_ENABLED) && (global_state.vel_z < VEL_Z_THRESH))
    	STATE_FLAG_SET(FLAG_LOCKOUT_APOGEE);

    uint8_t any_lockouts = 0; // starts 0, if any lockout is set, it will become 1
    any_lockouts |= STATE_FLAG_GET(FLAG_LOCKOUT_ACCEL);
    any_lockouts |= STATE_FLAG_GET(FLAG_LOCKOUT_ALTITUDE);
    any_lockouts |= STATE_FLAG_GET(FLAG_LOCKOUT_ELAPSED);
    any_lockouts |= STATE_FLAG_GET(FLAG_LOCKOUT_ATTITUDE);
    any_lockouts |= STATE_FLAG_GET(FLAG_LOCKOUT_APOGEE);
    STATE_FLAG_WRITE(FLAG_CONTROL_ENABLED, !any_lockouts); // if no lockouts are set, control enabled = 1. if any lockouts are set, control enabled = 0
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
    global_state.p_contrib = proportional;

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
    global_state.i_contrib = integral;

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
	lockouts_check();
    global_state.predicted = predict_apogee(global_state.alt_agl, global_state.vel_z, GRAVITY, ROCKET_CD, ROCKET_A_REF, ROCKET_MASS_EMPTY);
    global_state.output = pi_controller(global_state.predicted, STATE_FLAG_GET(FLAG_CONTROL_ENABLED), Kp, Ki, TARGET_APOGEE, dt);
}
