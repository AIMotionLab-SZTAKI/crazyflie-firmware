#include "controller_koopman.h"

static float integral[3] = {0.0, 0.0, 0.0};  // Integral state for x, y, z

void outer_loop_pi(float *current_pos, float *desired_pos, float *target_vel) {
    float dt = 0.1; // Time step

    // Calculate the error between desired and current position
    float error_vec[3];
    for (int i = 0; i < 3; i++) {
        error_vec[i] = desired_pos[i] - current_pos[i];
    }

    // Update the integral state
    for (int i = 0; i < 3; i++) {
        integral[i] += error_vec[i] * dt;
    }
    
    // Calculate the control input using matrix-vector multiplication
    float control_input[3];
    float integral_vec[3] = {integral[0], integral[1], integral[2]};
    matrix_vector_multiply(&Kp[0][0], error_vec, control_input, 3, 3);
    float integral_term[3];
    matrix_vector_multiply(&Ki[0][0], integral_vec, integral_term, 3, 3);
    for (int i = 0; i < 3; i++) {
        control_input[i] += integral_term[i];
    }
}