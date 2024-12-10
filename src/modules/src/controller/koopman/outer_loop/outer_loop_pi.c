#include "controller_koopman.h"

// Outer loop controller gains
float Kp[3][3] = {
    {0, 0, 0}, {0, 0, 0}, {0, 0, 0}
};
float Ki[3][3] = {
    {0.0, 0, 0}, {0, 0.0, 0}, {0, 0, 0.0}
};

float integral[3] = {0.0, 0.0, 0.0};  // Integral state for x, y, z

void outer_loop_pi(float *current_pos, float *desired_pos, float *target_vel) {

    // Calculate the error between desired and current position
    float error_vec[3];
    for (int i = 0; i < 3; i++) {
        error_vec[i] = desired_pos[i] - current_pos[i];
    }

    // Update the integral state
    for (int i = 0; i < 3; i++) {
        integral[i] += error_vec[i];
    }
    
    // Calculate the control input using matrix-vector multiplication
    float integral_vec[3] = {integral[0], integral[1], integral[2]};
    matrix_vector_multiply(&Kp[0][0], error_vec, target_vel, 3, 3);
    float integral_term[3];
    matrix_vector_multiply(&Ki[0][0], integral_vec, integral_term, 3, 3);
    for (int i = 0; i < 3; i++) {
        target_vel[i] += integral_term[i];
    }
}