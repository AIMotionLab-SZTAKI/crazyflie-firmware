#include "controller_koopman.h"
#include "log.h"

// Static variables for persistent filter states and control input
static float filter_states[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
static float u_normed_prev[4] = {0.0, 0.0, 0.0, 0.0};
static float target_state_normed_prev[9] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

void inner_loop_lpv(const float *current_state, const float *target_state, float *u) {
    // Temporary variables for intermediate calculations
    float performance_ch_error[4];                 // Error vector
    float filter_input[8];          // Filter input
    float temp_result[8];           // Temporary storage for matrix-vector results
    float normalized_cur_state[9];      // Normalized x_lpv
    float normalized_target_state[9];   // Normalized target state
    float subnet_cur_states[40];    // Subnet states estimated by encoder
    float target_subnet_states[40]; // Subnet states for the target
    float full_cur_state[48];        // Full state estimate
    float full_target_state[48];    // Full target state
    float e[nx_full];                    // Error vector for Koopman state estimation
    float p[np];                     // Scheduling variable
    float u_normed[4];              // Normalized control input
    float B_result[8];               // Temporary storage for matrix-vector results
    float target_filter_states[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // Target filter states
    uint8_t K_uint8[nu][nx_full];           // Feedback gain matrix
    float K[nu][nx_full];           // Feedback gain matrix

    // Normalize the current state: (current_state - y0) ./ ystd
    vector_normalize(current_state, y0_, ystd, normalized_cur_state, 9);

    // Normalize the target state: (target_state - y0) ./ ystd
    vector_normalize(target_state, y0_, ystd, normalized_target_state, 9);

    // Compute performance_ch_error vector [target_state - current_state]
    for (int i = 0; i < 3; i++) {
        performance_ch_error[i] = target_state_normed_prev[i] - normalized_cur_state[i]; // Positions (x, y, z)
    }
    performance_ch_error[3] = target_state_normed_prev[5] - normalized_cur_state[5]; // Yaw

    // Form the filter input vector [performance_ch_error; u_lpv]
    vector_stack(performance_ch_error, u_normed_prev, filter_input, 4, 4);

    // Calculate filter states: Wz_lpv_A * filter_est_prev + Wz_lpv_B * filter_input
    matrix_vector_multiply(&Wz_lpv_A[0][0], filter_states, temp_result, 8, 8); // Wz_lpv_A * filter_est_prev

    matrix_vector_multiply(&Wz_lpv_B[0][0], filter_input, B_result, 8, 8);       // Wz_lpv_B * filter_input

    // Sum the results
    vector_addition(temp_result, B_result, filter_states, 8);

    // Estimate subnet states using the encoder
    network_evaluate(normalized_cur_state, subnet_cur_states);

    // Concatenate filter states and subnet states to form full_cur_state
    vector_stack(filter_states, subnet_cur_states, full_cur_state, 8, 40);

    // Estimate target subnet states using the encoder
    network_evaluate(normalized_target_state, target_subnet_states);

    // Concatenate target filter states (zeros) and subnet states to form full_target_state
    vector_stack(target_filter_states, target_subnet_states, full_target_state, 8, 40);

    // Compute error vector e (negated compared to the MATLAB code)
    vector_subtraction(full_cur_state, full_target_state, e, nx_full);

    // Copy the last np states of normalized_cur_state to p
    for (int i = 0; i < np; i++) {
        p[i] = normalized_cur_state[i + (9 - np)];
    }

    // Get the feedback gain matrix K
    interpolate_feedback_gains(p, K_uint8);

    K_from_uint8_to_float(&K_uint8[0][0], K_min_val, K_range_val, &K[0][0], nu, nx_full);

    // Compute control input: u_normed = K * e (negated compared to the MATLAB code)
    matrix_vector_multiply(&K[0][0], e, u_normed, 4, nx_full);

    // Denormalize control input: u = u_normed .* ustd + u0
    vector_denormalize(u_normed, u0, ustd, u, 4);

    // Update the previous control input
    vector_copy(u_normed, u_normed_prev, 4);

    // Update the previous target state
    vector_copy(normalized_target_state, target_state_normed_prev, 9);
    
}


LOG_GROUP_START(Koopman)
LOG_ADD(LOG_FLOAT, thrust, &thrust)
LOG_ADD(LOG_FLOAT, thrust_normed, &thrust_normed)
LOG_GROUP_STOP(Koopman)