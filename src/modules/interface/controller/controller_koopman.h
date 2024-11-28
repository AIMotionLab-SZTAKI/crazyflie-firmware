#ifndef __CONTROLLER_KOOPMAN_H__
#define __CONTROLLER_KOOPMAN_H__

#include "stabilizer_types.h"

// Hardcoded feedback gains and scheduling points
#define nkoop 40
#define nz 8
#define grid_point_num 10
#define nx_full 48
#define nu 4
#define np 6

// Inner loop controller data declarations
extern float y0_[9];
extern float ystd[9];
extern float u0[4];
extern float ustd[4];


// Filter A and B matrices declarations
extern float Wz_lpv_A[8][8];
extern float Wz_lpv_B[8][8];


// Interpolation data declarations
extern float p_normed_LPV_grid[6][grid_point_num];
extern float K_LPV_grid[nu][nx_full][grid_point_num];


// Outer loop controller gains declarations
extern float Kp[3][3];
extern float Ki[3][3];

#define NUM_LAYERS 3
#define NUM_NEURONS 45
#define NUM_INPUTS 9
#define NUM_OUTPUTS nkoop

extern int structure[NUM_LAYERS][2];
extern float residual_weight[NUM_INPUTS][nkoop];
extern float residual_bias[nkoop];
extern float layer_0_weight[NUM_INPUTS][NUM_NEURONS];
extern float layer_0_bias[NUM_NEURONS];
extern float layer_1_weight[NUM_NEURONS][NUM_NEURONS];
extern float layer_1_bias[NUM_NEURONS];
extern float layer_2_weight[NUM_NEURONS][nkoop];
extern float layer_2_bias[nkoop];
extern float integral[3];
extern float filter_states[8];
extern float u_normed_prev[4];
extern float target_state_normed_prev[9];

void setKoopmanParams(float params[], int param_num, uint16_t timestamp);
void controllerKoopmanInit(void);
bool controllerKoopmanTest(void);
void controllerKoopmanReset(void);
void KoopmanRegisterMemoryHandler(void);
void controllerKoopman(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick);

void koopman_controller(float *current_state, float *desired_state, float *control_input);
void matrix_vector_multiply(const float *A, const float *x, float *result, int rows, int cols);

// Function to compute the Euclidean distance between two 3D points
float compute_distance(float *p_cur, float const *p_ref, int size);
float compute_squared_distance(float *p_cur, float const *p_ref, int size);

// Element-wise vector addition
void vector_addition(const float *v1, const float *v2, float *result, int size);

// Element-wise vector subtraction
void vector_subtraction(const float *v1, const float *v2, float *result, int size);

// Element-wise vector multiplication
void vector_multiplication(const float *v1, const float *v2, float *result, int size);

// Element-wise vector division
void vector_division(const float *v1, const float *v2, float *result, int size);

// Vector stacking function
void vector_stack(const float *v1, const float *v2, float *result, int size1, int size2);

// Vector copy
void vector_copy(const float *v, float *result, int size);

// Normalize vector
void vector_normalize(const float *v, const float *mean, const float *std, float *result, int size);

// Dennormalize vector
void vector_denormalize(const float *v, const float *mean, const float *std, float *result, int size);

void extrinsic_xyz_to_rotation_matrix(const float angles[3], float R[3][3]);

void scalar_first_quaternion_to_rotation_matrix(const float quaternion[4], float R[3][3]);

void matrix_to_intrinsic_xyz(const float R[3][3], float angles[3]);

void convert_state_extrinsic_intrinsic(float *current_state, float *desired_state);

void inner_loop_lpv(const float *target_state, const float *current_state, float *u);

void interpolate_feedback_gains(float *p_cur, float M_out[nu][nx_full]);

void network_evaluate(float *control_n, float *state_array);

void outer_loop_pi(float *current_pos, float *desired_pos, float *target_vel);

#endif //__CONTROLLER_KOOPMAN_H__
