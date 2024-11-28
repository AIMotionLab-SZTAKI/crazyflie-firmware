#include "controller_koopman.h"
#include <math.h>

// Matrix-vector multiplication function
void matrix_vector_multiply(const float *A, const float *x, float *result, int rows, int cols) {
    for (int i = 0; i < rows; i++) {
        result[i] = 0.0;
        for (int j = 0; j < cols; j++) {
            result[i] += A[i * cols + j] * x[j];
        }
    }
}

// Function to compute the Euclidean distance between two 3D points
float compute_distance(float *p_cur, float const *p_ref, int size) {
    float distance = 0.0;
    for (int i = 0; i < size; i++) {
        distance += (p_cur[i] - p_ref[i]) * (p_cur[i] - p_ref[i]);
    }
    return sqrt(distance);
}

// Function to compute the Euclidean distance between two 3D points
float compute_squared_distance(float *p_cur, float const *p_ref, int size) {
    float squared_distance = 0.0;
    for (int i = 0; i < size; i++) {
        squared_distance += (p_cur[i] - p_ref[i]) * (p_cur[i] - p_ref[i]);
    }
    return squared_distance;
}

// Element-wise vector addition
void vector_addition(const float *v1, const float *v2, float *result, int size) {
    for (int i = 0; i < size; i++) {
        result[i] = v1[i] + v2[i];
    }
}

// Element-wise vector subtraction
void vector_subtraction(const float *v1, const float *v2, float *result, int size) {
    for (int i = 0; i < size; i++) {
        result[i] = v1[i] - v2[i];
    }
}

// Element-wise vector multiplication
void vector_multiplication(const float *v1, const float *v2, float *result, int size) {
    for (int i = 0; i < size; i++) {
        result[i] = v1[i] * v2[i];
    }
}

// Element-wise vector division
void vector_division(const float *v1, const float *v2, float *result, int size) {
    for (int i = 0; i < size; i++) {
        result[i] = v1[i] / v2[i];
    }
}

// Vector stacking function
void vector_stack(const float *v1, const float *v2, float *result, int size1, int size2) {
    for (int i = 0; i < size1; i++) {
        result[i] = v1[i];
    }
    for (int i = 0; i < size2; i++) {
        result[size1 + i] = v2[i];
    }
}

// Vector copy
void vector_copy(const float *v, float *result, int size) {
    for (int i = 0; i < size; i++) {
        result[i] = v[i];
    }
}

// Normalize a vector as (v - mean) / std
void vector_normalize(const float *v, const float *mean, const float *std, float *result, int size) {
    for (int i = 0; i < size; i++) {
        result[i] = (v[i] - mean[i]) / std[i];
    }
}

// Denormalize a vector as v * std + mean
void vector_denormalize(const float *v, const float *mean, const float *std, float *result, int size) {
    for (int i = 0; i < size; i++) {
        result[i] = v[i] * std[i] + mean[i];
    }
}

void extrinsic_xyz_to_rotation_matrix(const float angles[3], float R[3][3]) {
    float alpha = angles[0]; // Rotation about X-axis
    float beta = angles[1];  // Rotation about Y-axis
    float gamma = angles[2]; // Rotation about Z-axis

    // Rotation about X-axis
    float R_x[3][3] = {
        {1, 0, 0},
        {0, cos(alpha), -sin(alpha)},
        {0, sin(alpha), cos(alpha)}
    };

    // Rotation about Y-axis
    float R_y[3][3] = {
        {cos(beta), 0, sin(beta)},
        {0, 1, 0},
        {-sin(beta), 0, cos(beta)}
    };

    // Rotation about Z-axis
    float R_z[3][3] = {
        {cos(gamma), -sin(gamma), 0},
        {sin(gamma), cos(gamma), 0},
        {0, 0, 1}
    };

    // Temporary matrix for R_z * R_y
    float R_temp[3][3];

    // Matrix multiplication R_temp = R_z * R_y
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            R_temp[i][j] = 0;
            for (int k = 0; k < 3; k++) {
                R_temp[i][j] += R_z[i][k] * R_y[k][j];
            }
        }
    }

    // Final matrix R = R_temp * R_x
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            R[i][j] = 0;
            for (int k = 0; k < 3; k++) {
                R[i][j] += R_temp[i][k] * R_x[k][j];
            }
        }
    }
}

void scalar_first_quaternion_to_rotation_matrix(const float quaternion[4], float R[3][3]) {
    float q0 = quaternion[0];
    float q1 = quaternion[1];
    float q2 = quaternion[2];
    float q3 = quaternion[3];

    // Compute the rotation matrix elements
    R[0][0] = 1 - 2 * (q2 * q2 + q3 * q3);
    R[0][1] = 2 * (q1 * q2 - q3 * q0);
    R[0][2] = 2 * (q1 * q3 + q2 * q0);

    R[1][0] = 2 * (q1 * q2 + q3 * q0);
    R[1][1] = 1 - 2 * (q1 * q1 + q3 * q3);
    R[1][2] = 2 * (q2 * q3 - q1 * q0);

    R[2][0] = 2 * (q1 * q3 - q2 * q0);
    R[2][1] = 2 * (q2 * q3 + q1 * q0);
    R[2][2] = 1 - 2 * (q1 * q1 + q2 * q2);
}

void matrix_to_intrinsic_xyz(const float R[3][3], float angles[3]) {
    // Compute roll, pitch, and yaw
    angles[0] = atan2(-R[1][2], R[2][2]);  // Roll
    angles[1] = asin(R[0][2]);            // Pitch
    angles[2] = atan2(-R[0][1], R[0][0]); // Yaw
}

void convert_state_extrinsic_intrinsic(float *current_state, float *desired_state) {

    float extrinsic_xyz_euler[3];

    float R[3][3];

    float intrinsic_xyz_euler[3];

    // Convert current state from extrinsic to intrinsic
    for (int i = 0; i < 3; i++) {
        extrinsic_xyz_euler[i] = current_state[i + 6];
    }

    extrinsic_xyz_to_rotation_matrix(extrinsic_xyz_euler, R);

    matrix_to_intrinsic_xyz(R, intrinsic_xyz_euler);

    for (int i = 0; i < 3; i++) {
        current_state[i + 6] = intrinsic_xyz_euler[i];
    }

    // Convert desired state from extrinsic to intrinsic
    for (int i = 0; i < 3; i++) {
        extrinsic_xyz_euler[i] = desired_state[i + 6];
    }
    
    extrinsic_xyz_to_rotation_matrix(extrinsic_xyz_euler, R);

    matrix_to_intrinsic_xyz(R, intrinsic_xyz_euler);
    
    for (int i = 0; i < 3; i++) {
        desired_state[i + 6] = intrinsic_xyz_euler[i];
    }
}