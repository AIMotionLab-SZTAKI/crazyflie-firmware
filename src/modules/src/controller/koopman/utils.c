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
