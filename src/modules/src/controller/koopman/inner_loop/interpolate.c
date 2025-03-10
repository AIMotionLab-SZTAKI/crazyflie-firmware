#include <stdio.h>
#include <math.h>
#include "controller_koopman.h"
#include "log.h"

static int16_t feedback_idx;

// Interpolation function
void interpolate_feedback_gains(float *p_cur, uint8_t M_out[nu][nx_full]) {
    float min_distance = 1e12;
    int best_dist_idx = -1;

    // Find the nearest point
    for (int i = 0; i < grid_point_num; i++) {
        float p_normed_LPV_grid_i[np];
        for (int j = 0; j < np; j++) {
            p_normed_LPV_grid_i[j] = p_normed_LPV_grid[j][i];
        }
        float distance = compute_squared_distance(p_cur, p_normed_LPV_grid_i, np);
        if (distance < min_distance) {
            min_distance = distance;
            best_dist_idx = i;
        }
    }

    // Copy the corresponding gain matrix into M_out
    for (int i = 0; i < nu; i++) {
        for (int j = 0; j < nx_full; j++) {
            M_out[i][j] = K_LPV_grid_uint8[i][j][best_dist_idx];
        }
    feedback_idx = best_dist_idx;
}

LOG_GROUP_START(Koopman)
LOG_ADD(LOG_INT16, feedback_idx, &feedback_idx)
LOG_GROUP_STOP(Koopman)