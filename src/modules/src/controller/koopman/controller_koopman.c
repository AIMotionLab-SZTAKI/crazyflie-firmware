/*
Koopman controller.
*/

#include <math.h>

#include "param.h"
#include "log.h"
#include "math3d.h"
#include "stabilizer.h"
#include "physicalConstants.h"
#include "controller_koopman.h"
#include "pm.h"
#include "debug.h"
#include "controller.h"
#include "stdlib.h"
#include "mem.h"

#define ATTITUDE_UPDATE_DT    (float)(1.0f/ATTITUDE_RATE)

// Logging variables

static float cmd_thrust;
static float cmd_roll;
static float cmd_pitch;
static float cmd_yaw;

void controllerKoopmanReset(void) {

}

void controllerKoopmanInit(void)
{

}

bool controllerKoopmanTest(void)
{
  return true;
}

// Koopman controller implementation
void koopman_controller(float *current_state, float *desired_state, float *control_input) {
    // Extract positions
    float current_pos[3] = {current_state[0], current_state[1], current_state[2]};
    float current_vel_state[9] = {current_state[3], current_state[4], current_state[5], current_state[6], current_state[7], current_state[8], current_state[9], current_state[10], current_state[11]};
    float desired_pos[3] = {desired_state[0], desired_state[1], desired_state[2]};
    float desired_vel_state[9] = {desired_state[3], desired_state[4], desired_state[5], desired_state[6], desired_state[7], desired_state[8], desired_state[9], desired_state[10], desired_state[11]};

    // Call the outer loop controller to compute velocity reference
    float outer_loop_vel[3] = {0.0, 0.0, 0.0};
    outer_loop_pi(current_pos, desired_pos, outer_loop_vel);

    // Create the inner loop reference (desired velocities + outer loop velocities)
    float inner_loop_ref[9] = {
        desired_vel_state[0] + outer_loop_vel[0],
        desired_vel_state[1] + outer_loop_vel[1],
        desired_vel_state[2] + outer_loop_vel[2],
        desired_vel_state[3], // Desired roll angle
        desired_vel_state[4], // Desired pitch angle
        desired_vel_state[5], // Desired yaw angle
        desired_vel_state[6], // Desired roll rate
        desired_vel_state[7], // Desired pitch rate
        desired_vel_state[8]  // Desired yaw rate
    };

    // Call the inner loop controller
    inner_loop_lpv(current_vel_state, inner_loop_ref, control_input);
}

void controllerKoopman(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{
  if (!RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
      return;
    }
  float current_state[12] = {
    state->position.x, state->position.y, state->position.z,
    state->velocity.x, state->velocity.y, state->velocity.z,
    radians(state->attitude.roll), radians(state->attitude.pitch), radians(state->attitude.yaw), // elojel?
    radians(sensors->gyro.x), radians(sensors->gyro.y), radians(sensors->gyro.z) // elojel?
  }; 
  float desired_state[12] = {
    setpoint->position.x, setpoint->position.y, setpoint->position.z,
    setpoint->velocity.x, setpoint->velocity.y, setpoint->velocity.z,
    radians(setpoint->attitude.roll), radians(setpoint->attitude.pitch), radians(setpoint->attitude.yaw), // elojel?
    radians(setpoint->attitudeRate.roll), radians(setpoint->attitudeRate.pitch), radians(setpoint->attitudeRate.yaw) // elojel?
  };
  float control_input[4] = {0, 0, 0, 0};
  koopman_controller(current_state, desired_state, control_input);
  control->thrust = control_input[0] * 132000;
  control->roll = (int16_t)(control_input[1] / 8.7823e-7f);
  control->pitch = (int16_t)(control_input[2] / 8.7823e-7f); // elojel?
  control->yaw = (int16_t)(control_input[3] / 1.6103e-7f); // elojel?
}


PARAM_GROUP_START(Koopman)
//PARAM_ADD(PARAM_FLOAT, drone_mass, &drone_mass)
//PARAM_ADD(PARAM_INT32, max_delay, &max_delay)
//PARAM_ADD(PARAM_UINT32, max_delay_time_ms, &max_delay_time_ms)
//PARAM_ADD(PARAM_UINT32, duration, &duration)
PARAM_GROUP_STOP(Koopman)


LOG_GROUP_START(Koopman)
LOG_ADD(LOG_FLOAT, cmd_thrust, &cmd_thrust)
LOG_ADD(LOG_FLOAT, cmd_pitch, &cmd_pitch)
LOG_ADD(LOG_FLOAT, cmd_roll, &cmd_roll)
LOG_ADD(LOG_FLOAT, cmd_yaw, &cmd_yaw)
LOG_GROUP_STOP(Koopman)