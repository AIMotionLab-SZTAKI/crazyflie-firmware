/*
MPC controller.
*/

#include <math.h>

#include "param.h"
#include "log.h"
#include "math3d.h"
#include "stabilizer.h"
#include "physicalConstants.h"
#include "controller_mpc.h"
#include "pm.h"
#include "debug.h"
#include "controller.h"
#include "stdlib.h"
#include "mem.h"
#include "attitude_controller.h"

#define ATTITUDE_UPDATE_DT    (float)(1.0f/ATTITUDE_RATE)

// Logging variables

static float cmd_thrust;
static float cmd_roll;
static float cmd_pitch;
static float cmd_yaw;

void controllerMpcReset(void) {
  //No integral part to reset
}

void controllerMpcInit(void)
{
  attitudeControllerInit(ATTITUDE_UPDATE_DT);
  controllerMpcReset();
}

bool controllerMpcTest(void)
{
  return true;
}

void controllerMpc(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{
  if (!RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
      return;
    }

  control->controlMode = controlModeLegacy;

  attitudeControllerCorrectRatePID(sensors->gyro.x, -sensors->gyro.y, sensors->gyro.z, 
  setpoint->attitudeRate.roll, setpoint->attitudeRate.pitch, setpoint->attitudeRate.yaw);

  attitudeControllerGetActuatorOutput(&control->roll,
                                      &control->pitch,
                                      &control->yaw);

  control->yaw = -control->yaw;
  control->thrust = setpoint->thrust;

  cmd_thrust = control->thrust;
  cmd_roll = control->roll;
  cmd_pitch = control->pitch;
  cmd_yaw = control->yaw;

  if (control->thrust == 0)
  {
    control->thrust = 0;
    control->roll = 0;
    control->pitch = 0;
    control->yaw = 0;

    cmd_thrust = control->thrust;
    cmd_roll = control->roll;
    cmd_pitch = control->pitch;
    cmd_yaw = control->yaw;

    attitudeControllerResetAllPID();
  }
}


PARAM_GROUP_START(Mpc)
//PARAM_ADD(PARAM_FLOAT, drone_mass, &drone_mass)
//PARAM_ADD(PARAM_INT32, max_delay, &max_delay)
//PARAM_ADD(PARAM_UINT32, max_delay_time_ms, &max_delay_time_ms)
//PARAM_ADD(PARAM_UINT32, duration, &duration)
PARAM_GROUP_STOP(Mpc)


LOG_GROUP_START(Mpc)
LOG_ADD(LOG_FLOAT, cmd_thrust, &cmd_thrust)
LOG_ADD(LOG_FLOAT, cmd_pitch, &cmd_pitch)
LOG_ADD(LOG_FLOAT, cmd_roll, &cmd_roll)
LOG_ADD(LOG_FLOAT, cmd_yaw, &cmd_yaw)
LOG_GROUP_STOP(Mpc)
