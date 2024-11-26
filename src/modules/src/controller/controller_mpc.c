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
#include "pid.h"
#include "platform_defaults.h"

#define ATTITUDE_UPDATE_DT    (float)(1.0f/ATTITUDE_RATE)

// Logging variables

static float cmd_thrust;
static float cmd_roll;
static float cmd_pitch;
static float cmd_yaw;

static bool rateFiltEnable = ATTITUDE_RATE_LPF_ENABLE;
static float omxFiltCutoff = ATTITUDE_ROLL_RATE_LPF_CUTOFF_FREQ;
static float omyFiltCutoff = ATTITUDE_PITCH_RATE_LPF_CUTOFF_FREQ;
static float omzFiltCutoff = ATTITUDE_YAW_RATE_LPF_CUTOFF_FREQ;

static inline int16_t saturateSignedInt16(float in)
{
  // don't use INT16_MIN, because later we may negate it, which won't work for that value.
  if (in > INT16_MAX)
    return INT16_MAX;
  else if (in < -INT16_MAX)
    return -INT16_MAX;
  else
    return (int16_t)in;
}

PidObject mpcRollRate = {
  .kp = PID_ROLL_RATE_KP,
  .ki = PID_ROLL_RATE_KI,
  .kd = PID_ROLL_RATE_KD,
  .kff = PID_ROLL_RATE_KFF,
};

PidObject mpcPitchRate = {
  .kp = PID_PITCH_RATE_KP,
  .ki = PID_PITCH_RATE_KI,
  .kd = PID_PITCH_RATE_KD,
  .kff = PID_PITCH_RATE_KFF,
};

PidObject mpcYawRate = {
  .kp = PID_YAW_RATE_KP,
  .ki = PID_YAW_RATE_KI,
  .kd = PID_YAW_RATE_KD,
  .kff = PID_YAW_RATE_KFF,
};

static int16_t rollOutput;
static int16_t pitchOutput;
static int16_t yawOutput;

static bool isInit;

static void mpcReset(void)
{
  pidReset(&mpcRollRate);
  pidReset(&mpcPitchRate);
  pidReset(&mpcYawRate);
}

static void attitudeControllerCorrectRatePID(
       float rollRateActual, float pitchRateActual, float yawRateActual,
       float rollRateDesired, float pitchRateDesired, float yawRateDesired)
{
  pidSetDesired(&mpcRollRate, rollRateDesired);
  rollOutput = saturateSignedInt16(pidUpdate(&mpcRollRate, rollRateActual, true));

  pidSetDesired(&mpcPitchRate, pitchRateDesired);
  pitchOutput = saturateSignedInt16(pidUpdate(&mpcPitchRate, pitchRateActual, true));

  pidSetDesired(&mpcYawRate, yawRateDesired);
  yawOutput = saturateSignedInt16(pidUpdate(&mpcYawRate, yawRateActual, true));
}

static void attitudeControllerGetActuatorOutput(int16_t* roll, int16_t* pitch, int16_t* yaw)
{
  *roll = rollOutput;
  *pitch = pitchOutput;
  *yaw = yawOutput;
}


void controllerMpcInit(void)
{
  if(isInit)
  return;
  pidInit(&mpcRollRate,  0, mpcRollRate.kp,  mpcRollRate.ki,  mpcRollRate.kd,
       mpcRollRate.kff,  ATTITUDE_UPDATE_DT, ATTITUDE_RATE, omxFiltCutoff, rateFiltEnable);
  pidInit(&mpcPitchRate, 0, mpcPitchRate.kp, mpcPitchRate.ki, mpcPitchRate.kd,
       mpcPitchRate.kff, ATTITUDE_UPDATE_DT, ATTITUDE_RATE, omyFiltCutoff, rateFiltEnable);
  pidInit(&mpcYawRate,   0, mpcYawRate.kp,   mpcYawRate.ki,   mpcYawRate.kd,
       mpcYawRate.kff,   ATTITUDE_UPDATE_DT, ATTITUDE_RATE, omzFiltCutoff, rateFiltEnable);

  pidSetIntegralLimit(&mpcRollRate,  PID_ROLL_RATE_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&mpcPitchRate, PID_PITCH_RATE_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&mpcYawRate,   PID_YAW_RATE_INTEGRATION_LIMIT);
  isInit = true;
  mpcReset();
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

    mpcReset();
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
