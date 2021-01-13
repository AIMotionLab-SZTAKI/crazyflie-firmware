/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2016 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * position_controller_sqrt.c: lin-sqrt implementation of the position controller
 */

#include <math.h>
#include "num.h"

#include "commander.h"
#include "log.h"
#include "param.h"
#include "pid.h"
#include "num.h"
#include "position_controller.h"

struct pidInit_s {
  float kp;
  float ki;
  float kd;
  float iLimit;
};

struct pidAxis_s {
  PidObject pid;

  struct pidInit_s init;
    stab_mode_t previousMode;
  float setpoint;

  float output;
};

struct this_s {
  struct pidAxis_s pidVX;
  struct pidAxis_s pidVY;
  struct pidAxis_s pidVZ;

  uint16_t thrustBase; // approximate throttle needed when in perfect hover. More weight/older battery can use a higher value
  uint16_t thrustMin;  // Minimum thrust value to output

  float pX; // linear gain of X sqrt position controller
  float pY; // linear gain of X sqrt position controller
  float pZ; // linear gain of Z sqrt position controller

  float vXY_ff[2]; // linear and second order gains of the feed forward term in the velocity controller
};

// Maximum roll/pitch angle permited
static float rpLimit  = 20;
static float rpLimitOverhead = 1.10f;
// Velocity maxima
static float xyVelMax = 3.0f;
static float zVelMax  = 0.6f;
// Acceleration maxima
static float xyAccMax = 6.0f;
static float zAccMax = 3.0f;
// Thrust parameters
static const float thrustScale = 1000.0f;

// Original input of the velocity controller, without any correction terms
static velocity_t originalVelocity = { 0.0f, 0.0f, 0.0f };

#define DT (float)(1.0f/POSITION_RATE)
#define POSITION_LPF_CUTOFF_FREQ 20.0f
#define POSITION_LPF_ENABLE true

#ifndef UNIT_TEST
static struct this_s this = {
  .pidVX = {
    .init = {
      .kp = 22.0f,
      .ki = 1.0f,
      .kd = 0.02f,
      .iLimit = 2.5f,
    },
    .pid.dt = DT,
  },

  .pidVY = {
    .init = {
      .kp = 22.0f,
      .ki = 1.0f,
      .kd = 0.02f,
      .iLimit = 2.5f,
    },
    .pid.dt = DT,
  },

  .pidVZ = {
    .init = {
      .kp = 25,
      .ki = 15,
      .kd = 0,
      .iLimit = 1.333333f,    /* 20.0 / ki */
    },
    .pid.dt = DT,
  },

  .thrustBase = 43000,
  .thrustMin  = 20000,

  .pX = 2,
  .pY = 2,
  .pZ = 0.5,

  .vXY_ff = { 1.1, 0 }
};
#endif

void positionControllerInit()
{
  pidInit(&this.pidVX.pid, this.pidVX.setpoint, this.pidVX.init.kp, this.pidVX.init.ki, this.pidVX.init.kd,
      this.pidVX.pid.dt, POSITION_RATE, POSITION_LPF_CUTOFF_FREQ, POSITION_LPF_ENABLE);
  pidInit(&this.pidVY.pid, this.pidVY.setpoint, this.pidVY.init.kp, this.pidVY.init.ki, this.pidVY.init.kd,
      this.pidVY.pid.dt, POSITION_RATE, POSITION_LPF_CUTOFF_FREQ, POSITION_LPF_ENABLE);
  pidInit(&this.pidVZ.pid, this.pidVZ.setpoint, this.pidVZ.init.kp, this.pidVZ.init.ki, this.pidVZ.init.kd,
      this.pidVZ.pid.dt, POSITION_RATE, POSITION_LPF_CUTOFF_FREQ, POSITION_LPF_ENABLE);

  pidSetIntegralLimit(&this.pidVX.pid, this.pidVX.init.iLimit);
  pidSetIntegralLimit(&this.pidVY.pid, this.pidVY.init.iLimit);
  pidSetIntegralLimit(&this.pidVZ.pid, this.pidVZ.init.iLimit);
}

static float sqrtController(float statePos, float setpointPos,
        float setpointVel, float p, float velMax, float accMax,
        float* correction, float* errorTerm)
{
    float x = fabs(setpointPos - statePos);
    /* In the low velocity regime we have linear v-x response */
    float v = x * p;
    /* Return zero velocity output on trivial out-of-context errors */
    if (p <= 0 || velMax <= 0 || accMax <= 0) {
        return 0;
    }
    /* In the high velocity regime we have constant acceleration response */
    if (v > accMax / p) {
        v = sqrt(2 * accMax * x - accMax * accMax / p / p);
    }
    /* set sign of output */
    if (setpointPos < statePos) {
        v *= -1;
    }
    /* store position error term */
    if (errorTerm) {
      *errorTerm = setpointPos - statePos;
    }
    /* store correction term */
    if (correction) {
      *correction = v;
    }
    /* output is velocity feed forward + lin_sqrt correction,
       constrained by maximum allowed velocity */
    return constrain(setpointVel + v, -velMax, velMax);
}

static float runPid(float input, struct pidAxis_s *axis, float setpoint, float dt) {
  axis->setpoint = setpoint;

  pidSetDesired(&axis->pid, axis->setpoint);
  return pidUpdate(&axis->pid, input, true);
}

void positionController(float* thrust, attitude_t *attitude, setpoint_t *setpoint,
                                                             const state_t *state)
{
  float cosyaw = cosf(state->attitude.yaw * (float)M_PI / 180.0f);
  float sinyaw = sinf(state->attitude.yaw * (float)M_PI / 180.0f);
  float bodyvx = setpoint->velocity.x;
  float bodyvy = setpoint->velocity.y;

  // Store the original velocity vector as we need it later for feed-forward
  originalVelocity = setpoint->velocity;

  // X
  if (setpoint->mode.x == modeAbs) {
    setpoint->velocity.x = sqrtController(state->position.x,
            setpoint->position.x, setpoint->velocity.x, this.pX,
            xyVelMax, xyAccMax, 0, 0);
  } else if (setpoint->velocity_body) {
    // TODO: why don't we have a proper controller here?
    setpoint->velocity.x = bodyvx * cosyaw - bodyvy * sinyaw;
  }
  // Y
  if (setpoint->mode.y == modeAbs) {
    setpoint->velocity.y = sqrtController(state->position.y,
            setpoint->position.y, setpoint->velocity.y, this.pY,
            xyVelMax, xyAccMax, 0, 0);
  } else if (setpoint->velocity_body) {
    // TODO: why don't we have a proper controller here?
    setpoint->velocity.y = bodyvy * cosyaw + bodyvx * sinyaw;
  }
  // Z
  if (setpoint->mode.z == modeAbs) {
    setpoint->velocity.z = sqrtController(state->position.z,
            setpoint->position.z, setpoint->velocity.z, this.pZ,
            zVelMax, zAccMax, 0, 0);
  }

  velocityController(thrust, attitude, setpoint, state);
}

void velocityController(float* thrust, attitude_t *attitude, setpoint_t *setpoint,
                                                             const state_t *state)
{
  this.pidVX.pid.outputLimit = rpLimit * rpLimitOverhead;
  this.pidVY.pid.outputLimit = rpLimit * rpLimitOverhead;
  // Set the output limit to the maximum thrust range
  // this.pidVZ.pid.outputLimit = (UINT16_MAX / 2 / thrustScale);
  // this.pidVZ.pid.outputLimit = (this.thrustBase - this.thrustMin) / thrustScale;
  this.pidVZ.pid.outputLimit = (UINT16_MAX - this.thrustBase) / thrustScale;

  // Roll and Pitch
  float rollRaw  = runPid(state->velocity.x, &this.pidVX, setpoint->velocity.x, DT);
  float pitchRaw = runPid(state->velocity.y, &this.pidVY, setpoint->velocity.y, DT);

  // Add feed-forward term (assuming no lateral wind) in the XY plane
  if (setpoint->mode.x == modeAbs && setpoint->mode.y == modeAbs) {
    rollRaw += originalVelocity.x * (this.vXY_ff[0] + originalVelocity.x * this.vXY_ff[1]);
    pitchRaw += originalVelocity.y * (this.vXY_ff[0] + originalVelocity.y * this.vXY_ff[1]);
  }

  float yawRad = state->attitude.yaw * (float)M_PI / 180;
  attitude->pitch = -(rollRaw  * cosf(yawRad)) - (pitchRaw * sinf(yawRad));
  attitude->roll  = -(pitchRaw * cosf(yawRad)) + (rollRaw  * sinf(yawRad));

  attitude->roll  = constrain(attitude->roll,  -rpLimit, rpLimit);
  attitude->pitch = constrain(attitude->pitch, -rpLimit, rpLimit);

  // Thrust
  float thrustRaw = runPid(state->velocity.z, &this.pidVZ, setpoint->velocity.z, DT);
  // Scale the thrust and add feed forward term
  *thrust = thrustRaw * thrustScale + this.thrustBase;
  // Check for minimum thrust
  if (*thrust < this.thrustMin) {
    *thrust = this.thrustMin;
  }
}

void positionControllerResetAllPID()
{
  pidReset(&this.pidVX.pid);
  pidReset(&this.pidVY.pid);
  pidReset(&this.pidVZ.pid);
}

LOG_GROUP_START(posCtl)

LOG_ADD(LOG_FLOAT, targetVX, &this.pidVX.pid.desired)
LOG_ADD(LOG_FLOAT, targetVY, &this.pidVY.pid.desired)
LOG_ADD(LOG_FLOAT, targetVZ, &this.pidVZ.pid.desired)

LOG_ADD(LOG_FLOAT, VXp, &this.pidVX.pid.outP)
LOG_ADD(LOG_FLOAT, VXi, &this.pidVX.pid.outI)
LOG_ADD(LOG_FLOAT, VXd, &this.pidVX.pid.outD)

LOG_ADD(LOG_FLOAT, VYp, &this.pidVY.pid.outP)
LOG_ADD(LOG_FLOAT, VYi, &this.pidVY.pid.outI)
LOG_ADD(LOG_FLOAT, VYd, &this.pidVY.pid.outD)

LOG_ADD(LOG_FLOAT, VZp, &this.pidVZ.pid.outP)
LOG_ADD(LOG_FLOAT, VZi, &this.pidVZ.pid.outI)
LOG_ADD(LOG_FLOAT, VZd, &this.pidVZ.pid.outD)

LOG_GROUP_STOP(posCtl)

PARAM_GROUP_START(velCtlPid)

PARAM_ADD(PARAM_FLOAT, vxKp, &this.pidVX.pid.kp)
PARAM_ADD(PARAM_FLOAT, vxKi, &this.pidVX.pid.ki)
PARAM_ADD(PARAM_FLOAT, vxKd, &this.pidVX.pid.kd)

PARAM_ADD(PARAM_FLOAT, vyKp, &this.pidVY.pid.kp)
PARAM_ADD(PARAM_FLOAT, vyKi, &this.pidVY.pid.ki)
PARAM_ADD(PARAM_FLOAT, vyKd, &this.pidVY.pid.kd)

PARAM_ADD(PARAM_FLOAT, vzKp, &this.pidVZ.pid.kp)
PARAM_ADD(PARAM_FLOAT, vzKi, &this.pidVZ.pid.ki)
PARAM_ADD(PARAM_FLOAT, vzKd, &this.pidVZ.pid.kd)

PARAM_GROUP_STOP(velCtlPid)

PARAM_GROUP_START(posCtlSqrt)

PARAM_ADD(PARAM_UINT16, thrustBase, &this.thrustBase)
PARAM_ADD(PARAM_UINT16, thrustMin, &this.thrustMin)

PARAM_ADD(PARAM_FLOAT, pX, &this.pX)
PARAM_ADD(PARAM_FLOAT, pY, &this.pY)
PARAM_ADD(PARAM_FLOAT, pZ, &this.pZ)

PARAM_ADD(PARAM_FLOAT, vXY_ff0, &this.vXY_ff[0])
PARAM_ADD(PARAM_FLOAT, vXY_ff1, &this.vXY_ff[1])

PARAM_ADD(PARAM_FLOAT, rpLimit,  &rpLimit)
PARAM_ADD(PARAM_FLOAT, xyVelMax, &xyVelMax)
PARAM_ADD(PARAM_FLOAT, zVelMax,  &zVelMax)
PARAM_ADD(PARAM_FLOAT, xyAccMax, &xyAccMax)
PARAM_ADD(PARAM_FLOAT, zAccMax,  &zAccMax)

PARAM_GROUP_STOP(posCtlSqrt)
