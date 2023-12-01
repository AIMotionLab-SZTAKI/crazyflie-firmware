/*
LQR controller.
*/

#include <math.h>

#include "param.h"
#include "log.h"
#include "math3d.h"
#include "stabilizer.h"
#include "physicalConstants.h"
#include "controller_lqr.h"
#include "pm.h"


// Logging variables

static float cmd_thrust_N;
static float cmd_roll;
static float cmd_pitch;
static float cmd_yaw;

static float drone_mass = 0.032;

static float K13 = 2.2115;
static float K16 = 0.7885;
static float K22 = -0.009;
static float K25 = -0.0052;
static float K27 = 0.007; // 0.0105
static float K210 = 0.0015; //
static float K31 = 0.009;
static float K34 = 0.0052;
static float K38 = 0.007; //
static float K311 = 0.0015; // 0.0025
static float K49 = 0.007;
static float K412 = 0.0015;

static float ex, ey, ez, evx, evy, evz, eyaw;

struct vec setpoint_rpy;

void controllerLqrReset(void)
{
  //No integral part to reset
}

void controllerLqrInit(void)
{
  controllerLqrReset();
}

bool controllerLqrTest(void)
{
  return true;
}

void controllerLqr(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{
  if (!RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
      return;
    }

  //Enter force-torque control
  control->controlMode = controlModeForceTorque;

  float vehicleWeight_N = drone_mass * GRAVITY_MAGNITUDE; //in N
 
  //Position in m, velocity in m/s
  struct vec setpointPos = mkvec(setpoint->position.x, setpoint->position.y, setpoint->position.z);
  struct vec setpointVel = mkvec(setpoint->velocity.x, setpoint->velocity.y, setpoint->velocity.z);

  // Angle setpoints are not calculated in pptraj.c
  struct vec zB = vnormalize(mkvec(setpoint->acceleration.x, setpoint->acceleration.y, setpoint->acceleration.z + GRAVITY_MAGNITUDE));
  struct vec xC = mkvec(cosf(radians(setpoint->attitude.yaw)), sinf(radians(setpoint->attitude.yaw)), 0);
  struct vec yB = vnormalize(vcross(zB, xC));
  struct vec xB = vcross(yB, zB);
  struct mat33 Rd = mcolumns(xB, yB, zB);
  setpoint_rpy = quat2rpy(mat2quat(Rd));

  // Compute error vector: drone position, swing angle
  ex = state->position.x - setpointPos.x;
  ey = state->position.y - setpointPos.y;
  ez = state->position.z - setpointPos.z;

  evx = state->velocity.x - setpointVel.x;
  evy = state->velocity.y - setpointVel.y;
  evz = state->velocity.z - setpointVel.z;

  float yaw = radians(state->attitude.yaw);
  float ex_tr = cosf(yaw) * ex + sinf(yaw) * ey;
  float ey_tr = -sinf(yaw) * ex + cosf(yaw) * ey;
  float evx_tr = cosf(yaw) * evx + sinf(yaw) * evy;
  float evy_tr = -sinf(yaw) * evx + cosf(yaw) * evy;

  float eroll = radians(state->attitude.roll);// - setpoint_rpy.x;
  float epitch = -radians(state->attitude.pitch);// - setpoint_rpy.y;
  eyaw = radians(state->attitude.yaw) - setpoint_rpy.z;
  while (eyaw > M_PI_F) {
    eyaw -= 2 * M_PI_F;
  }
  while (eyaw < -M_PI_F) {
    eyaw += 2 * M_PI_F;
  }

  // Angular velocity setpoints are calculated in pptraj.c
  float ewx = radians(sensors->gyro.x);// - setpoint->attitudeRate.roll;
  float ewy = radians(sensors->gyro.y);// - setpoint->attitudeRate.pitch;
  float ewz = radians(sensors->gyro.z) - radians(setpoint->attitudeRate.yaw);

  // Compute control inputs: u = -K * (x - x_r) + u_r
  cmd_thrust_N = -K13 * ez - K16 * evz + vehicleWeight_N; 
  cmd_roll = -K22 * ey_tr - K25 * evy_tr - K27 * eroll - K210 * ewx;
  cmd_pitch = -K31 * ex_tr - K34 * evx_tr - K38 * epitch - K311 * ewy;
  cmd_yaw = -K49 * eyaw - K412 * ewz;

  control->thrustSi = cmd_thrust_N;
  if (setpoint->mode.z == modeDisable) {
    control->thrustSi = setpoint->thrust;
  }
  if(control->thrustSi > 0){
    control->torqueX = cmd_roll;
    control->torqueY = cmd_pitch;
    control->torqueZ = cmd_yaw;
  } else {
    control->torqueX = 0;
    control->torqueY = 0;
    control->torqueZ = 0;
  }
}


PARAM_GROUP_START(Lqr)
PARAM_ADD(PARAM_FLOAT, drone_mass, &drone_mass)
PARAM_GROUP_STOP(Lqr)


LOG_GROUP_START(Lqr)
LOG_ADD(LOG_FLOAT, cmd_thrust_N, &cmd_thrust_N)
LOG_ADD(LOG_FLOAT, cmd_pitch, &cmd_pitch)
LOG_ADD(LOG_FLOAT, cmd_roll, &cmd_roll)
LOG_ADD(LOG_FLOAT, ex, &ex)
LOG_ADD(LOG_FLOAT, evx, &evx)
LOG_ADD(LOG_FLOAT, ey, &ey)
LOG_ADD(LOG_FLOAT, evy, &evy)
LOG_ADD(LOG_FLOAT, ez, &ez)
LOG_ADD(LOG_FLOAT, evz, &evz)
LOG_ADD(LOG_FLOAT, eyaw, &eyaw)
LOG_ADD(LOG_FLOAT, sroll, &setpoint_rpy.x)
LOG_ADD(LOG_FLOAT, spitch, &setpoint_rpy.y)
LOG_ADD(LOG_FLOAT, syaw, &setpoint_rpy.z)
LOG_GROUP_STOP(Lqr)
