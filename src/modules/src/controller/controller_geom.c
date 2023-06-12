#define DEBUG_MODULE "CONTROLLER"
#include "debug.h"

#include "FreeRTOS.h"
#include "task.h"

#include <math.h>

#include "param.h"
#include "log.h"
#include "math3d.h"
#include "stabilizer.h"
#include "physicalConstants.h"
#include "controller_geom.h"


// Inertia matrix components
static float Ixx = 0.000014;
static float Izz = 0.0000217;

static float dt = (float)(1.0f/ATTITUDE_RATE);

const static float thrust_scale = 132000; // 119460.0; TODO: find a solution (scale gains somehow)
//const static float mass_thrust = 0.28; //smaller than hover, for slow landing
const static float l = 0.0325; // L/sqrt(2), where L is half prop-to-prop length
const static float b = 0.025; // b/k to scale yaw

static float kr = 1.0;
static float kv = 0.35;
static float kR = 0.01;
static float kw = 0.0023;


static float pitch_q= 0.0;

// Logging variables

static float cmd_thrust_g;
static float cmd_roll;
static float cmd_pitch;
static float cmd_yaw;
static float r_roll;
static float r_pitch;
static float r_yaw;
static float thrust;
static uint8_t setPointMode_x;
static uint8_t setPointMode_y;
static uint8_t setPointMode_z;
static uint8_t setPointMode_roll;
static uint8_t setPointMode_pitch;
static uint8_t setPointMode_yaw;
static uint8_t setPointMode_quat;
static uint8_t ctrlMode;

static struct quat q;
static struct vec eR, ew, wd, er, ev, prev_wd;

static float g_vehicleMass = 0.032;

static float psi = 0;

static poseMeasurement_t load_pose;

void setLoadPose(const poseMeasurement_t *measurement)
{
  load_pose.x = measurement->x;
  load_pose.y = measurement->y;
  load_pose.z = measurement->z;
  load_pose.quat = measurement->quat;
}


void controllerGeomReset(void)
{
  //No integral part to reset
}

void controllerGeomInit(void)
{
  controllerGeomReset();
}

bool controllerGeomTest(void)
{
  return true;
}

void controllerGeom(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{
  if (!RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
      return;
    }


  //Enter force-torque control, as is natural with the geometric control
  control->controlMode = controlModeForceTorque;
  //Log variable
  ctrlMode = control->controlMode;
  //After this, we ought to work only with SI units, as force-torque control takes SI inputs
  float vehicleWeight_N = g_vehicleMass * GRAVITY_MAGNITUDE; //in N

  //Log variables
  setPointMode_x = setpoint->mode.x;
  setPointMode_y = setpoint->mode.y;
  setPointMode_z = setpoint->mode.z;
  setPointMode_roll = setpoint->mode.roll;
  setPointMode_pitch = setpoint->mode.pitch;
  setPointMode_yaw = setpoint->mode.yaw;
  setPointMode_quat = setpoint->mode.quat;

  struct vec r1, r2, r3, M;
  struct mat33 Rd;
 
  //Position in m, velocity in m/s
  struct vec setpointPos = mkvec(setpoint->position.x, setpoint->position.y, setpoint->position.z);
  struct vec setpointVel = mkvec(setpoint->velocity.x, setpoint->velocity.y, setpoint->velocity.z);
  //struct vec setpointPos = mkvec(load_pose.x-0.4f, load_pose.y, load_pose.z-0.05f);
  //struct vec setpointVel = vzero();
  struct vec statePos = mkvec(state->position.x, state->position.y, state->position.z);
  struct vec stateVel = mkvec(state->velocity.x, state->velocity.y, state->velocity.z);

  float yaw_des = 0.0f;
  //Rate-controlled YAW is moving YAW angle setpoint
  if (setpoint->mode.yaw == modeVelocity) {
    //When in velocity mode, integrate the desired yaw
    yaw_des = state->attitude.yaw + setpoint->attitudeRate.yaw * dt; //in degrees
  } else if (setpoint->mode.yaw == modeAbs) {
    yaw_des = setpoint->attitude.yaw; //in degrees
  } else if (setpoint->mode.quat == modeAbs) {
    struct quat setpoint_quat = mkquat(setpoint->attitudeQuaternion.x, setpoint->attitudeQuaternion.y, setpoint->attitudeQuaternion.z, setpoint->attitudeQuaternion.w);
    struct vec rpy = quat2rpy(setpoint_quat);
    yaw_des = degrees(rpy.z); //in degrees
  } else if (setpoint -> mode.yaw == modeDisable) {
    yaw_des = 0.0;
  }
  yaw_des = radians(yaw_des); //working with radians from now on

  // Position Error [m]
  er = vsub(statePos, setpointPos);

  // Velocity Error [m/s]
  ev = vsub(stateVel, setpointVel);

  //Construct thrust vector target_thrust [N]
  struct vec target_thrust = vzero();
  if (setpoint -> mode.x == modeDisable) {
    target_thrust.x = -sinf(radians(setpoint->attitude.pitch))*vehicleWeight_N;
    target_thrust.y = -sinf(radians(setpoint->attitude.roll))*vehicleWeight_N;
    if (setpoint->mode.z == modeAbs) {
      target_thrust.z = -kr*er.z - kv*ev.z + g_vehicleMass * GRAVITY_MAGNITUDE;
    }
    else {
      target_thrust.z = vehicleWeight_N;
    }
  } else {
    //In normal operation, this is the code path that we follow
    target_thrust.x = -kr*er.x - kv*ev.x + g_vehicleMass*setpoint->acceleration.x;
    target_thrust.y = -kr*er.y - kv*ev.y + g_vehicleMass*setpoint->acceleration.y;
    target_thrust.z = -kr*er.z - kv*ev.z + g_vehicleMass*(setpoint->acceleration.z + GRAVITY_MAGNITUDE);
  }  
  //Because the desired pose is not explicitly given (only in yaw), we construct it using
  //the differential flatness of the trajectory, from the yaw
  r3 = vnormalize(target_thrust);
  r2 = vnormalize(vcross(r3,mkvec(cosf(yaw_des), sinf(yaw_des), 0)));
  r1 = vcross(r2, r3);
  Rd = mcolumns(r1, r2, r3);

  wd = mkvec(radians(setpoint->attitudeRate.roll), radians(setpoint->attitudeRate.pitch), radians(setpoint->attitudeRate.yaw));
  
  //State estimate quaterion and the rotational matrix resulting from it
  q = mkquat(state->attitudeQuaternion.x, state->attitudeQuaternion.y, state->attitudeQuaternion.z, state->attitudeQuaternion.w);
  pitch_q = degrees(quat2rpy(q).y);
  
  /*
  struct vec rpy = mkvec(radians(state->attitude.roll), -radians(state->attitude.pitch), radians(state->attitude.yaw));
  struct mat33 R = quat2rotmat(rpy2quat(rpy));
  */

  struct mat33 R = quat2rotmat(q);


  struct mat33 eR1 = mmul(mtranspose(Rd),R);
  struct mat33 eR2 = mmul(mtranspose(R),Rd);
  struct mat33 tr_tmp = msub(meye(),eR1);
  //Attitude error function
  psi = 0.5f*(tr_tmp.m[0][0] + tr_tmp.m[1][1] + tr_tmp.m[2][2]);

  //Attitude tracking error
  struct mat33 eRm = msub(eR1,eR2);
  eR.x = 0.5f*eRm.m[2][1];
  eR.y = 0.5f*eRm.m[0][2];
  eR.z = 0.5f*eRm.m[1][0];

  //Desired and state attitude rate [rad/sec], pitch direction negative due to quirky and random crazyflie coordinate system
  struct vec w = mkvec(radians(sensors->gyro.x), radians(sensors->gyro.y), radians(sensors->gyro.z));

  struct vec w_target = mvmul(eR2,wd);

  //Attitude rate error [rad/s]
  ew = vsub(w,w_target);

  struct vec beta_desired = vzero();
  if (prev_wd.x == prev_wd.x) { //d part initialized
    beta_desired = vsub(wd, prev_wd);
    beta_desired.x = beta_desired.x/dt;
    beta_desired.y = beta_desired.y/dt;
    beta_desired.z = beta_desired.z/dt;
  }
  prev_wd = wd;

  struct vec diff_part = vsub(vcross(w, mvmul(eR2, wd)), mvmul(eR2, beta_desired));
  diff_part.x = Ixx*diff_part.x;
  diff_part.y = Ixx*diff_part.y;
  diff_part.z = Izz*diff_part.z;

  //Torque component relating to angular acceleration
  struct vec cross = vcross(w, mkvec(Ixx*w.x, Ixx*w.x, Izz*w.z));
  //Torque in each direction [Nm] Uncapped
  M.x = cross.x - kR * eR.x - kw * ew.x - diff_part.x;
  M.y = cross.y - kR * eR.y - kw * ew.y - diff_part.y;
  M.z = cross.z - kR * eR.z - kw * ew.z - diff_part.z;

  //Thrust pointing in crazyflie body frame Z direction  
  thrust = vdot(target_thrust, mcolumn(R,2));
  
  /*I'm not sure yet how to handle the fork below*/
  if (setpoint->mode.z == modeDisable) {
    control->thrustSi = setpoint->thrust;
  } else {
    control->thrustSi = thrust;
  }

  //Log variables
  cmd_thrust_g = control->thrustSi;
  r_roll = radians(sensors->gyro.x);
  r_pitch = radians(sensors->gyro.y);
  r_yaw = radians(sensors->gyro.z);

  if (control->thrustSi > 0) {
    control->torqueX = M.x;
    control->torqueY = M.y;
    control->torqueZ = M.z;

  } else {
    control->torqueX = 0;
    control->torqueY = 0;
    control->torqueZ = 0;
  }
  //Log variables
  cmd_roll = control->torqueX;
  cmd_pitch = control->torqueY;
  cmd_yaw = control -> torqueZ;  

}

float getPsi(){
  return psi;
}


PARAM_GROUP_START(ctrlGeom)
PARAM_ADD(PARAM_FLOAT, kr, &kr)
PARAM_ADD(PARAM_FLOAT, kv, &kv)
PARAM_ADD(PARAM_FLOAT, kR, &kR)
PARAM_ADD(PARAM_FLOAT, kw, &kw)
PARAM_ADD(PARAM_FLOAT, mass, &g_vehicleMass)
PARAM_GROUP_STOP(ctrlGeom)


LOG_GROUP_START(ctrlGeom)
LOG_ADD(LOG_FLOAT, cmd_thrust_g, &cmd_thrust_g)
LOG_ADD(LOG_FLOAT, cmd_roll, &cmd_roll)
LOG_ADD(LOG_FLOAT, cmd_pitch, &cmd_pitch)
LOG_ADD(LOG_FLOAT, cmd_yaw, &cmd_yaw)
LOG_ADD(LOG_FLOAT, r_roll, &r_roll)
LOG_ADD(LOG_FLOAT, r_pitch, &r_pitch)
LOG_ADD(LOG_FLOAT, thrust, &thrust)
LOG_ADD(LOG_FLOAT, qw, &q.w)
LOG_ADD(LOG_FLOAT, erx, &er.x)
LOG_ADD(LOG_FLOAT, ery, &er.y)
LOG_ADD(LOG_FLOAT, erz, &er.z)
LOG_ADD(LOG_FLOAT, evx, &ev.x)
LOG_ADD(LOG_FLOAT, evy, &ev.y)
LOG_ADD(LOG_FLOAT, evz, &ev.z)
LOG_ADD(LOG_FLOAT, eRx, &eR.x)
LOG_ADD(LOG_FLOAT, eRy, &eR.y)
LOG_ADD(LOG_FLOAT, eRz, &eR.z)
LOG_ADD(LOG_FLOAT, ewx, &ew.x)
LOG_ADD(LOG_FLOAT, ewy, &ew.y)
LOG_ADD(LOG_FLOAT, ewz, &ew.z)
LOG_ADD(LOG_FLOAT, psi, &psi)
LOG_ADD(LOG_FLOAT, pitch_q, &pitch_q)
LOG_ADD(LOG_UINT8, x_Mode, &setPointMode_x)
LOG_ADD(LOG_UINT8, y_Mode, &setPointMode_y)
LOG_ADD(LOG_UINT8, z_Mode, &setPointMode_z)
LOG_ADD(LOG_UINT8, roll_Mode, &setPointMode_roll)
LOG_ADD(LOG_UINT8, pitch_Mode, &setPointMode_pitch)
LOG_ADD(LOG_UINT8, yaw_Mode, &setPointMode_yaw)
LOG_ADD(LOG_UINT8, quat_Mode, &setPointMode_quat)
LOG_ADD(LOG_UINT8, ctrlMode, &ctrlMode)
LOG_ADD(LOG_FLOAT, load_x, &load_pose.x)
LOG_ADD(LOG_FLOAT, load_qx, &load_pose.quat.q0)
LOG_GROUP_STOP(ctrlGeom)
