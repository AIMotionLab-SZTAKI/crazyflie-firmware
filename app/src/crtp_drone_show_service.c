#include "FreeRTOS.h"
#include "timers.h"

#include <errno.h>
#include <math.h>
#include <string.h>

#include "arming.h"
#include "commander.h"
#include "crtp.h"
#include "crtp_drone_show_service.h"
#include "drone_show.h"
#include "estimator_kalman.h"
#include "fence.h"
#include "gcs_light_effects.h"
#include "light_program.h"
#include "log.h"
#include "param.h"
#include "pm.h"
#include "preflight.h"
#include "supervisor.h"
#include "system.h"

#define DEBUG_MODULE "SHOW"
#include "debug.h"

#define CONTROL_CH 0

#define CMD_START  0
#define CMD_PAUSE  1
#define CMD_STOP   2
#define CMD_STATUS 3
#define CMD_DEFINE_LIGHT_PROGRAM 4
#define CMD_RESTART 5
#define CMD_TRIGGER_GCS_LIGHT_EFFECT 6
#define CMD_ARM_OR_DISARM 7
#define CMD_DEFINE_GEOFENCE 8

struct data_arm_or_disarm {
  uint8_t arm;  /* 0 = disarm, 1 = arm, 2 = force-disarm, 3 = force-arm (not used yet) */
} __attribute__((packed));

struct data_define_geofence {
  struct fenceLocationDescription description;
} __attribute__((packed));

struct data_define_program {
  uint8_t programId;
  struct lightProgramDescription description;
} __attribute__((packed));

struct data_trigger_light_effect {
  uint8_t effect;
  uint8_t color[3];
} __attribute__((packed));

static bool isInit = false;

static struct {
  logVarId_t stateEstimateYaw;
  logVarId_t ledColorRed;
  logVarId_t ledColorGreen;
  logVarId_t ledColorBlue;
} logIds;
static struct {
  paramVarId_t highLevelCommanderEnabled;
} paramIds;

static void droneShowSrvCrtpCB(CRTPPacket* pk);
static void handleArmOrDisarmCommandPacket(CRTPPacket* pk);
static void handleDefineGeofencePacket(CRTPPacket* pk);
static void handleDefineLightProgramPacket(CRTPPacket* pk);
static void handleTriggerGcsLightEffectPacket(CRTPPacket* pk);
static void updatePacketWithStatusInformation(CRTPPacket* pk);

void droneShowSrvInit() {
  if (isInit) {
    return;
  }

  /* Retrieve the IDs of the log variables and parameters that we will need */
  logIds.ledColorRed = logGetVarId("show", "ledColorRed");
  logIds.ledColorGreen = logGetVarId("show", "ledColorGreen");
  logIds.ledColorBlue = logGetVarId("show", "ledColorBlue");
  logIds.stateEstimateYaw = logGetVarId("stateEstimate", "yaw");
  paramIds.highLevelCommanderEnabled = paramGetVarId("commander", "enHighLevel");

  if (
    !logVarIdIsValid(logIds.stateEstimateYaw) ||
    !PARAM_VARID_IS_VALID(paramIds.highLevelCommanderEnabled)
  ) {
    return;
  }

  crtpRegisterPortCB(CRTP_PORT_APP, droneShowSrvCrtpCB);

  isInit = true;
}

bool droneShowSrvTest(void) {
  return isInit;
}

static void droneShowSrvProcessControlPacket(CRTPPacket* pk) {
  if (pk->size < 1) {
    return;
  }

  /* commands are acknowledged by sending the same packet back.
   * status requests are responded to in the same packet */

  switch (pk->data[0]) {
    case CMD_START:
      if (pk->size > 2) {
        droneShowDelayedStart(*(int16_t*)(pk->data + 1));
      } else {
        droneShowStart();
      }
      pk->size = 1;
      break;

    case CMD_PAUSE:
      droneShowPause();
      break;

    case CMD_STOP:
      droneShowStop();
      break;

    case CMD_STATUS:
      updatePacketWithStatusInformation(pk);
      break;

    case CMD_RESTART:
      droneShowRestart();
      break;

    case CMD_DEFINE_LIGHT_PROGRAM:
      handleDefineLightProgramPacket(pk);
      break;

    case CMD_TRIGGER_GCS_LIGHT_EFFECT:
      handleTriggerGcsLightEffectPacket(pk);
      break;

    case CMD_ARM_OR_DISARM:
      handleArmOrDisarmCommandPacket(pk);
      break;

    case CMD_DEFINE_GEOFENCE:
      handleDefineGeofencePacket(pk);
      break;

    default:
      return;
  }

  crtpSendPacket(pk);
}

static void droneShowSrvCrtpCB(CRTPPacket* pk) {
  if (pk->channel == CONTROL_CH) {
    droneShowSrvProcessControlPacket(pk);
  }
}

static void handleArmOrDisarmCommandPacket(CRTPPacket* pk) {
  struct data_arm_or_disarm data = *((struct data_arm_or_disarm*)(pk->data + 1));
  
  /* put the response in the packet and trim it */
  if (pk->size >= sizeof(struct data_arm_or_disarm) + 1) {
    bool shouldBeArmed = data.arm & 1;
    bool forced = data.arm & 2;

    systemSetArmed(shouldBeArmed);
    if (!shouldBeArmed && forced) {
      armingForceDisarm();
    }

    pk->size = 3;
    pk->data[2] = 0;
  } else {
    pk->size = 3;
    pk->data[2] = EINVAL;
  }
}

static void handleDefineGeofencePacket(CRTPPacket* pk) {
  struct data_define_geofence data = *((struct data_define_geofence*)(pk->data + 1));

  /* put the response in the packet and trim it */
  if (pk->size >= sizeof(struct data_define_geofence) + 1) {
    pk->size = 3;
    pk->data[2] = fenceSetup(&data.description);
  } else {
    pk->size = 3;
    pk->data[2] = EINVAL;
  }
}

static void handleDefineLightProgramPacket(CRTPPacket* pk) {
  struct data_define_program data = *((struct data_define_program*)(pk->data + 1));

  /* put the response in the packet and trim it */
  if (pk->size >= sizeof(struct data_define_program) + 1) {
    pk->size = 3;
    pk->data[2] = lightProgramPlayerDefineProgram(data.programId, data.description);
  } else {
    pk->size = 3;
    pk->data[2] = EINVAL;
  }
}

static void handleTriggerGcsLightEffectPacket(CRTPPacket* pk) {
  struct data_trigger_light_effect data = *((struct data_trigger_light_effect*)(pk->data + 1));
  uint8_t color[3];

  /* put the response in the packet and trim it */
  if (pk->size >= sizeof(struct data_trigger_light_effect) + 1) {
    color[0] = data.color[0];
    color[1] = data.color[1];
    color[2] = data.color[2];

    pk->size = 3;
    pk->data[2] = gcsLightEffectTrigger(data.effect, color);
  } else {
    pk->size = 3;
    pk->data[2] = EINVAL;
  }
}

static void updatePacketWithStatusInformation(CRTPPacket* pk) {
  static logVarId_t setpointXVarId = 0xffffu, setpointYVarId, setpointZVarId;
  uint16_t voltage = ((int)round(pmGetBatteryVoltage() * 10));
  uint16_t colorAsRGB565;
  const uint8_t* lastColor;
  point_t pos;
  show_state_t show_state;

  pk->size = 16;

  /* clear the packet body */
  memset(pk->data + 1, 0, pk->size - 1);

  /* version number and state */
  show_state = droneShowGetState();
  pk->data[1] = (0 << 4) | (show_state & 0x0F);

  /* battery voltage level, in 1/10 volts */
  pk->data[2] = (voltage > 0xFF) ? 0xFF : voltage;

  /* status flags */
  pk->data[3] = (
    /* is the battery charging? */
    (pmIsCharging() ? 1 : 0) |
    /* is the high level commander turned on? */
    (paramGetUint(paramIds.highLevelCommanderEnabled) ? (1 << 1) : 0) |
    /* is the drone show mode turned on? */
    (droneShowIsEnabled() ? (1 << 2) : 0) |
    /* is the drone flying? */
    (supervisorIsFlying() ? (1 << 3) : 0) |
    /* is the drone show in testing mode? */
    (droneShowIsInTestingMode() ? (1 << 4) : 0) |
    /* is the drone _disarmed_? (backwards compatibility) */
    (systemIsArmed() ? 0 : (1 << 5)) |
    /* is the geofence breached? */
    (fenceIsBreached() ? (1 << 6) : 0)
  );

  /* preflight check status */
  *((preflight_check_status_t*)(pk->data + 4)) = getPreflightCheckStatus();

  /* position */
  if (droneShowIsProbablyAirborne() && droneShowIsInTestingMode()) {
    /* in testing mode, report the position where the drone is supposed to be
     * instead of the current position, for debugging purposes */
    if (setpointXVarId == 0xffffu) {
      setpointXVarId = logGetVarId("ctrltarget", "x");
      setpointYVarId = logGetVarId("ctrltarget", "y");
      setpointZVarId = logGetVarId("ctrltarget", "z");
    }

    pos.x = logGetFloat(setpointXVarId);
    pos.y = logGetFloat(setpointYVarId);
    pos.z = logGetFloat(setpointZVarId);
  } else {
    estimatorKalmanGetEstimatedPos(&pos);
  }
  *((int16_t*)(pk->data + 6)) = (int16_t) (pos.x * 1000);
  *((int16_t*)(pk->data + 8)) = (int16_t) (pos.y * 1000);
  *((int16_t*)(pk->data + 10)) = (int16_t) (pos.z * 1000);

  /* yaw */
  *((int16_t*)(pk->data + 12)) = (int16_t) (logGetFloat(logIds.stateEstimateYaw) * 10);

  /* color of the LED ring, encoded in RGB565 */
  lastColor = droneShowGetLastColor();
  colorAsRGB565 = (
    (((lastColor[0] >> 3) & 0x1f) << 11) |
    (((lastColor[1] >> 2) & 0x3f) << 5) |
    ((lastColor[2] >> 3) & 0x1f)
  );
  *((uint16_t*)(pk->data + 14)) = colorAsRGB565;
}
