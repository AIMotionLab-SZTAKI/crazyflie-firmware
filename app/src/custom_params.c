/*
 * Parameter updates required for the Crazyflie drone show execution module
 *
 * Copyright (C) 2022 CollMot Robotics. All rights reserved.
 */

#include "param_logic.h"

#include "custom_params.h"

typedef struct {
  const char* group;
  const char* name;
  float value;
  bool optional;
} customParamTableEntry_t;

#define NO_MORE_ENTRIES { 0, 0, 0 }
#define OPTIONAL 1

#if defined(CF_MODEL_SHOW_PROTO_V1) || defined(CF_MODEL_SHOW_PROTO_V2_LH) || defined(CF_MODEL_SHOW_PROTO_V2_UWB)
#  define DRONE_IS_BASED_ON_BOLT
#endif

static const customParamTableEntry_t params[] = {
  /* Attitude rate controller PID tuning */
#ifdef DRONE_IS_BASED_ON_BOLT
  { "pid_rate", "roll_kp",   70 },
  { "pid_rate", "roll_ki",  200 },
  { "pid_rate", "roll_kd",    2 },
  { "pid_rate", "pitch_kp",  70 },
  { "pid_rate", "pitch_ki", 200 },
  { "pid_rate", "pitch_kd",   2 },
#else
  { "pid_rate", "roll_kp",  220 },
  { "pid_rate", "roll_ki",  500 },
  { "pid_rate", "roll_kd",    2 },
  { "pid_rate", "pitch_kp", 220 },
  { "pid_rate", "pitch_ki", 500 },
  { "pid_rate", "pitch_kd",   2 },
#endif

  /* Attitude controller PID tuning */
#ifdef DRONE_IS_BASED_ON_BOLT
  { "pid_attitude", "roll_kp",  7 },
  { "pid_attitude", "pitch_kp", 7 },
  { "pid_attitude", "roll_ki",  3 },
  { "pid_attitude", "pitch_ki", 3 },
#else
  { "pid_attitude", "roll_kp",  6 },
  { "pid_attitude", "pitch_kp", 6 },
  { "pid_attitude", "roll_ki",  0 },
  { "pid_attitude", "pitch_ki", 0 },
#endif

  /* Position controller PID tuning */
  { "posCtlPid", "xVelMax", 2, OPTIONAL },
  { "posCtlPid", "yVelMax", 2, OPTIONAL },
  { "posCtlPid", "zVelMax", 0.8, OPTIONAL },
  { "posCtlPid", "vxKFF", 1, OPTIONAL },
  { "posCtlPid", "vyKFF", 1, OPTIONAL },

#ifdef DRONE_IS_BASED_ON_BOLT
  /* Larger Bolt-bases show drones are okay with a lower thrust base */
  #error "Lo thrust base!"
  { "posCtlPid", "thrustBase", 30000 }
#endif

  NO_MORE_ENTRIES
};

bool droneShowApplyCustomParameters(void) {
  const customParamTableEntry_t* entry;
  bool allValid = true;

  for (entry = params; entry->group && entry->name; entry++) {
    paramVarId_t paramId = paramGetVarId(entry->group, entry->name);

    if (PARAM_VARID_IS_VALID(paramId)) {
      if (paramGetFloat(paramId) != entry->value) {
        paramSetFloat(paramId, entry->value);
      }
    } else if (!entry->optional) {
      allValid = false;
    }
  }

  return allValid;
}
