/*
 * Crazyflie GCS-controlled light effect evaluation module
 *
 * Copyright (C) 2021- CollMot Robotics. All rights reserved.
 */

#include "gcs_light_effects.h"
#include "drone_show.h"

#include <errno.h>
#include <string.h>

#define DEBUG_MODULE "GCS_FX"
#include "debug.h"

static bool isInit = false;

/** Current light effect */
static gcs_light_effect_type_t currentEffect = GCS_LIGHT_EFFECT_OFF;

/** Current color associated to the light effect */
static uint8_t currentColor[3] = { 255, 255, 255 };

void gcsLightEffectsInit(void) {
  if (isInit) {
    return;
  }

  isInit = true;
}

bool gcsLightEffectsTest(void) {
  return isInit;
}

void gcsLightEffectDisable() {
  gcsLightEffectTrigger(GCS_LIGHT_EFFECT_OFF, 0);
}

void gcsLightEffectEvaluate(uint8_t* color) {
  switch (currentEffect) {
    case GCS_LIGHT_EFFECT_SOLID:
      memcpy(color, currentColor, 3);
      break;

    default:
      memset(color, 0, 3);
  }
}

int gcsLightEffectTrigger(gcs_light_effect_type_t effect, const uint8_t* color) {
  switch (effect) {
    case GCS_LIGHT_EFFECT_OFF:
      break;

    case GCS_LIGHT_EFFECT_SOLID:
      break;

    default:
      return ENOEXEC;
  }

  currentEffect = effect;
  if (color) {
    memcpy(currentColor, color, 3);
  }

  // Effect changed so it might happen that the drone show module needs to take
  // control of the LED ring
  droneShowRequestLEDRingControlModeEvaluation();

  return 0;
}

bool areGcsLightEffectsActive(void) {
  return currentEffect != GCS_LIGHT_EFFECT_OFF;
}