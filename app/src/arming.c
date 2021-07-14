#include "FreeRTOS.h"   /* bool is defined there */

#include "arming.h"
#include "param.h"
#include "system.h"

static bool isInit = false;

static struct {
  paramVarId_t forceArm;
} paramIds;

void armingInit() {
  if (isInit) {
    return;
  }

  /* Retrieve the IDs of the log variables and parameters that we will need */
  paramIds.forceArm = paramGetVarId("system", "forceArm");

  if (!PARAM_VARID_IS_VALID(paramIds.forceArm)) {
    return;
  }

  isInit = true;
}

bool armingTest(void) {
  return isInit;
}

bool armingShouldArmAutomaticallyBeforeTakeoff(void) {
  return true;
}

void armAutomaticallyIfNeeded(void) {
  if (!armingShouldArmAutomaticallyBeforeTakeoff() || systemIsArmed()) {
    return;
  }

  systemSetArmed(true);
}

bool armingShouldDisarmAutomaticallyAfterLanding(void) {
#ifdef START_DISARMED
  return true;
#else
  return false;
#endif
}

void armingForceDisarm() {
  systemSetArmed(false);
  paramSetInt(paramIds.forceArm, 0);
}
