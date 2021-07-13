#define DEBUG_MODULE "APP"

#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"

#include "app.h"
#include "debug.h"
#include "system.h"
#include "static_mem.h"

#include "arming.h"
#include "crtp_drone_show_service.h"
#include "drone_show.h"
#include "gcs_light_effects.h"
#include "light_program.h"
#include "preflight.h"

#ifndef DRONE_SHOW_APP_STACKSIZE
#define DRONE_SHOW_APP_STACKSIZE 300
#endif

#ifndef APP_PRIORITY
#define APP_PRIORITY 0
#endif

static bool isInit = false;

STATIC_MEM_TASK_ALLOC(appTask, DRONE_SHOW_APP_STACKSIZE);

static bool appTest();
static void appTask(void *param);

void appInit()
{
  if (isInit) {
    return;
  }

  // The light program player has to be initialized here, before the system
  // starts because it registers a new memory handler
  lightProgramPlayerInit();

  STATIC_MEM_TASK_CREATE(appTask, appTask, "app", NULL, APP_PRIORITY);

  isInit = true;
}

static bool appTest()
{
  bool pass = isInit;

  pass &= armingTest();
  pass &= preflightTest();
  pass &= gcsLightEffectsTest();
  pass &= lightProgramPlayerTest();
  pass &= droneShowSrvTest();
  pass &= droneShowTest();

  return pass;
}

static void appTask(void *param)
{
  systemWaitStart();

  armingInit();
  preflightInit();
  gcsLightEffectsInit();
  droneShowSrvInit();
  droneShowInit();

  if (appTest()) {
    appMain();
  } else {
    DEBUG_PRINT("Failure during drone show app startup.\n");
  }

  while(1) {
    vTaskDelay(portMAX_DELAY);
  }
}

void appMain() {
  DEBUG_PRINT("Drone show app started.\n");
}

