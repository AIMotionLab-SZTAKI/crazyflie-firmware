/*
 * Crazyflie safety fence module
 *
 * Copyright (C) 2021- CollMot Robotics. All rights reserved.
 */

#include <errno.h>
#include <memory.h>

/* FreeRtos includes */
#include "FreeRTOS.h"   /* bool is defined there */
#include "timers.h"

#include "estimator_kalman.h"
#include "fence.h"
#include "log.h"
#include "mem.h"
#include "param.h"
#include "stabilizer_types.h"
#include "worker.h"

#define DEBUG_MODULE "FENCE"
#include "debug.h"

/**
 * Time between consecutive fence breach checks, in milliseconds.
 */
#define FENCE_CHECK_INTERVAL_MSEC 250

/**
 * Minimum duration that the drone needs to stay outside the fence to declare it
 * a fence breach. Must be a multiple of FENCE_CHECK_INTERVAL_MSEC.
 */
#define FENCE_BREACH_MIN_DURATION_MSEC 500

/**
 * Size of the memory segment that can store safety fence data.
 */
#define FENCE_MEMORY_SIZE 64

uint8_t fenceMemory[FENCE_MEMORY_SIZE];

struct fenceDefinition
{
  enum FenceType_e type;
  union
  {
    struct {
      float xMin;
      float yMin;
      float zMin;
      float xMax;
      float yMax;
      float zMax;
    } __attribute__((packed)) axisAlignedBoundingBox;
  } parameters;
};

static struct fenceDefinition activeFence = {
  /* .type = */ FENCE_TYPE_UNLIMITED
};

static StaticTimer_t timerBuffer;

static bool isInit = false;

static uint8_t breachCounter = 0;
static bool isEnabled = false;
static bool isBreached = false;

static void fenceClear();
static void fenceClearCurrentBreach();
static bool fenceIsPointInside(const point_t* p);
static int fenceSetupFromMemory(struct fenceLocationDescription* description);
static void fenceTimer(xTimerHandle timer);
static void fenceWorker(void* data);

// Safety fence memory handling from the memory module
static uint32_t handleMemGetSize(void) { return fenceMemSize(); }
static bool handleMemRead(const uint32_t memAddr, const uint8_t readLen, uint8_t* buffer);
static bool handleMemWrite(const uint32_t memAddr, const uint8_t writeLen, const uint8_t* buffer);
static const MemoryHandlerDef_t memDef = {
  .type = MEM_TYPE_FENCE,
  .getSize = handleMemGetSize,
  .read = handleMemRead,
  .write = handleMemWrite,
};

void fenceInit() {
  if (isInit) {
    return;
  }

  xTimerHandle timer = xTimerCreateStatic("fenceTimer", M2T(FENCE_CHECK_INTERVAL_MSEC), pdTRUE, NULL, fenceTimer, &timerBuffer);
  xTimerStart(timer, FENCE_CHECK_INTERVAL_MSEC);

  memoryRegisterHandler(&memDef);

  /* Testing */
  activeFence.type = FENCE_TYPE_AXIS_ALIGNED_BOUNDING_BOX;
  activeFence.parameters.axisAlignedBoundingBox.xMin = -1;
  activeFence.parameters.axisAlignedBoundingBox.yMin = -1;
  activeFence.parameters.axisAlignedBoundingBox.zMin = -1000;
  activeFence.parameters.axisAlignedBoundingBox.xMax = 1;
  activeFence.parameters.axisAlignedBoundingBox.yMax = 1;
  activeFence.parameters.axisAlignedBoundingBox.zMax = 1000;

  isInit = true;
}

bool fenceTest(void) {
  return isInit;
}

/**
 * Defines the safety fence based on the contents of the fence memory.
 */
int fenceSetup(struct fenceLocationDescription* description) {
  int result = EINVAL;

  switch (description->fenceLocation) {
    case FENCE_LOCATION_INVALID:
      fenceClear();
      result = 0;
      break;

    case FENCE_LOCATION_MEM:
      result = fenceSetupFromMemory(description);
      break;

    default:
      break;
  }

  if (result == 0) {
    /* fence was updated so clear the current breach */
    fenceClearCurrentBreach();
  }

  return result;
}

/* TODO(ntamas): generate these with macros to reduce code duplication between
 * here and light_program.c */

uint32_t fenceMemSize() {
  return sizeof(fenceMemory);
}

bool fenceIsEnabled() {
  return isEnabled;
}

bool fenceIsBreached() {
  return isBreached;
}

static void fenceClear() {
  memset(&activeFence, 0, sizeof(activeFence));
  activeFence.type = FENCE_TYPE_UNLIMITED;
}

static void fenceClearCurrentBreach() {
  breachCounter = 0;
  isBreached = false;
}

/**
 * Returns whether the given point is inside the active geofence.
 */
static bool fenceIsPointInside(const point_t* p) {
  switch (activeFence.type) {
    case FENCE_TYPE_UNLIMITED:
      return true;

    case FENCE_TYPE_ALWAYS_BREACHED:
      return false;

    case FENCE_TYPE_AXIS_ALIGNED_BOUNDING_BOX:
      return (
        p->x >= activeFence.parameters.axisAlignedBoundingBox.xMin &&
        p->x <= activeFence.parameters.axisAlignedBoundingBox.xMax &&
        p->y >= activeFence.parameters.axisAlignedBoundingBox.yMin &&
        p->y <= activeFence.parameters.axisAlignedBoundingBox.yMax &&
        p->z >= activeFence.parameters.axisAlignedBoundingBox.zMin &&
        p->z <= activeFence.parameters.axisAlignedBoundingBox.zMax
      );

    default:
      return true;
  }
}

/**
 * Sets up a geofence from a description stored within the Crazyflie memory
 * subsystem.
 */
static int fenceSetupFromMemory(struct fenceLocationDescription* description) {
  struct fenceDefinition newFence;
  uint32_t offset, size;
  
  ASSERT(description->fenceLocation == FENCE_LOCATION_MEM);

  offset = description->fenceIdentifier.mem.offset;
  size = description->fenceIdentifier.mem.size;

  if (size < 1) {
    return EIO;
  }

  memset(&newFence, 0, sizeof(newFence));

  /* read the type of the fence first */
  if (!handleMemRead(offset, sizeof(newFence.type), &newFence.type)) {
    return EIO;
  }

  offset++;
  size--;

  switch (newFence.type) {
    case FENCE_TYPE_ALWAYS_BREACHED:
    case FENCE_TYPE_UNLIMITED:
      /* these are easy, check the size only */
      if (size != 0) {
        return EIO;
      }
      break;

    case FENCE_TYPE_AXIS_ALIGNED_BOUNDING_BOX:
      if (size != sizeof(newFence.parameters.axisAlignedBoundingBox)) {
        return EIO;
      }

      if (!handleMemRead(offset, size, (uint8_t*) &newFence.parameters.axisAlignedBoundingBox)) {
        return EIO;
      }
      break;

    default:
      return EINVAL;
  }

  /* activate the new fence */
  activeFence = newFence;

  return 0;
}

/**
 * Timer function that is called regularly (2 times every second by default).
 * This function executes the fence checks and sets the corresponding log
 * variable appropriately.
 */
static void fenceTimer(xTimerHandle timer) {
  workerSchedule(fenceWorker, NULL);
}

/**
 * Worker function that is called regularly on the worker thread. This
 * function is responsible for checking whether the fence has been breached.
 */
static void fenceWorker(void* data) {
  point_t pos;

  if (!isEnabled) {
    fenceClearCurrentBreach();
    return;
  }

  estimatorKalmanGetEstimatedPos(&pos);
  
  if (fenceIsPointInside(&pos)) {
    fenceClearCurrentBreach();
  } else {
    if (breachCounter <= (FENCE_BREACH_MIN_DURATION_MSEC - 1) / FENCE_CHECK_INTERVAL_MSEC) {
      breachCounter++;
    } else {
      isBreached = true;
    }
  }
}

static bool handleMemRead(const uint32_t memAddr, const uint8_t readLen, uint8_t* buffer) {
  bool result = false;

  if (memAddr + readLen <= sizeof(fenceMemory)) {
    memcpy(buffer, &(fenceMemory[memAddr]), readLen);
    result = true;
  }

  return result;
}

static bool handleMemWrite(const uint32_t memAddr, const uint8_t writeLen, const uint8_t* buffer) {
  bool result = false;

  if ((memAddr + writeLen) <= sizeof(fenceMemory)) {
    memcpy(&(fenceMemory[memAddr]), buffer, writeLen);
    result = true;
  }

  return result;
}

PARAM_GROUP_START(fence)
PARAM_ADD(PARAM_UINT8, enabled, &isEnabled)
PARAM_GROUP_STOP(fence)

LOG_GROUP_START(fence)
LOG_ADD(LOG_UINT8, breached, &isBreached)
LOG_GROUP_STOP(fence)
