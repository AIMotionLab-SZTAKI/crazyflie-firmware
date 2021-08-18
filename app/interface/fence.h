/*
 * Safety fence related functions in the Crazyflie drone show execution module
 *
 * Copyright (C) 2021- CollMot Robotics. All rights reserved.
 */

#ifndef __DRONE_SHOW_FENCE_H__
#define __DRONE_SHOW_FENCE_H__

enum FenceAction_e {
  FENCE_ACTION_NONE = 0,
  FENCE_ACTION_STOP_MOTORS = 1,
  FENCE_ACTION_SHUTDOWN = 2,
  FENCE_ACTION_LAND = 3,
};

enum FenceLocation_e {
  FENCE_LOCATION_INVALID = 0,
  FENCE_LOCATION_MEM     = 1,
  // Future features might include safety fence on flash or uSD card
};

enum FenceType_e {
  FENCE_TYPE_UNLIMITED = 0,
  FENCE_TYPE_ALWAYS_BREACHED = 1,
  FENCE_TYPE_AXIS_ALIGNED_BOUNDING_BOX = 2,
};

struct fenceLocationDescription
{
  uint8_t fenceLocation; // one of FenceLocation_e
  union
  {
    struct {
      uint32_t offset;     // offset in fence memory
      uint32_t size;       // size of data in fence memory
    } __attribute__((packed)) mem; // if fenceLocation is FENCE_LOCATION_MEM
  } fenceIdentifier;
} __attribute__((packed));

/**
 * Initializes the safety fence module.
 */
void fenceInit(void);

/**
 * Tests whether the safety fence module is ready to be used.
 */
bool fenceTest(void);

/**
 * Returns whether the fence is enabled.
 */
bool fenceIsEnabled(void);

/**
 * Returns whether the fence is breached.
 */
bool fenceIsBreached(void);

/**
 * Defines the safety fence based on the contents of the fence memory.
 */
int fenceSetup(struct fenceLocationDescription* description);

/**
 * Returns the total number of bytes available for storing the safety fence.
 */
uint32_t fenceMemSize();

#endif // __DRONE_SHOW_FENCE_H__
