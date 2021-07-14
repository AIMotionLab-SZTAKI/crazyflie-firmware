/*
 * Arming-related functions in the Crazyflie drone show execution module
 *
 * Copyright (C) 2021- CollMot Robotics. All rights reserved.
 */

#ifndef __DRONE_SHOW_ARMING_H__
#define __DRONE_SHOW_ARMING_H__

/**
 * Initializes the arming module.
 */
void armingInit(void);

/**
 * Tests whether the arming module is ready to be used.
 */
bool armingTest(void);

/**
 * Arms the drone if it is set to automatic arming before takeoff and it is
 * not armed yet.
 */
void armAutomaticallyIfNeeded(void);

/**
 * Force-disarm the drone even if it is currently force-armed.
 */
void armingForceDisarm(void);

/**
 * Returns whether the drone arms itself automatically before takeoff.
 */
bool armingShouldArmAutomaticallyBeforeTakeoff(void);

/**
 * Returns whether the drone disarms itself automatically after landing.
 */
bool armingShouldDisarmAutomaticallyAfterLanding(void);

#endif // __DRONE_SHOW_H__
