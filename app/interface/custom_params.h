/*
 * Parameter updates required for the Crazyflie drone show execution module
 *
 * Copyright (C) 2022 CollMot Robotics. All rights reserved.
 */

#ifndef __DRONE_SHOW_PARAM_CHANGES_H__
#define __DRONE_SHOW_PARAM_CHANGES_H__

#include <stdbool.h>

/**
 * Applies the parameter changes to the current configuration of the drone.
 * Must be called once at boot time.
 * 
 * Returns whether the changes were applied successfully.
 */
bool droneShowApplyCustomParameters(void);

#endif // __DRONE_SHOW_PARAM_CHANGES_H__
