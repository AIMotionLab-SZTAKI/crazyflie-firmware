/*
 * Parameter updates required for the Skybrush compatibility layer
 *
 * This file is part of the Skybrush compatibility layer for the Crazyflie firmware.
 *
 * Copyright 2022 CollMot Robotics Ltd.
 *
 * This app is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * This app is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef __SKYBRUSH_PARAM_CHANGES_H__
#define __SKYBRUSH_PARAM_CHANGES_H__

#include <stdbool.h>

/**
 * Applies the parameter changes to the current configuration of the drone.
 * Must be called once at boot time.
 * 
 * Returns whether the changes were applied successfully.
 */
bool droneShowApplyCustomParameters(void);

#endif // __SKYBRUSH_PARAM_CHANGES_H__
