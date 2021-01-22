/*
 * Crazyflie on-board preflight check module.
 *
 * Copyright (C) 2019- CollMot Robotics. All rights reserved.
 */

#ifndef __PREFLIGHT_H__
#define __PREFLIGHT_H__

/**
 * Enum describing the different results that a preflight check may return.
 * "Off" should be used for tests that are not relevant, "Fail" should be
 * returned for test failures and "Pass" should be returned for successful
 * preflight checks. If a preflight check takes a longer time, the test may
 * also return "Wait".
 */
typedef enum result_e {
  preflightResultOff = 0,
  preflightResultFail = 1,
  preflightResultWait = 2,
  preflightResultPass = 3
} preflight_check_result_t;

/**
 * Variable type that contains the result of all preflight checks. The results
 * are filled in from the LSB; each result is stored in two bits. In other
 * words, the result of the first test is stored in bits 0-1, the result of the
 * second test is stored in bits 2-3 and so on.
 */
typedef uint16_t preflight_check_status_t;

/**
 * Initializer function that should be called early during the startup process.
 */
void preflightInit(void);

/**
 * Tests whether the preflight module is ready to be used.
 */
bool preflightTest(void);

/**
 * Sets whether the preflight module is enabled.
 */
void preflightSetEnabled(bool value);

/**
 * Returns the detailed results of the preflight tests.
 */
preflight_check_status_t getPreflightCheckStatus();

/**
 * Returns the detailed result of a single preflight tests.
 */
preflight_check_result_t getSinglePreflightCheckStatus(uint8_t index);

/**
 * Returns a summary of the preflight tests.
 *
 * If there are no preflight tests enabled, the result will be "Off".
 * Otherwise, when at least one test is failing, the result will be "Fail".
 * Otherwise, when at least one test is still in progress, the result will be
 * "Wait". In all other cases, the result will be "Pass".
 */
preflight_check_result_t getPreflightCheckSummary();

/**
 * Returns whether a single preflight check is failing.
 */
bool isPreflightCheckFailing(uint8_t index);

/**
 * Returns whether a single preflight check is passing.
 */
bool isPreflightCheckPassing(uint8_t index);

/**
 * Resets the internal Kalman filter to the home position of the drone.
 */
void preflightResetKalmanFilterToHome();

#endif // __PREFLIGHT_H__
