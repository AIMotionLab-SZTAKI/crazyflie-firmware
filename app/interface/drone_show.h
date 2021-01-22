/*
 * Crazyflie drone show execution module
 *
 * Copyright (C) 2019- CollMot Robotics. All rights reserved.
 */

#ifndef __DRONE_SHOW_H__
#define __DRONE_SHOW_H__

typedef enum {
  STATE_INITIALIZING = 0,
  STATE_IDLE,
  STATE_WAIT_FOR_PREFLIGHT_CHECK,
  STATE_WAIT_FOR_START_SIGNAL,
  STATE_WAIT_FOR_TAKEOFF_TIME,
  STATE_TAKEOFF,
  STATE_PERFORMING_SHOW,
  STATE_LANDING,
  STATE_LANDED,

  /* Error states follow here */
  STATE_LANDING_LOW_BATTERY,
  STATE_EXHAUSTED,
  STATE_ERROR,

  /* Sentinel marker so we know how many states we have */
  NUM_STATES
} show_state_t;

/**
 * Initializes the drone show execution module.
 */
void droneShowInit(void);

/**
 * Tests whether the drone show module is ready to be used.
 */
bool droneShowTest(void);

/**
 * Returns the last RGB color emitted by the drone show execution module.
 */
const uint8_t* droneShowGetLastColor(void);

/**
 * Returns the current state of the drone show execution module.
 */
show_state_t droneShowGetState(void);

/**
 * Retufns whether the drone show execution module is enabled.
 */
bool droneShowIsEnabled(void);

/**
 * Returns whether the drone is probably airborne (or at least it is supposed
 * to be, judging from the current state of the show execution modle).
 */
bool droneShowIsProbablyAirborne(void);

/**
 * Returns whether the drone show execution module is in testing mode.
 */
bool droneShowIsInTestingMode(void);

/**
 * Sends a start signal that will trigger the execution of the configured
 * drone show, if the drone show execution module is currently waiting for the
 * start signal or is in the prearm checking phase. Ignored in all other states.
 *
 * This function is safe to be called from any task.
 */
void droneShowStart(void);

/**
 * Sends a start signal that will trigger the execution of the configured
 * drone show after a given delay, if the drone show execution module is
 * currently waiting for the start signal or is in the prearm checking phase.
 * Ignored in all other states.
 *
 * This function is safe to be called from any task.
 */
void droneShowDelayedStart(int16_t delayMsec);

/**
 * Sends a signal that will pause the drone show and cause the drones to
 * float mid-air.
 *
 * This function is safe to be called from any task.
 *
 * NOT IMPLEMENTED YET.
 */
void droneShowPause(void);

/**
 * Sends a signal to the UAV that it should prepare itself for the execution
 * of another flight. This function should be called only after the execution
 * of a flight and a successful landing; it is not handled during a flight
 * (so don't expect the drone to fly back to its starting place).
 */
void droneShowRestart(void);

/**
 * Sends a signal that will stop the drone show immediately and cause the drones
 * to land where they are.
 *
 * This function is safe to be called from any task.
 */
void droneShowStop(void);

#endif // __DRONE_SHOW_H__
