/*
 * Crazyflie preprogrammed light pattern execution module
 *
 * Copyright (C) 2019- CollMot Robotics. All rights reserved.
 */

#ifndef __LIGHT_PROGRAM_H__
#define __LIGHT_PROGRAM_H__

#include <stdint.h>

enum LightProgramLocation_e {
  LIGHT_PROGRAM_LOCATION_INVALID = 0,
  LIGHT_PROGRAM_LOCATION_MEM     = 1,
  // Future features might include light programs on flash or uSD card
};

enum LightProgramType_e {
  LIGHT_PROGRAM_TYPE_RGB = 0,
  LIGHT_PROGRAM_TYPE_RGB565 = 1,
  LIGHT_PROGRAM_TYPE_LEDCTRL = 2,
};

struct lightProgramDescription
{
  uint8_t lightProgramLocation; // one of LightProgramLocation_e
  uint8_t lightProgramType;     // one of LightProgramType_e
  uint8_t fps;                  // frames per second (if applicable)
  union
  {
    struct {
      uint32_t offset;     // offset in light program memory
      uint32_t size;       // size of data in light program memory
    } __attribute__((packed)) mem; // if lightProgramLocation is LIGHT_PROGRAM_LOCATION_MEM
  } lightProgramIdentifier;
} __attribute__((packed));

/* Public functions */

/**
 * Initializes the light program player.
 */
void lightProgramPlayerInit(void);

/**
 * Tests whether the light program playermodule is ready to be used.
 */
bool lightProgramPlayerTest(void);

/**
 * Defines a pre-programmed light sequence based on the contents of the
 * light program memory.
 */
int lightProgramPlayerDefineProgram(uint8_t programId, struct lightProgramDescription description);

/**
 * Evaluates the light program at the current timestamp.
 *
 * \param  color      pointer to a memory location where the evaluated RGB
 *         color should be written (3 bytes)
 */
void lightProgramPlayerEvaluate(uint8_t* color);

/**
 * Evaluates the light program at the given timestamp.
 *
 * \param  t      the number of seconds elapsed since the start of the
 *                light program
 * \param  color  pointer to a memory location where the evaluated RGB
 *         color should be written (3 bytes)
 */
void lightProgramPlayerEvaluateAt(float t, uint8_t* color);

/**
 * Returns whether the light program with the given ID is defined.
 */
bool lightProgramPlayerIsProgramDefined(uint8_t programId);

/**
 * Returns the total number of bytes available for light programs.
 */
uint32_t lightProgramPlayerMemSize();

/**
 * Starts playing a pre-programmed light sequence.
 *
 * @param  programId  index of the light sequence to start playing
 * @param  timescale  time scale; 1 means normal speed, 2 means 2x speedup etc
 */
int lightProgramPlayerPlay(uint8_t programId, float timescale);

/**
 * Pauses the current pre-programmed light sequence.
 */
int lightProgramPlayerPause(void);

/**
 * Resumes playing the current pre-programmed light sequence.
 */
int lightProgramPlayerResume(void);

/**
 * Schedules the player to start playing the given light program from the
 * given timestamp.
 *
 * @param  timestamp  the timestamp when the program should start, in
 *         microseconds.
 * @param  programId  index of the light sequence to start playing
 * @param  timescale  time scale; 1 means normal speed, 2 means 2x speedup etc
 */
int lightProgramPlayerSchedulePlayFrom(uint64_t timestamp, uint8_t programId, float timescale);

/**
 * Stops the current pre-programmed light sequence.
 */
int lightProgramPlayerStop(void);

#endif /* __LIGHT_PROGRAM_H__ */
