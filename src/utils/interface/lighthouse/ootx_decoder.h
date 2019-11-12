#pragma once

#include <stdbool.h>
#include <stdint.h>


#define OOTX_MAX_FRAME_LENGTH 40

// Content from https://github.com/nairol/LighthouseRedox/blob/master/docs/Base%20Station.md#base-station-info-block
struct ootxDataFrame_s {
  uint16_t protocolVersion:6;
  uint16_t firmwareVersion:10;
  uint32_t id;
  __fp16 phase0;
  __fp16 phase1;
  __fp16 tilt0;
  __fp16 tilt1;
  uint8_t unlockCount;
  uint8_t hwVersion;
  __fp16 curve0;
  __fp16 curve1;
  int8_t accelX;
  int8_t accelY;
  int8_t accelZ;
  __fp16 gibphase0;
  __fp16 gibphase1;
  __fp16 gibmag0;
  __fp16 gibmag1;
  uint8_t mode;
  uint8_t faults;
} __attribute__((packed));

typedef struct ootxDecoderState_s {
  int frameLength;
  int bytesReceived;

  uint16_t currentWord;

  uint32_t crc32;

  int bitInWord;
  int wordReceived;
  bool synchronized;
  int nZeros;
  enum {rxLength, rxData, rxCrc0, rxCrc1, rxDone} rxState;

  union {
    uint16_t data[(OOTX_MAX_FRAME_LENGTH+1) / 2];
    struct ootxDataFrame_s frame;
  };
} ootxDecoderState_t;

/**
 * @brief Process the next OOTX bit and indicate if a frame has been decoded
 *
 * If a frame is decoded, this function returns true and the
 * frame is available in state->frame
 *
 * Currently the CRC32 checksum of the frame is not checked!
 *
 * @param state Pointer to the decoder state
 * @param data OOTX data bit, should be 0 or 1
 * @return true If a frame has been decoded (ie at the last bit of the frame)
 * @return false If no frame has been decoded yet
 */
bool ootxDecoderProcessBit(ootxDecoderState_t * state, int data);
