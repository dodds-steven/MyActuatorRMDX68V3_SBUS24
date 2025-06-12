#ifndef HANDLE_SBUS_H
#define HANDLE_SBUS_H

#include <Arduino.h>
#include "SBUS.h"

#define SBUS_RX_PIN 0
#define SBUS_TX_PIN 1
#define SBUS_BAUD 100000
#define X_CHANNEL 10
#define Y_CHANNEL 9
#define SBUS_MIN 172
#define SBUS_MAX 1811
#define SBUS_CENTER 992
#define SBUS_DEADZONE 0.05
#define SBUS_STARTUP_DELAY 2000
#define SBUS_RETRY_INTERVAL 9 // Aligned with 9ms frame rate

class SBUSHandler {
private:
  SBUS sbus;
  uint16_t lastValidChannels[24];
  bool hasValidSBUS;
  uint32_t lastSBUSAttempt;
  uint32_t sbusFailCount;
  bool verboseFeedback;
  uint32_t readSuccessCount;
  uint32_t readFailCount;

public:
  SBUSHandler(bool verbose = false) : sbus(Serial1), hasValidSBUS(false), lastSBUSAttempt(0), sbusFailCount(0), verboseFeedback(verbose), readSuccessCount(0), readFailCount(0) {
    for (uint8_t i = 0; i < 24; i++) lastValidChannels[i] = SBUS_CENTER;
  }

  void begin();
  bool readChannels(float& xNorm, float& yNorm, uint16_t* channels);
  float normalizeSbus(uint16_t sbusValue);
  void printSummary();
};

#endif