#include "HandleSBUS.h"

void SBUSHandler::begin() {
  delay(SBUS_STARTUP_DELAY);
  sbus.begin(SBUS_RX_PIN, SBUS_TX_PIN, true, SBUS_BAUD);
  sbus.setEndPoints(X_CHANNEL, SBUS_MIN, SBUS_MAX);
  sbus.setEndPoints(Y_CHANNEL, SBUS_MIN, SBUS_MAX);
  if (verboseFeedback) {
    Serial.println("SBUS initialized with baud rate 100000, RX pin 0, TX pin 1");
  }
}

bool SBUSHandler::readChannels(float& xNorm, float& yNorm, uint16_t* channels) {
  if (millis() - lastSBUSAttempt < SBUS_RETRY_INTERVAL) {
    return hasValidSBUS;
  }

  int bytesAvailable = Serial1.available();
  if (bytesAvailable > 50) { // Flush at 50 bytes
    int bytesToFlush = bytesAvailable - 25;
    if (verboseFeedback) {
      Serial.printf("SBUS: Flushing %d excess bytes, initial bytes available: %d\n", bytesToFlush, bytesAvailable);
    }
    for (int i = 0; i < bytesToFlush && Serial1.available(); i++) {
      Serial1.read();
    }
    delay(5); // Align with frame boundary
    bytesAvailable = Serial1.available();
  }
  if (verboseFeedback) {
    Serial.printf("SBUS: Attempting read, bytes available: %d\n", bytesAvailable);
  }

  bool failsafe, lostFrame;
  if (sbus.read(&channels[0], &failsafe, &lostFrame)) {
    bool validRead = (channels[X_CHANNEL] >= 0 && channels[X_CHANNEL] <= 2047) &&
                     (channels[Y_CHANNEL] >= 0 && channels[Y_CHANNEL] <= 2047);
    if (validRead && !failsafe) {
      readSuccessCount++;
      sbusFailCount = 0;
      if (verboseFeedback) {
        Serial.printf("SBUS: Read success, failsafe=%d, lostFrame=%d\n", failsafe, lostFrame);
        Serial.print("Raw channels: ");
        for (uint8_t j = 0; j < 24; j++) {
          Serial.print(channels[j]);
          Serial.print(" ");
        }
        Serial.println();
      }
      hasValidSBUS = true;
      memcpy(lastValidChannels, channels, sizeof(uint16_t) * 24);
    } else {
      readFailCount++;
      sbusFailCount++;
      if (verboseFeedback) {
        Serial.printf("SBUS: Invalid read, X=%d, Y=%d, failsafe=%d, retrying %d/%d\n", 
                      channels[X_CHANNEL], channels[Y_CHANNEL], failsafe, sbusFailCount, 5);
      }
    }
  } else {
    readFailCount++;
    sbusFailCount++;
    if (verboseFeedback) {
      Serial.printf("SBUS: Read failed, retrying %d/%d, bytes available: %d\n", 
                    sbusFailCount, 5, Serial1.available());
    }
  }

  if (sbusFailCount > 5) { // Reduced retries to 5
    if (verboseFeedback) {
      Serial.println("SBUS: Read failed too many times, using center position (992, 992)");
    }
    channels[X_CHANNEL] = SBUS_CENTER;
    channels[Y_CHANNEL] = SBUS_CENTER;
    memcpy(lastValidChannels, channels, sizeof(uint16_t) * 24);
    hasValidSBUS = true;
    sbusFailCount = 0;
  }

  lastSBUSAttempt = millis();
  memcpy(channels, lastValidChannels, sizeof(uint16_t) * 24);
  xNorm = normalizeSbus(channels[X_CHANNEL]);
  yNorm = normalizeSbus(channels[Y_CHANNEL]);
  if (verboseFeedback) {
    Serial.printf("SBUS: Normalized X=%f, Y=%f, X_raw=%d, Y_raw=%d\n", 
                  xNorm, yNorm, channels[X_CHANNEL], channels[Y_CHANNEL]);
  }
  return hasValidSBUS;
}

float SBUSHandler::normalizeSbus(uint16_t sbusValue) {
  sbusValue = constrain(sbusValue, SBUS_MIN, SBUS_MAX);
  float norm = (float)(sbusValue - SBUS_CENTER) / (SBUS_MAX - SBUS_CENTER);
  if (abs(norm) < SBUS_DEADZONE) norm = 0.0;
  return constrain(norm, -1.0, 1.0);
}

void SBUSHandler::printSummary() {
  Serial.printf("SBUS: Test summary - Successes: %d, Failures: %d\n", readSuccessCount, readFailCount);
}