#include <MotorModeController.h>
#include <HandleSBUS.h>

#define DEBOUNCE_INTERVAL 200 // ms
#define STATIC_THRESHOLD 500
#define MOBILE_THRESHOLD 1300
#define ROLL_SCALE 1.0f // Sufficient roll movement in MOBILE mode

MotorModeController::MotorModeController(MotorController& motorController, MotorConfig* motors, uint16_t* sbusChannels)
  : motorController(motorController), motors(motors), sbusChannels(sbusChannels), currentMode(STATIC),
    recalibrationInProgress(false), lastModeSwitchTime(0), lastCh3Value(0), shutdownActive(false) {
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    motorPositions[i] = motors[i].centerPos;
  }
}

void MotorModeController::update() {
  Mode newMode = currentMode;
  uint16_t ch3 = sbusChannels[2]; // CH3 for mode control

  if (ch3 < STATIC_THRESHOLD) {
    newMode = STATIC;
  } else if (ch3 < MOBILE_THRESHOLD) {
    newMode = MOBILE;
  } else {
    newMode = SHUTDOWN;
  }

  if (newMode != currentMode && (millis() - lastModeSwitchTime) > DEBOUNCE_INTERVAL) {
    currentMode = newMode;
    lastModeSwitchTime = millis();
    lastCh3Value = ch3;
#if MC_DEBUG_VERBOSE
    Serial.print("Mode switched to: ");
    Serial.println(currentMode == STATIC ? "STATIC" : currentMode == MOBILE ? "MOBILE" : "SHUTDOWN");
#endif
    if (currentMode != SHUTDOWN) {
      shutdownActive = false; // Clear shutdown flag on mode change
    }
  }

  if (currentMode == SHUTDOWN) {
    if (!shutdownActive) {
      // Set shutdown positions and activate shutdown
      for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        int32_t shutdownPos = (i == 0 || i == 2) ? motors[i].maxPos : motors[i].minPos;
        motorPositions[i] = shutdownPos;
        if (motors[i].commActive && motors[i].safeToMove) {
          motorController.setPosition(i, shutdownPos, 500);
#if MC_DEBUG_VERBOSE
          Serial.print("M");
          Serial.print(i + 1);
          Serial.print(" Shutdown position set: Pos=");
          Serial.print(shutdownPos / 100.0);
          Serial.println("°");
#endif
        }
      }
      motorController.shutdown();
      shutdownActive = true; // Set shutdown flag
#if MC_DEBUG_VERBOSE
      Serial.println("Shutdown active, motors off until mode change");
#endif
    }
    return; // Skip gimbal processing in SHUTDOWN mode
  }

  SBUSHandler sbusHandler(false); // VERBOSE_DEBUG = false
  float xNorm, yNorm;
  bool channelsRead = sbusHandler.readChannels(xNorm, yNorm, sbusChannels); // Read channels for current mode
  if (!channelsRead) {
#if MC_DEBUG_VERBOSE
    Serial.println("Failed to read SBUS channels");
#endif
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
      motorPositions[i] = motors[i].centerPos;
    }
    return;
  }

  // Clamp normalized inputs to prevent invalid values
  xNorm = constrain(xNorm, -1.0f, 1.0f);
  yNorm = constrain(yNorm, -1.0f, 1.0f);

#if MC_DEBUG_VERBOSE
  Serial.print("xNorm=");
  Serial.print(xNorm);
  Serial.print(", yNorm=");
  Serial.println(yNorm);
#endif

  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    if (!motors[i].commActive || !motors[i].safeToMove) {
      motorPositions[i] = motors[i].centerPos;
      continue;
    }

    int32_t range = motors[i].maxPos - motors[i].minPos;
    int32_t newPosition;
    if (currentMode == STATIC) {
      if (i == 0 || i == 2) { // Motor1, Motor3 for roll (X)
        newPosition = motors[i].centerPos + (int32_t)(xNorm * range * (motors[i].UpIsPositive ? 1 : -1));
      } else { // Motor2, Motor4 for pitch (Y)
        newPosition = motors[i].centerPos + (int32_t)(yNorm * range * (motors[i].UpIsPositive ? 1 : -1));
      }
    } else { // MOBILE
      if (i == 0 || i == 2) { // Motor1, Motor3 for roll (X)
        newPosition = motors[i].centerPos + (int32_t)(xNorm * range * ROLL_SCALE * (motors[i].UpIsPositive ? 1 : -1));
      } else { // Motor2, Motor4 for pitch (Y)
        newPosition = motors[i].centerPos + (int32_t)(yNorm * range * (motors[i].UpIsPositive ? 1 : -1));
      }
    }
    newPosition = constrain(newPosition, motors[i].minPos, motors[i].maxPos);

#if MC_DEBUG_VERBOSE
    Serial.print("M");
    Serial.print(i + 1);
    Serial.print(" Calculated position: Pos=");
    Serial.print(newPosition / 100.0);
    Serial.println("°");
#endif

    if (motorController.setPosition(i, newPosition, 500)) {
      motorPositions[i] = newPosition;
#if MC_DEBUG_VERBOSE
      Serial.print("M");
      Serial.print(i + 1);
      Serial.print(" Position set: Pos=");
      Serial.print(newPosition / 100.0);
      Serial.println("°");
#endif
    } else {
#if MC_DEBUG_VERBOSE
      Serial.print("M");
      Serial.print(i + 1);
      Serial.println(" Failed to set position");
#endif
    }
    delay(5); // Reduced delay for responsiveness
  }
}

MotorModeController::Mode MotorModeController::getCurrentMode() {
  return currentMode;
}

int32_t MotorModeController::getMotorPosition(uint8_t index) {
  if (index < NUM_MOTORS) {
    return motorPositions[index];
  }
  return 0;
} // File: MotorModeController.cpp (119 lines)