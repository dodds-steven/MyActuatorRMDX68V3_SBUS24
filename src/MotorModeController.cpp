#include <MotorModeController.h>

// Declare global SBUSHandler instance from main.cpp
extern SBUSHandler sbusHandler;

// Constructor: Initializes with motor controller and SBUS channels
MotorModeController::MotorModeController(MotorController& motorController, uint16_t* sbusChannels)
  : motorController(motorController), sbusChannels(sbusChannels), currentMode(STATIC),
    lastModeSwitchTime(0), lastCh3Value(0), shutdownActive(false) {
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    motorPositions[i] = (i == 0 || i == 2) ? -500 : 500; // Initialize to shutdown positions (Motor1/Motor3: -5.00°, Motor2/Motor4: 5.00°)
    lastUpdateTime[i] = 0;
  }
}

// Updates motor positions based on mode and SBUS channels
void MotorModeController::update() {
  uint32_t currentTime = millis();

#if MC_DEBUG_VERBOSE
  Serial.println("Entering MotorModeController::update");
#endif

  // Safety check for sbusChannels
  if (!sbusChannels) {
#if MC_DEBUG_VERBOSE
    Serial.println("Error: sbusChannels is null, skipping update");
#endif
    return;
  }

  // Read SBUS channels
  bool channelsRead = sbusHandler.readChannels(sbusChannels);

#if MC_DEBUG_VERBOSE
  Serial.print("SBUS read: channelsRead=");
  Serial.print(channelsRead);
  if (channelsRead && CONTROL_MODE-1 < SBUS_CHANNELS && STATIC_Y_CHANNEL-1 < SBUS_CHANNELS && STATIC_X_CHANNEL-1 < SBUS_CHANNELS) {
    Serial.print(", CH3=");
    Serial.print(sbusChannels[CONTROL_MODE-1]);
    Serial.print(", CH2/Y=");
    Serial.print(sbusChannels[STATIC_Y_CHANNEL-1]);
    Serial.print(", CH1/X=");
    Serial.print(sbusChannels[STATIC_X_CHANNEL-1]);
    Serial.print(", CH18=");
    Serial.println(sbusChannels[FOOTLIFT_HEIGHT_CHANNEL-1]);
  } else {
    Serial.println(", Invalid channel indices or read failed");
  }
#endif

  // Determine new mode based on CH3 value
  Mode newMode = currentMode;
  uint16_t ch3 = (channelsRead && CONTROL_MODE-1 < SBUS_CHANNELS) ? sbusChannels[CONTROL_MODE-1] : lastCh3Value;
  if (ch3 < STATIC_THRESHOLD) {
    newMode = STATIC;
  } else if (ch3 < MOBILE_THRESHOLD) {
    newMode = MOBILE;
  } else {
    newMode = SHUTDOWN;
  }

  // Handle mode switch with debounce
  if (newMode != currentMode && (currentTime - lastModeSwitchTime) > DEBOUNCE_INTERVAL) {
    currentMode = newMode;
    lastModeSwitchTime = currentTime;
    lastCh3Value = ch3;
    shutdownActive = false;
#if MC_DEBUG_VERBOSE
    Serial.print("Mode switched to: ");
    Serial.println(currentMode == STATIC ? "STATIC" : currentMode == MOBILE ? "MOBILE" : "SHUTDOWN");
#endif
  }

  // Handle SHUTDOWN mode: Issue shutdown commands
  if (currentMode == SHUTDOWN && !shutdownActive) {
#if MC_DEBUG_VERBOSE
    Serial.println("Processing SHUTDOWN mode");
#endif
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
      if (motorController.isCommActive(i) && motorController.isSafeToMove(i)) {
        int32_t shutdownPos = (i == 0 || i == 2) ? motorController.getMaxPos(i) : motorController.getMinPos(i);
        // Check if motor is within 5° of shutdown position (accounting for END_STOP_BUFFER)
        if (abs(motorPositions[i] - shutdownPos) > 500) {
#if MC_DEBUG_VERBOSE
          Serial.print("M");
          Serial.print(i + 1);
          Serial.print(" Attempting shutdown position: Pos=");
          Serial.print(shutdownPos / 100.0f);
          Serial.println("°");
#endif
          if (motorController.setPosition(i, shutdownPos, 500)) {
            motorPositions[i] = shutdownPos;
#if MC_DEBUG_VERBOSE
            Serial.print("M");
            Serial.print(i + 1);
            Serial.print(" Shutdown position set: Pos=");
            Serial.print(shutdownPos / 100.0f);
            Serial.println("°");
#endif
          }
        }
      }
    }
    delay(SHUTDOWN_SETTLE_DELAY);
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
      if (motorController.isCommActive(i) && motorController.isSafeToMove(i)) {
        motorController.shutdownMotor(i);
      }
    }
    shutdownActive = true;
#if MC_DEBUG_VERBOSE
    Serial.println("Shutdown active, motors off until mode change");
#endif
    return;
  }

  // Skip motor updates in SHUTDOWN mode
  if (currentMode == SHUTDOWN) {
    return;
  }

  // Process gimbal and footlift inputs
  float xNorm = 0.0f, yNorm = 0.0f;
  float xNormLeft = 0.0f, yNormLeft = 0.0f;
  float footLiftNorm = 0.0f;
  if (currentMode == STATIC) {
    if (channelsRead && STATIC_X_CHANNEL-1 < SBUS_CHANNELS && STATIC_Y_CHANNEL-1 < SBUS_CHANNELS &&
        sbusChannels[STATIC_X_CHANNEL-1] >= SBUS_MIN && sbusChannels[STATIC_X_CHANNEL-1] <= SBUS_MAX &&
        sbusChannels[STATIC_Y_CHANNEL-1] >= SBUS_MIN && sbusChannels[STATIC_Y_CHANNEL-1] <= SBUS_MAX) {
      xNorm = sbusHandler.normalizeSbus(sbusChannels[STATIC_X_CHANNEL-1]) * 0.9f; // CH1/X, 90% scale (roll)
      yNorm = sbusHandler.normalizeSbus(sbusChannels[STATIC_Y_CHANNEL-1]) * 0.9f; // CH2/Y, 90% scale (pitch)
    }
    if (channelsRead && FOOTLIFT_HEIGHT_CHANNEL-1 < SBUS_CHANNELS &&
        sbusChannels[FOOTLIFT_HEIGHT_CHANNEL-1] >= SBUS_MIN && sbusChannels[FOOTLIFT_HEIGHT_CHANNEL-1] <= SBUS_MAX) {
      footLiftNorm = sbusHandler.normalizeSbus(sbusChannels[FOOTLIFT_HEIGHT_CHANNEL-1]);
    }
#if MC_DEBUG_VERBOSE
    if (!channelsRead || STATIC_X_CHANNEL-1 >= SBUS_CHANNELS || STATIC_Y_CHANNEL-1 >= SBUS_CHANNELS) {
      Serial.println("SBUS read failed or invalid STATIC channels, using xNorm=0, yNorm=0");
    }
    if (!channelsRead || FOOTLIFT_HEIGHT_CHANNEL-1 >= SBUS_CHANNELS) {
      Serial.println("SBUS read failed or invalid FootLift channel, using footLiftNorm=0");
    }
#endif
  } else { // MOBILE
    if (channelsRead && MOBILE_X_CHANNEL-1 < SBUS_CHANNELS && MOBILE_Y_CHANNEL-1 < SBUS_CHANNELS &&
        sbusChannels[MOBILE_X_CHANNEL-1] >= SBUS_MIN && sbusChannels[MOBILE_X_CHANNEL-1] <= SBUS_MAX &&
        sbusChannels[MOBILE_Y_CHANNEL-1] >= SBUS_MIN && sbusChannels[MOBILE_Y_CHANNEL-1] <= SBUS_MAX) {
      xNorm = sbusHandler.normalizeSbus(sbusChannels[MOBILE_X_CHANNEL-1]); // CH1/X
      yNorm = sbusHandler.normalizeSbus(sbusChannels[MOBILE_Y_CHANNEL-1]); // CH2/Y
    }
    if (channelsRead && 5 < SBUS_CHANNELS && 4 < SBUS_CHANNELS &&
        sbusChannels[5] >= SBUS_MIN && sbusChannels[5] <= SBUS_MAX &&
        sbusChannels[4] >= SBUS_MIN && sbusChannels[4] <= SBUS_MAX) {
      xNormLeft = sbusHandler.normalizeSbus(sbusChannels[4]); // CH5/Y (roll)
      yNormLeft = sbusHandler.normalizeSbus(sbusChannels[5]); // CH6/X (pitch)
    }
    if (channelsRead && FOOTLIFT_HEIGHT_CHANNEL-1 < SBUS_CHANNELS &&
        sbusChannels[FOOTLIFT_HEIGHT_CHANNEL-1] >= SBUS_MIN && sbusChannels[FOOTLIFT_HEIGHT_CHANNEL-1] <= SBUS_MAX) {
      footLiftNorm = sbusHandler.normalizeSbus(sbusChannels[FOOTLIFT_HEIGHT_CHANNEL-1]);
    }
#if MC_DEBUG_VERBOSE
    if (!channelsRead || MOBILE_X_CHANNEL-1 >= SBUS_CHANNELS || MOBILE_Y_CHANNEL-1 >= SBUS_CHANNELS) {
      Serial.println("SBUS read failed or invalid MOBILE channels, using xNorm=0, yNorm=0");
    }
    if (!channelsRead || 5 >= SBUS_CHANNELS || 4 >= SBUS_CHANNELS) {
      Serial.println("SBUS read failed or invalid Left Gimbal channels, using xNormLeft=0, yNormLeft=0");
    }
    if (!channelsRead || FOOTLIFT_HEIGHT_CHANNEL-1 >= SBUS_CHANNELS) {
      Serial.println("SBUS read failed or invalid FootLift channel, using footLiftNorm=0");
    }
#endif
  }

#if MC_DEBUG_VERBOSE
  Serial.print("xNorm=");
  Serial.print(xNorm);
  Serial.print(", yNorm=");
  Serial.print(yNorm);
  Serial.print(", xNormLeft=");
  Serial.print(xNormLeft);
  Serial.print(", yNormLeft=");
  Serial.print(yNormLeft);
  Serial.print(", footLiftNorm=");
  Serial.println(footLiftNorm);
#endif

  // Update motor positions for STATIC mode
  if (currentMode == STATIC) {
#if MC_DEBUG_VERBOSE
    Serial.println("Processing STATIC mode");
#endif
    int32_t newPositions[NUM_MOTORS];
    bool shouldUpdate[NUM_MOTORS] = {false};
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
      if (!motorController.isCommActive(i) || !motorController.isSafeToMove(i)) {
        continue;
      }

      if (currentTime - lastUpdateTime[i] < UPDATE_INTERVAL) {
        continue;
      }

      int32_t centerPos = (motorController.getMinPos(i) + motorController.getMaxPos(i)) / 2;
      int32_t range = motorController.getMaxPos(i) - motorController.getMinPos(i);
      float pitchDelta = yNorm; // Forward/aft (CH2/Y in STATIC)
      float rollDelta = (i == 0 || i == 2) ? -xNorm : xNorm; // Left/right (CH1/X in STATIC), reversed polarity
      newPositions[i] = centerPos + (int32_t)((pitchDelta + rollDelta) * range / 2.0f * (motorController.getUpIsPositive(i) ? (i == 2 || i == 3 ? -1 : 1) : (i == 2 || i == 3 ? 1 : -1))) +
                        (int32_t)((-footLiftNorm) * range / 2.0f * (motorController.getUpIsPositive(i) ? 1 : -1));
      newPositions[i] = constrain(newPositions[i], motorController.getMinPos(i), motorController.getMaxPos(i));

#if MC_DEBUG_VERBOSE
      Serial.print("M");
      Serial.print(i + 1);
      Serial.print(" Calculated position: Pos=");
      Serial.print(newPositions[i] / 100.0f);
      Serial.println("°");
#endif

      if (newPositions[i] != motorPositions[i]) {
        shouldUpdate[i] = true;
      }
    }

    // Apply motor updates simultaneously
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
      if (shouldUpdate[i]) {
#if MC_DEBUG_VERBOSE
        Serial.print("M");
        Serial.print(i + 1);
        Serial.print(" Attempting setPosition: Pos=");
        Serial.print(newPositions[i] / 100.0f);
        Serial.println("°");
#endif
        motorController.setPosition(i, newPositions[i], 500);
      }
    }
    // Update positions and timestamps after sending all commands
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
      if (shouldUpdate[i]) {
        motorPositions[i] = newPositions[i];
        lastUpdateTime[i] = currentTime;
#if MC_DEBUG_VERBOSE
        Serial.print("M");
        Serial.print(i + 1);
        Serial.print(" Position set: Pos=");
        Serial.print(newPositions[i] / 100.0f);
        Serial.println("°");
#endif
      }
    }
  } else { // MOBILE
#if MC_DEBUG_VERBOSE
    Serial.println("Processing MOBILE mode");
#endif
    int32_t newPositions[NUM_MOTORS];
    bool shouldUpdate[NUM_MOTORS] = {false};
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
      if (!motorController.isCommActive(i) || !motorController.isSafeToMove(i)) {
        continue;
      }

      if (currentTime - lastUpdateTime[i] < UPDATE_INTERVAL) {
        continue;
      }

      int32_t centerPos = (motorController.getMinPos(i) + motorController.getMaxPos(i)) / 2;
      int32_t range = motorController.getMaxPos(i) - motorController.getMinPos(i);
      float pitchDelta = (yNorm + yNormLeft) * 0.5f; // Forward/aft (CH2/Y + CH6/X)
      float rollDelta = (i == 0 || i == 2) ? -(xNorm + xNormLeft) * 0.5f : -(-(xNorm + xNormLeft) * 0.5f); // Left/right (CH1/X + CH5/Y)
      newPositions[i] = centerPos + (int32_t)((pitchDelta + rollDelta) * range / 2.0f * (motorController.getUpIsPositive(i) ? (i == 2 || i == 3 ? -1 : 1) : (i == 2 || i == 3 ? 1 : -1))) +
                        (int32_t)((-footLiftNorm) * range / 2.0f * (motorController.getUpIsPositive(i) ? 1 : -1));
      newPositions[i] = constrain(newPositions[i], motorController.getMinPos(i), motorController.getMaxPos(i));

#if MC_DEBUG_VERBOSE
      Serial.print("M");
      Serial.print(i + 1);
      Serial.print(" Calculated position: Pos=");
      Serial.print(newPositions[i] / 100.0f);
      Serial.println("°");
#endif

      if (newPositions[i] != motorPositions[i]) {
        shouldUpdate[i] = true;
      }
    }

    // Apply motor updates simultaneously
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
      if (shouldUpdate[i]) {
#if MC_DEBUG_VERBOSE
        Serial.print("M");
        Serial.print(i + 1);
        Serial.print(" Attempting setPosition: Pos=");
        Serial.print(newPositions[i] / 100.0f);
        Serial.println("°");
#endif
        motorController.setPosition(i, newPositions[i], 500);
      }
    }
    // Update positions and timestamps after sending all commands
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
      if (shouldUpdate[i]) {
        motorPositions[i] = newPositions[i];
        lastUpdateTime[i] = currentTime;
#if MC_DEBUG_VERBOSE
        Serial.print("M");
        Serial.print(i + 1);
        Serial.print(" Position set: Pos=");
        Serial.print(newPositions[i] / 100.0f);
        Serial.println("°");
#endif
      }
    }
  }

#if MC_DEBUG_VERBOSE
  Serial.printf("Update time: %dms\n", millis() - currentTime);
  Serial.println("Exiting MotorModeController::update");
#endif
}

// Returns current mode
MotorModeController::Mode MotorModeController::getCurrentMode() {
  return currentMode;
}

// Returns motor position for given index
int32_t MotorModeController::getMotorPosition(uint8_t index) {
  if (index < NUM_MOTORS) {
    return motorPositions[index];
  }
  return 0;
}

// Sets motor position for given index
void MotorModeController::setMotorPosition(uint8_t index, int32_t position) {
  if (index < NUM_MOTORS) {
    motorPositions[index] = position;
  }
}
// File: MotorModeController.cpp (227 lines)