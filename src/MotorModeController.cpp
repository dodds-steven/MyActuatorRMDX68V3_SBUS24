#include <MotorModeController.h>

// Declare global SBUSHandler instance from main.cpp
extern SBUSHandler sbusHandler;

// Constructor: Initializes with motor controller and SBUS channels
MotorModeController::MotorModeController(MotorController& motorController, uint16_t* sbusChannels)
  : motorController(motorController), sbusChannels(sbusChannels), currentMode(STATIC),
    lastModeSwitchTime(0), lastCh3Value(0), shutdownActive(false) {
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    motorPositions[i] = 0; // Initialize to zero, set after calibration in main.cpp
    lastUpdateTime[i] = 0;
  }
}

// Updates motor positions based on mode and SBUS inputs
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
    Serial.print(", CH10/Y=");
    Serial.print(sbusChannels[STATIC_Y_CHANNEL-1]);
    Serial.print(", CH11/X=");
    Serial.println(sbusChannels[STATIC_X_CHANNEL-1]);
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

  // Process gimbal inputs for STATIC or MOBILE mode
  float xNorm = 0.0f, yNorm = 0.0f;
  if (currentMode == STATIC) {
    if (channelsRead && STATIC_X_CHANNEL-1 < SBUS_CHANNELS && STATIC_Y_CHANNEL-1 < SBUS_CHANNELS &&
        sbusChannels[STATIC_X_CHANNEL-1] >= SBUS_MIN && sbusChannels[STATIC_X_CHANNEL-1] <= SBUS_MAX &&
        sbusChannels[STATIC_Y_CHANNEL-1] >= SBUS_MIN && sbusChannels[STATIC_Y_CHANNEL-1] <= SBUS_MAX) {
      xNorm = sbusHandler.normalizeSbus(sbusChannels[STATIC_X_CHANNEL-1]); // CH11/X
      yNorm = sbusHandler.normalizeSbus(sbusChannels[STATIC_Y_CHANNEL-1]); // CH10/Y
    }
#if MC_DEBUG_VERBOSE
    if (!channelsRead || STATIC_X_CHANNEL-1 >= SBUS_CHANNELS || STATIC_Y_CHANNEL-1 >= SBUS_CHANNELS) {
      Serial.println("SBUS read failed or invalid STATIC channels, using xNorm=0, yNorm=0");
    }
#endif
  } else { // MOBILE
    if (channelsRead && MOBILE_X_CHANNEL-1 < SBUS_CHANNELS && MOBILE_Y_CHANNEL-1 < SBUS_CHANNELS &&
        sbusChannels[MOBILE_X_CHANNEL-1] >= SBUS_MIN && sbusChannels[MOBILE_X_CHANNEL-1] <= SBUS_MAX &&
        sbusChannels[MOBILE_Y_CHANNEL-1] >= SBUS_MIN && sbusChannels[MOBILE_Y_CHANNEL-1] <= SBUS_MAX) {
      xNorm = sbusHandler.normalizeSbus(sbusChannels[MOBILE_X_CHANNEL-1]); // CH1/X
      yNorm = sbusHandler.normalizeSbus(sbusChannels[MOBILE_Y_CHANNEL-1]); // CH2/Y
    }
#if MC_DEBUG_VERBOSE
    if (!channelsRead || MOBILE_X_CHANNEL-1 >= SBUS_CHANNELS || MOBILE_Y_CHANNEL-1 >= SBUS_CHANNELS) {
      Serial.println("SBUS read failed or invalid MOBILE channels, using xNorm=0, yNorm=0");
    }
#endif
  }

#if MC_DEBUG_VERBOSE
  Serial.print("xNorm=");
  Serial.print(xNorm);
  Serial.print(", yNorm=");
  Serial.println(yNorm);
#endif

  // Update motor positions for STATIC mode
  if (currentMode == STATIC) {
#if MC_DEBUG_VERBOSE
    Serial.println("Processing STATIC mode");
#endif
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
      if (!motorController.isCommActive(i) || !motorController.isSafeToMove(i)) {
        continue;
      }

      if (currentTime - lastUpdateTime[i] < UPDATE_INTERVAL) {
        continue;
      }

      int32_t centerPos = (motorController.getMinPos(i) + motorController.getMaxPos(i)) / 2;
      float delta = (i == 0 || i == 2) ? xNorm : yNorm; // Roll (X) or Pitch (Y)
      int32_t range = motorController.getMaxPos(i) - motorController.getMinPos(i);
      int32_t newPosition = centerPos + (int32_t)(delta * range / 2.0f * (motorController.getUpIsPositive(i) ? 1 : -1));
      newPosition = constrain(newPosition, motorController.getMinPos(i), motorController.getMaxPos(i));

#if MC_DEBUG_VERBOSE
      Serial.print("M");
      Serial.print(i + 1);
      Serial.print(" Calculated position: Pos=");
      Serial.print(newPosition / 100.0f);
      Serial.println("°");
#endif

      if (newPosition != motorPositions[i]) {
#if MC_DEBUG_VERBOSE
        Serial.print("M");
        Serial.print(i + 1);
        Serial.print(" Attempting setPosition: Pos=");
        Serial.print(newPosition / 100.0f);
        Serial.println("°");
#endif
        if (motorController.setPosition(i, newPosition, 500)) {
          motorPositions[i] = newPosition;
          lastUpdateTime[i] = currentTime;
#if MC_DEBUG_VERBOSE
          Serial.print("M");
          Serial.print(i + 1);
          Serial.print(" Position set: Pos=");
          Serial.print(newPosition / 100.0f);
          Serial.println("°");
#endif
        } else {
#if MC_DEBUG_VERBOSE
          Serial.print("M");
          Serial.print(i + 1);
          Serial.println(" Failed to set position");
#endif
        }
      }
    }
  } else { // MOBILE
#if MC_DEBUG_VERBOSE
    Serial.println("Processing MOBILE mode");
#endif
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
      if (!motorController.isCommActive(i) || !motorController.isSafeToMove(i)) {
        continue;
      }

      if (currentTime - lastUpdateTime[i] < UPDATE_INTERVAL) {
        continue;
      }

      int32_t centerPos = (motorController.getMinPos(i) + motorController.getMaxPos(i)) / 2;
      float delta = (i == 0 || i == 2) ? xNorm * ROLL_SCALE : yNorm;
      int32_t range = motorController.getMaxPos(i) - motorController.getMinPos(i);
      int32_t newPosition = centerPos + (int32_t)(delta * range / 2.0f * (motorController.getUpIsPositive(i) ? 1 : -1));
      newPosition = constrain(newPosition, motorController.getMinPos(i), motorController.getMaxPos(i));

#if MC_DEBUG_VERBOSE
      Serial.print("M");
      Serial.print(i + 1);
      Serial.print(" Calculated position: Pos=");
      Serial.print(newPosition / 100.0f);
      Serial.println("°");
#endif

      if (newPosition != motorPositions[i]) {
#if MC_DEBUG_VERBOSE
        Serial.print("M");
        Serial.print(i + 1);
        Serial.print(" Attempting setPosition: Pos=");
        Serial.print(newPosition / 100.0f);
        Serial.println("°");
#endif
        if (motorController.setPosition(i, newPosition, 500)) {
          motorPositions[i] = newPosition;
          lastUpdateTime[i] = currentTime;
#if MC_DEBUG_VERBOSE
          Serial.print("M");
          Serial.print(i + 1);
          Serial.print(" Position set: Pos=");
          Serial.print(newPosition / 100.0f);
          Serial.println("°");
#endif
        } else {
#if MC_DEBUG_VERBOSE
          Serial.print("M");
          Serial.print(i + 1);
          Serial.println(" Failed to set position");
#endif
        }
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
// File: MotorModeController.cpp (180 lines)