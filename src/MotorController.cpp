#include <MotorController.h>
#include <Definitions.h>

#define MC_DEBUG_VERBOSE false // Disabled to reduce lag
#define STATUS_FEEDBACK false  // Disabled to reduce lag
#define STALL_CURRENT_LIMIT 2000 // Increased for testing

MotorController::MotorController(MotorConfig* motors, uint8_t numMotors)
  : motors(motors), numMotors(numMotors), firstRun(true) {
  if (numMotors == 0) {
    numMotors = 4; // Fallback to NUM_MOTORS
  }
  calData = new CalibrationData[numMotors];
  for (uint8_t i = 0; i < numMotors; i++) {
    calData[i].completed = false;
    calData[i].failureReason = "";
  }
}

bool MotorController::initialize() {
  bool allInitialized = true;
  for (uint8_t i = 0; i < numMotors; i++) {
    if (!initializeMotor(i)) {
      allInitialized = false;
      break;
    }
  }
  if (allInitialized && !motors[1].commActive && motors[1].motor) {
    if (!initializeMotor(1)) {
      allInitialized = false;
    }
  }
  return allInitialized;
}

bool MotorController::initializeMotor(uint8_t index) {
  if (!motors[index].motor) {
#if STATUS_FEEDBACK
    Serial.print("M");
    Serial.print(index + 1);
    Serial.println(" Initialization failed: No motor object");
#endif
    return false;
  }

  motors[index].motor->begin(115200);
#if STATUS_FEEDBACK
  Serial.print("M");
  Serial.print(index + 1);
  Serial.print(" (ID=0x");
  Serial.print(motors[index].id, HEX);
  Serial.print(") 0x77 Feedback: ");
#endif
  bool success = false;
  uint8_t retry = 0;
  uint32_t retryStart = millis();
  while (retry < 1 && millis() - retryStart < 30) {
    if (motors[index].motor->BrakeRelease(motors[index].id)) {
      MyActuatorRMDX6V3::Feedback feedback = motors[index].motor->getFeedback();
      if (feedback.command == 0x77) {
        success = true;
      } else if (feedback.command == 0x9A) {
#if MC_DEBUG_VERBOSE
        Serial.print("M");
        Serial.print(index + 1);
        Serial.print(" BrakeRelease attempt ");
        Serial.print(retry + 1);
        Serial.print(" received 0x9A: Temp=");
        Serial.print(feedback.temperature);
        Serial.println("°C");
#endif
        motors[index].commActive = false;
        Serial.print("M");
        Serial.print(index + 1);
        Serial.println(" Error detected (0x9A), disabling motor");
        break;
      }
    }
    if (success) break;
    retry++;
    delayMicroseconds(10);
  }

  if (success) {
    motors[index].commActive = true;
#if STATUS_FEEDBACK
    Serial.print("M");
    Serial.print(index + 1);
    Serial.println(" Initialized OK");
#endif
  } else {
    motors[index].commActive = false;
#if STATUS_FEEDBACK
    Serial.print("M");
    Serial.print(index + 1);
    Serial.println(" Initialization Failed");
#endif
  }
  return success;
}

bool MotorController::calibrate() {
  bool allCalibrated = true;
  bool motorCalibrated[4] = {false}; // Fixed size for NUM_MOTORS
  int32_t centerPositions[4]; // Fixed size for NUM_MOTORS

  // Load predefined limits from Definitions.h
  for (uint8_t i = 0; i < numMotors; i++) {
    float minDeg, maxDeg;
    switch (motors[i].id) {
      case 0x01:
        minDeg = MOTOR1_MIN; // -115.00°
        maxDeg = MOTOR1_MAX; // -5.00°
        break;
      case 0x02:
        minDeg = MOTOR2_MIN; // -5.00°
        maxDeg = MOTOR2_MAX; // 115.00°
        break;
      case 0x03:
        minDeg = MOTOR3_MIN; // -115.00°
        maxDeg = MOTOR3_MAX; // 5.00°
        break;
      case 0x04:
        minDeg = MOTOR4_MIN; // -5.00°
        maxDeg = MOTOR4_MAX; // 115.00°
        break;
      default:
        calData[i].failureReason = "Invalid motor ID";
        allCalibrated = false;
        continue;
    }
    motors[i].minPos = (int32_t)(minDeg * 100.0); // Convert to 0.01° units
    motors[i].maxPos = (int32_t)(maxDeg * 100.0);
    motors[i].centerPos = (motors[i].minPos + motors[i].maxPos) / 2;
    motors[i].basePos = motors[i].centerPos;
    motors[i].initialBasePos = motors[i].centerPos;
    centerPositions[i] = motors[i].centerPos;
  }

  // Safety check: Read 0x92 angle and verify within limits ±10°
  for (uint8_t i = 0; i < numMotors; i++) {
    if (!motors[i].commActive) {
      calData[i].failureReason = "Initial communication failure";
      continue;
    }

    float shaftAngle = 0.0;
    bool positionValid = false;
    if (motors[i].motor->ReadMultiTurnAngle(motors[i].id)) {
      MyActuatorRMDX6V3::Feedback feedback = motors[i].motor->getFeedback();
      if (feedback.command == 0x92) {
        shaftAngle = feedback.shaftAngle / 100.0; // Convert to degrees
        positionValid = true;
        Serial.print("0x92 Feedback: ShaftAngle=");
        Serial.print(shaftAngle);
        Serial.println(" degrees");
      } else if (feedback.command == 0x9A) {
#if MC_DEBUG_VERBOSE
        Serial.print("M");
        Serial.print(i + 1);
        Serial.print(" 0x92 received 0x9A: Temp=");
        Serial.print(feedback.temperature);
        Serial.println("°C");
#endif
        calData[i].failureReason = "Error reading shaft angle (0x9A)";
        Serial.print("M");
        Serial.print(i + 1);
        Serial.println(" Error detected (0x9A), disabling motor");
      }
    }

    if (!positionValid || !motors[i].commActive) {
#if STATUS_FEEDBACK
      Serial.print("M");
      Serial.print(i + 1);
      Serial.println(" Failed to read shaft angle");
#endif
      allCalibrated = false;
      continue;
    }

    // Check if angle is within limits ±10° (1000 units)
    int32_t angleUnits = (int32_t)(shaftAngle * 100.0); // Convert to 0.01° units
    int32_t minLimit = motors[i].minPos - 1000;
    int32_t maxLimit = motors[i].maxPos + 1000;
    if (angleUnits >= minLimit && angleUnits <= maxLimit) {
      motors[i].safeToMove = true;
      motorCalibrated[i] = true;
#if STATUS_FEEDBACK
      Serial.print("M");
      Serial.print(i + 1);
      Serial.println(" Safety check passed");
#endif
    } else {
      motors[i].safeToMove = false;
      calData[i].failureReason = "Shaft angle outside safe range";
      allCalibrated = false;
#if STATUS_FEEDBACK
      Serial.print("M");
      Serial.print(i + 1);
      Serial.print(" Safety check failed: Angle=");
      Serial.print(shaftAngle);
      Serial.print("°, Range=[");
      Serial.print(minLimit / 100.0);
      Serial.print("°, ");
      Serial.print(maxLimit / 100.0);
      Serial.println("°]");
#endif
    }
  }

  // Move safe motors to center positions concurrently
  bool positionSet[4] = {false}; // Fixed size for NUM_MOTORS
  for (uint8_t i = 0; i < numMotors; i++) {
    if (!motorCalibrated[i] || !motors[i].commActive || !motors[i].safeToMove) {
      continue;
    }
    if (setPosition(i, centerPositions[i], 500)) {
      positionSet[i] = true;
    } else {
#if STATUS_FEEDBACK
      Serial.print("M");
      Serial.print(i + 1);
      Serial.println(" Failed to set center position");
#endif
      calData[i].failureReason = "Failed to set center position";
      motorCalibrated[i] = false;
      allCalibrated = false;
    }
  }

  // Poll motors concurrently
  uint32_t startTime = millis();
  const uint32_t timeout = 1000;
  const float tolerance = 200;
  bool reached[4] = {false}; // Fixed size for NUM_MOTORS
  while (millis() - startTime < timeout) {
    bool allReached = true;
    for (uint8_t i = 0; i < numMotors; i++) {
      if (!positionSet[i] || reached[i] || !motorCalibrated[i]) {
        continue;
      }
      allReached = false;
      MyActuatorRMDX6V3::Feedback feedback;
      if (getFeedback(i, feedback) && feedback.command == 0x92) {
        float currentAngle = feedback.shaftAngle;
        if (abs(currentAngle - centerPositions[i]) <= tolerance) {
          reached[i] = true;
#if MC_DEBUG_VERBOSE
          Serial.print("M");
          Serial.print(i + 1);
          Serial.print(" Reached center: CurrentAngle=");
          Serial.print(currentAngle / 100.0);
          Serial.println("°");
#endif
        } else {
#if STATUS_FEEDBACK
          Serial.print("M");
          Serial.print(i + 1);
          Serial.print(" Moving to center: CurrentAngle=");
          Serial.print(currentAngle / 100.0);
          Serial.println("°");
#endif
        }
      }
    }
    if (allReached) break;
    delay(5);
  }

  // Check for timeouts and stalls
  for (uint8_t i = 0; i < numMotors; i++) {
    if (!motorCalibrated[i]) {
      continue;
    }
    if (!reached[i] && positionSet[i]) {
#if STATUS_FEEDBACK
      Serial.print("M");
      Serial.print(i + 1);
      Serial.println(" Timeout reaching center position");
#endif
      calData[i].failureReason = "Timeout reaching center position";
      motorCalibrated[i] = false;
      allCalibrated = false;
      continue;
    }
    MyActuatorRMDX6V3::Feedback feedback;
    if (getFeedback(i, feedback) && abs(feedback.current) > STALL_CURRENT_LIMIT) {
#if STATUS_FEEDBACK
      Serial.print("M");
      Serial.print(i + 1);
      Serial.println(" Stall detected, disabling motor");
#endif
      motors[i].commActive = false;
      motors[i].safeToMove = false;
      calData[i].failureReason = "Stall detected during center move";
      motorCalibrated[i] = false;
      allCalibrated = false;
    }
  }

  // One-time travel limits report
  if (firstRun) {
    Serial.println("\n--- Motor Travel Limits ---");
    for (uint8_t i = 0; i < numMotors; i++) {
      if (motorCalibrated[i]) {
        // Swap min/max for UpIsPositive=true motors in display
        float displayMin = motors[i].UpIsPositive ? motors[i].maxPos / 100.0 : motors[i].minPos / 100.0;
        float displayMax = motors[i].UpIsPositive ? motors[i].minPos / 100.0 : motors[i].maxPos / 100.0;
        Serial.printf("Motor ID 0x%02X: MinPos = %.2f°, MaxPos = %.2f°, CenterPos = %.2f°\n",
                      motors[i].id, displayMin, displayMax, motors[i].centerPos / 100.0);
        calData[i].completed = true;
        calData[i].postMinPos = motors[i].minPos;
        calData[i].postMaxPos = motors[i].maxPos;
        calData[i].safePos = motors[i].centerPos;
        calData[i].centerPos = motors[i].centerPos;
      } else {
        Serial.printf("Motor ID 0x%02X: Calibration failed - %s\n",
                      motors[i].id, calData[i].failureReason.c_str());
      }
    }
  }
  firstRun = false;
  return allCalibrated;
}

void MotorController::generateCalibrationReport() {
#if MC_DEBUG_VERBOSE
  Serial.println("Calibration Report:");
  for (uint8_t i = 0; i < numMotors; i++) {
    Serial.print("Motor ");
    Serial.print(i + 1);
    Serial.println(":");
    if (calData[i].completed) {
      Serial.print("  MinPos: ");
      Serial.print(calData[i].postMinPos / 100.0);
      Serial.println("°");
      Serial.print("  MaxPos: ");
      Serial.print(calData[i].postMaxPos / 100.0);
      Serial.println("°");
      Serial.print("  CenterPos: ");
      Serial.print(calData[i].centerPos / 100.0);
      Serial.println("°");
      Serial.print("  SafePos: ");
      Serial.print(calData[i].safePos / 100.0);
      Serial.println("°");
    } else {
      Serial.print("  Calibration incomplete: ");
      Serial.println(calData[i].failureReason);
    }
    Serial.println();
  }
#endif
}

bool MotorController::setPosition(uint8_t index, int32_t pos, uint16_t speed) {
  if (index >= numMotors || !motors[index].motor || !motors[index].commActive || !motors[index].safeToMove) {
#if MC_DEBUG_VERBOSE
    Serial.print("M");
    Serial.print(index + 1);
    Serial.println(" Command skipped: Motor not active or unsafe");
#endif
    return false;
  }

  int32_t minLimit = motors[index].minPos;
  int32_t maxLimit = motors[index].maxPos;
  pos = constrain(pos, minLimit, maxLimit);

  bool success = false;
  uint8_t retry = 0;
  uint32_t retryStart = millis();

  while (retry < 1 && millis() - retryStart < 30) {
    if (motors[index].motor->AbsolutePositionClosedLoopControl(motors[index].id, pos, speed)) {
      MyActuatorRMDX6V3::Feedback feedback = motors[index].motor->getFeedback();
      if (feedback.command == 0xA4) {
        success = true;
#if MC_DEBUG_VERBOSE
        Serial.print("M");
        Serial.print(index + 1);
        Serial.print(" 0xA4 success: Pos=");
        Serial.print(pos / 100.0);
        Serial.print("°, Current=");
        Serial.print(feedback.current / 100.0);
        Serial.print("A, Speed=");
        Serial.print(feedback.speed / 10.0);
        Serial.println("dps");
#endif
        if (abs(feedback.current) > STALL_CURRENT_LIMIT && abs(feedback.speed) < 10) {
#if STATUS_FEEDBACK
          Serial.print("M");
          Serial.print(index + 1);
          Serial.println(" Stall detected, disabling motor");
#endif
          motors[index].commActive = false;
          success = false;
          break;
        }
      } else if (feedback.command == 0x9A) {
#if MC_DEBUG_VERBOSE
        Serial.print("M");
        Serial.print(index + 1);
        Serial.print(" 0xA4 attempt ");
        Serial.print(retry + 1);
        Serial.print(" received 0x9A: Temp=");
        Serial.print(feedback.temperature);
        Serial.println("°C");
#endif
        motors[index].commActive = false;
        Serial.print("M");
        Serial.print(index + 1);
        Serial.println(" Error detected (0x9A), disabling motor");
        break;
      }
    }
    if (success) break;
    retry++;
    delayMicroseconds(10);
  }

  if (!success) {
#if STATUS_FEEDBACK
    Serial.print("M");
    Serial.print(index + 1);
    Serial.println(" 0xA4 command failed after retries");
#endif
  }
  return success;
}

bool MotorController::getFeedback(uint8_t index, MyActuatorRMDX6V3::Feedback& feedback) {
  if (index >= numMotors || !motors[index].motor || !motors[index].commActive) {
#if MC_DEBUG_VERBOSE
    Serial.print("M");
    Serial.print(index + 1);
    Serial.println(" Feedback read failed: Motor not active");
#endif
    return false;
  }

  bool success = false;
  uint8_t retry = 0;
  uint32_t retryStart = millis();

  while (retry < 2 && millis() - retryStart < 30 * 2) {
    if (motors[index].motor->ReadMultiTurnAngle(motors[index].id)) {
      feedback = motors[index].motor->getFeedback();
      if (feedback.command == 0x92) {
        success = true;
#if MC_DEBUG_VERBOSE
        Serial.print("M");
        Serial.print(index + 1);
        Serial.print(" Shaft angle read: ");
        Serial.print(feedback.shaftAngle / 100.0);
        Serial.println("°");
#endif
      } else if (feedback.command == 0x9A) {
#if MC_DEBUG_VERBOSE
        Serial.print("M");
        Serial.print(index + 1);
        Serial.print(" Shaft angle read attempt ");
        Serial.print(retry + 1);
        Serial.print(" received 0x9A: Temp=");
        Serial.print(feedback.temperature);
        Serial.println("°C");
#endif
        motors[index].commActive = false;
        Serial.print("M");
        Serial.print(index + 1);
        Serial.println(" Error detected (0x9A), disabling motor");
        break;
      }
    }
    if (success) break;
    retry++;
    delayMicroseconds(10);
  }

  if (!success) {
#if STATUS_FEEDBACK
    Serial.print("M");
    Serial.print(index + 1);
    Serial.println(" Failed to read shaft angle");
#endif
  }
  return success;
}

void MotorController::shutdown() {
  bool positionSet[4] = {false}; // Fixed size for NUM_MOTORS
  bool reached[4] = {false}; // Fixed size for NUM_MOTORS

#if STATUS_FEEDBACK
  Serial.println("Forced shutdown initiated");
#endif
#if MC_DEBUG_VERBOSE
  Serial.println("Entering shutdown...");
#endif

  int32_t targetPos[4]; // Fixed size for NUM_MOTORS
  for (uint8_t i = 0; i < numMotors; i++) {
    if (!motors[i].motor || !motors[i].commActive) {
#if MC_DEBUG_VERBOSE
      Serial.print("M");
      Serial.print(i + 1);
      Serial.println(" Skipped: Not active");
#endif
      reached[i] = true;
      continue;
    }

    if (!calData[i].completed) {
#if STATUS_FEEDBACK
      Serial.print("M");
      Serial.print(i + 1);
      Serial.println(" Warning: Calibration incomplete, skipping shutdown movement");
#endif
      reached[i] = true;
      continue;
    }

    int32_t minLimit = motors[i].minPos;
    int32_t maxLimit = motors[i].maxPos;
    if (i == 0 || i == 2) { // Motor1, Motor3: Robot Low at maxPos
      targetPos[i] = calData[i].postMaxPos;
    } else { // Motor2, Motor4: Robot Low at minPos
      targetPos[i] = calData[i].postMinPos;
    }
    targetPos[i] = constrain(targetPos[i], minLimit, maxLimit);

#if MC_DEBUG_VERBOSE
    Serial.print("M");
    Serial.print(i + 1);
    Serial.print(" Moving to Robot Low: Pos=");
    Serial.print(targetPos[i] / 100.0);
    Serial.println("°");
#endif

    if (setPosition(i, targetPos[i], 500)) {
      positionSet[i] = true;
    } else {
#if STATUS_FEEDBACK
      Serial.print("M");
      Serial.print(i + 1);
      Serial.println(" Failed to set Robot Low position");
#endif
      reached[i] = true;
    }
  }

  uint32_t startTime = millis();
  const uint32_t timeout = 2000;
  const float tolerance = 200;
  while (millis() - startTime < timeout) {
    bool allReached = true;
    for (uint8_t i = 0; i < numMotors; i++) {
      if (reached[i] || !positionSet[i] || !motors[i].commActive) {
        continue;
      }
      allReached = false;
      MyActuatorRMDX6V3::Feedback feedback;
      if (getFeedback(i, feedback) && feedback.command == 0x92) {
        float currentAngle = feedback.shaftAngle;
        if (abs(currentAngle - targetPos[i]) <= tolerance) {
          reached[i] = true;
#if MC_DEBUG_VERBOSE
          Serial.print("M");
          Serial.print(i + 1);
          Serial.print(" Reached Robot Low: CurrentAngle=");
          Serial.print(currentAngle / 100.0);
          Serial.println("°");
#endif
        } else {
#if MC_DEBUG_VERBOSE
          Serial.print("M");
          Serial.print(i + 1);
          Serial.print(" Moving to Robot Low: CurrentAngle=");
          Serial.print(currentAngle / 100.0);
          Serial.println("°");
#endif
        }
      }
    }
    if (allReached) break;
    delay(50);
  }

  for (uint8_t i = 0; i < numMotors; i++) {
    if (!reached[i] && positionSet[i] && motors[i].commActive) {
#if STATUS_FEEDBACK
      Serial.print("M");
      Serial.print(i + 1);
      Serial.println(" Timeout reaching Robot Low");
#endif
    }
  }

  for (uint8_t i = 0; i < numMotors; i++) {
    if (!motors[i].motor || !motors[i].commActive) {
      continue;
    }

    bool success = false;
    uint8_t retry = 0;
    uint32_t retryStart = millis();

    while (retry < 1 && millis() - retryStart < 30) {
      if (motors[i].motor->MotorShutdown(motors[i].id)) {
        MyActuatorRMDX6V3::Feedback feedback = motors[i].motor->getFeedback();
        if (feedback.command == 0x80) {
          success = true;
#if MC_DEBUG_VERBOSE
          Serial.print("M");
          Serial.print(i + 1);
          Serial.println(" Shutdown success");
#endif
        } else if (feedback.command == 0x9A) {
#if MC_DEBUG_VERBOSE
          Serial.print("M");
          Serial.print(i + 1);
          Serial.print(" Shutdown attempt ");
          Serial.print(retry + 1);
          Serial.print(" received 0x9A: Temp=");
          Serial.print(feedback.temperature);
          Serial.println("°C");
#endif
          motors[i].commActive = false;
          Serial.print("M");
          Serial.print(i + 1);
          Serial.println(" Error detected during shutdown (0x9A)");
          break;
        }
      }
      if (success) break;
      retry++;
      delayMicroseconds(10);
    }

    if (!success) {
#if STATUS_FEEDBACK
      Serial.print("M");
      Serial.print(i + 1);
      Serial.println(" Shutdown failed after retries");
#endif
      motors[i].commActive = false;
    }
  }

#if STATUS_FEEDBACK
  Serial.println("Shutdown complete");
#endif
#if MC_DEBUG_VERBOSE
  Serial.println("Shutdown complete");
#endif
} // File: MotorController.cpp (475 lines)