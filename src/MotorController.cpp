#include <MotorController.h>
#include <MyActuatorRMDX6V3.h>

MotorController::MotorController(MotorConfig* motors, uint8_t numMotors)
  : motors(motors), numMotors(numMotors) {
  Serial.print("MotorController initialized with numMotors=");
  Serial.println(numMotors);
  // Debug motors array in constructor
  Serial.println("Debugging motors array in MotorController constructor:");
  for (uint8_t i = 0; i < numMotors; i++) {
    Serial.print("M");
    Serial.print(i + 1);
    Serial.print(" ID=0x");
    Serial.print(motors[i].id, HEX);
    Serial.print(", UpIsPositive=");
    Serial.print(motors[i].UpIsPositive);
    Serial.print(", minPos=");
    Serial.print(motors[i].minPos / 100.0f);
    Serial.print("°, maxPos=");
    Serial.print(motors[i].maxPos / 100.0f);
    Serial.print("°, motor=");
    Serial.println((uintptr_t)motors[i].motor, HEX);
  }
}

bool MotorController::init() {
  bool success = true;
  Serial.println("Debugging motors array in MotorController::init:");
  for (uint8_t i = 0; i < numMotors; i++) {
    Serial.print("M");
    Serial.print(i + 1);
    Serial.print(" ID=0x");
    Serial.print(motors[i].id, HEX);
    Serial.print(", UpIsPositive=");
    Serial.print(motors[i].UpIsPositive);
    Serial.print(", minPos=");
    Serial.print(motors[i].minPos / 100.0f);
    Serial.print("°, maxPos=");
    Serial.print(motors[i].maxPos / 100.0f);
    Serial.print("°, motor=");
    Serial.println((uintptr_t)motors[i].motor, HEX);
  }
  for (uint8_t i = 0; i < numMotors; i++) {
    if (!motors[i].motor) {
      motors[i].commActive = false;
      motors[i].safeToMove = false;
      success = false;
      Serial.print("M");
      Serial.print(i + 1);
      Serial.println(" motor pointer null");
      continue;
    }
    // Initialize motor communication
    motors[i].motor->begin(115200); // Match RS485 baud rate
    if (motors[i].motor->ReadMultiTurnAngle(motors[i].id)) {
      motors[i].commActive = true;
      Serial.print("M");
      Serial.print(i + 1);
      Serial.println(" communication initialized and verified");
    } else {
      motors[i].commActive = false;
      motors[i].safeToMove = false;
      success = false;
      Serial.print("M");
      Serial.print(i + 1);
      Serial.println(" communication failed to verify");
      continue;
    }
    // Check motor status
    if (motors[i].motor->ReadSystemStatus(motors[i].id)) {
      MyActuatorRMDX6V3::Feedback feedback = motors[i].motor->getFeedback();
      Serial.print("M");
      Serial.print(i + 1);
      Serial.print(" status: error=");
      Serial.print(feedback.error);
      Serial.print(", errorMessage=");
      Serial.println(feedback.errorMessage);
    }
    // Calibrate to center position
    int32_t centerPos = (motors[i].minPos + motors[i].maxPos) / 2;
    Serial.print("M");
    Serial.print(i + 1);
    Serial.print(" attempting calibration to center: ");
    Serial.print(centerPos / 100.0f);
    Serial.println("°");
    if (motors[i].motor->AbsolutePositionClosedLoopControl(motors[i].id, centerPos, 500)) {
      motors[i].safeToMove = true;
      Serial.print("M");
      Serial.print(i + 1);
      Serial.print(" calibrated to center: ");
      Serial.print(centerPos / 100.0f);
      Serial.println("°");
    } else {
      motors[i].safeToMove = false;
      success = false;
      Serial.print("M");
      Serial.print(i + 1);
      Serial.println(" calibration failed");
    }
  }
  return success;
}

bool MotorController::setPosition(uint8_t motorIndex, int32_t position, uint16_t speed) {
  if (motorIndex >= numMotors || !motors[motorIndex].motor || !motors[motorIndex].commActive) {
    Serial.print("M");
    Serial.print(motorIndex + 1);
    Serial.println(" setPosition invalid: index, motor, or comm state");
    return false;
  }
  if (position < motors[motorIndex].minPos || position > motors[motorIndex].maxPos) {
    Serial.print("M");
    Serial.print(motorIndex + 1);
    Serial.print(" setPosition invalid: position=");
    Serial.print(position / 100.0f);
    Serial.print("°, min=");
    Serial.print(motors[motorIndex].minPos / 100.0f);
    Serial.print("°, max=");
    Serial.print(motors[motorIndex].maxPos / 100.0f);
    Serial.println("°");
    return false;
  }
  bool success = motors[motorIndex].motor->AbsolutePositionClosedLoopControl(motors[motorIndex].id, position, speed);
  if (success) {
    Serial.print("M");
    Serial.print(motorIndex + 1);
    Serial.print(" setPosition success: Pos=");
    Serial.print(position / 100.0f);
    Serial.println("°");
  } else {
    Serial.print("M");
    Serial.print(motorIndex + 1);
    Serial.println(" setPosition failed");
  }
  return success;
}

int32_t MotorController::getCurrentPosition(uint8_t motorIndex) {
  if (motorIndex >= numMotors || !motors[motorIndex].motor || !motors[motorIndex].commActive) {
    Serial.print("M");
    Serial.print(motorIndex + 1);
    Serial.println(" getCurrentPosition invalid: index, motor, or comm state");
    return 0;
  }
  int32_t position = motors[motorIndex].motor->ReadMultiTurnAngle(motors[motorIndex].id);
  Serial.print("M");
  Serial.print(motorIndex + 1);
  Serial.print(" getCurrentPosition: Pos=");
  Serial.print(position / 100.0f);
  Serial.println("°");
  return position;
}

void MotorController::shutdownMotor(uint8_t motorIndex) {
  if (motorIndex >= numMotors || !motors[motorIndex].motor || !motors[motorIndex].commActive) {
    Serial.print("M");
    Serial.print(motorIndex + 1);
    Serial.println(" shutdownMotor invalid: index, motor, or comm state");
    return;
  }
  motors[motorIndex].motor->MotorShutdown(motors[motorIndex].id);
  Serial.print("M");
  Serial.print(motorIndex + 1);
  Serial.println(" MotorShutdown issued");
}

int32_t MotorController::getMinPos(uint8_t motorIndex) {
  if (motorIndex < numMotors) {
    Serial.print("M");
    Serial.print(motorIndex + 1);
    Serial.print(" getMinPos: ");
    Serial.print(motors[motorIndex].minPos / 100.0f);
    Serial.println("°");
    return motors[motorIndex].minPos;
  }
  Serial.print("M");
  Serial.print(motorIndex + 1);
  Serial.println(" getMinPos: invalid index, returning 0");
  return 0;
}

int32_t MotorController::getMaxPos(uint8_t motorIndex) {
  if (motorIndex < numMotors) {
    Serial.print("M");
    Serial.print(motorIndex + 1);
    Serial.print(" getMaxPos: ");
    Serial.print(motors[motorIndex].maxPos / 100.0f);
    Serial.println("°");
    return motors[motorIndex].maxPos;
  }
  Serial.print("M");
  Serial.print(motorIndex + 1);
  Serial.println(" getMaxPos: invalid index, returning 0");
  return 0;
}

bool MotorController::getUpIsPositive(uint8_t motorIndex) {
  if (motorIndex < numMotors) {
    Serial.print("M");
    Serial.print(motorIndex + 1);
    Serial.print(" getUpIsPositive: ");
    Serial.println(motors[motorIndex].UpIsPositive);
    return motors[motorIndex].UpIsPositive;
  }
  Serial.print("M");
  Serial.print(motorIndex + 1);
  Serial.println(" getUpIsPositive: invalid index, returning false");
  return false;
}

bool MotorController::isCommActive(uint8_t motorIndex) {
  if (motorIndex < numMotors) {
    Serial.print("M");
    Serial.print(motorIndex + 1);
    Serial.print(" isCommActive: ");
    Serial.println(motors[motorIndex].commActive);
    return motors[motorIndex].commActive;
  }
  Serial.print("M");
  Serial.print(motorIndex + 1);
  Serial.println(" isCommActive: invalid index, returning false");
  return false;
}

bool MotorController::isSafeToMove(uint8_t motorIndex) {
  if (motorIndex < numMotors) {
    Serial.print("M");
    Serial.print(motorIndex + 1);
    Serial.print(" isSafeToMove: ");
    Serial.println(motors[motorIndex].safeToMove);
    return motors[motorIndex].safeToMove;
  }
  Serial.print("M");
  Serial.print(motorIndex + 1);
  Serial.println(" isSafeToMove: invalid index, returning false");
  return false;
}
// File: MotorController.cpp (184 lines)