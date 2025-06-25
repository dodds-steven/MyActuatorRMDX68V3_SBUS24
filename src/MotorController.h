#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>
#include <cstdint>
#include <MyActuatorRMDX6V3.h>
#include <MotorConfig.h>

struct CalibrationData {
  bool completed;
  bool polarityFlipped;
  bool preUpIsPositive;
  int32_t preMinPos;
  int32_t preMaxPos;
  float initialShaftAngle;
  bool postUpIsPositive;
  int32_t postMinPos;
  int32_t postMaxPos;
  int32_t safePos;
  int32_t centerPos;
  String failureReason;
};

class MotorController {
public:
  MotorController(MotorConfig* motors, uint8_t numMotors);
  bool initialize();
  bool initializeMotor(uint8_t index);
  bool calibrate();
  void generateCalibrationReport();
  bool setPosition(uint8_t index, int32_t pos, uint16_t speed);
  bool getFeedback(uint8_t index, MyActuatorRMDX6V3::Feedback& feedback);
  void shutdown();

private:
  MotorConfig* motors;
  uint8_t numMotors;
  CalibrationData* calData;
  bool firstRun;
};

// File: MotorController.h (42 lines)
#endif