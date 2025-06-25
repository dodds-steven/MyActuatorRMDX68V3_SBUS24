#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

#include <Arduino.h>
#include <MyActuatorRMDX6V3.h>

struct MotorConfig {
  uint8_t id; // Restored to uint8_t per protocol manual
  bool UpIsPositive;
  int32_t configMinPos;
  int32_t configMaxPos;
  MyActuatorRMDX6V3* motor;
  bool commActive;
  bool safeToMove;
  int32_t basePos;
  int32_t initialBasePos;
  int32_t minPos;
  int32_t maxPos;
  int32_t centerPos;
  float rangeScale;
  int32_t targetPos;
};

#endif

// File: MotorConfig.h (22 lines)