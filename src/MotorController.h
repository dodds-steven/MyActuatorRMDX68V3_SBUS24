#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>
#include "MotorConfig.h"

class MotorController {
public:
  // Constructor: Initializes with motor configurations and number of motors
  MotorController(MotorConfig* motors, uint8_t numMotors);
  
  // Initializes motor communication and calibrates to center position
  bool init();
  
  // Sets position for a specific motor (position in 0.01°, speed in dps)
  bool setPosition(uint8_t motorIndex, int32_t position, uint16_t speed);
  
  // Gets current position for a specific motor (in 0.01°)
  int32_t getCurrentPosition(uint8_t motorIndex);
  
  // Shuts down a specific motor
  void shutdownMotor(uint8_t motorIndex);
  
  // Gets minimum position for a specific motor (in 0.01°)
  int32_t getMinPos(uint8_t motorIndex);
  
  // Gets maximum position for a specific motor (in 0.01°)
  int32_t getMaxPos(uint8_t motorIndex);
  
  // Gets UpIsPositive flag for a specific motor
  bool getUpIsPositive(uint8_t motorIndex);
  
  // Checks if communication is active for a specific motor
  bool isCommActive(uint8_t motorIndex);
  
  // Checks if a specific motor is safe to move
  bool isSafeToMove(uint8_t motorIndex);

private:
  MotorConfig* motors; // Pointer to motor configurations
  uint8_t numMotors;  // Number of motors
};

#endif // MOTOR_CONTROLLER_H
// File: MotorController.h (35 lines)