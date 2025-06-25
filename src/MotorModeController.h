#ifndef MOTOR_MODE_CONTROLLER_H
#define MOTOR_MODE_CONTROLLER_H

#include <Arduino.h>
#include "MotorController.h"

#define NUM_MOTORS 4

class MotorModeController {
public:
  enum Mode { STATIC, MOBILE, SHUTDOWN };

  MotorModeController(MotorController& motorController, MotorConfig* motors, uint16_t* sbusChannels);
  void update();
  Mode getCurrentMode();
  int32_t getMotorPosition(uint8_t index);

private:
  MotorController& motorController;
  MotorConfig* motors;
  uint16_t* sbusChannels;
  Mode currentMode;
  bool recalibrationInProgress;
  uint32_t lastModeSwitchTime;
  uint16_t lastCh3Value;
  bool shutdownActive; // Flag to track shutdown state
  int32_t motorPositions[NUM_MOTORS];
};

#endif // MOTOR_MODE_CONTROLLER_H
// File: MotorModeController.h (36 lines)