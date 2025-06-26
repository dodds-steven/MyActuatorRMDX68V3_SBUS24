#ifndef MOTOR_MODE_CONTROLLER_H
#define MOTOR_MODE_CONTROLLER_H

#include <Arduino.h>
#include "MotorController.h"
#include "HandleSBUS.h"
#include "Definitions.h"

#define MC_DEBUG_VERBOSE true
#define STATIC_THRESHOLD 500 // CH3 threshold for STATIC mode
#define MOBILE_THRESHOLD 1300 // CH3 threshold for MOBILE mode
#define DEBOUNCE_INTERVAL 200 // ms, minimum time between mode switches
#define ROLL_SCALE 7.0f // Scaling factor for roll in MOBILE mode
#define UPDATE_INTERVAL 25 // 25ms update interval for motor commands
#define SHUTDOWN_SETTLE_DELAY 500 // 500ms delay before MotorShutdown

class MotorModeController {
public:
  enum Mode { STATIC, MOBILE, SHUTDOWN };

  // Constructor: Initializes with motor controller and SBUS channels
  MotorModeController(MotorController& motorController, uint16_t* sbusChannels);
  
  // Updates motor positions based on mode and SBUS inputs
  void update();
  
  // Returns current mode (STATIC, MOBILE, SHUTDOWN)
  Mode getCurrentMode();
  
  // Returns motor position for given index (0–3)
  int32_t getMotorPosition(uint8_t index);
  
  // Sets motor position for given index (0–3)
  void setMotorPosition(uint8_t index, int32_t position);

private:
  MotorController& motorController; // Reference to motor controller
  uint16_t* sbusChannels; // SBUS channel data
  Mode currentMode; // Current operating mode
  uint32_t lastModeSwitchTime; // Time of last mode switch
  uint16_t lastCh3Value; // Last CH3 value for mode switching
  bool shutdownActive; // Flag to track SHUTDOWN state
  int32_t motorPositions[NUM_MOTORS]; // Current motor positions (0.01°)
  uint32_t lastUpdateTime[NUM_MOTORS]; // Per-motor update timer
};

#endif // MOTOR_MODE_CONTROLLER_H
// File: MotorModeController.h (37 lines)