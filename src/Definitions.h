#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#include <Arduino.h>

// Port definitions
#define RS485_PORT Serial4
#define RS485_DIR_PIN 41
#define SBUS_PORT Serial1

// Motor definitions
#define NUM_MOTORS 4

// Critical setup constants for motor control
const float SAFETY_MARGIN = 0.95;
const float STALL_CURRENT_LIMIT = 350;
const float END_STOP_BUFFER = 500;

// Motor limits in degrees (converted to 0.01° in MotorConfig)
const float MOTOR1_MIN = -115.00; //-124.66; // Motor 1 RobotHigh
const float MOTOR1_MAX = -5.00;   //-6.86;   // Motor 1 RobotLow
const float MOTOR2_MIN = 5.00;  //65.87;   // Motor 2 RobotLow
const float MOTOR2_MAX = 115;  //196.02;  // Motor 2 RobotHigh
const float MOTOR3_MIN = -115.00; //-315.56; // Motor 3 RobotHigh
const float MOTOR3_MAX = -5.00;  //-173.06; // Motor 3 RobotLow
const float MOTOR4_MIN = 5.00;  //113.42;  // Motor 4 RobotLow
const float MOTOR4_MAX = 115.00;  //244.52;  // Motor 4 RobotHigh

// Motor direction definitions
#define MOTOR1_UP_IS_POSITIVE true  // M1 (FrontLeft)
#define MOTOR2_UP_IS_POSITIVE false // M2 (FrontRight)
#define MOTOR3_UP_IS_POSITIVE true  // M3 (BackRight)
#define MOTOR4_UP_IS_POSITIVE false // M4 (BackLeft)

#endif

// File: Definitions.h (31 lines)