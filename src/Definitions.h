#ifndef DEFINITIONS_H
#define DEFINITIONS_H

// Port definitions
#define RS485_PORT Serial4
#define RS485_BAUD_RATE 115200 // Baud rate for RS485 communication
#define RS485_DIR_PIN 41
#define SBUS_PORT Serial1
#define JUMPER_PIN 20 // Pin for jumper/switch to enable encoder zeroing

// Motor definitions
#define NUM_MOTORS 4

// Critical setup constants for motor control
const float SAFETY_MARGIN = 0.95;
const float STALL_CURRENT_LIMIT = 350;
const float END_STOP_BUFFER = 500;

// Motor limits in degrees (converted to 0.01° in MotorConfig)
const float MOTOR1_MIN = -115.00; // Motor 1 RobotHigh
const float MOTOR1_MAX = -5.00;   // Motor 1 RobotLow
const float MOTOR2_MIN = 5.00;    // Motor 2 RobotLow
const float MOTOR2_MAX = 115;     // Motor 2 RobotHigh
const float MOTOR3_MIN = -115.00; // Motor 3 RobotHigh
const float MOTOR3_MAX = -5.00;   // Motor 3 RobotLow
const float MOTOR4_MIN = 5.00;    // Motor 4 RobotLow
const float MOTOR4_MAX = 115.00;  // Motor 4 RobotHigh

// Motor direction definitions
#define MOTOR1_UP_IS_POSITIVE true  // M1 (FrontLeft)
#define MOTOR2_UP_IS_POSITIVE false // M2 (FrontRight)
#define MOTOR3_UP_IS_POSITIVE true  // M3 (BackRight)
#define MOTOR4_UP_IS_POSITIVE false // M4 (BackLeft)

// SBUS channel definitions
#define CONTROL_MODE 3            // CH3 (index 2): Selects operating mode (STATIC, MOBILE, SHUTDOWN)
#define STATIC_X_CHANNEL 11      // CH11/X (index 10): Controls roll (Motor1/Motor3) in STATIC mode
#define STATIC_Y_CHANNEL 10      // CH10/Y (index 9): Controls pitch (Motor2/Motor4) in STATIC mode
#define MOBILE_X_CHANNEL 11        // CH11/X (index 10): Controls roll (Motor1/Motor3) in MOBILE mode
#define MOBILE_Y_CHANNEL 10        // CH10/Y (index 9): Controls pitch (Motor2/Motor4) in MOBILE mode
#define STRAFE_X_CHANNEL 1       // CH1/X (index 0): Controls lateral movement (strafe) in applicable modes
#define FOOTLIFT_HEIGHT_CHANNEL 18 // CH18 (index 17): Controls footlift height in MOBILE mode

#endif
// File: Definitions.h (39 lines)