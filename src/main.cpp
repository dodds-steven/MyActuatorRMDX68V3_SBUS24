#include <Arduino.h>
#include "HandleSBUS.h"
#include "MyActuatorRMDX6V3.h"
#include "RS485Comm.h"

// Feedback control flag
#define STATUS_FEEDBACK true    // Status messages for new users

// Motor direction parameters
bool Motor1UpIsPositive = false; // M1: Decrease for up
bool Motor2UpIsPositive = true;  // M2: Increase for up
bool Motor3UpIsPositive = false; // M3: Decrease for up
bool Motor4UpIsPositive = true;  // M4: Increase for up

// RS-485 configuration
#define RS485_SERIAL Serial4
#define RS485_DIR_PIN 41
#define RS485_BAUD 115200

// LED configuration
#define LED_PIN 13
const uint32_t FLASH_SEQUENCE_MS = 5000;
const uint32_t FLASH_PAUSE_MS = 2000;
const uint32_t LONG_FLASH_MS = 1250;
const uint32_t SHORT_FLASH_MS = 625;

// Motor configuration
const int32_t POS_THRESHOLD = 100;     // 1° in 0.01°
const uint16_t SPEED_DPS = 200;        // 20 dps
const uint16_t CALIBRATION_SPEED_DPS = 50;
const uint32_t TEST_DURATION = 10000;
const uint32_t LOOP_INTERVAL_MS = 9;   // Match SBUS frame rate
const uint32_t COMMAND_INTERVAL = 10;  // Base interval in ms
const uint32_t COMMAND_TIMEOUT_MS = 30; // 30ms
const uint32_t DEBUG_INTERVAL_MS = 3000; // 3s
const bool verboseFeedback = false;     // Explicitly false
const uint8_t MOTOR_INIT_RETRIES = 3;
const uint8_t ENCODER_READ_RETRIES = 3;
const uint8_t BRAKE_RELEASE_RETRIES = 3;
const uint8_t CRC_RETRY_ATTEMPTS = 4;
const float SAFETY_MARGIN = 0.95;
const float STALL_CURRENT_LIMIT = 350; // 3.5A
const float END_STOP_BUFFER = 500;     // 5° in 0.01°

// Motor limits (in degrees, converted to 0.01°)
const float MOTOR1_MIN = -124.66; // RobotHigh
const float MOTOR1_MAX = -6.86;   // RobotLow
const float MOTOR2_MIN = 65.87;   // RobotLow
const float MOTOR2_MAX = 196.02;  // RobotHigh
const float MOTOR3_MIN = -315.56; // RobotHigh
const float MOTOR3_MAX = -173.06; // RobotLow
const float MOTOR4_MIN = 113.42;  // RobotLow
const float MOTOR4_MAX = 244.52;  // RobotHigh

struct MotorConfig {
  uint8_t id;
  int32_t configMinPos;
  int32_t configMaxPos;
  int32_t minPos;
  int32_t maxPos;
  int32_t centerPos;
  bool UpIsPositive;
  MyActuatorRMDX6V3* motor;
  float rangeScale;
  bool safeToMove;
  bool commActive;
  int32_t basePos;
  int32_t targetPos;
  uint32_t lastCommandTime;
  uint8_t commandState; // 0=idle, 1=move, 2=retry, 3=stall_retry
  uint8_t retryCount;
  uint32_t retryStartTime;
};
MotorConfig motors[] = {
  {0x01, 0, 0, 0, 0, 0, Motor1UpIsPositive, new MyActuatorRMDX6V3(RS485_SERIAL, RS485_DIR_PIN), 0, false, false, 0, 0, 0, 0, 0, 0}, // M1
  {0x02, 0, 0, 0, 0, 0, Motor2UpIsPositive, new MyActuatorRMDX6V3(RS485_SERIAL, RS485_DIR_PIN), 0, false, false, 0, 0, 0, 0, 0, 0}, // M2
  {0x03, 0, 0, 0, 0, 0, Motor3UpIsPositive, new MyActuatorRMDX6V3(RS485_SERIAL, RS485_DIR_PIN), 0, false, false, 0, 0, 0, 0, 0, 0}, // M3
  {0x04, 0, 0, 0, 0, 0, Motor4UpIsPositive, new MyActuatorRMDX6V3(RS485_SERIAL, RS485_DIR_PIN), 0, false, false, 0, 0, 0, 0, 0, 0}  // M4
};
const uint8_t NUM_MOTORS = 4;

struct FlashState {
  uint32_t lastChange;
  uint8_t currentMotor;
  uint8_t flashCount;
  bool ledOn;
  bool inPause;
};
FlashState flashState = {0, 0, 0, false, false};
SBUSHandler sbusHandler(verboseFeedback);
bool rs485Active = false;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 1000);
#if STATUS_FEEDBACK
  Serial.println("Starting SBUS Motor Control Test (10s, All Motors)...\n");
#endif

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  motors[0].configMinPos = MOTOR1_MIN * 100;
  motors[0].configMaxPos = MOTOR1_MAX * 100;
  motors[1].configMinPos = MOTOR2_MIN * 100;
  motors[1].configMaxPos = MOTOR2_MAX * 100;
  motors[2].configMinPos = MOTOR3_MIN * 100;
  motors[2].configMaxPos = MOTOR3_MAX * 100;
  motors[3].configMinPos = MOTOR4_MIN * 100;
  motors[3].configMaxPos = MOTOR4_MAX * 100;

  sbusHandler.begin();

  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    if (motors[i].motor) {
      rs485Active = true;
      motors[i].motor->begin(RS485_BAUD);
#if STATUS_FEEDBACK
      Serial.print("M");
      Serial.print(i + 1);
      Serial.print(" (ID=0x");
      Serial.print(motors[i].id, HEX);
      Serial.print(") 0x77 Feedback: ");
#endif
      bool success = false;
      uint8_t retry = 0;
      uint32_t retryStart = millis();
      while (retry < MOTOR_INIT_RETRIES && millis() - retryStart < COMMAND_TIMEOUT_MS * MOTOR_INIT_RETRIES) {
        if (motors[i].motor->BrakeRelease(motors[i].id)) {
          success = true;
          break;
        }
        retry++;
      }
      rs485Active = false;
#if STATUS_FEEDBACK
      if (success) {
        Serial.println("OK");
        motors[i].commActive = true;
      } else {
        Serial.println("Failed - No communication");
        motors[i].commActive = false;
      }
      Serial.println();
#else
      motors[i].commActive = success;
#endif
    }
  }

  // Minimal fix: Re-run BrakeRelease for Motor 2 if it failed
  if (!motors[1].commActive && motors[1].motor) {
    rs485Active = true;
#if STATUS_FEEDBACK
    Serial.println("Retrying M2 (ID=0x02) 0x77 Feedback: ");
#endif
    bool success = false;
    uint8_t retry = 0;
    uint32_t retryStart = millis();
    while (retry < MOTOR_INIT_RETRIES && millis() - retryStart < COMMAND_TIMEOUT_MS * MOTOR_INIT_RETRIES) {
      if (motors[1].motor->BrakeRelease(motors[1].id)) {
        success = true;
        break;
      }
      retry++;
    }
    rs485Active = false;
#if STATUS_FEEDBACK
    if (success) {
      Serial.println("OK");
      motors[1].commActive = true;
    } else {
      Serial.println("Failed - No communication");
    }
    Serial.println();
#else
    motors[1].commActive = success;
#endif
  }

  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    if (!motors[i].commActive) continue;

#if STATUS_FEEDBACK
    Serial.print("M");
    Serial.print(i + 1);
    Serial.print(" Pre-calibration BrakeRelease: ");
#endif
    rs485Active = true;
    bool brakeSuccess = false;
    uint8_t retry = 0;
    uint32_t retryStart = millis();
    while (retry < BRAKE_RELEASE_RETRIES && millis() - retryStart < COMMAND_TIMEOUT_MS * BRAKE_RELEASE_RETRIES) {
      if (motors[i].motor->BrakeRelease(motors[i].id)) {
        brakeSuccess = true;
        break;
      }
      retry++;
    }
    rs485Active = false;
    if (!brakeSuccess) {
#if STATUS_FEEDBACK
      Serial.println("Failed, disabling motor");
      Serial.println();
#endif
      motors[i].commActive = false;
      continue;
    }
#if STATUS_FEEDBACK
    Serial.println("OK");
#endif

#if STATUS_FEEDBACK
    Serial.print("M");
    Serial.print(i + 1);
    Serial.print(" Calibrating at robot low: ");
#endif
    int32_t lowPos = motors[i].configMaxPos;
    rs485Active = true;
    bool moveSuccess = false;
    retry = 0;
    retryStart = millis();
    while (retry < CRC_RETRY_ATTEMPTS && millis() - retryStart < COMMAND_TIMEOUT_MS * CRC_RETRY_ATTEMPTS) {
      if (motors[i].motor->AbsolutePositionClosedLoopControl(motors[i].id, lowPos, CALIBRATION_SPEED_DPS)) {
        moveSuccess = true;
        break;
      }
#if DEBUG_FEEDBACK
      Serial.print("M");
      Serial.print(i + 1);
      Serial.print(" 0xA4 CRC mismatch (Attempt ");
      Serial.print(retry + 1);
      Serial.println(")");
#endif
      retry++;
    }
    rs485Active = false;
    if (!moveSuccess) {
#if STATUS_FEEDBACK
      Serial.println("0xA4 Failed after retries");
      Serial.println();
#endif
      motors[i].commActive = false;
      continue;
    }
#if DEBUG_FEEDBACK
    MyActuatorRMDX6V3::Feedback initFeedback = motors[i].motor->getLastFeedback();
    Serial.print("0xA4 Feedback: Temp=");
    Serial.print(initFeedback.temperature);
    Serial.print("°C, Current=");
    Serial.print(initFeedback.current / 100.0);
    Serial.print("A, Speed=");
    Serial.print(initFeedback.speed / 10.0);
    Serial.print("dps, Delta=");
    Serial.print(initFeedback.shaftAngle / 100.0);
    Serial.println("°");
#endif
    if (abs(motors[i].motor->getLastFeedback().current) > STALL_CURRENT_LIMIT) {
#if STATUS_FEEDBACK
      Serial.print("M");
      Serial.print(i + 1);
      Serial.println(" Stall detected during calibration, retrying...");
#endif
      rs485Active = true;
      retry = 0;
      retryStart = millis();
      while (retry < 2 && millis() - retryStart < COMMAND_TIMEOUT_MS * 2) {
        if (motors[i].motor->AbsolutePositionClosedLoopControl(motors[i].id, lowPos, CALIBRATION_SPEED_DPS / 2)) {
          if (abs(motors[i].motor->getLastFeedback().current) <= STALL_CURRENT_LIMIT) {
            moveSuccess = true;
            break;
          }
        }
        retry++;
      }
      rs485Active = false;
      if (!moveSuccess) {
#if STATUS_FEEDBACK
        Serial.println(" Persistent stall, disabling motor");
        Serial.println();
#endif
        motors[i].safeToMove = false;
        continue;
      }
    }
    bool positionValid = false;
    float shaftAngle = 0.0;
    rs485Active = true;
    retry = 0;
    retryStart = millis();
    while (retry < ENCODER_READ_RETRIES && millis() - retryStart < COMMAND_TIMEOUT_MS * ENCODER_READ_RETRIES) {
      if (motors[i].motor->ReadMultiTurnAngle(motors[i].id)) {
        MyActuatorRMDX6V3::Feedback feedback = motors[i].motor->getLastFeedback();
        if (feedback.command == 0x92) {
          shaftAngle = feedback.shaftAngle;
          positionValid = true;
          break;
        }
      }
#if DEBUG_FEEDBACK
      Serial.print("M");
      Serial.print(i + 1);
      Serial.print(" 0x92 Read Failed (Attempt ");
      Serial.print(retry + 1);
      Serial.println(")");
#endif
      retry++;
    }
    rs485Active = false;
    if (positionValid) {
      motors[i].basePos = (int32_t)shaftAngle;
      int32_t configMaxTravel = abs(motors[i].configMaxPos - motors[i].configMinPos);
      int32_t maxTravel = (int32_t)(configMaxTravel * SAFETY_MARGIN);
      if (motors[i].UpIsPositive) {
        motors[i].minPos = motors[i].basePos;
        motors[i].maxPos = motors[i].basePos + maxTravel;
      } else {
        motors[i].maxPos = motors[i].basePos;
        motors[i].minPos = motors[i].basePos - maxTravel;
      }
      motors[i].centerPos = (motors[i].minPos + motors[i].maxPos) / 2;
      motors[i].rangeScale = maxTravel / 2.0;
      motors[i].safeToMove = true;
#if DEBUG_FEEDBACK
      Serial.print("0x92 Feedback: ShaftAngle=");
      Serial.print(shaftAngle / 100.0);
      Serial.print("°\nBasePos=");
      Serial.print(shaftAngle / 100.0);
      Serial.print("°, New Min=");
      Serial.print(motors[i].minPos / 100.0);
      Serial.print("°, Max=");
      Serial.print(motors[i].maxPos / 100.0);
      Serial.print("°, Center=");
      Serial.print(motors[i].centerPos / 100.0);
      Serial.println("°");
#endif
#if STATUS_FEEDBACK
      Serial.print("M");
      Serial.print(i + 1);
      Serial.print(" Test move to center: ");
#endif
      rs485Active = true;
      moveSuccess = false;
      retry = 0;
      retryStart = millis();
      while (retry < CRC_RETRY_ATTEMPTS && millis() - retryStart < COMMAND_TIMEOUT_MS * CRC_RETRY_ATTEMPTS) {
        if (motors[i].motor->AbsolutePositionClosedLoopControl(motors[i].id, motors[i].centerPos, CALIBRATION_SPEED_DPS)) {
          moveSuccess = true;
          break;
        }
#if DEBUG_FEEDBACK
        Serial.print("M");
        Serial.print(i + 1);
        Serial.print(" 0xA4 CRC mismatch (Test Move Attempt ");
        Serial.print(retry + 1);
        Serial.println(")");
#endif
        retry++;
      }
      rs485Active = false;
      if (!moveSuccess) {
#if STATUS_FEEDBACK
        Serial.println("0xA4 Test Move Failed after retries");
        Serial.println();
#endif
        motors[i].safeToMove = false;
        continue;
      }
#if DEBUG_FEEDBACK
      MyActuatorRMDX6V3::Feedback testFeedback = motors[i].motor->getLastFeedback();
      Serial.print("0xA4 Feedback: Temp=");
      Serial.print(testFeedback.temperature);
      Serial.print("°C, Current=");
      Serial.print(testFeedback.current / 100.0);
      Serial.print("A, Speed=");
      Serial.print(testFeedback.speed / 10.0);
      Serial.print("dps, Delta=");
      Serial.print(testFeedback.shaftAngle / 100.0);
      Serial.println("°");
#endif
      if (abs(motors[i].motor->getLastFeedback().current) > STALL_CURRENT_LIMIT) {
#if STATUS_FEEDBACK
        Serial.print("M");
        Serial.print(i + 1);
        Serial.println(" Stall detected during test move, retrying...");
#endif
        rs485Active = true;
        retry = 0;
        retryStart = millis();
        while (retry < 2 && millis() - retryStart < COMMAND_TIMEOUT_MS * 2) {
          if (motors[i].motor->AbsolutePositionClosedLoopControl(motors[i].id, motors[i].centerPos, CALIBRATION_SPEED_DPS / 2)) {
            if (abs(motors[i].motor->getLastFeedback().current) <= STALL_CURRENT_LIMIT) {
              moveSuccess = true;
              break;
            }
          }
          retry++;
        }
        rs485Active = false;
        if (!moveSuccess) {
#if STATUS_FEEDBACK
          Serial.print("M");
          Serial.print(i + 1);
          Serial.println(" Persistent stall, disabling motor");
#endif
          motors[i].safeToMove = false;
        }
      }
    } else {
#if STATUS_FEEDBACK
      Serial.println("Failed to read position, disabling motor");
#endif
      motors[i].safeToMove = false;
    }
#if STATUS_FEEDBACK
    Serial.println();
#endif
  }
#if STATUS_FEEDBACK
  Serial.println("Motor calibration completed.\n");
#endif
}

int32_t computeMotorPosition(float xNorm, float yNorm, uint8_t motorIndex) {
  if (abs(xNorm) < 0.2 && abs(yNorm) < 0.2) {
    return motors[motorIndex].centerPos;
  }
  int32_t basePos = motors[motorIndex].centerPos;
  float offset;
  switch (motorIndex) {
    case 0: offset = (-xNorm + yNorm) * motors[motorIndex].rangeScale; break;
    case 1: offset = (xNorm + yNorm) * motors[motorIndex].rangeScale; break;
    case 2: offset = (-xNorm + yNorm) * motors[motorIndex].rangeScale; break;
    case 3: offset = (xNorm + yNorm) * motors[motorIndex].rangeScale; break;
    default: offset = 0;
  }
  if (!motors[motorIndex].UpIsPositive) {
    offset = -offset;
  }
  int32_t motorPos = basePos + (int32_t)offset;
  int32_t minLimit = min(motors[motorIndex].minPos, motors[motorIndex].maxPos) + (motors[motorIndex].UpIsPositive ? END_STOP_BUFFER : -END_STOP_BUFFER);
  int32_t maxLimit = max(motors[motorIndex].minPos, motors[motorIndex].maxPos) + (motors[motorIndex].UpIsPositive ? -END_STOP_BUFFER : END_STOP_BUFFER);
  return constrain(motorPos, minLimit, maxLimit);
}

void updateLedFlash() {
  uint32_t now = millis();
  uint32_t elapsed = now - flashState.lastChange;

  if (flashState.inPause) {
    if (elapsed >= FLASH_PAUSE_MS) {
      flashState.inPause = false;
      flashState.currentMotor = 0;
      flashState.flashCount = 0;
      flashState.ledOn = true;
      flashState.lastChange = now;
      digitalWrite(LED_PIN, HIGH);
    }
    return;
  }

  if (motors[flashState.currentMotor].commActive) {
    if (elapsed >= LONG_FLASH_MS) {
      digitalWrite(LED_PIN, LOW);
      flashState.currentMotor++;
      flashState.flashCount = 0;
      flashState.ledOn = false;
      flashState.lastChange = now;
      if (flashState.currentMotor >= NUM_MOTORS) {
        flashState.inPause = true;
        flashState.currentMotor = 0;
      } else {
        digitalWrite(LED_PIN, HIGH);
      }
    }
  } else {
    if (flashState.flashCount == 0) {
      if (elapsed >= SHORT_FLASH_MS) {
        digitalWrite(LED_PIN, LOW);
        flashState.flashCount = 1;
        flashState.ledOn = false;
        flashState.lastChange = now;
      }
    } else if (flashState.flashCount == 1) {
      if (elapsed >= SHORT_FLASH_MS / 2) {
        digitalWrite(LED_PIN, HIGH);
        flashState.flashCount = 2;
        flashState.ledOn = true;
        flashState.lastChange = now;
      }
    } else if (flashState.flashCount == 2) {
      if (elapsed >= SHORT_FLASH_MS) {
        digitalWrite(LED_PIN, LOW);
        flashState.currentMotor++;
        flashState.flashCount = 0;
        flashState.ledOn = false;
        flashState.lastChange = now;
        if (flashState.currentMotor >= NUM_MOTORS) {
          flashState.inPause = true;
          flashState.currentMotor = 0;
        } else {
          digitalWrite(LED_PIN, HIGH);
        }
      }
    }
  }
}

void loop() {
  static uint32_t startTime = millis();
#if DEBUG_FEEDBACK
  static uint32_t lastDebug = 0;
  static uint16_t lastXChannel = 992;
  static uint16_t lastYChannel = 992;
  static bool initialPositionRead = false;
#endif
  static uint32_t lastLoop = 0;
  uint16_t channels[24];
  float xNorm, yNorm;

#if DEBUG_FEEDBACK
  uint32_t loopStart = millis(); // Start timing in milliseconds
#endif

  updateLedFlash();

  if (millis() - startTime > TEST_DURATION) {
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
      if (motors[i].motor && motors[i].commActive) {
        if (motors[i].commandState == 0) {
          rs485Active = true;
#if STATUS_FEEDBACK
          Serial.print("M");
          Serial.print(i + 1);
          Serial.print(" 0x80 Feedback: ");
#endif
          if (motors[i].motor->MotorShutdown(motors[i].id)) {
#if STATUS_FEEDBACK
            Serial.println("OK");
#endif
            motors[i].commActive = false;
          } else {
#if STATUS_FEEDBACK
            Serial.println("Failed");
#endif
            motors[i].commandState = 2;
            motors[i].retryCount = 0;
            motors[i].retryStartTime = millis();
          }
          rs485Active = false;
#if STATUS_FEEDBACK
          Serial.println();
#endif
        } else if (motors[i].commandState == 2 && millis() - motors[i].retryStartTime >= COMMAND_TIMEOUT_MS) {
          rs485Active = true;
          motors[i].retryCount++;
          if (motors[i].retryCount >= CRC_RETRY_ATTEMPTS) {
#if STATUS_FEEDBACK
            Serial.print("M");
            Serial.print(i + 1);
            Serial.println(" 0x80 Failed after retries");
            Serial.println();
#endif
            motors[i].commActive = false;
            motors[i].commandState = 0;
          } else {
#if DEBUG_FEEDBACK
            Serial.print("M");
            Serial.print(i + 1);
            Serial.print(" 0x80 CRC mismatch (Attempt ");
            Serial.print(motors[i].retryCount);
            Serial.println(")");
#endif
            if (motors[i].motor->MotorShutdown(motors[i].id)) {
#if STATUS_FEEDBACK
              Serial.println("OK");
#endif
              motors[i].commActive = false;
              motors[i].commandState = 0;
            }
            motors[i].retryStartTime = millis();
          }
          rs485Active = false;
        }
      }
    }
    if (!rs485Active) {
#if STATUS_FEEDBACK
      sbusHandler.printSummary();
      Serial.println("Test completed, motors stopped.");
#endif
      while (true);
    }
    return;
  }

  if (millis() - lastLoop < LOOP_INTERVAL_MS) {
#if DEBUG_FEEDBACK
    uint32_t loopTime = millis() - loopStart;
    Serial.printf("Loop runtime: %lu ms\n", loopTime);
#endif
    return;
  }

  if (!rs485Active && sbusHandler.readChannels(xNorm, yNorm, channels)) {
    // Removed redundant SBUS_MAX check
  }

  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    if (!motors[i].motor || !motors[i].safeToMove || !motors[i].commActive) continue;

    if (motors[i].commandState == 0) {
      int32_t targetPos = computeMotorPosition(xNorm, yNorm, i);
      if (abs(targetPos - motors[i].targetPos) > POS_THRESHOLD && millis() - motors[i].lastCommandTime >= COMMAND_INTERVAL) {
        // Dynamic command interval based on travel distance
        uint32_t dynamicInterval = (abs(targetPos - motors[i].targetPos) / 100 > 10) ? 100 : 10; // 100ms for >10°, 10ms otherwise
        if (millis() - motors[i].lastCommandTime >= dynamicInterval) {
          motors[i].targetPos = targetPos;
          motors[i].commandState = 1;
          motors[i].retryCount = 0;
          rs485Active = true;
          if (motors[i].motor->AbsolutePositionClosedLoopControl(motors[i].id, targetPos, SPEED_DPS)) {
            motors[i].lastCommandTime = millis();
            motors[i].commandState = 0;
#if DEBUG_FEEDBACK
            MyActuatorRMDX6V3::Feedback feedback = motors[i].motor->getLastFeedback();
            Serial.print("M");
            Serial.print(i + 1);
            Serial.print(" 0xA4 Feedback: Temp=");
            Serial.print(feedback.temperature);
            Serial.print("°C, Current=");
            Serial.print(feedback.current / 100.0);
            Serial.print("A, Speed=");
            Serial.print(feedback.speed / 10.0);
            Serial.print("dps, Delta=");
            Serial.print(feedback.shaftAngle / 100.0);
            Serial.println("°");
            if (abs(feedback.current) > STALL_CURRENT_LIMIT) {
              Serial.print("M");
              Serial.print(i + 1);
              Serial.println(" Stall detected, retrying...");
              motors[i].commandState = 3;
              motors[i].retryCount = 0;
              motors[i].retryStartTime = millis();
            }
            Serial.println();
#endif
          } else {
            motors[i].commandState = 2;
            motors[i].retryStartTime = millis();
#if DEBUG_FEEDBACK
            Serial.print("M");
            Serial.print(i + 1);
            Serial.print(" 0xA4 CRC mismatch (Attempt 1)");
            Serial.println();
#endif
          }
          rs485Active = false;
        }
      }
    } else if (motors[i].commandState == 2 && millis() - motors[i].retryStartTime >= COMMAND_TIMEOUT_MS) {
      rs485Active = true;
      motors[i].retryCount++;
      if (motors[i].retryCount >= CRC_RETRY_ATTEMPTS) {
#if DEBUG_FEEDBACK
        Serial.print("M");
        Serial.print(i + 1);
        Serial.println(" 0xA4 Failed after retries");
#endif
        motors[i].commandState = 0;
      } else {
        if (motors[i].motor->AbsolutePositionClosedLoopControl(motors[i].id, motors[i].targetPos, SPEED_DPS)) {
          motors[i].lastCommandTime = millis();
          motors[i].commandState = 0;
#if DEBUG_FEEDBACK
          MyActuatorRMDX6V3::Feedback feedback = motors[i].motor->getLastFeedback();
          Serial.print("M");
          Serial.print(i + 1);
          Serial.print(" 0xA4 Feedback: Temp=");
          Serial.print(feedback.temperature);
          Serial.print("°C, Current=");
          Serial.print(feedback.current / 100.0);
          Serial.print("A, Speed=");
          Serial.print(feedback.speed / 10.0);
          Serial.print("dps, Delta=");
          Serial.print(feedback.shaftAngle / 100.0);
          Serial.println("°");
          if (abs(feedback.current) > STALL_CURRENT_LIMIT) {
            motors[i].commandState = 3;
            motors[i].retryCount = 0;
            motors[i].retryStartTime = millis();
          }
          Serial.println();
#endif
        } else {
#if DEBUG_FEEDBACK
          Serial.print("M");
          Serial.print(i + 1);
          Serial.print(" 0xA4 CRC mismatch (Attempt ");
          Serial.print(motors[i].retryCount + 1);
          Serial.println(")");
#endif
          motors[i].retryStartTime = millis();
        }
      }
      rs485Active = false;
    } else if (motors[i].commandState == 3 && millis() - motors[i].retryStartTime >= COMMAND_TIMEOUT_MS) {
      rs485Active = true;
      motors[i].retryCount++;
      if (motors[i].retryCount >= 2) {
#if STATUS_FEEDBACK
        Serial.print("M");
        Serial.print(i + 1);
        Serial.println(" Persistent stall, disabling motor");
#endif
        motors[i].safeToMove = false;
        motors[i].commandState = 0;
      } else {
        if (motors[i].motor->AbsolutePositionClosedLoopControl(motors[i].id, motors[i].targetPos, CALIBRATION_SPEED_DPS / 2)) {
          MyActuatorRMDX6V3::Feedback feedback = motors[i].motor->getLastFeedback();
          if (abs(feedback.current) <= STALL_CURRENT_LIMIT) {
            motors[i].lastCommandTime = millis();
            motors[i].commandState = 0;
#if DEBUG_FEEDBACK
            Serial.print("M");
            Serial.print(i + 1);
            Serial.print(" 0xA4 Feedback: Temp=");
            Serial.print(feedback.temperature);
            Serial.print("°C, Current=");
            Serial.print(feedback.current / 100.0);
            Serial.print("A, Speed=");
            Serial.print(feedback.speed / 10.0);
            Serial.print("dps, Delta=");
            Serial.print(feedback.shaftAngle / 100.0);
            Serial.println("°");
            Serial.println();
#endif
          } else {
            motors[i].retryStartTime = millis();
          }
        }
      }
      rs485Active = false;
    }
  }

#if DEBUG_FEEDBACK
  if (millis() - lastDebug >= DEBUG_INTERVAL_MS) {
    Serial.print("SBUS: X Channel (11): ");
    Serial.print(channels[X_CHANNEL]);
    Serial.print(", Y Channel (10): ");
    Serial.print(channels[Y_CHANNEL]);
    Serial.print(", Ch9: ");
    Serial.print(channels[8]);
    Serial.print(", Ch12: ");
    Serial.println(channels[11]);
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
      Serial.print("M");
      Serial.print(i + 1);
      Serial.print(": Target=");
      int32_t targetPos = computeMotorPosition(xNorm, yNorm, i);
      Serial.print(targetPos / 100.0);
      Serial.print("°");
      if (!motors[i].motor || !motors[i].commActive) {
        Serial.println(", Status=Skipped: No communication");
        continue;
      }
      if (!motors[i].safeToMove) {
        Serial.println(", Status=Skipped: Unsafe Position");
        continue;
      }
      Serial.print(", 0x92 Feedback: ");
      if (!initialPositionRead) {
        Serial.println("Skipped: Calibration completed");
        continue;
      }
      rs485Active = true;
      bool readSuccess = false;
      uint8_t retry = 0;
      uint32_t retryStart = millis();
      while (retry < CRC_RETRY_ATTEMPTS && millis() - retryStart < COMMAND_TIMEOUT_MS * CRC_RETRY_ATTEMPTS) {
        if (motors[i].motor->ReadMultiTurnAngle(motors[i].id)) {
          readSuccess = true;
          break;
        }
        Serial.print("M");
        Serial.print(i + 1);
        Serial.print(" 0x92 CRC mismatch (Attempt ");
        Serial.print(retry + 1);
        Serial.println(")");
        retry++;
      }
      rs485Active = false;
      if (!readSuccess) {
        Serial.println("Failed after retries");
        continue;
      }
      MyActuatorRMDX6V3::Feedback feedback = motors[i].motor->getLastFeedback();
      if (feedback.command == 0x92) {
        float shaftAngle = feedback.shaftAngle / 100.0;
        Serial.print("ShaftAngle=");
        Serial.print(shaftAngle);
        Serial.print("°");
        int32_t minLimit = min(motors[i].minPos, motors[i].maxPos) / 100;
        int32_t maxLimit = max(motors[i].minPos, motors[i].maxPos) / 100;
        if (shaftAngle < minLimit - 0.5 || shaftAngle > maxLimit + 0.5) {
          Serial.print(", Status=Encoder Error");
        }
        Serial.println();
      } else {
        Serial.println("Failed");
      }
    }
    initialPositionRead = true;
    lastDebug = millis();
    lastXChannel = channels[X_CHANNEL];
    lastYChannel = channels[Y_CHANNEL];
  }
#endif

#if DEBUG_FEEDBACK
  uint32_t loopTime = millis() - loopStart;
  Serial.printf("Loop runtime: %lu ms\n", loopTime);
#endif
  lastLoop = millis();
}