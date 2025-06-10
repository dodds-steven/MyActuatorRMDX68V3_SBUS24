#include <Arduino.h>
#include "SBUS.h"
#include "MyActuatorRMDX6V3.h"
#include "RS485Comm.h"

// SBUS configuration
#define SBUS_RX_PIN 0  // Serial1 RX
#define SBUS_TX_PIN 1  // Serial1 TX
#define SBUS_BAUD 100000
#define X_CHANNEL 10  // Index 10 = Channel 11 (X gimbal)
#define Y_CHANNEL 9   // Index 9 = Channel 10 (Y gimbal)
SBUS sbus(Serial1);

// RS-485 configuration
#define RS485_SERIAL Serial4
#define RS485_DIR_PIN 41
#define RS485_BAUD 115200

// Motor configuration
const int32_t POS_THRESHOLD = 50;      // 0.5 degree in 0.01° units
const uint16_t SBUS_MIN = 172;         // Observed min
const uint16_t SBUS_MAX = 1811;        // Observed max
const uint16_t SBUS_CENTER = 992;      // SBUS center
const uint16_t SPEED_DPS = 500;        // 50 dps
const uint32_t TEST_DURATION = 10000;  // 10 seconds
const uint32_t LOOP_INTERVAL_MS = 10;  // 10ms loop
const uint32_t COMMAND_INTERVAL = 50;  // 50ms between 0xA4
const uint32_t DEBUG_INTERVAL_MS = 10000;  // 10 seconds
const bool VERBOSE_DEBUG = false;      // Disable debug
const uint32_t SBUS_RETRY_INTERVAL = 25; // 25ms between SBUS retries

// Motor limit configuration
const int32_t MOTOR1_MIN_POS = -600;    // -6°, highest height
const int32_t MOTOR1_MAX_POS = -13000;  // -130°, lowest height
const int32_t MOTOR1_CENTER_POS = -6800; // -68°, center
const float MOTOR1_RANGE_SCALE = 6200.0; // Half of travel range
const int32_t MOTOR2_MIN_POS = -15200;  // -152°, highest height
const int32_t MOTOR2_MAX_POS = -28900;  // -289°, lowest height
const int32_t MOTOR2_CENTER_POS = -22050; // -220.5°, center
const float MOTOR2_RANGE_SCALE = 6850.0; // Half of travel range
const int32_t MOTOR3_MIN_POS = -600;    // Placeholder
const int32_t MOTOR3_MAX_POS = -13000;
const int32_t MOTOR3_CENTER_POS = -6800;
const float MOTOR3_RANGE_SCALE = 6200.0;
const int32_t MOTOR4_MIN_POS = -600;    // Placeholder
const int32_t MOTOR4_MAX_POS = -13000;
const int32_t MOTOR4_CENTER_POS = -6800;
const float MOTOR4_RANGE_SCALE = 6200.0;

// Motor definitions
struct MotorConfig {
  uint8_t id;
  int32_t minPos;    // in 0.01° units
  int32_t maxPos;
  int32_t centerPos;
  bool invertDirection;
  MyActuatorRMDX6V3* motor;
  float rangeScale;  // Half of travel range
  bool safeToMove;   // Flag to allow movement
};
MotorConfig motors[] = {
  {0x01, 0, 0, 0, true, new MyActuatorRMDX6V3(RS485_SERIAL, RS485_DIR_PIN), 0, false},  // Front Left
  {0x02, 0, 0, 0, false, new MyActuatorRMDX6V3(RS485_SERIAL, RS485_DIR_PIN), 0, false}, // Front Right
  {0x03, 0, 0, 0, false, nullptr, 0, false},  // Back Right
  {0x04, 0, 0, 0, true, nullptr, 0, false}    // Back Left
};
const uint8_t NUM_MOTORS = 4;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 1000);
  Serial.println("Starting SBUS Motor Control Test (10s, Motor 1 & 2)...");

  // Assign motor limits
  motors[0].minPos = MOTOR1_MIN_POS;
  motors[0].maxPos = MOTOR1_MAX_POS;
  motors[0].centerPos = MOTOR1_CENTER_POS;
  motors[0].rangeScale = MOTOR1_RANGE_SCALE;
  motors[1].minPos = MOTOR2_MIN_POS;
  motors[1].maxPos = MOTOR2_MAX_POS;
  motors[1].centerPos = MOTOR2_CENTER_POS;
  motors[1].rangeScale = MOTOR2_RANGE_SCALE;
  motors[2].minPos = MOTOR3_MIN_POS;
  motors[2].maxPos = MOTOR3_MAX_POS;
  motors[2].centerPos = MOTOR3_CENTER_POS;
  motors[2].rangeScale = MOTOR3_RANGE_SCALE;
  motors[3].minPos = MOTOR4_MIN_POS;
  motors[3].maxPos = MOTOR4_MAX_POS;
  motors[3].centerPos = MOTOR4_CENTER_POS;
  motors[3].rangeScale = MOTOR4_RANGE_SCALE;

  // Initialize SBUS with inversion
  sbus.begin(SBUS_RX_PIN, SBUS_TX_PIN, true, SBUS_BAUD);
  sbus.setEndPoints(X_CHANNEL, SBUS_MIN, SBUS_MAX);
  sbus.setEndPoints(Y_CHANNEL, SBUS_MIN, SBUS_MAX);
  
  // Initialize motors
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    if (motors[i].motor) {
      motors[i].motor->begin(RS485_BAUD);
      Serial.print("M");
      Serial.print(i + 1);
      Serial.print(" 0x77 Feedback: ");
      if (!motors[i].motor->BrakeRelease(motors[i].id)) {  // 0x77
        Serial.println("Failed");
      } else {
        Serial.println("OK");
      }
      delay(100);
    }
  }
  Serial.println("Motor communication initialized.");
}

float normalizeSbus(uint16_t sbusValue) {
  sbusValue = constrain(sbusValue, SBUS_MIN, SBUS_MAX);
  float norm = (float)(sbusValue - SBUS_CENTER) / (SBUS_MAX - SBUS_CENTER);
  return constrain(norm, -1.0, 1.0);
}

int32_t computeMotorPosition(float xNorm, float yNorm, uint8_t motorIndex) {
  int32_t basePos = motors[motorIndex].centerPos;
  float offset;
  switch (motorIndex) {
    case 0: // Front Left: Lower with left X (-xNorm), forward Y (+yNorm)
      offset = (motors[motorIndex].invertDirection ? (xNorm - yNorm) : (-xNorm + yNorm)) * motors[motorIndex].rangeScale;
      break;
    case 1: // Front Right: Lower with right X (+xNorm), forward Y (+yNorm)
      offset = (motors[motorIndex].invertDirection ? (-xNorm - yNorm) : (xNorm + yNorm)) * motors[motorIndex].rangeScale;
      break;
    case 2: // Back Right: Raise with left X (-xNorm), forward Y (+yNorm)
      offset = (xNorm - yNorm) * motors[motorIndex].rangeScale;
      break;
    case 3: // Back Left: Raise with right X (+xNorm), forward Y (+yNorm)
      offset = (-xNorm - yNorm) * motors[motorIndex].rangeScale;
      break;
    default:
      offset = 0;
  }
  int32_t motorPos = basePos + (int32_t)offset;
  int32_t minLimit = min(motors[motorIndex].minPos, motors[motorIndex].maxPos);
  int32_t maxLimit = max(motors[motorIndex].minPos, motors[motorIndex].maxPos);
  return constrain(motorPos, minLimit, maxLimit);
}

void loop() {
  static uint32_t startTime = millis();
  static uint32_t lastDebug = 0;
  static uint32_t lastLoop = 0;
  static uint32_t lastCommand[NUM_MOTORS] = {0};
  static uint16_t lastXChannel = SBUS_CENTER;
  static uint16_t lastYChannel = SBUS_CENTER;
  static int32_t lastPos[NUM_MOTORS] = {-13000, -28900, -6800, -6800};
  static uint16_t lastValidChannels[24] = {992};
  static bool hasValidSBUS = false;
  static bool initialPositionRead = false;
  uint16_t channels[24];
  bool failsafe;
  bool lostFrame;
  static uint32_t lastSBUSAttempt = 0;

  if (millis() - startTime > TEST_DURATION) {
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
      if (motors[i].motor) {
        Serial.print("M");
        Serial.print(i + 1);
        Serial.print(" 0x80 Feedback: ");
        if (!motors[i].motor->MotorShutdown(motors[i].id)) {
          Serial.println("Failed");
        } else {
          Serial.println("OK");
        }
      }
    }
    Serial.println("Test completed, motors stopped.");
    while (true);
  }

  if (millis() - lastLoop < LOOP_INTERVAL_MS) {
    return;
  }
  uint32_t loopStart = millis();

  // Non-blocking SBUS read
  if (millis() - lastSBUSAttempt >= SBUS_RETRY_INTERVAL) {
    if (sbus.read(&channels[0], &failsafe, &lostFrame)) {
      if (failsafe) {
        Serial.println("SBUS Failsafe detected, using last valid inputs...");
      } else {
        hasValidSBUS = true;
        bool channelsChanged = false;
        for (uint8_t j = 0; j < 24; j++) {
          if (channels[j] != lastValidChannels[j]) {
            channelsChanged = true;
            break;
          }
        }
        if (channelsChanged || !hasValidSBUS) {
          memcpy(lastValidChannels, channels, sizeof(channels));
        }
      }
    } else if (!hasValidSBUS) {
      Serial.print("SBUS read failed, no valid data...");
      if (VERBOSE_DEBUG) {
        Serial.print(" Serial1 available: ");
        Serial.print(Serial1.available());
        Serial.println(" bytes");
      }
      Serial.println();
    }
    lastSBUSAttempt = millis();
  }

  if (!hasValidSBUS) {
    return;
  }
  memcpy(channels, lastValidChannels, sizeof(channels));

  uint16_t xChannel = channels[X_CHANNEL];
  uint16_t yChannel = channels[Y_CHANNEL];
  float xNorm = normalizeSbus(xChannel);
  float yNorm = normalizeSbus(yChannel);

  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    if (!motors[i].motor || !motors[i].safeToMove) continue;
    int32_t targetPos = computeMotorPosition(xNorm, yNorm, i);
    if (abs(targetPos - lastPos[i]) > POS_THRESHOLD && millis() - lastCommand[i] >= COMMAND_INTERVAL) {
      Serial.print("M");
      Serial.print(i + 1);
      Serial.print(" 0xA4 Feedback: ");
      if (!motors[i].motor->AbsolutePositionClosedLoopControl(motors[i].id, targetPos, SPEED_DPS)) {
        Serial.println("Failed");
      } else {
        MyActuatorRMDX6V3::Feedback feedback = motors[i].motor->getLastFeedback();
        Serial.print("Temp=");
        Serial.print(feedback.temperature);
        Serial.print("°C, Current=");
        Serial.print(feedback.current / 100.0);
        Serial.print("A, Speed=");
        Serial.print(feedback.speed / 10.0);
        Serial.print("dps, Delta=");
        Serial.print(feedback.shaftAngle / 100.0);
        Serial.println(" degrees");
      }
      lastPos[i] = targetPos;
      lastCommand[i] = millis();
    }
  }

  if (millis() - lastDebug >= DEBUG_INTERVAL_MS || abs(xChannel - lastXChannel) > 10 || abs(yChannel - lastYChannel) > 10 || !initialPositionRead) {
    Serial.print("X Channel (11): ");
    Serial.print(xChannel);
    Serial.print(", Y Channel (10): ");
    Serial.print(yChannel);
    Serial.print(", Ch9: ");
    Serial.print(channels[8]);
    Serial.print(", Ch12: ");
    Serial.print(channels[11]);
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
      Serial.print(", M");
      Serial.print(i + 1);
      Serial.print(" Target: ");
      int32_t targetPos = computeMotorPosition(xNorm, yNorm, i);
      Serial.print(targetPos / 100.0);
      Serial.print("°");
      if (motors[i].motor) {
        Serial.print(" M");
        Serial.print(i + 1);
        Serial.print(" 0x92 Feedback: ");
        if (!initialPositionRead) {
          // Initial position read
          if (motors[i].motor->ReadMultiTurnAngle(motors[i].id)) {
            MyActuatorRMDX6V3::Feedback feedback = motors[i].motor->getLastFeedback();
            if (feedback.command == 0x92) {
              float shaftAngle = feedback.shaftAngle / 100.0;
              Serial.print("ShaftAngle=");
              Serial.print(shaftAngle);
              Serial.print(" degrees");
              int32_t minLimit = min(motors[i].minPos, motors[i].maxPos) / 100;
              int32_t maxLimit = max(motors[i].minPos, motors[i].maxPos) / 100;
              Serial.print(" [Safety Check: Min=");
              Serial.print(minLimit);
              Serial.print("°, Max=");
              Serial.print(maxLimit);
              Serial.print("°, Tolerance=±0.5°]");
              if (shaftAngle >= minLimit - 0.5 && shaftAngle <= maxLimit + 0.5) {
                motors[i].safeToMove = true;
                Serial.print(" [Safe to Move]");
                // Initialize to robot low
                int32_t lowPos = (i == 0) ? MOTOR1_MAX_POS : MOTOR2_MAX_POS;
                Serial.print(" M");
                Serial.print(i + 1);
                Serial.print(" 0xA4 Feedback: ");
                if (!motors[i].motor->AbsolutePositionClosedLoopControl(motors[i].id, lowPos, SPEED_DPS)) {
                  Serial.println("Failed");
                  motors[i].safeToMove = false;
                } else {
                  MyActuatorRMDX6V3::Feedback initFeedback = motors[i].motor->getLastFeedback();
                  Serial.print("Temp=");
                  Serial.print(initFeedback.temperature);
                  Serial.print("°C, Current=");
                  Serial.print(initFeedback.current / 100.0);
                  Serial.print("A, Speed=");
                  Serial.print(initFeedback.speed / 10.0);
                  Serial.print("dps, Delta=");
                  Serial.print(initFeedback.shaftAngle / 100.0);
                  Serial.println(" degrees");
                }
              } else {
                motors[i].safeToMove = false;
                Serial.print(" [Out of Range, Skipping Movement]");
              }
              Serial.print(" Current: ");
              Serial.print(shaftAngle);
              Serial.print("°");
            } else {
              Serial.print("Failed");
              motors[i].safeToMove = false;
            }
          } else {
            Serial.print("Failed");
            motors[i].safeToMove = false;
          }
        } else if (motors[i].safeToMove) {
          // Regular position read for safe motors
          if (motors[i].motor->ReadMultiTurnAngle(motors[i].id)) {
            MyActuatorRMDX6V3::Feedback feedback = motors[i].motor->getLastFeedback();
            if (feedback.command == 0x92) {
              float shaftAngle = feedback.shaftAngle / 100.0;
              Serial.print("ShaftAngle=");
              Serial.print(shaftAngle);
              Serial.print(" degrees");
              int32_t minLimit = min(motors[i].minPos, motors[i].maxPos) / 100;
              int32_t maxLimit = max(motors[i].minPos, motors[i].maxPos) / 100;
              if (shaftAngle < minLimit - 0.5 || shaftAngle > maxLimit + 0.5) {
                Serial.print(" [Encoder Error]");
              }
              Serial.print(" Current: ");
              Serial.print(shaftAngle);
              Serial.print("°");
            } else {
              Serial.print("Failed");
            }
          } else {
            Serial.print("Failed");
          }
        } else {
          Serial.println("Skipped: Unsafe Position");
        }
      }
    }
    Serial.println();
    if (VERBOSE_DEBUG) {
      Serial.print("Loop time: ");
      Serial.print(millis() - loopStart);
      Serial.println("ms");
    }
    initialPositionRead = true;
    lastDebug = millis();
    lastXChannel = xChannel;
    lastYChannel = yChannel;
  }

  lastLoop = millis();
}