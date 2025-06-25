#include <Arduino.h>
#include <HandleSBUS.h>
#include <MotorController.h>
#include <MotorModeController.h>
#include <Definitions.h>
#include <MyActuatorRMDX6V3.h>

#define setupPause false

MotorConfig motors[NUM_MOTORS] = {
  {0x01, false, (int32_t)(MOTOR1_MIN * 100), (int32_t)(MOTOR1_MAX * 100)}, // FrontLeft
  {0x02, true, (int32_t)(MOTOR2_MIN * 100), (int32_t)(MOTOR2_MAX * 100)},  // FrontRight
  {0x03, false, (int32_t)(MOTOR3_MIN * 100), (int32_t)(MOTOR3_MAX * 100)}, // BackRight
  {0x04, true, (int32_t)(MOTOR4_MIN * 100), (int32_t)(MOTOR4_MAX * 100)}   // BackLeft
};

uint16_t sbusChannels[SBUS_CHANNELS];
SBUSHandler sbusHandler;
MotorController motorController(motors, NUM_MOTORS);
MotorModeController motorModeController(motorController, motors, sbusChannels);

void setup() {
  Serial.begin(115200);

  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    motors[i].motor = new MyActuatorRMDX6V3(RS485_PORT, RS485_DIR_PIN);
  }

  sbusHandler.begin();

  if (!motorController.initialize()) {
    while (true) {
      Serial.println("Motor initialization failed");
      delay(1000);
    }
  }

  if (!motorController.calibrate()) {
    while (true) {
      Serial.println("Motor calibration failed");
      delay(1000);
    }
  }

  if (setupPause) {
    while (!Serial.available()) {
      delay(100);
    }
  }
}

void loop() {
  float xNorm, yNorm;
  if (sbusHandler.readChannels(xNorm, yNorm, sbusChannels)) {
    // Channels updated
  }

  motorModeController.update();

  bool positionSet[NUM_MOTORS] = {false};
  for (int i = 0; i < NUM_MOTORS; i++) {
    int32_t targetPos = motorModeController.getMotorPosition(i);
    if (motorController.setPosition(i, targetPos, 500)) {
      positionSet[i] = true;
    }
  }

  uint32_t pollStart = millis();
  bool reached[NUM_MOTORS] = {false};
  while (millis() - pollStart < 100) {
    bool allReached = true;
    for (int i = 0; i < NUM_MOTORS; i++) {
      if (!positionSet[i] || reached[i]) continue;
      allReached = false;
      MyActuatorRMDX6V3::Feedback feedback;
      if (motorController.getFeedback(i, feedback) && feedback.command == 0x92) {
        float currentAngle = feedback.shaftAngle;
        if (abs(currentAngle - motorModeController.getMotorPosition(i)) <= 200) {
          reached[i] = true;
        }
      }
    }
    if (allReached) break;
  }
}

// File: main.cpp (84 lines)