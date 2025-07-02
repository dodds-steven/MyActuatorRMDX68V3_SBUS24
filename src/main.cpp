#include <Arduino.h>
#include <MotorModeController.h>
#include <HandleSBUS.h>
#include <MyActuatorRMDX6V3.h>
#include <Definitions.h>
#include <EncoderZeroing.h>

// Global motors array
MotorConfig motors[NUM_MOTORS];
uint16_t sbusChannels[24];
SBUSHandler sbusHandler(true);
MyActuatorRMDX6V3 motor1(RS485_PORT, RS485_DIR_PIN);
MyActuatorRMDX6V3 motor2(RS485_PORT, RS485_DIR_PIN);
MyActuatorRMDX6V3 motor3(RS485_PORT, RS485_DIR_PIN);
MyActuatorRMDX6V3 motor4(RS485_PORT, RS485_DIR_PIN);
MyActuatorRMDX6V3 *motorArray[NUM_MOTORS] = {&motor1, &motor2, &motor3, &motor4};
EncoderZeroing encoderZeroing(motorArray); // Instantiate with motor array
MotorController motorController(motors, NUM_MOTORS);
MotorModeController motorModeController(motorController, sbusChannels);

void setup()
{
  // Setup onboard LED pin for heartbeat activity light
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); // Initialize LED to off
  // Initialize serial communication for debugging
  Serial.begin(115200);
  while (!Serial && millis() < 1000)
    ;

  // Debug Definitions.h constants
  Serial.println("Debugging Definitions.h constants:");
  Serial.print("MOTOR1_MIN=");
  Serial.print(MOTOR1_MIN);
  Serial.print("°, MOTOR1_MAX=");
  Serial.print(MOTOR1_MAX);
  Serial.println("°");
  Serial.print("MOTOR2_MIN=");
  Serial.print(MOTOR2_MIN);
  Serial.print("°, MOTOR2_MAX=");
  Serial.print(MOTOR2_MAX);
  Serial.println("°");
  Serial.print("MOTOR3_MIN=");
  Serial.print(MOTOR3_MIN);
  Serial.print("°, MOTOR3_MAX=");
  Serial.print(MOTOR3_MAX);
  Serial.println("°");
  Serial.print("MOTOR4_MIN=");
  Serial.print(MOTOR4_MIN);
  Serial.print("°, MOTOR4_MAX=");
  Serial.print(MOTOR4_MAX);
  Serial.println("°");
  Serial.print("MOTOR1_UP_IS_POSITIVE=");
  Serial.print(MOTOR1_UP_IS_POSITIVE);
  Serial.print(", MOTOR2_UP_IS_POSITIVE=");
  Serial.print(MOTOR2_UP_IS_POSITIVE);
  Serial.print(", MOTOR3_UP_IS_POSITIVE=");
  Serial.print(MOTOR3_UP_IS_POSITIVE);
  Serial.print(", MOTOR4_UP_IS_POSITIVE=");
  Serial.println(MOTOR4_UP_IS_POSITIVE);

  // Initialize motors array
  motors[0].id = 0x01;
  motors[0].UpIsPositive = MOTOR1_UP_IS_POSITIVE;
  motors[0].minPos = -11500; // MOTOR1_MIN * 100
  motors[0].maxPos = -500;   // MOTOR1_MAX * 100
  motors[0].motor = &motor1;
  motors[0].commActive = false;
  motors[0].safeToMove = false;
  motors[1].id = 0x02;
  motors[1].UpIsPositive = MOTOR2_UP_IS_POSITIVE;
  motors[1].minPos = 500;   // MOTOR2_MIN * 100
  motors[1].maxPos = 11500; // MOTOR2_MAX * 100
  motors[1].motor = &motor2;
  motors[1].commActive = false;
  motors[1].safeToMove = false;
  motors[2].id = 0x03;
  motors[2].UpIsPositive = MOTOR3_UP_IS_POSITIVE;
  motors[2].minPos = -11500; // MOTOR3_MIN * 100
  motors[2].maxPos = -500;   // MOTOR3_MAX * 100
  motors[2].motor = &motor3;
  motors[2].commActive = false;
  motors[2].safeToMove = false;
  motors[3].id = 0x04;
  motors[3].UpIsPositive = MOTOR4_UP_IS_POSITIVE;
  motors[3].minPos = 500;   // MOTOR4_MIN * 100
  motors[3].maxPos = 11500; // MOTOR4_MAX * 100
  motors[3].motor = &motor4;
  motors[3].commActive = false;
  motors[3].safeToMove = false;

  // Debug motors array after initialization
  Serial.println("Debugging motors array after initialization:");
  for (uint8_t i = 0; i < NUM_MOTORS; i++)
  {
    Serial.print("M");
    Serial.print(i + 1);
    Serial.print(" ID=0x");
    Serial.print(motors[i].id, HEX);
    Serial.print(", UpIsPositive=");
    Serial.print(motors[i].UpIsPositive);
    Serial.print(", minPos=");
    Serial.print(motors[i].minPos / 100.0f);
    Serial.print("°, maxPos=");
    Serial.print(motors[i].maxPos / 100.0f);
    Serial.print("°, motor=");
    Serial.println((uintptr_t)motors[i].motor, HEX);
  }

  // Zero all lifter motors on first run
  // Run encoder zeroing if jumper is not intalled as defined in the Definitions.h is not installed you must do this once before using the motors!!
  if (encoderZeroing.zeroEncoders())
  {
    Serial.println("Encoder zeroing completed successfully");
  }
  else
  {
    Serial.println("Encoder zeroing skipped or already done");
  }

  // Initialize SBUS and motor controller
  sbusHandler.begin();
  motorController.init();

  // Debug motors array after controller init
  Serial.println("Debugging motors array after MotorController init:");
  for (uint8_t i = 0; i < NUM_MOTORS; i++)
  {
    Serial.print("M");
    Serial.print(i + 1);
    Serial.print(" ID=0x");
    Serial.print(motors[i].id, HEX);
    Serial.print(", UpIsPositive=");
    Serial.print(motors[i].UpIsPositive);
    Serial.print(", minPos=");
    Serial.print(motors[i].minPos / 100.0f);
    Serial.print("°, maxPos=");
    Serial.print(motors[i].maxPos / 100.0f);
    Serial.print("°, motor=");
    Serial.println((uintptr_t)motors[i].motor, HEX);
  }
}

void loop()
{
  static uint32_t lastLoopTime = 0;     // Last time main loop ran
  static uint32_t lastLEDToggle = 0;    // Last time LED was toggled
  static bool ledState = false;         // LED state (false = off, true = on)
  uint32_t currentTime = millis();      // Cache current time

  // Throttle main loop to 25ms
  if (currentTime - lastLoopTime < 25) {
    return;
  }

  // Update loop time
  lastLoopTime = currentTime;

  // Toggle LED every 1000ms (500ms on, 500ms off)
  if (currentTime - lastLEDToggle >= 1000) {
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState ? HIGH : LOW);
    lastLEDToggle = currentTime;
  }

  uint32_t loopStart = millis();
  motorModeController.update();
  Serial.printf("Loop time: %dms\n", millis() - loopStart);
}
// File: main.cpp (152 lines)