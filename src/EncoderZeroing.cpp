#include "EncoderZeroing.h"

// Initialize static members
const uint8_t EncoderZeroing::motorIDs[] = {0x01, 0x02, 0x03, 0x04};
const uint8_t EncoderZeroing::numMotors = 4;

// Constructor
EncoderZeroing::EncoderZeroing(MyActuatorRMDX6V3* motors[], uint8_t jumperPin)
    : motors(motors), jumperPin(jumperPin), hasRun(false) {
    pinMode(jumperPin, INPUT); // Set jumper pin as input
}

// Zeroes encoders if jumper is HIGH and routine hasn't run
bool EncoderZeroing::zeroEncoders() {
    // Check if routine has already run
    if (hasRun) {
        Serial.println("Encoder zeroing already completed");
        return false;
    }

    // Check if jumper is installed (HIGH)
    if (digitalRead(jumperPin) != HIGH) {
        Serial.println("Jumper not detected, skipping encoder zeroing");
        return false;
    }

    // Step 1: Read and print initial shaft angles for all motors
    Serial.println("\n--- Initial Shaft Angles ---");
    for (uint8_t i = 0; i < numMotors; i++) {
        uint8_t motorID = motorIDs[i];
        // Send command 0x92 to read multi-turn angle
        if (motors[i]->ReadMultiTurnAngle(motorID)) {
            // Get feedback to retrieve shaft angle
            MyActuatorRMDX6V3::Feedback feedback = motors[i]->getFeedback();
            // Print angle in degrees (shaftAngle is in 0.01° units)
            Serial.printf("Motor ID 0x%02X: Shaft Angle = %.2f degrees\n", 
                          motorID, feedback.shaftAngle / 100.0);
        } else {
            // Print error if command fails
            Serial.printf("Motor ID 0x%02X: Failed to read shaft angle\n", motorID);
        }
        delay(10); // Small delay to avoid overwhelming the bus
    }

    // Step 2: Set zero position and reset each motor
    Serial.println("\n--- Setting Zero Position and Resetting Motors ---");
    for (uint8_t i = 0; i < numMotors; i++) {
        uint8_t motorID = motorIDs[i];
        // Send command 0x64 to set current position as zero
        if (motors[i]->SetCurrentPositionAsZero(motorID)) {
            Serial.printf("Motor ID 0x%02X: Zero position set\n", motorID);
        } else {
            Serial.printf("Motor ID 0x%02X: Failed to set zero position\n", motorID);
        }
        // Wait 100ms as specified
        delay(100);
        // Send command 0x76 to reset the motor
        if (motors[i]->SystemReset(motorID)) {
            Serial.printf("Motor ID 0x%02X: System reset\n", motorID);
        } else {
            Serial.printf("Motor ID 0x%02X: Failed to reset system\n", motorID);
        }
        delay(100); // Small delay to avoid overwhelming the bus
    }

    // Step 3: Wait 500ms before reading final shaft angles
    Serial.println("\nWaiting 500ms...");
    delay(500);

    // Step 4: Read and print final shaft angles for all motors
    Serial.println("\n--- Final Shaft Angles After Zeroing and Reset ---");
    for (uint8_t i = 0; i < numMotors; i++) {
        uint8_t motorID = motorIDs[i];
        // Send command 0x92 to read multi-turn angle
        if (motors[i]->ReadMultiTurnAngle(motorID)) {
            // Get feedback to retrieve shaft angle
            MyActuatorRMDX6V3::Feedback feedback = motors[i]->getFeedback();
            // Print angle in degrees (shaftAngle is in 0.01° units)
            Serial.printf("Motor ID 0x%02X: Shaft Angle = %.2f degrees\n", 
                          motorID, feedback.shaftAngle / 100.0);
        } else {
            // Print error if command fails
            Serial.printf("Motor ID 0x%02X: Failed to read shaft angle\n", motorID);
        }
        delay(10); // Small delay to avoid overwhelming the bus
    }

    // Mark routine as completed
    hasRun = true;
    Serial.println("Encoder zeroing completed");
    return true;
}// EncoderZeroing.cpp  <92 Lines>