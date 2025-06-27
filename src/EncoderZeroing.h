#ifndef ENCODER_ZEROING_H
#define ENCODER_ZEROING_H

#include <Arduino.h>
#include "MyActuatorRMDX6V3.h"
#include "Definitions.h"

class EncoderZeroing {
public:
    // Constructor: Takes array of motor pointers and optional jumper pin
    EncoderZeroing(MyActuatorRMDX6V3* motors[], uint8_t jumperPin = JUMPER_PIN);

    // Executes the encoder zeroing routine if jumper is HIGH and hasn't run
    bool zeroEncoders();

private:
    MyActuatorRMDX6V3** motors; // Pointer to array of motor pointers
    uint8_t jumperPin;          // Pin for jumper/switch detection
    bool hasRun;                // Flag to ensure routine runs only once
    static const uint8_t motorIDs[]; // Array of motor IDs
    static const uint8_t numMotors;  // Number of motors
};

#endif // ENCODER_ZEROING_H <24 Lines>