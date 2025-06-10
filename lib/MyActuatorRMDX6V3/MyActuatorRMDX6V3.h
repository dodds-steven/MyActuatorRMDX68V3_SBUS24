#ifndef MYACTUATOR_RMDX6V3_H
#define MYACTUATOR_RMDX6V3_H

#include <Arduino.h>
#include "RS485Comm.h"

class MyActuatorRMDX6V3 {
public:
    struct Feedback {
        uint8_t command = 0;
        int8_t temperature = 0;
        int16_t current = 0;
        int16_t speed = 0;
        int32_t encoderPos = 0; // Raw encoder ticks (0x60, 0x61)
        int32_t encoderOff = 0; // Encoder offset ticks (0x62)
        int32_t shaftAngle = 0; // Multi-turn angle in 0.01° (0x92, ±36,000°)
        int16_t angle = 0;      // Delta angle in 0.01° (0xA4, 0xA8, 0xA1)
        uint8_t brake = 0;
        float voltage = 0.0;
        uint16_t error = 0;
        String errorMessage = "";
        uint8_t runmode = 0;    // Run mode (0x20)
        uint8_t pid[6] = {0};
        uint32_t acceleration = 0;
        uint32_t model = 0;
        uint32_t softwareVersion = 0;
        uint32_t hardwareVersion = 0;
        uint8_t deviceAddress = 0;
        bool active = false;
    };

    MyActuatorRMDX6V3(HardwareSerial& serial, uint8_t dirPin);
    void begin(uint32_t baud);

    // Control Commands
    bool MotorShutdown(uint8_t motorID);
    bool MotorPause(uint8_t motorID); // 0x81
    bool BrakeRelease(uint8_t motorID);
    bool AbsolutePositionClosedLoopControl(uint8_t motorID, int32_t positionUnits, uint16_t maxSpeed);
    bool IncrementalPositionClosedLoopControl(uint8_t motorID, float positionDeg, uint16_t maxSpeed);
    bool TorqueClosedLoopControl(uint8_t motorID, int16_t torque);

    // Status Commands
    bool ReadSingleTurnEncoderPosition(uint8_t motorID); // 0x60
    bool ReadMultiTurnEncoderPosition(uint8_t motorID);  // 0x61
    bool ReadMultiTurnEncoderZeroOffset(uint8_t motorID); // 0x62
    bool ReadMultiTurnAngle(uint8_t motorID);            // 0x92
    bool ReadSystemStatus(uint8_t motorID);              // 0x20

    // PID Commands
    bool ReadPositionPIDParameters(uint8_t motorID);
    bool ReadVelocityPIDParameters(uint8_t motorID);
    bool ReadTorquePIDParameters(uint8_t motorID);
    bool WritePositionPIDParametersToRAM(uint8_t motorID, uint8_t pP, uint8_t pI, uint8_t sP, uint8_t sI, uint8_t tP, uint8_t tI);
    bool WriteVelocityPIDParametersToRAM(uint8_t motorID, uint8_t pP, uint8_t pI, uint8_t sP, uint8_t sI, uint8_t tP, uint8_t tI);
    bool WriteTorquePIDParametersToRAM(uint8_t motorID, uint8_t pP, uint8_t pI, uint8_t sP, uint8_t sI, uint8_t tP, uint8_t tI);
    bool WritePositionPIDParametersToROM(uint8_t motorID, uint8_t pP, uint8_t pI, uint8_t sP, uint8_t sI, uint8_t tP, uint8_t tI);
    bool WriteVelocityPIDParametersToROM(uint8_t motorID, uint8_t pP, uint8_t pI, uint8_t sP, uint8_t sI, uint8_t tP, uint8_t tI);
    bool WriteTorquePIDParametersToROM(uint8_t motorID, uint8_t pP, uint8_t pI, uint8_t sP, uint8_t sI, uint8_t tP, uint8_t tI);

    Feedback getFeedback() const;
    Feedback getLastFeedback() const;

private:
    RS485Comm _comm;
    Feedback _lastFeedback;
    int32_t _zeroOffset = 0;
    bool processFeedback(uint8_t cmd, uint8_t *packet);
    String decodeError(uint16_t errorCode);
};

#endif