#include "MyActuatorRMDX6V3.h"

#define verboseDebug false
#define GEAR_RATIO 8.0
#define ENCODER_COUNTS_PER_REV 16384
#define DEGREES_PER_COUNT (360.0 / ENCODER_COUNTS_PER_REV / GEAR_RATIO) // ≈ 0.002746582°/count

MyActuatorRMDX6V3::MyActuatorRMDX6V3(HardwareSerial& serial, uint8_t dirPin)
    : _comm(serial, dirPin) {
    _lastFeedback = Feedback();
}

void MyActuatorRMDX6V3::begin(uint32_t baud) {
    _comm.begin(baud);
}

String MyActuatorRMDX6V3::decodeError(uint16_t errorCode) {
    if (errorCode == 0) return "No error";
    String errors = "";
    if (errorCode & 0x0002) errors += "Motor stall, ";
    if (errorCode & 0x0004) errors += "Low pressure, ";
    if (errorCode & 0x0008) errors += "Over-voltage, ";
    if (errorCode & 0x0010) errors += "Over-current, ";
    if (errorCode & 0x0040) errors += "Power overrun, ";
    if (errorCode & 0x0080) errors += "Calibration error, ";
    if (errorCode & 0x0100) errors += "Over-speed, ";
    if (errorCode & 0x1000) errors += "Over-temperature, ";
    if (errorCode & 0x2000) errors += "Encoder calibration error, ";
    if (errors == "") errors = "Invalid error code: " + String(errorCode);
    else errors.remove(errors.length() - 2);
    return errors;
}

bool MyActuatorRMDX6V3::processFeedback(uint8_t cmd, uint8_t *packet) {
    if (verboseDebug) {
        Serial.print("Received Packet: ");
        for (int i = 0; i < 13; i++) {
            Serial.printf("%02X ", packet[i]);
        }
        Serial.println();
        Serial.print("DATA[1-7]: ");
        for (int i = 4; i <= 10; i++) {
            Serial.printf("%02X ", packet[i]);
        }
        Serial.println();
    }

    _lastFeedback = Feedback();
    _lastFeedback.command = cmd;

    if (packet[3] != cmd) {
        Serial.printf("Warning: Command byte mismatch, expected 0x%02X, got packet[3]=0x%02X\n", cmd, packet[3]);
        return false;
    }

    switch (cmd) {
        case 0x60: // Single-Turn Encoder Position
        {
            int16_t position = (int16_t)(packet[7] | (packet[8] << 8)); // DATA[4-5]
            _lastFeedback.encoderPos = position;
            if (verboseDebug) {
                Serial.printf("0x%02X Raw: EncoderPos=%d ticks (%.2f degrees)\n",
                             cmd, position, position * DEGREES_PER_COUNT);
            }
            Serial.printf("0x%02X Feedback: EncoderPos=%d ticks (%.2f degrees)\n",
                         cmd, position, position * DEGREES_PER_COUNT);
            break;
        }
        case 0x61: // Multi-Turn Encoder Position
        {
            int32_t position = (int32_t)(packet[7] | (packet[8] << 8) | (packet[9] << 16) | (packet[10] << 24)); // DATA[4-7]
            _lastFeedback.encoderPos = position;
            if (verboseDebug) {
                Serial.printf("0x%02X Raw: EncoderPos=%d ticks (%.2f degrees)\n",
                             cmd, position, position * DEGREES_PER_COUNT);
            }
            Serial.printf("0x%02X Feedback: EncoderPos=%d ticks (%.2f degrees)\n",
                         cmd, position, position * DEGREES_PER_COUNT);
            break;
        }
        case 0x62: // Multi-Turn Encoder Zero Offset
        {
            int32_t offset = (int32_t)(packet[7] | (packet[8] << 8) | (packet[9] << 16) | (packet[10] << 24)); // DATA[4-7]
            _lastFeedback.encoderOff = offset;
            if (verboseDebug) {
                Serial.printf("0x%02X Raw: EncoderOff=%d ticks (%.2f degrees)\n",
                             cmd, offset, offset * DEGREES_PER_COUNT);
            }
            Serial.printf("0x%02X Feedback: EncoderOff=%d ticks (%.2f degrees)\n",
                         cmd, offset, offset * DEGREES_PER_COUNT);
            break;
        }
        case 0x64: // Set Current Position as Zero (updated to parse encoderOffset)
        {
            int32_t offset = (int32_t)(packet[7] | (packet[8] << 8) | (packet[9] << 16) | (packet[10] << 24)); // DATA[4-7]
            _lastFeedback.encoderOff = offset;
            if (verboseDebug) {
                Serial.printf("0x%02X Raw: EncoderOff=%d ticks (%.2f degrees)\n",
                             cmd, offset, offset * DEGREES_PER_COUNT);
            }
            Serial.printf("0x%02X Feedback: EncoderOff=%d ticks (%.2f degrees)\n",
                         cmd, offset, offset * DEGREES_PER_COUNT);
            break;
        }
        case 0x92: // Multi-Turn Angle
        {
            int32_t angle = (int32_t)(packet[7] | (packet[8] << 8) | (packet[9] << 16) | (packet[10] << 24)); // DATA[4-7]
            _lastFeedback.shaftAngle = angle;
            if (verboseDebug) {
                Serial.printf("0x%02X Raw: ShaftAngle=%d (%.2f degrees)\n",
                             cmd, angle, angle * 0.01);
                if (abs(angle) > 3600000) {
                    Serial.printf("Warning: ShaftAngle exceeds ±36,000° range\n");
                }
            }
            Serial.printf("0x%02X Feedback: ShaftAngle=%.2f degrees\n",
                         cmd, angle * 0.01);
            break;
        }
        case 0x20: // Read System Status
        {
            int8_t temp = (int8_t)packet[4]; // DATA[1]
            float voltage = (uint8_t)packet[5] * 0.1; // DATA[2]
            uint16_t error = (uint16_t)(packet[6] | (packet[7] << 8)); // DATA[3-4]
            uint8_t runmode = packet[9]; // DATA[5]
            _lastFeedback.temperature = temp;
            _lastFeedback.voltage = voltage;
            _lastFeedback.error = error;
            _lastFeedback.errorMessage = decodeError(error);
            _lastFeedback.runmode = runmode;
            if (verboseDebug) {
                Serial.printf("0x%02X Raw: Temp=%d, Voltage=%.1fV, Error=0x%04X, RunMode=0x%02X\n",
                             cmd, temp, voltage, error, runmode);
            }
            Serial.printf("0x%02X Feedback: Temp=%d°C, Voltage=%.1fV, Error=%s, RunMode=0x%02X\n",
                         cmd, temp, voltage, _lastFeedback.errorMessage.c_str(), runmode);
            break;
        }
        case 0xA4: case 0xA6: // Absolute Position
        {
            int8_t temp = (int8_t)packet[4]; // DATA[1]
            int16_t current = (int16_t)(packet[5] | (packet[6] << 8)); // DATA[2-3]
            int16_t speed = (int16_t)(packet[7] | (packet[8] << 8)); // DATA[4-5]
            int16_t delta = (int16_t)(packet[9] | (packet[10] << 8)); // DATA[6-7]
            _lastFeedback.temperature = temp;
            _lastFeedback.current = current;
            _lastFeedback.speed = speed;
            _lastFeedback.angle = delta;
            if (verboseDebug) {
                Serial.printf("0x%02X Raw: Temp=%d, Current=%d, Speed=%d, Delta=%d\n",
                             cmd, temp, current, speed, delta);
            }
            Serial.printf("0x%02X Feedback: Temp=%d°C, Current=%.2fA, Speed=%.1fdps, Delta=%.2f degrees\n",
                         cmd, temp, current * 0.01, speed * 0.1, delta * 0.01);
            _lastFeedback.active = true;
            break;
        }
        case 0xA8: // Incremental Position
        {
            int8_t temp = (int8_t)packet[4]; // DATA[1]
            int16_t current = (int16_t)(packet[5] | (packet[6] << 8)); // DATA[2-3]
            int16_t speed = (int16_t)(packet[7] | (packet[8] << 8)); // DATA[4-5]
            int16_t position = (int16_t)(packet[9] | (packet[10] << 8)); // DATA[6-7]
            _lastFeedback.temperature = temp;
            _lastFeedback.current = current;
            _lastFeedback.speed = speed;
            _lastFeedback.angle = position;
            if (verboseDebug) {
                Serial.printf("0x%02X Raw: Temp=%d, Current=%d, Speed=%d, Position=%d\n",
                             cmd, temp, current, speed, position);
            }
            Serial.printf("0x%02X Feedback: Temp=%d°C, Current=%.2fA, Speed=%.1fdps, Position=%.2f degrees\n",
                         cmd, temp, current * 0.01, speed * 0.1, position * 0.01);
            _lastFeedback.active = true;
            break;
        }
        case 0xA1: // Torque Closed-Loop Control
        {
            int8_t temp = (int8_t)packet[4]; // DATA[1]
            int16_t current = (int16_t)(packet[5] | (packet[6] << 8)); // DATA[2-3]
            int16_t speed = (int16_t)(packet[7] | (packet[8] << 8)); // DATA[4-5]
            int16_t angle = (int16_t)(packet[9] | (packet[10] << 8)); // DATA[6-7]
            _lastFeedback.temperature = temp;
            _lastFeedback.current = current;
            _lastFeedback.speed = speed;
            _lastFeedback.angle = angle;
            if (verboseDebug) {
                Serial.printf("0x%02X Raw: Temp=%d, Current=%d, Speed=%d, Angle=%d\n",
                             cmd, temp, current, speed, angle);
            }
            Serial.printf("0x%02X Feedback: Temp=%d°C, Current=%.2fA, Speed=%.1fdps, Angle=%.2f degrees\n",
                         cmd, temp, current * 0.01, speed * 0.1, angle * 0.01);
            _lastFeedback.active = true;
            break;
        }
        case 0x80: case 0x77: case 0x81: // No data
        {
            if (verboseDebug) {
                Serial.printf("0x%02X Raw: No data\n", cmd);
            }
            Serial.printf("0x%02X Feedback: No data\n", cmd);
            break;
        }
        case 0x76: // System Reset (handled separately in SystemReset, but included for completeness)
        {
            if (verboseDebug) {
                Serial.printf("0x%02X Raw: No data (system reset)\n", cmd);
            }
            Serial.printf("0x%02X Feedback: No data (system reset)\n", cmd);
            break;
        }
        default:
        {
            if (verboseDebug) {
                Serial.printf("0x%02X Raw: Unknown command\n", cmd);
            }
            Serial.printf("0x%02X Feedback: Unknown command\n", cmd);
            return false;
        }
    }

    return true;
}

// Control Commands
bool MyActuatorRMDX6V3::MotorShutdown(uint8_t motorID) {
    uint8_t data[8] = {0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    if (verboseDebug) {
        Serial.print("Sent Packet: ");
        for (int i = 0; i < 8; i++) {
            Serial.printf("%02X ", data[i]);
        }
        Serial.println();
    }
    uint8_t response[13];
    uint8_t responseLen = 0;
    uint8_t command = 0;
    if (!_comm.sendCommand(motorID, data, 8)) return false;
    if (!_comm.readFeedback(response, responseLen, command)) return false;
    return processFeedback(command, response);
}

bool MyActuatorRMDX6V3::MotorPause(uint8_t motorID) {
    uint8_t data[8] = {0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    if (verboseDebug) {
        Serial.print("Sent Packet: ");
        for (int i = 0; i < 8; i++) {
            Serial.printf("%02X ", data[i]);
        }
        Serial.println();
    }
    uint8_t response[13];
    uint8_t responseLen = 0;
    uint8_t command = 0;
    if (!_comm.sendCommand(motorID, data, 8)) return false;
    if (!_comm.readFeedback(response, responseLen, command)) return false;
    return processFeedback(command, response);
}

bool MyActuatorRMDX6V3::BrakeRelease(uint8_t motorID) {
    uint8_t data[8] = {0x77, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    if (verboseDebug) {
        Serial.print("Sent Packet: ");
        for (int i = 0; i < 8; i++) {
            Serial.printf("%02X ", data[i]);
        }
        Serial.println();
    }
    uint8_t response[13];
    uint8_t responseLen = 0;
    uint8_t command = 0;
    if (!_comm.sendCommand(motorID, data, 8)) return false;
    if (!_comm.readFeedback(response, responseLen, command)) return false;
    return processFeedback(command, response);
}

bool MyActuatorRMDX6V3::AbsolutePositionClosedLoopControl(uint8_t motorID, int32_t positionUnits, uint16_t maxSpeed) {
    uint8_t data[8];
    data[0] = 0xA4;
    data[1] = 0x00;
    data[2] = (uint8_t)(maxSpeed & 0xFF);
    data[3] = (uint8_t)(maxSpeed >> 8);
    data[4] = (uint8_t)(positionUnits & 0xFF);
    data[5] = (uint8_t)(positionUnits >> 8);
    data[6] = (uint8_t)(positionUnits >> 16);
    data[7] = (uint8_t)(positionUnits >> 24);
    if (verboseDebug) {
        Serial.print("Sent Packet: ");
        for (int i = 0; i < 8; i++) {
            Serial.printf("%02X ", data[i]);
        }
        Serial.println();
    }
    uint8_t response[13];
    uint8_t responseLen = 0;
    uint8_t command = 0;
    if (!_comm.sendCommand(motorID, data, 8)) return false;
    if (!_comm.readFeedback(response, responseLen, command)) return false;
    return processFeedback(command, response);
}

bool MyActuatorRMDX6V3::IncrementalPositionClosedLoopControl(uint8_t motorID, float positionDeg, uint16_t maxSpeed) {
    uint8_t data[8];
    data[0] = 0xA8;
    data[1] = 0x00;
    data[2] = (uint8_t)(maxSpeed & 0xFF);
    data[3] = (uint8_t)(maxSpeed >> 8);
    int32_t positionUnits = (int32_t)(positionDeg * 100);
    data[4] = (uint8_t)(positionUnits & 0xFF);
    data[5] = (uint8_t)(positionUnits >> 8);
    data[6] = (uint8_t)(positionUnits >> 16);
    data[7] = (uint8_t)(positionUnits >> 24);
    if (verboseDebug) {
        Serial.print("Sent Packet: ");
        for (int i = 0; i < 8; i++) {
            Serial.printf("%02X ", data[i]);
        }
        Serial.println();
    }
    uint8_t response[13];
    uint8_t responseLen = 0;
    uint8_t command = 0;
    if (!_comm.sendCommand(motorID, data, 8)) return false;
    if (!_comm.readFeedback(response, responseLen, command)) return false;
    return processFeedback(command, response);
}

bool MyActuatorRMDX6V3::TorqueClosedLoopControl(uint8_t motorID, int16_t torque) {
    uint8_t data[8];
    data[0] = 0xA1;
    data[1] = 0x00;
    data[2] = 0x00;
    data[3] = 0x00;
    data[4] = (uint8_t)(torque & 0xFF);
    data[5] = (uint8_t)(torque >> 8);
    data[6] = 0x00;
    data[7] = 0x00;
    if (verboseDebug) {
        Serial.print("Sent Packet: ");
        for (int i = 0; i < 8; i++) {
            Serial.printf("%02X ", data[i]);
        }
        Serial.println();
    }
    uint8_t response[13];
    uint8_t responseLen = 0;
    uint8_t command = 0;
    if (!_comm.sendCommand(motorID, data, 8)) return false;
    if (!_comm.readFeedback(response, responseLen, command)) return false;
    return processFeedback(command, response);
}

bool MyActuatorRMDX6V3::SetCurrentPositionAsZero(uint8_t motorID) {
    uint8_t data[8] = {0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    if (verboseDebug) {
        Serial.print("Sent Packet: ");
        for (int i = 0; i < 8; i++) {
            Serial.printf("%02X ", data[i]);
        }
        Serial.println();
    }
    uint8_t response[13];
    uint8_t responseLen = 0;
    uint8_t command = 0;
    if (!_comm.sendCommand(motorID, data, 8)) return false;
    if (!_comm.readFeedback(response, responseLen, command)) return false;
    return processFeedback(command, response);
}

bool MyActuatorRMDX6V3::SystemReset(uint8_t motorID) {
    // Construct data array for command 0x76
    uint8_t data[8] = {0x76, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    // Log sent packet if verbose debugging is enabled
    if (verboseDebug) {
        Serial.print("Sent Packet: ");
        for (int i = 0; i < 8; i++) {
            Serial.printf("%02X ", data[i]);
        }
        Serial.println();
    }
    // Send command via RS485Comm
    // Skip readFeedback since system reset may prevent a response
    if (!_comm.sendCommand(motorID, data, 8)) {
        return false;
    }
    // Assume success after sending, as per protocol behavior
    if (verboseDebug) {
        Serial.println("0x76 Raw: No data expected (system reset)");
    }
    Serial.println("0x76 Feedback: No data expected (system reset)");
    return true;
}

// Status Commands
bool MyActuatorRMDX6V3::ReadSingleTurnEncoderPosition(uint8_t motorID) {
    uint8_t data[8] = {0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    if (verboseDebug) {
        Serial.print("Sent Packet: ");
        for (int i = 0; i < 8; i++) {
            Serial.printf("%02X ", data[i]);
        }
        Serial.println();
    }
    uint8_t response[13];
    uint8_t responseLen = 0;
    uint8_t command = 0;
    if (!_comm.sendCommand(motorID, data, 8)) return false;
    if (!_comm.readFeedback(response, responseLen, command)) return false;
    return processFeedback(command, response);
}

bool MyActuatorRMDX6V3::ReadMultiTurnEncoderPosition(uint8_t motorID) {
    uint8_t data[8] = {0x61, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    if (verboseDebug) {
        Serial.print("Sent Packet: ");
        for (int i = 0; i < 8; i++) {
            Serial.printf("%02X ", data[i]);
        }
        Serial.println();
    }
    uint8_t response[13];
    uint8_t responseLen = 0;
    uint8_t command = 0;
    if (!_comm.sendCommand(motorID, data, 8)) return false;
    if (!_comm.readFeedback(response, responseLen, command)) return false;
    return processFeedback(command, response);
}

bool MyActuatorRMDX6V3::ReadMultiTurnEncoderZeroOffset(uint8_t motorID) {
    uint8_t data[8] = {0x62, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    if (verboseDebug) {
        Serial.print("Sent Packet: ");
        for (int i = 0; i < 8; i++) {
            Serial.printf("%02X ", data[i]);
        }
        Serial.println();
    }
    uint8_t response[13];
    uint8_t responseLen = 0;
    uint8_t command = 0;
    if (!_comm.sendCommand(motorID, data, 8)) return false;
    if (!_comm.readFeedback(response, responseLen, command)) return false;
    return processFeedback(command, response);
}

bool MyActuatorRMDX6V3::ReadMultiTurnAngle(uint8_t motorID) {
    uint8_t data[8] = {0x92, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    if (verboseDebug) {
        Serial.print("Sent Packet: ");
        for (int i = 0; i < 8; i++) {
            Serial.printf("%02X ", data[i]);
        }
        Serial.println();
    }
    uint8_t response[13];
    uint8_t responseLen = 0;
    uint8_t command = 0;
    if (!_comm.sendCommand(motorID, data, 8)) return false;
    if (!_comm.readFeedback(response, responseLen, command)) return false;
    return processFeedback(command, response);
}

bool MyActuatorRMDX6V3::ReadSystemStatus(uint8_t motorID) {
    uint8_t data[8] = {0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    if (verboseDebug) {
        Serial.print("Sent Packet: ");
        for (int i = 0; i < 8; i++) {
            Serial.printf("%02X ", data[i]);
        }
        Serial.println();
    }
    uint8_t response[13];
    uint8_t responseLen = 0;
    uint8_t command = 0;
    if (!_comm.sendCommand(motorID, data, 8)) return false;
    if (!_comm.readFeedback(response, responseLen, command)) return false;
    return processFeedback(command, response);
}

// PID Commands
bool MyActuatorRMDX6V3::ReadPositionPIDParameters(uint8_t motorID) {
    uint8_t data[8] = {0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t response[13];
    uint8_t responseLen = 0;
    uint8_t command = 0;
    if (!_comm.sendCommand(motorID, data, 8)) return false;
    if (!_comm.readFeedback(response, responseLen, command)) return false;
    return processFeedback(command, response);
}

bool MyActuatorRMDX6V3::ReadVelocityPIDParameters(uint8_t motorID) {
    uint8_t data[8] = {0x34, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t response[13];
    uint8_t responseLen = 0;
    uint8_t command = 0;
    if (!_comm.sendCommand(motorID, data, 8)) return false;
    if (!_comm.readFeedback(response, responseLen, command)) return false;
    return processFeedback(command, response);
}

bool MyActuatorRMDX6V3::ReadTorquePIDParameters(uint8_t motorID) {
    uint8_t data[8] = {0x35, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t response[13];
    uint8_t responseLen = 0;
    uint8_t command = 0;
    if (!_comm.sendCommand(motorID, data, 8)) return false;
    if (!_comm.readFeedback(response, responseLen, command)) return false;
    return processFeedback(command, response);
}

bool MyActuatorRMDX6V3::WritePositionPIDParametersToRAM(uint8_t motorID, uint8_t pP, uint8_t pI, uint8_t sP, uint8_t sI, uint8_t tP, uint8_t tI) {
    uint8_t data[8] = {0x36, pP, pI, sP, sI, tP, tI, 0x00};
    uint8_t response[13];
    uint8_t responseLen = 0;
    uint8_t command = 0;
    if (!_comm.sendCommand(motorID, data, 8)) return false;
    if (!_comm.readFeedback(response, responseLen, command)) return false;
    return processFeedback(command, response);
}

bool MyActuatorRMDX6V3::WriteVelocityPIDParametersToRAM(uint8_t motorID, uint8_t pP, uint8_t pI, uint8_t sP, uint8_t sI, uint8_t tP, uint8_t tI) {
    uint8_t data[8] = {0x37, pP, pI, sP, sI, tP, tI, 0x00};
    uint8_t response[13];
    uint8_t responseLen = 0;
    uint8_t command = 0;
    if (!_comm.sendCommand(motorID, data, 8)) return false;
    if (!_comm.readFeedback(response, responseLen, command)) return false;
    return processFeedback(command, response);
}

bool MyActuatorRMDX6V3::WriteTorquePIDParametersToRAM(uint8_t motorID, uint8_t pP, uint8_t pI, uint8_t sP, uint8_t sI, uint8_t tP, uint8_t tI) {
    uint8_t data[8] = {0x38, pP, pI, sP, sI, tP, tI, 0x00};
    uint8_t response[13];
    uint8_t responseLen = 0;
    uint8_t command = 0;
    if (!_comm.sendCommand(motorID, data, 8)) return false;
    if (!_comm.readFeedback(response, responseLen, command)) return false;
    return processFeedback(command, response);
}

bool MyActuatorRMDX6V3::WritePositionPIDParametersToROM(uint8_t motorID, uint8_t pP, uint8_t pI, uint8_t sP, uint8_t sI, uint8_t tP, uint8_t tI) {
    uint8_t data[8] = {0x30, pP, pI, sP, sI, tP, tI, 0x00};
    uint8_t response[13];
    uint8_t responseLen = 0;
    uint8_t command = 0;
    if (!_comm.sendCommand(motorID, data, 8)) return false;
    if (!_comm.readFeedback(response, responseLen, command)) return false;
    return processFeedback(command, response);
}

bool MyActuatorRMDX6V3::WriteVelocityPIDParametersToROM(uint8_t motorID, uint8_t pP, uint8_t pI, uint8_t sP, uint8_t sI, uint8_t tP, uint8_t tI) {
    uint8_t data[8] = {0x31, pP, pI, sP, sI, tP, tI, 0x00};
    uint8_t response[13];
    uint8_t responseLen = 0;
    uint8_t command = 0;
    if (!_comm.sendCommand(motorID, data, 8)) return false;
    if (!_comm.readFeedback(response, responseLen, command)) return false;
    return processFeedback(command, response);
}

bool MyActuatorRMDX6V3::WriteTorquePIDParametersToROM(uint8_t motorID, uint8_t pP, uint8_t pI, uint8_t sP, uint8_t sI, uint8_t tP, uint8_t tI) {
    uint8_t data[8] = {0x32, pP, pI, sP, sI, tP, tI, 0x00};
    uint8_t response[13];
    uint8_t responseLen = 0;
    uint8_t command = 0;
    if (!_comm.sendCommand(motorID, data, 8)) return false;
    if (!_comm.readFeedback(response, responseLen, command)) return false;
    return processFeedback(command, response);
}

MyActuatorRMDX6V3::Feedback MyActuatorRMDX6V3::getFeedback() const {
    return _lastFeedback;
}

MyActuatorRMDX6V3::Feedback MyActuatorRMDX6V3::getLastFeedback() const {
    return _lastFeedback;
}