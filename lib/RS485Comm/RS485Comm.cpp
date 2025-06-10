#include "RS485Comm.h"
#include <Arduino.h>

RS485Comm::RS485Comm(HardwareSerial& serial, uint8_t dirPin) : _serial(serial), _dirPin(dirPin) {}

void RS485Comm::begin(uint32_t baud) {
    _serial.begin(baud);
    pinMode(_dirPin, OUTPUT);
    digitalWrite(_dirPin, LOW); // Receive mode
}

uint16_t RS485Comm::computeCRC(uint8_t *buf, uint8_t len) {
    uint16_t crc = 0xFFFF;
    for (uint8_t pos = 0; pos < len; pos++) {
        crc ^= (uint16_t)buf[pos];
        for (uint8_t i = 8; i != 0; i--) {
            if ((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

bool RS485Comm::sendCommand(uint8_t motor_id, uint8_t *data, uint8_t data_length) {
    uint8_t packet[13];
    packet[0] = 0x3E; // Header
    packet[1] = motor_id;
    packet[2] = data_length;
    for (uint8_t i = 0; i < data_length; i++) {
        packet[3 + i] = data[i];
    }
    uint16_t crc = computeCRC(packet, 3 + data_length);
    packet[3 + data_length] = crc & 0xFF;
    packet[4 + data_length] = crc >> 8;

    _serial.flush();
    digitalWrite(_dirPin, HIGH); // Transmit mode
    _serial.write(packet, 5 + data_length);
    _serial.flush();
    digitalWrite(_dirPin, LOW); // Receive mode
    return true;
}

bool RS485Comm::readFeedback(uint8_t *response, uint8_t &response_length, uint8_t &command) {
    uint8_t buffer[64]; // Increased buffer to handle overruns
    uint8_t bytesRead = 0;
    unsigned long startTime = millis();

    // Flush buffer to clear unsolicited packets
    while (_serial.available()) {
        _serial.read();
    }

    // Read with timeout
    while (millis() - startTime < 100) {
        if (_serial.available()) {
            buffer[bytesRead] = _serial.read();
            bytesRead++;
            if (bytesRead >= 64) break; // Prevent overflow
            startTime = millis(); // Reset timeout on new byte
        }
    }

    // Parse packets
    uint8_t index = 0;
    while (index < bytesRead) {
        if (buffer[index] != 0x3E) {
            index++;
            continue;
        }
        if (index + 2 >= bytesRead) break; // Need header, motor_id, data_length
        uint8_t motor_id = buffer[index + 1];
        uint8_t data_length = buffer[index + 2];
        if (data_length > 8) {
            Serial.printf("Skipping packet at index %d: motor_id=0x%02X, data_length=%d\n", index, motor_id, data_length);
            index++;
            continue; // Invalid data length
        }
        if (index + 5 + data_length > bytesRead) break; // Incomplete packet
        uint16_t received_crc = buffer[index + 3 + data_length] | (buffer[index + 4 + data_length] << 8);
        uint16_t computed_crc = computeCRC(&buffer[index], 3 + data_length);
        if (received_crc != computed_crc) {
            Serial.printf("Error: CRC mismatch at index %d, computed=0x%04X, received=0x%04X\n", index, computed_crc, received_crc);
            index++;
            continue;
        }
        // Valid packet found
        memcpy(response, &buffer[index], 5 + data_length);
        response_length = 5 + data_length;
        command = buffer[index + 3]; // Command byte
        return true;
    }

    return false; // No valid packet
}