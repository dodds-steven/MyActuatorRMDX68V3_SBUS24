#include "RS485Comm.h"
#include <Arduino.h>

#define verboseDebug false // Set to false to disable debug output for faster runtime

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
    uint8_t buffer[64];
    uint8_t bytesRead = 0;
    unsigned long startTime = millis();

#if verboseDebug
    Serial.println("RS485: Clearing Serial4 buffer");
    while (_serial.available()) {
        uint8_t discarded = _serial.read();
        Serial.printf("RS485: Discarded byte: 0x%02X\n", discarded);
    }

    Serial.println("RS485: Reading response");
#endif

    while (millis() - startTime < 50) {
        if (_serial.available()) {
            buffer[bytesRead] = _serial.read();
#if verboseDebug
            Serial.printf("RS485: Byte %d: 0x%02X\n", bytesRead, buffer[bytesRead]);
#endif
            bytesRead++;
            if (bytesRead >= 64) {
#if verboseDebug
                Serial.println("RS485: Buffer overflow, stopping read");
#endif
                break;
            }
            startTime = millis();
        }
    }

#if verboseDebug
    Serial.printf("RS485: Read %d bytes\n", bytesRead);
    if (bytesRead == 0) {
        Serial.println("RS485: No response received");
        return false;
    }
#endif

    uint8_t index = 0;
    while (index < bytesRead) {
        if (buffer[index] != 0x3E) {
#if verboseDebug
            Serial.printf("RS485: No 0x3E header at index %d, found 0x%02X\n", index, buffer[index]);
#endif
            index++;
            continue;
        }

        if (index + 2 >= bytesRead) {
#if verboseDebug
            Serial.println("RS485: Incomplete header (need motor_id and data_length)");
#endif
            break;
        }

        
        uint8_t data_length = buffer[index + 2];
#if verboseDebug
        uint8_t motor_id = buffer[index + 1];
        Serial.printf("RS485: Found header at index %d, motor_id=0x%02X, data_length=%d\n", index, motor_id, data_length);
#endif

        if (data_length > 8) {
#if verboseDebug
            Serial.printf("RS485: Invalid data_length=%d at index %d\n", data_length, index);
#endif
            index++;
            continue;
        }

        if (index + 5 + data_length > bytesRead) {
#if verboseDebug
            Serial.printf("RS485: Incomplete packet at index %d, need %d bytes, have %d\n", 
                          index, 5 + data_length, bytesRead - index);
#endif
            break;
        }

        uint16_t received_crc = buffer[index + 3 + data_length] | (buffer[index + 4 + data_length] << 8);
        uint16_t computed_crc = computeCRC(&buffer[index], 3 + data_length);
        if (received_crc != computed_crc) {
#if verboseDebug
            Serial.printf("RS485: CRC mismatch at index %d, computed=0x%04X, received=0x%04X\n", 
                          index, computed_crc, received_crc);
#endif
            index++;
            continue;
        }

#if verboseDebug
        Serial.println("RS485: Valid packet received");
        Serial.print("RS485: Packet: ");
        for (uint8_t i = 0; i < 5 + data_length; i++) {
            Serial.printf("0x%02X ", buffer[index + i]);
        }
        Serial.println();
#endif

        memcpy(response, &buffer[index], 5 + data_length);
        response_length = 5 + data_length;
        command = buffer[index + 3];
        return true;
    }

#if verboseDebug
    Serial.println("RS485: No valid packet found");
#endif
    return false;
}