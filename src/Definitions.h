
#ifndef MYACTUATOR_RMDX6V3_H
#define MYACTUATOR_RMDX6V3_H

#include <Arduino.h>

#define RS485_CONTROL_PIN 41
#define RS485_SERIAL Serial4
#define FRAME_HEADER 0x3E
#define ALT_HEADER_1 0x5A
#define ALT_HEADER_2 0xA5
#define DATA_LENGTH 0x08
#define BROADCAST_ID 0xCD

class MyActuatorRMDX6V3 {
public:
  void begin(uint32_t baudRate = 115200) {
    pinMode(RS485_CONTROL_PIN, OUTPUT);
    digitalWrite(RS485_CONTROL_PIN, LOW);
    RS485_SERIAL.begin(baudRate);
    Serial.print("RS-485 initialized on Serial4 (Pins 16, 17) at ");
    Serial.print(baudRate);
    Serial.println(" bps, control pin: 41");
    Serial.print("Direction pin state: ");
    Serial.println(digitalRead(RS485_CONTROL_PIN) ? "HIGH (Transmit)" : "LOW (Receive)");
  }

  void sendCommand(uint8_t motorID, uint8_t command, uint8_t* data, bool useAltHeader = false) {
    uint8_t packet[11];
    if (useAltHeader) {
      packet[0] = ALT_HEADER_1; // 0x5A
      packet[1] = ALT_HEADER_2; // 0xA5
      packet[2] = motorID;
      packet[3] = command;
      for (int i = 0; i < 7; i++) {
        packet[4 + i] = data[i + 1];
      }
    } else {
      packet[0] = FRAME_HEADER; // 0x3E
      packet[1] = motorID;
      packet[2] = DATA_LENGTH; // 0x08
      packet[3] = command;
      for (int i = 0; i < 7; i++) {
        packet[4 + i] = data[i + 1];
      }
    }
    uint16_t crc = calculateCRC16(packet, 9);
    packet[9] = crc & 0xFF;
    packet[10] = (crc >> 8) & 0xFF;

    Serial.print("Sending command to Motor ID ");
    Serial.print(motorID, HEX);
    Serial.print(useAltHeader ? " (Alt Header): [" : ": [");
    for (int i = 0; i < 11; i++) {
      Serial.print(packet[i], HEX);
      if (i < 10) Serial.print(" ");
    }
    Serial.println("]");

    digitalWrite(RS485_CONTROL_PIN, HIGH);
    Serial.println("Direction pin set to HIGH (Transmit)");
    RS485_SERIAL.write(packet, 11);
    RS485_SERIAL.flush();
    delayMicroseconds(100);
    digitalWrite(RS485_CONTROL_PIN, LOW);
    Serial.println("Command sent, direction pin set to LOW (Receive)");
    delay(10);
  }

  void sendMultiMotorCommand(uint8_t command, uint8_t* data, bool useAltHeader = false) {
    Serial.println("Sending multi-motor command with ID 0xCD");
    sendCommand(BROADCAST_ID, command, data, useAltHeader);
  }

  bool receiveResponse(uint8_t* buffer, uint8_t maxLength, unsigned long timeout = 1000) {
    uint8_t index = 0;
    unsigned long startTime = millis();
    while (millis() - startTime < timeout && index < maxLength) {
      if (RS485_SERIAL.available()) {
        buffer[index++] = RS485_SERIAL.read();
      }
    }
    Serial.print("Received ");
    Serial.print(index);
    Serial.print(" bytes: [");
    for (int i = 0; i < index; i++) {
      Serial.print(buffer[i], HEX);
      if (i < index - 1) Serial.print(" ");
    }
    Serial.println("]");
    if (index < 11) {
      Serial.print("Response timeout or incomplete: received ");
      Serial.print(index);
      Serial.println(" bytes");
      return false;
    }
    // Relaxed validation: accept 0x3E or 0x5A 0xA5
    if (buffer[0] == FRAME_HEADER || (buffer[0] == ALT_HEADER_1 && buffer[1] == ALT_HEADER_2)) {
      uint16_t crc = calculateCRC16(buffer, 9);
      if (buffer[9] == (crc & 0xFF) && buffer[10] == ((crc >> 8) & 0xFF)) {
        Serial.println("Valid response received");
        return true;
      } else {
        Serial.println("CRC check failed");
        return false;
      }
    } else {
      Serial.println("Invalid response: incorrect header");
      return false;
    }
  }

  void sniffRawData(unsigned long duration = 5000) {
    Serial.println("Sniffing raw RS-485 data...");
    unsigned long startTime = millis();
    while (millis() - startTime < duration) {
      if (RS485_SERIAL.available()) {
        Serial.print(RS485_SERIAL.read(), HEX);
        Serial.print(" ");
      }
    }
    Serial.println("\nSniffing complete");
  }

  void setTorque(uint8_t motorID, int16_t torque, bool useAltHeader = false) {
    uint8_t data[8] = {0};
    data[0] = 0xA1;
    data[2] = torque & 0xFF;
    data[3] = (torque >> 8) & 0xFF;
    Serial.print("Setting torque for Motor ID ");
    Serial.print(motorID, HEX);
    Serial.print(" to ");
    Serial.print(torque);
    Serial.println(" mA");
    sendCommand(motorID, 0xA1, data, useAltHeader);
  }

  void setSpeed(uint8_t motorID, int32_t speed, bool useAltHeader = false) {
    uint8_t data[8] = {0};
    data[0] = 0xA2;
    data[2] = speed & 0xFF;
    data[3] = (speed >> 8) & 0xFF;
    data[4] = (speed >> 16) & 0xFF;
    data[5] = (speed >> 24) & 0xFF;
    Serial.print("Setting speed for Motor ID ");
    Serial.print(motorID, HEX);
    Serial.print(" to ");
    Serial.print(speed);
    Serial.println(" dps");
    sendCommand(motorID, 0xA2, data, useAltHeader);
  }

  void setAbsolutePosition(uint8_t motorID, int32_t position, bool useAltHeader = false) {
    uint8_t data[8] = {0};
    data[0] = 0xA4;
    data[2] = position & 0xFF;
    data[3] = (position >> 8) & 0xFF;
    data[4] = (position >> 16) & 0xFF;
    data[5] = (position >> 24) & 0xFF;
    Serial.print("Setting absolute position for Motor ID ");
    Serial.print(motorID, HEX);
    Serial.print(" to ");
    Serial.print(position);
    Serial.println(" pulses");
    sendCommand(motorID, 0xA4, data, useAltHeader);
  }

  void setSingleTurnPosition(uint8_t motorID, uint16_t angle, bool useAltHeader = false) {
    uint8_t data[8] = {0};
    data[0] = 0xA6;
    data[2] = angle & 0xFF;
    data[3] = (angle >> 8) & 0xFF;
    Serial.print("Setting single-turn position for Motor ID ");
    Serial.print(motorID, HEX);
    Serial.print(" to ");
    Serial.print(angle / 100.0);
    Serial.println(" degrees");
    sendCommand(motorID, 0xA6, data, useAltHeader);
  }

  bool readEncoderPosition(uint8_t motorID, int32_t& position, bool useAltHeader = false) {
    uint8_t data[8] = {0};
    data[0] = 0x60;
    Serial.print("Reading encoder position for Motor ID ");
    Serial.print(motorID, HEX);
    Serial.println("...");
    sendCommand(motorID, 0x60, data, useAltHeader);
    
    uint8_t buffer[11];
    if (receiveResponse(buffer, 11)) {
      if (buffer[3] == 0x60 || (useAltHeader && buffer[2] == 0x60)) {
        int dataOffset = useAltHeader ? 2 : 3;
        position = (int32_t)(buffer[dataOffset + 4] << 24) | (buffer[dataOffset + 3] << 16) | 
                  (buffer[dataOffset + 2] << 8) | buffer[dataOffset + 1];
        Serial.print("Encoder position: ");
        Serial.print(position);
        Serial.println(" pulses");
        return true;
      } else {
        Serial.println("Invalid command response (not 0x60)");
        return false;
      }
    }
    return false;
  }

  void setRS485ID(uint8_t currentID, uint8_t newID, bool useAltHeader = false) {
    uint8_t data[8] = {0};
    data[0] = 0x79;
    data[2] = 0; // Write flag
    data[7] = newID & 0xFF;
    Serial.print("Setting RS-485 ID from ");
    Serial.print(currentID, HEX);
    Serial.print(" to ");
    Serial.print(newID, HEX);
    Serial.println("...");
    sendCommand(currentID, 0x79, data, useAltHeader);
  }

  bool readRS485ID(uint8_t currentID, uint8_t& id, bool useAltHeader = false) {
    uint8_t data[8] = {0};
    data[0] = 0x79;
    data[2] = 1; // Read flag
    Serial.print("Reading RS-485 ID for Motor ID ");
    Serial.print(currentID, HEX);
    Serial.println("...");
    for (int retry = 0; retry < 3; retry++) {
      sendCommand(currentID, 0x79, data, useAltHeader);
      uint8_t buffer[11];
      if (receiveResponse(buffer, 11)) {
        int dataOffset = useAltHeader ? 2 : 3;
        if ((buffer[dataOffset] == 0x79 && buffer[dataOffset - 1] == 1) || 
            (useAltHeader && buffer[2] == 0x79)) {
          id = buffer[dataOffset + 4];
          Serial.print("RS-485 ID: ");
          Serial.println(id, HEX);
          return true;
        } else {
          Serial.println("Invalid ID read response");
        }
      }
      Serial.print("Retry ");
      Serial.print(retry + 1);
      Serial.println(" failed");
      delay(100);
    }
    Serial.println("Failed to read ID after 3 retries");
    return false;
  }

  void shutdownAllMotors(bool useAltHeader = false) {
    uint8_t data[8] = {0};
    data[0] = 0x80;
    Serial.println("Shutting down all motors...");
    sendMultiMotorCommand(0x80, data, useAltHeader);
  }

  void stopAllMotors(bool useAltHeader = false) {
    uint8_t data[8] = {0};
    data[0] = 0x81;
    Serial.println("Stopping all motors...");
    sendMultiMotorCommand(0x81, data, useAltHeader);
  }

  bool testCommunication(uint8_t motorID, bool useAltHeader = false) {
    Serial.print("Testing communication with Motor ID ");
    Serial.print(motorID, HEX);
    Serial.println(useAltHeader ? " (Alt Header)..." : "...");
    int32_t position;
    for (int retry = 0; retry < 3; retry++) {
      if (readEncoderPosition(motorID, position, useAltHeader)) {
        Serial.println("Communication successful!");
        return true;
      }
      Serial.print("Retry ");
      Serial.print(retry + 1);
      Serial.println(" failed");
      delay(100);
    }
    Serial.println("Communication test failed after 3 retries");
    return false;
  }

private:
  uint16_t calculateCRC16(uint8_t* data, uint8_t length) {
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < length; i++) {
      crc ^= data[i];
      for (uint8_t j = 0; j < 8; j++) {
        if (crc & 0x0001) {
          crc >>= 1;
          crc ^= 0xA001;
        } else {
          crc >>= 1;
        }
      }
    }
    Serial.print("Calculated CRC: ");
    Serial.println(crc, HEX);
    return crc;
  }
};

#endif
