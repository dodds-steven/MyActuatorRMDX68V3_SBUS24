#ifndef RS485_COMM_H
#define RS485_COMM_H

#include <Arduino.h>

class RS485Comm {
public:
    RS485Comm(HardwareSerial& serial, uint8_t dirPin);
    void begin(uint32_t baud);
    bool sendCommand(uint8_t motor_id, uint8_t *data, uint8_t data_length);
    bool readFeedback(uint8_t *response, uint8_t &response_length, uint8_t &command);

private:
    HardwareSerial& _serial;
    uint8_t _dirPin;
    uint16_t computeCRC(uint8_t *buf, uint8_t len);
};

#endif