#ifndef RS485_COMM_H
#define RS485_COMM_H

#include <Arduino.h>

// RS485 Communication speed configuration
#define RS485_COMM_DELAY 10 // milliseconds to wait after sending command before reading response (original value was 50ms)

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