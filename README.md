# MyActuatorRMDX68V3_SBUS24
Arduino-based motor control system for a droid robot using SBUS and MyActuator RMD X6-8 motors built with PlatformIO and Teensy 4.1

# Droid Motor Control

This repository contains an Arduino-based motor control system for a droid robot, designed to control four RX motors using SBUS input from a FrSky X20S transmitter. The system supports precise positioning of motors for a quadruped droid (B2-EMO), with safety checks to prevent out-of-range movements.

## Features
- **SBUS Control**: Reads gimbal inputs (Channels 10 and 11) for X/Y movement.
- **Motor Safety**: Validates motor positions before movement to prevent end stop crashes.
- **RS-485 Communication**: Controls MyActuator RMD-X6 V3 motors via Serial4.
- **XY Mixing**: Maps gimbal inputs to motor positions for coordinated droid tilt.
- **Configurable Limits**: Motor position ranges and centers defined in `main.cpp`.

## Hardware Requirements
- **Microcontroller**: Teensy 4.1 (or compatible Arduino board).
- **Motors**: MyActuator RMD-X6 V3 (4x, IDs 0x01-0x04).
- **Transmitter**: FrSky X20S configuered with 24 Channel SBUS output.
- **Reciever: FrSky TDR-10 with SBUS 24 output
- **Wiring**:
  - SBUS: Serial1 (RX pin 0, TX pin 1).
  - RS-485: Serial4, DE/RE pin 41.
- **Power Supply**: Suitable for motors (e.g., 24V DC).

## Software Requirements
- I developed this using Visual Studio Code and the platformIO extension with the Teensy 4.1 as the target.
- I have a custom board for my Teensy that provides for the SBUS inverted singal.  You may need to flip this option in your setup if you don't have the inverter on your hardware
- **Arduino IDE**: Version 2.0 or later.  Not sure about this as I did not use Arduino IDE.  But the teensy plug in for it seemed to be used for code upload
- **Libraries**:
  - SBUS a 24 Chanel SBUS capable library
  - Custom `MyActuatorRMDX6V3.h` and `RS485Comm.h` 
- **Teensyduino**: Add-on for Teensy boards in Arduino IDE.

## Installation
Configure motor limits in main.cpp (e.g., MOTOR1_MIN_POS, MOTOR2_MAX_POS, etc).
1. Clone the repository:
   ```bash
   git clone https://github.com/yourusername/droid-control.git
