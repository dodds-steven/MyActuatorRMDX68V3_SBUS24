# MyActuatorRMDX68V3_SBUS24

Use the Operating Modes Branch for all of the latest functionality.  Also, make sure you rename main.cpp to main.cppBAK and then ResetAllMotorToZero.cppBak to 
ResetAllMotorToZero.cpp.  Then have all of the motors at the lowest travel.  Compile and upload.  This will set the encoders to zero.  If you don't do this step the code will not work.  Once you have run the Zero procedure, rename the files back to their original names.  Then you can re-upload the code.  I have Motor1 in the Front left position, Motor2 the FrontRight, Motor3 the BackRight, and Motor4 the BackLeft.

### Updated README
<xaiArtifact artifact_id="e90104cd-d624-42c2-8dbc-23434fa12763" artifact_version_id="253de576-515e-472c-aee2-aa8e324c4ab3" title="README.md" contentType="text/markdown">
# MyActuatorRMDX68V3_SBUS24

Arduino-based motor control system for the B2-EMO Pro Drive, a custom droid robot using SBUS and MyActuator RMD-X6 V3 motors, built with PlatformIO and Teensy 4.1.

## Droid Motor Control

This repository contains an Arduino-based motor control system for the B2-EMO Pro Drive, a custom-designed aluminum chassis compatible with Mr. Baddeley’s V2 B2-EMO Droid. The system controls four MyActuator RMD-X6 V3 lifter motors via RS485 communication, paired with UUMotor 4-inch brushless hub drive motors mounted on AndyMark 6-inch mecanum wheels. It uses SBUS input from a multi-channel transmitter to achieve precise positioning and movement for a quadruped droid, with safety checks to prevent out-of-range movements.

## Setup Instructions for B2-EMO Pro Drive System

The B2-EMO Pro Drive system uses a Teensy 4.1 microcontroller to control four MyActuator RMDX6V3 lifter motors via RS485 communication, paired with UUMotor 4-inch brushless hub drive motors mounted on AndyMark 6-inch mecanum wheels, all integrated into a custom-designed aluminum chassis. This system is compatible with Mr. Baddeley’s V2 B2-EMO Droid. The following instructions detail the SBUS channel mapping and gimbal setup, referencing the `Definitions.h` configuration for precise control of the droid’s movements.

### Hardware Requirements

- **Microcontroller**: Teensy 4.1
- **Lifter Motors**: 4x MyActuator RMDX6V3 with RS485 communication (`RS485_PORT=Serial4`, `RS485_DIR_PIN=41`)
- **Drive Motors**: UUMotor 4-inch brushless hub motors on AndyMark 6-inch mecanum wheels
- **SBUS Receiver**: Connected to `SBUS_PORT=Serial1`
- **Transmitter**: Multi-channel SBUS-compatible transmitter (e.g., FrSky Taranis)
- **Chassis**: Custom aluminum chassis designed for B2-EMO Pro Drive system

### Software Setup

1. **Clone the Repository**:
   ```bash
   git clone https://github.com/Eebel/MyActuatorRMDX68V3_SBUS24.git
   cd MyActuatorRMDX68V3_SBUS24
   ```

2. **Install PlatformIO**:
   - Ensure PlatformIO is installed and configured for Teensy 4.1 in Visual Studio Code.
   - Note: A custom board may be required for SBUS signal inversion. If your hardware lacks an inverter, enable the SBUS inversion option in your receiver or transmitter settings.

3. **Compile and Upload**:
   ```bash
   pio run --target clean
   pio run --target upload
   ```

4. **Monitor Serial Output**:
   - Use a serial monitor at 115200 baud to verify operation:
     ```bash
     pio device monitor -b 115200
     ```

### SBUS Channel Mapping

The `Definitions.h` file (`src/Definitions.h`) configures the SBUS channels for controlling the droid’s modes and movements. Below is the channel mapping based on the provided configuration:

```cpp
// SBUS channel definitions
#define CONTROL_MODE 3            // CH3 (index 2): Selects operating mode (STATIC, MOBILE, SHUTDOWN)
#define STATIC_X_CHANNEL 11       // CH11/X (index 10): Controls roll (Motor1/Motor3) in STATIC mode
#define STATIC_Y_CHANNEL 10       // CH10/Y (index 9): Controls pitch (Motor2/Motor4) in STATIC mode
#define MOBILE_X_CHANNEL 11       // CH11/X (index 10): Controls roll (Motor1/Motor3) in MOBILE mode
#define MOBILE_Y_CHANNEL 10       // CH10/Y (index 9): Controls pitch (Motor2/Motor4) in MOBILE mode
#define STRAFE_X_CHANNEL 1       // CH1/X (index 0): Controls lateral movement (strafe) in MOBILE mode
#define FOOTLIFT_HEIGHT_CHANNEL 18 // CH18 (index 17): Controls footlift height in MOBILE mode
```

#### Channel Descriptions

- **CH3 (CONTROL_MODE, index 2)**: Selects the operating mode:
  - `< 500`: `STATIC` mode (stationary, gimbal-controlled pitch/roll).
  - `500–1300`: `MOBILE` mode (movement with strafe and gimbal control).
  - `> 1300`: `SHUTDOWN` mode (motors to RobotLow: Motor1/Motor3 to -5.00°, Motor2/Motor4 to 5.00°, then powered off).
- **STATIC Mode (CH10/Y, CH11/X)**:
  - **CH11/X (STATIC_X_CHANNEL, index 10)**: Controls roll (left/right tilt):
    - Left (`~172`): Motor1/Motor4 to RobotHigh (~-109.50°/109.50°), Motor2/Motor3 to RobotLow (~10.50°/-10.50°).
    - Right (`~1811`): Motor1/Motor4 to RobotLow (~-10.50°/10.50°), Motor2/Motor3 to RobotHigh (~109.50°/-109.50°).
  - **CH10/Y (STATIC_Y_CHANNEL, index 9)**: Controls pitch (forward/aft tilt):
    - Forward (`~172`): Motor1/Motor3 to RobotLow (~-10.50°), Motor2/Motor4 to RobotLow (~10.50°).
    - Aft (`~1811`): Motor1/Motor3 to RobotHigh (~-109.50°), Motor2/Motor4 to RobotHigh (~109.50°).
  - Scaling: 0.9 (90% of full range).
- **MOBILE Mode (CH10/Y, CH11/X, CH1/X, CH5/Y, CH6/X)**:
  - **CH11/X (MOBILE_X_CHANNEL, index 10)**: Controls roll (left/right tilt, same as STATIC).
  - **CH10/Y (MOBILE_Y_CHANNEL, index 9)**: Controls pitch (forward/aft tilt, same as STATIC).
  - **CH1/X (STRAFE_X_CHANNEL, index 0)**: Controls lateral movement (strafe, roll):
    - Left (`~172`): Motor1/Motor4 to RobotHigh (~-92.50°/87.50°), Motor2/Motor3 to RobotLow (~27.50°/-27.50°).
    - Right (`~1811`): Motor1/Motor4 to RobotLow (~-27.50°/27.50°), Motor2/Motor3 to RobotHigh (~87.50°/-92.50°).
  - **CH5/Y (index 4)**: Additional roll control.
  - **CH6/X (index 5)**: Additional pitch control.
  - Scaling: 0.5 (50% of full range, blended with CH5/CH6).
- **CH18 (FOOTLIFT_HEIGHT_CHANNEL, index 17)**: Controls height in MOBILE mode:
  - Low (`~172`): All motors to RobotHigh (Motor1/Motor3: -115.00°, Motor2/Motor4: 115.00°).
  - Mid (`~992`): All motors to center (Motor1/Motor3: -60.00°, Motor2/Motor4: 60.00°).
  - High (`~1811`): All motors to RobotLow (Motor1/Motor3: -5.00°, Motor2/Motor4: 5.00°).

### Gimbal Setup

- **Transmitter Configuration**:
  - Assign CH3 to a three-position switch for mode selection (STATIC, MOBILE, SHUTDOWN).
  - Map CH11/X and CH10/Y to the right gimbal for roll and pitch control in both STATIC and MOBILE modes.
  - Map CH1/X to the right gimbal’s X-axis for strafe (roll) in MOBILE mode.
  - Map CH5/Y and CH6/X to the left gimbal for additional roll and pitch in MOBILE mode.
  - Assign CH18 to a spring-loaded slider for footlift height, centered at ~992.
- **Calibration**:
  - Ensure SBUS receiver outputs values between 172 and 1811.
  - Calibrate transmitter to center gimbals at ~992 for neutral position.
  - Verify motor positions align with `RobotLow` and `RobotHigh` limits as defined in `Definitions.h`.

### Testing

1. **Power On**:
   - Start in `STATIC` mode (`CH3 < 500`) and droid will rise to mid level.
2. **STATIC Mode**:
   - Set `CH3 < 500`, test CH11/X and CH10/Y for roll and pitch.
3. **MOBILE Mode**:
   - Set `CH3 ≈ 992`, test CH1/X (strafe), CH10/Y, CH11/X, for tilt and roll and CH18 slider for height.
4. **Serial Monitoring**:
   - Check logs for SBUS reads, mode switches, and motor positions.
   - Ensure `Update time` < 100ms and no “setPosition failed” errors.

Adjust transmitter channel outputs as needed to match desired physical motion directions.

## Acknowledgments
- Built with the Arduino community and Teensy 4.1.
- Inspired by droid builders and Mr. Baddeley’s V2 B2-EMO Droid design.
- Special thanks to xAI’s Grok for code assistance and optimization.

This project is ongoing. Follow progress and provide feedback at [https://www.facebook.com/share/g/15HHf4JXkN/](https://www.facebook.com/share/g/15HHf4JXkN/).
