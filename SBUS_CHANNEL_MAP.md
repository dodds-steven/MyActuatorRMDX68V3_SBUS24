# SBUS Channel Map - B2-EMO Droid System
**Date: January 7, 2026**

## System Overview
This document maps all 24 SBUS channels from the FrSky X20RS transmitter to various subsystems in the B2-EMO droid.

**THIS CODE PROJECT:** Controls ANKLE TEENSY (4x RMD-X6 V3 lifter motors)

---

## Complete Channel Assignment Table

| Channel | Assignment | System | Description |
|---------|------------|--------|-------------|
| **CH1** | TD R6 - Head Tilt 1 | Head Control | Front Left servo |
| **CH2** | TD R6 - Head Tilt 2 | Head Control | Front Right servo |
| **CH3** | TD R6 - Head Tilt 3 | Head Control | Back Center servo |
| **CH4** | TD R6 - Head Yaw | Head Control | Rotate control |
| **CH5** | TD R6 - Body Expansion Top | Body Control | Top block *(Also: DUAL_GIMBAL_X_CHANNEL - DISABLED)* |
| **CH6** | TD R6 - Body Expansion Bottom | Body Control | Bottom block *(Also: DUAL_GIMBAL_Y_CHANNEL - DISABLED)* |
| **CH7** | KYBER Pad | Audio/Effects | [LUA Script] |
| **CH8** | KYBER Page 2 | Audio/Effects | [Switch SH] |
| **CH9** | KYBER Volume | Audio/Effects | [Pot 1] |
| **CH10** | KYBER Random | Audio/Effects | [Switch - see radio] |
| **CH11** | CORE ESP32 - Spinners mode | Core Lighting | [Switch - see radio] |
| **CH12** | CORE ESP32 - DFPlayer | Audio | [TO DO] |
| **CH13** | **ANKLE TEENSY - MOBILE/STATIC_X_CHANNEL** | **THIS CODE** | **Roll control (left/right lean)** |
| **CH14** | **ANKLE TEENSY - MOBILE/STATIC_Y_CHANNEL** | **THIS CODE** | **Pitch control (forward/back tilt)** |
| **CH15** | **ANKLE TEENSY - CONTROL_MODE** | **THIS CODE** | **Mode: STATIC/MOBILE/SHUTDOWN** |
| **CH16** | **ANKLE TEENSY - FOOTLIFT_HEIGHT** | **THIS CODE** | **Height control** |
| **CH17** | TD R10 - Strafe PIN R | Drive Motors | Right strafe |
| **CH18** | TD R10 - Drive PIN R | Drive Motors | Right drive |
| **CH19** | TD R10 - Strafe PIN L | Drive Motors | Left strafe |
| **CH20** | TD R10 - Drive PIN L | Drive Motors | Left drive |
| **CH21** | TD R10 - Drive ARM | Drive Motors | Motor enable/arm |
| **CH22** | TD R10 - ALL 4 SLIDERS | Drive Motors | Foot in/out movement |
| **CH23** | AVAILABLE | - | Unassigned |
| **CH24** | AVAILABLE | - | Unassigned |

---

## ANKLE TEENSY Channels (This Code)

### Channels Used by MotorModeController.cpp

#### Mode Control
- **CH15 (CONTROL_MODE)** - 3-position mode switch
  - `< 500`: MOBILE mode
  - `500-1300`: STATIC mode
  - `> 1300`: SHUTDOWN mode

#### STATIC Mode Inputs
- **CH13 (STATIC_X_CHANNEL)** - Roll control (90% scaled)
- **CH14 (STATIC_Y_CHANNEL)** - Pitch control (90% scaled)
- **CH16 (FOOTLIFT_HEIGHT_CHANNEL)** - Height offset

#### MOBILE Mode Inputs
- **CH13 (MOBILE_X_CHANNEL)** - Roll control
- **CH14 (MOBILE_Y_CHANNEL)** - Pitch control
- **CH16 (FOOTLIFT_HEIGHT_CHANNEL)** - Height offset

**Legacy Dual-Gimbal Feature (DISABLED):**
- **CH5 (DUAL_GIMBAL_X_CHANNEL)** - Left gimbal roll (conflicts with TD R6 Body Expansion)
- **CH6 (DUAL_GIMBAL_Y_CHANNEL)** - Left gimbal pitch (conflicts with TD R6 Body Expansion)
- Feature disabled via `ENABLE_DUAL_GIMBAL_AVERAGING 0` in MotorModeController.cpp
- Original intent: Average two gimbals for combined body position + compensation control
- Current issue: Channels conflict with TD R6 assignment; averaging with centered servos cuts control range to 50%

---

## Subsystem Breakdown

### TD R6 Head Controller (CH1-CH6)
- 3 servos for head tilt
- 1 servo for head yaw/rotation
- 2 channels for body expansion mechanism

### KYBER Audio/Effects (CH7-CH10)
- Pad control via LUA script
- Page selection
- Volume control
- Random sound trigger

### CORE ESP32 (CH11-CH12)
- Spinner light modes
- DFPlayer audio control (TO DO)

### ANKLE TEENSY - This Code (CH13-CH16)
**Controls 4x MyActuator RMD-X6 V3 lifter motors**
- Roll/pitch body position
- Mode selection
- Height adjustment

### TD R10 Drive Controller (CH17-CH22)
**Controls 4x mecanum wheel drive motors**
- Independent left/right drive
- Independent left/right strafe
- Drive arm/enable
- Foot position sliders

### Available (CH23-CH24)
- Free for future expansion

---

## Code Constants (Definitions.h)

```cpp
#define CONTROL_MODE 15            // CH15: Mode selection
#define STATIC_X_CHANNEL 13        // CH13: Roll in STATIC mode
#define STATIC_Y_CHANNEL 14        // CH14: Pitch in STATIC mode
#define MOBILE_X_CHANNEL 13        // CH13: Roll in MOBILE mode
#define MOBILE_Y_CHANNEL 14        // CH14: Pitch in MOBILE mode
#define FOOTLIFT_HEIGHT_CHANNEL 16 // CH16: Height control

// if dual gimbal averaging is enabled, these channels are used
#define DUAL_GIMBAL_X_CHANNEL 5    // CH5 (index 4): LEGACY - Left gimbal roll (DISABLED, conflicts with TD R6 Body Expansion. Tim used both sticks in drive for leaning?)
#define DUAL_GIMBAL_Y_CHANNEL 6    // CH6 (index 5): LEGACY - Left gimbal pitch (DISABLED, conflicts with TD R6 Body Expansion Tim used both sticks in drive for leaning?)

// the following channel is defined but not used in this code, unsure what the purpose was when Tim added the #define
//#define STRAFE_X_CHANNEL 17        // CH17: (Documented but not used in this code)
```

---

## Channel Conflicts / Notes

⚠️ **MOBILE Mode CH5/CH6 Usage:**
- MOBILE mode currently reads CH5/CH6 (Body Expansion channels)
- These are averaged with CH13/CH14
- This may be unintended legacy code
- CH5/CH6 are assigned to TD R6 Body Expansion in the system map
- Consider removing CH5/CH6 reading from MOBILE mode if not intentional

---

## SBUS Technical Specs

- **Total Channels:** 24 (16 standard + 8 extended)
- **Value Range:** 172-1811
- **Center Value:** 992
- **Protocol:** 100,000 baud, 8E2, inverted
- **Update Rate:** ~111Hz (~9ms per frame)
- **Port:** Serial1 (RX=0, TX=1) on Teensy 4.1

---

## Future Expansion

**Available Channels:**
- CH23
- CH24

**Potential Uses:**
- Additional sensor inputs
- Emergency stop
- Mode indicators
- Telemetry feedback

---

## Validation Checklist

When modifying channel assignments:
- [ ] Update Definitions.h constants
- [ ] Update this SBUS_CHANNEL_MAP.md
- [ ] Update radio mixer configuration
- [ ] Test channel in STATIC mode
- [ ] Test channel in MOBILE mode
- [ ] Verify no conflicts with other subsystems
- [ ] Document changes in git commit

---

## Related Files

- `src/Definitions.h` - Channel constant definitions
- `src/MotorModeController.cpp` - Channel reading and processing
- `src/HandleSBUS.cpp` - SBUS protocol implementation
- `README.md` - Project overview and setup

---

**Last Updated:** January 7, 2026  
**Code Version:** Current working version  
**Hardware:** Teensy 4.1 + FrSky TD-R10 + FrSky X20RS
