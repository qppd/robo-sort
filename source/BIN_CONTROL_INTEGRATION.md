# BIN Control Integration - TB6600 Stepper Motor

## Overview
Added complete Firebase-based remote control for the TB6600 stepper motor bin positioning system with 5 commands: BIN_HOME, BIN_1, BIN_2, BIN_3, BIN_4.

## Changes Made

### 1. Android App (`source/android/RoboSortControl/`)

#### Layout Changes (`activity_main.xml`)
- Added **BIN Control Section** between Lifter and Servo controls
- 5 buttons arranged in a clean layout:
  - 🏠 BIN HOME (orange button for homing operation)
  - BIN 1, BIN 2 (row 1)
  - BIN 3, BIN 4 (row 2)

#### Code Changes (`MainActivity.java`)
- **New UI Elements**: `btnBinHome`, `btnBin1`, `btnBin2`, `btnBin3`, `btnBin4`
- **New Method**: `setupBinControls()` - wires button click handlers
- **New Method**: `sendBinCommand(String binCommand)` - sends to Firebase at `robosort/commands/bin`
- Firebase structure: `{type: "bin", command: "BIN_HOME", timestamp: ...}`

### 2. Raspberry Pi (`source/rpi/yolo/rc_controler.py`)

#### Firebase Listener Extension
- Added handler in `on_command_change()` for `type == "bin"`
- Validates commands: `BIN_HOME`, `BIN_1`, `BIN_2`, `BIN_3`, `BIN_4`
- Forwards directly to Arduino via `send_text_command()`
- Logs confirmation: `✓ Sent BIN command: BIN_X`

### 3. Arduino (`source/arduino/RoboSort/RoboSort.ino`)

**Already implemented!** The Arduino sketch has full TB6600 stepper control:

#### Command Behaviors
- **BIN_HOME**: Rotates CCW until limit switch triggers (auto-calibration)
- **BIN_1**: Position 0 steps (HOME position)
- **BIN_2**: Position 1100 steps from HOME
- **BIN_3**: Position 2150 steps from HOME  
- **BIN_4**: Position 3100 steps from HOME

#### Safety Features
- Tracks current position to calculate relative moves (CW/CCW)
- Uses limit switch (pin 53) for homing safety stop
- Non-blocking operation via TB6600 class
- Buzzer feedback on successful command

## Data Flow

```
Android Button Click
    ↓
Firebase RTDB: robosort/commands/bin
    {type: "bin", command: "BIN_HOME", timestamp: ...}
    ↓
RPi rc_controler.py: on_command_change()
    ↓
Arduino Serial: "BIN_HOME\n"
    ↓
TB6600 Stepper Execution
```

## Firebase Schema Extension

```
robosort/
  commands/
    motor/           # Existing: DC motor commands
    servo1-5/        # Existing: servo commands  
    lifter/          # Existing: lifter commands
    bin/             # NEW: bin stepper commands
      type: "bin"
      command: "BIN_HOME" | "BIN_1" | "BIN_2" | "BIN_3" | "BIN_4"
      timestamp: <ms>
```

## Testing Checklist

- [x] Android builds without errors
- [x] Python syntax valid (import warnings expected in dev environment)
- [x] Arduino already supports all commands (verified in RoboSort.ino)
- [x] Firebase path consistent with existing patterns
- [x] No breaking changes to motor/servo/lifter controls

## Usage

1. **Start RPi controller**: `python3 rc_controler.py --arduino-port /dev/ttyUSB0 ...`
2. **Open Android app**: Connect to Firebase
3. **Home the stepper**: Press "🏠 BIN HOME" (required before absolute positioning)
4. **Move to bins**: Press BIN 1/2/3/4 buttons
5. **Monitor Arduino serial** for real-time step counts and limit switch feedback

## Notes

- **First-time setup**: Always run BIN_HOME after power-on to calibrate position
- **Position tracking**: Arduino maintains `currentBinPosition` in memory
- **Limit switch**: Digital pin 53 (INPUT_PULLUP), LOW = pressed
- **Step timing**: 750µs pulse, 1500µs gap (safe for TB6600)
- **Non-blocking**: TB6600 class uses update() loop for smooth operation
