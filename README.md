# RoboSort: Automated Sorting Robot

## Overview
RoboSort is an automated sorting robot designed to efficiently separate and organize objects using a robotic arm and conveyor system. The project integrates both Arduino and Raspberry Pi platforms, combining mechanical, electrical, and software components for a robust, scalable solution suitable for educational, prototyping, and light industrial applications.

## Architecture
The system follows a modular, two-tier architecture:
- **Low-level control**: Arduino microcontroller manages real-time servo operations and hardware interfacing
- **High-level logic**: Raspberry Pi handles advanced processing, decision-making, vision processing, and network communication
- **Communication**: Serial protocol enables bidirectional data exchange between Arduino and Raspberry Pi

## Features
- Five degrees of freedom (5-DOF) robotic arm driven by a 16-channel PWM servo driver
- Dual DC motor control system using E-Gizmo HPMD-3.1 motor driver for conveyor and movement operations
- Modular design with clear separation between control (Arduino) and high-level logic (Raspberry Pi)
- Serial command interface for manual testing and calibration
- Expandable architecture for additional sensors or actuators
- Comprehensive documentation and model images for easy assembly and understanding

## Hardware Requirements

### Main Body Dimensions
- Height: 4 inches
- Length: 35 inches
- Width: 20 inches

### Trash Bin Dimensions
- Height: 20 inches
- Radius: 10 inches (Diameter: 20 inches)

### Components
- **Robotic Arm**: 5-DOF articulated arm
- **Servos**: 5 standard servos (SG90/MG996R or similar)
- **Servo Driver**: Adafruit 16-channel PWM Servo Driver (PCA9685)
- **DC Motor Driver**: E-Gizmo HPMD-3.1 Dual H-Bridge Motor Driver
- **DC Motors**: 2 DC motors for conveyor belt and/or robot movement
- **Microcontroller**: Arduino-compatible board (Uno, Mega, etc.)
- **Single-board Computer**: Raspberry Pi (any model with GPIO)
- **Power Supply**: Adequate power for servos, DC motors, and logic circuits (recommend separate power for motors)

## Software Components

### Arduino Firmware
- **RoboSort.ino**: Main firmware integrating servo and motor control with serial command interface
- **SERVO_CONFIG.h**: Header file with servo configuration and function declarations
- **SERVO_CONFIG.cpp**: Implementation of servo control logic and movement functions
- **DC_CONFIG.h**: Header file for DC motor driver configuration and control declarations
- **DC_CONFIG.cpp**: Implementation of DC motor control logic with E-Gizmo HPMD-3.1 driver

### Raspberry Pi Software
- **RoboSort.py**: High-level control logic, sensor integration, and advanced features

### Serial Commands

#### Servo Commands
- `TEST`: Runs a test sequence on all servos
- `S<servo> <angle>`: Sets a specific servo (0-4) to a given angle (0-180)
  - Example: `S2 90` - Set servo 2 to 90 degrees

#### Motor Commands
- `MTEST`: Runs a comprehensive test sequence on both DC motors
- `M<motor> <direction> <speed>`: Controls individual motor with specified parameters
  - Motors: `A` or `B`
  - Directions: `F` (forward), `B` (backward), `S` (stop), `BR` (brake)
  - Speed: 0-255
  - Example: `MA F 200` - Motor A forward at speed 200
  - Example: `MB B 150` - Motor B backward at speed 150
- `MSTOP`: Immediately stops all DC motors

## Installation

### Arduino Setup
1. Install the Arduino IDE from [arduino.cc](https://www.arduino.cc)
2. Install the Adafruit PWM Servo Driver library:
   - Open Arduino IDE
   - Go to Tools > Manage Libraries
   - Search for "Adafruit PWM Servo Driver"
   - Install the library
3. Open `source/arduino/RoboSort/RoboSort.ino`
4. Select your board and port from the Tools menu
5. Upload the firmware to your Arduino

### Raspberry Pi Setup
1. Ensure Python 3.x is installed (pre-installed on Raspberry Pi OS)
2. Navigate to `source/rpi/RoboSort/`
3. Install required Python packages (if any dependencies are added in the future)
4. Run the Python script: `python3 RoboSort.py`

## Usage

### Basic Operation
1. Power on the Arduino and Raspberry Pi
2. Ensure proper power supply to motors (separate power recommended for high-current DC motors)
3. Connect to the Arduino via serial monitor (9600 baud)
4. Use the serial commands to control and test the servos and motors

### Testing Servos
```
TEST
```
This command runs a test sequence on all servos, moving each through its full range of motion.

### Manual Servo Control
```
S0 90
S1 45
S2 180
```
These commands set servo 0 to 90 degrees, servo 1 to 45 degrees, and servo 2 to 180 degrees respectively.

### Testing DC Motors
```
MTEST
```
This command runs a comprehensive test sequence on both DC motors, testing forward, backward, and stop operations at various speeds.

### Manual DC Motor Control
```
MA F 200
MB B 150
MA S 0
MSTOP
```
These commands demonstrate:
- Motor A forward at speed 200
- Motor B backward at speed 150
- Motor A stop
- Stop all motors immediately

## Hardware Models

### Front View
![RoboSort Front View](model/robosort-front.jpg)

The front view shows the main interface and access points of the RoboSort system, including the entry slot for sorting and the control panel.

### Side View
![RoboSort Side View](model/robosort-side.jpg)

The side view highlights the internal mechanism layout, conveyor system, and the relative position of the trash bin to the sorting area.

### Main/Top View
![RoboSort Main View](model/robosort-main.jpg)

The main/top view provides a comprehensive look at the overall structure, showing the arrangement of components and the spatial relationship between the body and the trash bin.

## Project Structure
```
robo-sort/
├── LICENSE
├── README.md
├── diagram/              # Circuit diagrams and schematics
├── model/                # 3D models and images
│   ├── robosort-front.jpg
│   ├── robosort-main.jpg
│   └── robosort-side.jpg
└── source/
    ├── arduino/
    │   └── RoboSort/
    │       ├── RoboSort.ino      # Main firmware with integrated control
    │       ├── SERVO_CONFIG.h     # Servo driver header
    │       ├── SERVO_CONFIG.cpp   # Servo driver implementation
    │       ├── DC_CONFIG.h        # DC motor driver header
    │       └── DC_CONFIG.cpp      # DC motor driver implementation
    └── rpi/
        └── RoboSort/
            └── RoboSort.py        # High-level control logic
```

## Configuration

### Servo Calibration
Adjust the servo pulse width limits in `SERVO_CONFIG.cpp`:
```cpp
#define SERVO_MIN_PULSE  150
#define SERVO_MAX_PULSE  600
```

### Servo Channel Mapping
Modify the servo channel assignments in the `ServoConfig` constructor to match your wiring configuration.

### DC Motor Pin Configuration
Configure motor driver pin connections in `DC_CONFIG.h` according to your wiring:
```cpp
#define MOTOR_A_PWM 9    // PWM pin for Motor A speed control
#define MOTOR_A_DIR1 7   // Direction pin 1 for Motor A
#define MOTOR_A_DIR2 8   // Direction pin 2 for Motor A

#define MOTOR_B_PWM 10   // PWM pin for Motor B speed control
#define MOTOR_B_DIR1 11  // Direction pin 1 for Motor B
#define MOTOR_B_DIR2 12  // Direction pin 2 for Motor B
```

### E-Gizmo HPMD-3.1 Wiring Guide
The E-Gizmo HPMD-3.1 is a dual H-bridge motor driver capable of controlling two DC motors independently:

**Power Connections:**
- Connect motor power supply (6-12V recommended) to the driver's VIN and GND
- Ensure adequate current rating for your motors (up to 3A per channel)

**Motor Connections:**
- Connect Motor A to MA+ and MA- terminals
- Connect Motor B to MB+ and MB- terminals

**Control Connections:**
- Connect Arduino PWM pins to PWM inputs (for speed control)
- Connect Arduino digital pins to DIR1 and DIR2 inputs (for direction control)
- Connect Arduino GND to driver GND (common ground required)

**Safety Notes:**
- Use separate power supply for motors to prevent voltage drops affecting the Arduino
- Add flyback diodes if not integrated in the driver
- Keep motor power wires short and twisted to reduce noise
- Consider adding capacitors (0.1µF) across motor terminals to reduce EMI

## Development

### Adding New Features
1. For hardware control: Edit Arduino files in `source/arduino/RoboSort/`
2. For high-level logic: Edit Python files in `source/rpi/RoboSort/`
3. Test thoroughly using the serial command interface

### Debugging
- Use the Arduino Serial Monitor for low-level debugging
- Add Serial.println() statements in the Arduino code
- Use Python's print() or logging module for Raspberry Pi debugging

## Contributing
Contributions are welcome! Please follow these guidelines:
1. Fork the repository
2. Create a feature branch
3. Commit your changes with clear, descriptive messages
4. Push to your fork
5. Submit a pull request

## License
This project is licensed under the MIT License. See the LICENSE file for details.

## Contact

For questions, suggestions, or contributions, feel free to reach out:

- **Email**: quezon.province.pd@gmail.com
- **GitHub**: [github.com/qppd](https://github.com/qppd)
- **Portfolio**: [sajed-mendoza.onrender.com](https://sajed-mendoza.onrender.com)
- **Facebook**: [facebook.com/qppd.dev](https://facebook.com/qppd.dev)
- **Facebook Page**: [facebook.com/QUEZONPROVINCEDEVS](https://facebook.com/QUEZONPROVINCEDEVS)