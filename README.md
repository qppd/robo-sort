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
- **Microcontroller**: Arduino-compatible board (Uno, Mega, etc.)
- **Single-board Computer**: Raspberry Pi (any model with GPIO)
- **Power Supply**: Adequate power for servos and logic circuits

## Software Components

### Arduino Firmware
- **RoboSort.ino**: Main firmware, handles serial commands and servo control
- **SERVO_CONFIG.h**: Header file with servo configuration and function declarations
- **SERVO_CONFIG.cpp**: Implementation of servo control logic and movement functions

### Raspberry Pi Software
- **RoboSort.py**: High-level control logic, sensor integration, and advanced features

### Serial Commands
- `TEST`: Runs a test sequence on all servos
- `S<servo> <angle>`: Sets a specific servo (0-4) to a given angle (0-180)

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
2. Connect to the Arduino via serial monitor (9600 baud)
3. Use the serial commands to control and test the servos

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
    │       ├── RoboSort.ino
    │       ├── SERVO_CONFIG.h
    │       └── SERVO_CONFIG.cpp
    └── rpi/
        └── RoboSort/
            └── RoboSort.py
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