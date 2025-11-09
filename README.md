# RoboSort: Automated Paper and Plastic Waste Segregation System

## Overview
RoboSort is an intelligent automated waste segregation system designed to efficiently separate paper and plastic waste materials using a robotic arm and conveyor system. The project addresses the growing need for automated waste management by combining computer vision, robotics, and embedded systems to accurately identify and sort recyclable materials. Integrating both Arduino and Raspberry Pi platforms, RoboSort combines mechanical, electrical, and software components for a robust, scalable solution suitable for educational institutions, recycling facilities, and smart waste management applications.

## Architecture
The system follows a modular, two-tier architecture:
- **Low-level control**: Arduino microcontroller manages real-time servo operations, motor control, and hardware interfacing
- **High-level logic**: Raspberry Pi handles vision processing, material classification (paper vs plastic), decision-making, and network communication
- **Communication**: Serial protocol enables bidirectional data exchange between Arduino and Raspberry Pi for coordinated waste sorting operations

## Features
- Five degrees of freedom (5-DOF) robotic arm for precise waste material handling
- Automated paper and plastic waste identification and segregation
- Conveyor belt system powered by dual DC motors using E-Gizmo HPMD-3.1 motor driver
- 16-channel PWM servo driver (PCA9685) for coordinated multi-servo control
- Computer vision capability for material classification
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
  - I2C interface requiring only 2 pins (SDA, SCL)
  - 12-bit resolution PWM output
  - Adjustable frequency up to 1.6 kHz
  - Chainable up to 62 boards (992 servos total)
  - I2C address: 0x40-0x7F (selectable via solder jumpers)
  - 5V tolerant logic (works with 3.3V or 5V systems)
  - Built-in clock for free-running PWM (no continuous signal needed)
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
- **RoboSort.py**: Main application with interactive command-line interface for testing and controlling the robot
- **serial_config.py**: Modular serial communication handler for Arduino-Raspberry Pi USB communication
- **requirements.txt**: Python package dependencies

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

#### Ultrasonic Commands
- `UTEST`: Runs comprehensive ultrasonic sensor test with multiple readings
- `UDIST`: Gets single distance measurement in centimeters
- `UAVG <samples>`: Gets average distance from multiple samples (1-10)
  - Example: `UAVG 5` - Average of 5 distance samples
- `UDETECT <threshold>`: Detects if object is within threshold distance (1-400 cm)
  - Example: `UDETECT 30` - Detect objects within 30 cm

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
3. Install required Python packages:
   ```bash
   pip install -r requirements.txt
   ```
   Or install pyserial directly:
   ```bash
   pip install pyserial
   ```
4. Configure serial port permissions:
   ```bash
   sudo usermod -a -G dialout $USER
   ```
   Then logout and login again for changes to take effect
5. Connect Arduino Mega to Raspberry Pi via USB cable
6. Identify the serial port (usually `/dev/ttyACM0` or `/dev/ttyUSB0`):
   ```bash
   ls /dev/tty*
   ```
7. Run the Python application:
   ```bash
   python3 RoboSort.py
   ```

## Usage

### Raspberry Pi Interactive Control
The RoboSort.py application provides a menu-driven interface for controlling the system:

```bash
python3 RoboSort.py
```

**Menu Options:**
1. Test all servos
2. Set specific servo angle
3. Test all motors
4. Control specific motor
5. Stop all motors
6. Test ultrasonic sensor
7. Get distance measurement
8. Get average distance
9. Detect object (with threshold)
10. Send custom command
0. Exit

**Python Module Usage Example:**
```python
from serial_config import SerialConfig

# Using context manager (automatic connection/disconnection)
with SerialConfig(port='/dev/ttyACM0') as serial_conn:
    # Test components
    serial_conn.test_servos()
    serial_conn.get_distance()
    
    # Control servo
    serial_conn.set_servo(0, 90)
    
    # Control motor
    serial_conn.control_motor('A', 'F', 200)
    
    # Stop when done
    serial_conn.stop_all_motors()
```

### Arduino Serial Monitor Testing

**Basic Operation:**
1. Connect Arduino to computer via USB
2. Open Arduino IDE Serial Monitor (9600 baud)
3. Send commands directly to test components

**Testing Servos:**
```
TEST
```
Runs a test sequence on all servos, moving each through its full range of motion.

**Manual Servo Control:**
```
S0 90
S1 45
S2 180
```
Set servo 0 to 90 degrees, servo 1 to 45 degrees, and servo 2 to 180 degrees.

**Testing DC Motors:**
```
MTEST
```
Runs a comprehensive test sequence on both DC motors.

**Manual DC Motor Control:**
```
MA F 200
MB B 150
MA S 0
MSTOP
```
Motor A forward at speed 200, Motor B backward at speed 150, Motor A stop, and stop all motors.

**Testing Ultrasonic Sensor:**
```
UTEST
UDIST
UAVG 5
UDETECT 30
```
Run sensor test, get single distance, get average of 5 samples, detect object within 30 cm.

## Wiring Diagram

### System Wiring Schematic
![RoboSort Wiring Diagram](diagram/Wiring.png)

The wiring diagram shows the complete electrical connections for the RoboSort system. All components are connected to the Arduino Mega 2560 microcontroller, which serves as the central control unit.

#### Component Connections:

**1. Adafruit PCA9685 16-Channel PWM Servo Driver**
- **VCC** → Arduino 5V
- **GND** → Arduino GND
- **SDA** → Arduino Pin 20 (SDA)
- **SCL** → Arduino Pin 21 (SCL)
- **V+** → External 5V Power Supply (for servos)
- **GND** → Common Ground with external power
- **Servo Outputs** → 5 Servos connected to channels 0-4

**2. E-Gizmo HPMD-3.1 Dual DC Motor Driver**
- **Motor A Control:**
  - PWM → Arduino Pin 9
  - DIR1 → Arduino Pin 7
  - DIR2 → Arduino Pin 8
- **Motor B Control:**
  - PWM → Arduino Pin 10
  - DIR1 → Arduino Pin 11
  - DIR2 → Arduino Pin 12
- **Power:**
  - VIN → External 6-12V Power Supply
  - GND → Common Ground with Arduino
- **Motor Outputs:**
  - MA+/MA- → DC Motor A (Conveyor/Movement)
  - MB+/MB- → DC Motor B (Conveyor/Movement)

**3. HC-SR04 Ultrasonic Sensor**
- **VCC** → Arduino 5V
- **GND** → Arduino GND
- **TRIG** → Arduino Pin 4
- **ECHO** → Arduino Pin 5

**4. Power Supply Configuration**
- **Arduino Power:** USB connection from Raspberry Pi or 7-12V DC adapter
- **Servo Power:** Separate 5V power supply (recommended 5V 5A for multiple servos)
- **Motor Power:** Separate 6-12V power supply (2A minimum per motor)
- **Common Ground:** All grounds must be connected together

#### Important Notes:
- Always use separate power supplies for motors and servos to prevent voltage drops
- Ensure all grounds are connected together (common ground)
- The Fritzing source file (`Wiring.fzz`) is available in the `diagram/` folder for editing
- Verify all connections before powering on the system
- Use appropriate wire gauges for motor connections (minimum 22 AWG)

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
├── diagram/                      # Circuit diagrams and schematics
│   ├── Wiring.fzz                # Fritzing wiring diagram (editable)
│   └── Wiring.png                # Wiring diagram image
├── model/                        # 3D models and images
│   ├── robosort-front.jpg
│   ├── robosort-main.jpg
│   └── robosort-side.jpg
└── source/
    ├── arduino/
    │   └── RoboSort/
    │       ├── RoboSort.ino              # Main firmware with integrated control
    │       ├── SERVO_CONFIG.h            # Servo driver header
    │       ├── SERVO_CONFIG.cpp          # Servo driver implementation
    │       ├── DC_CONFIG.h               # DC motor driver header
    │       ├── DC_CONFIG.cpp             # DC motor driver implementation
    │       ├── ULTRASONIC_CONFIG.h       # Ultrasonic sensor header
    │       └── ULTRASONIC_CONFIG.cpp     # Ultrasonic sensor implementation
    └── rpi/
        └── RoboSort/
            ├── RoboSort.py               # Main application with CLI interface
            ├── serial_config.py          # Serial communication module
            └── requirements.txt          # Python dependencies
```

## Configuration

### Adafruit PCA9685 16-Channel PWM/Servo Driver Setup

#### Technical Specifications
- **Chip**: NXP PCA9685 PWM controller
- **Resolution**: 12-bit (4096 steps)
- **PWM Frequency**: Adjustable from ~40Hz to ~1600Hz (typically 50Hz for servos)
- **Channels**: 16 independent PWM outputs
- **Interface**: I2C (uses only 2 pins: SDA and SCL)
- **Default I2C Address**: 0x40 (changeable via solder jumpers A0-A5)
- **Address Range**: 0x40 to 0x7F (allows up to 62 boards on one I2C bus)
- **Logic Level**: 5V tolerant I2C (works with 3.3V or 5V microcontrollers)
- **Power**: Separate power input for servos (up to 6V recommended)
- **Dimensions**: 2.5" x 1" x 0.1" (62.5mm x 25.4mm x 3mm)

#### Key Features
- **Free-Running PWM**: Built-in clock means no continuous signal needed from Arduino
- **Zero Processing Overhead**: Runs independently after initial setup
- **Chainable**: Control up to 992 servos using 62 boards with only 2 pins
- **Output Protection**: 220Ω series resistors on all outputs
- **Reverse Polarity Protection**: Built-in on terminal block
- **Configurable Output**: Push-pull or open-drain mode

#### Wiring Guide
**I2C Connections (Required):**
- SDA → Arduino A4 (Uno/Nano) or Pin 20 (Mega)
- SCL → Arduino A5 (Uno/Nano) or Pin 21 (Mega)
- VCC → Arduino 5V
- GND → Arduino GND

**Servo Power (Required):**
- V+ → External power supply positive (4.8V - 6V for most servos)
- GND → External power supply ground (must share common ground with Arduino)
- Note: Do NOT power servos from Arduino's 5V pin (insufficient current)

**Servo Connections:**
- Connect servos to any of the 16 three-pin headers
- Each header provides: PWM signal, VCC, and GND
- Servos can be plugged directly into the board

#### Address Selection
By default, the board uses I2C address 0x40. To change the address:
- Solder bridge combinations of A0-A5 jumpers on the board
- Each jumper adds to the base address (0x40)
- Example: Bridging A0 sets address to 0x41

#### Library Installation
Install the Adafruit PWM Servo Driver library in Arduino IDE:
```
Tools > Manage Libraries > Search "Adafruit PWM Servo Driver" > Install
```

### Servo Calibration
Adjust the servo pulse width limits in `SERVO_CONFIG.cpp`:
```cpp
#define SERVO_MIN_PULSE  150  // Minimum pulse length (0°)
#define SERVO_MAX_PULSE  600  // Maximum pulse length (180°)
```

**Calibration Tips:**
- Standard servos typically use 150-600 pulse range (1ms-2ms)
- Fine-tune these values if servos don't reach full range or jitter at endpoints
- Test with serial commands before finalizing pulse values
- Some servos may require 120-600 or 150-650 ranges

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