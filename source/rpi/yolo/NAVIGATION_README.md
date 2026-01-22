# RoboSort Autonomous Navigation System

Autonomous obstacle avoidance system for RoboSort using LD06 LIDAR and Arduino Mega motor controller.

## Overview

This system enables RoboSort to autonomously navigate and avoid obstacles using:
- **LD06 LIDAR** - 360° laser distance sensor for environment scanning
- **Obstacle Avoidance Algorithm** - Left vs right decision logic based on angle and distance analysis
- **Arduino Mega** - DC motor controller for forward, backward, turn left/right, and rotate movements

## System Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    Raspberry Pi                          │
│  ┌────────────────────────────────────────────────────┐ │
│  │          navigate.py (Main Control Loop)           │ │
│  └────────────────────────────────────────────────────┘ │
│           ▲                           │                  │
│           │                           ▼                  │
│  ┌────────────────┐         ┌───────────────────────┐  │
│  │  LIDAR Driver  │         │ Arduino Controller    │  │
│  │ (listen_to_    │         │  (Serial Commands)    │  │
│  │  lidar.py)     │         └───────────────────────┘  │
│  └────────────────┘                  │                  │
│           ▲                           │                  │
│           │                           ▼                  │
│  ┌────────────────┐         ┌───────────────────────┐  │
│  │  Obstacle      │         │   Motor Commands      │  │
│  │  Avoidance     │         │  (FORWARD, TURN,      │  │
│  │  Logic         │         │   ROTATE, etc.)       │  │
│  └────────────────┘         └───────────────────────┘  │
└──────────│────────────────────────────│─────────────────┘
           │                             │
    [USB: LD06]                   [USB: Arduino Mega]
           │                             │
     ┌─────┴─────┐                  ┌───┴────────┐
     │  LD06     │                  │  Arduino   │
     │  LIDAR    │                  │  Mega      │
     │  Sensor   │                  │  + L298N   │
     └───────────┘                  └────────────┘
```

## Files Description

### Python Files (Raspberry Pi)

#### 1. `obstacle_avoidance.py`
Core obstacle avoidance logic implementing left vs right decision-making.

**Key Features:**
- Analyzes LIDAR data by angular zones (front, left, right)
- Calculates minimum distances and obstacle counts per zone
- Implements scoring system for turn direction selection
- Provides multiple distance thresholds (safe, critical, danger)

**Main Class:** `ObstacleAvoidance`
- `analyze_obstacles()` - Categorizes obstacles by zone
- `decide_action()` - Returns (action, speed, info) tuple
- `calculate_turn_score()` - Scores left/right turn options

#### 2. `navigate.py`
Main autonomous navigation system integrating all components.

**Key Classes:**
- `ArduinoController` - Serial interface to Arduino
  - `forward()`, `backward()`, `turn_left()`, `turn_right()`
  - `rotate_left()`, `rotate_right()`, `stop()`
  
- `AutonomousNavigator` - Main navigation loop
  - `start()` - Initialize LIDAR and Arduino
  - `navigate_once()` - Single navigation decision cycle
  - `run()` - Main autonomous loop
  - `pause()`/`resume()` - Control navigation

**Usage:**
```python
navigator = AutonomousNavigator(
    lidar_port='/dev/ttyUSB0',
    arduino_port='/dev/ttyACM0',
    update_rate=0.1
)
navigator.run()
```

#### 3. `nav_config.py`
Centralized configuration for all navigation parameters.

**Key Parameters:**
- `SAFE_DISTANCE` - Minimum safe distance (default: 50cm)
- `CRITICAL_DISTANCE` - Immediate turn threshold (default: 30cm)
- `DANGER_DISTANCE` - Stop/reverse threshold (default: 15cm)
- `FORWARD_SPEED`, `TURN_SPEED`, `ROTATE_SPEED`
- Behavior modes: 'cautious', 'normal', 'aggressive'

#### 4. `test_navigation.py`
Comprehensive test suite for system validation.

**Test Modes:**
- `lidar` - Test LIDAR connection
- `arduino` - Test motor control
- `detection` - Test obstacle detection (no motors)
- `full` - Full autonomous navigation
- `config` - Display configuration

#### 5. LIDAR Driver (`ldrobot-ld06-lidar-python-driver-master/`)
- `listen_to_lidar.py` - Main LIDAR data collection
- `calc_lidar_data.py` - Parse LIDAR packets

### Arduino Files

#### `RoboSort.ino` (Updated)
Added autonomous mode support with safety features.

**New Commands:**
- `AUTO_START` - Enable autonomous mode
- `AUTO_STOP` - Disable autonomous mode and stop motors
- `AUTO_STATUS` - Check autonomous mode status
- `AUTO_HEARTBEAT` - Reset watchdog timer (required every 5 seconds)

**Safety Features:**
- 5-second watchdog timer
- Automatic motor stop on heartbeat timeout
- Autonomous mode status tracking

**Existing Motor Commands:**
- `FORWARD:<speed>` - Move forward (speed: 0-255)
- `BACKWARD:<speed>` - Move backward
- `LEFT:<speed>` - Differential turn left (right wheel faster)
- `RIGHT:<speed>` - Differential turn right (left wheel faster)
- `TURN_LEFT:<speed>` - Spot rotate left (rotate in place)
- `TURN_RIGHT:<speed>` - Spot rotate right
- `MSTOP` - Stop all motors

## Setup Instructions

### Hardware Requirements
- Raspberry Pi (any model with USB ports)
- Arduino Mega 2560
- LD06 LIDAR sensor
- L298N motor drivers (x2)
- DC motors (x2)
- Power supply

### Hardware Connections

#### LD06 LIDAR → Raspberry Pi
- Connect via USB (appears as `/dev/ttyUSB0`)
- Baudrate: 230400

#### Arduino Mega → Raspberry Pi
- Connect via USB (appears as `/dev/ttyACM0`)
- Baudrate: 9600

#### Motors → Arduino (see PINS.h)
- Motor A: IN1=Pin 7, IN2=Pin 8
- Motor B: IN1=Pin 10, IN2=Pin 11

### Software Setup

#### 1. Upload Arduino Firmware
```bash
# Open Arduino IDE
# Load: robo-sort/source/arduino/RoboSort/RoboSort.ino
# Select Board: Arduino Mega 2560
# Select Port: /dev/ttyACM0 (or appropriate port)
# Upload
```

#### 2. Install Python Dependencies
```bash
cd robo-sort/source/rpi/yolo

# Install required packages
pip install pyserial

# Verify installation
python -c "import serial; print('Serial library installed')"
```

#### 3. Configure Serial Ports
Edit `nav_config.py` if your ports differ:
```python
LIDAR_PORT = '/dev/ttyUSB0'  # Your LIDAR port
ARDUINO_PORT = '/dev/ttyACM0'  # Your Arduino port
```

To find ports:
```bash
# List all serial devices
ls /dev/tty*

# Monitor device connections
dmesg | grep tty
```

## Usage

### Quick Start

#### 1. Test Individual Components
```bash
cd robo-sort/source/rpi/yolo

# Test LIDAR
python test_navigation.py lidar

# Test Arduino motors (robot will move briefly!)
python test_navigation.py arduino

# Test obstacle detection (no motor movement)
python test_navigation.py detection --duration 30
```

#### 2. Run Autonomous Navigation
```bash
# Run with default settings
python navigate.py

# Run with custom ports
python navigate.py --lidar-port /dev/ttyUSB0 --arduino-port /dev/ttyACM0

# Run with custom update rate
python navigate.py --update-rate 0.2

# Run in quiet mode
python navigate.py --quiet
```

### Command Line Options

```bash
python navigate.py --help

Options:
  --lidar-port PORT         LIDAR serial port (default: /dev/ttyUSB0)
  --arduino-port PORT       Arduino serial port (default: /dev/ttyACM0)
  --arduino-baudrate BAUD   Arduino baudrate (default: 9600)
  --update-rate SECONDS     Navigation update rate (default: 0.1)
  --quiet                   Disable verbose output
```

### Behavior Modes

Change navigation behavior in `nav_config.py`:

```python
# Apply cautious mode (slower, more conservative)
config.apply_behavior_mode('cautious')

# Apply aggressive mode (faster, closer to obstacles)
config.apply_behavior_mode('aggressive')
```

## How It Works

### 1. LIDAR Data Collection
- LD06 scans 360° continuously at 230400 baud
- Returns distance measurements at various angles (0-360°)
- Data updated in real-time via threaded listener

### 2. Obstacle Analysis
Environment divided into three zones:
- **Front**: ±30° from forward (0°)
- **Left**: 30° to 150°
- **Right**: 210° to 330°

For each zone, system calculates:
- Minimum distance to nearest obstacle
- Number of obstacles within safe distance
- Average clear space
- Number of clear path readings

### 3. Decision Logic

```
if front_distance < DANGER_DISTANCE (15cm):
    → BACKWARD (emergency reverse)

elif front_distance < CRITICAL_DISTANCE (30cm):
    → ROTATE_LEFT or ROTATE_RIGHT (spot turn based on scores)

elif front_distance < SAFE_DISTANCE (50cm):
    → TURN_LEFT or TURN_RIGHT (differential turn)

else:
    → FORWARD (path clear)
```

### 4. Turn Direction Selection

Score calculated for each direction:
```
score = clear_space + (clear_count × 10) - (obstacles × 20)
```

Higher score = better direction to turn

### 5. Motor Control
Commands sent via serial to Arduino:
```
FORWARD:180   → Both motors forward at speed 180
LEFT:120      → Right motor faster (differential turn)
TURN_LEFT:180 → Right forward, left backward (spot rotate)
MSTOP         → Stop all motors
```

### 6. Safety Features
- **Watchdog Timer**: Arduino requires heartbeat every 5 seconds
- **Timeout Protection**: Motors auto-stop if no heartbeat
- **Data Validation**: Filters invalid LIDAR readings
- **Emergency Stop**: Ctrl+C stops navigation immediately

## Configuration Parameters

### Distance Thresholds
```python
SAFE_DISTANCE = 50.0       # Minimum safe distance (cm)
CRITICAL_DISTANCE = 30.0   # Turn required (cm)
DANGER_DISTANCE = 15.0     # Stop/reverse required (cm)
CLEAR_PATH_THRESHOLD = 80.0 # Consider path clear (cm)
```

### Angular Zones
```python
FRONT_ANGLE_RANGE = 30      # ±30° from forward
LEFT_ANGLE_RANGE = (30, 150)   # Left side zone
RIGHT_ANGLE_RANGE = (210, 330)  # Right side zone
```

### Motor Speeds (0-255)
```python
FORWARD_SPEED = 180    # Normal forward speed
BACKWARD_SPEED = 150   # Reverse speed
TURN_SPEED = 120       # Differential turn speed
ROTATE_SPEED = 180     # Spot rotation speed
```

### Timing
```python
UPDATE_RATE = 0.1           # Navigation decisions per second
ACTION_DURATION = 0.5       # Minimum action execution time
HEARTBEAT_INTERVAL = 2.0    # Arduino heartbeat interval
```

## Troubleshooting

### LIDAR Not Connecting
```bash
# Check device
ls -l /dev/ttyUSB*

# Check permissions
sudo chmod 666 /dev/ttyUSB0

# Test with screen
screen /dev/ttyUSB0 230400
```

### Arduino Not Responding
```bash
# Check device
ls -l /dev/ttyACM*

# Check permissions
sudo chmod 666 /dev/ttyACM0

# Test connection
screen /dev/ttyACM0 9600
# Type: MSTOP (should respond)
```

### Motors Not Moving
1. Check power supply connections
2. Verify L298N connections (see PINS.h)
3. Test with manual commands:
   ```bash
   python -c "from navigate import ArduinoController; a = ArduinoController(); a.connect(); a.forward(150)"
   ```

### Robot Turning Wrong Direction
- Swap motor A and B wire connections
- Or modify `DC_CONFIG.cpp` motor directions

### LIDAR Data Issues
- Check baudrate (must be 230400)
- Verify USB cable quality
- Ensure LIDAR has power
- Check for LIDAR motor spinning

## Advanced Usage

### Custom Obstacle Avoidance
```python
from obstacle_avoidance import ObstacleAvoidance

# Create custom instance
oa = ObstacleAvoidance(
    safe_distance=60.0,
    critical_distance=40.0,
    danger_distance=20.0,
    front_angle_range=45,  # Wider front zone
)

# Use with LIDAR data
action, speed, info = oa.decide_action(distances)
```

### Integration with YOLO Detection
```python
from navigate import AutonomousNavigator
from yolo_detect import YOLODetector

navigator = AutonomousNavigator()
yolo = YOLODetector()

# Navigate while detecting objects
while True:
    navigator.navigate_once()
    detections = yolo.detect()
    # Custom logic based on detections
```

### Logging Data
```python
import csv
import time

# Log navigation decisions
with open('nav_log.csv', 'w') as f:
    writer = csv.writer(f)
    writer.writerow(['timestamp', 'action', 'speed', 'front_dist', 'left_dist', 'right_dist'])
    
    while running:
        action, speed, info = navigator.navigate_once()
        analysis = oa.analyze_obstacles(distances)
        writer.writerow([
            time.time(),
            action,
            speed,
            analysis['front_min_distance'],
            analysis['left_min_distance'],
            analysis['right_min_distance']
        ])
```

## Safety Guidelines

⚠️ **IMPORTANT SAFETY NOTES:**

1. **Always test in open area** - Ensure at least 3m × 3m clear space
2. **Monitor first runs** - Stand ready to press Ctrl+C for emergency stop
3. **Check power connections** - Verify motor power supply before autonomous mode
4. **Clear obstacles** - Remove tripping hazards and fragile objects
5. **Watch for cables** - Ensure cables don't get tangled in wheels
6. **Supervise always** - Never leave robot running unattended

## Performance Tips

1. **Update Rate**: Lower `UPDATE_RATE` (0.2-0.3s) for smoother operation on slower Pi models
2. **Speed Settings**: Start with lower speeds (100-120) when testing
3. **Distance Thresholds**: Increase for larger robots or faster speeds
4. **Behavior Mode**: Use 'cautious' mode in tight spaces

## Project Structure
```
robo-sort/
├── source/
│   ├── arduino/
│   │   └── RoboSort/
│   │       ├── RoboSort.ino          ← Updated with AUTO commands
│   │       ├── DC_CONFIG.h/cpp       ← Motor control
│   │       └── PINS.h                ← Pin definitions
│   └── rpi/
│       └── yolo/
│           ├── navigate.py           ← Main navigation (NEW)
│           ├── obstacle_avoidance.py ← Avoidance logic (NEW)
│           ├── nav_config.py         ← Configuration (NEW)
│           ├── test_navigation.py    ← Test suite (NEW)
│           └── ldrobot-ld06-lidar-python-driver-master/
│               ├── listen_to_lidar.py
│               └── calc_lidar_data.py
```

## Contributing

To extend the system:
1. Add new behavior modes in `nav_config.py`
2. Implement custom scoring in `obstacle_avoidance.py`
3. Add sensors in Arduino `RoboSort.ino`
4. Create test cases in `test_navigation.py`

## License

See repository LICENSE file.

## Credits

- LD06 LIDAR driver: ldrobot-ld06-lidar-python-driver
- RoboSort project: Original hardware and Arduino control system

---

**Last Updated**: January 2026
**Version**: 1.0.0
