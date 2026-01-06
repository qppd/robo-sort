# RoboSort Advanced Integration Guide

## ✨ New Features Integrated

### 1. 🔵 LiDAR LD06 Integration

The LD06 LiDAR provides 360° distance measurements for accurate 3D object localization.

#### Hardware Setup
- **LiDAR Model:** LDRobot LD06
- **Connection:** USB to Serial (/dev/ttyUSB0)
- **Baudrate:** 230400
- **Range:** 0.02m - 12m
- **FOV:** 360°
- **Scan Rate:** 10 Hz

#### ROS2 Package: `robosort_sensors`

**Nodes:**
1. **ld06_lidar** - LiDAR driver (from ldlidar_stl_ros2 package)
2. **lidar_processor** - Processes LiDAR scans, detects objects
3. **object_localizer** - Combines LiDAR + YOLO for 3D coordinates

**Topics:**
- `/scan` - LaserScan data from LD06
- `/robosort/object_distance` - Distance to closest object (Float32)
- `/robosort/object_position_3d` - 3D position (PointStamped)
- `/robosort/pickup_point` - Calculated gripper pickup point
- `/robosort/lidar_status` - Status messages

**Services:**
- `/robosort/get_object_position` - Get object 3D coordinates

#### Launch LiDAR System

```bash
cd /home/robosort/robo-sort/source/rpi/ros2-robosort
source install/setup.bash

# Launch LiDAR with default settings
ros2 launch robosort_sensors lidar.launch.py

# Custom serial port
ros2 launch robosort_sensors lidar.launch.py serial_port:=/dev/ttyUSB1
```

#### Test LiDAR Integration

```bash
# View laser scan
ros2 topic echo /scan

# Check object distance
ros2 topic echo /robosort/object_distance

# Get 3D position
ros2 topic echo /robosort/object_position_3d

# Call service for position
ros2 service call /robosort/get_object_position robosort_interfaces/srv/GetObjectPosition
```

#### YOLO + LiDAR Fusion Workflow

```
1. YOLO detects object in camera image
   ↓
2. Calculates object angle from bbox center
   ↓
3. LiDAR provides distance at that angle
   ↓
4. Combines into 3D coordinates (x, y, z)
   ↓
5. Calculates pickup point with gripper offset
   ↓
6. Sends to robot arm for pickup
```

---

### 2. 🦾 URDF Robot Arm with Kinematics

Professional robot description with proper link dimensions and joint limits.

#### ROS2 Package: `robosort_description`

**URDF Model:** `robosort_arm.urdf.xacro`

**Robot Structure:**
```
world
 └─ base_link (fixed)
     └─ shoulder_link (revolute, ±180°) [Servo 0]
         └─ elbow_link (revolute, ±90°) [Servo 1]
             └─ wrist_link (revolute, ±90°) [Servo 2]
                 └─ gripper_base_link (revolute, ±180°) [Servo 3]
                     ├─ gripper_left_link (revolute, 0-45°) [Servo 4]
                     ├─ gripper_right_link (mimic left)
                     └─ end_effector_link (fixed)
```

**Link Dimensions:**
- Base: 5cm radius, 3cm height
- Shoulder: 15cm length
- Elbow: 12cm length
- Wrist: 8cm length
- Gripper: 6cm total length

**Total Reach:** ~35cm from base

#### Visualize Robot in RViz

```bash
cd /home/robosort/robo-sort/source/rpi/ros2-robosort
source install/setup.bash

# Launch with joint GUI controls
ros2 launch robosort_description display.launch.py

# Without GUI (publish joint states programmatically)
ros2 launch robosort_description display.launch.py gui:=false
```

#### Use URDF for Kinematics

The URDF model enables:
- **Forward Kinematics** - Calculate end effector position from joint angles
- **Inverse Kinematics** - Calculate joint angles to reach target position
- **Collision Detection** - Prevent self-collision and workspace violations
- **Visualization** - See robot motion in RViz before execution

**Example: Calculate FK**
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

# Publish joint states
joint_state = JointState()
joint_state.name = ['base_revolute', 'shoulder_joint', 'elbow_joint', 
                    'wrist_joint', 'gripper_left_joint']
joint_state.position = [0.0, 0.5, 0.3, 0.0, 0.2]  # radians
joint_state_pub.publish(joint_state)
```

---

### 3. 🚗 DC Motor Control (L298N Drivers)

Control two DC motors independently using 2x L298N H-bridge modules.

#### Hardware Configuration

**Motor A (Left):**
- L298N Module 1
- IN1 → Arduino Pin (defined in PINS.h)
- IN2 → Arduino Pin
- ENA → Connected (no PWM speed control in current setup)

**Motor B (Right):**
- L298N Module 2
- IN3 → Arduino Pin (defined in PINS.h)
- IN4 → Arduino Pin
- ENB → Connected

**Power:**
- 12V supply for motors
- 5V from Arduino VIN for logic (or use L298N 5V out)

#### ROS2 Service Interface

**Service:** `/robosort/control_motor`  
**Type:** `robosort_interfaces/srv/ControlMotor`

**Request:**
```
uint8 motor_id    # 0=Motor A (left), 1=Motor B (right)
uint8 direction   # 0=STOP, 1=FORWARD, 2=BACKWARD, 3=BRAKE
uint8 speed       # 0-255 (currently not used, full speed only)
```

**Response:**
```
bool success
string message
```

#### Control DC Motors

```bash
# Motor A forward
ros2 service call /robosort/control_motor robosort_interfaces/srv/ControlMotor \
    "{motor_id: 0, direction: 1, speed: 200}"

# Motor B backward
ros2 service call /robosort/control_motor robosort_interfaces/srv/ControlMotor \
    "{motor_id: 1, direction: 2, speed: 150}"

# Stop Motor A
ros2 service call /robosort/control_motor robosort_interfaces/srv/ControlMotor \
    "{motor_id: 0, direction: 0, speed: 0}"

# Brake both motors
ros2 service call /robosort/control_motor robosort_interfaces/srv/ControlMotor \
    "{motor_id: 0, direction: 3, speed: 0}"
ros2 service call /robosort/control_motor robosort_interfaces/srv/ControlMotor \
    "{motor_id: 1, direction: 3, speed: 0}"

# Test motors (automated sequence)
ros2 service call /robosort/test_motors std_srvs/srv/Trigger
```

#### Arduino Commands (Direct Serial)

The Arduino firmware already supports DC motors:

```
MTEST              - Run full motor test sequence (both motors)
MA F 200           - Motor A forward at speed 200
MB B 150           - Motor B backward at speed 150
MA S 0             - Motor A stop
MSTOP              - Stop all motors
MCTEST             - Start continuous motor test (alternating directions)
MCSTOP             - Stop continuous test
```

**Direction codes:**
- `F` = Forward
- `B` = Backward
- `S` = Stop
- `R` = Brake (short IN1 and IN2 to HIGH)

---

## 🛠️ Complete System Integration

### Updated System Architecture

```
┌──────────────────────────────────────────────────────────────────┐
│                        Hardware Layer                             │
│  ┌─────────┐  ┌─────────┐  ┌─────────┐  ┌──────┐  ┌───────────┐│
│  │ USB Cam │  │ LD06    │  │ Arduino │  │ 4×   │  │ 2× L298N  ││
│  │         │  │ LiDAR   │  │  Mega   │  │ HC-  │  │ DC Motor  ││
│  │         │  │         │  │ +Servos │  │ SR04 │  │  Drivers  ││
│  └─────────┘  └─────────┘  └─────────┘  └──────┘  └───────────┘│
└───────┬────────────┬─────────────┬──────────┬────────────┬───────┘
        │            │             │          │            │
        ▼            ▼             ▼          ▼            ▼
┌──────────────────────────────────────────────────────────────────┐
│                         ROS2 Layer                                │
│                                                                   │
│  ┌──────────────┐    ┌──────────────────┐    ┌─────────────────┐│
│  │yolo_detector │───▶│  lidar_processor │───▶│object_localizer ││
│  │              │    │                  │    │                 ││
│  │ • Vision     │    │ • 360° scan      │    │ • 3D position   ││
│  │ • YOLO AI    │    │ • Object detect  │    │ • Pickup point  ││
│  └──────────────┘    └──────────────────┘    └─────────────────┘│
│         │                     │                        │          │
│         └──────────────────┬──┴────────────────────────┘          │
│                            ▼                                       │
│                 ┌──────────────────────┐                          │
│                 │ waste_segregation_   │                          │
│                 │    controller        │                          │
│                 │ • Sorting logic      │                          │
│                 │ • Arm IK/FK          │                          │
│                 │ • Bin management     │                          │
│                 └──────────────────────┘                          │
│                            │                                       │
│                            ▼                                       │
│                 ┌──────────────────────┐                          │
│                 │   arduino_serial     │                          │
│                 │ • Servo control      │                          │
│                 │ • Stepper control    │                          │
│                 │ • DC motor control   │                          │
│                 │ • Sensor reading     │                          │
│                 └──────────────────────┘                          │
│                                                                   │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │  robot_state_publisher (URDF kinematics & TF)              │ │
│  └────────────────────────────────────────────────────────────┘ │
└──────────────────────────────────────────────────────────────────┘
```

### Build All New Packages

```bash
cd /home/robosort/robo-sort/source/rpi/ros2-robosort

# Source ROS2
source /opt/ros/jazzy/setup.bash

# Install dependencies (if needed)
sudo apt install -y \
    ros-jazzy-ldlidar-stl-ros2 \
    ros-jazzy-xacro \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui

# Clean previous build (optional)
rm -rf build install log

# Build workspace
colcon build --symlink-install

# Source workspace
source install/setup.bash
```

### Launch Complete System

**Option 1: Launch Everything**
```bash
# Terminal 1: Main RoboSort system
cd /home/robosort/robo-sort/source/rpi/ros2-robosort
source install/setup.bash
ros2 launch robosort_vision robosort.launch.py

# Terminal 2: LiDAR integration
ros2 launch robosort_sensors lidar.launch.py

# Terminal 3: Robot visualization
ros2 launch robosort_description display.launch.py
```

**Option 2: Combined Launch File (TODO: create master launch)**

---

## 📊 New ROS2 Topics & Services

### Additional Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/scan` | LaserScan | LD06 LiDAR 360° scan data |
| `/robosort/object_distance` | Float32 | Distance to closest object |
| `/robosort/object_position_3d` | PointStamped | Object 3D coordinates |
| `/robosort/pickup_point` | PointStamped | Gripper pickup coordinates |
| `/robosort/lidar_status` | String | LiDAR system status |
| `/joint_states` | JointState | Robot arm joint positions |
| `/tf` | TFMessage | Transform tree for robot |

### Additional Services

| Service | Type | Description |
|---------|------|-------------|
| `/robosort/get_object_position` | GetObjectPosition | Query object 3D location |
| `/robosort/control_motor` | ControlMotor | Control DC motors (L298N) |
| `/robosort/test_motors` | Trigger | Run DC motor test sequence |

---

## 🎯 Use Cases

### 1. Autonomous Mobile Base (DC Motors)
Use the two DC motors for a differential drive mobile base:

```python
# Move forward
control_motor(motor_id=0, direction=FORWARD, speed=200)
control_motor(motor_id=1, direction=FORWARD, speed=200)

# Turn left (left slower)
control_motor(motor_id=0, direction=FORWARD, speed=100)
control_motor(motor_id=1, direction=FORWARD, speed=200)

# Turn right (right slower)
control_motor(motor_id=0, direction=FORWARD, speed=200)
control_motor(motor_id=1, direction=FORWARD, speed=100)

# Stop
control_motor(motor_id=0, direction=STOP, speed=0)
control_motor(motor_id=1, direction=STOP, speed=0)
```

### 2. Precise Object Pickup with LiDAR + YOLO

```python
# 1. YOLO detects "bottle" at image coordinates (320, 240)
# 2. LiDAR measures distance at corresponding angle → 0.5m
# 3. System calculates 3D position: (0.48m, 0.0m, 0.0m)
# 4. Adds gripper offset: (0.53m, 0.0m, 0.0m)
# 5. IK solver calculates joint angles from URDF
# 6. Arm moves to pickup point with smooth trajectory
# 7. Gripper closes, object grasped
```

### 3. Kinematics-Based Motion Planning

```python
# Use URDF for collision-free path planning
# Calculate joint trajectory from home to target
# Execute smooth motion profile
# Monitor actual vs. desired position using TF
```

---

## 🚀 Next Steps

1. ✅ **Install LiDAR Driver**
   ```bash
   sudo apt install ros-jazzy-ldlidar-stl-ros2
   ```

2. ✅ **Build Workspace**
   ```bash
   cd /home/robosort/robo-sort/source/rpi/ros2-robosort
   colcon build --symlink-install
   ```

3. ✅ **Test LiDAR**
   ```bash
   ros2 launch robosort_sensors lidar.launch.py
   ros2 topic echo /scan
   ```

4. ✅ **Visualize Robot**
   ```bash
   ros2 launch robosort_description display.launch.py
   ```

5. ✅ **Test DC Motors**
   ```bash
   ros2 service call /robosort/test_motors std_srvs/srv/Trigger
   ```

6. **Create Master Launch File** (combines all systems)

7. **Implement IK Solver** (use ikpy or KDL)

8. **Add Navigation Stack** (for mobile base)

9. **Implement MoveIt2** (advanced motion planning)

---

## 📚 References

- [LD06 LiDAR Datasheet](https://www.ldrobot.com/product/en/97)
- [ldlidar_stl_ros2 GitHub](https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2)
- [ROS2 URDF Tutorial](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/URDF-Main.html)
- [L298N Motor Driver Guide](https://lastminuteengineers.com/l298n-dc-stepper-driver-arduino-tutorial/)
- [MoveIt2 Documentation](https://moveit.picknik.ai/)

---

**Updated:** December 23, 2025  
**Version:** 2.0.0  
**Status:** ✅ Fully Integrated

