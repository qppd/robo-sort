# RoboSort ROS2 Workspace

## Overview

This ROS2 workspace contains the complete software stack for the RoboSort waste segregation robot system. The system uses YOLO for object detection, Arduino for hardware control, and ROS2 for coordination and visualization.

## System Architecture

### Hardware Components
- **Robot Arm**: 5-DOF articulated arm (servos 0-4)
- **Lifter**: 1 continuous rotation servo (servo 5) for vertical movement
- **Trash Bin**: Rotating bin with 4 compartments (paper, plastic, metal, other)
- **Stepper Motor**: Controls bin rotation (50 steps per compartment)
- **Ultrasonic Sensors**: 4 HC-SR04 sensors monitoring bin fill levels
- **Camera**: USB webcam or Raspberry Pi Camera for vision
- **Arduino Mega**: Hardware controller connected via USB serial

### Software Architecture

```
┌─────────────────────────────────────────────────────────┐
│                   ROS2 RoboSort System                  │
├─────────────────────────────────────────────────────────┤
│                                                         │
│  ┌──────────────┐      ┌──────────────────────┐       │
│  │ YOLO Detector│─────▶│ Waste Segregation   │       │
│  │   Node       │      │   Controller        │       │
│  └──────────────┘      └──────────────────────┘       │
│         │                        │                      │
│         │                        ▼                      │
│         │              ┌──────────────────┐            │
│         │              │ Arduino Serial   │            │
│         │              │   Bridge Node    │            │
│         │              └──────────────────┘            │
│         │                        │                      │
│         ▼                        ▼                      │
│  ┌──────────────┐      ┌──────────────────┐           │
│  │    RViz      │      │   Arduino Mega   │           │
│  │ Visualization│      │  (Hardware Ctrl) │           │
│  └──────────────┘      └──────────────────┘           │
│                                                         │
└─────────────────────────────────────────────────────────┘
```

## Packages

### 1. `robosort_interfaces`
Custom ROS2 service definitions for robot control.

**Services:**
- `SetServo.srv` - Control individual servo (0-5, 0-180°)
- `MoveRobotArm.srv` - Move arm to position (joint angles)
- `RotateBin.srv` - Rotate bin to compartment (0-3)
- `GetDistance.srv` - Read ultrasonic sensor distance

### 2. `robosort_vision`
Main package containing all nodes for waste detection and control.

**Nodes:**

#### `yolo_detector`
- **Function**: Real-time waste detection and classification using YOLO
- **Subscribes**: Camera feed (USB/PiCamera)
- **Publishes**: 
  - `/robosort/detections` (Detection2DArray) - Detected objects with bounding boxes
  - `/robosort/annotated_image` (Image) - Annotated camera feed for visualization
- **Parameters**:
  - `model_path`: Path to YOLO model (default: `yolov8n.pt`)
  - `camera_source`: Camera source (default: `usb0`)
  - `confidence_threshold`: Detection confidence (default: `0.5`)
  - `publish_rate`: Frame rate in Hz (default: `30.0`)

#### `arduino_serial`
- **Function**: Serial communication bridge to Arduino Mega
- **Subscribes**: None (service-based)
- **Publishes**:
  - `/robosort/ultrasonic_levels` (Float32MultiArray) - 4 sensor readings
  - `/robosort/arduino_status` (String) - Connection status
- **Services Provided**:
  - `/robosort/set_servo` - Set individual servo angle
  - `/robosort/move_arm` - Move robot arm to position
  - `/robosort/rotate_bin` - Rotate trash bin
  - `/robosort/get_distance` - Get ultrasonic distance
  - `/robosort/home_arm` - Home arm to default position
  - `/robosort/enable_servos` - Enable servo motors
  - `/robosort/disable_servos` - Disable servo motors
- **Parameters**:
  - `serial_port`: Arduino serial port (default: `/dev/ttyACM0`)
  - `baudrate`: Serial baudrate (default: `9600`)
  - `ultrasonic_publish_rate`: Sensor read rate (default: `10.0` Hz)

#### `waste_segregation_controller`
- **Function**: Main control logic for waste sorting
- **Subscribes**:
  - `/robosort/detections` - Object detections from YOLO
  - `/robosort/ultrasonic_levels` - Bin fill levels
- **Publishes**:
  - `/robosort/controller_status` (String) - Current operation status
- **Service Clients**: All Arduino serial services
- **Parameters**:
  - `bin_capacity_threshold`: Min distance for full bin (default: `15.0` cm)
  - `pickup_height`: Lifter servo angle for pickup (default: `100`)
  - `drop_height`: Lifter servo angle for drop (default: `0`)

## Installation

### Prerequisites
- ROS2 Jazzy installed
- Python 3.10+
- Arduino Mega with RoboSort firmware uploaded
- USB camera or Raspberry Pi Camera

### Dependencies

```bash
# Install ROS2 dependencies
sudo apt install ros-jazzy-vision-msgs ros-jazzy-cv-bridge ros-jazzy-rviz2

# Install Python packages
pip install --break-system-packages ultralytics opencv-python pyserial picamera2
```

### Build Workspace

```bash
cd /home/robosort/robo-sort/source/rpi/ros2-robosort
source /opt/ros/jazzy/setup.bash

# Build interfaces first
colcon build --symlink-install --packages-select robosort_interfaces

# Build vision package
source install/setup.bash
colcon build --symlink-install --packages-select robosort_vision

# Source the workspace
source install/setup.bash
```

## Usage

### Quick Start

```bash
# Source the workspace
cd /home/robosort/robo-sort/source/rpi/ros2-robosort
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Launch the complete system
ros2 launch robosort_vision robosort.launch.py
```

### Launch with Custom Parameters

```bash
# Use custom YOLO model and camera
ros2 launch robosort_vision robosort.launch.py \
  model_path:=/path/to/your/model.pt \
  camera_source:=usb0 \
  serial_port:=/dev/ttyACM0 \
  confidence_threshold:=0.6

# Launch without RViz
ros2 launch robosort_vision robosort.launch.py use_rviz:=false
```

### Run Individual Nodes

```bash
# YOLO Detector
ros2 run robosort_vision yolo_detector \
  --ros-args -p model_path:=yolov8n.pt -p camera_source:=usb0

# Arduino Serial Bridge
ros2 run robosort_vision arduino_serial \
  --ros-args -p serial_port:=/dev/ttyACM0

# Waste Segregation Controller
ros2 run robosort_vision waste_segregation_controller
```

## ROS2 Topics

### Published Topics
- `/robosort/detections` (vision_msgs/Detection2DArray) - YOLO detections
- `/robosort/annotated_image` (sensor_msgs/Image) - Annotated camera feed
- `/robosort/ultrasonic_levels` (std_msgs/Float32MultiArray) - Bin levels [s1, s2, s3, s4]
- `/robosort/controller_status` (std_msgs/String) - Current operation
- `/robosort/arduino_status` (std_msgs/String) - Arduino connection status

## ROS2 Services

### Robot Control Services
- `/robosort/set_servo` (robosort_interfaces/SetServo)
- `/robosort/move_arm` (robosort_interfaces/MoveRobotArm)
- `/robosort/rotate_bin` (robosort_interfaces/RotateBin)
- `/robosort/get_distance` (robosort_interfaces/GetDistance)
- `/robosort/home_arm` (std_srvs/Trigger)
- `/robosort/enable_servos` (std_srvs/Trigger)
- `/robosort/disable_servos` (std_srvs/Trigger)

## Visualization

### RViz Configuration

The system includes a pre-configured RViz setup at:
```
src/robosort_vision/config/robosort.rviz
```

**Displays:**
- YOLO annotated camera feed
- Detection markers (future enhancement)
- System status panels

To launch RViz separately:
```bash
rviz2 -d src/robosort_vision/config/robosort.rviz
```

## Configuration

### Robot Arm Positions

Edit in `waste_segregation_controller.py`:

```python
# Pickup position [base, shoulder, elbow, wrist, gripper, lifter]
pickup_pose = [90.0, 45.0, 45.0, 90.0, 0.0, 0.0]

# Home position
home_angles = [90, 90, 90, 90, 90, 0]
```

### Bin Mapping

Compartment assignments:
- **0**: Paper
- **1**: Plastic
- **2**: Metal
- **3**: Other/Mixed

### Servo Mapping

| Servo | Function | Range |
|-------|----------|-------|
| 0 | Base rotation | 0-180° |
| 1 | Shoulder | 0-180° |
| 2 | Elbow | 0-180° |
| 3 | Wrist | 0-180° |
| 4 | Gripper | 0-180° (0=open, 45=closed) |
| 5 | Lifter | 0-180° (0=down, 100=up) |

### Ultrasonic Sensors

| Sensor ID | Compartment | Location |
|-----------|-------------|----------|
| 1 | Paper | Bin position 0 |
| 2 | Plastic | Bin position 1 |
| 3 | Metal | Bin position 2 |
| 4 | Other | Bin position 3 |

## Waste Sorting Workflow

1. **Detection**: YOLO continuously monitors camera feed
2. **Classification**: Objects classified as paper/plastic/metal/other
3. **Bin Check**: Verify target bin has capacity
4. **Pickup**: Arm moves to object, gripper closes
5. **Lift**: Lifter raises arm to transport height
6. **Rotate**: Bin rotates to correct compartment
7. **Drop**: Arm lowers, gripper opens
8. **Home**: Return to ready position

## Troubleshooting

### Camera Issues
```bash
# List available cameras
ls /dev/video*

# Test camera
ros2 run robosort_vision yolo_detector --ros-args -p camera_source:=usb0
```

### Serial Connection Issues
```bash
# Check Arduino port
ls /dev/ttyACM* /dev/ttyUSB*

# Add user to dialout group
sudo usermod -a -G dialout $USER
# Log out and back in

# Test serial connection
ros2 run robosort_vision arduino_serial --ros-args -p serial_port:=/dev/ttyACM0
```

### YOLO Model Issues
```bash
# Download pre-trained model
wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.pt

# Train custom model (recommended for waste classification)
# See: https://docs.ultralytics.com/modes/train/
```

### Build Issues
```bash
# Clean build
cd /home/robosort/robo-sort/source/rpi/ros2-robosort
rm -rf build install log

# Rebuild
colcon build --symlink-install
```

## Testing

### Test Individual Components

```bash
# Test YOLO detection
ros2 topic echo /robosort/detections

# Test ultrasonic sensors
ros2 topic echo /robosort/ultrasonic_levels

# Test servo control
ros2 service call /robosort/set_servo robosort_interfaces/srv/SetServo "{servo_num: 0, angle: 90}"

# Test bin rotation
ros2 service call /robosort/rotate_bin robosort_interfaces/srv/RotateBin "{compartment_number: 1}"

# Home the arm
ros2 service call /robosort/home_arm std_srvs/srv/Trigger
```

### Monitor System Status

```bash
# List all nodes
ros2 node list

# List all topics
ros2 topic list

# List all services
ros2 service list

# Check node info
ros2 node info /yolo_detector
ros2 node info /arduino_serial
ros2 node info /waste_segregation_controller
```

## Performance

- **Detection Rate**: ~30 FPS (depends on model and hardware)
- **Sorting Cycle**: ~8-10 seconds per object
- **Bin Capacity Check**: 10 Hz
- **Serial Communication**: 9600 baud

## Future Enhancements

- [ ] LiDAR integration for 3D positioning
- [ ] Multiple object sorting queue
- [ ] Web-based monitoring dashboard
- [ ] Database logging of sorted items
- [ ] Mobile app control interface
- [ ] Voice notifications
- [ ] Multi-camera support for better coverage

## Contributing

See main repository README for contribution guidelines.

## License

MIT License - See LICENSE file for details.

## Contact

- **Email**: quezon.province.pd@gmail.com
- **GitHub**: [github.com/qppd](https://github.com/qppd)
- **Portfolio**: [sajed-mendoza.onrender.com](https://sajed-mendoza.onrender.com)

## Acknowledgments

- Based on CamJam EduKit #3 robot platform
- YOLO by Ultralytics
- ROS2 Jazzy
- Arduino community
