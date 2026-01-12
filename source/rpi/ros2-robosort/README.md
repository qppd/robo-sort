# RoboSort ROS2 Workspace

## 📍 Quick Reference

This is the ROS2 Jazzy workspace for the RoboSort waste segregation system.

**👉 For complete documentation, installation instructions, and usage guide, see:**
**[Main README.md](../../../README.md#ros2-integration--lidar-testing)**

## 🚀 Quick Start

### Build Workspace
```bash
cd ~/robo-sort/source/rpi/ros2-robosort
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Test LiDAR LD06
```bash
# Find your LiDAR serial port
ls /dev/ttyUSB* /dev/ttyACM*

# Launch LiDAR system (adjust port as needed)
ros2 launch robosort_sensors lidar.launch.py serial_port:=/dev/ttyUSB1

# In another terminal - Launch RViz visualization
ros2 launch robosort_description display.launch.py

# Monitor LiDAR data
ros2 topic echo /robosort/object_distance
ros2 topic echo /scan --once
```

## 📦 Package Structure

### Core Packages
- **robosort_interfaces** - Custom ROS2 service definitions
- **robosort_vision** - YOLO detector, Arduino serial bridge, waste controller
- **robosort_sensors** - LiDAR LD06 integration & 3D object localization
- **robosort_description** - URDF 5-DOF robot arm model
- **camjam_control** - DC motor control
- **camjam_sensors** - Additional sensors

### External Dependencies
- **ldlidar_stl_ros2** - LD06 LiDAR driver

## 🎯 Launch Commands

```bash
# Main system (YOLO + Servos + Stepper)
ros2 launch robosort_vision robosort.launch.py

# LiDAR system (adjust serial port)
ros2 launch robosort_sensors lidar.launch.py serial_port:=/dev/ttyUSB1

# Robot visualization
ros2 launch robosort_description display.launch.py
```

## 🔧 Key Services

```bash
# Arm control
ros2 service call /robosort/set_servo robosort_interfaces/srv/SetServo \
    "{servo_num: 0, angle: 90}"

# Bin rotation
ros2 service call /robosort/rotate_bin robosort_interfaces/srv/RotateBin \
    "{compartment_number: 1}"

# DC motors
ros2 service call /robosort/control_motor robosort_interfaces/srv/ControlMotor \
    "{motor_id: 0, direction: 1, speed: 200}"

# LiDAR 3D position
ros2 service call /robosort/get_object_position \
    robosort_interfaces/srv/GetObjectPosition
```

## 📊 Key Topics

```bash
# LiDAR
/scan                           # Raw 360° scan
/robosort/object_distance       # Distance to object
/robosort/object_position_3d    # 3D coordinates

# Vision
/robosort/detections            # YOLO objects
/robosort/annotated_image       # Camera feed

# Status
/robosort/arduino_status        # Serial status
/robosort/controller_status     # System status
```

## 🛠️ System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    ROS2 RoboSort System                     │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌──────────────┐    ┌─────────────┐    ┌──────────────┐  │
│  │ YOLO Detector│───▶│ LiDAR LD06  │───▶│   Object     │  │
│  │     Node     │    │  Processor  │    │  Localizer   │  │
│  └──────────────┘    └─────────────┘    └──────────────┘  │
│         │                    │                    │         │
│         └────────────────────┴────────────────────┘         │
│                              │                               │
│                              ▼                               │
│                   ┌──────────────────────┐                  │
│                   │ Waste Segregation   │                  │
│                   │    Controller       │                  │
│                   └──────────────────────┘                  │
│                              │                               │
│                              ▼                               │
│                   ┌──────────────────────┐                  │
│                   │  Arduino Serial      │                  │
│                   │    Bridge Node       │                  │
│                   └──────────────────────┘                  │
│                              │                               │
│         ┌────────────────────┼────────────────────┐        │
│         ▼                    ▼                    ▼        │
│  ┌──────────┐       ┌──────────────┐      ┌──────────┐   │
│  │  RViz2   │       │ Arduino Mega │      │ Hardware │   │
│  │Visualize │       │  (Servos +   │      │ Sensors  │   │
│  └──────────┘       │   Stepper)   │      └──────────┘   │
│                     └──────────────┘                       │
└─────────────────────────────────────────────────────────────┘
```

## 📚 Full Documentation

For complete details including:
- Detailed package descriptions
- Configuration parameters
- Troubleshooting guide
- Advanced usage examples
- Hardware integration details

**See the [Main Project README](../../../README.md)**

## 🔗 Quick Links

- **Installation**: [README.md#ros2-installation--setup](../../../README.md#ros2-installation--setup)
- **LiDAR Testing**: [README.md#lidar-ld06-test--visualization](../../../README.md#lidar-ld06-test--visualization)
- **Troubleshooting**: [README.md#troubleshooting-ros2--lidar](../../../README.md#troubleshooting-ros2--lidar)
- **Services & Topics**: [README.md#ros2-topics--services](../../../README.md#ros2-topics--services)

---

**Status**: ✅ Fully Integrated  
**ROS Version**: ROS2 Jazzy  
**Last Updated**: January 12, 2026

## Contact

- **Email**: quezon.province.pd@gmail.com
- **GitHub**: [github.com/qppd](https://github.com/qppd)
- **Portfolio**: [sajed-mendoza.onrender.com](https://sajed-mendoza.onrender.com)
