# 🚀 RoboSort Quick Reference

## System Status: ✅ FULLY INTEGRATED

---

## 🔵 LiDAR LD06 - 3D Object Localization

**Launch:**
```bash
ros2 launch robosort_sensors lidar.launch.py
```

**Topics:**
- `/scan` - Raw LiDAR data
- `/robosort/object_distance` - Distance to object
- `/robosort/object_position_3d` - 3D coordinates
- `/robosort/pickup_point` - Gripper target

**Service:**
```bash
ros2 service call /robosort/get_object_position \
    robosort_interfaces/srv/GetObjectPosition
```

---

## 🦾 URDF Robot Arm - Professional Kinematics

**Launch:**
```bash
ros2 launch robosort_description display.launch.py
```

**Features:**
- 5-DOF arm model
- Joint limits defined
- Gripper with 2 fingers
- RViz visualization
- Ready for IK/FK

**Joints:**
- base_revolute (±180°)
- shoulder_joint (±90°)
- elbow_joint (±90°)
- wrist_joint (±180°)
- gripper_left_joint (0-45°)

---

## 🚗 DC Motors - L298N Control

**Motor A (Left) - Forward:**
```bash
ros2 service call /robosort/control_motor \
    robosort_interfaces/srv/ControlMotor \
    "{motor_id: 0, direction: 1, speed: 200}"
```

**Motor B (Right) - Backward:**
```bash
ros2 service call /robosort/control_motor \
    robosort_interfaces/srv/ControlMotor \
    "{motor_id: 1, direction: 2, speed: 150}"
```

**Stop Both:**
```bash
ros2 service call /robosort/control_motor \
    robosort_interfaces/srv/ControlMotor \
    "{motor_id: 0, direction: 0, speed: 0}"
ros2 service call /robosort/control_motor \
    robosort_interfaces/srv/ControlMotor \
    "{motor_id: 1, direction: 0, speed: 0}"
```

**Test Motors:**
```bash
ros2 service call /robosort/test_motors std_srvs/srv/Trigger
```

**Direction Codes:**
- 0 = STOP
- 1 = FORWARD
- 2 = BACKWARD
- 3 = BRAKE

---

## 🎯 Complete System Launch

```bash
# Terminal 1: Main System
cd /home/robosort/robo-sort/source/rpi/ros2-robosort
source install/setup.bash
ros2 launch robosort_vision robosort.launch.py

# Terminal 2: LiDAR
ros2 launch robosort_sensors lidar.launch.py

# Terminal 3: Visualization
ros2 launch robosort_description display.launch.py
```

---

## 📊 All Services

```bash
# Arm Control
/robosort/set_servo          # Single servo
/robosort/move_arm           # Full arm
/robosort/home_arm           # Home position

# Bin & Sensors
/robosort/rotate_bin         # Bin rotation
/robosort/get_distance       # Ultrasonic

# NEW: 3D Localization
/robosort/get_object_position  # LiDAR 3D coords

# NEW: DC Motors
/robosort/control_motor        # Motor A/B
/robosort/test_motors          # Test sequence

# Servo Enable/Disable
/robosort/enable_servos
/robosort/disable_servos
```

---

## 🔧 Build & Source

```bash
cd /home/robosort/robo-sort/source/rpi/ros2-robosort
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## 📚 Documentation

- **INTEGRATION_GUIDE.md** - Detailed guide
- **INTEGRATION_SUMMARY.md** - Quick summary  
- **ANSWERS.md** - Question answers
- **SYSTEM_GUIDE.md** - Complete system docs

---

**Version:** 2.0.0  
**Date:** December 23, 2025  
**Status:** ✅ Ready to Use!
