# RoboSort Robot - Quick Start Guide

## Installation

### 1. Build the Package

```bash
cd ~/robo-sort/source/rpi/ros2-robosort
colcon build --packages-select robosort_description
source install/setup.bash
```

### 2. Install Dependencies (if not already installed)

```bash
# Robot description tools
sudo apt install ros-jazzy-xacro ros-jazzy-robot-state-publisher

# Teleoperation
sudo apt install ros-jazzy-teleop-twist-keyboard

# For simulation (optional - may not be available on ARM64/Raspberry Pi)
# sudo apt install ros-jazzy-gazebo-ros-pkgs ros-jazzy-gazebo-plugins
```

## Quick Launch Commands

### Hardware Mode (Recommended - Real Robot)
```bash
ros2 launch robosort_control robosort.launch.py \
    arduino_port:=/dev/ttyACM0 \
    lidar_port:=/dev/ttyUSB0 \
    use_rviz:=true \
    use_teleop:=true
```
*Launches all hardware nodes (Arduino motors, LiDAR), RViz visualization, and teleop control*

### Visualization Only (RViz)
```bash
ros2 launch robosort_description view_robot.launch.py
```
*Opens RViz to view the robot model without hardware or simulation*

### Gazebo Simulation (May not work on ARM64/Raspberry Pi)
```bash
ros2 launch robosort_description simulation.launch.py
```
*Full simulation environment with obstacle avoidance (requires gazebo_ros packages)*

### SLAM Mapping (Live Map Creation)
```bash
# Install SLAM Toolbox first
sudo apt install ros-jazzy-slam-toolbox

# Launch SLAM with hardware (3 terminals needed)
# Terminal 1: Hardware + RViz
ros2 launch robosort_control robosort.launch.py \
    arduino_port:=/dev/ttyACM0 \
    lidar_port:=/dev/ttyUSB0 \
    use_rviz:=true

# Terminal 2: SLAM Toolbox
ros2 launch slam_toolbox online_async_launch.py \
    params_file:=$HOME/robo-sort/source/rpi/ros2-robosort/config/mapper_params_online_async.yaml

# Terminal 3: Teleop Control
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
*Creates live map as you drive the robot around*

## Control the Robot

### Option 1: Hardware with Built-in Teleop (Recommended)
```bash
cd ~/robo-sort/source/rpi/ros2-robosort
source install/setup.bash
ros2 launch robosort_control robosort.launch.py \
    arduino_port:=/dev/ttyACM0 \
    lidar_port:=/dev/ttyUSB0 \
    use_rviz:=true \
    use_teleop:=true
```
*This opens teleop in a separate xterm window automatically*

### Option 2: Manual Teleop in Separate Terminal

**Terminal 1: Launch Hardware**
```bash
cd ~/robo-sort/source/rpi/ros2-robosort
source install/setup.bash
ros2 launch robosort_control robosort.launch.py \
    arduino_port:=/dev/ttyACM0 \
    lidar_port:=/dev/ttyUSB0 \
    use_rviz:=true
```

**Terminal 2: Keyboard Teleop**
```bash
source ~/robo-sort/source/rpi/ros2-robosort/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Keyboard Controls:**
- `i` - Forward
- `k` - Stop
- `,` - Backward
- `j` - Turn left
- `l` - Turn right
- `u` - Forward + left
- `o` - Forward + right
- `q` - Increase speed
- `z` - Decrease speed

### Manual Commands

```bash
# Move forward at 0.5 m/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"

# Rotate counter-clockwise at 0.5 rad/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}"

# Stop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

## Monitoring

### View All Topics
```bash
ros2 topic list
```

### Monitor Velocity Commands
```bash
ros2 topic echo /cmd_vel
```

### Monitor Odometry
```bash
ros2 topic echo /odom
```

### Monitor LiDAR Scans
```bash
ros2 topic echo /scan
```

### Monitor Motor Controller Status
```bash
ros2 topic echo /robosort/motor_status
```

### Monitor SLAM Map
```bash
ros2 topic echo /map
```

### Save SLAM Map
```bash
ros2 run nav2_map_server map_saver_cli -f my_live_map
```

### View TF Tree
```bash
# Install if needed
sudo apt install ros-jazzy-tf2-tools

# Generate PDF of transform tree
ros2 run tf2_tools view_frames

# View the generated PDF
evince frames.pdf
```

## Troubleshooting

### "Package not found"
```bash
# Make sure you sourced the workspace
source ~/robo-sort/source/rpi/ros2-robosort/install/setup.bash
```

### Robot not moving in Gazebo
1. Check if `/cmd_vel` is receiving messages:
   ```bash
   ros2 topic echo /cmd_vel
   ```
2. Verify differential drive plugin loaded (check Gazebo terminal output)
3. Try resetting the simulation: Ctrl+R in Gazebo

### RViz shows no robot
1. Set Fixed Frame to `base_link` or `odom`
2. Add RobotModel display if not present
3. Check that robot_state_publisher is running:
   ```bash
   ros2 node list
   ```

### LiDAR not visible in RViz
1. Add LaserScan display type
2. Set topic to `/scan`
3. Set size to 0.05 for visibility
4. Ensure Fixed Frame is `odom`

## Next Steps

- **Autonomous Navigation**: Add Nav2 for goal-based navigation
- **SLAM**: Create maps using SLAM Toolbox
- **Hardware Integration**: Run on real robot with motor_controller node
- **Sensor Fusion**: Add camera for vision-based navigation

## File Locations

- **URDF Model**: `src/robosort_description/urdf/robosort.urdf.xacro`
- **Launch Files**: `src/robosort_description/launch/`
- **RViz Config**: `src/robosort_description/rviz/view_robot.rviz`
- **Parameters**: `src/robosort_description/config/robot_params.yaml`

## Getting Help

Check the full documentation: [README.md](README.md)
