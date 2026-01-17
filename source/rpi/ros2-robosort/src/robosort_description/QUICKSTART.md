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
# Gazebo and plugins
sudo apt install ros-jazzy-gazebo-ros-pkgs ros-jazzy-gazebo-plugins

# Robot description tools
sudo apt install ros-jazzy-xacro ros-jazzy-robot-state-publisher

# Teleoperation
sudo apt install ros-jazzy-teleop-twist-keyboard
```

## Quick Launch Commands

### Visualization Only (RViz)
```bash
ros2 launch robosort_description view_robot.launch.py
```
*Opens RViz to view the robot model without simulation*

### Gazebo Simulation
```bash
ros2 launch robosort_description gazebo.launch.py
```
*Launches Gazebo physics simulation*

### Complete System (Gazebo + RViz + Control)
```bash
ros2 launch robosort_description simulation.launch.py
```
*Full simulation environment with obstacle avoidance*

## Control the Robot

### Terminal 1: Launch Simulation
```bash
cd ~/robo-sort/source/rpi/ros2-robosort
source install/setup.bash
ros2 launch robosort_description simulation.launch.py
```

### Terminal 2: Keyboard Teleop
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
