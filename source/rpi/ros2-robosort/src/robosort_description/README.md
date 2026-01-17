# RoboSort Robot URDF Model

Complete URDF/XACRO model for the RoboSort differential drive robot with LiDAR sensor, ready for RViz2 visualization and Gazebo simulation.

## Robot Specifications

### Physical Dimensions
- **Body**: 35" × 20" × 4" (0.889m × 0.508m × 0.1016m)
- **Drive Wheels**: 12" diameter (0.3048m), 2" width
- **Wheel Separation**: 22" (0.558m)
- **Caster Wheels**: Two front casters for stability
- **Total Mass**: ~16 kg (body + wheels + sensors)

### Sensors
- **LiDAR**: LD06 360° laser scanner
  - Mounted at top-front center
  - Range: 0.12m - 12m
  - 360 samples per scan
  - Topic: `/scan`

### Control
- **Differential Drive**: Two rear drive wheels
- **Command Topic**: `/cmd_vel` (geometry_msgs/Twist)
- **Odometry Topic**: `/odom` (nav_msgs/Odometry)
- **Hardware Interface**: Arduino Mega via serial

## Files Structure

```
robosort_description/
├── urdf/
│   ├── robosort.urdf.xacro        # Main differential drive robot
│   └── robosort_arm.urdf.xacro    # 5-DOF robotic arm
├── launch/
│   ├── view_robot.launch.py       # RViz visualization only
│   ├── gazebo.launch.py           # Gazebo simulation only
│   └── simulation.launch.py       # Complete simulation + RViz + control
└── rviz/
    └── view_robot.rviz            # RViz configuration
```

## Usage

### 1. RViz Visualization (No Simulation)

View the robot model in RViz:

```bash
cd ~/robo-sort/source/rpi/ros2-robosort
source install/setup.bash
ros2 launch robosort_description view_robot.launch.py
```

This shows the robot's TF tree and links without physics simulation.

### 2. Gazebo Simulation

Launch the robot in Gazebo with physics:

```bash
ros2 launch robosort_description gazebo.launch.py
```

Optional arguments:
- `x_pose:=1.0` - Initial X position
- `y_pose:=2.0` - Initial Y position
- `z_pose:=0.1` - Initial Z position
- `world:=empty` - Gazebo world name

### 3. Complete Simulation System

Launch Gazebo + RViz + obstacle avoidance:

```bash
ros2 launch robosort_description simulation.launch.py
```

This starts:
- Gazebo physics simulation
- RViz visualization
- Obstacle avoidance node
- All necessary ROS2 nodes

### 4. Control the Robot

#### Manual Teleop

```bash
# Install teleop package if needed
sudo apt install ros-jazzy-teleop-twist-keyboard

# Run teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

#### Programmatic Control

Publish to `/cmd_vel`:

```bash
# Move forward
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Rotate in place
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"

# Stop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

## Features

### 1. Accurate Physics Simulation

- Proper inertial properties for all links
- Realistic wheel friction (μ = 1.0)
- Low-friction caster wheels (μ = 0.01)
- Wheel torque and acceleration limits

### 2. Gazebo Plugins

#### Differential Drive Controller
- Converts `/cmd_vel` commands to wheel velocities
- Publishes odometry to `/odom`
- Publishes TF transforms (odom → base_footprint)
- 50 Hz update rate

#### LiDAR Sensor
- Simulates LD06 laser scanner
- 360° coverage with 360 samples
- 12m max range
- Publishes to `/scan` topic
- Gaussian noise (σ = 0.01m)

#### Joint State Publisher
- Publishes wheel joint states
- Enables proper TF tree visualization

### 3. TF Frame Tree

```
odom
└── base_footprint
    └── base_link
        ├── left_wheel_link
        ├── right_wheel_link
        ├── front_left_caster_link
        ├── front_right_caster_link
        └── lidar_link
```

### 4. ROS2 Topic Interface

**Subscribed Topics:**
- `/cmd_vel` (geometry_msgs/Twist) - Velocity commands

**Published Topics:**
- `/odom` (nav_msgs/Odometry) - Robot odometry
- `/scan` (sensor_msgs/LaserScan) - LiDAR data
- `/joint_states` (sensor_msgs/JointState) - Wheel positions
- `/tf` - Transform tree

## Integration with Hardware

### Using with Real Robot

When running on actual hardware (not simulation):

1. **Launch hardware nodes:**
```bash
ros2 launch robosort_control control.launch.py
```

2. **Launch visualization only:**
```bash
ros2 launch robosort_description view_robot.launch.py use_sim_time:=false
```

3. **Hardware publishes:**
   - LiDAR data via `ldlidar_stl_ros2` package
   - TF transforms via `tf_broadcaster` node
   - Motor control via `motor_controller` node

### Switching Between Simulation and Hardware

Use the `use_sim_time` parameter:
- `use_sim_time:=true` - Gazebo simulation mode
- `use_sim_time:=false` - Real hardware mode (default for view_robot.launch.py)

## Customization

### Adjusting Robot Dimensions

Edit [urdf/robosort.urdf.xacro](urdf/robosort.urdf.xacro) properties:

```xml
<!-- Body dimensions -->
<xacro:property name="body_height" value="0.1016"/>
<xacro:property name="body_length" value="0.889"/>
<xacro:property name="body_width" value="0.508"/>

<!-- Wheel properties -->
<xacro:property name="wheel_radius" value="0.1524"/>
<xacro:property name="wheel_separation" value="0.558"/>
```

### Adding Sensors

Add new sensor links and Gazebo plugins in the URDF:

```xml
<link name="camera_link">
  <!-- Visual and collision geometry -->
</link>

<joint name="camera_joint" type="fixed">
  <parent link="base_link"/>
  <child link="camera_link"/>
  <origin xyz="0.3 0 0.1" rpy="0 0 0"/>
</joint>

<gazebo reference="camera_link">
  <sensor name="camera" type="camera">
    <!-- Camera configuration -->
  </sensor>
</gazebo>
```

### Tuning Differential Drive

Edit plugin parameters in URDF:

```xml
<max_wheel_torque>10.0</max_wheel_torque>
<max_wheel_acceleration>1.0</max_wheel_acceleration>
<wheel_separation>${wheel_separation}</wheel_separation>
<wheel_diameter>${2*wheel_radius}</wheel_diameter>
```

## Troubleshooting

### Robot Falls Through Ground
Increase collision kp/kd values for wheels or reduce simulation time step.

### Robot Doesn't Move
- Check `/cmd_vel` topic is publishing: `ros2 topic echo /cmd_vel`
- Verify Gazebo plugin loaded: Check Gazebo console output
- Ensure wheel joints are not fixed

### LiDAR Not Visible in RViz
- Add LaserScan display in RViz
- Set topic to `/scan`
- Check Fixed Frame is `odom` or `base_link`

### TF Errors
- For simulation: Set `use_sim_time:=true`
- For hardware: Set `use_sim_time:=false`
- Check all nodes use consistent time source

## Dependencies

### Required Packages

```bash
# Gazebo simulation
sudo apt install ros-jazzy-gazebo-ros-pkgs

# Robot description tools
sudo apt install ros-jazzy-xacro
sudo apt install ros-jazzy-robot-state-publisher
sudo apt install ros-jazzy-joint-state-publisher

# Visualization
sudo apt install ros-jazzy-rviz2

# Teleoperation
sudo apt install ros-jazzy-teleop-twist-keyboard
```

### RoboSort Packages
- `robosort_control` - Motor controller, obstacle avoidance, TF broadcaster
- `robosort_interfaces` - Custom service definitions
- `ldlidar_stl_ros2` - LD06 LiDAR driver (hardware only)

## Building

```bash
cd ~/robo-sort/source/rpi/ros2-robosort
colcon build --packages-select robosort_description
source install/setup.bash
```

## Testing

### Verify URDF is Valid

```bash
check_urdf install/robosort_description/share/robosort_description/urdf/robosort.urdf.xacro
```

### View TF Tree

```bash
ros2 run tf2_tools view_frames
```

Generates `frames.pdf` showing the complete transform tree.

### Monitor Topics

```bash
# List all topics
ros2 topic list

# Echo cmd_vel commands
ros2 topic echo /cmd_vel

# Echo odometry
ros2 topic echo /odom

# Echo LiDAR scans
ros2 topic echo /scan
```

## Next Steps

1. **Navigation**: Add Nav2 for autonomous navigation
2. **Mapping**: Use SLAM Toolbox with LiDAR for map creation
3. **Path Planning**: Integrate with Move Base for goal-based navigation
4. **Arm Integration**: Combine with robosort_arm.urdf.xacro for manipulation tasks
5. **Camera**: Add camera sensor for vision processing

## License

MIT License - See repository LICENSE file
