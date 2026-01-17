# RoboSort Gazebo Simulation Guide

Based on the [Articulated Robotics Gazebo Tutorial](https://articulatedrobotics.xyz/tutorials/mobile-robot/concept-design/concept-gazebo)

This guide explains how to run the RoboSort robot in Gazebo simulation with lidar scanning and obstacles.

## Overview

The simulation includes:
- **Robot Model**: Full RoboSort differential drive robot with wheels, casters, and LiDAR sensor
- **LiDAR Simulation**: 360-degree laser scanner publishing to `/scan` topic
- **Differential Drive Control**: Gazebo plugin for `/cmd_vel` control
- **Odometry**: Position estimation published to `/odom`
- **World Files**: Pre-configured environments with obstacles
- **Visualization**: RViz2 for real-time visualization of robot state and sensor data

## Prerequisites

Make sure you have the following ROS2 packages installed:
```bash
sudo apt-get install ros-humble-gazebo-ros-pkgs
sudo apt-get install ros-humble-gazebo-plugins
sudo apt-get install ros-humble-robot-state-publisher
sudo apt-get install ros-humble-rviz2
sudo apt-get install ros-humble-teleop-twist-keyboard
```

## Building the Package

Navigate to your ROS2 workspace and build:
```bash
cd ~/ros2-robosort
colcon build --packages-select robosort_description
source install/setup.bash
```

## Running the Simulation (Step-by-Step Method)

Following the Articulated Robotics tutorial pattern:

### Step 1: Launch Robot State Publisher with Sim Time

First, start the robot state publisher which publishes the URDF to `/robot_description`:

```bash
ros2 launch robosort_description rsp.launch.py use_sim_time:=true
```

This publishes the full URDF to `/robot_description` topic.

### Step 2: Launch Gazebo

In a new terminal, launch Gazebo:
```bash
ros2 launch gazebo_ros gazebo.launch.py
```

This opens an empty Gazebo window.

### Step 3: Spawn the Robot

In another terminal, spawn the robot:
```bash
ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity robosort
```

You should see your robot appear in the Gazebo window!

---

## All-in-One Launch (Recommended)

Instead of running the three steps separately, use our combined launch file:

### Option 1: Warehouse World (Default)
```bash
ros2 launch robosort_description launch_sim.launch.py
```

### Option 2: Obstacle Course (Fun driving practice!)
```bash
ros2 launch robosort_description launch_sim.launch.py \
    world:=$(ros2 pkg prefix robosort_description)/share/robosort_description/worlds/robosort_obstacle_course.world
```

### Option 3: Simple World (Quick testing)
```bash
ros2 launch robosort_description launch_sim.launch.py \
    world:=$(ros2 pkg prefix robosort_description)/share/robosort_description/worlds/robosort_simple.world
```

### Launch Parameters

```bash
ros2 launch robosort_description launch_sim.launch.py \
    world:=/full/path/to/my.world \
    x_pose:=-6.0 \
    y_pose:=-6.0 \
    z_pose:=0.25 \
    use_rviz:=true
```

**Parameters:**
- `world`: Full path to world file (default: robosort_warehouse.world)
- `x_pose`: Initial X position of robot (default: 0.0)
- `y_pose`: Initial Y position of robot (default: 0.0)
- `z_pose`: Initial Z position of robot (default: 0.25)
- `use_rviz`: Start RViz2 for visualization (default: true)

## Controlling the Robot with teleop_twist_keyboard

This is the tool mentioned in the tutorial! It lets you drive the robot using your keyboard.

### Install (if not already):
```bash
sudo apt-get install ros-humble-teleop-twist-keyboard
```

### Run teleop:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Controls:
```
   u    i    o
   j    k    l
   m    ,    .

i - forward
, - backward
j - turn left
l - turn right
k - stop
u - forward + left
o - forward + right
m - backward + left
. - backward + right

q/z - increase/decrease max speeds
```

**IMPORTANT**: The terminal window running teleop must be active (in focus) for keyboard input to work! Keep it visible and click on it before pressing keys.

### Alternative: Using a Game Controller

For more practical control, you can use `teleop_twist_joy`:
```bash
sudo apt-get install ros-humble-teleop-twist-joy ros-humble-joy
ros2 launch teleop_twist_joy teleop-launch.py
```

## Understanding the Control System

When running the simulation, the Gazebo differential drive plugin:

1. **Subscribes to `/cmd_vel`** - Takes velocity commands (Twist messages)
2. **Controls the wheel joints** - Calculates motor commands for each wheel
3. **Publishes `/odom`** - Odometry (position estimate) from dead reckoning
4. **Broadcasts TF** - Transform from `odom` → `base_footprint`

For a differential drive robot, only two values in `/cmd_vel` matter:
- `linear.x` - Forward/backward speed
- `angular.z` - Rotation speed (turning)

## Visualizing in RViz

When RViz opens, you should see:

1. **Robot Model** - The 3D representation from URDF
2. **TF Frames** - Coordinate frames showing robot structure
3. **LaserScan** - Red dots showing LiDAR detections
4. **Odometry** - Arrows showing robot trajectory

### Key frames to observe:
- `odom` - The world origin (where robot started)
- `base_footprint` - Robot's ground projection
- `base_link` - Main robot body
- `lidar_link` - Where the laser scanner is

As you drive around in Gazebo, watch RViz to see the position update!

## Available World Files

### robosort_obstacle_course.world
A fun obstacle course with:
- Boundary walls (16x16m enclosed area)
- Slalom course (5 colored cylinders to weave through)
- Various box obstacles
- Narrow passage for skill testing
- Barrel obstacles
- Start (green) and finish (red) zone markers

**Challenge**: Start at the green marker, navigate through obstacles, reach the red marker!

### robosort_warehouse.world
A realistic warehouse with:
- 20x20m enclosed space with walls
- Large box obstacles
- Cylinder obstacles (barrels)
- Internal divider wall
- 3 sorting bins (red, green, blue)

### robosort_simple.world
Minimal testing environment:
- 2 box obstacles
- 1 cylinder obstacle
- Open ground plane

## Topics Published by Simulation

```bash
# List all topics
ros2 topic list
```

Expected topics:
- `/scan` - sensor_msgs/LaserScan (LiDAR data)
- `/odom` - nav_msgs/Odometry (position/velocity)
- `/cmd_vel` - geometry_msgs/Twist (velocity commands)
- `/joint_states` - sensor_msgs/JointState (wheel positions)
- `/tf` - Transform tree
- `/robot_description` - Robot URDF

### Check topics:
```bash
# View lidar data
ros2 topic echo /scan

# View odometry
ros2 topic echo /odom

# Check cmd_vel being sent
ros2 topic echo /cmd_vel

# Check scan frequency
ros2 topic hz /scan
```

## Gazebo Tags in URDF

The simulation works because of special `<gazebo>` tags in the URDF:

### 1. Material Colors
```xml
<gazebo reference="base_link">
    <material>Gazebo/Grey</material>
</gazebo>
```

### 2. Friction Settings (Critical for casters!)
```xml
<gazebo reference="front_left_caster_link">
    <material>Gazebo/Black</material>
    <mu1>0.01</mu1>
    <mu2>0.01</mu2>
</gazebo>
```
Low friction (0.01) on casters prevents erratic behavior!

### 3. Differential Drive Plugin
Located in `urdf/gazebo_control.xacro`:
```xml
<plugin name='diff_drive' filename='libgazebo_ros_diff_drive.so'>
    <left_joint>left_wheel_joint</left_joint>
    <right_joint>right_wheel_joint</right_joint>
    <wheel_separation>0.558</wheel_separation>
    <wheel_diameter>0.3048</wheel_diameter>
    <!-- ... more settings ... -->
</plugin>
```

### 4. LiDAR Sensor Plugin
```xml
<sensor name="lidar_sensor" type="ray">
    <!-- Publishes to /scan topic -->
</sensor>
```

## Troubleshooting

### Robot doesn't move:
1. Check teleop terminal is in focus (active window)
2. Verify cmd_vel is being published: `ros2 topic echo /cmd_vel`
3. Check Gazebo isn't paused (spacebar toggles pause)
4. Ensure differential drive plugin is loaded

### Robot moves erratically:
1. Check caster friction is low (mu1, mu2 = 0.01)
2. Verify wheel separation matches your robot
3. Check inertia values are sensible

### Robot falls through ground:
1. Increase z_pose to 0.3 or higher
2. Check collision geometries are defined

### No LiDAR data:
1. Check `/scan` topic exists: `ros2 topic list | grep scan`
2. Verify lidar plugin is loaded
3. Check frame_name matches your URDF

### Gazebo won't start:
```bash
# Kill zombie processes
killall -9 gzserver gzclient

# Retry
ros2 launch robosort_description launch_sim.launch.py
```

### Colors wrong in Gazebo:
- Make sure each link has a `<gazebo reference="link_name">` tag with a material

## File Structure

```
robosort_description/
├── urdf/
│   ├── robosort.urdf.xacro      # Main robot description
│   └── gazebo_control.xacro      # Gazebo plugins (separate file)
├── worlds/
│   ├── robosort_warehouse.world  # Default warehouse
│   ├── robosort_obstacle_course.world  # Fun driving course
│   └── robosort_simple.world     # Minimal testing
├── launch/
│   ├── rsp.launch.py             # Robot State Publisher only
│   ├── launch_sim.launch.py      # Complete simulation
│   └── gazebo.launch.py          # Basic Gazebo
├── rviz/
│   └── simulation.rviz           # RViz config
└── CMakeLists.txt                # Build configuration
```

## Next Steps

After getting comfortable with manual driving:

1. **Add SLAM** - Create maps while driving: `ros2 launch slam_toolbox online_async_launch.py`
2. **Add Navigation** - Autonomous navigation with Nav2
3. **Record Data** - `ros2 bag record /scan /odom /cmd_vel`
4. **Custom Worlds** - Create your own .world files
5. **Add Camera** - Simulate camera for vision tasks

## References

- [Articulated Robotics - Gazebo Tutorial](https://articulatedrobotics.xyz/tutorials/mobile-robot/concept-design/concept-gazebo)
- [Articulated Robotics YouTube](https://www.youtube.com/watch?v=IjFcr5r0nMs)
- [Gazebo ROS Packages](http://gazebosim.org/tutorials?tut=ros2_overview)
- [teleop_twist_keyboard](https://github.com/ros2/teleop_twist_keyboard)
