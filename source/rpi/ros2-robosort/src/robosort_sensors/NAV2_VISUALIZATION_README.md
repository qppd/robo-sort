# RoboSort Nav2 Visualization

Complete visualization setup for robot navigation with RViz and Nav2.

## What It Launches

1. **Robot Description** - Publishes robot URDF and TF tree
2. **SLAM Toolbox** - Creates live occupancy map from LiDAR
3. **Nav2 Stack** - Navigation and path planning
4. **RViz** - 3D visualization with navigation tools

## Launch Command

```bash
# Build first
colcon build --packages-select robosort_sensors robosort_description
source install/setup.bash

# Launch everything
ros2 launch robosort_sensors nav2_visualization.launch.py

# If your LiDAR is on ttyUSB1:
ros2 launch robosort_sensors nav2_visualization.launch.py lidar_serial_port:=/dev/ttyUSB1
```

## Prerequisites

Make sure you have running:
- **Odometry source** (publishing `/odom`)
- **Velocity commands** (subscribing to `/cmd_vel`)

Note: `nav2_visualization.launch.py` also launches the LiDAR driver for `/scan`.

## RViz Features

- **Robot Model**: Shows 3D robot visualization
- **Laser Scan**: LiDAR point cloud
- **Map**: Live SLAM occupancy grid
- **Navigation Tools**:
  - 2D Pose Estimate (set initial position)
  - 2D Nav Goal (set navigation target)
  - 2D Navigate (manual control)

## Topics Required

- `/odom` (nav_msgs/Odometry) - Robot odometry
- `/scan` (sensor_msgs/LaserScan) - LiDAR data
- `/cmd_vel` (geometry_msgs/Twist) - Velocity commands
- `/tf`, `/tf_static` - Transform tree

## Troubleshooting

### No Robot Showing
- Check `/robot_description` topic is published
- Verify TF tree: `ros2 run tf2_tools view_frames`

### No Map Appearing
- Check `/scan` topic is publishing
- SLAM needs movement to build map

### Navigation Not Working
- Ensure odometry is publishing accurate data
- Check Nav2 status: `ros2 lifecycle get /controller_server`

## Manual Launch (Alternative)

If you want to launch components separately:

```bash
# 1. Robot description
ros2 launch robosort_description display.launch.py

# 2. Your odometry source (whatever provides /odom)

# 3. LiDAR
ros2 launch robosort_sensors lidar.launch.py

# 4. Nav2 SLAM
ros2 launch robosort_sensors nav2_slam.launch.py

# 5. RViz
ros2 run rviz2 rviz2 -d src/robosort_sensors/config/slam_nav2.rviz
```</content>
<parameter name="filePath">c:\Users\sajed\Desktop\PROJECTS\robo-sort\source\rpi\ros2-robosort\src\robosort_sensors\NAV2_VISUALIZATION_README.md