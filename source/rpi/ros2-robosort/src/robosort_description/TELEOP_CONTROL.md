# RoboSort Teleop Keyboard Control

## Overview
RoboSort now includes keyboard teleop control for manual driving in RViz2 and Gazebo simulations.

## Usage

### 1. RViz Visualization (Standalone)
```bash
ros2 launch robosort_description view_robot.launch.py
```
This launches:
- Robot model in RViz
- Joint state publisher
- **Teleop keyboard control** (in separate xterm window)

### 2. Gazebo Simulation
```bash
ros2 launch robosort_description simulation.launch.py
```
This launches:
- Gazebo with robot model
- RViz visualization
- Obstacle avoidance
- **Teleop keyboard control** (in separate xterm window)

### 3. Hardware Mode with Visualization
```bash
ros2 launch robosort_control robosort.launch.py \
    arduino_port:=/dev/ttyACM0 \
    lidar_port:=/dev/ttyUSB0 \
    use_rviz:=true \
    use_teleop:=true
```
This launches:
- All hardware nodes (Arduino, LiDAR)
- RViz visualization
- **Teleop keyboard control** (in separate xterm window)

## Keyboard Controls

The teleop window will display these controls:

```
Reading from the keyboard and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
```

### Key Mappings:
- **i** = Forward
- **,** = Backward  
- **j** = Turn left
- **l** = Turn right
- **u** = Forward + left
- **o** = Forward + right
- **m** = Backward + left
- **.** = Backward + right
- **k** or **space** = Stop

### Speed Control:
- **q/z** = Increase/decrease all speeds
- **w/x** = Increase/decrease linear speed only
- **e/c** = Increase/decrease angular speed only

## Notes

### xterm Requirement
The teleop node uses `xterm` as the terminal prefix. Install if missing:
```bash
# Ubuntu/Debian
sudo apt install xterm

# Alternative: Remove the prefix if you want it in the same terminal
# Edit the launch file and remove: prefix='xterm -e'
```

### Gazebo Simulation
- The differential drive plugin in Gazebo automatically handles `/cmd_vel` commands
- Published odometry on `/odom` topic
- Publishes TF transforms for `odom -> base_footprint`

### Hardware Mode
- Teleop commands go to `motor_controller` node
- Arduino translates to differential drive motor commands
- Obstacle avoidance can override commands if obstacles detected

## Troubleshooting

### Teleop window doesn't open
- Check if `xterm` is installed: `which xterm`
- Or remove the `prefix='xterm -e'` from launch file to run in same terminal

### Robot doesn't move in Gazebo
- Verify `/cmd_vel` topic is publishing: `ros2 topic echo /cmd_vel`
- Check Gazebo is not paused (bottom toolbar)
- Verify differential drive plugin loaded: check Gazebo terminal output

### Robot doesn't move in hardware mode
- Verify Arduino is connected and responding
- Check motor controller status: `ros2 topic echo /robosort/motor_status`
- Test manually: `ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"`

## Architecture

```
┌─────────────────┐
│ teleop_twist    │
│   _keyboard     │
└────────┬────────┘
         │ /cmd_vel
         ▼
┌────────────────────┐         ┌─────────────────┐
│  Gazebo Diff Drive │   OR    │ motor_controller│
│      Plugin        │         │   (Hardware)    │
└────────────────────┘         └─────────────────┘
```

