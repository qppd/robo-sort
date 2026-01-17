# RoboSort URDF Testing Checklist

Use this checklist to verify the URDF model is working correctly.

## Pre-Testing Setup

- [ ] Built the package successfully
  ```bash
  cd ~/robo-sort/source/rpi/ros2-robosort
  colcon build --packages-select robosort_description
  source install/setup.bash
  ```

- [ ] Installed all dependencies
  ```bash
  sudo apt install ros-jazzy-gazebo-ros-pkgs \
                   ros-jazzy-xacro \
                   ros-jazzy-robot-state-publisher \
                   ros-jazzy-teleop-twist-keyboard
  ```

## Part 1: URDF Validation

### 1.1 Check URDF Syntax
```bash
cd ~/robo-sort/source/rpi/ros2-robosort/install/robosort_description/share/robosort_description/urdf
check_urdf robosort.urdf.xacro
```
**Expected**: "robot name is: robosort" with no errors

- [ ] URDF passes syntax check
- [ ] All links are defined
- [ ] All joints are valid

### 1.2 Generate URDF from XACRO
```bash
xacro robosort.urdf.xacro > /tmp/robosort.urdf
```
- [ ] XACRO processes without errors
- [ ] Generated URDF file created

### 1.3 Visualize URDF Structure
```bash
urdf_to_graphiz /tmp/robosort.urdf
```
- [ ] PDF generated showing link hierarchy
- [ ] All expected links present (base, wheels, casters, lidar)

## Part 2: RViz Visualization

### 2.1 Launch RViz
```bash
ros2 launch robosort_description view_robot.launch.py
```

**Check in RViz:**
- [ ] Robot appears in viewport
- [ ] Robot is oriented correctly (X forward, Z up)
- [ ] All links visible (body, wheels, casters, lidar)
- [ ] Colors render properly
- [ ] No warning messages in terminal

### 2.2 Verify TF Tree
```bash
# In new terminal
ros2 run tf2_tools view_frames
evince frames.pdf
```
- [ ] TF tree shows complete hierarchy
- [ ] odom → base_footprint → base_link chain exists
- [ ] All wheel and sensor frames present
- [ ] No disconnected frames

### 2.3 Check RViz Displays
In RViz interface:
- [ ] RobotModel display shows robot
- [ ] TF display shows all frames
- [ ] Axes are correct size and orientation
- [ ] Can rotate view smoothly

## Part 3: Gazebo Simulation

### 3.1 Launch Gazebo
```bash
ros2 launch robosort_description gazebo.launch.py
```

**Check in Gazebo:**
- [ ] Gazebo opens without errors
- [ ] Robot spawns successfully
- [ ] Robot is above ground (not sinking)
- [ ] Wheels touch ground properly
- [ ] LiDAR sensor visible
- [ ] Physics is paused initially

### 3.2 Test Physics
1. Click "Play" button in Gazebo
- [ ] Robot stays stable (doesn't fall over)
- [ ] Wheels don't sink into ground
- [ ] Casters support robot correctly
- [ ] No excessive vibration

### 3.3 Verify Gazebo Plugins

**Check plugins loaded:**
```bash
# In new terminal
ros2 topic list
```
- [ ] `/cmd_vel` topic exists (diff drive plugin)
- [ ] `/odom` topic exists (odometry plugin)
- [ ] `/scan` topic exists (lidar plugin)
- [ ] `/joint_states` topic exists (joint state plugin)
- [ ] `/tf` topic exists (transform plugin)

### 3.4 Test LiDAR Sensor
```bash
ros2 topic echo /scan --once
```
- [ ] LaserScan messages are published
- [ ] 360 range readings present
- [ ] Range values are reasonable (0.12m - 12m)
- [ ] LiDAR visual appears in Gazebo

## Part 4: Robot Control

### 4.1 Manual Command Test
```bash
# Send forward command
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}" --once
```
- [ ] Robot moves forward in Gazebo
- [ ] Movement is smooth
- [ ] Wheels rotate correctly
- [ ] Speed seems appropriate

```bash
# Send rotation command
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}" --once
```
- [ ] Robot rotates in place
- [ ] Both wheels turn (opposite directions)
- [ ] Rotation is centered

```bash
# Stop robot
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" --once
```
- [ ] Robot stops moving
- [ ] No drift or sliding

### 4.2 Keyboard Teleop
```bash
# New terminal
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
- [ ] Can control robot with keyboard
- [ ] 'i' moves forward
- [ ] 'k' stops
- [ ] ',' moves backward
- [ ] 'j'/'l' turn left/right
- [ ] Speed adjustable with 'q'/'z'

### 4.3 Verify Odometry
```bash
# Echo odometry while moving robot
ros2 topic echo /odom
```
- [ ] Odometry messages published
- [ ] Position updates when robot moves
- [ ] Orientation changes when rotating
- [ ] Velocities match commands

## Part 5: Complete System Test

### 5.1 Launch Full Simulation
```bash
ros2 launch robosort_description simulation.launch.py
```
- [ ] Gazebo launches
- [ ] RViz launches
- [ ] Obstacle avoidance node starts
- [ ] No error messages
- [ ] Robot visible in both Gazebo and RViz

### 5.2 Test in RViz
**In RViz:**
- [ ] Add LaserScan display (topic: /scan)
- [ ] LiDAR data visible as points
- [ ] Add Odometry display (topic: /odom)
- [ ] Odometry arrow shows robot pose
- [ ] TF frames update in real-time

### 5.3 Test Obstacle Avoidance
1. Place object in front of robot in Gazebo (Insert → Box)
2. Send forward command:
   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}" -r 10
   ```
- [ ] LiDAR detects obstacle
- [ ] Obstacle visible in RViz LaserScan
- [ ] Robot slows down approaching obstacle
- [ ] Robot stops before collision
- [ ] Terminal shows obstacle detection messages

## Part 6: Performance Tests

### 6.1 Check Update Rates
```bash
# Check cmd_vel rate
ros2 topic hz /cmd_vel

# Check odom rate
ros2 topic hz /odom

# Check scan rate
ros2 topic hz /scan
```
- [ ] `/odom` at ~50 Hz
- [ ] `/scan` at ~10 Hz
- [ ] All rates stable

### 6.2 Monitor Node Health
```bash
ros2 node list
ros2 node info /robot_state_publisher
```
- [ ] All expected nodes running
- [ ] No zombie processes
- [ ] Memory usage reasonable

### 6.3 Check TF Performance
```bash
ros2 run tf2_ros tf2_echo odom base_footprint
```
- [ ] Transform updates continuously
- [ ] No lag or delays
- [ ] Translation and rotation data present

## Part 7: Integration Tests

### 7.1 Test with robosort_control Package
```bash
# Launch obstacle avoidance
ros2 run robosort_control obstacle_avoidance
```
- [ ] Node starts without errors
- [ ] Subscribes to /scan
- [ ] Publishes to /cmd_vel when obstacle detected

### 7.2 Parameter Configuration
```bash
# List parameters
ros2 param list /robot_state_publisher
```
- [ ] `use_sim_time` parameter exists
- [ ] `robot_description` parameter set
- [ ] Parameters are configurable

## Troubleshooting Results

### Issues Found
1. _______________________________________________
2. _______________________________________________
3. _______________________________________________

### Fixes Applied
1. _______________________________________________
2. _______________________________________________
3. _______________________________________________

## Final Verification

### Simulation Quality
- [ ] Robot behaves realistically
- [ ] No physics glitches
- [ ] Stable at all speeds
- [ ] Collisions work properly

### Control Responsiveness
- [ ] Commands execute immediately
- [ ] No input lag
- [ ] Smooth acceleration/deceleration
- [ ] Accurate turning radius

### Sensor Accuracy
- [ ] LiDAR detects obstacles
- [ ] Range measurements accurate
- [ ] Odometry tracks movement
- [ ] TF transforms correct

### Documentation Accuracy
- [ ] README instructions work
- [ ] QUICKSTART guide accurate
- [ ] All example commands valid
- [ ] File paths correct

## Sign-Off

**Tester**: _______________
**Date**: _______________
**Status**: [ ] PASS  [ ] FAIL  [ ] PARTIAL

**Notes**:
_____________________________________________________
_____________________________________________________
_____________________________________________________

## Next Steps After Testing

If all tests pass:
1. [ ] Tag this version in git
2. [ ] Document any parameter tweaks needed
3. [ ] Begin Nav2 integration
4. [ ] Test on real hardware

If tests fail:
1. [ ] Document failures in detail
2. [ ] Check Gazebo/ROS2 versions
3. [ ] Review URDF syntax
4. [ ] Test individual components
5. [ ] Consult troubleshooting section in README
