# LiDAR Odometry Setup for RoboSort

## Overview
RoboSort now uses `rf2o_laser_odometry` for accurate odometry estimation from lidar scan matching instead of dead-reckoning or fake odometry. This significantly reduces drift during navigation.

## What Changed

### 1. Replaced Dead-Reckoning with Lidar Odometry
- **Before**: `tf_broadcaster` computed odometry from `cmd_vel` commands (dead-reckoning)
- **After**: `rf2o_laser_odometry` computes odometry by matching consecutive laser scans (ICP)

### 2. TF Tree Structure (Unchanged)
```
map
 └─ odom (from slam_toolbox)
     └─ base_footprint (from rf2o_laser_odometry)
         ├─ lidar_link (from lidar_tf_publisher)
         └─ base_link (from robot_state_publisher)
             ├─ left_wheel_link
             ├─ right_wheel_link
             ├─ front_left_caster_link
             └─ right_caster_link
```

### 3. Key Benefits
- **Reduced Drift**: Lidar scan matching is much more accurate than dead-reckoning
- **No Wheel Encoders Needed**: Works without encoder feedback
- **Better Loop Closure**: Improved odometry helps slam_toolbox close loops more accurately
- **Nav2 Compatible**: Maintains proper TF tree for Nav2 navigation

## Installation

### Install rf2o_laser_odometry Package

```bash
# For ROS 2 Humble/Jazzy
sudo apt update
sudo apt install ros-$ROS_DISTRO-rf2o-laser-odometry
```

If the package is not available in your ROS distribution, build from source:

```bash
cd ~/robo-sort/source/rpi/ros2-robosort/src
git clone https://github.com/MAPIRlab/rf2o_laser_odometry.git
cd ~/robo-sort/source/rpi/ros2-robosort
colcon build --packages-select rf2o_laser_odometry
source install/setup.bash
```

## Configuration

### RF2O Parameters (`config/rf2o_params.yaml`)
Key parameters you can tune:

- **Frequency**: `freq: 20.0` - How often odometry is computed (Hz)
- **Max Correspondence Distance**: `max_correspondence_dist: 0.3` - Maximum distance for point matching
- **Covariance**: `sigma_scan_xy` and `sigma_scan_angle` - Uncertainty estimates for Nav2

### SLAM Toolbox Parameters (`config/mapper_params_online_async.yaml`)
Updated for better integration:

- **Minimum Travel**: Reduced from 0.5 to 0.2m for more frequent map updates
- **Loop Closure**: Enabled with tuned parameters for rf2o odometry

## Usage

### Build and Source
```bash
cd ~/robo-sort/source/rpi/ros2-robosort
colcon build --packages-select robosort_control
source install/setup.bash
```

### Launch (Same commands as before)

**Terminal 1: SLAM Toolbox**
```bash
ros2 launch slam_toolbox online_async_launch.py \
    params_file:=$HOME/robo-sort/source/rpi/ros2-robosort/config/mapper_params_online_async.yaml
```

**Terminal 2: RoboSort with Navigation**
```bash
ros2 launch robosort_control robosort.launch.py \
    arduino_port:=/dev/ttyUSB0 \
    lidar_port:=/dev/ttyUSB1 \
    use_rviz:=true \
    use_teleop:=true \
    use_nav2:=true
```

## Verification

### Check TF Tree
```bash
ros2 run tf2_tools view_frames
# Opens a PDF showing your TF tree - verify odom->base_footprint is from rf2o
```

### Monitor Odometry
```bash
# Watch odometry messages
ros2 topic echo /odom

# Check odometry publishing rate
ros2 topic hz /odom
# Should be ~20 Hz
```

### Verify RF2O is Running
```bash
ros2 node list | grep rf2o
# Should show: /rf2o_laser_odometry

ros2 node info /rf2o_laser_odometry
# Shows subscriptions and publications
```

## Troubleshooting

### RF2O Not Publishing Odometry
**Symptoms**: No `/odom` topic or TF warnings
**Solutions**:
1. Check lidar is publishing: `ros2 topic hz /scan`
2. Verify frame IDs match: `ros2 topic echo /scan --field header.frame_id`
3. Check rf2o logs: `ros2 node list` and look for errors

### High Drift Still Present
**Symptoms**: Robot position drifts during movement
**Solutions**:
1. Increase scan matching frequency: Edit `rf2o_params.yaml`, set `freq: 30.0`
2. Reduce max correspondence distance: Set `max_correspondence_dist: 0.2`
3. Ensure lidar is mounted rigidly (no vibration)
4. Check lidar quality: `ros2 topic echo /scan` - look for consistent readings

### Nav2 Not Working
**Symptoms**: Nav2 fails to plan or execute paths
**Solutions**:
1. Verify TF tree: `ros2 run tf2_ros tf2_echo map base_footprint`
2. Check clock synchronization: All nodes should use same time source
3. Verify costmaps are updating: `ros2 topic hz /local_costmap/costmap`

### Slow Performance
**Symptoms**: High CPU usage or lag
**Solutions**:
1. Reduce rf2o frequency: `freq: 10.0` in `rf2o_params.yaml`
2. Increase max_correspondence_dist: `max_correspondence_dist: 0.5`
3. Reduce max_iterations: `max_iterations: 3`

## Technical Details

### How RF2O Works
1. **Scan Acquisition**: Receives laser scans from `/scan` topic
2. **Point Cloud Generation**: Converts scan to point cloud in `lidar_link` frame
3. **ICP Matching**: Matches current scan with previous scan using Iterative Closest Point
4. **Odometry Computation**: Calculates robot motion from scan displacement
5. **TF Publishing**: Publishes `odom->base_footprint` transform
6. **Odometry Publishing**: Publishes `/odom` nav_msgs/Odometry message

### Integration with SLAM Toolbox
- RF2O provides `odom->base_footprint` (short-term accurate odometry)
- SLAM Toolbox provides `map->odom` (long-term drift correction)
- Together they maintain `map->odom->base_footprint` chain
- SLAM uses odometry to predict robot motion, then corrects with loop closures

## Performance Tuning

### For Faster Movement
```yaml
# rf2o_params.yaml
freq: 30.0                      # Higher frequency
max_iterations: 3               # Fewer ICP iterations
max_correspondence_dist: 0.4    # Larger matching tolerance
```

### For Higher Accuracy (Slower Movement)
```yaml
# rf2o_params.yaml
freq: 20.0                      # Standard frequency
max_iterations: 8               # More ICP iterations
max_correspondence_dist: 0.2    # Stricter matching
epsilon_xy: 0.00001            # Tighter convergence
```

### For Noisy Environments
```yaml
# rf2o_params.yaml
sigma_scan_xy: 0.1             # Higher uncertainty
sigma_scan_angle: 0.1          # Higher angular uncertainty
max_correspondence_dist: 0.4    # More tolerant matching
```

## References
- [RF2O GitHub Repository](https://github.com/MAPIRlab/rf2o_laser_odometry)
- [RF2O Paper](http://mapir.isa.uma.es/mapirwebsite/index.php/mapir-downloads/papers/217)
- [SLAM Toolbox Documentation](https://github.com/SteveMacenski/slam_toolbox)
