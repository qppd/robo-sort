# Fake Odometry Publisher for Nav2

This package provides a fake odometry publisher that generates odometry data from `/cmd_vel` commands. This is useful for testing Nav2 without actual robot hardware or encoders.

## Features

- **Subscribes to**: `/cmd_vel` (geometry_msgs/Twist)
- **Publishes**:
  - `/odom` (nav_msgs/Odometry) at 50 Hz
  - TF transform: `odom` → `base_link`
  
## Robot Specifications

- **Wheel Diameter**: 12 inches (0.3048 meters)
- **Wheel Base**: 0.2 meters (adjustable in code)
- **Update Rate**: 50 Hz

## Building

```bash
cd ~/ros2-robosort
colcon build --packages-select robosort_sensors
source install/setup.bash
```

## Running

### Option 1: Using Launch File
```bash
ros2 launch robosort_sensors fake_odom_launch.py
```

### Option 2: Running Node Directly
```bash
ros2 run robosort_sensors fake_odom
```

## Verifying Topics

Check that the required Nav2 topics are being published:

```bash
# List all topics
ros2 topic list

# Check /odom topic
ros2 topic echo /odom

# Check TF transforms
ros2 run tf2_ros tf2_echo odom base_link
```

You should see:
- `/cmd_vel` - Command velocity input
- `/odom` - Odometry output
- `/tf` - Transform data

## Integration with Nav2

This fake odometry satisfies Nav2's requirements:
- ✅ `/cmd_vel` - Subscribed (input)
- ✅ `/odom` - Published
- ✅ `/scan` - Need LiDAR (from ldlidar_stl_ros2)
- ✅ `/tf` - Published (odom→base_link)
- ✅ `/tf_static` - Need robot URDF/static transforms

## Customization

Edit [fake_odom.py](robosort_sensors/fake_odom.py) to adjust:
- `wheel_diameter` - Change wheel size
- `wheel_base` - Distance between wheels
- Timer frequency (currently 50 Hz / 0.02s)
- Covariance values

## Notes

- This is a **fake odometry** based on commanded velocity, not actual wheel encoders
- Position drift will occur over time (no feedback from actual movement)
- For production use, replace with real odometry from encoders
- The odometry integrates velocity commands over time to estimate position
