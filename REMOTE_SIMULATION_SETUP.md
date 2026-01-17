# RoboSort Remote Simulation Setup
# Option 3: Gazebo on Remote Machine, Robot Control on Raspberry Pi

## Overview
- **Remote Machine (Powerful Computer)**: Runs Gazebo simulation
- **Raspberry Pi**: Runs robot control code
- **Communication**: ROS2 network between machines

## Step 1: Network Setup

### Find IP Addresses
On both machines, run:
```bash
ip addr show
```
Look for your local network IP (usually `192.168.x.x` or `10.0.x.x`)

### Set ROS2 Network Environment Variables

**On Remote Machine (Gazebo):**
```bash
export ROS_DOMAIN_ID=42  # Same on both machines
export ROS_LOCALHOST_ONLY=0  # Allow network communication
export ROS_DISCOVERY_SERVER=  # Use multicast discovery
```

**On Raspberry Pi (Robot):**
```bash
export ROS_DOMAIN_ID=42  # Same as remote machine
export ROS_LOCALHOST_ONLY=0  # Allow network communication
export ROS_DISCOVERY_SERVER=  # Use multicast discovery
```

### Optional: Explicit IP Configuration
If multicast doesn't work, specify IPs explicitly:

**On Remote Machine:**
```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export ROS_IP=192.168.1.100  # Remote machine IP
```

**On Raspberry Pi:**
```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export ROS_IP=192.168.1.200  # Raspberry Pi IP
```

## Step 2: Remote Machine Setup (Gazebo)

### Install ROS2 Jazzy and Gazebo
```bash
# Install ROS2 Jazzy
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
sudo echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt upgrade
sudo apt install ros-jazzy-desktop

# Install Gazebo
sudo apt install ros-jazzy-gazebo-ros-pkgs ros-jazzy-gazebo-plugins
sudo apt install ros-jazzy-teleop-twist-keyboard ros-jazzy-rviz2
```

### Clone and Build RoboSort Workspace
```bash
cd ~
git clone https://github.com/qppd/robo-sort.git
cd robo-sort/source/rpi/ros2-robosort

# Source ROS2
source /opt/ros/jazzy/setup.bash

# Build workspace
colcon build --packages-select robosort_description
source install/setup.bash
```

### Launch Gazebo Simulation
```bash
# Set network environment
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0

# Launch simulation
ros2 launch robosort_description launch_sim.launch.py
```

## Step 3: Raspberry Pi Setup (Robot Control)

### Install ROS2 Jazzy (Minimal)
```bash
# On Raspberry Pi, install minimal ROS2
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
sudo echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-jazzy-ros-base

# Install teleop tools
sudo apt install ros-jazzy-teleop-twist-keyboard
```

### Clone RoboSort Code
```bash
cd ~
git clone https://github.com/qppd/robo-sort.git
cd robo-sort/source/rpi/ros2-robosort

# Source ROS2
source /opt/ros/jazzy/setup.bash

# Build your control packages
colcon build --packages-select robosort_control robosort_interfaces
source install/setup.bash
```

### Test Network Communication
```bash
# Set network environment
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0

# Check if you can see topics from remote machine
ros2 topic list

# You should see topics like:
/scan
/odom
/cmd_vel
/robot_description
```

### Control the Simulated Robot
```bash
# Set network environment
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0

# Launch teleop control
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Step 4: Testing the Setup

### 1. Start Gazebo on Remote Machine
```bash
# Remote machine
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
ros2 launch robosort_description launch_sim.launch.py
```

### 2. Test Communication from Pi
```bash
# Raspberry Pi
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0

# Check topics
ros2 topic list

# View lidar data
ros2 topic echo /scan --once

# View odometry
ros2 topic echo /odom --once
```

### 3. Control Robot from Pi
```bash
# Raspberry Pi
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0

# Launch teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Step 5: Integration with Your Robot Code

### Modify Your Robot Control Code
When you want to run your actual robot control code on the Pi but test with simulation:

**In your robot control launch file, add conditions:**
```python
# In your launch file
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

# Add launch argument
declare_simulation = DeclareLaunchArgument(
    'simulation',
    default_value='true',  # Set to 'false' for real robot
    description='Run in simulation mode'
)

# Conditional nodes
real_robot_node = Node(
    package='robosort_control',
    executable='real_robot_driver',
    condition=UnlessCondition(LaunchConfiguration('simulation'))
)

simulated_robot_node = Node(
    package='robosort_control',
    executable='simulated_robot_driver',
    condition=IfCondition(LaunchConfiguration('simulation'))
)
```

### Launch Commands

**For Simulation Testing:**
```bash
# Remote machine: Gazebo
ros2 launch robosort_description launch_sim.launch.py

# Raspberry Pi: Your control code in simulation mode
ros2 launch robosort_control your_launch_file.launch.py simulation:=true
```

**For Real Robot:**
```bash
# Raspberry Pi: Your control code for real hardware
ros2 launch robosort_control your_launch_file.launch.py simulation:=false
```

## Troubleshooting

### Can't See Topics Between Machines
1. **Check firewall:** `sudo ufw status` (disable if blocking)
2. **Check ROS_DOMAIN_ID:** Must be same on both machines
3. **Check ROS_LOCALHOST_ONLY:** Must be 0 on both
4. **Check network:** Both machines on same subnet
5. **Test ping:** `ping <remote_ip>`

### Gazebo Performance Issues
- Close unnecessary applications on remote machine
- Use simpler world: `world:=robosort_simple.world`
- Reduce RViz complexity

### Network Latency
- Use wired Ethernet instead of WiFi
- Keep machines physically close
- Monitor with: `ros2 topic hz /scan`

### ROS2 Discovery Issues
- Try explicit IP configuration instead of multicast
- Restart ROS2 daemon: `ros2 daemon stop && ros2 daemon start`
- Check ROS2 version compatibility

## Network Configuration Script

Create this script on both machines for easy setup:

**network_setup.sh:**
```bash
#!/bin/bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
# export ROS_IP=192.168.1.xxx  # Uncomment and set if needed

echo "ROS2 Network Configured:"
echo "  ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "  ROS_LOCALHOST_ONLY: $ROS_LOCALHOST_ONLY"
echo "  ROS_IP: ${ROS_IP:-auto}"
```

Then run: `source network_setup.sh`

## Next Steps

1. **Test basic communication** between machines
2. **Verify teleop control** works from Pi to Gazebo
3. **Integrate your control algorithms** with simulation
4. **Test navigation and obstacle avoidance**
5. **Deploy to real robot** when ready

This setup gives you the best of both worlds: powerful simulation for testing, and your actual robot control code running on the target hardware.