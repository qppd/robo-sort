#!/bin/bash

# RoboSort Network Test Script
# Run this on the Raspberry Pi to test connection to remote Gazebo machine

echo "=========================================="
echo "RoboSort Network Connection Test"
echo "=========================================="

# Check ROS2 environment
echo "ROS2 Environment:"
echo "  ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-not set}"
echo "  ROS_LOCALHOST_ONLY: ${ROS_LOCALHOST_ONLY:-not set}"
echo "  ROS_IP: ${ROS_IP:-auto}"
echo ""

# Test topic discovery
echo "Testing topic discovery..."
TOPICS=$(timeout 5 ros2 topic list 2>/dev/null | wc -l)

if [ "$TOPICS" -gt 0 ]; then
    echo "✅ Found $TOPICS topics"
    echo ""
    echo "Available topics:"
    ros2 topic list | head -10
    if [ "$(ros2 topic list | wc -l)" -gt 10 ]; then
        echo "... and $(($(ros2 topic list | wc -l) - 10)) more"
    fi
else
    echo "❌ No topics found. Check network configuration."
    echo ""
    echo "Troubleshooting:"
    echo "1. Make sure Gazebo is running on remote machine"
    echo "2. Check ROS_DOMAIN_ID matches (both should be 42)"
    echo "3. Check ROS_LOCALHOST_ONLY is 0 on both machines"
    echo "4. Verify both machines are on same network"
    echo "5. Try: ping <remote_machine_ip>"
    exit 1
fi

echo ""
echo "Testing key topics..."

# Test /scan topic
echo -n "  /scan (LiDAR): "
if ros2 topic info /scan >/dev/null 2>&1; then
    echo "✅ Available"
    SCAN_HZ=$(timeout 3 ros2 topic hz /scan 2>/dev/null | grep -o "[0-9.]* Hz" | head -1)
    if [ ! -z "$SCAN_HZ" ]; then
        echo "    Publishing at: $SCAN_HZ"
    fi
else
    echo "❌ Not available"
fi

# Test /odom topic
echo -n "  /odom (Odometry): "
if ros2 topic info /odom >/dev/null 2>&1; then
    echo "✅ Available"
else
    echo "❌ Not available"
fi

# Test /cmd_vel topic
echo -n "  /cmd_vel (Commands): "
if ros2 topic info /cmd_vel >/dev/null 2>&1; then
    echo "✅ Available"
else
    echo "❌ Not available"
fi

echo ""
echo "=========================================="

# Test teleop if available
if command -v ros2 run teleop_twist_keyboard teleop_twist_keyboard >/dev/null 2>&1; then
    echo "Ready to control robot!"
    echo "Run: ros2 run teleop_twist_keyboard teleop_twist_keyboard"
    echo ""
    echo "Controls:"
    echo "  i - forward"
    echo "  k - stop"
    echo "  j - turn left"
    echo "  l - turn right"
    echo "  q/z - increase/decrease speed"
else
    echo "Install teleop: sudo apt install ros-jazzy-teleop-twist-keyboard"
fi

echo "=========================================="