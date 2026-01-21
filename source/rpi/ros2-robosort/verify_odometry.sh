#!/bin/bash
# Odometry Verification Script
# Run this to diagnose odometry and TF issues

echo "=================================="
echo "RoboSort Odometry Diagnostics"
echo "=================================="
echo ""

# Check if ROS2 is running
if ! ros2 node list &> /dev/null; then
    echo "❌ ROS2 is not running or no nodes are active"
    exit 1
fi

echo "1. Checking Active Nodes..."
echo "----------------------------"
ros2 node list | grep -E "rf2o|ld06_lidar|robot_state"
echo ""

echo "2. Checking /scan Topic..."
echo "----------------------------"
if ros2 topic info /scan &> /dev/null; then
    echo "✓ /scan topic exists"
    ros2 topic hz /scan --window 10 &
    SCAN_PID=$!
    sleep 3
    kill $SCAN_PID 2>/dev/null
else
    echo "✗ /scan topic NOT publishing (LiDAR not running?)"
fi
echo ""

echo "3. Checking /odom Topic..."
echo "----------------------------"
if ros2 topic info /odom &> /dev/null; then
    echo "✓ /odom topic exists"
    echo "Publishers:"
    ros2 topic info /odom | grep "Publisher count"
    ros2 topic hz /odom --window 10 &
    ODOM_PID=$!
    sleep 3
    kill $ODOM_PID 2>/dev/null
else
    echo "✗ /odom topic NOT publishing"
fi
echo ""

echo "4. Checking TF Tree..."
echo "----------------------------"
if ros2 run tf2_tools view_frames &> /dev/null; then
    echo "✓ TF tree generated (see frames.pdf)"
else
    echo "Checking transforms manually..."
    echo "odom -> base_footprint:"
    ros2 run tf2_ros tf2_echo odom base_footprint | head -n 10 &
    TF_PID=$!
    sleep 2
    kill $TF_PID 2>/dev/null
fi
echo ""

echo "5. Checking RF2O Parameters..."
echo "----------------------------"
if ros2 node list | grep -q "rf2o_laser_odometry"; then
    echo "✓ RF2O node is running"
    ros2 param list /rf2o_laser_odometry | grep -E "frame_id|publish_tf|topic"
else
    echo "✗ RF2O node NOT running"
fi
echo ""

echo "6. Checking Frame IDs..."
echo "----------------------------"
echo "Laser scan frame:"
ros2 topic echo /scan --once | grep "frame_id" | head -n 1
echo "Odom frame:"
ros2 topic echo /odom --once | grep "frame_id" | head -n 2
echo ""

echo "=================================="
echo "Diagnostic Complete"
echo "=================================="
echo ""
echo "Common Issues:"
echo "- If /scan is not publishing: Check LiDAR connection"
echo "- If /odom is not publishing: Check RF2O configuration"
echo "- If TF is broken: Ensure frame IDs match in all configs"
echo "- If robot doesn't move in RViz: Wait 2-3 seconds for RF2O to initialize"
