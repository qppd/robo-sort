#!/bin/bash
# RF2O Laser Odometry Verification Script
# Run this to diagnose rf2o issues

echo "================================"
echo "RF2O Laser Odometry Diagnostics"
echo "================================"
echo ""

# Check if rf2o node is running
echo "1. Checking if RF2O node is running..."
if ros2 node list | grep -q "rf2o_laser_odometry"; then
    echo "   ✓ RF2O node is running"
else
    echo "   ✗ RF2O node is NOT running"
    echo "   → Launch your system first!"
    exit 1
fi
echo ""

# Check if lidar is publishing
echo "2. Checking if LiDAR is publishing scans..."
if ros2 topic list | grep -q "/scan"; then
    echo "   ✓ /scan topic exists"
    SCAN_HZ=$(timeout 3s ros2 topic hz /scan 2>/dev/null | grep "average rate" | awk '{print $3}')
    if [ ! -z "$SCAN_HZ" ]; then
        echo "   ✓ LiDAR publishing at ${SCAN_HZ} Hz"
    else
        echo "   ✗ /scan topic exists but no data"
    fi
else
    echo "   ✗ /scan topic does NOT exist"
    echo "   → Check if lidar node is running"
fi
echo ""

# Check frame ID
echo "3. Checking LiDAR frame ID..."
FRAME_ID=$(timeout 2s ros2 topic echo /scan --field header.frame_id --once 2>/dev/null)
if [ "$FRAME_ID" = "lidar_link" ]; then
    echo "   ✓ Frame ID is correct: lidar_link"
else
    echo "   ✗ Frame ID is: $FRAME_ID (expected: lidar_link)"
fi
echo ""

# Check TF tree
echo "4. Checking TF tree..."
if timeout 2s ros2 run tf2_ros tf2_echo base_footprint lidar_link 2>/dev/null 1>/dev/null; then
    echo "   ✓ TF base_footprint->lidar_link exists"
else
    echo "   ✗ TF base_footprint->lidar_link NOT found"
    echo "   → Check if lidar_tf_publisher is running"
fi

if timeout 2s ros2 run tf2_ros tf2_echo odom base_footprint 2>/dev/null 1>/dev/null; then
    echo "   ✓ TF odom->base_footprint exists"
else
    echo "   ⚠ TF odom->base_footprint NOT found (RF2O should publish this)"
fi
echo ""

# Check odometry topic
echo "5. Checking odometry output..."
if ros2 topic list | grep -q "/odom"; then
    echo "   ✓ /odom topic exists"
    ODOM_HZ=$(timeout 3s ros2 topic hz /odom 2>/dev/null | grep "average rate" | awk '{print $3}')
    if [ ! -z "$ODOM_HZ" ]; then
        echo "   ✓ Odometry publishing at ${ODOM_HZ} Hz"
    else
        echo "   ✗ /odom topic exists but no data"
    fi
else
    echo "   ✗ /odom topic does NOT exist"
fi
echo ""

# Check rf2o subscriptions
echo "6. Checking RF2O node info..."
echo "   Subscriptions:"
ros2 node info /rf2o_laser_odometry | grep -A 10 "Subscribers:" | grep "sensor_msgs"
echo ""
echo "   Publishers:"
ros2 node info /rf2o_laser_odometry | grep -A 10 "Publishers:" | grep -E "nav_msgs|tf"
echo ""

# Summary
echo "================================"
echo "Summary:"
echo "================================"
if ros2 topic hz /odom --once 2>/dev/null | grep -q "average"; then
    echo "✓ RF2O is working correctly!"
else
    echo "✗ RF2O has issues - check the output above"
    echo ""
    echo "Common fixes:"
    echo "1. Ensure lidar is connected and publishing"
    echo "2. Check frame IDs match in rf2o_params.yaml"
    echo "3. Verify TF tree: ros2 run tf2_tools view_frames"
    echo "4. Check rf2o logs for errors"
fi
echo ""
