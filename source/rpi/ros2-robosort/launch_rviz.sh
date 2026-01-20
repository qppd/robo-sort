#!/bin/bash
# RViz2 Quick Launch Script for RoboSort
# Use this if RViz doesn't launch from robosort.launch.py

echo "Starting RViz2 for RoboSort..."
echo ""

# Get the package share directory
CONTROL_DIR=$(ros2 pkg prefix robosort_control)/share/robosort_control

# Check if config file exists
if [ -f "$CONTROL_DIR/config/robosort.rviz" ]; then
    echo "✓ Found RViz config: $CONTROL_DIR/config/robosort.rviz"
    rviz2 -d "$CONTROL_DIR/config/robosort.rviz"
else
    echo "✗ Config file not found, launching RViz with default config..."
    echo "  You can add displays manually:"
    echo "  - Add -> By topic -> /scan (LaserScan)"
    echo "  - Add -> By topic -> /map (Map)"
    echo "  - Add -> By display type -> RobotModel"
    echo "  - Add -> By display type -> TF"
    echo ""
    echo "  Set Fixed Frame to: map"
    echo ""
    rviz2
fi
