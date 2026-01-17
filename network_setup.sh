#!/bin/bash

# RoboSort ROS2 Network Setup Script
# Run this on both machines (remote computer and Raspberry Pi)
# Usage: source network_setup.sh

# Set ROS2 network configuration for multi-machine setup
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0

# Optional: Uncomment and set explicit IPs if multicast doesn't work
# export ROS_IP=192.168.1.xxx  # Replace with your machine's IP

echo "=========================================="
echo "RoboSort ROS2 Network Configuration"
echo "=========================================="
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "ROS_LOCALHOST_ONLY: $ROS_LOCALHOST_ONLY"
echo "ROS_IP: ${ROS_IP:-auto (multicast)}"
echo ""
echo "To test connection:"
echo "  ros2 topic list"
echo "  ros2 topic echo /scan --once"
echo "=========================================="