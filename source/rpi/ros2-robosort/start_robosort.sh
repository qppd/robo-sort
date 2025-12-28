#!/bin/bash
# RoboSort Quick Start Script
# This script launches the complete RoboSort waste segregation system

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}╔════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║   RoboSort Waste Segregation System   ║${NC}"
echo -e "${GREEN}║        ROS2 Quick Start Script         ║${NC}"
echo -e "${GREEN}╚════════════════════════════════════════╝${NC}"
echo ""

# Check if running from correct directory
WORKSPACE_DIR="/home/robosort/robo-sort/source/rpi/ros2-robosort"
if [ "$PWD" != "$WORKSPACE_DIR" ]; then
    echo -e "${YELLOW}Changing to workspace directory...${NC}"
    cd "$WORKSPACE_DIR"
fi

# Source ROS2 and workspace
echo -e "${GREEN}[1/4]${NC} Sourcing ROS2 Jazzy..."
source /opt/ros/jazzy/setup.bash

echo -e "${GREEN}[2/4]${NC} Sourcing workspace..."
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
else
    echo -e "${RED}Error: Workspace not built. Run 'colcon build' first.${NC}"
    exit 1
fi

# Check if Arduino is connected
echo -e "${GREEN}[3/4]${NC} Checking Arduino connection..."
if [ -e /dev/ttyACM0 ]; then
    echo -e "${GREEN}✓${NC} Arduino found at /dev/ttyACM0"
    SERIAL_PORT="/dev/ttyACM0"
elif [ -e /dev/ttyUSB0 ]; then
    echo -e "${YELLOW}!${NC} Arduino found at /dev/ttyUSB0"
    SERIAL_PORT="/dev/ttyUSB0"
else
    echo -e "${RED}✗${NC} Arduino not found. Please connect Arduino Mega."
    echo -e "${YELLOW}Continuing without Arduino (some features disabled)...${NC}"
    SERIAL_PORT="/dev/ttyACM0"
fi

# Parse arguments
USE_RVIZ="true"
MODEL_PATH="yolov8n.pt"
CAMERA_SOURCE="usb0"
CONFIDENCE="0.5"

while [[ $# -gt 0 ]]; do
    case $1 in
        --no-rviz)
            USE_RVIZ="false"
            shift
            ;;
        --model)
            MODEL_PATH="$2"
            shift 2
            ;;
        --camera)
            CAMERA_SOURCE="$2"
            shift 2
            ;;
        --confidence)
            CONFIDENCE="$2"
            shift 2
            ;;
        --help)
            echo "Usage: ./start_robosort.sh [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --no-rviz              Disable RViz visualization"
            echo "  --model PATH           Path to YOLO model (default: yolov8n.pt)"
            echo "  --camera SOURCE        Camera source: usb0, picamera0 (default: usb0)"
            echo "  --confidence THRESHOLD Confidence threshold 0-1 (default: 0.5)"
            echo "  --help                 Show this help message"
            echo ""
            echo "Examples:"
            echo "  ./start_robosort.sh"
            echo "  ./start_robosort.sh --no-rviz --camera picamera0"
            echo "  ./start_robosort.sh --model custom_model.pt --confidence 0.6"
            exit 0
            ;;
        *)
            echo -e "${RED}Unknown option: $1${NC}"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

# Display configuration
echo -e "${GREEN}[4/4]${NC} Launching RoboSort system..."
echo ""
echo "Configuration:"
echo "  • YOLO Model: $MODEL_PATH"
echo "  • Camera: $CAMERA_SOURCE"
echo "  • Confidence: $CONFIDENCE"
echo "  • Serial Port: $SERIAL_PORT"
echo "  • RViz: $USE_RVIZ"
echo ""

# Launch the system
echo -e "${GREEN}Starting nodes...${NC}"
echo ""

ros2 launch robosort_vision robosort.launch.py \
    model_path:="$MODEL_PATH" \
    camera_source:="$CAMERA_SOURCE" \
    serial_port:="$SERIAL_PORT" \
    confidence_threshold:="$CONFIDENCE" \
    use_rviz:="$USE_RVIZ"

# Cleanup message
echo ""
echo -e "${GREEN}RoboSort system shutdown complete.${NC}"
