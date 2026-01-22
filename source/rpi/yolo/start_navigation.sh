#!/bin/bash
# Quick start script for RoboSort Autonomous Navigation

echo "======================================"
echo "RoboSort Autonomous Navigation"
echo "======================================"
echo ""

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Default ports
LIDAR_PORT="/dev/ttyUSB0"
ARDUINO_PORT="/dev/ttyACM0"

# Function to check if port exists
check_port() {
    if [ -e "$1" ]; then
        echo -e "${GREEN}✓${NC} Found: $1"
        return 0
    else
        echo -e "${RED}✗${NC} Not found: $1"
        return 1
    fi
}

# Function to set port permissions
fix_permissions() {
    echo ""
    echo "Attempting to fix port permissions..."
    sudo chmod 666 "$1" 2>/dev/null
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✓${NC} Permissions fixed for $1"
    else
        echo -e "${YELLOW}⚠${NC} Could not fix permissions. You may need to run:"
        echo "  sudo chmod 666 $1"
    fi
}

# Check for Python
echo "Checking system requirements..."
if ! command -v python3 &> /dev/null; then
    echo -e "${RED}✗${NC} Python 3 not found. Please install Python 3."
    exit 1
fi
echo -e "${GREEN}✓${NC} Python 3 found"

# Check for pyserial
python3 -c "import serial" 2>/dev/null
if [ $? -ne 0 ]; then
    echo -e "${YELLOW}⚠${NC} pyserial not installed"
    echo "Installing pyserial..."
    pip3 install pyserial
fi

# Check ports
echo ""
echo "Checking serial ports..."
check_port "$LIDAR_PORT"
LIDAR_OK=$?

check_port "$ARDUINO_PORT"
ARDUINO_OK=$?

# Offer to fix permissions if ports exist but may not be accessible
if [ $LIDAR_OK -eq 0 ] || [ $ARDUINO_OK -eq 0 ]; then
    echo ""
    read -p "Fix port permissions? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        [ $LIDAR_OK -eq 0 ] && fix_permissions "$LIDAR_PORT"
        [ $ARDUINO_OK -eq 0 ] && fix_permissions "$ARDUINO_PORT"
    fi
fi

# List available ports
echo ""
echo "Available serial ports:"
ls /dev/tty{USB,ACM}* 2>/dev/null || echo "  No USB/ACM ports found"

# Menu
echo ""
echo "======================================"
echo "Select an option:"
echo "======================================"
echo "1) Test LIDAR connection"
echo "2) Test Arduino connection"
echo "3) Test obstacle detection (no motors)"
echo "4) Run autonomous navigation"
echo "5) Run all tests"
echo "6) Configure ports"
echo "0) Exit"
echo ""
read -p "Enter choice [0-6]: " choice

case $choice in
    1)
        echo ""
        echo "Testing LIDAR connection..."
        python3 test_navigation.py lidar --lidar-port "$LIDAR_PORT"
        ;;
    2)
        echo ""
        echo -e "${YELLOW}⚠ WARNING: Robot will move briefly!${NC}"
        read -p "Continue? (y/n) " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            python3 test_navigation.py arduino --arduino-port "$ARDUINO_PORT"
        fi
        ;;
    3)
        echo ""
        echo "Testing obstacle detection (30 seconds)..."
        python3 test_navigation.py detection --lidar-port "$LIDAR_PORT"
        ;;
    4)
        echo ""
        echo -e "${YELLOW}⚠ WARNING: Robot will move autonomously!${NC}"
        echo -e "${YELLOW}⚠ Ensure clear area (at least 3m x 3m)${NC}"
        echo -e "${YELLOW}⚠ Press Ctrl+C for emergency stop${NC}"
        echo ""
        read -p "Continue? (y/n) " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            python3 navigate.py --lidar-port "$LIDAR_PORT" --arduino-port "$ARDUINO_PORT"
        fi
        ;;
    5)
        echo ""
        echo "Running all tests..."
        python3 test_navigation.py all --lidar-port "$LIDAR_PORT" --arduino-port "$ARDUINO_PORT"
        ;;
    6)
        echo ""
        echo "Current configuration:"
        echo "  LIDAR Port: $LIDAR_PORT"
        echo "  Arduino Port: $ARDUINO_PORT"
        echo ""
        read -p "Enter LIDAR port [$LIDAR_PORT]: " new_lidar
        LIDAR_PORT="${new_lidar:-$LIDAR_PORT}"
        read -p "Enter Arduino port [$ARDUINO_PORT]: " new_arduino
        ARDUINO_PORT="${new_arduino:-$ARDUINO_PORT}"
        echo ""
        echo "Updated configuration:"
        echo "  LIDAR Port: $LIDAR_PORT"
        echo "  Arduino Port: $ARDUINO_PORT"
        ;;
    0)
        echo "Exiting..."
        exit 0
        ;;
    *)
        echo -e "${RED}✗${NC} Invalid choice"
        exit 1
        ;;
esac

echo ""
echo "======================================"
echo "Done!"
echo "======================================"
