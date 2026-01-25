#!/bin/bash
# USB Serial Port Reset Utility
# Helps clean up stuck USB serial devices

echo "========================================"
echo "USB Serial Port Reset Utility"
echo "========================================"

# Show current USB devices
echo -e "\nCurrent USB serial devices:"
ls -la /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo "  No USB serial devices found"

# Check for processes using serial ports
echo -e "\nChecking for processes using serial ports..."
for port in /dev/ttyUSB* /dev/ttyACM*; do
    if [ -e "$port" ]; then
        echo "Checking $port:"
        lsof "$port" 2>/dev/null || echo "  Not in use"
    fi
done

# Option to reset USB subsystem (requires root)
echo -e "\n========================================"
echo "Reset Options:"
echo "1. Kill Python processes using serial ports"
echo "2. Reload USB serial drivers (requires sudo)"
echo "3. Show device info"
echo "4. Exit"
echo "========================================"

read -p "Select option (1-4): " option

case $option in
    1)
        echo "Killing Python processes using serial ports..."
        for port in /dev/ttyUSB* /dev/ttyACM*; do
            if [ -e "$port" ]; then
                pids=$(lsof -t "$port" 2>/dev/null)
                if [ -n "$pids" ]; then
                    echo "Killing processes for $port: $pids"
                    kill -9 $pids
                fi
            fi
        done
        echo "Done!"
        ;;
    2)
        echo "Reloading USB serial drivers..."
        sudo modprobe -r ch341 pl2303 ftdi_sio usbserial cp210x 2>/dev/null
        sleep 1
        sudo modprobe ch341 pl2303 ftdi_sio usbserial cp210x 2>/dev/null
        echo "Drivers reloaded. Check /dev/ttyUSB* again"
        ls -la /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
        ;;
    3)
        echo -e "\nUSB Device Information:"
        for port in /dev/ttyUSB* /dev/ttyACM*; do
            if [ -e "$port" ]; then
                echo -e "\n$port:"
                udevadm info -q all -n "$port" | grep -E 'ID_VENDOR|ID_MODEL|ID_SERIAL'
            fi
        done
        ;;
    4)
        echo "Exiting..."
        exit 0
        ;;
    *)
        echo "Invalid option"
        ;;
esac
