# USB Serial Port Issues - Troubleshooting Guide

## Problem: USB Port Numbers Keep Incrementing

You may notice that USB serial devices (LIDAR and Arduino) keep getting new port numbers after each run:
- ttyUSB0 → ttyUSB1 → ttyUSB2 → ttyUSB3, etc.

### Why This Happens

1. **Serial ports not properly closed**: When the program crashes or exits with errors, the serial port may not be properly released
2. **I/O errors**: Hardware disconnections or communication errors can leave the port in a bad state
3. **Linux kernel behavior**: When a device isn't cleanly disconnected, Linux assigns a new device number on reconnect

## Solutions

### Solution 1: Fix Serial Port Cleanup (Already Implemented)

The code has been updated with:
- Better error handling in `navigate.py`
- Proper cleanup on exit
- Automatic reconnection detection
- Timeout handling for LIDAR stop

### Solution 2: Reset Stuck USB Devices

When ports get stuck, use the reset utility:

```bash
cd source/rpi/yolo
chmod +x reset_usb.sh
./reset_usb.sh
```

Options:
1. **Kill processes**: Terminates Python processes holding serial ports
2. **Reload drivers**: Resets USB serial drivers (requires sudo)
3. **Show info**: Display device information

### Solution 3: Use Stable Device Names (Recommended)

Instead of `/dev/ttyUSB0` (which changes), use permanent names like `/dev/robosort-lidar`.

#### Installation Steps:

1. **Find your device vendor/product IDs:**
   ```bash
   # For LIDAR
   udevadm info -a -n /dev/ttyUSB0 | grep -E 'idVendor|idProduct'
   
   # For Arduino
   udevadm info -a -n /dev/ttyACM0 | grep -E 'idVendor|idProduct'
   ```

2. **Edit udev rules if needed:**
   Edit `99-robosort.rules` and update vendor/product IDs to match your devices.

3. **Install udev rules:**
   ```bash
   sudo cp 99-robosort.rules /etc/udev/rules.d/
   sudo udevadm control --reload-rules
   sudo udevadm trigger
   ```

4. **Unplug and replug** both USB devices

5. **Verify stable names exist:**
   ```bash
   ls -la /dev/robosort-*
   ```

6. **Update nav_config.py:**
   ```python
   LIDAR_PORT = '/dev/robosort-lidar'
   ARDUINO_PORT = '/dev/robosort-arduino'
   ```

### Solution 4: Manual Reset

If devices are stuck, manually reset them:

```bash
# Kill processes using ports
sudo pkill -9 -f "python.*test_navigation"
sudo pkill -9 -f "python.*navigate"

# Find and kill specific port users
sudo lsof /dev/ttyUSB0  # Find PID
sudo kill -9 <PID>

# Reload USB drivers
sudo modprobe -r ch341 ftdi_sio
sudo modprobe ch341 ftdi_sio

# Or fully reset USB subsystem (nuclear option)
sudo systemctl restart udev
```

### Solution 5: Check Physical Connections

The I/O errors you're seeing suggest:
- **Loose USB cable**: Check connections
- **Power issues**: Ensure adequate power supply
- **Cable quality**: Use a good quality USB cable
- **USB hub issues**: Try direct connection to Raspberry Pi

## Quick Fixes

### Before Each Run:

```bash
# Check current ports
ls -la /dev/ttyUSB* /dev/ttyACM*

# If they're incrementing, reset
sudo modprobe -r ch341
sudo modprobe ch341

# Verify they reset to ttyUSB0
ls -la /dev/ttyUSB*
```

### After Errors:

```bash
# Kill stuck processes
sudo pkill -9 -f python

# Wait a moment
sleep 1

# Unplug and replug USB devices

# Verify ports are back to normal
ls -la /dev/ttyUSB* /dev/ttyACM*
```

## Prevention

1. **Always use proper shutdown**: Use Ctrl+C to stop the program cleanly
2. **Check connections before running**: Ensure USB cables are secure
3. **Use stable device names**: Implement udev rules (Solution 3)
4. **Monitor errors**: Watch for I/O errors and restart if they occur
5. **Don't force-kill**: Avoid `kill -9` unless necessary

## Error Codes Reference

- **[Errno 5] Input/output error**: Serial communication failed - check cable/connection
- **[Errno 16] Device or resource busy**: Port already in use - kill processes
- **[Errno 19] No such device**: Device disconnected - check USB connection

## Testing

After implementing fixes, test:

```bash
cd source/rpi/yolo
python test_navigation.py lidar /dev/robosort-lidar

# If using numbered ports:
python test_navigation.py lidar /dev/ttyUSB0
```

Should see:
```
✓ LIDAR connected
✓ Receiving data: 460 distance readings
```

If you see errors, the device is still stuck - use reset scripts above.
