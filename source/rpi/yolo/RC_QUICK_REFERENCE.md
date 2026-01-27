# 🎮 RoboSort RC Control - Quick Reference

## 🚀 Quick Start

### Raspberry Pi
```bash
cd /path/to/robo-sort/source/rpi/yolo
python3 rc_controler.py --source=usb0 --arduino=/dev/ttyUSB1
```

### Android App
1. Open RoboSortControl app
2. Wait for "Connected" status (green)
3. Press and hold buttons to control:
   - **FORWARD** - Move forward
   - **BACKWARD** - Move backward
   - **LEFT** - Turn left
   - **RIGHT** - Turn right
4. Release button to **STOP**
5. Use sliders for servo control (0-180°)

---

## 📡 Command Reference

### Motor Commands (Python → Arduino)
| Command | Serial | Action |
|---------|--------|--------|
| FORWARD | `M:F\n` | Move forward |
| BACKWARD | `M:B\n` | Move backward |
| TURN_LEFT | `M:L\n` | Turn left |
| TURN_RIGHT | `M:R\n` | Turn right |
| STOP | `M:S\n` | Stop motors |

### Servo Commands (Python → Arduino)
| Format | Example | Action |
|--------|---------|--------|
| `S:ID:ANGLE\n` | `S:1:90\n` | Set servo 1 to 90° |
| `S:ID:ANGLE\n` | `S:2:45\n` | Set servo 2 to 45° |
| `S:ID:ANGLE\n` | `S:3:180\n` | Set servo 3 to 180° |
| `S:ID:ANGLE\n` | `S:4:0\n` | Set servo 4 to 0° |

### Feedback (Arduino → Python)
| Format | Example | Meaning |
|--------|---------|---------|
| `M:STATE\n` | `M:FORWARD\n` | Motors moving forward |
| `M:STATE\n` | `M:STOP\n` | Motors stopped |
| `S:ID:ANGLE\n` | `S:1:90\n` | Servo 1 at 90° |

---

## 🔌 Connection Ports

### Common Arduino Ports
- Linux/RPi: `/dev/ttyUSB0`, `/dev/ttyUSB1`, `/dev/ttyACM0`
- Windows: `COM3`, `COM4`, `COM5`
- macOS: `/dev/cu.usbserial-*`, `/dev/cu.usbmodem-*`

### Find Arduino Port
```bash
# Linux/RPi
ls /dev/ttyUSB* /dev/ttyACM*

# Or use dmesg
dmesg | grep tty
```

### Camera Sources
- `usb0` → First USB camera
- `usb1` → Second USB camera
- `0` → Built-in camera (Pi Camera usually)

---

## 🗂️ Firebase Paths

### Commands (Android → RPi)
```
/robosort/commands/
├── motor: "FORWARD" | "BACKWARD" | "TURN_LEFT" | "TURN_RIGHT" | "STOP"
├── servo1: 0-180
├── servo2: 0-180
├── servo3: 0-180
├── servo4: 0-180
└── timestamp: Long
```

### Status (RPi → Android)
```
/robosort/status/
├── motor_state: String (current state)
├── servo1: 0-180
├── servo2: 0-180
├── servo3: 0-180
├── servo4: 0-180
├── connected: Boolean
└── last_update: Long
```

---

## ⌨️ Keyboard Controls

### In Camera Window
- `q` - Quit program
- Window displays:
  - Motor state
  - Servo 1 & 2 angles
  - Servo 3 & 4 angles

### In Terminal
- `Ctrl+C` - Emergency stop and quit

---

## 🔧 Configuration

### Update Firebase Config
Edit [rc_controler.py](rc_controler.py) lines 15-20:
```python
firebase_config = {
    "apiKey": "YOUR_API_KEY_HERE",
    "authDomain": "your-project.firebaseapp.com",
    "databaseURL": "https://your-project.firebaseio.com",
    "storageBucket": "your-project.appspot.com"
}
```

### Adjust Serial Baud Rate
Line 48 in `rc_controler.py`:
```python
self.arduino = serial.Serial(self.arduino_port, 115200, timeout=0.01)
```
**Arduino must match:** `Serial.begin(115200);`

### Adjust Camera Resolution
Lines 70-72:
```python
self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
self.camera.set(cv2.CAP_PROP_FPS, 30)
```

---

## 🐛 Quick Troubleshooting

| Problem | Solution |
|---------|----------|
| "Permission denied" on serial | `sudo usermod -a -G dialout $USER` then reboot |
| Camera not found | Try different index: `--source=usb1` |
| Firebase connection failed | Check config, internet connection |
| Robot doesn't respond | Verify Arduino is running correct code |
| Delayed response | Check WiFi quality, use 5GHz if possible |
| Motors don't stop | Check Android button release logic |

---

## 📊 Expected Performance

| Metric | Value |
|--------|-------|
| Button press → Firebase | < 50ms |
| Firebase → RPi | < 100ms |
| RPi → Arduino | < 5ms |
| **Total latency** | **~150-200ms** |
| Camera FPS | 30 FPS (configurable) |
| Serial baud rate | 115200 |

---

## 📞 Support Files

- **Full Setup Guide:** [RC_CONTROL_SETUP.md](RC_CONTROL_SETUP.md)
- **Firebase Structure:** [FIREBASE_STRUCTURE.md](FIREBASE_STRUCTURE.md)
- **Serial Protocol:** [../../arduino/SERIAL_PROTOCOL.md](../../arduino/SERIAL_PROTOCOL.md)
- **Android Code:** [../../android/RoboSortControl/ANDROID_RC_IMPLEMENTATION.md](../../android/RoboSortControl/ANDROID_RC_IMPLEMENTATION.md)

---

## ✅ Pre-Flight Checklist

Before operating:
- [ ] Firebase config updated in Python script
- [ ] Arduino programmed with RC control code
- [ ] Arduino serial port identified
- [ ] Camera working (test with --source argument)
- [ ] WiFi connected on RPi and Android
- [ ] Android app has google-services.json
- [ ] Robot in safe area for testing
- [ ] Emergency stop accessible

---

## 🎯 Operation Flow

1. **Power on** Arduino and Raspberry Pi
2. **Connect** Arduino to RPi via USB
3. **Run** Python script on RPi
4. **Verify** connections (Arduino, Camera, Firebase)
5. **Open** Android app
6. **Check** "Connected" status (green)
7. **Test** STOP button first
8. **Control** robot with buttons
9. **Monitor** camera feed on RPi screen
10. **Press** `q` or `Ctrl+C` to quit safely

---

## 🎮 Control Tips

### For Smooth Operation
- **Short taps** for small movements
- **Hold buttons** for continuous movement
- **Release immediately** to stop
- **Practice** in open area first
- **Monitor** camera feed for obstacles

### RC Driving Technique
- Forward + Left/Right for turns while moving
- Use BACKWARD carefully (limited rear visibility)
- Stop before changing direction for safety
- Servo adjustments can be made while moving

---

## 🔒 Safety Notes

- Always have physical access to power switch
- Test STOP button before driving
- Keep emergency stop (Ctrl+C) ready
- Operate in safe, open area
- Monitor battery levels
- Don't operate near stairs or water

---

**Last Updated:** January 2026  
**Version:** 1.0  
**Python:** 3.7+  
**Android:** API 21+
