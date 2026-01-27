# RoboSort Real-Time RC Control - Complete Setup Guide

## 📋 System Overview

This system provides real-time remote control of your RoboSort robot using:
- **Android App** → Sends commands via button press/release
- **Firebase Realtime Database** → Cloud messaging layer
- **Raspberry Pi** → Processes commands and controls hardware
- **Arduino** → Executes motor and servo commands
- **Camera** → Live view on Raspberry Pi (local only)

**Key Feature:** ZERO artificial delays - pure event-driven architecture for instant response!

---

## 🔧 Part 1: Firebase Setup

### 1.1 Create Firebase Project
1. Go to [Firebase Console](https://console.firebase.google.com/)
2. Create a new project (or use existing)
3. Enable **Realtime Database**
4. Set database rules (for testing, you can use public access):
```json
{
  "rules": {
    ".read": true,
    ".write": true
  }
}
```
⚠️ **For production, implement proper authentication!**

### 1.2 Get Firebase Configuration
1. Project Settings → General
2. Scroll to "Your apps" section
3. Click the Android icon to add Android app
4. Download `google-services.json` (for Android)
5. Note the following for Python (pyrebase4):
   - API Key
   - Auth Domain
   - Database URL  
   - Storage Bucket

### 1.3 Update Python Script
Edit [rc_controler.py](rc_controler.py) line 15-20:
```python
firebase_config = {
    "apiKey": "YOUR_ACTUAL_API_KEY",
    "authDomain": "your-project-id.firebaseapp.com",
    "databaseURL": "https://your-project-id.firebaseio.com",
    "storageBucket": "your-project-id.appspot.com"
}
```

---

## 🍓 Part 2: Raspberry Pi Setup

### 2.1 Install Dependencies
```bash
cd /path/to/robo-sort/source/rpi/yolo
pip3 install -r requirements_rc.txt
```

### 2.2 Configure Camera
Test your camera:
```bash
python3 -c "import cv2; print(cv2.VideoCapture(0).isOpened())"
```
Should print `True`. If not, try different camera indices (1, 2, etc.)

### 2.3 Configure Arduino Serial
Find Arduino port:
```bash
ls /dev/ttyUSB* /dev/ttyACM*
```
Common ports: `/dev/ttyUSB0`, `/dev/ttyUSB1`, `/dev/ttyACM0`

### 2.4 Run the RC Controller
```bash
python3 rc_controler.py --source=usb0 --arduino=/dev/ttyUSB1
```

**Arguments:**
- `--source=usb0` → Use USB camera 0 (use `usb1` for camera 1, etc.)
- `--arduino=/dev/ttyUSB1` → Arduino serial port

**Controls:**
- Press `q` in camera window to quit
- `Ctrl+C` in terminal also works

---

## 🤖 Part 3: Arduino Implementation

### 3.1 Serial Protocol
See [SERIAL_PROTOCOL.md](../../../arduino/SERIAL_PROTOCOL.md) for full details.

**Quick Reference:**

**Incoming Commands (RPi → Arduino):**
- `M:F\n` → Forward
- `M:B\n` → Backward
- `M:L\n` → Turn left
- `M:R\n` → Turn right
- `M:S\n` → Stop
- `S:1:90\n` → Set servo 1 to 90°

**Outgoing Feedback (Arduino → RPi):**
- `M:FORWARD\n` → Motors moving forward
- `M:STOP\n` → Motors stopped
- `S:1:90\n` → Servo 1 at 90°

### 3.2 Arduino Example Code

```cpp
#include <Servo.h>

Servo servo1, servo2, servo3, servo4;

void setup() {
    Serial.begin(115200);
    
    // Motor pins setup
    pinMode(MOTOR_L_FWD, OUTPUT);
    pinMode(MOTOR_L_BWD, OUTPUT);
    pinMode(MOTOR_R_FWD, OUTPUT);
    pinMode(MOTOR_R_BWD, OUTPUT);
    
    // Servo setup
    servo1.attach(SERVO1_PIN);
    servo2.attach(SERVO2_PIN);
    servo3.attach(SERVO3_PIN);
    servo4.attach(SERVO4_PIN);
}

void loop() {
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        
        if (command.startsWith("M:")) {
            handleMotorCommand(command.charAt(2));
        }
        else if (command.startsWith("S:")) {
            handleServoCommand(command);
        }
    }
}

void handleMotorCommand(char action) {
    switch(action) {
        case 'F':
            // Forward
            digitalWrite(MOTOR_L_FWD, HIGH);
            digitalWrite(MOTOR_L_BWD, LOW);
            digitalWrite(MOTOR_R_FWD, HIGH);
            digitalWrite(MOTOR_R_BWD, LOW);
            Serial.println("M:FORWARD");
            break;
        case 'B':
            // Backward
            digitalWrite(MOTOR_L_FWD, LOW);
            digitalWrite(MOTOR_L_BWD, HIGH);
            digitalWrite(MOTOR_R_FWD, LOW);
            digitalWrite(MOTOR_R_BWD, HIGH);
            Serial.println("M:BACKWARD");
            break;
        case 'L':
            // Left
            digitalWrite(MOTOR_L_FWD, LOW);
            digitalWrite(MOTOR_L_BWD, HIGH);
            digitalWrite(MOTOR_R_FWD, HIGH);
            digitalWrite(MOTOR_R_BWD, LOW);
            Serial.println("M:LEFT");
            break;
        case 'R':
            // Right
            digitalWrite(MOTOR_L_FWD, HIGH);
            digitalWrite(MOTOR_L_BWD, LOW);
            digitalWrite(MOTOR_R_FWD, LOW);
            digitalWrite(MOTOR_R_BWD, HIGH);
            Serial.println("M:RIGHT");
            break;
        case 'S':
            // Stop
            digitalWrite(MOTOR_L_FWD, LOW);
            digitalWrite(MOTOR_L_BWD, LOW);
            digitalWrite(MOTOR_R_FWD, LOW);
            digitalWrite(MOTOR_R_BWD, LOW);
            Serial.println("M:STOP");
            break;
    }
}

void handleServoCommand(String cmd) {
    int firstColon = cmd.indexOf(':');
    int secondColon = cmd.indexOf(':', firstColon + 1);
    
    int servoId = cmd.substring(firstColon + 1, secondColon).toInt();
    int angle = cmd.substring(secondColon + 1).toInt();
    angle = constrain(angle, 0, 180);
    
    switch(servoId) {
        case 1: servo1.write(angle); break;
        case 2: servo2.write(angle); break;
        case 3: servo3.write(angle); break;
        case 4: servo4.write(angle); break;
    }
    
    Serial.print("S:");
    Serial.print(servoId);
    Serial.print(":");
    Serial.println(angle);
}
```

---

## 📱 Part 4: Android App Implementation

### 4.1 Add Firebase to Android Project

1. Place `google-services.json` in `app/` directory
2. Ensure Firebase dependencies in `app/build.gradle`:
```gradle
implementation 'com.google.firebase:firebase-database:20.3.0'
```

### 4.2 Implement MainActivity

See [ANDROID_RC_IMPLEMENTATION.md](../android/RoboSortControl/ANDROID_RC_IMPLEMENTATION.md) for complete code.

**Key Features:**
- `OnTouchListener` for RC-style buttons (press = move, release = stop)
- SeekBars for servo control (0-180°)
- Real-time status updates from Firebase
- Connection status indicator

### 4.3 Button Behavior Example

```java
btnForward.setOnTouchListener(new View.OnTouchListener() {
    @Override
    public boolean onTouch(View v, MotionEvent event) {
        switch (event.getAction()) {
            case MotionEvent.ACTION_DOWN:
                // Button pressed - start moving
                commandsRef.child("motor").setValue("FORWARD");
                return true;
            case MotionEvent.ACTION_UP:
                // Button released - stop immediately
                commandsRef.child("motor").setValue("STOP");
                v.performClick();
                return true;
        }
        return false;
    }
});
```

---

## 🔄 Data Flow Diagram

```
┌─────────────┐
│ Android App │  [User presses FORWARD button]
└──────┬──────┘
       │ Write: /robosort/commands/motor = "FORWARD"
       ▼
┌──────────────────┐
│ Firebase RTDB    │  [Cloud database]
└──────┬───────────┘
       │ Stream update (< 100ms)
       ▼
┌──────────────────┐
│ Raspberry Pi     │  [Python listener receives]
│ rc_controler.py  │  Sends: "M:F\n" to Arduino
└──────┬───────────┘
       │ Serial @ 115200 baud (< 1ms)
       ▼
┌──────────────────┐
│ Arduino          │  [Executes motor command]
│                  │  Sends back: "M:FORWARD\n"
└──────┬───────────┘
       │
       ▼
┌──────────────────┐
│ Raspberry Pi     │  [Receives feedback]
│                  │  Updates Firebase status
└──────┬───────────┘
       │ Write: /robosort/status/motor_state = "FORWARD"
       ▼
┌──────────────────┐
│ Firebase RTDB    │  [Status updated]
└──────┬───────────┘
       │ ValueEventListener callback
       ▼
┌──────────────────┐
│ Android App      │  [UI updates showing "FORWARD"]
└──────────────────┘
```

---

## 🧪 Testing & Verification

### Test 1: Firebase Connection
Run Python script:
```bash
python3 rc_controler.py --source=usb0 --arduino=/dev/ttyUSB1
```
Look for: `✓ Firebase connected`

### Test 2: Arduino Connection
Look for: `✓ Arduino connected on /dev/ttyUSB1`

### Test 3: Camera Feed
Look for: `✓ Camera initialized: usb0`
Window should open showing live camera feed

### Test 4: Android Control
1. Open Android app
2. Check connection status (should be green "Connected")
3. Press and HOLD Forward button → robot should move forward
4. Release button → robot should STOP immediately

### Test 5: Servo Control
1. Move a servo slider in Android app
2. Watch the servo angle update in:
   - Arduino (physical movement)
   - RPi camera overlay (displays servo angles)
   - Android app status (feedback from robot)

---

## 🐛 Troubleshooting

### Problem: "Firebase connection failed"
**Solution:**
- Check Firebase config in `rc_controler.py`
- Verify database URL is correct
- Check internet connection on RPi

### Problem: "Arduino connection failed"
**Solution:**
```bash
# Check available ports
ls -l /dev/ttyUSB* /dev/ttyACM*

# Add user to dialout group
sudo usermod -a -G dialout $USER

# Reboot required after adding to group
sudo reboot
```

### Problem: "Camera not accessible"
**Solution:**
```bash
# Check camera device
ls -l /dev/video*

# Test with v4l2
v4l2-ctl --list-devices

# Try different camera index
python3 rc_controler.py --source=usb1 --arduino=/dev/ttyUSB1
```

### Problem: Delayed response
**Possible causes:**
- Poor WiFi connection → Test Firebase latency
- Arduino serial buffer full → Ensure Arduino sends feedback properly
- Camera processing too slow → Reduce resolution in script

### Problem: Robot doesn't stop when button released
**Check:**
1. Android app properly sends "STOP" on ACTION_UP
2. Firebase receives the STOP command
3. Arduino handles 'M:S' command correctly
4. Serial connection is stable

---

## ⚡ Performance Tips

### For Fastest Response:
1. **Use 5GHz WiFi** (not 2.4GHz) for RPi and Android
2. **Reduce camera resolution** if CPU is overloaded
3. **Keep Firebase database rules simple**
4. **Ensure Arduino serial rate is 115200** (not 9600)
5. **Monitor Firebase connection quality** in Android

### Expected Latencies:
- Button press → Firebase write: < 50ms
- Firebase → RPi stream: < 100ms
- RPi → Arduino serial: < 5ms
- **Total end-to-end:** ~150-200ms (very good for cloud-based RC!)

---

## 🚀 Running at Startup (Optional)

### Create systemd service for auto-start:
```bash
sudo nano /etc/systemd/system/robosort-rc.service
```

```ini
[Unit]
Description=RoboSort RC Control
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/robo-sort/source/rpi/yolo
ExecStart=/usr/bin/python3 rc_controler.py --source=usb0 --arduino=/dev/ttyUSB1
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl daemon-reload
sudo systemctl enable robosort-rc.service
sudo systemctl start robosort-rc.service
```

---

## 📊 Firebase Database Structure

See [FIREBASE_STRUCTURE.md](FIREBASE_STRUCTURE.md) for complete JSON schema and examples.

**Quick Reference:**
- Commands (written by Android): `/robosort/commands/`
- Status (written by RPi): `/robosort/status/`
- Motor states: "FORWARD", "BACKWARD", "TURN_LEFT", "TURN_RIGHT", "STOP"
- Servo angles: 0-180 (integer)

---

## 📝 Summary

You now have a complete real-time RC control system with:

✅ **Zero-delay** event-driven architecture  
✅ **Bidirectional** communication (commands + feedback)  
✅ **RC-style controls** (press to move, release to stop)  
✅ **Live camera** feed on Raspberry Pi  
✅ **Cloud-based** control via Firebase  
✅ **4 servo channels** with angle feedback  
✅ **Motor control** with state feedback  

**Next Steps:**
1. Test each component individually
2. Integrate and test full system
3. Fine-tune motor speeds in Arduino code
4. Add authentication to Firebase for security
5. Implement emergency stop button in Android app

🎮 **Happy RC controlling!**
