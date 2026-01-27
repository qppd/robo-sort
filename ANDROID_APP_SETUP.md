# RoboSort Android App Setup Guide

## Overview
This guide explains how to set up and use the RoboSort Android remote control app with real-time Firebase communication.

## Prerequisites
- Android Studio (latest version)
- Android device or emulator (API 24+)
- Firebase project (same as Raspberry Pi)
- Internet connection

## Setup Instructions

### 1. Open Project in Android Studio
```bash
cd source/android/RoboSortControl
# Open this folder in Android Studio
```

### 2. Configure Firebase
The `google-services.json` file should already be in `app/` directory. If not:
1. Go to Firebase Console
2. Select your RoboSort project
3. Add Android app with package name: `com.qppd.robosortcontrol`
4. Download `google-services.json`
5. Place it in `source/android/RoboSortControl/app/`

### 3. Update Firebase Database URL
In `MainActivity.java`, find the line:
```java
database.useEmulator("10.0.2.2", 9000); // Remove this line for production
```

**For Production (Real Device):**
Delete or comment out this line completely. The app will use the real Firebase database URL from `google-services.json`.

**For Emulator Testing:**
Keep this line to connect to Firebase emulator at `10.0.2.2:9000`.

### 4. Sync Gradle
1. Click "Sync Project with Gradle Files" in Android Studio toolbar
2. Wait for dependencies to download (Firebase SDK, Material Components, etc.)
3. Resolve any dependency conflicts if prompted

### 5. Build and Run
1. Connect Android device via USB or start emulator
2. Click Run button (▶) in Android Studio
3. Select target device
4. Wait for app to install and launch

## App Features

### Motor Control
- **Forward Button**: Hold to move forward at 200 speed
- **Backward Button**: Hold to move backward at 200 speed
- **Left Button**: Hold to turn left at 150 speed
- **Right Button**: Hold to turn right at 150 speed
- **Stop Button**: Immediate emergency stop

All motor buttons are **hold-to-move** - robot stops automatically when you release.

### Servo Control
Four sliders for precise servo control:
- **Slider 1**: Arm Base (0-180°, default 90°)
- **Slider 2**: Arm Shoulder (0-180°, default 90°)
- **Slider 3**: Arm Elbow (0-180°, default 90°)
- **Slider 4**: Gripper (0-180°, default 90°)

Changes are sent to Firebase **instantly** as you drag the slider.

### Real-Time Feedback
The feedback display shows:
- Connection timestamp
- Robot status (OK, ERROR, etc.)
- Current motor state
- Current servo positions
- Ultrasonic sensor distance
- Any error messages

Updates automatically in real-time from Firebase.

### Connection Status
- **Green "Connected"**: Firebase connection active
- **Red "Disconnected"**: No Firebase connection

## Firebase Data Structure

### Commands Sent (App → Firebase → RPi)
```json
{
  "robosort": {
    "commands": {
      "motor": {
        "type": "motor",
        "direction": "FORWARD",
        "speed": 200,
        "timestamp": 1234567890
      },
      "servo1": {
        "type": "servo",
        "servo": 1,
        "angle": 90,
        "timestamp": 1234567890
      }
    }
  }
}
```

### Feedback Received (RPi → Firebase → App)
```json
{
  "robosort": {
    "feedback": {
      "timestamp": 1234567890,
      "status": "OK",
      "motor_state": "FORWARD",
      "servo_positions": {
        "1": 90,
        "2": 90,
        "3": 90,
        "4": 90
      },
      "sensors": {
        "ultrasonic_distance": 25.5
      },
      "error": ""
    }
  }
}
```

## Troubleshooting

### App Won't Connect to Firebase
1. Check internet connection
2. Verify `google-services.json` is correct
3. Check Firebase Console → Database → Rules (should allow read/write)
4. For emulator: Ensure Firebase emulator is running
5. For real device: Remove `database.useEmulator()` line

### Commands Not Reaching Robot
1. Ensure Raspberry Pi `rc_controler.py` is running
2. Check RPi has internet connection
3. Verify Firebase database URL matches in both app and RPi
4. Check Firebase Console → Database to see if commands appear

### Feedback Not Updating
1. Verify RPi is sending feedback to Firebase
2. Check Firebase Console → Database → `robosort/feedback` path
3. Ensure Firebase listener is active (check Android logcat)

### Build Errors
1. **Firebase dependency issues**: Sync Gradle again
2. **Compilation errors**: Update Android SDK to API 36
3. **Plugin errors**: Update Android Gradle Plugin in `libs.versions.toml`

### Runtime Crashes
1. Check Android logcat for exception messages
2. Verify all UI elements exist in layout XML
3. Ensure Firebase initialization completes before sending commands

## Testing Checklist

### Before First Run
- [ ] `google-services.json` file in place
- [ ] Firebase emulator line removed/uncommented correctly
- [ ] Gradle sync successful
- [ ] No compilation errors

### During Testing
- [ ] Connection status shows green "Connected"
- [ ] Motor buttons move robot (hold to move)
- [ ] Robot stops when button released
- [ ] Stop button works immediately
- [ ] Servo sliders update robot arm
- [ ] Feedback display updates in real-time
- [ ] Sensor data appears correctly

### Safety Tests
- [ ] Emergency stop button works
- [ ] Robot stops when app closes
- [ ] Robot stops when connection lost
- [ ] Robot stops when button released

## Performance Notes

### Real-Time Communication
- **Command latency**: < 100ms (depends on internet)
- **Feedback updates**: Real-time via Firebase listeners
- **No artificial delays**: All communication is instant
- **Auto-reconnection**: Firebase handles connection drops

### Network Requirements
- Stable internet connection required for both app and RPi
- Recommended: Wi-Fi connection for lowest latency
- Cellular data works but may have higher latency
- Local network mode: RPi and phone on same Wi-Fi

## Advanced Configuration

### Change Update Rates
In `MainActivity.java`, modify:
```java
// Servo update rate is instant (on slider change)
// To add throttling, use Handler.postDelayed()
```

### Custom Motor Speeds
In `setupMotorControls()`, modify speed values:
```java
sendMotorCommand("FORWARD", 200);  // Change 200 to desired speed
sendMotorCommand("LEFT", 150);     // Change 150 to desired turn speed
```

### Add More Servos
1. Add slider in `activity_main.xml`
2. Add listener in `setupServoControls()`
3. Call `sendServoCommand(servoNum, angle)`

### Customize Feedback Display
In `displayFeedback()`, add custom feedback parsing:
```java
// Example: Display custom sensor data
Object customData = feedback.get("custom_sensor");
if (customData != null) {
    sb.append("Custom: ").append(customData).append("\n");
}
```

## Security Considerations

### Firebase Security Rules
Update rules in Firebase Console:
```json
{
  "rules": {
    "robosort": {
      "commands": {
        ".read": true,
        ".write": true
      },
      "feedback": {
        ".read": true,
        ".write": true
      }
    }
  }
}
```

**For Production**: Add authentication and restrict access:
```json
{
  "rules": {
    "robosort": {
      ".read": "auth != null",
      ".write": "auth != null"
    }
  }
}
```

### App Permissions
Minimal permissions required (already configured):
- `INTERNET`: Firebase communication
- `ACCESS_NETWORK_STATE`: Connection status

## Support
- Check Firebase Console for real-time database activity
- Use Android Studio Logcat for debugging
- Monitor RPi console for command reception
- Refer to `FIREBASE_STRUCTURE.md` for data format details
