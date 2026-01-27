# RoboSort Command Protocol Reference

## Complete Command Flow

```
Android App → Firebase RTDB → Raspberry Pi → Arduino → Robot
```

## Motor Commands

### Android → Firebase
**Format:** JSON object
```json
{
  "direction": "FORWARD",
  "speed": 255
}
```

**Commands:**
- `FORWARD` - Move forward (speed: 255)
- `BACKWARD` - Move backward (speed: 255)
- `TURN_LEFT` - Spot turn left (rotate in place, speed: 255)
- `TURN_RIGHT` - Spot turn right (rotate in place, speed: 255)
- `STOP` - Stop all motors (speed: 0)

### RPi → Arduino
**Format:** Text command
```
FORWARD:255
BACKWARD:255
TURN_LEFT:255
TURN_RIGHT:255
MSTOP
```

### Arduino Processing
Arduino expects:
```cpp
FORWARD:<speed>    // Both motors forward
BACKWARD:<speed>   // Both motors backward
TURN_LEFT:<speed>  // Spot turn left (left motor back, right motor forward)
TURN_RIGHT:<speed> // Spot turn right (left motor forward, right motor back)
MSTOP              // Stop all motors
```

**Speed Range:** 0-255 (PWM duty cycle)

## Servo Commands

### Android → Firebase
**Format:** JSON object per servo
```json
{
  "servo": 1,
  "angle": 90
}
```

**Servos:**
1. Servo 1 - Arm Base (0-180°)
2. Servo 2 - Arm Shoulder (0-180°)
3. Servo 3 - Arm Elbow (0-180°)
4. Servo 4 - Gripper (0-180°)

### RPi → Arduino
**Format:** Text command
```
S1 90
S2 90
S3 90
S4 90
```

### Arduino Processing
Arduino expects:
```cpp
S<servo> <angle>
```

Examples:
- `S1 90` - Set servo 1 to 90 degrees
- `S2 45` - Set servo 2 to 45 degrees
- `S3 180` - Set servo 3 to 180 degrees
- `S4 0` - Set servo 4 to 0 degrees (gripper open)

**Angle Range:** 0-180 degrees

## Firebase Database Structure

### Commands Path: `robosort/commands/`

**Motor Command:**
```json
{
  "robosort": {
    "commands": {
      "motor": {
        "type": "motor",
        "direction": "FORWARD",
        "speed": 255,
        "timestamp": 1706313600000
      }
    }
  }
}
```

**Servo Commands:**
```json
{
  "robosort": {
    "commands": {
      "servo1": {
        "type": "servo",
        "servo": 1,
        "angle": 90,
        "timestamp": 1706313600000
      },
      "servo2": {
        "type": "servo",
        "servo": 2,
        "angle": 90,
        "timestamp": 1706313600000
      }
    }
  }
}
```

### Feedback Path: `robosort/feedback/`

```json
{
  "robosort": {
    "feedback": {
      "timestamp": 1706313600000,
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

## Speed Settings

### Current Configuration (All at 255 - Full Power)

**Forward/Backward:** 255 (100% duty cycle)
- Maximum speed for straight movement
- Both motors at full power

**Turning (Spot Turn):** 255 (100% duty cycle)
- Maximum rotation speed
- One motor forward, one backward at full power

### Customizing Speeds

**In Android App** ([MainActivity.java](source/android/RoboSortControl/app/src/main/java/com/qppd/robosortcontrol/MainActivity.java)):
```java
// Line 122: Forward speed
sendMotorCommand("FORWARD", 255);  // Change 255 to desired speed

// Line 135: Backward speed
sendMotorCommand("BACKWARD", 255); // Change 255 to desired speed

// Line 148: Left turn speed
sendMotorCommand("TURN_LEFT", 255); // Change 255 to desired speed

// Line 161: Right turn speed
sendMotorCommand("TURN_RIGHT", 255); // Change 255 to desired speed
```

**Speed Recommendations:**
- **Slow/Precise:** 100-150 (40-60% power)
- **Medium:** 150-200 (60-80% power)
- **Fast:** 200-255 (80-100% power)
- **Maximum:** 255 (100% power) ← Current setting

## Command Examples

### Complete Flow Example: Forward Movement

**Step 1: Android sends to Firebase**
```java
// User presses and holds FORWARD button
Map<String, Object> command = new HashMap<>();
command.put("type", "motor");
command.put("direction", "FORWARD");
command.put("speed", 255);
commandsRef.child("motor").setValue(command);
```

**Step 2: Firebase stores**
```json
{
  "robosort/commands/motor": {
    "type": "motor",
    "direction": "FORWARD",
    "speed": 255
  }
}
```

**Step 3: RPi receives and processes**
```python
def on_command_change(self, message):
    data = message["data"]
    if "motor" in data:
        command = data["motor"]
        self.send_motor_command(command)

def send_motor_command(self, command):
    direction = command.get("direction", "STOP")
    speed = command.get("speed", 0)
    arduino_cmd = f"{direction}:{speed}\n"  # Result: "FORWARD:255\n"
    self.arduino.write(arduino_cmd.encode())
```

**Step 4: Arduino receives via Serial**
```
FORWARD:255
```

**Step 5: Arduino processes**
```cpp
if (input.startsWith("FORWARD:")) {
    int speed = input.substring(8).toInt(); // Extract 255
    dcConfig.moveForward(speed);
    Serial.print("Moving forward at speed ");
    Serial.println(speed);
}
```

**Step 6: Robot moves forward at full speed**

### Complete Flow Example: Servo Movement

**Step 1: Android sends to Firebase**
```java
// User drags Servo 1 slider to 45 degrees
Map<String, Object> command = new HashMap<>();
command.put("type", "servo");
command.put("servo", 1);
command.put("angle", 45);
commandsRef.child("servo1").setValue(command);
```

**Step 2: Firebase stores**
```json
{
  "robosort/commands/servo1": {
    "type": "servo",
    "servo": 1,
    "angle": 45
  }
}
```

**Step 3: RPi receives and processes**
```python
def on_command_change(self, message):
    data = message["data"]
    if "servo1" in data:
        self.send_servo_command(1, data["servo1"])

def send_servo_command(self, servo_num, command_data):
    angle = command_data.get("angle", 90)
    arduino_cmd = f"S{servo_num} {angle}\n"  # Result: "S1 45\n"
    self.arduino.write(arduino_cmd.encode())
```

**Step 4: Arduino receives via Serial**
```
S1 45
```

**Step 5: Arduino processes**
```cpp
if (input.startsWith("S")) {
    int servo = input.substring(1, 2).toInt();  // Extract 1
    int angle = input.substring(3).toInt();     // Extract 45
    servoConfig.setAngle(servo, angle);
    Serial.print("Servo ");
    Serial.print(servo);
    Serial.print(" set to ");
    Serial.println(angle);
}
```

**Step 6: Servo moves to 45 degrees**

## Troubleshooting

### Commands Not Working

**Check Android App:**
- Verify connection status shows green "Connected"
- Check Firebase Console → Database → Commands path
- Commands should appear when buttons pressed

**Check Firebase:**
- Open Firebase Console → Realtime Database
- Navigate to `robosort/commands/motor`
- Should see commands updating in real-time

**Check RPi:**
- Look for: `Sent to Arduino: FORWARD:255`
- If not appearing, Firebase connection issue
- If appearing but robot not moving, check Arduino

**Check Arduino:**
- Open Serial Monitor (115200 baud)
- Should see: `Moving forward at speed 255`
- If not appearing, check USB connection
- Verify baud rate matches (115200)

### Wrong Command Format

**Symptoms:**
- Arduino receives commands but doesn't respond
- Error messages in Arduino Serial Monitor

**Common Issues:**
1. **Extra characters:** Check for spaces, newlines
2. **Wrong separator:** Arduino expects `:` for motors, space for servos
3. **Case sensitivity:** Commands are case-sensitive (FORWARD not forward)

**Debug Commands:**

Add to Arduino setup():
```cpp
Serial.println("Ready to receive commands");
```

Add to Arduino loop():
```cpp
if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    Serial.print("Received: [");
    Serial.print(input);
    Serial.println("]");
    // ... rest of processing
}
```

### Speed Issues

**Robot too fast:**
- Reduce speed values in Android app
- Example: Change 255 to 200 or 150

**Robot too slow:**
- Increase speed values
- Check battery voltage (low battery = slow)
- Verify motor driver connections

**Uneven movement:**
- May need motor calibration
- Adjust individual motor speeds in Arduino code

## Protocol Summary

| Component | Motor Format | Servo Format | Speed Range |
|-----------|-------------|--------------|-------------|
| **Android** | JSON: `{direction, speed}` | JSON: `{servo, angle}` | 0-255 |
| **Firebase** | Same as Android | Same as Android | 0-255 |
| **RPi** | Text: `FORWARD:255` | Text: `S1 90` | 0-255 |
| **Arduino** | Expects: `FORWARD:255` | Expects: `S1 90` | 0-255 |

## Current Configuration

✅ All commands aligned across entire system
✅ Motor speeds set to 255 (full power)
✅ Spot turning uses TURN_LEFT/TURN_RIGHT
✅ Servo commands use correct format
✅ Commands flow correctly through all layers

The system is now fully synchronized!
