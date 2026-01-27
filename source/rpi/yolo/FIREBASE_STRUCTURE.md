# Firebase Realtime Database Structure

## JSON Structure for RoboSort RC Control

```json
{
  "robosort": {
    "commands": {
      "motor": "STOP",
      "servo1": 90,
      "servo2": 90,
      "servo3": 90,
      "servo4": 90,
      "timestamp": 1234567890
    },
    "status": {
      "motor_state": "STOP",
      "servo1": 90,
      "servo2": 90,
      "servo3": 90,
      "servo4": 90,
      "connected": true,
      "last_update": 1234567890
    }
  }
}
```

## Data Flow

### Commands Path: `/robosort/commands`
**Written by:** Android App
**Read by:** Raspberry Pi (streaming listener)

- `motor`: String - One of: "FORWARD", "BACKWARD", "TURN_LEFT", "TURN_RIGHT", "STOP"
- `servo1`: Integer (0-180) - Servo 1 angle
- `servo2`: Integer (0-180) - Servo 2 angle
- `servo3`: Integer (0-180) - Servo 3 angle
- `servo4`: Integer (0-180) - Servo 4 angle
- `timestamp`: Long - Timestamp of command

### Status Path: `/robosort/status`
**Written by:** Raspberry Pi
**Read by:** Android App (streaming listener)

- `motor_state`: String - Current motor state
- `servo1`: Integer - Current servo 1 angle
- `servo2`: Integer - Current servo 2 angle
- `servo3`: Integer - Current servo 3 angle
- `servo4`: Integer - Current servo 4 angle
- `connected`: Boolean - RPi connection status
- `last_update`: Long - Last status update timestamp

## Firebase Rules (Recommended)

```json
{
  "rules": {
    "robosort": {
      "commands": {
        ".write": true,
        ".read": true
      },
      "status": {
        ".write": true,
        ".read": true
      }
    }
  }
}
```

## Android Implementation Notes

### Sending Commands (Press Event)
```java
// When button is PRESSED
DatabaseReference commandsRef = database.getReference("robosort/commands");
commandsRef.child("motor").setValue("FORWARD");
commandsRef.child("timestamp").setValue(ServerValue.TIMESTAMP);
```

### Sending Stop (Release Event)
```java
// When button is RELEASED
DatabaseReference commandsRef = database.getReference("robosort/commands");
commandsRef.child("motor").setValue("STOP");
commandsRef.child("timestamp").setValue(ServerValue.TIMESTAMP);
```

### Listening to Status
```java
DatabaseReference statusRef = database.getReference("robosort/status");
statusRef.addValueEventListener(new ValueEventListener() {
    @Override
    public void onDataChange(DataSnapshot dataSnapshot) {
        String motorState = dataSnapshot.child("motor_state").getValue(String.class);
        Integer servo1 = dataSnapshot.child("servo1").getValue(Integer.class);
        // Update UI with current status
    }
});
```

## Real-Time Behavior

### Button Press/Release Pattern

1. **User presses FORWARD button:**
   - Android writes: `commands/motor = "FORWARD"`
   - RPi receives instantly (Firebase stream)
   - RPi sends to Arduino: `M:F\n`
   - Arduino starts motors
   - Arduino confirms: `M:FORWARD`
   - RPi writes: `status/motor_state = "FORWARD"`
   - Android UI updates to show "FORWARD"

2. **User releases FORWARD button:**
   - Android writes: `commands/motor = "STOP"`
   - RPi receives instantly
   - RPi sends to Arduino: `M:S\n`
   - Arduino stops motors
   - Arduino confirms: `M:STOP`
   - RPi writes: `status/motor_state = "STOP"`
   - Android UI updates to show "STOP"

### No Delays
- Firebase streams provide near-instant updates (<100ms typical)
- Serial communication at 115200 baud is fast (~1ms per command)
- No `time.sleep()` or artificial delays anywhere
- Event-driven architecture ensures immediate response
