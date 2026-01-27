# Arduino Serial Protocol for RoboSort RC Control

## Communication Settings
- **Baud Rate:** 115200
- **Format:** 8N1 (8 data bits, no parity, 1 stop bit)
- **Line Ending:** `\n` (newline)

## Commands (Raspberry Pi → Arduino)

### Motor Commands
Format: `M:<command>\n`

| Command | Description | Example |
|---------|-------------|---------|
| `M:F\n` | Move forward | `M:F\n` |
| `M:B\n` | Move backward | `M:B\n` |
| `M:L\n` | Turn left | `M:L\n` |
| `M:R\n` | Turn right | `M:R\n` |
| `M:S\n` | Stop all motors | `M:S\n` |

### Servo Commands
Format: `S:<servo_id>:<angle>\n`

| Parameter | Type | Range | Description |
|-----------|------|-------|-------------|
| servo_id | Integer | 1-4 | Servo number |
| angle | Integer | 0-180 | Target angle in degrees |

**Examples:**
- `S:1:90\n` - Set servo 1 to 90 degrees
- `S:2:45\n` - Set servo 2 to 45 degrees
- `S:3:180\n` - Set servo 3 to 180 degrees
- `S:4:0\n` - Set servo 4 to 0 degrees

## Feedback (Arduino → Raspberry Pi)

### Motor State Feedback
Format: `M:<state>\n`

**Examples:**
- `M:FORWARD\n` - Motors are moving forward
- `M:BACKWARD\n` - Motors are moving backward
- `M:LEFT\n` - Motors are turning left
- `M:RIGHT\n` - Motors are turning right
- `M:STOP\n` - Motors are stopped

### Servo Position Feedback
Format: `S:<servo_id>:<angle>\n`

**Examples:**
- `S:1:90\n` - Servo 1 is at 90 degrees
- `S:2:45\n` - Servo 2 is at 45 degrees

## Arduino Implementation Example

```cpp
void setup() {
    Serial.begin(115200);
    // Initialize motors and servos
}

void loop() {
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        
        if (command.startsWith("M:")) {
            handleMotorCommand(command);
        }
        else if (command.startsWith("S:")) {
            handleServoCommand(command);
        }
    }
}

void handleMotorCommand(String cmd) {
    char action = cmd.charAt(2);
    
    switch(action) {
        case 'F':
            // Move forward
            digitalWrite(MOTOR_L_FWD, HIGH);
            digitalWrite(MOTOR_L_BWD, LOW);
            digitalWrite(MOTOR_R_FWD, HIGH);
            digitalWrite(MOTOR_R_BWD, LOW);
            Serial.println("M:FORWARD");
            break;
            
        case 'B':
            // Move backward
            digitalWrite(MOTOR_L_FWD, LOW);
            digitalWrite(MOTOR_L_BWD, HIGH);
            digitalWrite(MOTOR_R_FWD, LOW);
            digitalWrite(MOTOR_R_BWD, HIGH);
            Serial.println("M:BACKWARD");
            break;
            
        case 'L':
            // Turn left
            digitalWrite(MOTOR_L_FWD, LOW);
            digitalWrite(MOTOR_L_BWD, HIGH);
            digitalWrite(MOTOR_R_FWD, HIGH);
            digitalWrite(MOTOR_R_BWD, LOW);
            Serial.println("M:LEFT");
            break;
            
        case 'R':
            // Turn right
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
    // Format: S:servo_id:angle
    int firstColon = cmd.indexOf(':');
    int secondColon = cmd.indexOf(':', firstColon + 1);
    
    int servoId = cmd.substring(firstColon + 1, secondColon).toInt();
    int angle = cmd.substring(secondColon + 1).toInt();
    
    // Clamp angle
    angle = constrain(angle, 0, 180);
    
    // Set servo position
    switch(servoId) {
        case 1:
            servo1.write(angle);
            break;
        case 2:
            servo2.write(angle);
            break;
        case 3:
            servo3.write(angle);
            break;
        case 4:
            servo4.write(angle);
            break;
    }
    
    // Send confirmation
    Serial.print("S:");
    Serial.print(servoId);
    Serial.print(":");
    Serial.println(angle);
}
```

## Timing Considerations

- **No delays in command processing:** Commands should be handled immediately
- **Feedback should be sent after action:** Send confirmation only after motors/servos are set
- **Buffer management:** Use `Serial.readStringUntil('\n')` to avoid partial commands
- **Command validation:** Always validate servo IDs and angles before applying

## Error Handling

- Invalid commands should be ignored (no response)
- Out-of-range servo angles should be clamped to 0-180
- If servo ID is invalid (not 1-4), ignore the command
- Motors should always respond to STOP command regardless of state
