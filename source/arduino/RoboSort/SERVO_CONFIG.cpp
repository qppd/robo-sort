#include "SERVO_CONFIG.h"
#include <Wire.h>

// Typical min/max pulse lengths for servos
#define SERVO_MIN_PULSE  150
#define SERVO_MAX_PULSE  600

// Continuous servo timing (approximate for 360° servos)
// One full rotation takes about 1-2 seconds depending on servo
#define ROTATION_TIME_MS 1500  // Time for one full rotation

ServoConfig::ServoConfig() : pwm(0x40), _continuousTest(false), _lastMoveTime(0), _currentAngle(0), _direction(1), 
                           _lifterMoving(false), _lifterGoingUp(false), _lifterStartTime(0), _lifterRotations(0), _lifterSpeed(0) {
    // Default channels for 5 servos (0-4)
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
        servoChannels[i] = i;
        servoTypes[i] = POSITION_SERVO; // Default to position servos
    }
    
    // Configure servo 0 as continuous rotation servo
    servoTypes[0] = CONTINUOUS_SERVO;
}

void ServoConfig::begin() {
    pinMode(SERVO_OE_PIN, OUTPUT);
    digitalWrite(SERVO_OE_PIN, HIGH); // Disable servos at startup
    pwm.begin();
    pwm.setPWMFreq(50); // Analog servos run at ~50 Hz
    
    // Ensure continuous servo is stopped at startup
    setContinuousSpeed(0, 0);
}

void ServoConfig::setServoAngle(uint8_t servo, uint16_t angle) {
    if (servo >= NUM_SERVOS) return;
    
    if (servoTypes[servo] == CONTINUOUS_SERVO) {
        // For continuous servos, treat angle as speed control
        // 0 = full speed one direction, 90 = stop, 180 = full speed other direction
        int8_t speed = map(angle, 0, 180, -100, 100);
        setContinuousSpeed(servo, speed);
    } else {
        // Standard position servo
        uint16_t pulse = angleToPulse(angle);
        pwm.setPWM(servoChannels[servo], 0, pulse);
    }
}

void ServoConfig::setContinuousSpeed(uint8_t servo, int8_t speed) {
    if (servo >= NUM_SERVOS || servoTypes[servo] != CONTINUOUS_SERVO) return;
    
    uint16_t pulse = speedToPulse(speed);
    pwm.setPWM(servoChannels[servo], 0, pulse);
}

uint16_t ServoConfig::angleToPulse(uint16_t angle) {
    // Map 0-180 degrees to SERVO_MIN_PULSE - SERVO_MAX_PULSE
    return map(angle, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
}

uint16_t ServoConfig::speedToPulse(int8_t speed) {
    // Map -100 to +100 speed to pulse width
    // 90 (1.5ms) = stop, <90 = CCW, >90 = CW
    return map(speed, -100, 100, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
}

void ServoConfig::lifterUp() {
    if (_lifterMoving) return; // Already moving
    
    _lifterMoving = true;
    _lifterGoingUp = true;
    _lifterStartTime = millis();
    _lifterRotations = 0;
    _lifterSpeed = 80; // Medium speed up
    setContinuousSpeed(0, _lifterSpeed); // Servo 0
    Serial.println("Lifter moving UP - will stop when ARM limit switch is pressed");
}

void ServoConfig::lifterDown() {
    if (_lifterMoving) return; // Already moving
    
    _lifterMoving = true;
    _lifterGoingUp = false;
    _lifterStartTime = millis();
    _lifterRotations = 0;
    _lifterSpeed = -80; // Medium speed down (negative = opposite direction)
    setContinuousSpeed(0, _lifterSpeed); // Servo 0
    Serial.println("Lifter moving DOWN - 3 rotations then stop");
}

void ServoConfig::lifterStop() {
    setContinuousSpeed(0, 0); // Stop servo immediately
    _lifterMoving = false;
    Serial.println("Lifter emergency stop!");
}

void ServoConfig::testServos() {
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
        if (servoTypes[i] == CONTINUOUS_SERVO) {
            // For continuous servos, just test stop and slow speeds
            setContinuousSpeed(i, 0);   // Stop
            delay(500);
            setContinuousSpeed(i, 30);  // Slow one direction
            delay(1000);
            setContinuousSpeed(i, -30); // Slow other direction
            delay(1000);
            setContinuousSpeed(i, 0);   // Stop
            delay(500);
        } else {
            // Standard position servo test
            setServoAngle(i, 0);
            delay(500);
            setServoAngle(i, 90);
            delay(500);
            setServoAngle(i, 180);
            delay(500);
            setServoAngle(i, 90);
            delay(500);
        }
    }
}

void ServoConfig::enableServos() {
    digitalWrite(SERVO_OE_PIN, LOW);
    // Ensure all servos are in safe state when enabled
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
        if (servoTypes[i] == CONTINUOUS_SERVO) {
            setContinuousSpeed(i, 0); // Stop continuous servos
        } else {
            setServoAngle(i, 90); // Center position servos
        }
    }
}

void ServoConfig::disableServos() {
    digitalWrite(SERVO_OE_PIN, HIGH);
}

void ServoConfig::startContinuousTest() {
    enableServos(); // Ensure servos are enabled
    _continuousTest = true;
    _currentAngle = 0;
    _direction = 1;
    _lastMoveTime = millis();
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
        if (servoTypes[i] == POSITION_SERVO) { // Only test position servos
            setServoAngle(i, _currentAngle);
        }
    }
}

void ServoConfig::stopContinuousTest() {
    _continuousTest = false;
}

void ServoConfig::update() {
    // Handle continuous test
    if (_continuousTest) {
        unsigned long now = millis();
        if (now - _lastMoveTime >= 5000) { // Change direction every 5 seconds
            _direction = -_direction;
            _currentAngle = (_direction == 1) ? 0 : 180;
            for (uint8_t i = 0; i < NUM_SERVOS; i++) {
                if (servoTypes[i] == POSITION_SERVO) {
                    setServoAngle(i, _currentAngle);
                }
            }
            _lastMoveTime = now;
        }
    }
    
    // Handle lifter movement timing
    if (_lifterMoving) {
        if (_lifterGoingUp) {
            // Check ARM limit switch for UP direction
            bool armState = digitalRead(ARM_LIMIT_PIN);
            if (armState == LOW) { // Limit switch pressed
                setContinuousSpeed(0, 0); // Stop servo
                _lifterMoving = false;
                Serial.println("Lifter stopped - ARM limit switch pressed!");
            }
        } else {
            // For DOWN direction, use 3-rotation timing
            unsigned long now = millis();
            unsigned long elapsed = now - _lifterStartTime;
            
            // Check if one rotation is complete
            if (elapsed >= ROTATION_TIME_MS) {
                _lifterRotations++;
                _lifterStartTime = now;
                
                if (_lifterRotations >= 3) {
                    // Stop after 3 rotations
                    setContinuousSpeed(0, 0); // Stop servo
                    _lifterMoving = false;
                    Serial.print("Lifter stopped after ");
                    Serial.print(_lifterRotations);
                    Serial.println(" rotations");
                }
            }
        }
    }
}
