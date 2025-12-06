#include "DC_CONFIG.h"

DCConfig::DCConfig() : _continuousTest(false), _lastChangeTime(0), _direction(FORWARD) {
    // Constructor
}

void DCConfig::begin() {
    // Initialize motor control pins
    pinMode(MOTOR_A_PWM, OUTPUT);
    pinMode(MOTOR_A_DIR, OUTPUT);
    pinMode(MOTOR_A_WD, OUTPUT);
    
    pinMode(MOTOR_B_PWM, OUTPUT);
    pinMode(MOTOR_B_DIR, OUTPUT);
    pinMode(MOTOR_B_WD, OUTPUT);
    
    // Initialize watchdog pins low
    digitalWrite(MOTOR_A_WD, LOW);
    digitalWrite(MOTOR_B_WD, LOW);
    
    // Stop all motors initially
    stopAll();
}

void DCConfig::setMotorPins(uint8_t motor, uint8_t dirState, uint8_t pwmValue) {
    if (motor == MOTOR_A) {
        digitalWrite(MOTOR_A_DIR, dirState);
        analogWrite(MOTOR_A_PWM, pwmValue);
    } else if (motor == MOTOR_B) {
        digitalWrite(MOTOR_B_DIR, dirState);
        analogWrite(MOTOR_B_PWM, pwmValue);
    }
}

void DCConfig::setMotorSpeed(uint8_t motor, int speed) {
    // Constrain speed to valid range (-255 to 255)
    speed = constrain(speed, -255, 255);
    
    if (speed > 0) {
        // Forward
        setMotorDirection(motor, FORWARD);
        if (motor == MOTOR_A) {
            analogWrite(MOTOR_A_PWM, speed);
        } else if (motor == MOTOR_B) {
            analogWrite(MOTOR_B_PWM, speed);
        }
    } else if (speed < 0) {
        // Backward
        setMotorDirection(motor, BACKWARD);
        if (motor == MOTOR_A) {
            analogWrite(MOTOR_A_PWM, abs(speed));
        } else if (motor == MOTOR_B) {
            analogWrite(MOTOR_B_PWM, abs(speed));
        }
    } else {
        // Stop
        stopMotor(motor);
    }
}

void DCConfig::setMotorDirection(uint8_t motor, uint8_t direction) {
    if (direction == FORWARD) {
        setMotorPins(motor, HIGH, 0);
    } else if (direction == BACKWARD) {
        setMotorPins(motor, LOW, 0);
    } else if (direction == BRAKE) {
        brakeMotor(motor);
    } else {
        setMotorPins(motor, LOW, 0);
    }
}

void DCConfig::moveMotor(uint8_t motor, uint8_t direction, uint8_t speed) {
    speed = constrain(speed, 0, 255);
    
    if (direction == FORWARD) {
        setMotorPins(motor, HIGH, speed);
    } else if (direction == BACKWARD) {
        setMotorPins(motor, LOW, speed);
    } else if (direction == BRAKE) {
        brakeMotor(motor);
    } else {
        stopMotor(motor);
    }
}

void DCConfig::stopMotor(uint8_t motor) {
    setMotorPins(motor, LOW, 0);
}

void DCConfig::brakeMotor(uint8_t motor) {
    // Since no dedicated brake pin, use high PWM for braking
    if (motor == MOTOR_A) {
        analogWrite(MOTOR_A_PWM, 255);
        delay(100); // Short brake pulse
        analogWrite(MOTOR_A_PWM, 0);
    } else if (motor == MOTOR_B) {
        analogWrite(MOTOR_B_PWM, 255);
        delay(100);
        analogWrite(MOTOR_B_PWM, 0);
    }
}

void DCConfig::updateWatchdog() {
    static unsigned long lastToggleA = 0;
    static unsigned long lastToggleB = 0;
    unsigned long now = millis();
    
    // Toggle watchdog for Motor A every 500ms
    if (now - lastToggleA >= 500) {
        digitalWrite(MOTOR_A_WD, !digitalRead(MOTOR_A_WD));
        lastToggleA = now;
    }
    
    // Toggle watchdog for Motor B every 500ms
    if (now - lastToggleB >= 500) {
        digitalWrite(MOTOR_B_WD, !digitalRead(MOTOR_B_WD));
        lastToggleB = now;
    }
}

void DCConfig::testMotors() {
    Serial.println("Testing Motor A...");
    
    // Motor A Forward
    Serial.println("Motor A: Forward at 150 speed");
    moveMotor(MOTOR_A, FORWARD, 150);
    delay(2000);
    
    // Motor A Backward
    Serial.println("Motor A: Backward at 150 speed");
    moveMotor(MOTOR_A, BACKWARD, 150);
    delay(2000);
    
    // Motor A Stop
    Serial.println("Motor A: Stop");
    stopMotor(MOTOR_A);
    delay(1000);
    
    Serial.println("Testing Motor B...");
    
    // Motor B Forward
    Serial.println("Motor B: Forward at 150 speed");
    moveMotor(MOTOR_B, FORWARD, 150);
    delay(2000);
    
    // Motor B Backward
    Serial.println("Motor B: Backward at 150 speed");
    moveMotor(MOTOR_B, BACKWARD, 150);
    delay(2000);
    
    // Motor B Stop
    Serial.println("Motor B: Stop");
    stopMotor(MOTOR_B);
    delay(1000);
    
    Serial.println("Testing both motors...");
    
    // Both motors forward
    Serial.println("Both motors: Forward at 180 speed");
    moveMotor(MOTOR_A, FORWARD, 180);
    moveMotor(MOTOR_B, FORWARD, 180);
    delay(2000);
    
    // Both motors backward
    Serial.println("Both motors: Backward at 180 speed");
    moveMotor(MOTOR_A, BACKWARD, 180);
    moveMotor(MOTOR_B, BACKWARD, 180);
    delay(2000);
    
    // Stop all
    Serial.println("All motors: Stop");
    stopAll();
    
    Serial.println("Motor test complete!");
}

void DCConfig::startContinuousTest() {
    _continuousTest = true;
    _direction = FORWARD;
    _lastChangeTime = millis();
    moveMotor(MOTOR_A, _direction, 150);
    moveMotor(MOTOR_B, _direction, 150);
    Serial.println("Continuous motor test started.");
}

void DCConfig::stopContinuousTest() {
    _continuousTest = false;
    stopAll();
    Serial.println("Continuous motor test stopped.");
}

void DCConfig::update() {
    if (_continuousTest) {
        unsigned long now = millis();
        if (now - _lastChangeTime >= 2000) { // Change direction every 2 seconds
            _direction = (_direction == FORWARD) ? BACKWARD : FORWARD;
            moveMotor(MOTOR_A, _direction, 150);
            moveMotor(MOTOR_B, _direction, 150);
            _lastChangeTime = now;
        }
    }
}

void DCConfig::stopAll() {
    stopMotor(MOTOR_A);
    stopMotor(MOTOR_B);
}
