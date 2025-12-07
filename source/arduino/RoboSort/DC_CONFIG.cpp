#include "DC_CONFIG.h"

DCConfig::DCConfig() : _continuousTest(false), _lastChangeTime(0), _direction(FORWARD) {
    // Constructor
}

void DCConfig::begin() {
    // Initialize L298N Module 1 (Motor A) pins
    pinMode(MOTOR_A_IN1, OUTPUT);
    pinMode(MOTOR_A_IN2, OUTPUT);
    pinMode(MOTOR_A_ENA, OUTPUT);
    
    // Initialize L298N Module 2 (Motor B) pins
    pinMode(MOTOR_B_IN1, OUTPUT);
    pinMode(MOTOR_B_IN2, OUTPUT);
    pinMode(MOTOR_B_ENB, OUTPUT);
    
    // Stop all motors initially
    stopAll();
}

void DCConfig::setMotorPins(uint8_t motor, uint8_t in1State, uint8_t in2State, uint8_t pwmValue) {
    if (motor == MOTOR_A) {
        digitalWrite(MOTOR_A_IN1, in1State);
        digitalWrite(MOTOR_A_IN2, in2State);
        analogWrite(MOTOR_A_ENA, pwmValue);
    } else if (motor == MOTOR_B) {
        digitalWrite(MOTOR_B_IN1, in1State);
        digitalWrite(MOTOR_B_IN2, in2State);
        analogWrite(MOTOR_B_ENB, pwmValue);
    }
}

void DCConfig::setMotorSpeed(uint8_t motor, int speed) {
    // Constrain speed to valid range (-255 to 255)
    speed = constrain(speed, -255, 255);
    
    if (speed > 0) {
        // Forward: IN1=HIGH, IN2=LOW
        setMotorPins(motor, HIGH, LOW, speed);
    } else if (speed < 0) {
        // Backward: IN1=LOW, IN2=HIGH
        setMotorPins(motor, LOW, HIGH, abs(speed));
    } else {
        // Stop: IN1=LOW, IN2=LOW
        stopMotor(motor);
    }
}

void DCConfig::setMotorDirection(uint8_t motor, uint8_t direction) {
    if (direction == FORWARD) {
        // Forward: IN1=HIGH, IN2=LOW, PWM=0 (will be set by speed)
        setMotorPins(motor, HIGH, LOW, 0);
    } else if (direction == BACKWARD) {
        // Backward: IN1=LOW, IN2=HIGH, PWM=0 (will be set by speed)
        setMotorPins(motor, LOW, HIGH, 0);
    } else if (direction == BRAKE) {
        brakeMotor(motor);
    } else {
        // Stop: IN1=LOW, IN2=LOW
        stopMotor(motor);
    }
}

void DCConfig::moveMotor(uint8_t motor, uint8_t direction, uint8_t speed) {
    speed = constrain(speed, 0, 255);
    
    if (direction == FORWARD) {
        // Forward: IN1=HIGH, IN2=LOW
        setMotorPins(motor, HIGH, LOW, speed);
    } else if (direction == BACKWARD) {
        // Backward: IN1=LOW, IN2=HIGH
        setMotorPins(motor, LOW, HIGH, speed);
    } else if (direction == BRAKE) {
        brakeMotor(motor);
    } else {
        // Stop: IN1=LOW, IN2=LOW
        stopMotor(motor);
    }
}

void DCConfig::stopMotor(uint8_t motor) {
    // Stop: IN1=LOW, IN2=LOW, PWM=0
    setMotorPins(motor, LOW, LOW, 0);
}

void DCConfig::brakeMotor(uint8_t motor) {
    // Brake: IN1=HIGH, IN2=HIGH, PWM=255
    setMotorPins(motor, HIGH, HIGH, 255);
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
