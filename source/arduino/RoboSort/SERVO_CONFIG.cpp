#include "SERVO_CONFIG.h"
#include <Wire.h>

// Typical min/max pulse lengths for servos
#define SERVO_MIN_PULSE  150
#define SERVO_MAX_PULSE  600

ServoConfig::ServoConfig() : pwm(0x40), _continuousTest(false), _lastMoveTime(0), _currentAngle(0), _direction(1) {
    // Default channels for 5 servos (0-4)
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
        servoChannels[i] = i;
    }
}

void ServoConfig::begin() {
    pinMode(SERVO_OE_PIN, OUTPUT);
    digitalWrite(SERVO_OE_PIN, HIGH); // Disable servos at startup
    pwm.begin();
    pwm.setPWMFreq(50); // Analog servos run at ~50 Hz
}

void ServoConfig::setServoAngle(uint8_t servo, uint16_t angle) {
    if (servo >= NUM_SERVOS) return;
    uint16_t pulse = angleToPulse(angle);
    pwm.setPWM(servoChannels[servo], 0, pulse);
}

uint16_t ServoConfig::angleToPulse(uint16_t angle) {
    // Map 0-180 degrees to SERVO_MIN_PULSE - SERVO_MAX_PULSE
    return map(angle, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
}

void ServoConfig::testServos() {
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
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

void ServoConfig::enableServos() {
    digitalWrite(SERVO_OE_PIN, LOW);
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
        setServoAngle(i, _currentAngle);
    }
}

void ServoConfig::stopContinuousTest() {
    _continuousTest = false;
}

void ServoConfig::update() {
    if (_continuousTest) {
        unsigned long now = millis();
        if (now - _lastMoveTime >= 5000) { // Change direction every 5 seconds
            _direction = -_direction;
            _currentAngle = (_direction == 1) ? 0 : 180;
            for (uint8_t i = 0; i < NUM_SERVOS; i++) {
                setServoAngle(i, _currentAngle);
            }
            _lastMoveTime = now;
        }
    }
}
