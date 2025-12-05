#ifndef SERVO_CONFIG_H
#define SERVO_CONFIG_H

#include <Adafruit_PWMServoDriver.h>
#include "PINS.h"

#define NUM_SERVOS 6

class ServoConfig {
public:
    ServoConfig();
    void begin();
    void setServoAngle(uint8_t servo, uint16_t angle);
    void testServos();
    void enableServos();
    void disableServos();
    void startContinuousTest();
    void stopContinuousTest();
    void update();
private:
    Adafruit_PWMServoDriver pwm;
    uint8_t servoChannels[NUM_SERVOS];
    uint16_t angleToPulse(uint16_t angle);
    bool _continuousTest;
    unsigned long _lastMoveTime;
    int _currentAngle;
    int _direction;
};

#endif // SERVO_CONFIG_H
