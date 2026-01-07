#ifndef SERVO_CONFIG_H
#define SERVO_CONFIG_H

#include <Adafruit_PWMServoDriver.h>
#include "PINS.h"

#define NUM_SERVOS 5

enum ServoType { POSITION_SERVO, CONTINUOUS_SERVO };

class ServoConfig {
public:
    ServoConfig();
    void begin();
    void setServoAngle(uint8_t servo, uint16_t angle);
    void setContinuousSpeed(uint8_t servo, int8_t speed); // -100 to +100
    void testServos();
    void enableServos();
    void disableServos();
    void startContinuousTest();
    void stopContinuousTest();
    void update();
    void lifterUp();
    void lifterDown();
private:
    Adafruit_PWMServoDriver pwm;
    ServoType servoTypes[NUM_SERVOS];
    uint8_t servoChannels[NUM_SERVOS];
    uint16_t angleToPulse(uint16_t angle);
    uint16_t speedToPulse(int8_t speed); // Convert -100/+100 to pulse width
    bool _continuousTest;
    unsigned long _lastMoveTime;
    int _currentAngle;
    int _direction;
    // Lifter control variables
    bool _lifterMoving;
    unsigned long _lifterStartTime;
    int _lifterRotations;
    int8_t _lifterSpeed;
};

#endif // SERVO_CONFIG_H
