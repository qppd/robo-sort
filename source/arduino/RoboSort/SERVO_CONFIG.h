#ifndef SERVO_CONFIG_H
#define SERVO_CONFIG_H

#include <Adafruit_PWMServoDriver.h>

#define NUM_SERVOS 5

class ServoConfig {
public:
    ServoConfig();
    void begin();
    void setServoAngle(uint8_t servo, uint16_t angle);
    void testServos();
private:
    Adafruit_PWMServoDriver pwm;
    uint8_t servoChannels[NUM_SERVOS];
    uint16_t angleToPulse(uint16_t angle);
};

#endif // SERVO_CONFIG_H
