#ifndef DC_CONFIG_H
#define DC_CONFIG_H

#include <Arduino.h>
#include "PINS.h"

// Motor constants
#define MOTOR_A 0
#define MOTOR_B 1
#define FORWARD 1
#define BACKWARD 2
#define BRAKE 3
#define STOP 0

class DCConfig {
public:
    DCConfig();
    void begin();
    void setMotorSpeed(uint8_t motor, int speed);
    void setMotorDirection(uint8_t motor, uint8_t direction);
    void moveMotor(uint8_t motor, uint8_t direction, uint8_t speed);
    void stopMotor(uint8_t motor);
    void brakeMotor(uint8_t motor);
    void stopAll();
    void testMotors();
    void updateWatchdog();
    void startContinuousTest();
    void stopContinuousTest();
    void update();
private:
    void setMotorPins(uint8_t motor, uint8_t dirState, uint8_t pwmValue);
    bool _continuousTest;
    unsigned long _lastChangeTime;
    uint8_t _direction;
};

#endif // DC_CONFIG_H
