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
    void moveForward(uint8_t speed);
    void moveBackward(uint8_t speed);
    void rotateRight(uint8_t speed);
    void rotateLeft(uint8_t speed);
    void turnLeft(uint8_t speed);  // Turn left: only left motor forward (right motor stopped)
    void turnRight(uint8_t speed); // Turn right: only right motor forward (left motor stopped)
    void turnAbout(uint8_t direction, uint8_t speed); // Spot turn (0=left, 1=right)
    void update();  // Call this in main loop for software PWM
private:
    void setMotorPins(uint8_t motor, uint8_t in1State, uint8_t in2State);
    void setSoftwarePWMSpeed(uint8_t motor, uint8_t speed);
    uint8_t currentSpeedA;
    uint8_t currentSpeedB;
    uint8_t currentDirectionA;
    uint8_t currentDirectionB;
    unsigned long lastPWMUpdate;
};

#endif // DC_CONFIG_H
