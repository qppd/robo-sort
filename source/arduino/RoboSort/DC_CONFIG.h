#ifndef DC_CONFIG_H
#define DC_CONFIG_H

#include <Arduino.h>

// Motor driver pin definitions
// Adjust these pins according to your wiring
#define MOTOR_A_PWM 9    // PWM pin for Motor A speed
#define MOTOR_A_DIR1 7   // Direction pin 1 for Motor A
#define MOTOR_A_DIR2 8   // Direction pin 2 for Motor A

#define MOTOR_B_PWM 10   // PWM pin for Motor B speed
#define MOTOR_B_DIR1 11  // Direction pin 1 for Motor B
#define MOTOR_B_DIR2 12  // Direction pin 2 for Motor B

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
private:
    void setMotorPins(uint8_t motor, uint8_t dir1State, uint8_t dir2State, uint8_t pwmValue);
};

#endif // DC_CONFIG_H
