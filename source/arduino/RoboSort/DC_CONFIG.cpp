#include "DC_CONFIG.h"

DCConfig::DCConfig() {
    // Constructor
    currentSpeedA = 0;
    currentSpeedB = 0;
    currentDirectionA = STOP;
    currentDirectionB = STOP;
    lastPWMUpdate = 0;
}

void DCConfig::begin() {
    // Initialize L298N Module 1 (Motor A - RIGHT wheel) pins
    pinMode(MOTOR_A_IN1, OUTPUT);
    pinMode(MOTOR_A_IN2, OUTPUT);
    
    // Initialize L298N Module 2 (Motor B - LEFT wheel) pins
    pinMode(MOTOR_B_IN1, OUTPUT);
    pinMode(MOTOR_B_IN2, OUTPUT);
    
    // Stop all motors initially
    stopAll();
}

void DCConfig::setMotorPins(uint8_t motor, uint8_t in1State, uint8_t in2State) {
    if (motor == MOTOR_A) {
        digitalWrite(MOTOR_A_IN1, in1State);
        digitalWrite(MOTOR_A_IN2, in2State);
    } else if (motor == MOTOR_B) {
        digitalWrite(MOTOR_B_IN1, in1State);
        digitalWrite(MOTOR_B_IN2, in2State);
    }
}

void DCConfig::setSoftwarePWMSpeed(uint8_t motor, uint8_t speed) {
    speed = constrain(speed, 0, 255);
    
    if (motor == MOTOR_A) {
        currentSpeedA = speed;
    } else if (motor == MOTOR_B) {
        currentSpeedB = speed;
    }
}

void DCConfig::setMotorSpeed(uint8_t motor, int speed) {
    // Constrain speed to valid range (-255 to 255)
    speed = constrain(speed, -255, 255);
    
    if (speed > 0) {
        // Forward: IN1=HIGH, IN2=LOW
        setMotorPins(motor, HIGH, LOW);
    } else if (speed < 0) {
        // Backward: IN1=LOW, IN2=HIGH
        setMotorPins(motor, LOW, HIGH);
    } else {
        // Stop: IN1=LOW, IN2=LOW
        stopMotor(motor);
    }
}

void DCConfig::setMotorDirection(uint8_t motor, uint8_t direction) {
    if (direction == FORWARD) {
        // Forward: IN1=HIGH, IN2=LOW
        setMotorPins(motor, HIGH, LOW);
    } else if (direction == BACKWARD) {
        // Backward: IN1=LOW, IN2=HIGH
        setMotorPins(motor, LOW, HIGH);
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
        setMotorPins(motor, HIGH, LOW);
        setSoftwarePWMSpeed(motor, speed);
        if (motor == MOTOR_A) currentDirectionA = FORWARD;
        else currentDirectionB = FORWARD;
    } else if (direction == BACKWARD) {
        // Backward: IN1=LOW, IN2=HIGH
        setMotorPins(motor, LOW, HIGH);
        setSoftwarePWMSpeed(motor, speed);
        if (motor == MOTOR_A) currentDirectionA = BACKWARD;
        else currentDirectionB = BACKWARD;
    } else if (direction == BRAKE) {
        brakeMotor(motor);
    } else {
        // Stop: IN1=LOW, IN2=LOW
        stopMotor(motor);
    }
}

void DCConfig::stopMotor(uint8_t motor) {
    // Stop: IN1=LOW, IN2=LOW
    setMotorPins(motor, LOW, LOW);
    setSoftwarePWMSpeed(motor, 0);
    if (motor == MOTOR_A) currentDirectionA = STOP;
    else currentDirectionB = STOP;
}

void DCConfig::brakeMotor(uint8_t motor) {
    // Brake: IN1=HIGH, IN2=HIGH (short brake without speed control)
    setMotorPins(motor, HIGH, HIGH);
    setSoftwarePWMSpeed(motor, 0);
    if (motor == MOTOR_A) currentDirectionA = BRAKE;
    else currentDirectionB = BRAKE;
}

void DCConfig::moveForward(uint8_t speed) {
    // Robot forward: motors rotate opposite directions since they face each other
    moveMotor(MOTOR_A, FORWARD, speed);
    moveMotor(MOTOR_B, BACKWARD, speed);
}

void DCConfig::moveBackward(uint8_t speed) {
    // Robot backward: motors rotate opposite directions since they face each other
    moveMotor(MOTOR_A, BACKWARD, speed);
    moveMotor(MOTOR_B, FORWARD, speed);
}

void DCConfig::rotateRight(uint8_t speed) {
    // Rotate right: both motors same direction for rotation
    moveMotor(MOTOR_A, FORWARD, speed);
    moveMotor(MOTOR_B, FORWARD, speed);
}

void DCConfig::rotateLeft(uint8_t speed) {
    // Rotate left: both motors same direction for rotation
    moveMotor(MOTOR_A, BACKWARD, speed);
    moveMotor(MOTOR_B, BACKWARD, speed);
}

void DCConfig::stopAll() {
    stopMotor(MOTOR_A);
    stopMotor(MOTOR_B);
}

void DCConfig::turnLeft(uint8_t speed) {
    // Turn left: only left wheel forward, right wheel stopped
    // MOTOR_A = right wheel, MOTOR_B = left wheel
    // Use BACKWARD for MOTOR_B since motors face each other
    stopMotor(MOTOR_A);           // Right wheel stopped
    moveMotor(MOTOR_B, BACKWARD, speed);  // Left wheel "forward" (backward direction)
}

void DCConfig::turnRight(uint8_t speed) {
    // Turn right: only right wheel forward, left wheel stopped
    // MOTOR_A = right wheel, MOTOR_B = left wheel
    // MOTOR_A FORWARD makes robot turn right in forward direction
    moveMotor(MOTOR_A, FORWARD, speed);  // Right wheel forward
    stopMotor(MOTOR_B);           // Left wheel stopped
}

void DCConfig::turnAbout(uint8_t direction, uint8_t speed) {
    // Spot turn: both motors same direction for rotation (since motors face each other)
    // MOTOR_A = right wheel, MOTOR_B = left wheel
    if (direction == 0) {
        // TURN_LEFT: both motors backward for left rotation
        moveMotor(MOTOR_A, BACKWARD, speed);  // Right wheel backward
        moveMotor(MOTOR_B, BACKWARD, speed);  // Left wheel backward
    } else {
        // TURN_RIGHT: both motors forward for right rotation
        moveMotor(MOTOR_A, FORWARD, speed);   // Right wheel forward
        moveMotor(MOTOR_B, FORWARD, speed);   // Left wheel forward
    }
}

void DCConfig::update() {
    // Continuous operation - motors stay ON once direction is set
    // No software PWM needed for L298N driver with enable pins always HIGH
    // Motor direction pins are already set by moveMotor(), no need to pulse them
    // This function can be used for future enhancements if needed
}
