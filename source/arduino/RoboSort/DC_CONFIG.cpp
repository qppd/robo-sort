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
    // Initialize L298N Module 1 (Motor A) pins
    pinMode(MOTOR_A_IN1, OUTPUT);
    pinMode(MOTOR_A_IN2, OUTPUT);
    
    // Initialize L298N Module 2 (Motor B) pins
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
    // Differential turn left: right wheel faster, left wheel slower
    // Right wheel (Motor B) at full speed, left wheel (Motor A) at half speed
    uint8_t slowSpeed = speed / 2;
    moveMotor(MOTOR_A, FORWARD, slowSpeed);  // Left wheel slower
    moveMotor(MOTOR_B, BACKWARD, speed);     // Right wheel faster
}

void DCConfig::turnRight(uint8_t speed) {
    // Differential turn right: left wheel faster, right wheel slower
    // Left wheel (Motor A) at full speed, right wheel (Motor B) at half speed
    uint8_t slowSpeed = speed / 2;
    moveMotor(MOTOR_A, FORWARD, speed);      // Left wheel faster
    moveMotor(MOTOR_B, BACKWARD, slowSpeed); // Right wheel slower
}

void DCConfig::turnAbout(uint8_t direction, uint8_t speed) {
    // Spot turn: one wheel forward, one wheel backward (opposite directions)
    if (direction == 0) {
        // Turn left: left wheel backward, right wheel forward
        moveMotor(MOTOR_A, FORWARD, speed);   // Left wheel backward
        moveMotor(MOTOR_B, BACKWARD, speed);  // Right wheel forward
    } else {
        // Turn right: left wheel forward, right wheel backward
        moveMotor(MOTOR_A, BACKWARD, speed);  // Left wheel forward
        moveMotor(MOTOR_B, FORWARD, speed);   // Right wheel backward
    }
}

void DCConfig::update() {
    // Continuous operation - motors stay ON once direction is set
    // No software PWM needed for L298N driver with enable pins always HIGH
    // Motor direction pins are already set by moveMotor(), no need to pulse them
    // This function can be used for future enhancements if needed
}