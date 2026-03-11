#ifndef PINS_H
#define PINS_H

// ===== SERVO DRIVER (PCA9685) PINS =====
// I2C Communication Pins (Hardware I2C)
// SDA - A4 (Arduino Uno/Nano) or Pin 20 (Arduino Mega)
// SCL - A5 (Arduino Uno/Nano) or Pin 21 (Arduino Mega)
// Default I2C Address: 0x40
#define SERVO_OE_PIN 2   // Output Enable pin connected to GPIO2

// ===== DC MOTOR DRIVER (L298N) PINS =====
// L298N Module 1 - Motor A Pins (RIGHT wheel)
#define MOTOR_A_IN1 7    // IN1 pin for Motor A
#define MOTOR_A_IN2 8    // IN2 pin for Motor A

// L298N Module 2 - Motor B Pins (LEFT wheel)
#define MOTOR_B_IN1 10   // IN1 pin for Motor B
#define MOTOR_B_IN2 11   // IN2 pin for Motor B


// ===== ULTRASONIC SENSORS (HC-SR04) PINS =====
// Ultrasonic Sensor 1
#define ULTRASONIC1_TRIG_PIN 9   // Trigger pin for ultrasonic sensor 1
#define ULTRASONIC1_ECHO_PIN 6   // Echo pin for ultrasonic sensor 1

// ===== STEPPER MOTOR DRIVER (TB6600) PINS =====
#define STEPPER_STEP_PIN 3  // PUL+ connected to pin 3
#define STEPPER_DIR_PIN 4   // DIR+ connected to pin 4
#define STEPPER_ENA_PIN 5   // ENA+ connected to pin 5

// ===== BUZZER PIN =====
#define BUZZER_PIN 33        // Buzzer connected to pin 6

// ===== LIMIT SWITCH PINS =====
#define ARM_LIMIT_PIN 42     // Arm limit switch connected to pin 42
#define BIN_LIMIT_PIN 43     // Bin limit switch connected to pin 43

// ===== FRONT ULTRASONIC SENSORS (Obstacle Avoidance) =====
#define FRONT_LEFT_TRIG  52  // Trigger pin for front-left obstacle sensor
#define FRONT_LEFT_ECHO  53  // Echo pin for front-left obstacle sensor
#define FRONT_RIGHT_TRIG 50  // Trigger pin for front-right obstacle sensor
#define FRONT_RIGHT_ECHO 51  // Echo pin for front-right obstacle sensor
#endif // PINS_H
