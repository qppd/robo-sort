#ifndef PINS_H
#define PINS_H

// ===== SERVO DRIVER (PCA9685) PINS =====
// I2C Communication Pins (Hardware I2C)
// SDA - A4 (Arduino Uno/Nano) or Pin 20 (Arduino Mega)
// SCL - A5 (Arduino Uno/Nano) or Pin 21 (Arduino Mega)
// Default I2C Address: 0x40
#define SERVO_OE_PIN 2   // Output Enable pin connected to GPIO2

// ===== DC MOTOR DRIVER (L298N) PINS =====
// L298N Module 1 - Motor A Pins
#define MOTOR_A_IN1 7    // IN1 pin for Motor A
#define MOTOR_A_IN2 9    // IN2 pin for Motor A
#define MOTOR_A_ENA 11   // ENA (PWM) pin for Motor A speed control

// L298N Module 2 - Motor B Pins
#define MOTOR_B_IN1 10   // IN1 pin for Motor B
#define MOTOR_B_IN2 12   // IN2 pin for Motor B
#define MOTOR_B_ENB 8    // ENB (PWM) pin for Motor B speed control

// ===== ULTRASONIC SENSOR (HC-SR04) PINS =====
#define TRIG_PIN 4       // Trigger pin for ultrasonic sensor
#define ECHO_PIN 5       // Echo pin for ultrasonic sensor

// ===== STEPPER MOTOR DRIVER (TB6600) PINS =====
#define STEPPER_STEP_PIN 3  // PUL+ connected to pin 3
#define STEPPER_DIR_PIN 22  // DIR+ connected to pin 22
#define STEPPER_ENA_PIN 23  // ENA+ connected to pin 23

// ===== BUZZER PIN =====
#define BUZZER_PIN 6        // Buzzer connected to pin 6

#endif // PINS_H
