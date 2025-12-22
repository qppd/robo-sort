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
#define MOTOR_A_IN2 8    // IN2 pin for Motor A

// L298N Module 2 - Motor B Pins
#define MOTOR_B_IN1 10   // IN1 pin for Motor B
#define MOTOR_B_IN2 11   // IN2 pin for Motor B


// ===== ULTRASONIC SENSORS (HC-SR04) PINS =====
// Ultrasonic Sensor 1
#define ULTRASONIC1_TRIG_PIN 25  // Trigger pin for ultrasonic sensor 1
#define ULTRASONIC1_ECHO_PIN 23  // Echo pin for ultrasonic sensor 1

// Ultrasonic Sensor 2
#define ULTRASONIC2_TRIG_PIN 27  // Trigger pin for ultrasonic sensor 2
#define ULTRASONIC2_ECHO_PIN 29  // Echo pin for ultrasonic sensor 2

// Ultrasonic Sensor 3
#define ULTRASONIC3_TRIG_PIN 31  // Trigger pin for ultrasonic sensor 3
#define ULTRASONIC3_ECHO_PIN 33  // Echo pin for ultrasonic sensor 3

// Ultrasonic Sensor 4
#define ULTRASONIC4_TRIG_PIN 35  // Trigger pin for ultrasonic sensor 4
#define ULTRASONIC4_ECHO_PIN 37  // Echo pin for ultrasonic sensor 4

// ===== STEPPER MOTOR DRIVER (TB6600) PINS =====
#define STEPPER_STEP_PIN 3  // PUL+ connected to pin 3
#define STEPPER_DIR_PIN 4   // DIR+ connected to pin 4
#define STEPPER_ENA_PIN 5   // ENA+ connected to pin 5

// ===== BUZZER PIN =====
#define BUZZER_PIN 6        // Buzzer connected to pin 6

#endif // PINS_H
