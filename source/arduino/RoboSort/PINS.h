#ifndef PINS_H
#define PINS_H

// ===== SERVO DRIVER (PCA9685) PINS =====
// I2C Communication Pins (Hardware I2C)
// SDA - A4 (Arduino Uno/Nano) or Pin 20 (Arduino Mega)
// SCL - A5 (Arduino Uno/Nano) or Pin 21 (Arduino Mega)
// Default I2C Address: 0x40
#define SERVO_OE_PIN 2   // Output Enable pin connected to GPIO2

// ===== DC MOTOR DRIVER (E-Gizmo HPMD-3.1) PINS =====
// Motor A Pins
#define MOTOR_A_PWM 11   // PWM pin for Motor A speed control
#define MOTOR_A_DIR 10   // Direction pin for Motor A
#define MOTOR_A_WD 9     // Watchdog pin for Motor A

// Motor B Pins
#define MOTOR_B_PWM 8    // PWM pin for Motor B speed control
#define MOTOR_B_DIR 7    // Direction pin for Motor B
#define MOTOR_B_WD 6     // Watchdog pin for Motor B

// ===== ULTRASONIC SENSOR (HC-SR04) PINS =====
#define TRIG_PIN 4       // Trigger pin for ultrasonic sensor
#define ECHO_PIN 5       // Echo pin for ultrasonic sensor

// ===== STEPPER MOTOR DRIVER (TB6600) PINS =====
#define STEPPER_STEP_PIN 3  // PUL+ connected to pin 3
#define STEPPER_DIR_PIN 4   // DIR+ connected to pin 4
#define STEPPER_ENA_PIN 5   // ENA+ connected to pin 5

#endif // PINS_H
