#include "SERVO_CONFIG.h"
#include <Wire.h>

// Typical min/max pulse lengths for servos
#define SERVO_MIN_PULSE  150
#define SERVO_MAX_PULSE  600

// Microsecond ranges matching Adafruit sample
#define SERVO_MIN_US     600
#define SERVO_MAX_US     2400
#define SERVO_NEUTRAL_US 1500

// Continuous servo timing (approximate for 360° servos)
// One full rotation takes about 1-2 seconds depending on servo
#define ROTATION_TIME_MS 1500  // Time for one full rotation

ServoConfig::ServoConfig() : pwm(Adafruit_PWMServoDriver()) {
  lifterRunning = false;
  lifterDirection = false;
  lifterStartTime = 0;
  lifterTimeout = 3000;  // Default 3 seconds
}

void ServoConfig::begin() {
  // Configure servo OE pin
  pinMode(SERVO_OE_PIN, OUTPUT);
  
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);  // Analog servos run at ~50 Hz
  delay(10);
  
  // Stop lifter servo initially
  lifterStop();
  
  // Enable servos by default
  enableServos();
  
  Serial.println("Servo config initialized");
}

void ServoConfig::update() {
  if (!lifterRunning) return;  // Nothing to do if lifter is OFF
  
  // LIFTER DOWN: Auto-stop after variable timeout (3s or 75s)
  if (!lifterDirection) {
    if (millis() - lifterStartTime >= lifterTimeout) {
      lifterStop();
      Serial.println("LIFTER DOWN complete (" + String(lifterTimeout/1000) + "s)");
    }
  }
  
  // LIFTER UP: Auto-stop when ARM limit switch pressed
  else {
    if (digitalRead(ARM_LIMIT_PIN) == LOW) {
      lifterStop();
      Serial.println("LIFTER UP stopped by ARM limit switch");
    }
  }
}

void ServoConfig::lifterUp() {
  // Turn ON - rotate UP until arm limit switch pressed
  pwm.setPWM(LIFTER_SERVO_CHANNEL, 0, LIFTER_UP_SPEED);
  lifterRunning = true;
  lifterDirection = true;  // UP
  Serial.println("LIFTER UP - ON (auto-stop on ARM switch)");
}

void ServoConfig::lifterDown() {
  // Check if ARM limit switch is currently pressed
  bool armSwitchPressed = (digitalRead(ARM_LIMIT_PIN) == LOW);
  
  // Set timeout based on ARM switch state
  if (armSwitchPressed) {
    lifterTimeout = 75000;  // 75 seconds when ARM switch is pressed
    Serial.println("LIFTER DOWN - ON (ARM switch pressed, 75s timeout)");
  } else {
    lifterTimeout = 3000;   // 3 seconds normal timeout
    Serial.println("LIFTER DOWN - ON (auto-stop after 3s)");
  }
  
  // Turn ON - rotate DOWN
  pwm.setPWM(LIFTER_SERVO_CHANNEL, 0, LIFTER_DOWN_SPEED);
  lifterRunning = true;
  lifterDirection = false;  // DOWN
  lifterStartTime = millis();
}

void ServoConfig::lifterStop() {
  // Turn OFF - send stop signal multiple times to ensure complete stop
  pwm.setPWM(LIFTER_SERVO_CHANNEL, 0, LIFTER_STOP);
  delay(10);
  pwm.setPWM(LIFTER_SERVO_CHANNEL, 0, LIFTER_STOP);
  delay(10);
  pwm.setPWM(LIFTER_SERVO_CHANNEL, 0, LIFTER_STOP);
  
  lifterRunning = false;
  Serial.println("LIFTER - OFF");
}

void ServoConfig::testServos() {
  Serial.println("Testing all servos...");
  
  // Test each servo by moving it to 0°, 90°, and 180°
  for (int servoNum = 0; servoNum < NUM_SERVOS; servoNum++) {
    if (servoNum == LIFTER_SERVO_CHANNEL) {
      // Skip lifter servo (360° continuous) for angle testing
      continue;
    }
    
    Serial.print("Testing servo ");
    Serial.println(servoNum);
    
    // Move to 0°
    setServoAngle(servoNum, 0);
    delay(500);
    
    // Move to 90°
    setServoAngle(servoNum, 90);
    delay(500);
    
    // Move to 180°
    setServoAngle(servoNum, 180);
    delay(500);
    
    // Return to 90°
    setServoAngle(servoNum, 90);
    delay(500);
  }
  
  Serial.println("Servo test complete.");
}

void ServoConfig::setServoAngle(int servoNum, int angle) {
  // Convert angle (0-180) to pulse length (150-600)
  int pulseLength = map(angle, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  
  // Set the PWM signal
  pwm.setPWM(servoNum, 0, pulseLength);
}

void ServoConfig::enableServos() {
  digitalWrite(SERVO_OE_PIN, LOW);
}

void ServoConfig::disableServos() {
  digitalWrite(SERVO_OE_PIN, HIGH);
}

void ServoConfig::startContinuousTest() {
  // Implement if needed
}

void ServoConfig::stopContinuousTest() {
  // Implement if needed
}

