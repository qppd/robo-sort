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

#define LIFTER_DOWN_TIME 3000  // 3 seconds for LIFTER DOWN

ServoConfig::ServoConfig() : pwm(Adafruit_PWMServoDriver()) {
  lifterRunning = false;
  lifterDirection = false;
  lifterStartTime = 0;
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
  
  // LIFTER DOWN: Auto-stop after 3 seconds
  if (!lifterDirection) {
    if (millis() - lifterStartTime >= LIFTER_DOWN_TIME) {
      lifterStop();
      Serial.println("LIFTER DOWN complete (3 seconds)");
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
  pwm.writeMicroseconds(LIFTER_SERVO_CHANNEL, LIFTER_UP_SPEED);
  lifterRunning = true;
  lifterDirection = true;  // UP
  Serial.println("LIFTER UP - ON (auto-stop on ARM switch)");
}

void ServoConfig::lifterDown() {
  // Turn ON - rotate DOWN for 3 seconds then auto-stop
  pwm.writeMicroseconds(LIFTER_SERVO_CHANNEL, LIFTER_DOWN_SPEED);
  lifterRunning = true;
  lifterDirection = false;  // DOWN
  lifterStartTime = millis();
  Serial.println("LIFTER DOWN - ON (auto-stop after 3s)");
}

void ServoConfig::lifterStop() {
  // Turn OFF - send stop signal multiple times to ensure complete stop
  pwm.writeMicroseconds(LIFTER_SERVO_CHANNEL, LIFTER_STOP);
  delay(10);
  pwm.writeMicroseconds(LIFTER_SERVO_CHANNEL, LIFTER_STOP);
  delay(10);
  pwm.writeMicroseconds(LIFTER_SERVO_CHANNEL, LIFTER_STOP);
  
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

