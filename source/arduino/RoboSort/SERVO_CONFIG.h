#ifndef SERVO_CONFIG_H
#define SERVO_CONFIG_H

#include <Adafruit_PWMServoDriver.h>
#include "PINS.h"

#define NUM_SERVOS 6
#define LIFTER_SERVO_CHANNEL 0  // 360° continuous servo on channel 0

// Continuous servo control values for PCA9685 (pulse length: 0-4096)
#define LIFTER_STOP 310        // Neutral position (stopped)
#define LIFTER_UP_SPEED 360    // Rotate up direction
#define LIFTER_DOWN_SPEED 260  // Rotate down direction

class ServoConfig {
private:
  Adafruit_PWMServoDriver pwm;
  
  // Lifter control variables (simple ON/OFF for continuous servo)
  bool lifterRunning;
  bool lifterDirection;  // true = UP, false = DOWN
  unsigned long lifterStartTime;  // For DOWN timer only
  unsigned long lifterTimeout;    // Variable timeout for DOWN (3s or 75s)
  
public:
  ServoConfig();
  void begin();
  void update();
  
  // Lifter control functions
  void lifterUp();
  void lifterDown();
  void lifterStop();
  
  // Servo control functions
  void testServos();
  void setServoAngle(int servoNum, int angle);
  void armRotate(int angle);  // Control MG996R on channel 1
  
  // Existing functions (add these if you have them)
  void enableServos();
  void disableServos();
  void startContinuousTest();
  void stopContinuousTest();
};

#endif // SERVO_CONFIG_H
