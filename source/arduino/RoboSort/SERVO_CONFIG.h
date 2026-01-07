#ifndef SERVO_CONFIG_H
#define SERVO_CONFIG_H

#include <Adafruit_PWMServoDriver.h>
#include "PINS.h"

#define NUM_SERVOS 6
#define LIFTER_SERVO_CHANNEL 0  // 360° continuous servo on channel 0

// Continuous servo control values for PCA9685
#define LIFTER_STOP 1500      // Neutral position (stopped)
#define LIFTER_UP_SPEED 1300  // Rotate one direction
#define LIFTER_DOWN_SPEED 1700 // Rotate opposite direction

class ServoConfig {
private:
  Adafruit_PWMServoDriver pwm;
  
  // Lifter control variables
  bool lifterMoving;
  unsigned long lifterStartTime;
  bool lifterIsUp;  // Track if lifter is moving up
  
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
  
  // Existing functions (add these if you have them)
  void enableServos();
  void disableServos();
  void startContinuousTest();
  void stopContinuousTest();
};

#endif // SERVO_CONFIG_H
