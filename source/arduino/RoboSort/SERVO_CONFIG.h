#ifndef SERVO_CONFIG_H
#define SERVO_CONFIG_H

#include <Adafruit_PWMServoDriver.h>
#include "PINS.h"

#define NUM_SERVOS 6
#define LIFTER_SERVO_CHANNEL 0  // 360° continuous servo on channel 0

// Continuous servo control values for PCA9685 (pulse length: 0-4096)
#define LIFTER_STOP 310        // Neutral position (stopped)
#define LIFTER_UP_SPEED 480    // Rotate up direction (maximum speed)
#define LIFTER_DOWN_SPEED 140  // Rotate down direction (maximum speed)

class ServoConfig {
private:
  Adafruit_PWMServoDriver pwm;
  
  // Lifter control variables (simple ON/OFF for continuous servo)
  bool lifterRunning;
  bool lifterDirection;  // true = UP, false = DOWN
  unsigned long lifterStartTime;  // For DOWN timer only
  unsigned long lifterTimeout;    // Variable timeout for DOWN (3s or 75s)
  int lifterRotationCount;        // Count rotations during DOWN
  
  // Arm servo position tracking
  int currentArmAngle;  // Track current position of channel 1 arm servo
  
  // Gripper servo position tracking
  int currentGripperAngle;  // Track current position of channel 2 gripper servo
  
  // Gripper rotation servo position tracking
  int currentGripperRotationAngle;  // Track current position of channel 3 gripper rotation servo
  
  // Arm extension servo position tracking
  int currentArmExtensionAngle;  // Track current position of channel 4 arm extension servo
  
  // Look servo position tracking
  int currentLookAngle;  // Track current position of channel 5 look servo
  
  // Mutual exclusion flags for smooth operation
  bool armExtendOperating;  // Flag to track if ARM-EXTEND servo is operating
  bool lookOperating;       // Flag to track if LOOK servo is operating
  
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
  void gripperRotate(int angle);  // Control gripper servo on channel 2
  void gripperRotationRotate(int angle);  // Control gripper rotation servo on channel 3
  void armExtend(int angle);  // Control arm extension servo on channel 4
  void lookRotate(int angle);  // Control look servo on channel 5
  
  // Mutual exclusion status functions
  bool isArmExtendOperating() { return armExtendOperating; }
  bool isLookOperating() { return lookOperating; }
  
  // Position getter functions
  int getArmExtensionAngle() { return currentArmExtensionAngle; }
  int getLookAngle() { return currentLookAngle; }
  
  // Existing functions (add these if you have them)
  void enableServos();
  void disableServos();
  void startContinuousTest();
  void stopContinuousTest();
};

#endif // SERVO_CONFIG_H
