#include "SERVO_CONFIG.h"
#include <Wire.h>

// Typical min/max pulse lengths for servos
#define SERVO_MIN_PULSE  102
#define SERVO_MAX_PULSE  512

// Microsecond ranges matching Adafruit sample
#define SERVO_MIN_US     500
#define SERVO_MAX_US     2500
#define SERVO_NEUTRAL_US 1500

// Continuous servo timing (approximate for 360° servos)
// One full rotation takes about 1-2 seconds depending on servo
#define ROTATION_TIME_MS 1500  // Time for one full rotation

ServoConfig::ServoConfig() : pwm(Adafruit_PWMServoDriver()) {
  lifterRunning = false;
  lifterDirection = false;
  lifterStartTime = 0;
  lifterTimeout = 3000;  // Default 3 seconds
  lifterRotationCount = 0;  // Initialize rotation count
  lifterMaxRotations = 0;   // Default to time-based
  currentArmAngle = 180;  // Initialize arm to 180 degrees
  currentGripperAngle = 120;  // Initialize gripper to 120 degrees (safe minimum, default closed)
  currentGripperRotationAngle = 90;  // Initialize gripper rotation to 90 degrees (default position)
  currentArmExtensionAngle = 180;  // Initialize arm extension to 180 degrees (HOME position)
  currentLookAngle = 180;  // Initialize look servo to 180 degrees (HOME position)
  
  // Initialize mutual exclusion flags
  armExtendOperating = false;
  lookOperating = false;
  
  // Initialize homing state
  lifterHoming = false;
  homingPhase = true;  // Start with UP phase
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
  
  // Set arm servo to default position (180 degrees)
  setServoAngle(0, 180);
  
  // Set gripper servo to default position (120 degrees, safe minimum / closed)
  // Must match currentGripperAngle initialised in the constructor (120)
  // to avoid a position-tracking desync that causes a jerk on the first command.
  setServoAngle(4, 120);
  currentGripperAngle = 120;
  
  // Set gripper rotation servo to default position (90 degrees)
  setServoAngle(3, 0);
  
  // Set arm extension servo to HOME position (90 degrees)
  setServoAngle(1, 90);
  
  // Set look servo to HOME position (180 degrees)
  setServoAngle(2, 160);
  currentLookAngle = 160;
  
  // Enable servos by default
  enableServos();
  
  Serial.println("Servo config initialized");
  
  // Perform lifter homing sequence
  // Serial.println("Starting lifter homing sequence...");
  // lifterHome();
}

void ServoConfig::update() {
  if (!lifterRunning) return;  // Nothing to do if lifter is OFF
  
  // LIFTER DOWN: Auto-stop after variable timeout or max rotations
  if (!lifterDirection) {
    unsigned long elapsed = millis() - lifterStartTime;
    int rotations = elapsed / ROTATION_TIME_MS;
    if (rotations > lifterRotationCount) {
      lifterRotationCount = rotations;
      Serial.println("LIFTER DOWN rotation count: " + String(lifterRotationCount));
    }
    bool shouldStop = false;
    if (lifterMaxRotations > 0 && lifterRotationCount >= lifterMaxRotations) {
      shouldStop = true;
      Serial.println("LIFTER DOWN complete (" + String(lifterMaxRotations) + " rotations)");
    } else if (elapsed >= lifterTimeout) {
      shouldStop = true;
      Serial.println("LIFTER DOWN complete (" + String(lifterTimeout/1000) + "s)");
    }
    if (shouldStop) {
      lifterStop();
    }
  }
  
  // LIFTER UP: Auto-stop after lifterMaxRotations rotations
  else {
    unsigned long elapsed = millis() - lifterStartTime;
    int rotations = elapsed / ROTATION_TIME_MS;
    if (rotations > lifterRotationCount) {
      lifterRotationCount = rotations;
      Serial.println("LIFTER UP rotation count: " + String(lifterRotationCount));
    }
    if (lifterRotationCount >= lifterMaxRotations) {
      lifterStop();
      Serial.println("LIFTER UP complete (" + String(lifterMaxRotations) + " rotations)");
    }
  }
}

void ServoConfig::lifterUp() {
  // Turn ON - rotate UP for exactly DEFAULT_LIFTER_MAX_ROTATIONS rotations
  lifterMaxRotations = DEFAULT_LIFTER_MAX_ROTATIONS;
  lifterTimeout = 1000000;  // Large timeout, rely on rotation count
  lifterRotationCount = 0;
  lifterStartTime = millis();
  
  pwm.setPWM(LIFTER_SERVO_CHANNEL, 0, LIFTER_UP_SPEED);
  lifterRunning = true;
  lifterDirection = true;  // UP
  Serial.println("LIFTER UP - ON (" + String(lifterMaxRotations) + " rotations)");
}

void ServoConfig::lifterDown() {
  // Always rotate DOWN exactly DEFAULT_LIFTER_MAX_ROTATIONS rotations
  lifterMaxRotations = DEFAULT_LIFTER_MAX_ROTATIONS;
  lifterTimeout = 1000000;  // Large timeout, rely on rotation count
  lifterRotationCount = 0;
  
  // Turn ON - rotate DOWN
  pwm.setPWM(LIFTER_SERVO_CHANNEL, 0, LIFTER_DOWN_SPEED);
  lifterRunning = true;
  lifterDirection = false;  // DOWN
  lifterStartTime = millis();
  Serial.println("LIFTER DOWN - ON (" + String(lifterMaxRotations) + " rotations)");
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

void ServoConfig::lifterHome() {
  // Home: Reset lifter to UP position (8 rotations from current position)
  Serial.println("LIFTER HOME: Resetting lifter UP " + String(DEFAULT_LIFTER_MAX_ROTATIONS) + " rotations");
  
  // Simple UP-only reset (no homing flag, just do UP)
  lifterMaxRotations = DEFAULT_LIFTER_MAX_ROTATIONS;
  lifterTimeout = 1000000;
  lifterRotationCount = 0;
  lifterStartTime = millis();
  
  pwm.setPWM(LIFTER_SERVO_CHANNEL, 0, LIFTER_UP_SPEED);
  lifterRunning = true;
  lifterDirection = true;  // UP only
  
  // No homing flags - just execute UP as a normal operation
}

void ServoConfig::testServos() {
  Serial.println("Testing all servos...");
  
  // Test each servo by moving it to 0°, 90°, and 180°
  for (int servoNum = 0; servoNum < NUM_SERVOS; servoNum++) {
    if (servoNum == LIFTER_SERVO_CHANNEL) {
      // Skip lifter servo (360° continuous) for angle testing
      continue;
    }
    if (servoNum == 4) {
      // Skip gripper servo — its safe range (120-180°) is incompatible with the
      // 0/90/180 test pattern; use GRIP:<angle> to exercise it directly.
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
  // ARM-ROTATE (channel 0) minimum angle safety clamp
  if (servoNum == 0) {
    angle = constrain(angle, 26, 180);
  }
  // Convert angle (0-180) to pulse length (102-512 for 500-2500us)
  int pulseLength = map(angle, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  
  Serial.print("Ch");
  Serial.print(servoNum);
  Serial.print(" angle ");
  Serial.print(angle);
  Serial.print(" -> pulse ");
  Serial.println(pulseLength);
  
  // Set the PWM signal
  pwm.setPWM(servoNum, 0, pulseLength);
}

void ServoConfig::armRotate(int angle) {
  // Control MG996R servo on channel 0 (26-180 degrees)
  angle = constrain(angle, 26, 180);

  Serial.print("ARM rotating from ");
  Serial.print(currentArmAngle);
  Serial.print(" to ");
  Serial.print(angle);
  Serial.println(" degrees (smooth)");
  
  // Smooth movement with 1-degree steps
  int step = (angle > currentArmAngle) ? 1 : -1;
  
  while (currentArmAngle != angle) {
    currentArmAngle += step;
    setServoAngle(0, currentArmAngle);
    delay(15);  // 15ms per degree for smooth motion
  }
  
  Serial.print("ARM rotation complete at ");
  Serial.print(angle);
  Serial.println(" degrees");
}

void ServoConfig::gripperRotate(int angle) {
  // Control gripper servo on channel 4 (120-180 degrees only)
  // Hard-clamp to the safe operating range — prevents any caller (direct command,
  // snapshot replay, or testServos) from driving the gripper below 120°.
  if (angle < 120) angle = 120;
  if (angle > 180) angle = 180;

  Serial.print("GRIPPER rotating from ");
  Serial.print(currentGripperAngle);
  Serial.print(" to ");
  Serial.print(angle);
  Serial.println(" degrees (smooth)");

  // Smooth movement with 1-degree steps.
  // Serial.println is intentionally kept OUT of this loop: at 9600 baud the
  // ~28-char debug message per step overflows the 64-byte TX buffer after
  // only a few steps, causing Serial.print to block for ~29 ms before each
  // pwm.setPWM call.  That turns the clean 15 ms/step cadence into an
  // irregular 15-44 ms/step, which manifests as jitter on a loaded gripper
  // servo.  The pulse-length formula is identical to setServoAngle().
  int step = (angle > currentGripperAngle) ? 1 : -1;

  while (currentGripperAngle != angle) {
    currentGripperAngle += step;
    int pulseLength = map(currentGripperAngle, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
    pwm.setPWM(4, 0, pulseLength);
    delay(15);  // 15 ms per degree – matches all other servo functions
  }

  Serial.print("GRIPPER rotation complete at ");
  Serial.print(angle);
  Serial.println(" degrees");
}

void ServoConfig::gripperRotationRotate(int angle) {
  // Control gripper rotation servo on channel 3 (grip-rotator, 0-180 degrees)
  if (angle < 0 || angle > 180) {
    Serial.println("Invalid angle. Range: 0-180");
    return;
  }
  
  Serial.print("GRIPPER-ROTATION rotating from ");
  Serial.print(currentGripperRotationAngle);
  Serial.print(" to ");
  Serial.print(angle);
  Serial.println(" degrees (smooth)");
  
  // Smooth movement with 1-degree steps
  int step = (angle > currentGripperRotationAngle) ? 1 : -1;
  
  while (currentGripperRotationAngle != angle) {
    currentGripperRotationAngle += step;
    setServoAngle(3, currentGripperRotationAngle);
    delay(15);  // 15ms per degree for smooth motion
  }
  
  Serial.print("GRIPPER-ROTATION rotation complete at ");
  Serial.print(angle);
  Serial.println(" degrees");
}

void ServoConfig::armExtend(int angle) {
  // Control arm extension servo on channel 1 (0-180 degrees)
  if (angle < 0 || angle > 180) {
    Serial.println("Invalid angle. Range: 0-180");
    return;
  }
  
  // Mutual exclusion: Loosen LOOK servo when ARM-EXTEND is operating
  if (lookOperating) {
    Serial.println("Loosening LOOK servo for smooth ARM-EXTEND operation");
    // LOOK servo is on channel 2, loosen it by turning off PWM
    pwm.setPWM(2, 0, 0);
    lookOperating = false;
  }
  
  // Set ARM-EXTEND as operating
  armExtendOperating = true;
  
  Serial.print("ARM-EXTENSION extending from ");
  Serial.print(currentArmExtensionAngle);
  Serial.print(" to ");
  Serial.print(angle);
  Serial.println(" degrees (smooth, LOOK loosened)");
  
  // Smooth movement with 1-degree steps
  int step = (angle > currentArmExtensionAngle) ? 1 : -1;
  
  while (currentArmExtensionAngle != angle) {
    currentArmExtensionAngle += step;
    setServoAngle(1, currentArmExtensionAngle);
    delay(15);  // 15ms per degree for smooth motion
  }
  
  Serial.print("ARM-EXTENSION extension complete at ");
  Serial.print(angle);
  Serial.println(" degrees");
  
  // Clear operating flag
  armExtendOperating = false;
}

void ServoConfig::lookRotate(int angle) {
  // Control look servo on channel 2 (0-180 degrees)
  if (angle < 0 || angle > 180) {
    Serial.println("Invalid angle. Range: 0-180");
    return;
  }
  
  // Mutual exclusion: Loosen ARM-EXTEND servo when LOOK is operating
  if (armExtendOperating) {
    Serial.println("Loosening ARM-EXTEND servo for smooth LOOK operation");
    // ARM-EXTEND servo is on channel 1, loosen it by turning off PWM
    pwm.setPWM(1, 0, 0);
    armExtendOperating = false;
  }
  
  // Set LOOK as operating
  lookOperating = true;
  
  Serial.print("LOOK rotating from ");
  Serial.print(currentLookAngle);
  Serial.print(" to ");
  Serial.print(angle);
  Serial.println(" degrees (smooth, ARM-EXTEND loosened)");
  
  // Smooth movement with 1-degree steps
  int step = (angle > currentLookAngle) ? 1 : -1;
  
  while (currentLookAngle != angle) {
    currentLookAngle += step;
    setServoAngle(2, currentLookAngle);
    delay(15);  // 15ms per degree for smooth motion
  }
  
  Serial.print("LOOK rotation complete at ");
  Serial.print(angle);
  Serial.println(" degrees");
  
  // Clear operating flag
  lookOperating = false;
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

