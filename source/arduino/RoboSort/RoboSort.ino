
#include <Wire.h>
#include "SERVO_CONFIG.h"
#include "DC_CONFIG.h"
#include "ULTRASONIC_CONFIG.h"
#include "TB6600.h"
#include "BUZZER_CONFIG.h"

ServoConfig servoConfig;
DCConfig dcConfig;
UltrasonicConfig ultrasonicConfig;
TB6600 stepper(STEPPER_DIR_PIN, STEPPER_STEP_PIN, STEPPER_ENA_PIN);
BuzzerConfig buzzerConfig;

// Limit switch variables
bool limitTestingActive = false;
bool lastArmState = HIGH;  // Track previous states (HIGH = OPEN, LOW = PRESSED)
bool lastBinState = HIGH;

// Stepper limit testing variables
bool stepperLimitTestingActive = false;
bool stepperNeedsRestart = false;

void setup() {
  Serial.begin(9600);
  
  // Initialize limit switches as INPUT_PULLUP
  pinMode(ARM_LIMIT_PIN, INPUT_PULLUP);
  pinMode(BIN_LIMIT_PIN, INPUT_PULLUP);
  
  servoConfig.begin();
  dcConfig.begin();
  ultrasonicConfig.begin();
  stepper.begin();
  buzzerConfig.begin();
  Serial.println("RoboSort Control System Ready!");
  buzzerConfig.startupBeep();
  Serial.println("Servo Commands: TEST, S<servo> <angle>, STEST, SSTOP, SENABLE, SDISABLE");
  Serial.println("Motor Commands: FORWARD <speed>, BACKWARD <speed>, RIGHT <speed>, LEFT <speed>, MSTOP");
  Serial.println("  Individual: M<motor> <direction> <speed>, MSTOP");
  Serial.println("  Motors: A or B, Directions: F (forward), B (backward), S (stop), BR (brake)");
  Serial.println("  Speed: 0-255 (default: 150)");
  Serial.println("Ultrasonic Commands: UTEST <sensor>, UDIST <sensor>, UAVG <sensor> <samples>, UDETECT <sensor> <threshold>, UCTEST <sensor>, UCSTOP <sensor>");
  Serial.println("  Sensors: 1-4 (default: 1)");
  Serial.println("Stepper Commands: STEPTEST, STEP <steps> <dir>, STEPSTOP, STEPCTEST, STEPCSTOP, HOME");
  Serial.println("  Dir: 0 (CW), 1 (CCW)");
  Serial.println("  HOME: Rotate CCW until BIN limit switch is triggered");
  Serial.println("Buzzer Commands: BTEST, BSUCCESS, BERROR, BWARNING");
  Serial.println("Limit Switch Commands: LTEST, LREAD, LCTEST, LCSTOP");
}

void loop() {
  stepper.update();
  servoConfig.update();
  ultrasonicConfig.update();
  buzzerConfig.update();
  
  // Handle continuous limit switch testing
  if (limitTestingActive) {
    bool armState = digitalRead(ARM_LIMIT_PIN);
    bool binState = digitalRead(BIN_LIMIT_PIN);
    
    // Only print when a switch is pressed (state changes from HIGH to LOW)
    if (armState == LOW && lastArmState == HIGH) {
      Serial.println("ARM LIMIT SWITCH PRESSED!");
    }
    if (binState == LOW && lastBinState == HIGH) {
      Serial.println("BIN LIMIT SWITCH PRESSED!");
    }
    
    // Update previous states
    lastArmState = armState;
    lastBinState = binState;
  }
  
  // Handle stepper limit switch testing
  if (stepperLimitTestingActive) {
    bool binState = digitalRead(BIN_LIMIT_PIN);
    if (binState == LOW) {  // Bin limit switch pressed
      stepper.emergencyStop();
      stepperLimitTestingActive = false;
      stepperNeedsRestart = false;
      Serial.println("Stepper stopped by BIN limit switch!");
      buzzerConfig.successBeep();
    } else if (!stepper.isBusy() && stepperNeedsRestart) {
      // Restart stepper in CCW direction
      stepper.setDirection(1); // CCW
      stepper.startSteps(10000, 750, 1500); // Large number of steps (25% faster)
    }
  }
  
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    // Servo commands
    if (input.equalsIgnoreCase("TEST")) {
      servoConfig.testServos();
      Serial.println("Servo test sequence complete.");
      buzzerConfig.successBeep();
    } else if (input.startsWith("S") && !input.startsWith("ST") && !input.equalsIgnoreCase("SENABLE") && !input.equalsIgnoreCase("SDISABLE")) {
      int spaceIdx = input.indexOf(' ');
      if (spaceIdx > 1) {
        int servoNum = input.substring(1, spaceIdx).toInt();
        int angle = input.substring(spaceIdx + 1).toInt();
        if (servoNum >= 0 && servoNum < NUM_SERVOS && angle >= 0 && angle <= 180) {
          servoConfig.setServoAngle(servoNum, angle);
          Serial.print("Set servo ");
          Serial.print(servoNum);
          Serial.print(" to angle ");
          Serial.println(angle);
          buzzerConfig.successBeep();
        } else {
          Serial.println("Invalid servo number or angle.");
        }
      } else {
        Serial.println("Invalid command format. Use: S<servo> <angle>");
      }
    } else if (input.equalsIgnoreCase("STEST")) {
      servoConfig.startContinuousTest();
      Serial.println("Continuous servo test started.");
    } else if (input.equalsIgnoreCase("SSTOP")) {
      servoConfig.stopContinuousTest();
      Serial.println("Continuous servo test stopped.");
    } else if (input.equalsIgnoreCase("SENABLE")) {
      servoConfig.enableServos();
      Serial.println("Servos enabled.");
    } else if (input.equalsIgnoreCase("SDISABLE")) {
      servoConfig.disableServos();
      Serial.println("Servos disabled.");
    }
    // Motor commands
    else if (input.equalsIgnoreCase("MSTOP")) {
      dcConfig.stopAll();
      Serial.println("All motors stopped.");
    } else if (input.startsWith("M") && !input.equalsIgnoreCase("MSTOP")) {
      // Parse motor command: M<motor> <direction> <speed>
      input = input.substring(1); // Remove 'M'
      int firstSpace = input.indexOf(' ');
      int secondSpace = input.indexOf(' ', firstSpace + 1);
      
      if (firstSpace > 0 && secondSpace > firstSpace) {
        char motorChar = input.charAt(0);
        String dirStr = input.substring(firstSpace + 1, secondSpace);
        int speed = input.substring(secondSpace + 1).toInt();
        
        uint8_t motor = (motorChar == 'A' || motorChar == 'a') ? MOTOR_A : MOTOR_B;
        uint8_t direction;
        
        if (dirStr.equalsIgnoreCase("F")) {
          direction = FORWARD;
        } else if (dirStr.equalsIgnoreCase("B")) {
          direction = BACKWARD;
        } else if (dirStr.equalsIgnoreCase("S")) {
          direction = STOP;
        } else if (dirStr.equalsIgnoreCase("BR")) {
          direction = BRAKE;
        } else {
          Serial.println("Invalid direction. Use: F, B, S, or BR");
          return;
        }
        
        if (speed >= 0 && speed <= 255) {
          dcConfig.moveMotor(motor, direction, speed);
          Serial.print("Motor ");
          Serial.print(motorChar);
          Serial.print(": ");
          Serial.print(dirStr);
          Serial.print(" at speed ");
          Serial.println(speed);
          buzzerConfig.successBeep();
        } else {
          Serial.println("Invalid speed. Range: 0-255");
        }
      } else {
        Serial.println("Invalid command format. Use: M<motor> <direction> <speed>");
      }
    }
    // Movement commands
    else if (input.startsWith("FORWARD")) {
      int speed = 150; // Default speed
      int spaceIdx = input.indexOf(' ');
      if (spaceIdx > 0) {
        speed = input.substring(spaceIdx + 1).toInt();
        if (speed < 0 || speed > 255) {
          Serial.println("Invalid speed. Range: 0-255");
          return;
        }
      }
      dcConfig.moveForward(speed);
      Serial.print("Moving forward at speed ");
      Serial.println(speed);
      buzzerConfig.successBeep();
    } else if (input.startsWith("BACKWARD")) {
      int speed = 150; // Default speed
      int spaceIdx = input.indexOf(' ');
      if (spaceIdx > 0) {
        speed = input.substring(spaceIdx + 1).toInt();
        if (speed < 0 || speed > 255) {
          Serial.println("Invalid speed. Range: 0-255");
          return;
        }
      }
      dcConfig.moveBackward(speed);
      Serial.print("Moving backward at speed ");
      Serial.println(speed);
      buzzerConfig.successBeep();
    } else if (input.startsWith("RIGHT")) {
      int speed = 150; // Default speed
      int spaceIdx = input.indexOf(' ');
      if (spaceIdx > 0) {
        speed = input.substring(spaceIdx + 1).toInt();
        if (speed < 0 || speed > 255) {
          Serial.println("Invalid speed. Range: 0-255");
          return;
        }
      }
      dcConfig.rotateRight(speed);
      Serial.print("Rotating right at speed ");
      Serial.println(speed);
      buzzerConfig.successBeep();
    } else if (input.startsWith("LEFT")) {
      int speed = 150; // Default speed
      int spaceIdx = input.indexOf(' ');
      if (spaceIdx > 0) {
        speed = input.substring(spaceIdx + 1).toInt();
        if (speed < 0 || speed > 255) {
          Serial.println("Invalid speed. Range: 0-255");
          return;
        }
      }
      dcConfig.rotateLeft(speed);
      Serial.print("Rotating left at speed ");
      Serial.println(speed);
      buzzerConfig.successBeep();
    }
    // Ultrasonic commands
    else if (input.startsWith("UTEST")) {
      int sensor = 0; // Default sensor 1 (0-indexed)
      int spaceIdx = input.indexOf(' ');
      if (spaceIdx > 0) {
        sensor = input.substring(spaceIdx + 1).toInt() - 1; // Convert to 0-indexed
        if (sensor < 0 || sensor >= NUM_ULTRASONIC) {
          Serial.println("Invalid sensor number. Range: 1-4");
          return;
        }
      }
      ultrasonicConfig.testSensor(sensor);
    } else if (input.startsWith("UDIST")) {
      int sensor = 0; // Default sensor 1
      int spaceIdx = input.indexOf(' ');
      if (spaceIdx > 0) {
        sensor = input.substring(spaceIdx + 1).toInt() - 1;
        if (sensor < 0 || sensor >= NUM_ULTRASONIC) {
          Serial.println("Invalid sensor number. Range: 1-4");
          return;
        }
      }
      long distance = ultrasonicConfig.getDistance(sensor);
      Serial.print("Ultrasonic ");
      Serial.print(sensor + 1);
      Serial.print(" Distance: ");
      if (distance == 0) {
        Serial.println("Out of range or no echo");
      } else {
        Serial.print(distance);
        Serial.println(" cm");
      }
    } else if (input.startsWith("UAVG")) {
      // Parse: UAVG <sensor> <samples>
      input = input.substring(4); // Remove 'UAVG'
      int firstSpace = input.indexOf(' ');
      int secondSpace = input.indexOf(' ', firstSpace + 1);
      
      int sensor = 0; // Default sensor 1
      int samples = 3; // Default
      
      if (firstSpace > 0) {
        sensor = input.substring(0, firstSpace).toInt() - 1;
        if (sensor < 0 || sensor >= NUM_ULTRASONIC) {
          Serial.println("Invalid sensor number. Range: 1-4");
          return;
        }
        
        if (secondSpace > firstSpace) {
          samples = input.substring(secondSpace + 1).toInt();
        } else {
          samples = input.substring(firstSpace + 1).toInt();
        }
        
        if (samples < 1 || samples > 10) {
          Serial.println("Invalid sample count. Range: 1-10");
          return;
        }
      }
      
      long avgDistance = ultrasonicConfig.getDistanceAverage(sensor, samples);
      Serial.print("Ultrasonic ");
      Serial.print(sensor + 1);
      Serial.print(" Average distance (");
      Serial.print(samples);
      Serial.print(" samples): ");
      if (avgDistance == 0) {
        Serial.println("No valid readings");
      } else {
        Serial.print(avgDistance);
        Serial.println(" cm");
      }
    } else if (input.startsWith("UDETECT")) {
      // Parse: UDETECT <sensor> <threshold>
      input = input.substring(7); // Remove 'UDETECT'
      int spaceIdx = input.indexOf(' ');
      
      int sensor = 0; // Default sensor 1
      long threshold = 50; // Default threshold
      
      if (spaceIdx > 0) {
        sensor = input.substring(0, spaceIdx).toInt() - 1;
        threshold = input.substring(spaceIdx + 1).toInt();
        
        if (sensor < 0 || sensor >= NUM_ULTRASONIC) {
          Serial.println("Invalid sensor number. Range: 1-4");
          return;
        }
        if (threshold < 1 || threshold > MAX_DISTANCE) {
          Serial.println("Invalid threshold. Range: 1-400 cm");
          return;
        }
      }
      
      bool detected = ultrasonicConfig.isObjectDetected(sensor, threshold);
      Serial.print("Ultrasonic ");
      Serial.print(sensor + 1);
      Serial.print(" Object detection (threshold: ");
      Serial.print(threshold);
      Serial.print(" cm): ");
      Serial.println(detected ? "DETECTED" : "NOT DETECTED");
    } else if (input.startsWith("UCTEST")) {
      int sensor = 0; // Default sensor 1
      int spaceIdx = input.indexOf(' ');
      if (spaceIdx > 0) {
        sensor = input.substring(spaceIdx + 1).toInt() - 1;
        if (sensor < 0 || sensor >= NUM_ULTRASONIC) {
          Serial.println("Invalid sensor number. Range: 1-4");
          return;
        }
      }
      ultrasonicConfig.startContinuousMonitor(sensor);
    } else if (input.startsWith("UCSTOP")) {
      int sensor = 0; // Default sensor 1
      int spaceIdx = input.indexOf(' ');
      if (spaceIdx > 0) {
        sensor = input.substring(spaceIdx + 1).toInt() - 1;
        if (sensor < 0 || sensor >= NUM_ULTRASONIC) {
          Serial.println("Invalid sensor number. Range: 1-4");
          return;
        }
      }
      ultrasonicConfig.stopContinuousMonitor(sensor);
    }
    // Stepper commands
    else if (input.equalsIgnoreCase("STEPTEST")) {
      stepper.setDirection(0); // CW
      stepper.stepMany(200, 100, 800); // 200 steps
      delay(500);
      stepper.setDirection(1); // CCW
      stepper.stepMany(200, 100, 800);
      Serial.println("Stepper test sequence complete.");
      buzzerConfig.successBeep();
    } else if (input.equalsIgnoreCase("STEPSTOP")) {
      stepper.emergencyStop();
      Serial.println("Stepper emergency stop.");
    } else if (input.startsWith("STEP") && !input.equalsIgnoreCase("STEPTEST") && !input.equalsIgnoreCase("STEPSTOP") && !input.equalsIgnoreCase("STEPCTEST") && !input.equalsIgnoreCase("STEPCSTOP") && !input.equalsIgnoreCase("HOME")) {
      // Parse stepper command: STEP <steps> <dir>
      input = input.substring(4); // Remove 'STEP'
      input.trim(); // Remove leading/trailing whitespace
      int spaceIdx = input.indexOf(' ');
      if (spaceIdx > 0) {
        unsigned long steps = strtoul(input.substring(0, spaceIdx).c_str(), NULL, 10);
        int dir = input.substring(spaceIdx + 1).toInt();
        if (steps > 0 && (dir == 0 || dir == 1)) {
          stepper.setDirection(dir);
          if (stepper.startSteps(steps, 100, 800)) {
            Serial.print("Starting stepper: ");
            Serial.print(steps);
            Serial.print(" steps ");
            Serial.println(dir ? "CCW" : "CW");
            buzzerConfig.successBeep();
          } else {
            Serial.println("Stepper busy, cannot start new operation.");
          }
        } else {
          Serial.println("Invalid steps or direction. Steps > 0, Dir: 0 or 1");
        }
      } else {
        Serial.println("Invalid command format. Use: STEP <steps> <dir>");
      }
    } else if (input.equalsIgnoreCase("STEPCTEST")) {
      stepper.startContinuousTest();
      Serial.println("Continuous stepper test started.");
    } else if (input.equalsIgnoreCase("STEPCSTOP")) {
      stepper.stopContinuousTest();
      Serial.println("Continuous stepper test stopped.");
    } else if (input.equalsIgnoreCase("HOME")) {
      if (!stepperLimitTestingActive) {
        stepper.setDirection(1); // CCW direction
        stepper.startSteps(10000, 750, 1500); // Start with large number of steps (25% faster)
        stepperLimitTestingActive = true;
        stepperNeedsRestart = true;
        Serial.println("Stepper rotating CCW until BIN limit switch is triggered...");
      } else {
        Serial.println("Stepper limit testing already active.");
      }
    }
    // Buzzer commands
    else if (input.equalsIgnoreCase("BTEST")) {
      buzzerConfig.beep();
      Serial.println("Buzzer test beep.");
    } else if (input.equalsIgnoreCase("BSUCCESS")) {
      buzzerConfig.successBeep();
      Serial.println("Success beep pattern.");
    } else if (input.equalsIgnoreCase("BERROR")) {
      buzzerConfig.errorBeep();
      Serial.println("Error beep pattern.");
    } else if (input.equalsIgnoreCase("BWARNING")) {
      buzzerConfig.warningBeep();
      Serial.println("Warning beep pattern.");
    }
    // Limit switch commands
    else if (input.equalsIgnoreCase("LTEST")) {
      bool armState = digitalRead(ARM_LIMIT_PIN);
      bool binState = digitalRead(BIN_LIMIT_PIN);
      Serial.println("=== Limit Switch Test ===");
      Serial.print("Arm Limit (Pin 52): ");
      Serial.println(armState ? "OPEN" : "PRESSED");
      Serial.print("Bin Limit (Pin 53): ");
      Serial.println(binState ? "OPEN" : "PRESSED");
      Serial.println("=========================");
      buzzerConfig.successBeep();
    } else if (input.equalsIgnoreCase("LREAD")) {
      bool armState = digitalRead(ARM_LIMIT_PIN);
      bool binState = digitalRead(BIN_LIMIT_PIN);
      Serial.print("Arm: ");
      Serial.print(armState ? "OPEN" : "PRESSED");
      Serial.print(", Bin: ");
      Serial.println(binState ? "OPEN" : "PRESSED");
    } else if (input.equalsIgnoreCase("LCTEST")) {
      limitTestingActive = true;
      // Initialize previous states
      lastArmState = digitalRead(ARM_LIMIT_PIN);
      lastBinState = digitalRead(BIN_LIMIT_PIN);
      Serial.println("Continuous limit switch testing started.");
      Serial.println("Will only print when switches are PRESSED. Use LCSTOP to stop.");
    } else if (input.equalsIgnoreCase("LCSTOP")) {
      limitTestingActive = false;
      Serial.println("Continuous limit switch testing stopped.");
    } else {
      buzzerConfig.errorBeep();
      Serial.println("Unknown command.");
      Serial.println("Servo: TEST, S<servo> <angle>, STEST, SSTOP, SENABLE, SDISABLE");
      Serial.println("Motor: FORWARD <speed>, BACKWARD <speed>, RIGHT <speed>, LEFT <speed>, MSTOP");
      Serial.println("  Individual: M<motor> <direction> <speed>");
      Serial.println("Ultrasonic: UTEST <sensor>, UDIST <sensor>, UAVG <sensor> <samples>, UDETECT <sensor> <threshold>, UCTEST <sensor>, UCSTOP <sensor>");
      Serial.println("Stepper: STEPTEST, STEP <steps> <dir>, STEPSTOP, STEPCTEST, STEPCSTOP, HOME");
      Serial.println("Buzzer: BTEST, BSUCCESS, BERROR, BWARNING");
      Serial.println("Limit Switch: LTEST, LREAD, LCTEST, LCSTOP");
    }
  }
}