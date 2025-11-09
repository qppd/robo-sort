
#include <Wire.h>
#include "SERVO_CONFIG.h"
#include "DC_CONFIG.h"
#include "ULTRASONIC_CONFIG.h"

ServoConfig servoConfig;
DCConfig dcConfig;
UltrasonicConfig ultrasonicConfig;

void setup() {
  Serial.begin(9600);
  servoConfig.begin();
  dcConfig.begin();
  ultrasonicConfig.begin();
  Serial.println("RoboSort Control System Ready!");
  Serial.println("Servo Commands: TEST, S<servo> <angle>");
  Serial.println("Motor Commands: MTEST, M<motor> <direction> <speed>, MSTOP");
  Serial.println("  Motors: A or B");
  Serial.println("  Directions: F (forward), B (backward), S (stop), BR (brake)");
  Serial.println("  Speed: 0-255");
  Serial.println("Ultrasonic Commands: UTEST, UDIST, UAVG <samples>, UDETECT <threshold>");
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    // Servo commands
    if (input.equalsIgnoreCase("TEST")) {
      servoConfig.testServos();
      Serial.println("Servo test sequence complete.");
    } else if (input.startsWith("S") && !input.startsWith("ST")) {
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
        } else {
          Serial.println("Invalid servo number or angle.");
        }
      } else {
        Serial.println("Invalid command format. Use: S<servo> <angle>");
      }
    }
    // Motor commands
    else if (input.equalsIgnoreCase("MTEST")) {
      dcConfig.testMotors();
      Serial.println("Motor test sequence complete.");
    } else if (input.equalsIgnoreCase("MSTOP")) {
      dcConfig.stopAll();
      Serial.println("All motors stopped.");
    } else if (input.startsWith("M") && !input.equalsIgnoreCase("MTEST") && !input.equalsIgnoreCase("MSTOP")) {
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
        } else {
          Serial.println("Invalid speed. Range: 0-255");
        }
      } else {
        Serial.println("Invalid command format. Use: M<motor> <direction> <speed>");
      }
    }
    // Ultrasonic commands
    else if (input.equalsIgnoreCase("UTEST")) {
      ultrasonicConfig.testSensor();
    } else if (input.equalsIgnoreCase("UDIST")) {
      long distance = ultrasonicConfig.getDistance();
      if (distance == 0) {
        Serial.println("Distance: Out of range or no echo");
      } else {
        Serial.print("Distance: ");
        Serial.print(distance);
        Serial.println(" cm");
      }
    } else if (input.startsWith("UAVG")) {
      int spaceIdx = input.indexOf(' ');
      int samples = 3; // Default
      if (spaceIdx > 0) {
        samples = input.substring(spaceIdx + 1).toInt();
        if (samples < 1 || samples > 10) {
          Serial.println("Invalid sample count. Range: 1-10");
          return;
        }
      }
      long avgDistance = ultrasonicConfig.getDistanceAverage(samples);
      if (avgDistance == 0) {
        Serial.println("Average distance: No valid readings");
      } else {
        Serial.print("Average distance (");
        Serial.print(samples);
        Serial.print(" samples): ");
        Serial.print(avgDistance);
        Serial.println(" cm");
      }
    } else if (input.startsWith("UDETECT")) {
      int spaceIdx = input.indexOf(' ');
      if (spaceIdx > 0) {
        long threshold = input.substring(spaceIdx + 1).toInt();
        if (threshold < 1 || threshold > MAX_DISTANCE) {
          Serial.println("Invalid threshold. Range: 1-400 cm");
          return;
        }
        bool detected = ultrasonicConfig.isObjectDetected(threshold);
        Serial.print("Object detection (threshold: ");
        Serial.print(threshold);
        Serial.print(" cm): ");
        Serial.println(detected ? "DETECTED" : "NOT DETECTED");
      } else {
        Serial.println("Invalid command format. Use: UDETECT <threshold>");
      }
    } else {
      Serial.println("Unknown command.");
      Serial.println("Servo: TEST, S<servo> <angle>");
      Serial.println("Motor: MTEST, M<motor> <direction> <speed>, MSTOP");
      Serial.println("Ultrasonic: UTEST, UDIST, UAVG <samples>, UDETECT <threshold>");
    }
  }
}