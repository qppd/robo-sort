
#include <Wire.h>
#include "SERVO_CONFIG.h"

ServoConfig servoConfig;

void setup() {
  Serial.begin(9600);
  servoConfig.begin();
  Serial.println("RoboSort Servo Test Ready. Commands: TEST, S<servo> <angle>");
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.equalsIgnoreCase("TEST")) {
      servoConfig.testServos();
      Serial.println("Test sequence complete.");
    } else if (input.startsWith("S")) {
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
    } else {
      Serial.println("Unknown command. Use: TEST or S<servo> <angle>");
    }
  }
}