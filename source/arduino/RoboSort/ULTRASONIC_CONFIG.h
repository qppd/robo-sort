#ifndef ULTRASONIC_CONFIG_H
#define ULTRASONIC_CONFIG_H

#include <Arduino.h>

// Ultrasonic sensor pin definitions
#define TRIG_PIN 4
#define ECHO_PIN 5

// Measurement constants
#define MAX_DISTANCE 400  // Maximum distance in cm (HC-SR04 limit)
#define TIMEOUT 30000     // Timeout in microseconds

class UltrasonicConfig {
public:
    UltrasonicConfig();
    void begin();
    long getDistance();
    long getDistanceAverage(uint8_t samples = 3);
    bool isObjectDetected(long threshold);
    void testSensor();
private:
    long measureDistance();
};

#endif // ULTRASONIC_CONFIG_H
