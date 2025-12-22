#ifndef ULTRASONIC_CONFIG_H
#define ULTRASONIC_CONFIG_H

#include <Arduino.h>
#include "PINS.h"

// Number of ultrasonic sensors
#define NUM_ULTRASONIC 4

// Measurement constants
#define MAX_DISTANCE 400  // Maximum distance in cm (HC-SR04 limit)
#define TIMEOUT 30000     // Timeout in microseconds

class UltrasonicConfig {
public:
    UltrasonicConfig();
    void begin();
    long getDistance(uint8_t sensor);
    long getDistanceAverage(uint8_t sensor, uint8_t samples = 3);
    bool isObjectDetected(uint8_t sensor, long threshold);
    void testSensor(uint8_t sensor);
    void testAllSensors();
    void startContinuousMonitor(uint8_t sensor);
    void stopContinuousMonitor(uint8_t sensor);
    void update();
private:
    long measureDistance(uint8_t sensor);
    uint8_t _trigPins[NUM_ULTRASONIC];
    uint8_t _echoPins[NUM_ULTRASONIC];
    bool _continuousMonitor[NUM_ULTRASONIC];
    unsigned long _monitorStartTime[NUM_ULTRASONIC];
    unsigned long _lastReadTime[NUM_ULTRASONIC];
};

#endif // ULTRASONIC_CONFIG_H
