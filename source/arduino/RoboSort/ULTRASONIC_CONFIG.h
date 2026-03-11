#ifndef ULTRASONIC_CONFIG_H
#define ULTRASONIC_CONFIG_H

#include <Arduino.h>
#include "PINS.h"

// Number of ultrasonic sensors
#define NUM_ULTRASONIC 1

// Measurement constants
#define MAX_DISTANCE 400      // Maximum distance in cm (HC-SR04 limit)
#define TIMEOUT 30000         // Timeout in microseconds
#define OBSTACLE_DISTANCE 25  // Legacy front obstacle threshold (kept for compatibility)

// --- Per-sensor front collision distance limits (easy to tune) ---
#define FRONT_LEFT_WARN_CM       35   // Left sensor: start gentle arc correction (cm)
#define FRONT_LEFT_COLLISION_CM  20   // Left sensor: hard stop + reverse + spot-turn (cm)
#define FRONT_RIGHT_WARN_CM      35   // Right sensor: start gentle arc correction (cm)
#define FRONT_RIGHT_COLLISION_CM 20   // Right sensor: hard stop + reverse + spot-turn (cm)

// Front sensor mean-averaging
#define FRONT_AVG_SAMPLES 5         // Rolling window size for mean filter
#define FRONT_SENSOR_INTERVAL 100   // ms between each individual front sensor update

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
    // Front obstacle avoidance sensors
    long readFrontLeftDistance();
    long readFrontRightDistance();
    // Mean-averaged front sensor readings (updated non-blocking in update())
    long getAvgFrontLeftDistance();
    long getAvgFrontRightDistance();
private:
    long measureDistance(uint8_t sensor);
    long measureDistanceOnPins(uint8_t trigPin, uint8_t echoPin);
    uint8_t _trigPins[NUM_ULTRASONIC];
    uint8_t _echoPins[NUM_ULTRASONIC];
    bool _continuousMonitor[NUM_ULTRASONIC];
    unsigned long _monitorStartTime[NUM_ULTRASONIC];
    unsigned long _lastReadTime[NUM_ULTRASONIC];
    // Front sensor mean-averaging ring buffers
    long _frontLeftBuf[FRONT_AVG_SAMPLES];
    long _frontRightBuf[FRONT_AVG_SAMPLES];
    uint8_t _frontLeftIdx;
    uint8_t _frontRightIdx;
    uint8_t _frontLeftCount;
    uint8_t _frontRightCount;
    uint8_t _frontSensorTurn;   // 0=left next, 1=right next (alternates each update)
    unsigned long _lastFrontUpdate;
};

#endif // ULTRASONIC_CONFIG_H
