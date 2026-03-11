#include "ULTRASONIC_CONFIG.h"

UltrasonicConfig::UltrasonicConfig() {
    // Initialize pin arrays
    _trigPins[0] = ULTRASONIC1_TRIG_PIN;
    _echoPins[0] = ULTRASONIC1_ECHO_PIN;
    
    // Initialize monitoring arrays
    for (uint8_t i = 0; i < NUM_ULTRASONIC; i++) {
        _continuousMonitor[i] = false;
        _monitorStartTime[i] = 0;
        _lastReadTime[i] = 0;
    }

    // Initialize front sensor mean-averaging buffers
    for (uint8_t i = 0; i < FRONT_AVG_SAMPLES; i++) {
        _frontLeftBuf[i]  = 0;
        _frontRightBuf[i] = 0;
    }
    _frontLeftIdx    = 0;
    _frontRightIdx   = 0;
    _frontLeftCount  = 0;
    _frontRightCount = 0;
    _frontSensorTurn = 0;
    _lastFrontUpdate = 0;
}

void UltrasonicConfig::begin() {
    for (uint8_t i = 0; i < NUM_ULTRASONIC; i++) {
        pinMode(_trigPins[i], OUTPUT);
        pinMode(_echoPins[i], INPUT);
        digitalWrite(_trigPins[i], LOW);
    }
    // Front obstacle avoidance sensors
    pinMode(FRONT_LEFT_TRIG,  OUTPUT);
    pinMode(FRONT_LEFT_ECHO,  INPUT);
    pinMode(FRONT_RIGHT_TRIG, OUTPUT);
    pinMode(FRONT_RIGHT_ECHO, INPUT);
    digitalWrite(FRONT_LEFT_TRIG,  LOW);
    digitalWrite(FRONT_RIGHT_TRIG, LOW);
    delayMicroseconds(2);
}

long UltrasonicConfig::measureDistance(uint8_t sensor) {
    if (sensor >= NUM_ULTRASONIC) return 0;
    
    // Send ultrasonic pulse
    digitalWrite(_trigPins[sensor], LOW);
    delayMicroseconds(2);
    digitalWrite(_trigPins[sensor], HIGH);
    delayMicroseconds(10);
    digitalWrite(_trigPins[sensor], LOW);
    
    // Read echo pulse
    long duration = pulseIn(_echoPins[sensor], HIGH, TIMEOUT);
    
    // Calculate distance in cm
    // Speed of sound is 343 m/s or 29.1 microseconds per cm
    // Distance = (duration / 2) / 29.1
    long distance = duration / 58.2;
    
    // Return 0 if out of range
    if (distance == 0 || distance > MAX_DISTANCE) {
        return 0;
    }
    
    return distance;
}

long UltrasonicConfig::getDistance(uint8_t sensor) {
    return measureDistance(sensor);
}

long UltrasonicConfig::getDistanceAverage(uint8_t sensor, uint8_t samples) {
    if (sensor >= NUM_ULTRASONIC) return 0;
    
    long sum = 0;
    uint8_t validSamples = 0;
    
    for (uint8_t i = 0; i < samples; i++) {
        long distance = measureDistance(sensor);
        if (distance > 0) {
            sum += distance;
            validSamples++;
        }
        delay(10); // Small delay between measurements
    }
    
    if (validSamples == 0) return 0;
    return sum / validSamples;
}

bool UltrasonicConfig::isObjectDetected(uint8_t sensor, long threshold) {
    if (sensor >= NUM_ULTRASONIC) return false;
    
    long distance = getDistance(sensor);
    return (distance > 0 && distance <= threshold);
}

void UltrasonicConfig::testSensor(uint8_t sensor) {
    if (sensor >= NUM_ULTRASONIC) return;
    
    Serial.print("=== Ultrasonic Sensor ");
    Serial.print(sensor + 1);
    Serial.println(" Test ===");
    Serial.println("Taking 10 readings...");
    
    for (int i = 0; i < 10; i++) {
        long distance = getDistance(sensor);
        Serial.print("Reading ");
        Serial.print(i + 1);
        Serial.print(": ");
        
        if (distance == 0) {
            Serial.println("Out of range or no echo");
        } else {
            Serial.print(distance);
            Serial.println(" cm");
        }
        
        delay(500);
    }
    
    Serial.println("\nAverage distance test (5 samples):");
    long avgDistance = getDistanceAverage(sensor, 5);
    if (avgDistance == 0) {
        Serial.println("No valid readings");
    } else {
        Serial.print("Average distance: ");
        Serial.print(avgDistance);
        Serial.println(" cm");
    }
    
    Serial.println("=== Test Complete ===");
}

void UltrasonicConfig::testAllSensors() {
    for (uint8_t i = 0; i < NUM_ULTRASONIC; i++) {
        testSensor(i);
        Serial.println();
    }
}

void UltrasonicConfig::startContinuousMonitor(uint8_t sensor) {
    if (sensor >= NUM_ULTRASONIC) return;
    
    _continuousMonitor[sensor] = true;
    _monitorStartTime[sensor] = millis();
    _lastReadTime[sensor] = 0;
    Serial.print("Ultrasonic ");
    Serial.print(sensor + 1);
    Serial.println(" continuous monitoring started (20 seconds).");
}

void UltrasonicConfig::stopContinuousMonitor(uint8_t sensor) {
    if (sensor >= NUM_ULTRASONIC) return;
    
    _continuousMonitor[sensor] = false;
    Serial.print("Ultrasonic ");
    Serial.print(sensor + 1);
    Serial.println(" continuous monitoring stopped.");
}

void UltrasonicConfig::update() {
    unsigned long now = millis();
    
    for (uint8_t sensor = 0; sensor < NUM_ULTRASONIC; sensor++) {
        if (_continuousMonitor[sensor]) {
            // Check if 20 seconds elapsed
            if (now - _monitorStartTime[sensor] >= 20000) {
                stopContinuousMonitor(sensor);
                continue;
            }
            
            // Read distance every 500ms
            if (now - _lastReadTime[sensor] >= 500) {
                long distance = getDistance(sensor);
                Serial.print("Ultrasonic ");
                Serial.print(sensor + 1);
                Serial.print(" Distance: ");
                if (distance == 0) {
                    Serial.println("Out of range");
                } else {
                    Serial.print(distance);
                    Serial.println(" cm");
                }
                _lastReadTime[sensor] = now;
            }
        }
    }

    // Non-blocking front sensor mean-averaging: alternate one sensor per interval
    if (now - _lastFrontUpdate >= FRONT_SENSOR_INTERVAL) {
        if (_frontSensorTurn == 0) {
            long d = measureDistanceOnPins(FRONT_LEFT_TRIG, FRONT_LEFT_ECHO);
            _frontLeftBuf[_frontLeftIdx] = d;
            _frontLeftIdx = (_frontLeftIdx + 1) % FRONT_AVG_SAMPLES;
            if (_frontLeftCount < FRONT_AVG_SAMPLES) _frontLeftCount++;
        } else {
            long d = measureDistanceOnPins(FRONT_RIGHT_TRIG, FRONT_RIGHT_ECHO);
            _frontRightBuf[_frontRightIdx] = d;
            _frontRightIdx = (_frontRightIdx + 1) % FRONT_AVG_SAMPLES;
            if (_frontRightCount < FRONT_AVG_SAMPLES) _frontRightCount++;
        }
        _frontSensorTurn = 1 - _frontSensorTurn;
        _lastFrontUpdate = now;
    }
}

// --- Front obstacle avoidance sensor helpers ---

long UltrasonicConfig::measureDistanceOnPins(uint8_t trigPin, uint8_t echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH, TIMEOUT);
    long distance = duration / 58.2;

    if (distance == 0 || distance > MAX_DISTANCE) {
        return 0;
    }
    return distance;
}

long UltrasonicConfig::readFrontLeftDistance() {
    return measureDistanceOnPins(FRONT_LEFT_TRIG, FRONT_LEFT_ECHO);
}

long UltrasonicConfig::readFrontRightDistance() {
    return measureDistanceOnPins(FRONT_RIGHT_TRIG, FRONT_RIGHT_ECHO);
}

// getAvgFrontLeftDistance() and getAvgFrontRightDistance() are defined
// inline in ULTRASONIC_CONFIG.h to guarantee visibility at link time.