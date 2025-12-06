#include "ULTRASONIC_CONFIG.h"

UltrasonicConfig::UltrasonicConfig() : _continuousMonitor(false), _monitorStartTime(0), _lastReadTime(0) {
    // Constructor
}

void UltrasonicConfig::begin() {
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
}

long UltrasonicConfig::measureDistance() {
    // Send ultrasonic pulse
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    
    // Read echo pulse
    long duration = pulseIn(ECHO_PIN, HIGH, TIMEOUT);
    
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

long UltrasonicConfig::getDistance() {
    return measureDistance();
}

long UltrasonicConfig::getDistanceAverage(uint8_t samples) {
    long sum = 0;
    uint8_t validSamples = 0;
    
    for (uint8_t i = 0; i < samples; i++) {
        long distance = measureDistance();
        if (distance > 0) {
            sum += distance;
            validSamples++;
        }
        delay(10); // Small delay between samples
    }
    
    if (validSamples == 0) {
        return 0;
    }
    
    return sum / validSamples;
}

bool UltrasonicConfig::isObjectDetected(long threshold) {
    long distance = getDistance();
    return (distance > 0 && distance < threshold);
}

void UltrasonicConfig::testSensor() {
    Serial.println("=== Ultrasonic Sensor Test ===");
    Serial.println("Taking 10 readings...");
    
    for (int i = 0; i < 10; i++) {
        long distance = getDistance();
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
    long avgDistance = getDistanceAverage(5);
    if (avgDistance == 0) {
        Serial.println("No valid readings");
    } else {
        Serial.print("Average distance: ");
        Serial.print(avgDistance);
        Serial.println(" cm");
    }
    
    Serial.println("=== Test Complete ===");
}

void UltrasonicConfig::startContinuousMonitor() {
    _continuousMonitor = true;
    _monitorStartTime = millis();
    _lastReadTime = 0;
    Serial.println("Ultrasonic continuous monitoring started (20 seconds).");
}

void UltrasonicConfig::stopContinuousMonitor() {
    _continuousMonitor = false;
    Serial.println("Ultrasonic continuous monitoring stopped.");
}

void UltrasonicConfig::update() {
    if (_continuousMonitor) {
        unsigned long now = millis();
        
        // Check if 20 seconds elapsed
        if (now - _monitorStartTime >= 20000) {
            stopContinuousMonitor();
            return;
        }
        
        // Read distance every 500ms
        if (now - _lastReadTime >= 500) {
            long distance = getDistance();
            Serial.print("Distance: ");
            if (distance == 0) {
                Serial.println("Out of range");
            } else {
                Serial.print(distance);
                Serial.println(" cm");
            }
            _lastReadTime = now;
        }
    }
}
