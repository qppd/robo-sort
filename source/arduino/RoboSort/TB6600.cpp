#include "TB6600.h"

// Constructor
TB6600::TB6600(uint8_t dirPin, uint8_t stepPin, uint8_t enablePin)
  : _dirPin(dirPin), _stepPin(stepPin), _enablePin(enablePin), _dir(false),
    _remainingSteps(0), _pulseUs(100), _gapUs(800), _nextToggleMicros(0),
    _pulseState(false), _busy(false), _continuousTest(false), _currentDirection(0) {}

// Initialize pins
void TB6600::begin() {
  pinMode(_dirPin, OUTPUT);
  pinMode(_stepPin, OUTPUT);
  if (_enablePin != 255) {
    pinMode(_enablePin, OUTPUT);
    digitalWrite(_enablePin, HIGH); // Disable by default (assuming active low)
  }
  digitalWrite(_dirPin, _dir);
  digitalWrite(_stepPin, LOW);
}

// Enable or disable the driver
void TB6600::enable(bool en) {
  if (_enablePin != 255) {
    digitalWrite(_enablePin, !en); // Assuming active low
  }
}

// Single blocking step
void TB6600::stepOnce(unsigned int pulseUs) {
  digitalWrite(_stepPin, HIGH);
  delayMicroseconds(pulseUs);
  digitalWrite(_stepPin, LOW);
  delayMicroseconds(pulseUs); // Optional gap
}

// Multiple blocking steps
void TB6600::stepMany(unsigned long steps, unsigned int pulseUs, unsigned int gapUs) {
  for (unsigned long i = 0; i < steps; i++) {
    digitalWrite(_stepPin, HIGH);
    delayMicroseconds(pulseUs);
    digitalWrite(_stepPin, LOW);
    delayMicroseconds(gapUs);
  }
}

// Set direction
void TB6600::setDirection(bool dir) {
  _dir = dir;
  digitalWrite(_dirPin, _dir);
  Serial.print("Stepper direction: ");
  Serial.println(dir ? "CCW" : "CW");
}

// Emergency stop
void TB6600::emergencyStop() {
  _busy = false;
  _remainingSteps = 0;
  _continuousTest = false; // Also stop continuous test
  enable(false);
}

// Continuous test
void TB6600::startContinuousTest() {
  _continuousTest = true;
  _currentDirection = 0;
  setDirection(_currentDirection);
  // ~5555 steps for 5 seconds at 100us pulse + 800us gap
  startSteps(5555, 100, 800);
}

void TB6600::stopContinuousTest() {
  _continuousTest = false;
  emergencyStop();
}

// Start non-blocking steps
bool TB6600::startSteps(unsigned long steps, unsigned int pulseUs, unsigned int gapUs) {
  if (_busy) return false;
  _remainingSteps = steps;
  _pulseUs = pulseUs;
  _gapUs = gapUs;
  _nextToggleMicros = micros();
  _pulseState = false;
  _busy = true;
  enable(true);
  return true;
}

// Update non-blocking steps
void TB6600::update() {
  if (!_busy) return;
  unsigned long now = micros();
  if (now >= _nextToggleMicros) {
    if (_pulseState) {
      digitalWrite(_stepPin, LOW);
      _nextToggleMicros = now + _gapUs;
      _pulseState = false;
      _remainingSteps--;
      if (_remainingSteps == 0) {
        _busy = false;
        enable(false);
        // For continuous test, reverse direction and start again
        if (_continuousTest) {
          _currentDirection = 1 - _currentDirection;
          setDirection(_currentDirection);
          // Calculate steps for 5 seconds: 5000ms / (pulseUs + gapUs) microseconds per step
          // Using 100us pulse + 800us gap = 900us per step = ~5555 steps in 5 seconds
          startSteps(5555, 100, 800);
        }
      }
    } else {
      digitalWrite(_stepPin, HIGH);
      _nextToggleMicros = now + _pulseUs;
      _pulseState = true;
    }
  }
}