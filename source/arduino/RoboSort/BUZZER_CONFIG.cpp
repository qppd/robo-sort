#include "BUZZER_CONFIG.h"

BuzzerConfig::BuzzerConfig() : _beeping(false), _beepStartTime(0), _beepDuration(0) {
}

void BuzzerConfig::begin() {
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
}

void BuzzerConfig::beep(uint16_t duration) {
    digitalWrite(BUZZER_PIN, HIGH);
    _beeping = true;
    _beepStartTime = millis();
    _beepDuration = duration;
}

void BuzzerConfig::beepPattern(uint8_t numBeeps, uint16_t duration, uint16_t pause) {
    for (uint8_t i = 0; i < numBeeps; i++) {
        beep(duration);
        while (_beeping) {
            update();
        }
        delay(pause);
    }
}

void BuzzerConfig::startupBeep() {
    // Professional startup pattern: short-short-long
    beepPattern(2, BEEP_SHORT, BEEP_PAUSE);
    delay(BEEP_PAUSE);
    beep(BEEP_LONG);
}

void BuzzerConfig::successBeep() {
    // Success: two short beeps
    beepPattern(2, BEEP_SHORT, BEEP_PAUSE);
}

void BuzzerConfig::errorBeep() {
    // Error: long beep
    beep(BEEP_LONG);
}

void BuzzerConfig::warningBeep() {
    // Warning: three short beeps
    beepPattern(3, BEEP_SHORT, BEEP_PAUSE);
}

void BuzzerConfig::update() {
    if (_beeping && (millis() - _beepStartTime >= _beepDuration)) {
        digitalWrite(BUZZER_PIN, LOW);
        _beeping = false;
    }
}