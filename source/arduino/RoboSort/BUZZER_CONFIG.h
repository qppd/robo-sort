#ifndef BUZZER_CONFIG_H
#define BUZZER_CONFIG_H

#include <Arduino.h>
#include "PINS.h"

// Buzzer patterns
#define BEEP_SHORT 100   // Short beep duration in ms
#define BEEP_LONG 500    // Long beep duration in ms
#define BEEP_PAUSE 100   // Pause between beeps in ms

class BuzzerConfig {
public:
    BuzzerConfig();
    void begin();
    void beep(uint16_t duration = BEEP_SHORT);
    void beepPattern(uint8_t numBeeps, uint16_t duration = BEEP_SHORT, uint16_t pause = BEEP_PAUSE);
    void startupBeep();
    void successBeep();
    void errorBeep();
    void warningBeep();
    void update();
private:
    bool _beeping;
    unsigned long _beepStartTime;
    uint16_t _beepDuration;
};

#endif // BUZZER_CONFIG_H