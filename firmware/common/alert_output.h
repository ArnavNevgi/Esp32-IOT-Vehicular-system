// Optional local LED / buzzer indication, driven without blocking.
//
// Pins are disabled with -1 (the default: no LED or buzzer is assumed to be
// installed). The buzzer output is a plain on/off level, which suits an
// *active* buzzer module; a passive piezo would need a tone generator.
//
// Precedence: a short "chirp" burst (impact) > alarm (fast blink, beeping)
// > attention (slow LED blink, silent) > off.
#pragma once

#include <Arduino.h>

class AlertOutput {
 public:
  void begin(int ledPin, bool ledActiveHigh, int buzzerPin, bool buzzerActiveHigh);
  void setAlarm(bool on) { alarm_ = on; }
  void setAttention(bool on) { attention_ = on; }
  void chirp(uint8_t beeps, uint32_t now);
  void update(uint32_t now);

 private:
  void write(int pin, bool activeHigh, bool on);
  int led_ = -1, buzzer_ = -1;
  bool ledHigh_ = true, buzzerHigh_ = true;
  bool alarm_ = false, attention_ = false;
  uint8_t chirps_ = 0;
  uint32_t chirpStart_ = 0;
};
