#include "alert_output.h"

#include "vc_timing.h"

void AlertOutput::begin(int ledPin, bool ledActiveHigh, int buzzerPin, bool buzzerActiveHigh) {
  led_ = ledPin;
  buzzer_ = buzzerPin;
  ledHigh_ = ledActiveHigh;
  buzzerHigh_ = buzzerActiveHigh;
  if (led_ >= 0) pinMode(led_, OUTPUT);
  if (buzzer_ >= 0) pinMode(buzzer_, OUTPUT);
  write(led_, ledHigh_, false);
  write(buzzer_, buzzerHigh_, false);
}

void AlertOutput::chirp(uint8_t beeps, uint32_t now) {
  chirps_ = beeps;
  chirpStart_ = now;
}

void AlertOutput::write(int pin, bool activeHigh, bool on) {
  if (pin < 0) return;
  digitalWrite(pin, (on == activeHigh) ? HIGH : LOW);
}

void AlertOutput::update(uint32_t now) {
  bool ledOn = false, buzzOn = false;
  if (chirps_ > 0) {
    // 100 ms on / 100 ms off per beep.
    uint32_t t = vc::elapsedMs(now, chirpStart_);
    if (t >= (uint32_t)chirps_ * 200u) {
      chirps_ = 0;
    } else {
      ledOn = buzzOn = (t % 200u) < 100u;
    }
  }
  if (chirps_ == 0) {
    if (alarm_) {
      ledOn = (now % 250u) < 125u;
      buzzOn = (now % 1000u) < 150u;
    } else if (attention_) {
      ledOn = (now % 2000u) < 100u;
    }
  }
  write(led_, ledHigh_, ledOn);
  write(buzzer_, buzzerHigh_, buzzOn);
}
