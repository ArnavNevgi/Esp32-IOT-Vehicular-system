#include "shock_capture.h"

#include <Arduino.h>
#include <esp_timer.h>

#include "vc_timing.h"

namespace shock {

namespace {

const unsigned kRing = 8;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
ImpactRecord ring[kRing];
volatile unsigned head = 0, tail = 0;  // head: next write, tail: next read
volatile bool openRecord = false;       // ring[head-1] still collecting pulses
volatile int64_t lastImpactUs = 0;
volatile bool haveImpact = false;
volatile uint32_t edgeCount = 0, impactCount = 0, overflowCount = 0;
int64_t refractoryUs = 200000;
uint32_t refractoryMsCfg = 200;

void IRAM_ATTR onEdge() {
  int64_t now = esp_timer_get_time();  // IRAM-safe, same base as millis()
  portENTER_CRITICAL_ISR(&mux);
  edgeCount = edgeCount + 1;
  if (!haveImpact || now - lastImpactUs >= refractoryUs) {
    haveImpact = true;
    lastImpactUs = now;
    impactCount = impactCount + 1;
    unsigned next = (head + 1) % kRing;
    if (next == tail) {
      overflowCount = overflowCount + 1;
      openRecord = false;
    } else {
      ring[head].detectedMs = (uint32_t)(now / 1000);
      ring[head].pulses = 1;
      head = next;
      openRecord = true;
    }
  } else if (openRecord) {
    ImpactRecord& r = ring[(head + kRing - 1) % kRing];
    if (r.pulses < 0xFFFF) ++r.pulses;
  }
  portEXIT_CRITICAL_ISR(&mux);
}

}  // namespace

void begin(int pin, bool activeHigh, uint32_t refractoryMs) {
  refractoryMsCfg = refractoryMs;
  refractoryUs = (int64_t)refractoryMs * 1000;
  // Keep the input at its inactive level when the module output floats.
  pinMode(pin, activeHigh ? INPUT_PULLDOWN : INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pin), onEdge, activeHigh ? RISING : FALLING);
}

bool pop(ImpactRecord& out, uint32_t nowMs) {
  bool got = false;
  portENTER_CRITICAL(&mux);
  if (tail != head && vc::hasElapsed(nowMs, ring[tail].detectedMs, refractoryMsCfg)) {
    out = ring[tail];
    tail = (tail + 1) % kRing;
    if (tail == head) openRecord = false;
    got = true;
  }
  portEXIT_CRITICAL(&mux);
  return got;
}

uint32_t edges() { return edgeCount; }
uint32_t impacts() { return impactCount; }
uint32_t overflows() { return overflowCount; }

}  // namespace shock
