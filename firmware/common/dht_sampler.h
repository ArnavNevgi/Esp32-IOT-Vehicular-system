// Rate-limited DHT11/DHT22 sampling with cached values, freshness and status.
//
// The Adafruit driver spends ~20 ms (DHT11) or ~1 ms (DHT22) in a yielding
// start pulse, then reads the 40-bit frame with interrupts disabled on the
// calling core for roughly 4-5 ms (longer only on timeouts). Reads are
// therefore spaced out and their duration is measured.
#pragma once

#include <Arduino.h>
#include <DHT.h>

#include "vc_protocol.h"
#include "vc_timing.h"

class DhtSampler {
 public:
  DhtSampler(uint8_t pin, uint8_t type) : dht_(pin, type), type_(type) {}

  // intervalMs: time between reads (>= 2000, the driver's minimum).
  // freshMs: a value older than this is reported Stale.
  // maxCacheMs: a value older than this is no longer reported at all.
  void begin(uint32_t now, uint32_t intervalMs, uint32_t freshMs, uint32_t maxCacheMs, uint32_t firstReadDelayMs);

  // Reads the sensor if a read is due. Returns true when a read was attempted.
  bool poll(uint32_t now);

  vc::proto::DhtReading reading(uint32_t now) const;
  uint32_t lastReadDurationUs() const { return lastReadUs_; }
  uint32_t maxReadDurationUs() const { return maxReadUs_; }
  bool lastReadOk() const { return lastOk_; }

 private:
  DHT dht_;
  uint8_t type_;
  vc::PeriodicTimer timer_;
  uint32_t freshMs_ = 0, maxCacheMs_ = 0;
  float tempC_ = NAN, humidity_ = NAN;
  uint32_t valueAtMs_ = 0;
  bool hasValue_ = false;
  bool lastOk_ = false;
  uint16_t errors_ = 0;
  uint32_t attempts_ = 0;
  uint32_t lastReadUs_ = 0, maxReadUs_ = 0;
};
