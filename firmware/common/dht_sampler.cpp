#include "dht_sampler.h"

using vc::proto::SensorStatus;

void DhtSampler::begin(uint32_t now, uint32_t intervalMs, uint32_t freshMs, uint32_t maxCacheMs,
                       uint32_t firstReadDelayMs) {
  dht_.begin();
  if (intervalMs < 2000) intervalMs = 2000;
  timer_.setPeriod(intervalMs);
  // First read after firstReadDelayMs (sensor power-up settle / staggering).
  timer_.start(now - intervalMs + firstReadDelayMs);
  freshMs_ = freshMs;
  maxCacheMs_ = maxCacheMs;
}

bool DhtSampler::poll(uint32_t now) {
  if (!timer_.due(now)) return false;
  uint32_t t0 = micros();
  // force=true: our timer already enforces the minimum interval.
  bool ok = dht_.read(true);
  float h = ok ? dht_.readHumidity(false) : NAN;
  float t = ok ? dht_.readTemperature(false, false) : NAN;
  lastReadUs_ = micros() - t0;
  if (lastReadUs_ > maxReadUs_) maxReadUs_ = lastReadUs_;
  ++attempts_;
  // Range limits of the respective sensor (DHT11: 0-50 C, DHT22: -40-80 C).
  float tMin = type_ == DHT11 ? 0.0f : -40.0f, tMax = type_ == DHT11 ? 50.0f : 80.0f;
  lastOk_ = ok && isfinite(t) && isfinite(h) && t >= tMin && t <= tMax && h >= 0 && h <= 100;
  if (lastOk_) {
    tempC_ = t;
    humidity_ = h;
    valueAtMs_ = now;
    hasValue_ = true;
    errors_ = 0;
  } else if (errors_ < 0xFFFF) {
    ++errors_;
  }
  return true;
}

vc::proto::DhtReading DhtSampler::reading(uint32_t now) const {
  vc::proto::DhtReading r;
  r.consecutiveErrors = errors_;
  uint32_t age = vc::elapsedMs(now, valueAtMs_);
  if (hasValue_ && age <= maxCacheMs_) {
    r.status = age <= freshMs_ ? SensorStatus::Ok : SensorStatus::Stale;
    r.tempC = tempC_;
    r.humidityPct = humidity_;
    r.ageMs = age;
  } else {
    // Never read successfully after several attempts: treat as not connected.
    r.status = (!hasValue_ && attempts_ >= 3) ? SensorStatus::Absent : SensorStatus::Error;
    r.tempC = NAN;
    r.humidityPct = NAN;
  }
  return r;
}
