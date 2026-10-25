#include "vc_sensor_app.h"

#include <math.h>

#include "vc_log.h"

namespace vc {

using namespace proto;

void SensorApp::begin(const SensorAppConfig& cfg, SensorLinkCore* link) {
  cfg_ = cfg;
  link_ = link;
  tilt_ = HysteresisAlarm(cfg.tiltOnDeg, cfg.tiltOffDeg, cfg.tiltActivateMs, cfg.tiltClearMs);
}

EventBody SensorApp::newEvent(EventType type, EventState state, uint32_t detectedMs) {
  EventBody e;
  e.eventCounter = ++eventCounter_;
  e.type = type;
  e.state = state;
  e.detectedUptimeMs = detectedMs;
  return e;
}

void SensorApp::onPirSample(bool level) {
  if (!cfg_.pirEnabled) return;
  if (level && !pirLevel_) ++pirRises_;
  pirLevel_ = level;
}

AlarmTransition SensorApp::evaluateTilt(const ImuSample& s, uint32_t now) {
  if (!cfg_.imuEnabled) return AlarmTransition::None;
  bool valid = s.status == SensorStatus::Ok && sampleAge(now, s.sampleMs) < cfg_.imuMaxAgeMs;
  float angle = fmaxf(fabsf(s.rollDeg), fabsf(s.pitchDeg));
  AlarmTransition t = tilt_.update(angle, valid, now);
  if (t == AlarmTransition::None) return t;
  bool active = t == AlarmTransition::Activated;
  EventBody e = newEvent(EventType::Tilt, active ? EventState::Active : EventState::Cleared, now);
  e.angleDeg = active ? angle : tilt_.peak();
  if (link_) link_->queueEvent(e);
  logf("[event] tilt %s (%.1f deg)", active ? "ACTIVE" : "cleared", (double)e.angleDeg);
  return t;
}

void SensorApp::onImpact(uint32_t detectedMs, uint16_t pulses, float peakAccelG) {
  if (!cfg_.shockEnabled) return;
  EventBody e = newEvent(EventType::Impact, EventState::Detected, detectedMs);
  e.pulseCount = pulses;
  e.peakAccelG = cfg_.imuEnabled ? peakAccelG : 0;
  if (link_) link_->queueEvent(e);
  logf("[event] impact #%lu at %lu ms, %u pulse(s), peak %.2f g", (unsigned long)e.eventCounter,
       (unsigned long)detectedMs, (unsigned)pulses, (double)e.peakAccelG);
}

TelemetryBody SensorApp::buildTelemetry(uint32_t now, const DhtReading& a, const DhtReading& b, const ImuSample& s,
                                        const ShockCounters& shock) const {
  TelemetryBody t;
  const DhtReading* in[2] = {&a, &b};
  for (int i = 0; i < 2; ++i) {
    if (cfg_.dhtEnabled[i]) {
      t.dht[i] = *in[i];
    } else {
      t.dht[i] = DhtReading();
      t.dht[i].status = SensorStatus::Disabled;
    }
  }
  ImuReading& m = t.imu;
  if (!cfg_.imuEnabled) {
    m = ImuReading();
    m.status = SensorStatus::Disabled;
  } else {
    m.status = s.status;
    uint32_t age = sampleAge(now, s.sampleMs);
    if (s.status == SensorStatus::Ok && age >= cfg_.imuMaxAgeMs) m.status = SensorStatus::Stale;
    m.accelXG = s.accelXG;
    m.accelYG = s.accelYG;
    m.accelZG = s.accelZG;
    m.accelMagG = s.accelMagG;
    m.peakAccelG = s.peakAccelG;
    m.rollDeg = s.rollDeg;
    m.pitchDeg = s.pitchDeg;
    m.rateHz = s.rateHz;
    m.maxGapMs = s.maxGapMs;
    m.ageMs = age;
  }
  t.features = (uint8_t)((cfg_.pirEnabled ? kFeaturePir : 0) | (cfg_.shockEnabled ? kFeatureShock : 0));
  t.pirActive = cfg_.pirEnabled && pirLevel_;
  t.tiltActive = cfg_.imuEnabled && tilt_.active();
  t.counters.pirRises = cfg_.pirEnabled ? pirRises_ : 0;
  if (cfg_.shockEnabled) {
    t.counters.impacts = shock.impacts;
    t.counters.shockEdges = shock.edges;
    t.counters.impactOverflows = shock.overflows;
  }
  if (link_) {
    t.counters.eventsDropped = link_->eventsDropped();
    t.counters.eventsExpired = link_->eventsExpired();
    t.counters.linkRestarts = link_->linkRestarts();
  }
  return t;
}

}  // namespace vc
