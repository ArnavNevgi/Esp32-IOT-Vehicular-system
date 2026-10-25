// Gateway-side aggregation of sensor-node telemetry and gateway sensors into
// one routine-summary window (default 60 s).
#pragma once

#include <stdint.h>

#include "vc_protocol.h"
#include "vc_util.h"

namespace vc {

struct DhtWindow {
  WindowStats tempC;
  WindowStats humidityPct;
  proto::SensorStatus status = proto::SensorStatus::Absent;  // latest status
  uint32_t ageMs = 0;                                        // latest value age
  uint16_t consecutiveErrors = 0;
  bool seen = false;
  uint32_t samples = 0;  // distinct measurements in this window
};

struct SensorNodeWindow {
  bool everSeen = false;
  bool synthetic = false;
  uint32_t bootId = 0;
  uint32_t packets = 0;
  DhtWindow dht[2];
  proto::SensorStatus imuStatus = proto::SensorStatus::Absent;
  WindowStats accelG;      // |a| of each received packet's latest sample
  WindowStats peakAccelG;  // per-packet peaks -> window max
  WindowStats rollDeg;
  WindowStats pitchDeg;
  WindowStats imuRateHz;
  uint16_t imuMaxGapMs = 0;
  uint8_t features = proto::kFeatureMask;  // latest kFeature* bits (PIR / shock enabled)
  bool pirActive = false;
  uint32_t pirActivePackets = 0;
  bool tiltActive = false;
  uint32_t lastPacketUptimeMs = 0;  // sensor clock of the newest telemetry
  // Deltas of the sensor's cumulative counters within this window.
  uint32_t pirRises = 0, impacts = 0, shockEdges = 0;
  uint32_t impactOverflows = 0, eventsDropped = 0, eventsExpired = 0;
  uint16_t linkRestarts = 0;
};

// The sensor repeats its cached DHT value in every telemetry packet. A value is
// counted as a new measurement only when its age went down (the sensor took a
// new reading) and its status is Ok; stale values are never counted.
inline void addDhtSample(DhtWindow& w, const proto::DhtReading& r, bool fresh);

inline void addDht(DhtWindow& w, const proto::DhtReading& r) {
  bool fresh = r.status == proto::SensorStatus::Ok && (!w.seen || !proto::statusHasValue(w.status) || r.ageMs < w.ageMs);
  addDhtSample(w, r, fresh);
}

// For a sampler read directly (the gateway's DHT22): the caller knows whether
// this reading is a new successful measurement, so ages are not compared.
inline void addDhtSample(DhtWindow& w, const proto::DhtReading& r, bool fresh) {
  fresh = fresh && r.status == proto::SensorStatus::Ok;
  w.seen = true;
  w.status = r.status;
  w.ageMs = r.ageMs;
  w.consecutiveErrors = r.consecutiveErrors;
  if (fresh) {
    w.tempC.add(r.tempC);
    w.humidityPct.add(r.humidityPct);
    ++w.samples;
  }
}

class SensorNodeAggregator {
 public:
  void onTelemetry(const proto::Header& h, const proto::TelemetryBody& t) {
    SensorNodeWindow& w = win_;
    const proto::Counters& c = t.counters;
    if (haveBaseline_ && h.bootId == baselineBoot_) {
      w.pirRises += c.pirRises - base_.pirRises;
      w.impacts += c.impacts - base_.impacts;
      w.shockEdges += c.shockEdges - base_.shockEdges;
      w.impactOverflows += c.impactOverflows - base_.impactOverflows;
      w.eventsDropped += c.eventsDropped - base_.eventsDropped;
      w.eventsExpired += c.eventsExpired - base_.eventsExpired;
      w.linkRestarts = (uint16_t)(w.linkRestarts + (uint16_t)(c.linkRestarts - base_.linkRestarts));
    } else if (haveBaseline_) {
      // Sensor rebooted: its counters restarted from zero.
      w.pirRises += c.pirRises;
      w.impacts += c.impacts;
      w.shockEdges += c.shockEdges;
      w.impactOverflows += c.impactOverflows;
      w.eventsDropped += c.eventsDropped;
      w.eventsExpired += c.eventsExpired;
      w.linkRestarts = (uint16_t)(w.linkRestarts + c.linkRestarts);
    }
    // First packet after gateway boot: counts before it are unknown, so it
    // only establishes the baseline.
    base_ = c;
    baselineBoot_ = h.bootId;
    haveBaseline_ = true;

    w.everSeen = true;
    w.bootId = h.bootId;
    w.synthetic = (h.flags & proto::kFlagSynthetic) != 0;
    w.features = t.features;
    w.lastPacketUptimeMs = h.uptimeMs;
    ++w.packets;
    addDht(w.dht[0], t.dht[0]);
    addDht(w.dht[1], t.dht[1]);
    w.imuStatus = t.imu.status;
    if (proto::statusHasValue(t.imu.status)) {
      w.accelG.add(t.imu.accelMagG);
      w.peakAccelG.add(t.imu.peakAccelG);
      w.rollDeg.add(t.imu.rollDeg);
      w.pitchDeg.add(t.imu.pitchDeg);
    }
    w.imuRateHz.add(t.imu.rateHz);
    if (t.imu.maxGapMs > w.imuMaxGapMs) w.imuMaxGapMs = t.imu.maxGapMs;
    w.pirActive = t.pirActive;
    if (t.pirActive) ++w.pirActivePackets;
    w.tiltActive = t.tiltActive;
  }

  const SensorNodeWindow& window() const { return win_; }

  // Starts a new window, keeping the latest state (status, PIR, tilt, boot).
  void resetWindow() {
    SensorNodeWindow next;
    next.everSeen = win_.everSeen;
    next.synthetic = win_.synthetic;
    next.bootId = win_.bootId;
    next.imuStatus = win_.imuStatus;
    next.features = win_.features;
    next.pirActive = win_.pirActive;
    next.tiltActive = win_.tiltActive;
    next.lastPacketUptimeMs = win_.lastPacketUptimeMs;
    for (int i = 0; i < 2; ++i) {
      next.dht[i].status = win_.dht[i].status;
      next.dht[i].ageMs = win_.dht[i].ageMs;
      next.dht[i].consecutiveErrors = win_.dht[i].consecutiveErrors;
      next.dht[i].seen = win_.dht[i].seen;
    }
    win_ = next;
  }

 private:
  SensorNodeWindow win_;
  proto::Counters base_;
  uint32_t baselineBoot_ = 0;
  bool haveBaseline_ = false;
};

struct GatewayWindow {
  DhtWindow dht22;  // status Disabled when the DHT22 is not fitted
  bool flameEnabled = true;
  WindowStats flamePct;
  bool flameActive = false;
  bool potEnabled = true;
  WindowStats potPct;
  void reset() {
    DhtWindow d;
    d.status = dht22.status;
    d.ageMs = dht22.ageMs;
    d.consecutiveErrors = dht22.consecutiveErrors;
    d.seen = dht22.seen;
    dht22 = d;
    flamePct.reset();
    potPct.reset();
  }
};

}  // namespace vc
