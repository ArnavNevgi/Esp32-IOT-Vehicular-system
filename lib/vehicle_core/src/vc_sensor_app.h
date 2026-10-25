// Sensor-node application logic (hardware independent): tilt alarm, impact
// events, PIR edge counting and telemetry assembly.
//
// The firmware (firmware/sensor_node/main.cpp) owns the drivers: the IMU task,
// the shock ISR, the DHT samplers and the PIR pin. It passes their latest
// values in and forwards events and telemetry through SensorLinkCore. The
// host integration harness drives exactly the same class.
#pragma once

#include <stdint.h>

#include "vc_alarm.h"
#include "vc_protocol.h"
#include "vc_sensor_link.h"

namespace vc {

struct SensorAppConfig {
  float tiltOnDeg = 45.0f, tiltOffDeg = 35.0f;
  uint32_t tiltActivateMs = 1000, tiltClearMs = 2000;
  uint32_t imuMaxAgeMs = 200;  // older IMU samples are "stale" and never raise tilt
  // Optional hardware: a disabled sensor is reported "disabled", never alarms.
  bool dhtEnabled[2] = {true, true};
  bool imuEnabled = true;
  bool shockEnabled = true;
  bool pirEnabled = true;
};

// Latest IMU state as published by the IMU task.
struct ImuSample {
  proto::SensorStatus status = proto::SensorStatus::Absent;
  float accelXG = 0, accelYG = 0, accelZG = 0, accelMagG = 0;
  float peakAccelG = 0;  // since the previous telemetry packet
  float rollDeg = 0, pitchDeg = 0;
  float rateHz = 0;
  uint16_t maxGapMs = 0;
  uint32_t sampleMs = 0;  // millis() of the sample
};

struct ShockCounters {
  uint32_t impacts = 0, edges = 0, overflows = 0;
};

class SensorApp {
 public:
  void begin(const SensorAppConfig& cfg, SensorLinkCore* link);

  void onPirSample(bool level);
  // Evaluates the tilt alarm; queues an Active / Cleared event on a transition.
  AlarmTransition evaluateTilt(const ImuSample& s, uint32_t now);
  // An impact whose refractory window has closed; queues an Impact event.
  void onImpact(uint32_t detectedMs, uint16_t pulses, float peakAccelG);

  proto::TelemetryBody buildTelemetry(uint32_t now, const proto::DhtReading& a, const proto::DhtReading& b,
                                      const ImuSample& imu, const ShockCounters& shock) const;

  bool tiltActive() const { return tilt_.active(); }
  bool pirLevel() const { return pirLevel_; }
  uint32_t pirRises() const { return pirRises_; }
  uint32_t eventsCreated() const { return eventCounter_; }
  const SensorAppConfig& config() const { return cfg_; }

  // Age of a sample stamped by another task, which may be slightly newer than `now`.
  static uint32_t sampleAge(uint32_t now, uint32_t sampleMs) {
    int32_t d = (int32_t)(now - sampleMs);
    return d < 0 ? 0 : (uint32_t)d;
  }

 private:
  proto::EventBody newEvent(proto::EventType type, proto::EventState state, uint32_t detectedMs);

  SensorAppConfig cfg_;
  SensorLinkCore* link_ = nullptr;
  HysteresisAlarm tilt_{45.0f, 35.0f, 1000, 2000};
  uint32_t eventCounter_ = 0;
  bool pirLevel_ = false;
  uint32_t pirRises_ = 0;
};

}  // namespace vc
