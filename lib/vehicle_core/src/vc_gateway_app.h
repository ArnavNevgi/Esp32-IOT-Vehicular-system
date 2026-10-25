// Gateway application logic (hardware independent): local alarms, reminders,
// cloud alerts and summaries, and the outbox service loop.
//
// The firmware (firmware/gateway/main.cpp) owns the hardware: it samples the
// DHT22 / flame / potentiometer and hands the values in, drives the LED and
// buzzer from alarmActive() / attention() / takeChirp(), and wires the
// GatewayLinkCore and CloudLinkCore to ESP-NOW and esp-mqtt. The host
// integration harness drives exactly the same class.
//
// Alerts (see docs/TELEMETRY.md):
//   impact            sensor event, one alert per accepted impact
//   tilt              sensor events active / cleared, reminders while active
//   flame             gateway flame index with hysteresis, reminders while active
//   sensor_link_lost  no accepted sensor packet for linkLossMs, reminders while lost
// Reminders repeat the activation record (same eventId, original detection
// time) with an increasing "reminder" number and messageId "<eventId>:r<n>".
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "vc_aggregate.h"
#include "vc_alarm.h"
#include "vc_cloud_link.h"
#include "vc_cloud_queue.h"
#include "vc_gateway_link.h"
#include "vc_ports.h"
#include "vc_telemetry.h"
#include "vc_timing.h"

#ifndef GW_CLOUD_ALERT_SLOTS
#define GW_CLOUD_ALERT_SLOTS 16
#endif
#ifndef GW_CLOUD_PAYLOAD_CAP
#define GW_CLOUD_PAYLOAD_CAP 2048
#endif

namespace vc {

struct GatewayAppConfig {
  const char* gatewayId = "unconfigured-gateway";
  const char* sensorNodeId = "sensor-node-01";
  uint32_t gatewayBootId = 1;
  // Optional hardware. A disabled sensor is never sampled, raises no alarm
  // and is reported with status "disabled".
  bool dht22Enabled = true;
  bool flameEnabled = true;
  bool potEnabled = true;
  float flameOnPct = 60.0f, flameOffPct = 40.0f;
  uint32_t flameActivateMs = 300, flameClearMs = 3000;
  uint32_t linkLossMs = 5000, linkRestoreMs = 2000, linkBootGraceMs = 30000;
  uint32_t reminderMs = 600000;
  uint32_t summaryMs = 60000;
  uint32_t cloudMinSpacingMs = 1000;
  unsigned cloudMaxInFlight = 2;
  uint32_t cloudAckTimeoutMs = 60000;
  uint32_t meterBlockBytes = 512;
  uint32_t tiltSyncFreshMs = 2000;
};

// Status the gateway reports about its own platform in each summary.
class PlatformStatus {
 public:
  virtual ~PlatformStatus() {}
  virtual const char* wifiState() const = 0;
  virtual int rssi() const = 0;
  virtual uint32_t freeHeap() const = 0;
  virtual uint32_t minFreeHeap() const = 0;
  virtual uint32_t radioRxOverflows() const = 0;
};

typedef CloudOutbox<GW_CLOUD_ALERT_SLOTS, GW_CLOUD_PAYLOAD_CAP> GatewayOutbox;

class GatewayApp : public SensorEventSink {
 public:
  struct Stats {
    uint32_t alertsCreated = 0;
    uint32_t alertsRejected = 0;  // did not serialize or could not be queued
    uint32_t summariesCreated = 0;
    uint32_t summariesTooLarge = 0;
    uint32_t publishAttempts = 0;
    uint32_t publishFailures = 0;  // client refused the message
    uint32_t publishedOk = 0;      // PUBACK received
    uint32_t estMeterUnits = 0;    // local estimate of metered messages
    uint32_t outboxHighWater = 0;
  };

  // `link` may be unconfigured (no ESP-NOW keys); the app then raises no
  // link-loss alarm and reports the link as "unconfigured".
  void begin(const GatewayAppConfig& cfg, WallClock* clock, CloudLinkCore* cloud, GatewayLinkCore* link,
             PlatformStatus* platform, uint32_t now);

  // Inputs from the gateway's own sensors (call only for enabled sensors).
  // newMeasurement: this reading comes from a read that just succeeded.
  void onDht22Reading(const proto::DhtReading& r, bool newMeasurement);
  void onFlameSample(float pct, uint32_t now);
  void onPotSample(float pct);

  // SensorEventSink: a new impact / tilt event from the sensor node.
  bool onSensorEvent(const proto::Header& h, const proto::EventBody& e, uint32_t rxMs) override;

  // Alarm checks, reminders, summary timer and outbox service.
  void tick(uint32_t now);

  bool flameActive() const { return flame_.active(); }
  bool tiltActive() const { return tiltAlarm_.active; }
  bool linkLost() const { return linkLost_.active(); }
  bool alarmActive() const { return flame_.active() || tiltAlarm_.active; }
  bool attention() const;
  bool takeChirp() {
    bool c = chirp_;
    chirp_ = false;
    return c;
  }
  float lastFlamePct() const { return lastFlamePct_; }
  const GatewayOutbox& outbox() const { return outbox_; }
  const Stats& stats() const { return stats_; }
  const GatewayWindow& gatewayWindow() const { return window_; }
  uint32_t cloudSeq() const { return cloudSeq_; }

 private:
  // Activation record of an alarm that is still active. Reminders repeat it.
  struct ActiveAlarm {
    bool active = false;
    char eventId[64] = "";
    char sourceDeviceId[48] = "";
    const char* alertType = "";
    const char* sourceRole = "";
    const char* eventTsSource = "";
    uint32_t sourceBootId = 0;
    uint32_t detectedUptimeMs = 0;
    int64_t eventUnixMs = 0;
    bool eventTsValid = false;
    int64_t receivedUnixMs = 0;
    bool receivedTsValid = false;
    bool synthetic = false;
    ReminderTimer timer{600000};
    void remember(const telemetry::Alert& a, uint32_t now);
    void forget();
    bool reminderDue(uint32_t now, telemetry::Alert& out);
  };

  telemetry::Envelope makeEnvelope(uint32_t now);
  bool queueAlert(const telemetry::Alert& a, uint32_t now);
  void newGatewayEventId(char* out, size_t cap);
  telemetry::Alert gatewayAlert(const char* type, const char* state, const char* eventId, uint32_t now);
  void checkSensorLink(uint32_t now);
  void syncTiltState(uint32_t now);
  void sendReminders(uint32_t now);
  void queueSummary(uint32_t now);
  void serviceCloud(uint32_t now);

  GatewayAppConfig cfg_;
  WallClock* clock_ = nullptr;
  CloudLinkCore* cloud_ = nullptr;
  GatewayLinkCore* link_ = nullptr;
  PlatformStatus* platform_ = nullptr;
  uint32_t bootMs_ = 0;
  bool graceOver_ = false;
  uint32_t cloudSeq_ = 0;
  uint32_t gatewayEventCounter_ = 0;
  GatewayWindow window_;
  GatewayOutbox outbox_;
  HysteresisAlarm flame_{60.0f, 40.0f, 300, 3000};
  DebouncedAlarm linkLost_{0, 2000};
  float lastFlamePct_ = NAN;
  ActiveAlarm flameAlarm_, linkAlarm_, tiltAlarm_;
  PeriodicTimer summaryTimer_;
  uint32_t lastPublishMs_ = 0;
  bool published_ = false;
  bool chirp_ = false;
  Stats stats_;
  char json_[GW_CLOUD_PAYLOAD_CAP];
};

}  // namespace vc
