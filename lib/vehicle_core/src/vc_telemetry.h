// Cloud telemetry contract (schemaVersion 2): JSON messages the gateway sends to
// Azure IoT Hub. See docs/TELEMETRY.md for the field reference and examples.
#pragma once

#include <math.h>
#include <stddef.h>
#include <stdint.h>

#include "vc_aggregate.h"
#include "vc_protocol.h"

namespace vc {
namespace telemetry {

const int kSchemaVersion = 2;

// Common fields of every cloud message.
struct Envelope {
  const char* gatewayId = "";
  uint32_t gatewayBootId = 0;
  uint32_t seq = 0;       // per gateway boot, +1 per created message
  uint32_t uptimeMs = 0;  // gateway millis() when the message was created
  int64_t unixMs = 0;     // gateway UTC clock when created (if timeValid)
  bool timeValid = false;
};

struct LinkInfo {
  const char* deviceId = "";
  bool configured = false;
  bool up = false;
  bool everSeen = false;
  uint32_t lastRxAgeMs = 0;
};

struct Health {
  const char* wifi = "";
  int rssi = 0;
  const char* cloud = "";
  uint32_t mqttConnects = 0;
  uint32_t sasRenewals = 0;
  uint32_t queueDepth = 0;
  uint32_t queueHighWater = 0;
  uint32_t alertsDropped = 0;
  uint32_t summariesCoalesced = 0;
  uint32_t publishFailures = 0;
  uint32_t rxAccepted = 0;
  uint32_t rxRejected = 0;  // malformed + replay + unknown session + unexpected peer
  uint32_t rxReplay = 0;
  uint32_t rxUnknownPeer = 0;
  uint32_t rxBadTag = 0;
  uint32_t rxOverflows = 0;  // radio frames dropped because the receive queue was full
  uint32_t freeHeap = 0;
  uint32_t minFreeHeap = 0;
  uint32_t estMeterUnits = 0;  // local estimate of metered messages since boot
};

struct Summary {
  uint32_t windowMs = 0;
  LinkInfo link;
  const SensorNodeWindow* sensor = nullptr;
  const GatewayWindow* gateway = nullptr;
  Health health;
};

struct Alert {
  const char* eventId = "";
  const char* alertType = "";  // impact | tilt | flame | sensor_link_lost
  const char* state = "";      // detected | active | cleared
  uint32_t reminder = 0;       // 0 = transition, n = n-th reminder of an active alarm
  const char* sourceRole = ""; // sensorNode | gateway
  const char* sourceDeviceId = "";
  uint32_t sourceBootId = 0;
  uint32_t detectedUptimeMs = 0;  // source clock (millis) at detection
  // Wall-clock event time, if known, and how it was obtained.
  int64_t eventUnixMs = 0;
  bool eventTsValid = false;
  const char* eventTsSource = "";  // gateway_clock | gateway_estimate
  int64_t receivedUnixMs = 0;      // gateway receipt time (sensor events only)
  bool receivedTsValid = false;
  // Details; NaN / negative values are omitted.
  float angleDeg = NAN;
  float peakAccelG = NAN;
  int32_t pulseCount = -1;
  float flamePct = NAN;
  int64_t silentMs = -1;
  bool synthetic = false;
};

const char* statusName(proto::SensorStatus s);

// Serialize into `out`. Returns the length (excluding the terminator), or 0 if
// the document did not fit; a truncated message is never returned.
size_t serializeSummary(const Envelope& env, const Summary& s, char* out, size_t cap);
size_t serializeAlert(const Envelope& env, const Alert& a, char* out, size_t cap);

}  // namespace telemetry
}  // namespace vc
