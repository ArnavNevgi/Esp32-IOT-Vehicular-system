#include "vc_telemetry.h"

#include <ArduinoJson.h>

#include "vc_util.h"

namespace vc {
namespace telemetry {

namespace {

// Numbers are rounded to a fixed number of decimals and written as JSON
// numbers; non-finite values become null.
// Helpers take (parent, key): in ArduinoJson 7 a member that does not exist
// yet must be assigned through its parent.
void setNum(JsonObject parent, const char* key, float value, int decimals) {
  if (!isfinite(value)) {
    parent[key] = nullptr;
    return;
  }
  double scale = 1;
  for (int i = 0; i < decimals; ++i) scale *= 10;
  parent[key] = round((double)value * scale) / scale;
}

void setTs(JsonObject parent, const char* key, int64_t unixMs, bool valid) {
  char buf[32];
  if (valid && formatUtcIso8601(unixMs, buf, sizeof(buf))) parent[key] = (char*)buf;  // char* is copied
  else parent[key] = nullptr;
}

void setStats(JsonObject parent, const char* key, const WindowStats& s, int decimals, bool withMinAvg = true) {
  if (s.empty()) {
    parent[key] = nullptr;
    return;
  }
  JsonObject o = parent[key].to<JsonObject>();
  setNum(o, "last", s.last, decimals);
  if (withMinAvg) setNum(o, "min", s.minV, decimals);
  setNum(o, "max", s.maxV, decimals);
  if (withMinAvg) setNum(o, "avg", s.mean(), decimals);
}

void setDht(JsonObject o, const DhtWindow& d) {
  bool disabled = d.status == proto::SensorStatus::Disabled;
  o["status"] = d.seen || disabled ? statusName(d.status) : "no_data";
  if (d.seen && proto::statusHasValue(d.status)) o["ageMs"] = d.ageMs;
  else o["ageMs"] = nullptr;
  if (d.consecutiveErrors) o["errors"] = d.consecutiveErrors;
  o["samples"] = d.samples;
  setStats(o, "tempC", d.tempC, 1);
  setStats(o, "humidityPct", d.humidityPct, 1);
}

void writeEnvelope(JsonObject doc, const Envelope& env, const char* messageType) {
  doc["schemaVersion"] = kSchemaVersion;
  doc["messageType"] = messageType;
  doc["gatewayId"] = env.gatewayId;
  doc["gatewayBootId"] = env.gatewayBootId;
  doc["seq"] = env.seq;
  doc["uptimeMs"] = env.uptimeMs;
  setTs(doc, "ts", env.unixMs, env.timeValid);
  doc["timeValid"] = env.timeValid;
}

size_t finish(JsonDocument& doc, char* out, size_t cap) {
  if (doc.overflowed() || out == nullptr || cap == 0) return 0;
  size_t need = measureJson(doc);
  if (need + 1 > cap) return 0;
  size_t n = serializeJson(doc, out, cap);
  return n == need ? n : 0;
}

}  // namespace

const char* statusName(proto::SensorStatus s) {
  switch (s) {
    case proto::SensorStatus::Ok: return "ok";
    case proto::SensorStatus::Stale: return "stale";
    case proto::SensorStatus::Error: return "error";
    case proto::SensorStatus::Absent: return "absent";
    case proto::SensorStatus::Calibrating: return "calibrating";
    case proto::SensorStatus::Disabled: return "disabled";
  }
  return "unknown";
}

size_t serializeSummary(const Envelope& env, const Summary& s, char* out, size_t cap) {
  JsonDocument jd;
  JsonObject doc = jd.to<JsonObject>();
  writeEnvelope(doc, env, "summary");
  doc["windowMs"] = s.windowMs;

  JsonObject sn = doc["sensorNode"].to<JsonObject>();
  sn["deviceId"] = s.link.deviceId;
  sn["link"] = !s.link.configured ? "unconfigured" : (s.link.up ? "up" : (s.link.everSeen ? "lost" : "never_seen"));
  if (s.link.everSeen) sn["lastRxAgeMs"] = s.link.lastRxAgeMs;
  else sn["lastRxAgeMs"] = nullptr;
  const SensorNodeWindow* w = s.sensor;
  if (w && w->everSeen) {
    sn["bootId"] = w->bootId;
    if (w->synthetic) sn["synthetic"] = true;
    sn["packets"] = w->packets;
    JsonArray dht = sn["dht11"].to<JsonArray>();
    setDht(dht.add<JsonObject>(), w->dht[0]);
    setDht(dht.add<JsonObject>(), w->dht[1]);
    // A sensor disabled in the build reports status "disabled" and null values.
    bool imuOn = w->imuStatus != proto::SensorStatus::Disabled;
    JsonObject imu = sn["imu"].to<JsonObject>();
    imu["status"] = statusName(w->imuStatus);
    setStats(imu, "accelG", w->accelG, 2, false);
    if (!w->peakAccelG.empty()) setNum(imu, "peakAccelG", w->peakAccelG.maxV, 2);
    else imu["peakAccelG"] = nullptr;
    setStats(imu, "rollDeg", w->rollDeg, 1, true);
    setStats(imu, "pitchDeg", w->pitchDeg, 1, true);
    if (imuOn && !w->imuRateHz.empty()) setNum(imu, "rateHzMin", w->imuRateHz.minV, 1);
    else imu["rateHzMin"] = nullptr;
    if (imuOn) imu["maxGapMs"] = w->imuMaxGapMs;
    else imu["maxGapMs"] = nullptr;
    if (imuOn) sn["tiltActive"] = w->tiltActive;
    else sn["tiltActive"] = nullptr;
    bool pirOn = (w->features & proto::kFeaturePir) != 0;
    JsonObject pir = sn["pir"].to<JsonObject>();
    pir["status"] = pirOn ? "ok" : "disabled";
    if (pirOn) {
      pir["active"] = w->pirActive;
      pir["rises"] = w->pirRises;
      setNum(pir, "activeFraction", w->packets ? (float)w->pirActivePackets / (float)w->packets : NAN, 2);
    } else {
      pir["active"] = nullptr;
      pir["rises"] = nullptr;
      pir["activeFraction"] = nullptr;
    }
    bool shockOn = (w->features & proto::kFeatureShock) != 0;
    JsonObject shock = sn["shock"].to<JsonObject>();
    shock["status"] = shockOn ? "ok" : "disabled";
    if (shockOn) {
      shock["impacts"] = w->impacts;
      shock["edges"] = w->shockEdges;
    } else {
      shock["impacts"] = nullptr;
      shock["edges"] = nullptr;
    }
    if (w->impactOverflows || w->eventsDropped || w->eventsExpired || w->linkRestarts) {
      JsonObject lost = sn["lost"].to<JsonObject>();
      lost["impactOverflows"] = w->impactOverflows;
      lost["eventsDropped"] = w->eventsDropped;
      lost["eventsExpired"] = w->eventsExpired;
      lost["linkRestarts"] = w->linkRestarts;
    }
  }

  if (s.gateway) {
    JsonObject gw = doc["gateway"].to<JsonObject>();
    const GatewayWindow& g = *s.gateway;
    setDht(gw["dht22"].to<JsonObject>(), g.dht22);
    JsonObject flame = gw["flame"].to<JsonObject>();
    flame["status"] = g.flameEnabled ? "ok" : "disabled";
    setStats(flame, "pct", g.flamePct, 1, false);
    if (g.flameEnabled) flame["active"] = g.flameActive;
    else flame["active"] = nullptr;
    JsonObject pot = gw["pot"].to<JsonObject>();
    pot["status"] = g.potEnabled ? "ok" : "disabled";
    setStats(pot, "pct", g.potPct, 1, false);
  }

  const Health& h = s.health;
  JsonObject hj = doc["health"].to<JsonObject>();
  hj["wifi"] = h.wifi;
  if (h.rssi != 0) hj["rssi"] = h.rssi;
  hj["cloud"] = h.cloud;
  hj["mqttConnects"] = h.mqttConnects;
  hj["sasRenewals"] = h.sasRenewals;
  hj["queueDepth"] = h.queueDepth;
  hj["queueHighWater"] = h.queueHighWater;
  hj["alertsDropped"] = h.alertsDropped;
  hj["summariesCoalesced"] = h.summariesCoalesced;
  hj["publishFailures"] = h.publishFailures;
  JsonObject rx = hj["rx"].to<JsonObject>();
  rx["ok"] = h.rxAccepted;
  rx["rejected"] = h.rxRejected;
  rx["replay"] = h.rxReplay;
  rx["unknownPeer"] = h.rxUnknownPeer;
  rx["badTag"] = h.rxBadTag;
  rx["overflows"] = h.rxOverflows;
  hj["freeHeap"] = h.freeHeap;
  hj["minFreeHeap"] = h.minFreeHeap;
  hj["estMeterUnits"] = h.estMeterUnits;
  return finish(jd, out, cap);
}

size_t serializeAlert(const Envelope& env, const Alert& a, char* out, size_t cap) {
  JsonDocument jd;
  JsonObject doc = jd.to<JsonObject>();
  writeEnvelope(doc, env, "alert");
  doc["eventId"] = a.eventId;
  doc["alertType"] = a.alertType;
  doc["state"] = a.state;
  doc["reminder"] = a.reminder;
  JsonObject src = doc["source"].to<JsonObject>();
  src["role"] = a.sourceRole;
  src["deviceId"] = a.sourceDeviceId;
  src["bootId"] = a.sourceBootId;
  src["detectedUptimeMs"] = a.detectedUptimeMs;
  if (a.synthetic) src["synthetic"] = true;
  setTs(doc, "eventTs", a.eventUnixMs, a.eventTsValid);
  if (a.eventTsValid) doc["eventTsSource"] = a.eventTsSource;
  else doc["eventTsSource"] = nullptr;
  if (a.receivedTsValid || strcmp(a.sourceRole, "sensorNode") == 0) setTs(doc, "receivedTs", a.receivedUnixMs, a.receivedTsValid);

  JsonObject d = doc["details"].to<JsonObject>();
  if (isfinite(a.angleDeg)) setNum(d, "angleDeg", a.angleDeg, 1);
  if (isfinite(a.peakAccelG)) setNum(d, "peakAccelG", a.peakAccelG, 2);
  if (a.pulseCount >= 0) d["pulseCount"] = a.pulseCount;
  if (isfinite(a.flamePct)) setNum(d, "flamePct", a.flamePct, 1);
  if (a.silentMs >= 0) d["silentMs"] = a.silentMs;
  return finish(jd, out, cap);
}

}  // namespace telemetry
}  // namespace vc
