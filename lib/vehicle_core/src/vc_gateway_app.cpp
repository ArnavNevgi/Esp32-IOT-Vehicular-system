#include "vc_gateway_app.h"

#include <stdio.h>
#include <string.h>

#include "vc_log.h"

namespace vc {

using namespace proto;

void GatewayApp::ActiveAlarm::remember(const telemetry::Alert& a, uint32_t now) {
  active = true;
  snprintf(eventId, sizeof(eventId), "%s", a.eventId);
  snprintf(sourceDeviceId, sizeof(sourceDeviceId), "%s", a.sourceDeviceId);
  alertType = a.alertType;
  sourceRole = a.sourceRole;
  eventTsSource = a.eventTsSource;
  sourceBootId = a.sourceBootId;
  detectedUptimeMs = a.detectedUptimeMs;
  eventUnixMs = a.eventUnixMs;
  eventTsValid = a.eventTsValid;
  receivedUnixMs = a.receivedUnixMs;
  receivedTsValid = a.receivedTsValid;
  synthetic = a.synthetic;
  timer.arm(now);
}

void GatewayApp::ActiveAlarm::forget() {
  active = false;
  timer.disarm();
}

bool GatewayApp::ActiveAlarm::reminderDue(uint32_t now, telemetry::Alert& out) {
  if (!active || !timer.due(now)) return false;
  out.eventId = eventId;
  out.alertType = alertType;
  out.state = "active";
  out.reminder = timer.count();
  out.sourceRole = sourceRole;
  out.sourceDeviceId = sourceDeviceId;
  out.sourceBootId = sourceBootId;
  out.detectedUptimeMs = detectedUptimeMs;
  out.eventUnixMs = eventUnixMs;
  out.eventTsValid = eventTsValid;
  out.eventTsSource = eventTsSource;
  out.receivedUnixMs = receivedUnixMs;
  out.receivedTsValid = receivedTsValid;
  out.synthetic = synthetic;
  return true;
}

void GatewayApp::begin(const GatewayAppConfig& cfg, WallClock* clock, CloudLinkCore* cloud, GatewayLinkCore* link,
                       PlatformStatus* platform, uint32_t now) {
  cfg_ = cfg;
  clock_ = clock;
  cloud_ = cloud;
  link_ = link;
  platform_ = platform;
  bootMs_ = now;
  flame_ = HysteresisAlarm(cfg.flameOnPct, cfg.flameOffPct, cfg.flameActivateMs, cfg.flameClearMs);
  linkLost_ = DebouncedAlarm(0, cfg.linkRestoreMs);
  flameAlarm_.timer.setInterval(cfg.reminderMs);
  linkAlarm_.timer.setInterval(cfg.reminderMs);
  tiltAlarm_.timer.setInterval(cfg.reminderMs);
  window_.flameEnabled = cfg.flameEnabled;
  window_.potEnabled = cfg.potEnabled;
  if (!cfg.dht22Enabled) window_.dht22.status = SensorStatus::Disabled;
  summaryTimer_.setPeriod(cfg.summaryMs);
  summaryTimer_.start(now);
}

bool GatewayApp::attention() const { return linkLost_.active() || !(cloud_ && cloud_->connected()); }

telemetry::Envelope GatewayApp::makeEnvelope(uint32_t now) {
  telemetry::Envelope env;
  env.gatewayId = cfg_.gatewayId;
  env.gatewayBootId = cfg_.gatewayBootId;
  env.seq = ++cloudSeq_;
  env.uptimeMs = now;
  env.timeValid = clock_ && clock_->valid();
  env.unixMs = env.timeValid ? clock_->unixMs() : 0;
  return env;
}

// Queues an alert. messageId = eventId, plus ":r<n>" for reminders, so a
// retransmission keeps its id while each reminder is a distinct message.
bool GatewayApp::queueAlert(const telemetry::Alert& a, uint32_t now) {
  ++stats_.alertsCreated;
  size_t n = telemetry::serializeAlert(makeEnvelope(now), a, json_, sizeof(json_));
  if (n == 0) {
    ++stats_.alertsRejected;
    logf("[cloud] alert did not fit the buffer; not sent");
    return false;
  }
  char mid[64];
  int m = a.reminder ? snprintf(mid, sizeof(mid), "%s:r%lu", a.eventId, (unsigned long)a.reminder)
                     : snprintf(mid, sizeof(mid), "%s", a.eventId);
  bool ok = m > 0 && (size_t)m < sizeof(mid) && outbox_.pushAlert(mid, json_, n, a.reminder != 0);
  if (!ok) ++stats_.alertsRejected;
  if (outbox_.depth() > stats_.outboxHighWater) stats_.outboxHighWater = outbox_.depth();
  logf("[alert] %s %s%s (%s) -> %s", a.alertType, a.state, a.reminder ? " reminder" : "", a.eventId,
       ok ? "queued" : "REJECTED");
  return ok;
}

void GatewayApp::newGatewayEventId(char* out, size_t cap) {
  snprintf(out, cap, "%s:%lu:%lu", cfg_.gatewayId, (unsigned long)cfg_.gatewayBootId,
           (unsigned long)++gatewayEventCounter_);
}

telemetry::Alert GatewayApp::gatewayAlert(const char* type, const char* state, const char* eventId, uint32_t now) {
  telemetry::Alert a;
  a.eventId = eventId;
  a.alertType = type;
  a.state = state;
  a.sourceRole = "gateway";
  a.sourceDeviceId = cfg_.gatewayId;
  a.sourceBootId = cfg_.gatewayBootId;
  a.detectedUptimeMs = now;
  a.eventTsValid = clock_ && clock_->valid();
  a.eventUnixMs = a.eventTsValid ? clock_->unixMs() : 0;
  a.eventTsSource = "gateway_clock";
  return a;
}

bool GatewayApp::onSensorEvent(const Header& h, const EventBody& e, uint32_t rxMs) {
  // Called synchronously by GatewayLinkCore::onFrame, so rxMs is "now".
  char eventId[64];
  snprintf(eventId, sizeof(eventId), "%s:%lu:%lu", cfg_.sensorNodeId, (unsigned long)h.bootId,
           (unsigned long)e.eventCounter);
  telemetry::Alert a;
  a.eventId = eventId;
  a.sourceRole = "sensorNode";
  a.sourceDeviceId = cfg_.sensorNodeId;
  a.sourceBootId = h.bootId;
  a.detectedUptimeMs = e.detectedUptimeMs;
  a.synthetic = (h.flags & kFlagSynthetic) != 0;
  a.receivedTsValid = clock_ && clock_->valid();
  a.receivedUnixMs = a.receivedTsValid ? clock_->unixMs() : 0;
  // The sensor has no wall clock: estimate the event time from the sensor-side
  // delay between detection and this transmission. Labelled as an estimate.
  uint32_t delay = h.uptimeMs - e.detectedUptimeMs;
  if (a.receivedTsValid && (int32_t)delay >= 0) {
    a.eventTsValid = true;
    a.eventUnixMs = a.receivedUnixMs - delay;
    a.eventTsSource = "gateway_estimate";
  }
  if (e.type == EventType::Impact) {
    a.alertType = "impact";
    a.state = "detected";
    // Without a fitted IMU there is no peak measurement: omit it, never "0 g".
    bool imuFitted = !link_ || link_->aggregator().window().imuStatus != SensorStatus::Disabled;
    a.peakAccelG = imuFitted ? e.peakAccelG : NAN;
    a.pulseCount = e.pulseCount;
  } else {
    a.alertType = "tilt";
    a.state = e.state == EventState::Active ? "active" : "cleared";
    a.angleDeg = e.angleDeg;
  }
  bool queued = queueAlert(a, rxMs);
  if (!queued) return false;
  if (e.type == EventType::Impact) chirp_ = true;
  if (e.type == EventType::Tilt) {
    if (e.state == EventState::Active) tiltAlarm_.remember(a, rxMs);
    else tiltAlarm_.forget();
  }
  return true;
}

void GatewayApp::onDht22Reading(const DhtReading& r, bool newMeasurement) {
  if (cfg_.dht22Enabled) addDhtSample(window_.dht22, r, newMeasurement);
}

void GatewayApp::onPotSample(float pct) {
  if (cfg_.potEnabled) window_.potPct.add(pct);
}

void GatewayApp::onFlameSample(float pct, uint32_t now) {
  if (!cfg_.flameEnabled) return;  // never evaluate an input that is not wired
  lastFlamePct_ = pct;
  window_.flamePct.add(pct);
  AlarmTransition t = flame_.update(pct, true, now);
  if (t == AlarmTransition::Activated) {
    char id[64];
    newGatewayEventId(id, sizeof(id));
    telemetry::Alert a = gatewayAlert("flame", "active", id, now);
    a.flamePct = pct;
    queueAlert(a, now);
    flameAlarm_.remember(a, now);
  } else if (t == AlarmTransition::Cleared) {
    char id[64];
    newGatewayEventId(id, sizeof(id));
    telemetry::Alert a = gatewayAlert("flame", "cleared", id, now);
    a.flamePct = flame_.peak();
    queueAlert(a, now);
    flameAlarm_.forget();
  }
  window_.flameActive = flame_.active();
}

void GatewayApp::checkSensorLink(uint32_t now) {
  if (!link_ || !link_->configured()) return;
  // Latched, so neither the millis() wrap nor a late begin() can reset it.
  if (!graceOver_ && hasElapsed(now, bootMs_, cfg_.linkBootGraceMs)) graceOver_ = true;
  uint32_t silence = elapsedMs(now, link_->lastRxMs());
  bool condition = link_->everSeen() ? silence >= cfg_.linkLossMs : graceOver_;
  AlarmTransition t = linkLost_.update(condition, now);
  if (t == AlarmTransition::Activated) {
    char id[64];
    newGatewayEventId(id, sizeof(id));
    telemetry::Alert a = gatewayAlert("sensor_link_lost", "active", id, now);
    a.sourceDeviceId = cfg_.sensorNodeId;  // the device that went silent
    a.silentMs = link_->everSeen() ? (int64_t)silence : -1;
    queueAlert(a, now);
    linkAlarm_.remember(a, now);
  } else if (t == AlarmTransition::Cleared) {
    char id[64];
    newGatewayEventId(id, sizeof(id));
    telemetry::Alert a = gatewayAlert("sensor_link_lost", "cleared", id, now);
    a.sourceDeviceId = cfg_.sensorNodeId;
    queueAlert(a, now);
    linkAlarm_.forget();
  }
}

// The sensor's live telemetry is authoritative for whether tilt is still
// active (e.g. it rebooted while tilted and never sent a "cleared" event).
// Only telemetry the sensor sent AFTER the activation counts: an older packet
// delivered late, or the packet just before the event, must not cancel it.
void GatewayApp::syncTiltState(uint32_t now) {
  if (!tiltAlarm_.active || !link_ || !link_->everSeen() || hasElapsed(now, link_->lastRxMs(), cfg_.tiltSyncFreshMs))
    return;
  const SensorNodeWindow& w = link_->aggregator().window();
  if (w.tiltActive) return;
  bool rebooted = w.bootId != tiltAlarm_.sourceBootId;
  bool newer = (int32_t)(w.lastPacketUptimeMs - tiltAlarm_.detectedUptimeMs) > 0;
  if (rebooted || newer) tiltAlarm_.forget();
}

// Controlled reminders for alarms that stay active (default every 10 min).
// The envelope's "ts" is the reminder time; eventTs stays the detection time.
void GatewayApp::sendReminders(uint32_t now) {
  telemetry::Alert a;
  if (flameAlarm_.reminderDue(now, a)) {
    a.flamePct = lastFlamePct_;
    queueAlert(a, now);
  }
  a = telemetry::Alert();
  if (linkAlarm_.reminderDue(now, a)) {
    a.silentMs = link_ && link_->everSeen() ? (int64_t)elapsedMs(now, link_->lastRxMs()) : -1;
    queueAlert(a, now);
  }
  a = telemetry::Alert();
  if (tiltAlarm_.reminderDue(now, a)) queueAlert(a, now);
}

void GatewayApp::queueSummary(uint32_t now) {
  const CloudQueueStats& qs = outbox_.stats();
  telemetry::Summary s;
  s.windowMs = cfg_.summaryMs;
  s.link.deviceId = cfg_.sensorNodeId;
  bool linkOn = link_ && link_->configured();
  s.link.configured = linkOn;
  s.link.everSeen = linkOn && link_->everSeen();
  s.link.up = s.link.everSeen && !linkLost_.active();
  s.link.lastRxAgeMs = linkOn ? elapsedMs(now, link_->lastRxMs()) : 0;
  s.sensor = linkOn ? &link_->aggregator().window() : nullptr;
  s.gateway = &window_;
  s.health.wifi = platform_ ? platform_->wifiState() : "unknown";
  s.health.rssi = platform_ ? platform_->rssi() : 0;
  s.health.cloud = cloud_ ? cloud_->stateName() : "unconfigured";
  s.health.mqttConnects = cloud_ ? cloud_->connects() : 0;
  s.health.sasRenewals = cloud_ ? cloud_->sasRenewals() : 0;
  s.health.queueDepth = outbox_.depth();
  s.health.queueHighWater = stats_.outboxHighWater;
  s.health.alertsDropped = qs.alertsDropped + qs.remindersDropped + qs.rejected;
  s.health.summariesCoalesced = qs.summariesCoalesced;
  s.health.publishFailures = stats_.publishFailures;
  if (linkOn) {
    const GatewayLinkCore::Stats& ls = link_->stats();
    s.health.rxAccepted = ls.accepted;
    s.health.rxRejected = link_->rejectedTotal();
    s.health.rxReplay = ls.replay;
    s.health.rxUnknownPeer = ls.unknownPeer;
    s.health.rxBadTag = ls.badTag;
  }
  s.health.rxOverflows = platform_ ? platform_->radioRxOverflows() : 0;
  s.health.freeHeap = platform_ ? platform_->freeHeap() : 0;
  s.health.minFreeHeap = platform_ ? platform_->minFreeHeap() : 0;
  s.health.estMeterUnits = stats_.estMeterUnits;
  telemetry::Envelope env = makeEnvelope(now);
  ++stats_.summariesCreated;
  size_t n = telemetry::serializeSummary(env, s, json_, sizeof(json_));
  if (n == 0) {
    ++stats_.summariesTooLarge;
    logf("[cloud] summary did not fit the buffer; not sent");
  } else {
    char mid[64];
    snprintf(mid, sizeof(mid), "%s:%lu:s%lu", cfg_.gatewayId, (unsigned long)cfg_.gatewayBootId,
             (unsigned long)env.seq);
    outbox_.pushSummary(mid, json_, n);
    if (outbox_.depth() > stats_.outboxHighWater) stats_.outboxHighWater = outbox_.depth();
  }
  if (linkOn) link_->aggregator().resetWindow();
  window_.reset();
}

void GatewayApp::serviceCloud(uint32_t now) {
  if (!cloud_) return;
  CloudLinkCore::Event ev;
  while (cloud_->pollEvent(ev)) {
    if (ev.kind == CloudLinkCore::Event::Kind::Published) {
      if (outbox_.onPublished(ev.msgId)) ++stats_.publishedOk;
    } else if (ev.kind == CloudLinkCore::Event::Kind::Deleted) {
      outbox_.onDeleted(ev.msgId);
    } else {
      outbox_.onDisconnected();  // unacknowledged messages will be re-sent
    }
  }
  outbox_.requeueTimedOut(now, cfg_.cloudAckTimeoutMs);
  if (!cloud_->connected() || (published_ && !hasElapsed(now, lastPublishMs_, cfg_.cloudMinSpacingMs))) return;
  GatewayOutbox::Message* m = outbox_.nextToSend(cfg_.cloudMaxInFlight);
  if (!m) return;
  lastPublishMs_ = now;
  published_ = true;
  ++stats_.publishAttempts;
  size_t wire = 0;
  int id = cloud_->publish(m->messageId, m->kind == CloudKind::Alert ? "alert" : "summary", m->payload, m->len, &wire);
  if (id > 0) {
    outbox_.markInFlight(m, id, now);
    stats_.estMeterUnits += meterUnits(wire, cfg_.meterBlockBytes);
  } else {
    // -1 failure, -2 client outbox full, 0 unexpected for QoS 1.
    outbox_.markSendFailed(m);
    ++stats_.publishFailures;
  }
}

void GatewayApp::tick(uint32_t now) {
  checkSensorLink(now);
  syncTiltState(now);
  sendReminders(now);
  if (summaryTimer_.due(now)) queueSummary(now);
  serviceCloud(now);
}

}  // namespace vc
