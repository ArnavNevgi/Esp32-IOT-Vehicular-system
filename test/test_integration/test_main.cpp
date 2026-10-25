// HOST INTEGRATION HARNESS scenarios: production sensor-side and gateway-side
// logic connected through simulated radio / storage / clock / MQTT (see
// harness.h for what is and is not exercised). Run with:
//   pio test -e native -f test_integration
// Set VC_HARNESS_VERBOSE=1 to print the cores' log lines.
#include <unity.h>

#include <set>

#include "harness.h"

using namespace sim;

void setUp() { logLines().clear(); }
void tearDown() {}

// ------------------------------------------------------------- helpers ----

struct AlertView {
  std::string eventId, alertType, state, mid, eventTs, ts, payload;
  uint32_t reminder = 0;
  uint32_t sourceBootId = 0;
};

static std::vector<AlertView> alerts(const World& w, const char* type = nullptr) {
  std::vector<AlertView> out;
  for (size_t i = 0; i < w.broker.hub.size(); ++i) {
    const HubMessage& m = w.broker.hub[i];
    if (m.messageType != "alert") continue;
    JsonDocument d;
    TEST_ASSERT_TRUE_MESSAGE(strictJson(m.payload, d), m.payload.c_str());
    AlertView a;
    a.eventId = d["eventId"].as<const char*>();
    a.alertType = d["alertType"].as<const char*>();
    a.state = d["state"].as<const char*>();
    a.reminder = d["reminder"].as<uint32_t>();
    a.sourceBootId = d["source"]["bootId"].as<uint32_t>();
    a.eventTs = d["eventTs"].isNull() ? "" : d["eventTs"].as<const char*>();
    a.ts = d["ts"].isNull() ? "" : d["ts"].as<const char*>();
    a.mid = m.messageId;
    a.payload = m.payload;
    if (!type || a.alertType == type) out.push_back(a);
  }
  return out;
}

static size_t uniqueMids(const World& w, const char* type) {
  std::set<std::string> ids;
  for (size_t i = 0; i < w.broker.hub.size(); ++i)
    if (w.broker.hub[i].messageType == type) ids.insert(w.broker.hub[i].messageId);
  return ids.size();
}

// "YYYY-MM-DDTHH:MM:SS.mmmZ" -> Unix ms (inverse of vc::formatUtcIso8601).
static int64_t parseIso(const std::string& s) {
  int y, mo, d, h, mi, se, ms;
  if (sscanf(s.c_str(), "%4d-%2d-%2dT%2d:%2d:%2d.%3dZ", &y, &mo, &d, &h, &mi, &se, &ms) != 7) return -1;
  y -= mo <= 2;
  int64_t era = (y >= 0 ? y : y - 399) / 400;
  unsigned yoe = (unsigned)(y - era * 400);
  unsigned doy = (153 * (mo + (mo > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  int64_t days = era * 146097 + (int64_t)doe - 719468;
  return ((days * 86400 + h * 3600 + mi * 60 + se) * 1000) + ms;
}

static size_t uniqueMidsOfAlertType(const World& w, const char* type) {
  std::set<std::string> ids;
  std::vector<AlertView> a = alerts(w, type);
  for (size_t i = 0; i < a.size(); ++i) ids.insert(a[i].mid);
  return ids.size();
}

static void assertNoSecretsLogged() {
  for (size_t i = 0; i < logLines().size(); ++i) {
    TEST_ASSERT_TRUE_MESSAGE(logLines()[i].find("SharedAccessSignature") == std::string::npos, logLines()[i].c_str());
    TEST_ASSERT_TRUE_MESSAGE(logLines()[i].find("sig=") == std::string::npos, logLines()[i].c_str());
  }
}

static void printExample(const char* label, const std::string& payload) {
  printf("[example] %s (%u bytes): %s\n", label, (unsigned)payload.size(), payload.c_str());
}

// ---------------------------------------------------------- scenarios ----

static void test_normal_telemetry_and_report_timing() {
  World w;
  w.runSeconds(305);
  TEST_ASSERT_TRUE(w.sensor->link.linked());
  TEST_ASSERT_EQUAL_UINT8(6, w.sensor->link.channel());  // found the AP's channel by sweeping
  std::vector<const HubMessage*> sums = w.broker.ofType("summary");
  TEST_ASSERT_EQUAL(5, (int)sums.size());  // at 60, 120, 180, 240, 300 s
  uint32_t lastUptime = 0, lastSeq = 0;
  for (size_t i = 0; i < sums.size(); ++i) {
    JsonDocument d;
    TEST_ASSERT_TRUE(strictJson(sums[i]->payload, d));
    TEST_ASSERT_EQUAL(2, d["schemaVersion"].as<int>());
    TEST_ASSERT_EQUAL_STRING("summary", d["messageType"]);
    TEST_ASSERT_TRUE(d["timeValid"].as<bool>());
    TEST_ASSERT_EQUAL_STRING("up", d["sensorNode"]["link"]);
    TEST_ASSERT_EQUAL_STRING("ok", d["sensorNode"]["dht11"][0]["status"]);
    uint32_t up = d["uptimeMs"].as<uint32_t>(), seq = d["seq"].as<uint32_t>();
    if (i > 0) {
      // Regression: the gateway's own DHT22 used to count only the first
      // reading after boot (sampler ages are always ~0), so its window
      // statistics became null while the status said "ok".
      TEST_ASSERT_UINT32_WITHIN(1, 20, d["gateway"]["dht22"]["samples"].as<uint32_t>());  // every 3 s
      TEST_ASSERT_FLOAT_WITHIN(0.01f, 21.0f, d["gateway"]["dht22"]["tempC"]["avg"].as<float>());
      TEST_ASSERT_UINT32_WITHIN(1, 15, d["sensorNode"]["dht11"][0]["samples"].as<uint32_t>());  // every 4 s
      TEST_ASSERT_FLOAT_WITHIN(0.01f, 37.5f, d["gateway"]["pot"]["pct"]["last"].as<float>());
      TEST_ASSERT_EQUAL_UINT32(60000, up - lastUptime);  // routine report period
      TEST_ASSERT_TRUE(seq > lastSeq);
      TEST_ASSERT_UINT32_WITHIN(2, 120, d["sensorNode"]["packets"].as<uint32_t>());  // 500 ms telemetry
    }
    char mid[64];
    snprintf(mid, sizeof(mid), "vehicle-gateway-01:1:s%lu", (unsigned long)seq);
    TEST_ASSERT_EQUAL_STRING(mid, sums[i]->messageId.c_str());
    lastUptime = up;
    lastSeq = seq;
  }
  printExample("summary from the integration harness", sums.back()->payload);
  TEST_ASSERT_EQUAL(0, (int)alerts(w).size());
  const GatewayLinkCore::Stats& ls = w.gateway->link.stats();
  TEST_ASSERT_UINT32_WITHIN(5, 609, ls.accepted);
  TEST_ASSERT_EQUAL_UINT32(0, ls.badTag + ls.replay + ls.malformed + ls.unknownPeer);
  TEST_ASSERT_EQUAL_UINT32(1, w.gateway->cloud.connects());
  assertNoSecretsLogged();
}

static void test_impact_event_end_to_end() {
  World w;
  w.runSeconds(5);
  uint32_t impactWorld = w.now;
  w.impact(3, 2.6f);
  w.runSeconds(5);
  std::vector<AlertView> a = alerts(w, "impact");
  TEST_ASSERT_EQUAL(1, (int)a.size());
  TEST_ASSERT_EQUAL_STRING("sensor-node-01:1:1", a[0].eventId.c_str());
  TEST_ASSERT_EQUAL_STRING("sensor-node-01:1:1", a[0].mid.c_str());
  TEST_ASSERT_EQUAL_STRING("detected", a[0].state.c_str());
  JsonDocument d;
  strictJson(a[0].payload, d);
  TEST_ASSERT_EQUAL(3, d["details"]["pulseCount"].as<int>());
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 2.6f, d["details"]["peakAccelG"].as<float>());
  TEST_ASSERT_EQUAL_STRING("gateway_estimate", d["eventTsSource"]);
  // The estimate corrects for the 200 ms refractory window and queueing.
  int64_t trueTs = kUnixBase + impactWorld;
  TEST_ASSERT_TRUE(llabs(parseIso(a[0].eventTs) - trueTs) <= 5);
  TEST_ASSERT_TRUE(parseIso(d["receivedTs"].as<const char*>()) >= parseIso(a[0].eventTs) + 200);
  TEST_ASSERT_EQUAL(0, (int)w.sensor->link.eventsPending());  // acknowledged
  TEST_ASSERT_TRUE(w.gateway->app.takeChirp());
  printExample("impact alert from the integration harness", a[0].payload);
}

static void test_tilt_activation_reminders_and_clear() {
  World w;
  w.runSeconds(10);
  w.setTilt(50.0f);
  w.runSeconds(25 * 60);
  w.setTilt(5.0f);
  w.runSeconds(5);
  std::vector<AlertView> a = alerts(w, "tilt");
  TEST_ASSERT_EQUAL(4, (int)a.size());
  TEST_ASSERT_EQUAL_STRING("active", a[0].state.c_str());
  TEST_ASSERT_EQUAL_UINT32(0, a[0].reminder);
  for (int r = 1; r <= 2; ++r) {
    TEST_ASSERT_EQUAL_STRING("active", a[r].state.c_str());
    TEST_ASSERT_EQUAL_UINT32((uint32_t)r, a[r].reminder);
    TEST_ASSERT_EQUAL_STRING(a[0].eventId.c_str(), a[r].eventId.c_str());   // same event
    TEST_ASSERT_EQUAL_STRING(a[0].eventTs.c_str(), a[r].eventTs.c_str());   // original detection time
    TEST_ASSERT_EQUAL_STRING((a[0].eventId + ":r" + std::to_string(r)).c_str(), a[r].mid.c_str());
    // Reminders 10 min apart (envelope ts = reminder creation time).
    TEST_ASSERT_INT64_WITHIN(1000, (int64_t)r * 600000, parseIso(a[r].ts) - parseIso(a[0].ts));
  }
  TEST_ASSERT_EQUAL_STRING("cleared", a[3].state.c_str());
  TEST_ASSERT_TRUE(a[3].eventId != a[0].eventId);
  TEST_ASSERT_FALSE(w.gateway->app.tiltActive());
  TEST_ASSERT_FALSE(w.sensor->app.tiltActive());
  printExample("tilt reminder from the integration harness", a[1].payload);
}

// Regression: the gateway used to forget an active tilt alarm when the
// telemetry packet just before the event had tiltActive=false (e.g. the packet
// carrying "true" was lost), silently stopping reminders.
static void test_tilt_alarm_survives_lost_telemetry() {
  WorldConfig c;
  c.gatewayApp.reminderMs = 60000;
  World w(c);
  w.runSeconds(10);
  w.toGateway.dropMsgType = (uint8_t)MsgType::Telemetry;  // only telemetry is lost
  w.setTilt(60.0f);
  w.runSeconds(3);
  TEST_ASSERT_TRUE(w.gateway->app.tiltActive());
  w.toGateway.dropMsgType = 0;
  w.runSeconds(70);
  TEST_ASSERT_TRUE(w.gateway->app.tiltActive());
  std::vector<AlertView> a = alerts(w, "tilt");
  TEST_ASSERT_EQUAL(2, (int)a.size());  // active + reminder 1
  TEST_ASSERT_EQUAL_UINT32(1, a[1].reminder);

  // Sensor reboots while level: its fresh telemetry clears the stale alarm.
  w.setTilt(5.0f);
  w.bootSensor();
  w.setTilt(5.0f);
  w.runSeconds(5);
  TEST_ASSERT_FALSE(w.gateway->app.tiltActive());
}

static void test_flame_hysteresis_reminders_and_clear() {
  WorldConfig c;
  c.gatewayApp.reminderMs = 120000;
  World w(c);
  w.runSeconds(10);
  w.flamePct = 50.0f;  // between thresholds: nothing
  w.runSeconds(5);
  TEST_ASSERT_EQUAL(0, (int)alerts(w, "flame").size());
  w.flamePct = 80.0f;
  w.runSeconds(250);
  w.flamePct = 45.0f;  // still above the 40 % clear threshold
  w.runSeconds(5);
  w.flamePct = 20.0f;
  w.runSeconds(5);
  std::vector<AlertView> a = alerts(w, "flame");
  TEST_ASSERT_EQUAL(4, (int)a.size());  // active, r1, r2, cleared
  TEST_ASSERT_EQUAL_STRING("active", a[0].state.c_str());
  TEST_ASSERT_EQUAL_UINT32(2, a[2].reminder);
  TEST_ASSERT_EQUAL_STRING("cleared", a[3].state.c_str());
  TEST_ASSERT_FALSE(w.gateway->app.flameActive());
}

static void test_disabled_flame_sensor_never_alarms() {
  WorldConfig c;
  c.gatewayApp.flameEnabled = false;  // not fitted: a floating ADC must not be monitored
  c.gatewayApp.potEnabled = false;
  World w(c);
  for (int i = 0; i < 1300; ++i) {  // the floating input would swing wildly
    w.flamePct = (i / 3) % 2 ? 100.0f : 0.0f;
    w.run(50);
  }
  TEST_ASSERT_EQUAL(0, (int)alerts(w, "flame").size());
  std::vector<const HubMessage*> sums = w.broker.ofType("summary");
  TEST_ASSERT_TRUE(sums.size() >= 1);
  JsonDocument d;
  TEST_ASSERT_TRUE(strictJson(sums.back()->payload, d));
  TEST_ASSERT_EQUAL_STRING("disabled", d["gateway"]["flame"]["status"]);
  TEST_ASSERT_TRUE(d["gateway"]["flame"]["pct"].isNull());
  TEST_ASSERT_TRUE(d["gateway"]["flame"]["active"].isNull());
  TEST_ASSERT_EQUAL_STRING("disabled", d["gateway"]["pot"]["status"]);
}

static void test_disabled_sensor_node_sensors() {
  WorldConfig c;
  c.sensorApp.dhtEnabled[1] = false;
  c.sensorApp.imuEnabled = false;
  c.sensorApp.pirEnabled = false;
  c.sensorApp.shockEnabled = false;
  World w(c);
  w.setTilt(80.0f);  // would be a tilt if the IMU were fitted
  w.impact();         // would be an impact if the shock sensor were fitted
  w.runSeconds(65);
  TEST_ASSERT_EQUAL(0, (int)alerts(w).size());
  std::vector<const HubMessage*> sums = w.broker.ofType("summary");
  TEST_ASSERT_EQUAL(1, (int)sums.size());
  JsonDocument d;
  TEST_ASSERT_TRUE(strictJson(sums[0]->payload, d));
  JsonObject sn = d["sensorNode"];
  TEST_ASSERT_EQUAL_STRING("ok", sn["dht11"][0]["status"]);
  TEST_ASSERT_EQUAL_STRING("disabled", sn["dht11"][1]["status"]);
  TEST_ASSERT_TRUE(sn["dht11"][1]["tempC"].isNull());
  TEST_ASSERT_EQUAL_STRING("disabled", sn["imu"]["status"]);
  TEST_ASSERT_TRUE(sn["tiltActive"].isNull());
  TEST_ASSERT_EQUAL_STRING("disabled", sn["pir"]["status"]);
  TEST_ASSERT_TRUE(sn["pir"]["active"].isNull());
  TEST_ASSERT_EQUAL_STRING("disabled", sn["shock"]["status"]);
  TEST_ASSERT_TRUE(sn["shock"]["impacts"].isNull());
  printExample("summary with sensors disabled", sums[0]->payload);

  // Shock sensor fitted but no IMU: impacts are reported without a peak value.
  WorldConfig c2;
  c2.sensorApp.imuEnabled = false;
  World w2(c2);
  w2.runSeconds(5);
  w2.impact(2, 3.3f);
  w2.runSeconds(3);
  std::vector<AlertView> a = alerts(w2, "impact");
  TEST_ASSERT_EQUAL(1, (int)a.size());
  JsonDocument ad;
  TEST_ASSERT_TRUE(strictJson(a[0].payload, ad));
  TEST_ASSERT_TRUE(ad["details"]["peakAccelG"].isNull());
  TEST_ASSERT_EQUAL(2, ad["details"]["pulseCount"].as<int>());
}

static void test_failed_sensor_readings_are_null_not_nan() {
  World w;
  w.sensor->dht[0].status = SensorStatus::Error;
  w.sensor->dht[0].consecutiveErrors = 7;
  w.sensor->dht[1].status = SensorStatus::Absent;
  w.sensor->imu.status = SensorStatus::Absent;
  w.sensor->imu.rollDeg = 90.0f;  // garbage behind an absent status must not raise tilt
  w.dht22.status = SensorStatus::Error;
  w.runSeconds(65);
  TEST_ASSERT_EQUAL(0, (int)alerts(w, "tilt").size());
  std::vector<const HubMessage*> sums = w.broker.ofType("summary");
  TEST_ASSERT_EQUAL(1, (int)sums.size());
  JsonDocument d;
  TEST_ASSERT_TRUE(strictJson(sums[0]->payload, d));
  TEST_ASSERT_EQUAL_STRING("error", d["sensorNode"]["dht11"][0]["status"]);
  TEST_ASSERT_EQUAL(7, d["sensorNode"]["dht11"][0]["errors"].as<int>());
  TEST_ASSERT_TRUE(d["sensorNode"]["dht11"][0]["tempC"].isNull());
  TEST_ASSERT_EQUAL_STRING("absent", d["sensorNode"]["imu"]["status"]);
  TEST_ASSERT_TRUE(d["sensorNode"]["imu"]["rollDeg"].isNull());
  TEST_ASSERT_EQUAL_STRING("error", d["gateway"]["dht22"]["status"]);
}

static void test_link_loss_alarm_and_recovery_with_queued_event() {
  World w;
  w.runSeconds(10);
  w.inRange = false;
  w.runSeconds(2);
  w.impact();  // happens during the outage: kept in the sensor's queue
  w.runSeconds(28);
  std::vector<AlertView> lost = alerts(w, "sensor_link_lost");
  TEST_ASSERT_EQUAL(1, (int)lost.size());
  TEST_ASSERT_EQUAL_STRING("active", lost[0].state.c_str());
  JsonDocument d;
  strictJson(lost[0].payload, d);
  TEST_ASSERT_TRUE(d["details"]["silentMs"].as<int>() >= 5000);
  TEST_ASSERT_FALSE(w.sensor->link.linked());
  TEST_ASSERT_TRUE(w.sensor->link.linkRestarts() >= 1);
  TEST_ASSERT_EQUAL(1, (int)w.sensor->link.eventsPending());
  w.inRange = true;
  w.runSeconds(45);  // worst case: the sensor is in a 30 s sweep backoff
  TEST_ASSERT_TRUE(w.sensor->link.linked());
  lost = alerts(w, "sensor_link_lost");
  TEST_ASSERT_EQUAL(2, (int)lost.size());
  TEST_ASSERT_EQUAL_STRING("cleared", lost[1].state.c_str());
  TEST_ASSERT_EQUAL(1, (int)alerts(w, "impact").size());  // delivered after recovery
  TEST_ASSERT_EQUAL_UINT32(0, w.sensor->link.eventsExpired());
}

static void test_channel_change_resweep() {
  World w;
  w.runSeconds(5);
  TEST_ASSERT_EQUAL_UINT8(6, w.sensor->link.channel());
  w.apChannel = 11;  // the AP moved; the gateway follows it
  w.runSeconds(40);
  TEST_ASSERT_TRUE(w.sensor->link.linked());
  TEST_ASSERT_EQUAL_UINT8(11, w.sensor->link.channel());
}

static void test_loss_duplication_reordering_events_exactly_once() {
  WorldConfig c;
  c.seed = 777;
  World w(c);
  w.runSeconds(5);
  AirPolicy p;
  p.lossPct = 20;
  p.dupPct = 15;
  p.minDelayMs = 1;
  p.maxDelayMs = 60;  // > 2 x telemetry spacing: frames arrive out of order
  w.toGateway = p;
  w.toSensor = p;
  for (int i = 0; i < 10; ++i) {
    w.impact(1 + i % 3, 2.0f + i * 0.1f);
    w.runSeconds(3);
  }
  // Lose every EventAck for a while: the 11th impact is retransmitted many
  // times and must still be processed exactly once.
  w.toSensor.dropMsgType = (uint8_t)MsgType::EventAck;
  w.impact();
  w.runSeconds(3);
  w.toSensor.dropMsgType = 0;
  w.runSeconds(30);
  std::vector<AlertView> a = alerts(w, "impact");
  TEST_ASSERT_EQUAL(11, (int)a.size());          // each impact exactly once in the cloud
  TEST_ASSERT_EQUAL(11, (int)uniqueMids(w, "alert"));
  for (int i = 0; i < 11; ++i)
    TEST_ASSERT_EQUAL_STRING(("sensor-node-01:1:" + std::to_string(i + 1)).c_str(), a[i].eventId.c_str());
  const GatewayLinkCore::Stats& ls = w.gateway->link.stats();
  TEST_ASSERT_TRUE(ls.replay > 0);           // radio duplicates dropped by the window
  TEST_ASSERT_TRUE(ls.duplicateEvents > 0);  // retransmissions acked, not reprocessed
  TEST_ASSERT_EQUAL_UINT32(0, w.sensor->link.eventsExpired());
  TEST_ASSERT_EQUAL_UINT32(0, w.sensor->link.eventsDropped());
  printf("[harness] lossy link: accepted=%lu replay=%lu duplicateEvents=%lu radioFailures=%lu restarts=%u\n",
         (unsigned long)ls.accepted, (unsigned long)ls.replay, (unsigned long)ls.duplicateEvents,
         (unsigned long)w.sensor->link.stats().radioSendFailures, (unsigned)w.sensor->link.linkRestarts());
}

static void test_malformed_unauthenticated_and_replayed_frames() {
  World w;
  w.runSeconds(20);
  w.impact();
  w.runSeconds(2);
  uint32_t session = w.gateway->link.session().session();
  uint32_t highest = 0;
  const std::vector<Frame> captured = w.capturedToGateway;
  const GatewayLinkCore::Stats before = w.gateway->link.stats();

  // 1) Random garbage claiming to be the sensor.
  Rng r(4242);
  for (int i = 0; i < 300; ++i) {
    std::vector<uint8_t> junk(r.random32() % 251);
    for (size_t k = 0; k < junk.size(); ++k) junk[k] = (uint8_t)r.random32();
    if (!junk.empty() && i % 2) junk[0] = kMagic;
    w.injectToGateway(kSensorMac, junk);
  }
  // 2) Valid frames retransmitted from a stranger's MAC.
  for (int i = 0; i < 20; ++i) w.injectToGateway(kStrangerMac, captured[captured.size() - 1 - i].data);
  // 3) A captured frame with one bit flipped, and frames made with another key.
  std::vector<uint8_t> flipped = captured.back().data;
  flipped[30] ^= 0x04;
  w.injectToGateway(kSensorMac, flipped);
  uint8_t otherLmk[16] = {9, 9, 9};
  AuthContext forged = makeAuthContext(deriveLinkKeys(otherLmk), Direction::SensorToGateway, kSensorMac, kGatewayMac);
  Packet ev;
  ev.header.type = MsgType::Event;
  ev.header.bootId = w.sensor->bootId;
  ev.header.sessionId = session;
  ev.header.seq = 0x7FFFFFFF;
  ev.event.eventCounter = 999;
  ev.event.attempt = 1;
  uint8_t buf[kMaxPacketSize];
  size_t n = encode(ev, forged, buf, sizeof(buf));
  w.injectToGateway(kSensorMac, std::vector<uint8_t>(buf, buf + n));
  w.run(20);
  // None of the above may move the session or its replay window.
  TEST_ASSERT_EQUAL_UINT32(session, w.gateway->link.session().session());
  const GatewayLinkCore::Stats mid = w.gateway->link.stats();
  TEST_ASSERT_EQUAL_UINT32(before.sessions, mid.sessions);
  TEST_ASSERT_TRUE(mid.badTag - before.badTag >= 2);           // bit flip + foreign key
  TEST_ASSERT_TRUE(mid.malformed - before.malformed >= 290);    // junk fails framing first
  TEST_ASSERT_EQUAL_UINT32(20, mid.unknownPeer - before.unknownPeer);

  // 4) Exact replays of already accepted data frames (telemetry and the
  // impact Event). Replayed Hellos are covered by the next scenario.
  int replayed = 0;
  for (size_t i = captured.size(); i-- > 0 && replayed < 30;) {
    uint8_t type = captured[i].data[2];
    if (type != (uint8_t)MsgType::Telemetry && type != (uint8_t)MsgType::Event) continue;
    w.injectToGateway(kSensorMac, captured[i].data);
    ++replayed;
  }
  (void)highest;
  w.run(20);
  TEST_ASSERT_EQUAL(30, replayed);
  TEST_ASSERT_TRUE(w.gateway->link.stats().replay - mid.replay >= 25);
  TEST_ASSERT_EQUAL_UINT32(session, w.gateway->link.session().session());

  // 5) Reflection: a gateway->sensor frame sent back to the gateway.
  Packet ack;
  ack.header.type = MsgType::EventAck;
  ack.header.bootId = w.sensor->bootId;
  ack.header.sessionId = session;
  ack.header.seq = 1;
  ack.ackEventCounter = 1;
  AuthContext g2s = makeAuthContext(deriveLinkKeys(kLmk), Direction::GatewayToSensor, kSensorMac, kGatewayMac);
  n = encode(ack, g2s, buf, sizeof(buf));
  uint32_t malformedBefore = w.gateway->link.stats().malformed;
  w.injectToGateway(kSensorMac, std::vector<uint8_t>(buf, buf + n));
  w.run(5);
  TEST_ASSERT_EQUAL_UINT32(malformedBefore + 1, w.gateway->link.stats().malformed);  // wrong direction

  w.runSeconds(10);
  TEST_ASSERT_EQUAL(1, (int)alerts(w, "impact").size());  // nothing extra reached the cloud
  TEST_ASSERT_TRUE(w.sensor->link.linked());
  TEST_ASSERT_EQUAL(0, (int)w.sensor->link.linkRestarts());
  TEST_ASSERT_EQUAL(0, (int)alerts(w, "sensor_link_lost").size());
}

static void test_delayed_and_replayed_session_establishment() {
  World w;
  w.runSeconds(10);
  std::vector<Frame> oldBoot = w.capturedToGateway;  // includes the first boot's Hellos
  w.bootSensor();  // boot 2
  w.runSeconds(5);
  TEST_ASSERT_TRUE(w.sensor->link.linked());
  TEST_ASSERT_EQUAL_UINT32(2, w.gatewaySensorBootStore.value);
  uint32_t session = w.gateway->link.session().session();
  uint32_t restarts = w.sensor->link.linkRestarts();

  // Delayed data from the previous boot / session: rejected; the resulting
  // SessionReject is addressed to the old boot and ignored by the sensor.
  for (size_t i = oldBoot.size() - 10; i < oldBoot.size(); ++i) w.injectToGateway(kSensorMac, oldBoot[i].data);
  // Replayed Hello from boot 1: refused, current session untouched.
  for (size_t i = 0; i < oldBoot.size(); ++i)
    if (oldBoot[i].data[2] == (uint8_t)MsgType::Hello) w.injectToGateway(kSensorMac, oldBoot[i].data);
  w.runSeconds(3);
  TEST_ASSERT_EQUAL_UINT32(session, w.gateway->link.session().session());
  TEST_ASSERT_TRUE(w.gateway->link.stats().helloRefused >= 1);
  TEST_ASSERT_TRUE(w.gateway->link.stats().sessionRejected >= 1);
  TEST_ASSERT_EQUAL(restarts, w.sensor->link.linkRestarts());
  TEST_ASSERT_TRUE(w.sensor->link.stats().rxWrongBoot >= 1);

  // Replayed Hello from the CURRENT boot: forces a new session (availability
  // only). The sensor's next frames are rejected, it re-handshakes, and no
  // data is ever accepted under the attacker-triggered session.
  std::vector<uint8_t> currentHello;
  for (size_t i = oldBoot.size(); i < w.capturedToGateway.size(); ++i)
    if (w.capturedToGateway[i].data[2] == (uint8_t)MsgType::Hello) currentHello = w.capturedToGateway[i].data;
  TEST_ASSERT_FALSE(currentHello.empty());
  w.injectToGateway(kSensorMac, currentHello);
  w.runSeconds(3);
  TEST_ASSERT_TRUE(w.sensor->link.linked());
  TEST_ASSERT_EQUAL(restarts + 1, w.sensor->link.linkRestarts());
  TEST_ASSERT_TRUE(w.gateway->link.session().session() != session);
  w.impact();
  w.runSeconds(3);
  TEST_ASSERT_EQUAL(1, (int)alerts(w, "impact").size());
}

static void test_sensor_and_gateway_reboots() {
  World w;
  w.runSeconds(10);
  w.impact();
  w.runSeconds(2);
  // Sensor-only reboot.
  w.bootSensor();
  w.runSeconds(5);
  TEST_ASSERT_TRUE(w.sensor->link.linked());
  TEST_ASSERT_EQUAL_UINT32(2, w.sensor->bootId);
  w.impact();
  w.runSeconds(2);
  std::vector<AlertView> a = alerts(w, "impact");
  TEST_ASSERT_EQUAL(2, (int)a.size());
  TEST_ASSERT_EQUAL_STRING("sensor-node-01:1:1", a[0].eventId.c_str());
  TEST_ASSERT_EQUAL_STRING("sensor-node-01:2:1", a[1].eventId.c_str());  // never reused

  // Gateway-only reboot while the sensor is linked: SessionReject, re-handshake.
  uint16_t restarts = w.sensor->link.linkRestarts();
  w.bootGateway();
  w.runSeconds(3);
  TEST_ASSERT_TRUE(w.sensor->link.linked());
  TEST_ASSERT_EQUAL(restarts + 1, w.sensor->link.linkRestarts());
  TEST_ASSERT_EQUAL_UINT32(2, w.gateway->bootId);

  // Gateway down for a while with an event happening meanwhile.
  w.powerOffGateway();
  w.runSeconds(4);
  w.impact();
  w.runSeconds(6);
  w.bootGateway();
  w.runSeconds(45);
  TEST_ASSERT_TRUE(w.sensor->link.linked());
  a = alerts(w, "impact");
  TEST_ASSERT_EQUAL(3, (int)a.size());
  TEST_ASSERT_EQUAL_STRING("sensor-node-01:2:2", a[2].eventId.c_str());

  // Simultaneous reboot.
  w.bootSensor();
  w.bootGateway();
  w.runSeconds(10);
  TEST_ASSERT_TRUE(w.sensor->link.linked());
  TEST_ASSERT_EQUAL_UINT32(3, w.sensor->bootId);
  TEST_ASSERT_EQUAL_UINT32(3, w.gatewaySensorBootStore.value);
  // Summary ids carry the gateway boot, so they never repeat across reboots.
  TEST_ASSERT_EQUAL((int)w.broker.ofType("summary").size(), (int)uniqueMids(w, "summary"));
}

static void test_sensor_nvs_erased_recovers_without_reusing_ids() {
  World w;
  w.runSeconds(10);
  w.impact();
  w.runSeconds(2);
  w.bootSensor();  // boot 2
  w.runSeconds(5);
  w.impact();
  w.runSeconds(2);
  // Erase the sensor's NVS: it starts again at boot 1 (older than granted),
  w.sensorBootStore.value = 0;
  w.bootSensor();
  w.runSeconds(5);
  TEST_ASSERT_TRUE(w.sensor->link.linked());
  TEST_ASSERT_EQUAL_UINT32(3, w.sensor->link.bootId());  // advanced past the granted boot 2
  TEST_ASSERT_EQUAL_UINT32(3, w.sensorBootStore.value);   // and persisted
  w.impact();
  w.runSeconds(2);
  // ... and once more, now landing exactly on the granted boot id (2 -> boot 3
  // was granted, so erase to 2: next boot is 3 == granted, different nonce).
  w.sensorBootStore.value = 2;
  w.bootSensor();
  TEST_ASSERT_EQUAL_UINT32(3, w.sensor->bootId);
  w.runSeconds(5);
  TEST_ASSERT_TRUE(w.sensor->link.linked());
  TEST_ASSERT_EQUAL_UINT32(4, w.sensor->link.bootId());
  w.impact();
  w.runSeconds(2);
  std::vector<AlertView> a = alerts(w, "impact");
  TEST_ASSERT_EQUAL(4, (int)a.size());
  TEST_ASSERT_EQUAL(4, (int)uniqueMids(w, "alert"));  // no event id was ever reused
  TEST_ASSERT_EQUAL_STRING("sensor-node-01:3:1", a[2].eventId.c_str());
  TEST_ASSERT_EQUAL_STRING("sensor-node-01:4:1", a[3].eventId.c_str());
}

static void test_cloud_disconnects_and_delayed_acks() {
  World w;
  w.runSeconds(10);
  w.broker.pubackDelayMs = 5000;  // slow PUBACKs
  for (int i = 0; i < 4; ++i) {
    w.impact();
    w.runSeconds(1);
  }
  w.broker.dropConnection();  // while messages are in flight
  w.runSeconds(3);
  w.broker.pubackDelayMs = 40;
  w.runSeconds(30);
  std::vector<AlertView> a = alerts(w, "impact");
  TEST_ASSERT_TRUE(a.size() >= 4);
  TEST_ASSERT_EQUAL(4, (int)uniqueMids(w, "alert"));  // re-sends keep $.mid
  // Each re-sent copy is byte-identical to the first one with that id.
  for (size_t i = 0; i < a.size(); ++i)
    for (size_t j = 0; j < i; ++j)
      if (a[i].mid == a[j].mid) TEST_ASSERT_EQUAL_STRING(a[j].payload.c_str(), a[i].payload.c_str());
  TEST_ASSERT_EQUAL_UINT32(2, w.gateway->cloud.connects());
  TEST_ASSERT_EQUAL(0, (int)w.gateway->app.outbox().depth());  // everything acknowledged
  // Reconnect waited for the transport backoff (>= 1.5 s with jitter).
  TEST_ASSERT_TRUE(w.broker.startTimes[1] - w.broker.startTimes[0] >= 1500);

  // PUBACKs that never arrive: the ack timeout re-sends, still with the same id.
  w.broker.withholdPubacks = true;
  w.impact();
  w.runSeconds(70);
  w.broker.withholdPubacks = false;
  w.runSeconds(65);
  std::vector<AlertView> b = alerts(w, "impact");
  TEST_ASSERT_EQUAL(5, (int)uniqueMids(w, "alert"));
  TEST_ASSERT_TRUE(b.size() >= a.size() + 2);  // delivered at least twice
  TEST_ASSERT_TRUE(w.gateway->app.outbox().depth() <= 1);
}

static void test_queue_saturation_during_long_outage() {
  World w;
  w.runSeconds(10);
  w.broker.reachable = false;  // IoT Hub unreachable for an hour
  for (int i = 0; i < 40; ++i) {
    w.impact();
    w.runSeconds(90);
  }
  const GatewayOutbox& q = w.gateway->app.outbox();
  TEST_ASSERT_TRUE(q.depth() <= GW_CLOUD_ALERT_SLOTS + 2);  // bounded
  TEST_ASSERT_EQUAL(GW_CLOUD_ALERT_SLOTS, (int)q.pendingAlerts());
  TEST_ASSERT_TRUE(q.stats().alertsDropped >= 40 - GW_CLOUD_ALERT_SLOTS - 1);
  TEST_ASSERT_TRUE(q.stats().summariesCoalesced >= 55);  // only the newest summary is kept
  TEST_ASSERT_EQUAL(0, (int)w.sensor->link.eventsPending());  // the gateway still acked every event
  // Reconnect attempts backed off instead of hammering the hub.
  TEST_ASSERT_TRUE(w.broker.starts < 60);
  size_t before = w.broker.hub.size();
  w.broker.reachable = true;
  w.runSeconds(180);
  std::vector<AlertView> a = alerts(w, "impact");
  TEST_ASSERT_EQUAL(GW_CLOUD_ALERT_SLOTS, (int)a.size());
  // The newest alerts survived (the oldest were dropped), in order.
  TEST_ASSERT_EQUAL_STRING("sensor-node-01:1:40", a.back().eventId.c_str());
  // Drain is paced: at least the configured spacing between publishes.
  for (size_t i = before + 1; i < w.broker.hub.size(); ++i)
    TEST_ASSERT_TRUE(w.broker.hub[i].receivedAt - w.broker.hub[i - 1].receivedAt >= 999);
  printf("[harness] outage: queued=%u highWater=%lu alertsDropped=%lu coalesced=%lu starts=%u\n", q.depth(),
         (unsigned long)w.gateway->app.stats().outboxHighWater, (unsigned long)q.stats().alertsDropped,
         (unsigned long)q.stats().summariesCoalesced, w.broker.starts);
}

static void test_clock_unavailable_then_synchronised() {
  World w;
  w.clockSynced = false;
  w.runSeconds(10);
  w.impact();
  w.runSeconds(170);
  TEST_ASSERT_EQUAL(0u, w.broker.starts);  // no TLS/SAS attempt without a plausible clock
  TEST_ASSERT_EQUAL_STRING("waiting_for_network", w.gateway->cloud.stateName());
  w.clockSynced = true;
  w.runSeconds(10);
  TEST_ASSERT_TRUE(w.gateway->cloud.connected());
  std::vector<AlertView> a = alerts(w, "impact");
  TEST_ASSERT_EQUAL(1, (int)a.size());
  JsonDocument d;
  strictJson(a[0].payload, d);
  TEST_ASSERT_FALSE(d["timeValid"].as<bool>());  // created before the clock was valid
  TEST_ASSERT_TRUE(d["ts"].isNull());
  TEST_ASSERT_TRUE(d["eventTs"].isNull());
  std::vector<const HubMessage*> sums = w.broker.ofType("summary");
  TEST_ASSERT_EQUAL(1, (int)sums.size());  // three were coalesced into the newest
  strictJson(sums[0]->payload, d);
  TEST_ASSERT_TRUE(d["ts"].isNull());
  w.runSeconds(60);
  sums = w.broker.ofType("summary");
  strictJson(sums.back()->payload, d);
  TEST_ASSERT_TRUE(d["timeValid"].as<bool>());
}

static void test_sas_renewal_while_events_are_queued() {
  WorldConfig c;
  c.cloud.sasLifetimeSec = 600;
  c.cloud.refreshMarginSec = 120;  // renew every 480 s
  World w(c);
  w.runSeconds(5);
  w.broker.pubackDelayMs = 3000;  // renewals will catch messages in flight
  for (int i = 0; i < 45; ++i) {
    w.impact();
    w.runSeconds(53);
  }
  w.broker.pubackDelayMs = 40;
  w.runSeconds(120);
  CloudLinkCore& cloud = w.gateway->cloud;
  TEST_ASSERT_TRUE(cloud.sasRenewals() >= 4);
  TEST_ASSERT_EQUAL_UINT32(0, cloud.failures());
  // Every connection used a newly generated token, never one shown twice.
  std::set<std::string> tokens(w.broker.passwords.begin(), w.broker.passwords.end());
  TEST_ASSERT_EQUAL(w.broker.passwords.size(), tokens.size());
  for (size_t i = 0; i < w.broker.passwords.size(); ++i)
    TEST_ASSERT_TRUE(w.broker.passwords[i].find("SharedAccessSignature") == 0);
  TEST_ASSERT_EQUAL(45, (int)uniqueMids(w, "alert"));  // nothing lost across renewals
  // Renewal spacing follows the monotonic schedule.
  for (size_t i = 1; i < w.broker.startTimes.size(); ++i)
    TEST_ASSERT_UINT32_WITHIN(1000, 480000, w.broker.startTimes[i] - w.broker.startTimes[i - 1]);
  assertNoSecretsLogged();
  printf("[harness] SAS renewals=%lu connects=%lu alerts=%u hubMessages=%u\n", (unsigned long)cloud.sasRenewals(),
         (unsigned long)cloud.connects(), (unsigned)uniqueMids(w, "alert"), (unsigned)w.broker.hub.size());
}

static void test_auth_refusal_backs_off_slowly() {
  World w;
  w.broker.refuseCode = kConnackNotAuthorized;
  w.runSeconds(3600);
  // 1 min doubling (with up to 25 % jitter): about 1 + log2 progression in an hour.
  TEST_ASSERT_TRUE(w.broker.starts >= 4 && w.broker.starts <= 7);
  for (size_t i = 1; i < w.broker.startTimes.size(); ++i)
    TEST_ASSERT_TRUE(w.broker.startTimes[i] - w.broker.startTimes[i - 1] >= 45000);
  TEST_ASSERT_EQUAL_UINT32(w.broker.starts, w.gateway->cloud.authFailures());
  TEST_ASSERT_TRUE(strstr(w.gateway->cloud.lastError(), "not authorized") != nullptr);
  w.broker.refuseCode = 0;  // key fixed
  w.runSeconds(1900);       // worst case: one full 30 min backoff
  TEST_ASSERT_TRUE(w.gateway->cloud.connected());
}

static void test_millis_wrap_on_both_devices() {
  WorldConfig c;
  c.sensorMillisAtBoot = 0xFFFFFFFFu - 40000u;  // wraps 40 s into the run
  c.gatewayMillisAtBoot = 0xFFFFFFFFu - 100000u;  // wraps 100 s into the run
  c.gatewayApp.reminderMs = 60000;
  World w(c);
  w.runSeconds(20);
  w.setTilt(50.0f);  // active across the sensor's wrap
  w.runSeconds(40);
  w.impact();
  w.runSeconds(245);
  TEST_ASSERT_TRUE(w.sensor->link.linked());
  TEST_ASSERT_EQUAL(0, (int)alerts(w, "sensor_link_lost").size());
  std::vector<const HubMessage*> sums = w.broker.ofType("summary");
  TEST_ASSERT_EQUAL(5, (int)sums.size());
  JsonDocument d0, d1;
  for (size_t i = 1; i < sums.size(); ++i) {
    strictJson(sums[i - 1]->payload, d0);
    strictJson(sums[i]->payload, d1);
    TEST_ASSERT_EQUAL_UINT32(60000, d1["uptimeMs"].as<uint32_t>() - d0["uptimeMs"].as<uint32_t>());
  }
  std::vector<AlertView> tilt = alerts(w, "tilt");
  TEST_ASSERT_TRUE(tilt.size() >= 4);  // active + reminders every 60 s across both wraps
  TEST_ASSERT_EQUAL(1, (int)alerts(w, "impact").size());
  JsonDocument ia;
  strictJson(alerts(w, "impact")[0].payload, ia);
  TEST_ASSERT_TRUE(ia["eventTs"].is<const char*>());  // estimate valid across the wrap
}

static void test_boot_grace_then_never_seen_alarm() {
  World w;
  w.powerOffSensor();
  w.runSeconds(29);
  TEST_ASSERT_EQUAL(0, (int)alerts(w, "sensor_link_lost").size());
  w.runSeconds(5);
  std::vector<AlertView> a = alerts(w, "sensor_link_lost");
  TEST_ASSERT_EQUAL(1, (int)a.size());
  JsonDocument d;
  strictJson(a[0].payload, d);
  TEST_ASSERT_TRUE(d["details"]["silentMs"].isNull());  // never seen since gateway boot
}

static void test_unconfigured_link_and_cloud() {
  WorldConfig c;
  c.linkConfigured = false;
  c.cloudConfigured = false;
  World w(c);
  w.runSeconds(130);
  TEST_ASSERT_EQUAL(0u, w.broker.starts);  // never attempts a login
  TEST_ASSERT_EQUAL(0, (int)w.capturedToGateway.size());  // never transmits
  TEST_ASSERT_FALSE(w.gateway->app.linkLost());  // no alarm for a link that is not configured
  TEST_ASSERT_EQUAL_UINT32(2, w.gateway->app.stats().summariesCreated);
}

// Accelerated 24 h run: bounded queues, retries, reminders and token refresh.
static void test_accelerated_24h_run() {
  WorldConfig c;
  c.seed = 2026;
  World w(c);
  AirPolicy p;
  p.lossPct = 5;
  p.dupPct = 2;
  p.maxDelayMs = 20;
  w.toGateway = p;
  w.toSensor = p;
  unsigned impacts = 0, maxOutbox = 0, maxSensorQueue = 0;
  for (uint32_t minute = 0; minute < 24 * 60; ++minute) {
    if (minute % 7 == 3) {
      w.impact();
      ++impacts;
    }
    if (minute % 180 == 100) w.setTilt(55.0f);
    if (minute % 180 == 140) w.setTilt(3.0f);
    if (minute % 240 == 200) w.broker.dropConnection();
    if (minute == 600) w.inRange = false;  // 2-minute radio outage
    if (minute == 602) w.inRange = true;
    w.runSeconds(60);
    if (w.gateway->app.outbox().depth() > maxOutbox) maxOutbox = w.gateway->app.outbox().depth();
    if (w.sensor->link.eventsPending() > maxSensorQueue) maxSensorQueue = w.sensor->link.eventsPending();
  }
  w.runSeconds(60);
  TEST_ASSERT_TRUE(maxOutbox <= GW_CLOUD_ALERT_SLOTS + 2);
  TEST_ASSERT_TRUE(maxSensorQueue <= SensorLinkCore::kEventSlots);
  TEST_ASSERT_INT_WITHIN(2, 24 * 60, (int)w.broker.ofType("summary").size());
  TEST_ASSERT_EQUAL((int)impacts, (int)uniqueMidsOfAlertType(w, "impact"));
  TEST_ASSERT_TRUE(w.gateway->cloud.sasRenewals() >= 20);  // 60 min tokens renewed every 55 min
  TEST_ASSERT_EQUAL_UINT32(0, w.sensor->link.eventsExpired());
  // Message sizes and metering from what the implementation actually produced.
  struct Size {
    size_t n = 0, minBody = 1u << 30, maxBody = 0, sumBody = 0, maxProps = 0;
    uint32_t free512 = 0, std4k = 0;
  } sz[2];
  for (size_t i = 0; i < w.broker.hub.size(); ++i) {
    const HubMessage& m = w.broker.hub[i];
    Size& s = sz[m.messageType == "summary" ? 0 : 1];
    size_t props = propertyBagLength(m.messageId, m.messageType);
    size_t body = m.payload.size();
    ++s.n;
    s.sumBody += body;
    if (body < s.minBody) s.minBody = body;
    if (body > s.maxBody) s.maxBody = body;
    if (props > s.maxProps) s.maxProps = props;
    s.free512 += meterUnits(body + props, 512);
    s.std4k += meterUnits(body + props, 4096);
  }
  const char* names[2] = {"summary", "alert"};
  for (int k = 0; k < 2; ++k)
    printf("[size] %-7s n=%u body min/avg/max=%u/%u/%u B, properties<=%u B, free-tier blocks=%lu, S1 blocks=%lu\n",
           names[k], (unsigned)sz[k].n, (unsigned)sz[k].minBody, (unsigned)(sz[k].sumBody / (sz[k].n ? sz[k].n : 1)),
           (unsigned)sz[k].maxBody, (unsigned)sz[k].maxProps, (unsigned long)sz[k].free512, (unsigned long)sz[k].std4k);
  printf("[quota] simulated day: %lu metered messages on the free tier (8,000/day), %lu on B1/S1 (400,000/day)\n",
         (unsigned long)(sz[0].free512 + sz[1].free512), (unsigned long)(sz[0].std4k + sz[1].std4k));
  TEST_ASSERT_TRUE(sz[0].maxBody + sz[0].maxProps < GW_CLOUD_PAYLOAD_CAP);
  printf("[harness] 24h: summaries=%u impacts=%u tilt alerts=%u link-lost alerts=%u sasRenewals=%lu "
         "mqttConnects=%lu maxOutbox=%u maxSensorQueue=%u estMeterUnits=%lu hubMessages=%u\n",
         (unsigned)w.broker.ofType("summary").size(), impacts, (unsigned)alerts(w, "tilt").size(),
         (unsigned)alerts(w, "sensor_link_lost").size(), (unsigned long)w.gateway->cloud.sasRenewals(),
         (unsigned long)w.gateway->cloud.connects(), maxOutbox, maxSensorQueue,
         (unsigned long)w.gateway->app.stats().estMeterUnits, (unsigned)w.broker.hub.size());
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_normal_telemetry_and_report_timing);
  RUN_TEST(test_impact_event_end_to_end);
  RUN_TEST(test_tilt_activation_reminders_and_clear);
  RUN_TEST(test_tilt_alarm_survives_lost_telemetry);
  RUN_TEST(test_flame_hysteresis_reminders_and_clear);
  RUN_TEST(test_disabled_flame_sensor_never_alarms);
  RUN_TEST(test_disabled_sensor_node_sensors);
  RUN_TEST(test_failed_sensor_readings_are_null_not_nan);
  RUN_TEST(test_link_loss_alarm_and_recovery_with_queued_event);
  RUN_TEST(test_channel_change_resweep);
  RUN_TEST(test_loss_duplication_reordering_events_exactly_once);
  RUN_TEST(test_malformed_unauthenticated_and_replayed_frames);
  RUN_TEST(test_delayed_and_replayed_session_establishment);
  RUN_TEST(test_sensor_and_gateway_reboots);
  RUN_TEST(test_sensor_nvs_erased_recovers_without_reusing_ids);
  RUN_TEST(test_cloud_disconnects_and_delayed_acks);
  RUN_TEST(test_queue_saturation_during_long_outage);
  RUN_TEST(test_clock_unavailable_then_synchronised);
  RUN_TEST(test_sas_renewal_while_events_are_queued);
  RUN_TEST(test_auth_refusal_backs_off_slowly);
  RUN_TEST(test_millis_wrap_on_both_devices);
  RUN_TEST(test_boot_grace_then_never_seen_alarm);
  RUN_TEST(test_unconfigured_link_and_cloud);
  RUN_TEST(test_accelerated_24h_run);
  return UNITY_END();
}
