// HOST INTEGRATION HARNESS (not hardware): connects the production sensor-side
// logic (vc::SensorApp + vc::SensorLinkCore) to the production gateway-side
// logic (vc::GatewayLinkCore + vc::GatewayApp + vc::CloudLinkCore) through a
// simulated radio, simulated persistent storage, simulated clocks and a
// scripted MQTT broker / IoT Hub. Packet encoding, authentication, replay
// handling, event tracking, queues, alarms and JSON generation are the real
// library code; only the platform ports below are simulated.
//
// What this does NOT exercise: the ESP32 radio and ESP-NOW driver, CCMP, the
// Wi-Fi task / callback timing, FreeRTOS queues, sensor drivers, the IMU task,
// interrupts, esp-mqtt, TLS, the Azure SDK, real IoT Hub behaviour or heap
// behaviour on the ESP32. The "firmware loop" cadence below mirrors
// firmware/*/main.cpp (timers only) so the cores are driven the same way.
#pragma once

#include <ArduinoJson.h>
#include <stdio.h>
#include <string.h>

#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "vc_cloud_link.h"
#include "vc_gateway_app.h"
#include "vc_gateway_link.h"
#include "vc_log.h"
#include "vc_sensor_app.h"
#include "vc_sensor_link.h"

namespace sim {

using namespace vc;
using namespace vc::proto;

// ------------------------------------------------------------- utilities ----

struct Rng : public RandomSource {
  explicit Rng(uint32_t seed = 1) : s(seed ? seed : 1) {}
  uint32_t random32() override {
    s ^= s << 13;
    s ^= s >> 17;
    s ^= s << 5;
    return s;
  }
  bool chance(uint32_t percent) { return percent && random32() % 100 < percent; }
  uint32_t s;
};

// NVS stand-in. Survives "reboots" of the device that owns it.
struct MemStore : public CounterStore {
  uint32_t value = 0;
  bool failStores = false;
  unsigned writes = 0;
  bool load(uint32_t& v) override {
    v = value;
    return true;
  }
  bool store(uint32_t v) override {
    if (failStores) return false;
    value = v;
    ++writes;
    return true;
  }
};

// Captured log lines (for "no secrets in logs" assertions).
inline std::vector<std::string>& logLines() {
  static std::vector<std::string> lines;
  return lines;
}
inline void captureLog(const char* line) {
  std::vector<std::string>& l = logLines();
  if (l.size() > 20000) l.erase(l.begin(), l.begin() + 10000);
  l.push_back(line);
  if (getenv("VC_HARNESS_VERBOSE")) printf("    | %s\n", line);
}

static const uint8_t kLmk[16] = {0x7a, 0x11, 0x3c, 0x90, 0x01, 0xfe, 0x55, 0x2b,
                                 0x68, 0x0d, 0xc4, 0x9e, 0x31, 0x77, 0xa2, 0x40};
static const uint8_t kSensorMac[6] = {0x24, 0x6F, 0x28, 0x10, 0x20, 0x01};
static const uint8_t kGatewayMac[6] = {0x24, 0x6F, 0x28, 0x10, 0x20, 0x02};
static const uint8_t kStrangerMac[6] = {0x24, 0x6F, 0x28, 0x10, 0x20, 0x99};
static const int64_t kUnixBase = 1761393600000LL;  // 2025-10-25T12:00:00Z

inline size_t propertyBagLength(const std::string& messageId, const std::string& messageType);

// ------------------------------------------------------------------- air ----

struct AirPolicy {
  uint32_t lossPct = 0;       // frame lost (sender sees a failed MAC-level send)
  uint32_t dupPct = 0;        // frame delivered twice
  uint32_t minDelayMs = 1;
  uint32_t maxDelayMs = 3;    // random delay in [min, max]: larger spreads reorder frames
  uint8_t dropMsgType = 0;    // != 0: every frame of this protocol type is lost
};

struct Frame {
  bool toGateway = true;
  uint8_t src[6];
  uint8_t channel = 1;
  uint32_t deliverAt = 0;  // world time
  uint64_t order = 0;
  std::vector<uint8_t> data;
};

class World;

class SensorRadio : public LinkRadio {
 public:
  explicit SensorRadio(World* w) : w_(w) {}
  bool send(const uint8_t* data, size_t len) override;
  void setChannel(uint8_t ch) override { channel = ch; }
  uint8_t channel = 1;

 private:
  World* w_;
};

class GatewayRadio : public LinkRadio {
 public:
  explicit GatewayRadio(World* w) : w_(w) {}
  bool send(const uint8_t* data, size_t len) override;

 private:
  World* w_;
};

// --------------------------------------------------------- IoT Hub / MQTT ----

struct HubMessage {
  uint32_t generation;
  int mqttMsgId;
  std::string messageId;  // $.mid
  std::string messageType;
  std::string payload;
  uint32_t receivedAt;  // world time
};

class Broker : public MqttPort {
 public:
  explicit Broker(World* w) : w_(w) {}
  // --- behaviour knobs
  bool reachable = true;         // TCP/TLS connect possible and traffic flows
  int refuseCode = 0;            // != 0: CONNACK refuses with this code
  uint32_t connackDelayMs = 80;
  uint32_t pubackDelayMs = 40;
  bool withholdPubacks = false;  // hub receives, but the PUBACK never comes back
  int forcePublishResult = 1;    // <= 0: publish() returns this (client refuses)
  // --- observations
  std::vector<HubMessage> hub;   // messages IoT Hub actually received
  std::vector<std::string> passwords;  // SAS tokens presented, one per start()
  unsigned starts = 0, stops = 0;
  std::vector<uint32_t> startTimes;    // world time of each start()

  bool start(uint32_t generation, const char* password) override;
  void stop() override;
  int publish(const char* messageId, const char* messageType, const char* payload, size_t len,
              size_t* wireBytes) override;
  // Broker-side drop of the current connection (e.g. IoT Hub restart).
  void dropConnection();
  void step(uint32_t worldNow, CloudLinkCore& cloud, uint32_t deviceNow);
  bool connected() const { return connected_; }

  std::vector<const HubMessage*> ofType(const char* type) const {
    std::vector<const HubMessage*> out;
    for (size_t i = 0; i < hub.size(); ++i)
      if (hub[i].messageType == type) out.push_back(&hub[i]);
    return out;
  }

 private:
  struct Scheduled {
    uint32_t at;
    MqttEvent ev;
    bool deliverToHub;
    HubMessage msg;
  };
  void schedule(const Scheduled& s) {
    // Keep events ordered by due time (stable for equal times).
    std::deque<Scheduled>::iterator it = events_.end();
    while (it != events_.begin() && (int32_t)((it - 1)->at - s.at) > 0) --it;
    events_.insert(it, s);
  }
  World* w_;
  bool running_ = false;
  bool connected_ = false;
  uint32_t gen_ = 0;
  int nextId_ = 1;
  std::deque<Scheduled> events_;
};

class FakeTokens : public SasTokenSource {
 public:
  explicit FakeTokens(World* w) : w_(w) {}
  bool generate(uint32_t lifetimeSec, char* out, size_t cap) override;
  bool fail = false;
  unsigned generated = 0;

 private:
  World* w_;
};

class SimClock : public WallClock {
 public:
  explicit SimClock(World* w) : w_(w) {}
  bool valid() const override;
  int64_t unixMs() const override;

 private:
  World* w_;
};

class SimPlatform : public PlatformStatus {
 public:
  const char* wifiState() const override { return networkUp ? "connected" : "connecting"; }
  int rssi() const override { return networkUp ? -58 : 0; }
  uint32_t freeHeap() const override { return 150000; }
  uint32_t minFreeHeap() const override { return 140000; }
  uint32_t radioRxOverflows() const override { return 0; }
  bool networkUp = true;
};

// ----------------------------------------------------------------- nodes ----

struct PendingImpact {
  uint32_t edgeAtDevice;  // device millis of the first edge
  uint16_t pulses;
  float peakG;
};

// Sensor-node firmware stand-in: same cores, same cadence as main.cpp.
struct SensorNode {
  explicit SensorNode(World* w) : radio(w) {}
  SensorRadio radio;
  Rng rng{0xC0FFEE};
  SensorLinkCore link;
  SensorApp app;
  PeriodicTimer tiltTimer{20}, telemetryTimer{500};
  uint32_t bootWorld = 0, millisAtBoot = 0;
  uint32_t bootId = 0;
  std::deque<bool> sendResults;
  // Inputs the "drivers" would provide.
  DhtReading dht[2];
  ImuSample imu;
  bool imuLive = true;  // the IMU task keeps stamping fresh samples
  ShockCounters shock;
  std::vector<PendingImpact> impacts;
  uint32_t telemetrySent = 0;
  uint32_t dhtReadAt[2] = {0, 0};  // like DhtSampler: a new value every 4 s
};

struct GatewayNode {
  explicit GatewayNode(World* w) : radio(w), clock(w), tokens(w) {}
  GatewayRadio radio;
  SimClock clock;
  FakeTokens tokens;
  SimPlatform platform;
  Rng rng{0xBADC0DE};
  GatewayLinkCore link;
  CloudLinkCore cloud;
  GatewayApp app;
  PeriodicTimer flameTimer{100}, dhtTimer{3000}, potTimer{500};
  uint32_t bootWorld = 0, millisAtBoot = 0;
  uint32_t bootId = 0;
};

struct WorldConfig {
  uint32_t seed = 12345;
  uint32_t sensorMillisAtBoot = 0;   // set near 0xFFFFFFFF to test the millis() wrap
  uint32_t gatewayMillisAtBoot = 0;
  bool linkConfigured = true;
  SensorAppConfig sensorApp;
  SensorLinkConfig sensorLink;
  GatewayAppConfig gatewayApp;
  CloudLinkConfig cloud;
  bool cloudConfigured = true;
};

class World {
  WorldConfig cfg_;

 public:
  explicit World(const WorldConfig& cfg = WorldConfig()) : cfg_(cfg), rng(cfg.seed), broker(this) {
    setLogSink(captureLog);
    bootSensor();
    bootGateway();
  }
  ~World() { setLogSink(nullptr); }

  // ---------------------------------------------------------- environment
  uint32_t now = 0;  // world milliseconds since the scenario started
  AirPolicy toGateway, toSensor;
  bool inRange = true;
  bool sensorAlive = true, gatewayAlive = true;
  uint8_t apChannel = 6;  // the gateway's channel, dictated by its AP
  bool clockSynced = true;
  bool networkUp = true;
  float flamePct = 5.0f;  // what the flame ADC reads (if the gateway samples it)
  float potPct = 37.5f;
  DhtReading dht22;
  Rng rng;
  Broker broker;
  MemStore sensorBootStore, gatewayBootStore, gatewaySensorBootStore, gatewaySensorNonceStore;
  std::vector<Frame> capturedToGateway;  // everything the sensor transmitted
  uint64_t frameOrder = 0;
  std::vector<Frame> air;

  std::unique_ptr<SensorNode> sensor;
  std::unique_ptr<GatewayNode> gateway;

  uint32_t sensorMillis() const { return sensor->millisAtBoot + (now - sensor->bootWorld); }
  uint32_t gatewayMillis() const { return gateway->millisAtBoot + (now - gateway->bootWorld); }

  // ------------------------------------------------------------- lifecycle
  void bootSensor(uint32_t millisAtBoot = 0xFFFFFFFFu) {
    sensor.reset(new SensorNode(this));
    SensorNode& s = *sensor;
    s.bootWorld = now;
    s.millisAtBoot = millisAtBoot == 0xFFFFFFFFu ? cfg_.sensorMillisAtBoot : millisAtBoot;
    s.radio.channel = 1;
    s.bootId = advanceBootCounter(sensorBootStore);
    s.rng.s = rng.random32() | 1;  // the hardware RNG differs on every boot
    s.dht[0].status = SensorStatus::Ok;
    s.dht[0].tempC = 23.4f;
    s.dht[0].humidityPct = 48.0f;
    s.dht[1] = s.dht[0];
    s.imu.status = SensorStatus::Ok;
    s.imu.accelZG = 1.0f;
    s.imu.accelMagG = 1.0f;
    s.imu.peakAccelG = 1.02f;
    s.imu.rateHz = 100.0f;
    s.imu.maxGapMs = 11;
    uint32_t t = sensorMillis();
    s.app.begin(cfg_.sensorApp, &s.link);
    if (cfg_.linkConfigured && s.bootId != 0)
      s.link.begin(deriveLinkKeys(kLmk), kSensorMac, kGatewayMac, s.bootId, false, &s.radio, &s.rng,
                   &sensorBootStore, cfg_.sensorLink, t);
    s.tiltTimer.start(t);
    s.telemetryTimer.start(t);
    s.dhtReadAt[0] = t;
    s.dhtReadAt[1] = t - 2000;  // staggered like the firmware
    sensorAlive = true;
  }

  void bootGateway(uint32_t millisAtBoot = 0xFFFFFFFFu) {
    gateway.reset(new GatewayNode(this));
    GatewayNode& g = *gateway;
    g.bootWorld = now;
    g.millisAtBoot = millisAtBoot == 0xFFFFFFFFu ? cfg_.gatewayMillisAtBoot : millisAtBoot;
    g.bootId = advanceBootCounter(gatewayBootStore);
    g.rng.s = rng.random32() | 1;
    uint32_t t = gatewayMillis();
    if (cfg_.cloudConfigured) g.cloud.begin(&broker, &g.tokens, &g.rng, cfg_.cloud);
    if (cfg_.linkConfigured)
      g.link.begin(deriveLinkKeys(kLmk), kSensorMac, kGatewayMac, &g.radio, &g.rng, &gatewaySensorBootStore,
                   &gatewaySensorNonceStore, &g.app);
    GatewayAppConfig ac = cfg_.gatewayApp;
    ac.gatewayId = "vehicle-gateway-01";
    ac.sensorNodeId = "sensor-node-01";
    ac.gatewayBootId = g.bootId;
    g.app.begin(ac, &g.clock, cfg_.cloudConfigured ? &g.cloud : nullptr, &g.link, &g.platform, t);
    g.flameTimer.start(t);
    g.dhtTimer.start(t);
    g.potTimer.start(t);
    gatewayAlive = true;
    dht22.status = SensorStatus::Ok;
    dht22.tempC = 21.0f;
    dht22.humidityPct = 40.0f;
  }

  void powerOffSensor() { sensorAlive = false; }
  void powerOffGateway() {
    gatewayAlive = false;
    broker.stop();  // the TCP connection dies with the board
  }

  // --------------------------------------------------------------- inputs
  // A shock pulse train starting now; the record is released (like the ISR
  // ring) once its 200 ms refractory window has closed.
  void impact(uint16_t pulses = 3, float peakG = 2.6f) {
    SensorNode& s = *sensor;
    s.impacts.push_back({sensorMillis(), pulses, peakG});
    ++s.shock.impacts;
    s.shock.edges += pulses;
  }
  void setTilt(float deg) { sensor->imu.rollDeg = deg; }

  // Raw frame injection (attacker / corrupted radio) towards the gateway.
  void injectToGateway(const uint8_t src[6], const std::vector<uint8_t>& data, uint32_t delayMs = 1) {
    Frame f;
    f.toGateway = true;
    memcpy(f.src, src, 6);
    f.channel = apChannel;
    f.deliverAt = now + delayMs;
    f.order = frameOrder++;
    f.data = data;
    air.push_back(f);
  }
  void injectToSensor(const uint8_t src[6], const std::vector<uint8_t>& data, uint32_t delayMs = 1) {
    Frame f;
    f.toGateway = false;
    memcpy(f.src, src, 6);
    f.channel = sensor->radio.channel;
    f.deliverAt = now + delayMs;
    f.order = frameOrder++;
    f.data = data;
    air.push_back(f);
  }

  // ------------------------------------------------------------ transport
  bool transmit(bool toGw, const uint8_t* data, size_t len) {
    // Returns whether the MAC-level ACK would come back.
    AirPolicy& p = toGw ? toGateway : toSensor;
    uint8_t txChannel = toGw ? sensor->radio.channel : apChannel;
    Frame f;
    f.toGateway = toGw;
    memcpy(f.src, toGw ? kSensorMac : kGatewayMac, 6);
    f.channel = txChannel;
    f.data.assign(data, data + len);
    if (toGw) capturedToGateway.push_back(f);
    bool receiverOn = toGw ? gatewayAlive : sensorAlive;
    uint8_t rxChannel = toGw ? apChannel : sensor->radio.channel;
    if (!inRange || !receiverOn || rxChannel != txChannel || rng.chance(p.lossPct)) return false;
    if (p.dropMsgType && len > 2 && data[2] == p.dropMsgType) return false;
    unsigned copies = rng.chance(p.dupPct) ? 2 : 1;
    for (unsigned c = 0; c < copies; ++c) {
      uint32_t span = p.maxDelayMs > p.minDelayMs ? p.maxDelayMs - p.minDelayMs : 0;
      f.deliverAt = now + p.minDelayMs + (span ? rng.random32() % (span + 1) : 0);
      f.order = frameOrder++;
      air.push_back(f);
    }
    return true;
  }

  // ----------------------------------------------------------------- time
  void step() {
    ++now;
    deliverFrames();
    if (sensorAlive) stepSensor();
    if (gatewayAlive) stepGateway();
  }
  void run(uint32_t ms) {
    for (uint32_t i = 0; i < ms; ++i) step();
  }
  void runSeconds(uint32_t s) { run(s * 1000u); }

  const WorldConfig& config() const { return cfg_; }

 private:
  void deliverFrames() {
    // Deliver due frames in delivery-time order (ties by send order).
    for (;;) {
      int best = -1;
      for (size_t i = 0; i < air.size(); ++i) {
        if ((int32_t)(now - air[i].deliverAt) < 0) continue;
        if (best < 0 || air[i].deliverAt < air[best].deliverAt ||
            (air[i].deliverAt == air[best].deliverAt && air[i].order < air[best].order))
          best = (int)i;
      }
      if (best < 0) return;
      Frame f = air[best];
      air.erase(air.begin() + best);
      if (f.toGateway) {
        if (gatewayAlive && inRange && f.channel == apChannel)
          gateway->link.onFrame(f.src, f.data.data(), f.data.size(), gatewayMillis(), f.channel, -60);
      } else {
        if (sensorAlive && inRange && f.channel == sensor->radio.channel)
          sensor->link.onFrame(f.src, f.data.data(), f.data.size(), sensorMillis());
      }
    }
  }

  void stepSensor() {
    SensorNode& s = *sensor;
    uint32_t t = sensorMillis();
    while (!s.sendResults.empty()) {
      s.link.onSendResult(s.sendResults.front());
      s.sendResults.pop_front();
    }
    if (s.imuLive) s.imu.sampleMs = t;
    for (int i = 0; i < 2; ++i) {
      if (hasElapsed(t, s.dhtReadAt[i], 4000)) s.dhtReadAt[i] = t;
      s.dht[i].ageMs = statusHasValue(s.dht[i].status) ? elapsedMs(t, s.dhtReadAt[i]) : 0;
    }
    // shock::pop(): records whose refractory window has closed.
    for (size_t i = 0; i < s.impacts.size();) {
      if (hasElapsed(t, s.impacts[i].edgeAtDevice, 200)) {
        s.app.onImpact(s.impacts[i].edgeAtDevice, s.impacts[i].pulses, s.impacts[i].peakG);
        s.impacts.erase(s.impacts.begin() + i);
      } else {
        ++i;
      }
    }
    if (s.tiltTimer.due(t)) s.app.evaluateTilt(s.imu, t);
    if (s.telemetryTimer.due(t)) {
      TelemetryBody body = s.app.buildTelemetry(t, s.dht[0], s.dht[1], s.imu, s.shock);
      if (s.link.sendTelemetry(body, t)) ++s.telemetrySent;
    }
    s.link.tick(t);
  }

  void stepGateway() {
    GatewayNode& g = *gateway;
    uint32_t t = gatewayMillis();
    g.platform.networkUp = networkUp;
    broker.step(now, g.cloud, t);
    g.cloud.tick(t, networkUp, clockSynced);
    if (g.flameTimer.due(t) && cfg_.gatewayApp.flameEnabled) g.app.onFlameSample(flamePct, t);
    // Like DhtSampler: a successful read yields a fresh value (age 0).
    if (g.dhtTimer.due(t) && cfg_.gatewayApp.dht22Enabled) {
      DhtReading r = dht22;
      r.ageMs = 0;
      g.app.onDht22Reading(r, r.status == SensorStatus::Ok);
    }
    if (g.potTimer.due(t) && cfg_.gatewayApp.potEnabled) g.app.onPotSample(potPct);
    g.app.tick(t);
  }

  friend class SensorRadio;
  friend class GatewayRadio;
};

// ------------------------------------------------------- implementations ----

inline bool SensorRadio::send(const uint8_t* data, size_t len) {
  bool acked = w_->transmit(true, data, len);
  w_->sensor->sendResults.push_back(acked);
  return true;  // the driver accepted the frame
}

inline bool GatewayRadio::send(const uint8_t* data, size_t len) {
  w_->transmit(false, data, len);
  return true;
}

inline bool Broker::start(uint32_t generation, const char* password) {
  ++starts;
  startTimes.push_back(w_->now);
  passwords.push_back(password ? password : "");
  running_ = true;
  connected_ = false;
  gen_ = generation;
  Scheduled s;
  s.at = w_->now + connackDelayMs;
  s.ev.generation = generation;
  s.deliverToHub = false;
  if (!reachable || !w_->networkUp) {
    s.ev.kind = MqttEvent::Kind::TransportError;
    s.ev.sockErrno = 113;  // EHOSTUNREACH
    s.at = w_->now + 3000;
  } else if (refuseCode) {
    s.ev.kind = MqttEvent::Kind::Refused;
    s.ev.refuseCode = refuseCode;
  } else {
    s.ev.kind = MqttEvent::Kind::Connected;
  }
  schedule(s);
  return true;
}

inline void Broker::stop() {
  if (running_) ++stops;
  running_ = false;
  connected_ = false;
  // Events already scheduled stay queued, like records already sitting in the
  // firmware's FreeRTOS queue: the core must ignore them by generation.
  // In-flight messages that have not reached the hub are lost with the socket.
  for (size_t i = 0; i < events_.size(); ++i)
    if (events_[i].deliverToHub && events_[i].ev.generation == gen_) events_[i].deliverToHub = false;
}

inline int Broker::publish(const char* messageId, const char* messageType, const char* payload, size_t len,
                           size_t* wireBytes) {
  if (forcePublishResult <= 0) return forcePublishResult;
  if (!running_) return -1;
  int id = nextId_;
  nextId_ = nextId_ >= 65535 ? 1 : nextId_ + 1;
  Scheduled s;
  s.at = w_->now + pubackDelayMs;
  s.ev.generation = gen_;
  s.ev.kind = MqttEvent::Kind::Published;
  s.ev.msgId = id;
  s.deliverToHub = connected_ && reachable;
  s.msg.generation = gen_;
  s.msg.mqttMsgId = id;
  s.msg.messageId = messageId;
  s.msg.messageType = messageType;
  s.msg.payload.assign(payload, len);
  schedule(s);
  if (wireBytes) *wireBytes = len + propertyBagLength(messageId, messageType);
  return id;
}

inline void Broker::dropConnection() {
  if (!running_ || !connected_) return;
  connected_ = false;
  for (size_t i = 0; i < events_.size(); ++i)
    if (events_[i].ev.generation == gen_) events_[i].deliverToHub = false;
  Scheduled s;
  s.at = w_->now + 1;
  s.ev.generation = gen_;
  s.ev.kind = MqttEvent::Kind::Disconnected;
  s.deliverToHub = false;
  schedule(s);
}

inline void Broker::step(uint32_t worldNow, CloudLinkCore& cloud, uint32_t deviceNow) {
  while (!events_.empty() && (int32_t)(worldNow - events_.front().at) >= 0) {
    Scheduled s = events_.front();
    events_.pop_front();
    if (s.ev.kind == MqttEvent::Kind::Connected) {
      if (!running_ || s.ev.generation != gen_) continue;  // that client is gone
      connected_ = true;
    }
    if (s.ev.kind == MqttEvent::Kind::Published) {
      bool same = running_ && s.ev.generation == gen_;
      if (s.deliverToHub && reachable) {
        s.msg.receivedAt = worldNow;
        hub.push_back(s.msg);
      } else {
        continue;  // never reached the hub: no PUBACK either
      }
      if (!same || withholdPubacks) continue;
    }
    cloud.onMqttEvent(s.ev, deviceNow);
  }
  // Network outage while connected: the transport notices and reports it.
  if (connected_ && (!reachable || !w_->networkUp)) dropConnection();
}

inline bool FakeTokens::generate(uint32_t lifetimeSec, char* out, size_t cap) {
  if (fail || !w_->clockSynced) return false;
  ++generated;
  int64_t se = (w_->gateway->clock.unixMs() / 1000) + lifetimeSec;
  // Looks like a real token so the log check is meaningful.
  int n = snprintf(out, cap, "SharedAccessSignature sr=test-hub.azure-devices.net%%2Fdevices%%2Fgw&sig=SECRET%04uX&se=%lld",
                   generated, (long long)se);
  return n > 0 && (size_t)n < cap;
}

inline bool SimClock::valid() const { return w_->clockSynced; }
inline int64_t SimClock::unixMs() const { return kUnixBase + (int64_t)w_->now; }

// ------------------------------------------------------------- assertions ----

// Length of the property bag the gateway appends to the telemetry topic
// (firmware/gateway/azure_link.cpp builds exactly these four properties).
inline size_t propertyBagLength(const std::string& messageId, const std::string& messageType) {
  char enc[200];
  urlEncode(messageId.c_str(), enc, sizeof(enc));
  return strlen("$.ct=application%2Fjson&$.ce=utf-8&$.mid=") + strlen(enc) + strlen("&messageType=") +
         messageType.size();
}

inline bool strictJson(const std::string& s, JsonDocument& doc) {
  if (s.find("nan") != std::string::npos || s.find("inf") != std::string::npos) return false;
  return deserializeJson(doc, s) == DeserializationError::Ok;
}

}  // namespace sim
