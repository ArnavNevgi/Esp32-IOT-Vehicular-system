// Gateway side of the authenticated local link (hardware independent).
//
// Frames are accepted only if they (1) come from the configured sensor MAC,
// (2) pass framing and direction checks and the HMAC tag, and (3) pass the
// session / replay policy in vc::PeerSession. No session or replay state
// changes before the tag has been verified. Rejections are counted per reason.
//
// Events: a new (boot, eventCounter) is handed to the SensorEventSink; it is
// acknowledged to the sensor only once the sink reports it queued, and every
// retransmitted copy is acknowledged again without being processed twice.
#pragma once

#include <stdint.h>

#include "vc_aggregate.h"
#include "vc_ports.h"
#include "vc_protocol.h"
#include "vc_session.h"

namespace vc {

class SensorEventSink {
 public:
  virtual ~SensorEventSink() {}
  // Return true once the event is safely queued (only then is it acked).
  virtual bool onSensorEvent(const proto::Header& h, const proto::EventBody& e, uint32_t rxMs) = 0;
};

struct GatewayLinkConfig {
  uint32_t rejectMinIntervalMs = 500;  // limits SessionReject traffic
};

class GatewayLinkCore {
 public:
  static const unsigned kRecentEvents = 32;

  struct Stats {
    uint32_t accepted = 0;
    uint32_t unknownPeer = 0;
    uint32_t malformed = 0;  // framing, direction, field checks
    uint32_t badTag = 0;
    uint32_t replay = 0;
    uint32_t sessionRejected = 0;  // no / wrong session, stale boot
    uint32_t helloRefused = 0;
    uint32_t duplicateEvents = 0;
    uint32_t eventsNotQueued = 0;  // sink refused: left unacknowledged
    uint32_t sessions = 0;
    uint32_t rejectsSent = 0;
    uint32_t persistFailures = 0;  // highest granted boot could not be stored
  };

  // `bootStore` / `nonceStore` hold the highest sensor boot ever granted a
  // session and that boot's nonce (see vc_session.h).
  bool begin(const proto::LinkKeys& keys, const uint8_t sensorMac[6], const uint8_t gatewayMac[6],
             LinkRadio* radio, RandomSource* rng, CounterStore* bootStore, CounterStore* nonceStore,
             SensorEventSink* sink, const GatewayLinkConfig& cfg = GatewayLinkConfig());

  void onFrame(const uint8_t src[6], const uint8_t* data, size_t len, uint32_t now, uint8_t channel = 0,
               int8_t rssi = 0);

  bool configured() const { return configured_; }
  bool everSeen() const { return everSeen_; }
  uint32_t lastRxMs() const { return lastRxMs_; }
  const SensorNodeAggregator& aggregator() const { return agg_; }
  SensorNodeAggregator& aggregator() { return agg_; }
  const PeerSession& session() const { return session_; }
  const Stats& stats() const { return stats_; }
  uint32_t rejectedTotal() const {
    return stats_.unknownPeer + stats_.malformed + stats_.badTag + stats_.replay + stats_.sessionRejected +
           stats_.helloRefused;
  }

 private:
  void reply(proto::Packet& p, uint32_t sensorBoot, uint32_t session, uint32_t now);
  void sendReject(uint32_t boot, uint32_t session, proto::RejectReason reason, uint32_t value, uint32_t now);

  bool configured_ = false;
  GatewayLinkConfig cfg_;
  proto::AuthContext tx_;  // gateway -> sensor
  proto::AuthContext rx_;  // sensor -> gateway
  uint8_t sensorMac_[6] = {0};
  LinkRadio* radio_ = nullptr;
  RandomSource* rng_ = nullptr;
  CounterStore* bootStore_ = nullptr;
  CounterStore* nonceStore_ = nullptr;
  SensorEventSink* sink_ = nullptr;
  PeerSession session_;
  RecentEventIds<kRecentEvents> recentEvents_;
  SensorNodeAggregator agg_;
  Stats stats_;
  uint32_t txSeq_ = 0;
  bool rejectSent_ = false;
  uint32_t lastRejectSentMs_ = 0;
  uint32_t lastRxMs_ = 0;
  bool everSeen_ = false;
};

}  // namespace vc
