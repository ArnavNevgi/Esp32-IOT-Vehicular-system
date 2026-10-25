// Sensor-node side of the authenticated local link (hardware independent).
//
// Channel coordination: the gateway's radio channel is dictated by the access
// point it is associated with, so the sensor never forces a channel on the
// gateway. Instead the sensor (not associated with any AP) sweeps channels
// 1..maxChannel, sending an authenticated Hello on each and dwelling briefly
// for a HelloAck; it then stays on the channel that answered. If the AP (and
// therefore the gateway) moves to another channel, the sensor's sends stop
// being acknowledged at the radio level, or the gateway rejects the session,
// and the sensor sweeps again, starting from the last good channel. Sweeps
// that find nothing back off exponentially (1 s .. 30 s, jittered).
//
// Boot recovery: every Hello carries the boot id and a random per-boot nonce.
// If the gateway refuses the Hello because this boot id is older than the
// newest one it has granted, or equal to it with a different nonce (the
// sensor's NVS was erased or a write was lost), it answers with an
// authenticated SessionReject(StaleBoot) carrying that highest granted boot.
// The sensor then persists highest+1 and retries, so it never reuses a boot
// id (and therefore never reuses an event id).
//
// The firmware adapter (firmware/sensor_node/link_client.*) feeds received
// frames and radio send results in from the Wi-Fi task via queues and calls
// tick() from the loop; nothing here runs in interrupt or callback context.
#pragma once

#include <stdint.h>

#include "vc_backoff.h"
#include "vc_event_queue.h"
#include "vc_ports.h"
#include "vc_protocol.h"

namespace vc {

struct SensorLinkConfig {
  uint32_t helloDwellMs = 60;
  uint8_t maxChannel = 13;
  uint32_t failsBeforeRescan = 8;  // consecutive unacknowledged radio sends
  uint32_t eventRetryMs = 300;
  uint8_t eventMaxAttempts = 40;
  uint32_t rejectMinIntervalMs = 2000;
  uint32_t scanBackoffBaseMs = 1000;
  uint32_t scanBackoffCapMs = 30000;
  uint32_t rehandshakeAtSeq = 0xFFFFFF00u;  // renew the session before seq wraps
};

// Increments a persisted boot counter and returns the new value, or 0 if the
// store failed or the counter is exhausted. The new value is returned only
// after store() reports it committed, so a power loss during the write can at
// worst skip a value, never reuse one.
uint32_t advanceBootCounter(CounterStore& store);

class SensorLinkCore {
 public:
  enum class State : uint8_t { Disabled, Scanning, Backoff, Linked };
  static const unsigned kEventSlots = 8;

  struct Stats {
    uint32_t rxRejected = 0;  // all rejected frames (sum of the reasons below and more)
    uint32_t rxUnknownPeer = 0;
    uint32_t rxBadTag = 0;
    uint32_t rxMalformed = 0;
    uint32_t rxWrongBoot = 0;
    uint32_t rxUnexpected = 0;  // valid frame, wrong state / session / nonce
    uint32_t radioSendFailures = 0;
    uint32_t helloSent = 0;
    uint32_t sessions = 0;
    uint32_t eventAcks = 0;
    uint32_t bootAdvances = 0;
    uint16_t restarts = 0;
  };

  // Returns false (and stays Disabled) if bootId is 0 or a port is missing.
  bool begin(const proto::LinkKeys& keys, const uint8_t sensorMac[6], const uint8_t gatewayMac[6], uint32_t bootId,
             bool synthetic, LinkRadio* radio, RandomSource* rng, CounterStore* bootStore,
             const SensorLinkConfig& cfg, uint32_t now);

  // A frame received from `src` (copied out of the radio callback).
  void onFrame(const uint8_t src[6], const uint8_t* data, size_t len, uint32_t now);
  // Radio-level result of an earlier send (MAC-layer ACK or not).
  void onSendResult(bool delivered);
  void tick(uint32_t now);

  bool sendTelemetry(const proto::TelemetryBody& t, uint32_t now);
  // Queue an event for acknowledged delivery (kept across re-handshakes).
  void queueEvent(const proto::EventBody& e);

  State state() const { return state_; }
  const char* stateName() const;
  bool linked() const { return state_ == State::Linked; }
  uint8_t channel() const { return channel_; }
  uint32_t bootId() const { return boot_; }
  uint32_t bootNonce() const { return bootNonce_; }
  uint32_t session() const { return session_; }
  uint32_t txSeq() const { return txSeq_; }
  uint16_t linkRestarts() const { return stats_.restarts; }
  uint32_t eventsDropped() const { return events_.dropped(); }
  uint32_t eventsExpired() const { return events_.expired(); }
  unsigned eventsPending() const { return events_.size(); }
  unsigned eventsHighWater() const { return eventsHighWater_; }
  uint32_t lastGatewayRxAgeMs(uint32_t now) const { return elapsedMs(now, lastGatewayRxMs_); }
  const Stats& stats() const { return stats_; }

 private:
  bool send(proto::Packet& p, uint32_t now);
  void startScan(uint32_t now, bool fromCurrentChannel);
  void sendHello(uint32_t now);
  void serviceEvents(uint32_t now);
  void reject(uint32_t& counter);

  State state_ = State::Disabled;
  SensorLinkConfig cfg_;
  proto::AuthContext tx_;  // sensor -> gateway
  proto::AuthContext rx_;  // gateway -> sensor
  uint8_t gatewayMac_[6] = {0};
  LinkRadio* radio_ = nullptr;
  RandomSource* rng_ = nullptr;
  CounterStore* bootStore_ = nullptr;
  uint32_t boot_ = 0;
  uint32_t bootNonce_ = 0;  // random per boot (RAM only)
  bool synthetic_ = false;
  uint32_t session_ = 0;
  uint32_t txSeq_ = 0;
  uint32_t helloNonce_ = 0;
  uint8_t channel_ = 1;
  uint8_t lastGoodChannel_ = 1;
  uint8_t scanned_ = 0;
  uint32_t dwellStart_ = 0;
  uint32_t failStreak_ = 0;
  bool rejectSeen_ = false;
  uint32_t lastRejectMs_ = 0;
  uint32_t lastGatewayRxMs_ = 0;
  unsigned eventsHighWater_ = 0;
  RetryTimer scanRetry_{1000, 30000, 25};
  RetransmitQueue<kEventSlots> events_;
  Stats stats_;
};

}  // namespace vc
