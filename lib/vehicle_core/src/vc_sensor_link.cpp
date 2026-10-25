#include "vc_sensor_link.h"

#include <string.h>

#include "vc_log.h"

namespace vc {

using namespace proto;

uint32_t advanceBootCounter(CounterStore& store) {
  uint32_t last = 0;
  if (!store.load(last)) return 0;
  // 0 is never a valid boot id; at UINT32_MAX the counter is exhausted and the
  // link stays off rather than wrapping to an id the gateway already granted.
  if (last == 0xFFFFFFFFu) return 0;
  uint32_t next = last + 1;
  if (!store.store(next)) return 0;
  return next;
}

const char* SensorLinkCore::stateName() const {
  switch (state_) {
    case State::Disabled: return "disabled";
    case State::Scanning: return "scanning";
    case State::Backoff: return "backoff";
    case State::Linked: return "linked";
  }
  return "?";
}

bool SensorLinkCore::begin(const LinkKeys& keys, const uint8_t sensorMac[6], const uint8_t gatewayMac[6],
                           uint32_t bootId, bool synthetic, LinkRadio* radio, RandomSource* rng,
                           CounterStore* bootStore, const SensorLinkConfig& cfg, uint32_t now) {
  state_ = State::Disabled;
  if (bootId == 0 || !radio || !rng || cfg.maxChannel == 0) return false;
  cfg_ = cfg;
  tx_ = makeAuthContext(keys, Direction::SensorToGateway, sensorMac, gatewayMac);
  rx_ = makeAuthContext(keys, Direction::GatewayToSensor, sensorMac, gatewayMac);
  memcpy(gatewayMac_, gatewayMac, 6);
  radio_ = radio;
  rng_ = rng;
  bootStore_ = bootStore;
  boot_ = bootId;
  bootNonce_ = rng->nonZero();
  synthetic_ = synthetic;
  scanRetry_ = RetryTimer(cfg.scanBackoffBaseMs, cfg.scanBackoffCapMs, 25);
  startScan(now, false);
  return true;
}

void SensorLinkCore::queueEvent(const EventBody& e) {
  events_.push(e);
  if (events_.size() > eventsHighWater_) eventsHighWater_ = events_.size();
}

bool SensorLinkCore::send(Packet& p, uint32_t now) {
  p.header.bootId = boot_;
  p.header.flags = synthetic_ ? kFlagSynthetic : 0;
  p.header.uptimeMs = now;
  if (p.header.type != MsgType::Hello) {
    p.header.sessionId = session_;
    p.header.seq = ++txSeq_;
  }
  uint8_t buf[kMaxPacketSize];
  size_t n = encode(p, tx_, buf, sizeof(buf));
  if (n == 0) return false;
  return radio_->send(buf, n);
}

void SensorLinkCore::startScan(uint32_t now, bool fromCurrentChannel) {
  state_ = State::Scanning;
  session_ = 0;
  scanned_ = 0;
  failStreak_ = 0;
  if (!fromCurrentChannel) channel_ = lastGoodChannel_;
  radio_->setChannel(channel_);
  sendHello(now);
}

void SensorLinkCore::sendHello(uint32_t now) {
  helloNonce_ = rng_->nonZero();
  Packet p;
  p.header.type = MsgType::Hello;
  p.header.sessionId = 0;
  p.header.seq = 0;
  p.nonce = helloNonce_;
  p.bootNonce = bootNonce_;
  send(p, now);
  ++stats_.helloSent;
  dwellStart_ = now;
}

bool SensorLinkCore::sendTelemetry(const TelemetryBody& t, uint32_t now) {
  if (state_ != State::Linked) return false;
  Packet p;
  p.header.type = MsgType::Telemetry;
  p.telemetry = t;
  return send(p, now);
}

void SensorLinkCore::serviceEvents(uint32_t now) {
  events_.expire(cfg_.eventMaxAttempts, now, cfg_.eventRetryMs);
  // At most one event transmission per tick keeps bursts short.
  RetransmitQueue<kEventSlots>::Entry* e = events_.nextDue(now, cfg_.eventRetryMs);
  if (!e) return;
  events_.markSent(e, now);
  Packet p;
  p.header.type = MsgType::Event;
  p.event = e->event;  // same eventCounter on every attempt
  send(p, now);
}

void SensorLinkCore::reject(uint32_t& counter) {
  ++counter;
  ++stats_.rxRejected;
}

void SensorLinkCore::onSendResult(bool delivered) {
  if (delivered) {
    failStreak_ = 0;
  } else {
    ++failStreak_;
    ++stats_.radioSendFailures;
  }
}

void SensorLinkCore::onFrame(const uint8_t src[6], const uint8_t* data, size_t len, uint32_t now) {
  if (state_ == State::Disabled) return;
  if (memcmp(src, gatewayMac_, 6) != 0) return reject(stats_.rxUnknownPeer);
  Packet p;
  DecodeError err = decode(data, len, rx_, p);
  if (err == DecodeError::BadTag) return reject(stats_.rxBadTag);
  if (err != DecodeError::Ok) return reject(stats_.rxMalformed);
  // Gateway frames are addressed to one sensor boot; older ones are stale.
  if (p.header.bootId != boot_) return reject(stats_.rxWrongBoot);

  switch (p.header.type) {
    case MsgType::HelloAck:
      // Only an answer to our outstanding Hello (fresh random nonce) counts.
      if (state_ != State::Scanning || p.nonce != helloNonce_) return reject(stats_.rxUnexpected);
      session_ = p.header.sessionId;
      txSeq_ = 0;
      lastGoodChannel_ = channel_;
      state_ = State::Linked;
      scanRetry_.succeeded();
      failStreak_ = 0;
      lastGatewayRxMs_ = now;
      ++stats_.sessions;
      logf("[link] linked on channel %u, session %08lx", (unsigned)channel_, (unsigned long)session_);
      return;
    case MsgType::EventAck:
      if (state_ != State::Linked || p.header.sessionId != session_) return reject(stats_.rxUnexpected);
      if (events_.ack(p.ackEventCounter)) ++stats_.eventAcks;
      lastGatewayRxMs_ = now;
      return;
    case MsgType::SessionReject:
      if (p.header.sessionId == 0) {
        // Answer to a Hello: this boot id is older than one the gateway has
        // already granted. Move past it (persisted first) and say Hello again.
        if (state_ != State::Scanning || p.rejectReason != RejectReason::StaleBoot ||
            p.rejectValue < boot_ || p.rejectValue == 0xFFFFFFFFu)
          return reject(stats_.rxUnexpected);
        uint32_t next = p.rejectValue + 1;
        if (!bootStore_ || !bootStore_->store(next)) {
          logf("[link] gateway requires boot id > %lu but it could not be persisted; link disabled",
               (unsigned long)p.rejectValue);
          state_ = State::Disabled;
          return;
        }
        logf("[link] gateway has granted boot %lu; advancing boot id %lu -> %lu", (unsigned long)p.rejectValue,
             (unsigned long)boot_, (unsigned long)next);
        boot_ = next;
        ++stats_.bootAdvances;
        lastGatewayRxMs_ = now;
        sendHello(now);
        return;
      }
      // Rate-limited and bound to the current session: a replayed or delayed
      // reject can force at most one re-handshake per interval, and only
      // within the session it was issued for.
      if (state_ != State::Linked || p.header.sessionId != session_ ||
          (rejectSeen_ && !hasElapsed(now, lastRejectMs_, cfg_.rejectMinIntervalMs)))
        return reject(stats_.rxUnexpected);
      rejectSeen_ = true;
      lastRejectMs_ = now;
      ++stats_.restarts;
      logf("[link] session rejected by gateway (reason %u); re-handshaking", (unsigned)p.rejectReason);
      startScan(now, true);
      return;
    default:
      return reject(stats_.rxMalformed);  // decode() already enforces direction
  }
}

void SensorLinkCore::tick(uint32_t now) {
  switch (state_) {
    case State::Disabled:
      return;
    case State::Scanning:
      if (hasElapsed(now, dwellStart_, cfg_.helloDwellMs)) {
        if (++scanned_ >= cfg_.maxChannel) {
          uint32_t wait = scanRetry_.failed(now, rng_->random32());
          state_ = State::Backoff;
          logf("[link] no gateway answered on channels 1-%u; next sweep in %lu ms", (unsigned)cfg_.maxChannel,
               (unsigned long)wait);
          return;
        }
        channel_ = (uint8_t)(channel_ % cfg_.maxChannel + 1);
        radio_->setChannel(channel_);
        sendHello(now);
      }
      return;
    case State::Backoff:
      if (scanRetry_.ready(now)) startScan(now, false);
      return;
    case State::Linked:
      if (failStreak_ >= cfg_.failsBeforeRescan) {
        ++stats_.restarts;
        logf("[link] %lu consecutive unacknowledged sends; gateway lost, rescanning", (unsigned long)failStreak_);
        startScan(now, true);
        return;
      }
      if (txSeq_ >= cfg_.rehandshakeAtSeq) {  // sequence space nearly exhausted
        startScan(now, true);
        return;
      }
      serviceEvents(now);
      return;
  }
}

}  // namespace vc
