#include "vc_gateway_link.h"

#include <string.h>

#include "vc_log.h"

namespace vc {

using namespace proto;

bool GatewayLinkCore::begin(const LinkKeys& keys, const uint8_t sensorMac[6], const uint8_t gatewayMac[6],
                            LinkRadio* radio, RandomSource* rng, CounterStore* bootStore,
                            CounterStore* nonceStore, SensorEventSink* sink, const GatewayLinkConfig& cfg) {
  configured_ = false;
  if (!radio || !rng) return false;
  cfg_ = cfg;
  tx_ = makeAuthContext(keys, Direction::GatewayToSensor, sensorMac, gatewayMac);
  rx_ = makeAuthContext(keys, Direction::SensorToGateway, sensorMac, gatewayMac);
  memcpy(sensorMac_, sensorMac, 6);
  radio_ = radio;
  rng_ = rng;
  bootStore_ = bootStore;
  nonceStore_ = nonceStore;
  sink_ = sink;
  uint32_t persisted = 0, nonce = 0;
  if (bootStore_ && bootStore_->load(persisted)) {
    // A missing or unreadable nonce is treated as unknown (accepted once).
    if (!nonceStore_ || !nonceStore_->load(nonce)) nonce = 0;
    session_.restorePersistedBoot(persisted, nonce);
  }
  configured_ = true;
  return true;
}

void GatewayLinkCore::reply(Packet& p, uint32_t sensorBoot, uint32_t session, uint32_t now) {
  p.header.bootId = sensorBoot;  // addressed to that sensor boot
  p.header.sessionId = session;
  p.header.seq = ++txSeq_;
  p.header.uptimeMs = now;
  p.header.flags = 0;
  uint8_t buf[kMaxPacketSize];
  size_t n = encode(p, tx_, buf, sizeof(buf));
  if (n) radio_->send(buf, n);
}

void GatewayLinkCore::sendReject(uint32_t boot, uint32_t session, RejectReason reason, uint32_t value,
                                 uint32_t now) {
  // Rate-limited so a flood of stale frames cannot keep the radio busy.
  if (rejectSent_ && !hasElapsed(now, lastRejectSentMs_, cfg_.rejectMinIntervalMs)) return;
  rejectSent_ = true;
  lastRejectSentMs_ = now;
  ++stats_.rejectsSent;
  Packet p;
  p.header.type = MsgType::SessionReject;
  p.rejectReason = reason;
  p.rejectValue = value;
  reply(p, boot, session, now);
}

void GatewayLinkCore::onFrame(const uint8_t src[6], const uint8_t* data, size_t len, uint32_t now, uint8_t channel,
                              int8_t rssi) {
  if (!configured_) return;
  if (memcmp(src, sensorMac_, 6) != 0) {
    ++stats_.unknownPeer;
    return;
  }
  Packet p;
  DecodeError err = decode(data, len, rx_, p);
  if (err == DecodeError::BadTag) {
    ++stats_.badTag;
    return;
  }
  if (err != DecodeError::Ok) {
    ++stats_.malformed;
    return;
  }
  // From here on the frame is authentic (it was produced with the sensor's
  // key for this direction and these addresses); it may still be old.
  const Header& h = p.header;

  if (h.type == MsgType::Hello) {
    bool persist = false;
    uint32_t sid = session_.onHello(h.bootId, p.bootNonce, rng_->nonZero(), &persist);
    if (sid == 0) {
      ++stats_.helloRefused;
      logf("[link] refused Hello from sensor boot %lu (highest granted %lu%s)", (unsigned long)h.bootId,
           (unsigned long)session_.persistedBoot(),
           h.bootId == session_.persistedBoot() ? ", boot id reused by a different boot" : "");
      // Tell the sensor which boot ids are used up (see vc_sensor_link.h).
      sendReject(h.bootId, 0, RejectReason::StaleBoot, session_.persistedBoot(), now);
      return;
    }
    // Any order of these two writes is safe: an interrupted update leaves a
    // (boot, nonce) pair that the sensor's next Hello resolves by advancing.
    if (persist && !(bootStore_ && bootStore_->store(h.bootId) && nonceStore_ &&
                     nonceStore_->store(p.bootNonce))) {
      // The session is still granted; after a gateway reboot an older Hello
      // could then be accepted again (availability only, see PROTOCOL.md).
      ++stats_.persistFailures;
      logf("[link] WARNING: could not persist sensor boot %lu", (unsigned long)h.bootId);
    }
    txSeq_ = 0;
    ++stats_.sessions;
    Packet ack;
    ack.header.type = MsgType::HelloAck;
    ack.nonce = p.nonce;
    reply(ack, h.bootId, sid, now);
    logf("[link] session %08lx for sensor boot %lu on channel %u (RSSI %d)", (unsigned long)sid,
         (unsigned long)h.bootId, (unsigned)channel, (int)rssi);
    return;
  }

  switch (session_.onData(h.bootId, h.sessionId, h.seq)) {
    case PeerSession::Verdict::Accept:
      break;
    case PeerSession::Verdict::Replay:
      ++stats_.replay;  // duplicate or too old: drop silently
      return;
    case PeerSession::Verdict::StaleBoot:
      ++stats_.sessionRejected;
      sendReject(h.bootId, h.sessionId, RejectReason::StaleBoot, h.seq, now);
      return;
    case PeerSession::Verdict::NoSession:
    case PeerSession::Verdict::WrongSession:
      ++stats_.sessionRejected;
      sendReject(h.bootId, h.sessionId, RejectReason::UnknownSession, h.seq, now);
      return;
  }

  ++stats_.accepted;
  lastRxMs_ = now;
  everSeen_ = true;
  if (h.type == MsgType::Telemetry) {
    agg_.onTelemetry(h, p.telemetry);
    return;
  }
  // Event: process once, acknowledge every copy.
  const EventBody& e = p.event;
  Packet ack;
  ack.header.type = MsgType::EventAck;
  ack.ackEventCounter = e.eventCounter;
  if (recentEvents_.contains(h.bootId, e.eventCounter)) {
    ++stats_.duplicateEvents;
    ack.ackResult = AckResult::Duplicate;
  } else if (sink_ && sink_->onSensorEvent(h, e, now)) {
    recentEvents_.insert(h.bootId, e.eventCounter);
    ack.ackResult = AckResult::Accepted;
  } else {
    ++stats_.eventsNotQueued;
    return;  // not queued: no ack, the sensor will retransmit
  }
  reply(ack, h.bootId, h.sessionId, now);
}

}  // namespace vc
