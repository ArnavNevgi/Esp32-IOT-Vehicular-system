#include "vc_protocol.h"

#include <math.h>
#include <string.h>

#include "vc_sha256.h"

namespace vc {
namespace proto {

namespace {

const int16_t kNoTemp = INT16_MIN;
const uint16_t kNoHumidity = 0xFFFF;

// Plausibility limits applied on both encode and decode.
const float kMinTempC = -40.0f, kMaxTempC = 85.0f;
const float kMaxAccelG = 16.0f;      // MPU6050 full-scale maximum
const float kMaxAccelMagG = 27.72f;  // sqrt(3) * 16 g
const float kMaxAngleDeg = 180.0f;
const float kMaxRateHz = 1000.0f;

const size_t kDhtSize = 1 + 2 + 2 + 4 + 2;
const size_t kImuSize = 1 + 2 * 3 + 2 + 2 + 2 * 2 + 2 + 2 + 4;
const size_t kCountersSize = 4 * 6 + 2;
const size_t kTelemetrySize = 2 * kDhtSize + kImuSize + 1 + 1 + 1 + kCountersSize;
const size_t kEventSize = 4 + 1 + 1 + 1 + 4 + 2 + 2 + 2;

class Writer {
 public:
  Writer(uint8_t* buf, size_t cap) : buf_(buf), cap_(cap) {}
  void u8(uint8_t v) {
    if (pos_ + 1 > cap_) {
      ok_ = false;
      return;
    }
    buf_[pos_++] = v;
  }
  void u16(uint16_t v) {
    u8((uint8_t)v);
    u8((uint8_t)(v >> 8));
  }
  void i16(int16_t v) { u16((uint16_t)v); }
  void u32(uint32_t v) {
    u16((uint16_t)v);
    u16((uint16_t)(v >> 16));
  }
  bool ok() const { return ok_; }
  size_t pos() const { return pos_; }

 private:
  uint8_t* buf_;
  size_t cap_;
  size_t pos_ = 0;
  bool ok_ = true;
};

class Reader {
 public:
  Reader(const uint8_t* buf, size_t len) : buf_(buf), len_(len) {}
  uint8_t u8() {
    if (pos_ + 1 > len_) {
      ok_ = false;
      return 0;
    }
    return buf_[pos_++];
  }
  uint16_t u16() {
    uint16_t lo = u8();
    return (uint16_t)(lo | ((uint16_t)u8() << 8));
  }
  int16_t i16() { return (int16_t)u16(); }
  uint32_t u32() {
    uint32_t lo = u16();
    return lo | ((uint32_t)u16() << 16);
  }
  bool ok() const { return ok_; }

 private:
  const uint8_t* buf_;
  size_t len_;
  size_t pos_ = 0;
  bool ok_ = true;
};

int16_t scaledI16(float v, float scale, float limit) {
  if (!isfinite(v)) v = 0;
  if (v > limit) v = limit;
  if (v < -limit) v = -limit;
  return (int16_t)lroundf(v * scale);
}

uint16_t scaledU16(float v, float scale, float limit) {
  if (!isfinite(v) || v < 0) v = 0;
  if (v > limit) v = limit;
  return (uint16_t)lroundf(v * scale);
}

bool dhtValuesPlausible(float t, float h) {
  return isfinite(t) && isfinite(h) && t >= kMinTempC && t <= kMaxTempC && h >= 0 && h <= 100.0f;
}

void writeDht(Writer& w, const DhtReading& d) {
  SensorStatus s = d.status;
  if (statusHasValue(s) && !dhtValuesPlausible(d.tempC, d.humidityPct)) s = SensorStatus::Error;
  w.u8((uint8_t)s);
  if (statusHasValue(s)) {
    w.i16((int16_t)lroundf(d.tempC * 100.0f));
    w.u16((uint16_t)lroundf(d.humidityPct * 100.0f));
  } else {
    w.i16(kNoTemp);
    w.u16(kNoHumidity);
  }
  bool disabled = s == SensorStatus::Disabled;
  w.u32(disabled ? 0 : d.ageMs);
  w.u16(disabled ? 0 : d.consecutiveErrors);
}

bool readDht(Reader& r, DhtReading& d) {
  uint8_t s = r.u8();
  int16_t t = r.i16();
  uint16_t h = r.u16();
  d.ageMs = r.u32();
  d.consecutiveErrors = r.u16();
  if (s > kSensorStatusMax || s == (uint8_t)SensorStatus::Calibrating) return false;
  d.status = (SensorStatus)s;
  if (statusHasValue(d.status)) {
    if (t == kNoTemp || h == kNoHumidity) return false;
    d.tempC = t / 100.0f;
    d.humidityPct = h / 100.0f;
    if (!dhtValuesPlausible(d.tempC, d.humidityPct)) return false;
  } else {
    if (t != kNoTemp || h != kNoHumidity) return false;
    if (d.status == SensorStatus::Disabled && (d.ageMs || d.consecutiveErrors)) return false;
    d.tempC = NAN;
    d.humidityPct = NAN;
  }
  return true;
}

void writeImu(Writer& w, const ImuReading& m) {
  SensorStatus s = m.status;
  bool hasValue = statusHasValue(s);
  w.u8((uint8_t)s);
  w.i16(hasValue ? scaledI16(m.accelXG, 1000.0f, kMaxAccelG) : 0);
  w.i16(hasValue ? scaledI16(m.accelYG, 1000.0f, kMaxAccelG) : 0);
  w.i16(hasValue ? scaledI16(m.accelZG, 1000.0f, kMaxAccelG) : 0);
  w.u16(hasValue ? scaledU16(m.accelMagG, 1000.0f, kMaxAccelMagG) : 0);
  w.u16(hasValue ? scaledU16(m.peakAccelG, 1000.0f, kMaxAccelMagG) : 0);
  w.i16(hasValue ? scaledI16(m.rollDeg, 100.0f, kMaxAngleDeg) : 0);
  w.i16(hasValue ? scaledI16(m.pitchDeg, 100.0f, kMaxAngleDeg) : 0);
  bool disabled = s == SensorStatus::Disabled;
  w.u16(disabled ? 0 : scaledU16(m.rateHz, 10.0f, kMaxRateHz));
  w.u16(disabled ? 0 : m.maxGapMs);
  w.u32(disabled ? 0 : m.ageMs);
}

bool readImu(Reader& r, ImuReading& m) {
  uint8_t s = r.u8();
  int16_t ax = r.i16(), ay = r.i16(), az = r.i16();
  uint16_t mag = r.u16(), peak = r.u16();
  int16_t roll = r.i16(), pitch = r.i16();
  uint16_t rate = r.u16();
  m.maxGapMs = r.u16();
  m.ageMs = r.u32();
  if (s > kSensorStatusMax) return false;
  m.status = (SensorStatus)s;
  const int16_t accelLimit = 16000, angleLimit = 18000;
  if (ax > accelLimit || ax < -accelLimit || ay > accelLimit || ay < -accelLimit || az > accelLimit ||
      az < -accelLimit)
    return false;
  if (mag > 27720 || peak > 27720) return false;
  if (roll > angleLimit || roll < -angleLimit || pitch > angleLimit || pitch < -angleLimit) return false;
  if (rate > 10000) return false;
  m.rateHz = rate / 10.0f;
  // Readings without a value carry zeros (what writeImu() produces).
  if (!statusHasValue(m.status) && (ax || ay || az || mag || peak || roll || pitch)) return false;
  // A disabled IMU never runs, so it has no update rate either.
  if (m.status == SensorStatus::Disabled && (rate || m.maxGapMs || m.ageMs)) return false;
  if (statusHasValue(m.status)) {
    m.accelXG = ax / 1000.0f;
    m.accelYG = ay / 1000.0f;
    m.accelZG = az / 1000.0f;
    m.accelMagG = mag / 1000.0f;
    m.peakAccelG = peak / 1000.0f;
    m.rollDeg = roll / 100.0f;
    m.pitchDeg = pitch / 100.0f;
  } else {
    m.accelXG = m.accelYG = m.accelZG = m.accelMagG = m.peakAccelG = m.rollDeg = m.pitchDeg = NAN;
  }
  return true;
}

bool readBool(Reader& r, bool& out) {
  uint8_t v = r.u8();
  out = v != 0;
  return v <= 1;
}

}  // namespace

const char* decodeErrorName(DecodeError e) {
  switch (e) {
    case DecodeError::Ok: return "ok";
    case DecodeError::TooShort: return "too_short";
    case DecodeError::TooLong: return "too_long";
    case DecodeError::BadMagic: return "bad_magic";
    case DecodeError::UnsupportedVersion: return "unsupported_version";
    case DecodeError::UnknownType: return "unknown_type";
    case DecodeError::WrongDirection: return "wrong_direction";
    case DecodeError::BadLength: return "bad_length";
    case DecodeError::BadTag: return "bad_tag";
    case DecodeError::BadField: return "bad_field";
  }
  return "unknown";
}

size_t bodySize(MsgType type) {
  switch (type) {
    case MsgType::Hello: return 8;
    case MsgType::HelloAck: return 4;
    case MsgType::Telemetry: return kTelemetrySize;
    case MsgType::Event: return kEventSize;
    case MsgType::EventAck: return 5;
    case MsgType::SessionReject: return 5;
  }
  return 0;
}

Direction directionOf(MsgType type) {
  switch (type) {
    case MsgType::HelloAck:
    case MsgType::EventAck:
    case MsgType::SessionReject:
      return Direction::GatewayToSensor;
    default:
      return Direction::SensorToGateway;
  }
}

LinkKeys deriveLinkKeys(const uint8_t lmk[16]) {
  static const char kSalt[] = "vehicle-iot/espnow/v2";
  static const char kInfoS2G[] = "auth sensor->gateway";
  static const char kInfoG2S[] = "auth gateway->sensor";
  LinkKeys k;
  hkdfSha256((const uint8_t*)kSalt, sizeof(kSalt) - 1, lmk, 16, (const uint8_t*)kInfoS2G, sizeof(kInfoS2G) - 1,
             k.sensorToGateway, sizeof(k.sensorToGateway));
  hkdfSha256((const uint8_t*)kSalt, sizeof(kSalt) - 1, lmk, 16, (const uint8_t*)kInfoG2S, sizeof(kInfoG2S) - 1,
             k.gatewayToSensor, sizeof(k.gatewayToSensor));
  return k;
}

AuthContext makeAuthContext(const LinkKeys& keys, Direction dir, const uint8_t sensorMac[6],
                            const uint8_t gatewayMac[6]) {
  AuthContext c;
  c.direction = dir;
  bool s2g = dir == Direction::SensorToGateway;
  memcpy(c.key, s2g ? keys.sensorToGateway : keys.gatewayToSensor, sizeof(c.key));
  memcpy(c.srcMac, s2g ? sensorMac : gatewayMac, 6);
  memcpy(c.dstMac, s2g ? gatewayMac : sensorMac, 6);
  return c;
}

size_t packetSize(MsgType type) {
  size_t body = bodySize(type);
  return body ? kHeaderSize + body + kTagSize : 0;
}

namespace {
// HMAC-SHA256(K_dir, srcMAC | dstMAC | data), truncated to kTagSize bytes.
void computeTag(const uint8_t* data, size_t len, const AuthContext& ctx, uint8_t tag[kTagSize]) {
  uint8_t msg[12 + kMaxPacketSize];
  memcpy(msg, ctx.srcMac, 6);
  memcpy(msg + 6, ctx.dstMac, 6);
  memcpy(msg + 12, data, len);
  uint8_t mac[32];
  hmacSha256(ctx.key, sizeof(ctx.key), msg, 12 + len, mac);
  memcpy(tag, mac, kTagSize);
}
}  // namespace

void sign(uint8_t* packet, size_t len, const AuthContext& ctx) {
  if (len < kHeaderSize + kTagSize || len > kMaxPacketSize) return;
  computeTag(packet, len - kTagSize, ctx, packet + len - kTagSize);
}

size_t encode(const Packet& p, const AuthContext& ctx, uint8_t* out, size_t cap) {
  size_t body = bodySize(p.header.type);
  size_t total = packetSize(p.header.type);
  if (body == 0 || total > kMaxPacketSize || total > cap || out == nullptr) return 0;
  if (directionOf(p.header.type) != ctx.direction) return 0;
  Writer w(out, cap);
  w.u8(kMagic);
  w.u8(kVersion);
  w.u8((uint8_t)p.header.type);
  w.u8(p.header.flags);
  w.u32(p.header.bootId);
  w.u32(p.header.sessionId);
  w.u32(p.header.seq);
  w.u32(p.header.uptimeMs);

  switch (p.header.type) {
    case MsgType::Hello:
      w.u32(p.nonce);
      w.u32(p.bootNonce);
      break;
    case MsgType::HelloAck:
      w.u32(p.nonce);
      break;
    case MsgType::Telemetry: {
      const TelemetryBody& t = p.telemetry;
      writeDht(w, t.dht[0]);
      writeDht(w, t.dht[1]);
      writeImu(w, t.imu);
      w.u8(t.pirActive ? 1 : 0);
      w.u8(t.tiltActive ? 1 : 0);
      w.u8(t.features & kFeatureMask);
      w.u32(t.counters.pirRises);
      w.u32(t.counters.impacts);
      w.u32(t.counters.shockEdges);
      w.u32(t.counters.impactOverflows);
      w.u32(t.counters.eventsDropped);
      w.u32(t.counters.eventsExpired);
      w.u16(t.counters.linkRestarts);
      break;
    }
    case MsgType::Event: {
      const EventBody& e = p.event;
      w.u32(e.eventCounter);
      w.u8((uint8_t)e.type);
      w.u8((uint8_t)e.state);
      w.u8(e.attempt);
      w.u32(e.detectedUptimeMs);
      w.i16(scaledI16(e.angleDeg, 100.0f, kMaxAngleDeg));
      w.u16(scaledU16(e.peakAccelG, 1000.0f, kMaxAccelMagG));
      w.u16(e.pulseCount);
      break;
    }
    case MsgType::EventAck:
      w.u32(p.ackEventCounter);
      w.u8((uint8_t)p.ackResult);
      break;
    case MsgType::SessionReject:
      w.u8((uint8_t)p.rejectReason);
      w.u32(p.rejectValue);
      break;
  }
  if (!w.ok() || w.pos() != kHeaderSize + body) return 0;
  computeTag(out, kHeaderSize + body, ctx, out + kHeaderSize + body);
  return total;
}

DecodeError decode(const uint8_t* data, size_t len, const AuthContext& ctx, Packet& out) {
  if (data == nullptr || len < kHeaderSize + kTagSize) return DecodeError::TooShort;
  if (len > kMaxPacketSize) return DecodeError::TooLong;
  Reader r(data, len);
  if (r.u8() != kMagic) return DecodeError::BadMagic;
  if (r.u8() != kVersion) return DecodeError::UnsupportedVersion;
  uint8_t type = r.u8();
  size_t body = bodySize((MsgType)type);
  if (body == 0) return DecodeError::UnknownType;
  if (directionOf((MsgType)type) != ctx.direction) return DecodeError::WrongDirection;
  if (len != kHeaderSize + body + kTagSize) return DecodeError::BadLength;
  uint8_t tag[kTagSize];
  computeTag(data, kHeaderSize + body, ctx, tag);
  if (!constantTimeEqual(tag, data + kHeaderSize + body, kTagSize)) return DecodeError::BadTag;

  Packet p;
  p.header.type = (MsgType)type;
  p.header.flags = r.u8();
  p.header.bootId = r.u32();
  p.header.sessionId = r.u32();
  p.header.seq = r.u32();
  p.header.uptimeMs = r.u32();
  if ((p.header.flags & ~kFlagSynthetic) != 0) return DecodeError::BadField;
  if (p.header.bootId == 0) return DecodeError::BadField;

  bool ok = true;
  switch (p.header.type) {
    case MsgType::Hello:
    case MsgType::HelloAck:
      p.nonce = r.u32();
      ok = p.nonce != 0;
      if (p.header.type == MsgType::Hello) {
        p.bootNonce = r.u32();
        ok = ok && p.bootNonce != 0 && p.header.sessionId == 0;
      } else {
        ok = ok && p.header.sessionId != 0;
      }
      break;
    case MsgType::Telemetry: {
      TelemetryBody& t = p.telemetry;
      ok = readDht(r, t.dht[0]) && readDht(r, t.dht[1]) && readImu(r, t.imu) && readBool(r, t.pirActive) &&
           readBool(r, t.tiltActive);
      t.features = r.u8();
      ok = ok && (t.features & ~kFeatureMask) == 0;
      // A disabled sensor cannot report activity.
      ok = ok && ((t.features & kFeaturePir) || !t.pirActive);
      ok = ok && (t.imu.status != SensorStatus::Disabled || !t.tiltActive);
      t.counters.pirRises = r.u32();
      t.counters.impacts = r.u32();
      t.counters.shockEdges = r.u32();
      t.counters.impactOverflows = r.u32();
      t.counters.eventsDropped = r.u32();
      t.counters.eventsExpired = r.u32();
      t.counters.linkRestarts = r.u16();
      break;
    }
    case MsgType::Event: {
      EventBody& e = p.event;
      e.eventCounter = r.u32();
      uint8_t et = r.u8();
      uint8_t es = r.u8();
      e.attempt = r.u8();
      e.detectedUptimeMs = r.u32();
      int16_t angle = r.i16();
      uint16_t peak = r.u16();
      e.pulseCount = r.u16();
      ok = e.eventCounter != 0 && e.attempt != 0 && angle >= -18000 && angle <= 18000 && peak <= 27720;
      if (et == (uint8_t)EventType::Impact) ok = ok && es == (uint8_t)EventState::Detected;
      else if (et == (uint8_t)EventType::Tilt)
        ok = ok && (es == (uint8_t)EventState::Active || es == (uint8_t)EventState::Cleared);
      else ok = false;
      e.type = (EventType)et;
      e.state = (EventState)es;
      e.angleDeg = angle / 100.0f;
      e.peakAccelG = peak / 1000.0f;
      break;
    }
    case MsgType::EventAck: {
      p.ackEventCounter = r.u32();
      uint8_t res = r.u8();
      ok = p.ackEventCounter != 0 && res <= (uint8_t)AckResult::Duplicate;
      p.ackResult = (AckResult)res;
      break;
    }
    case MsgType::SessionReject: {
      uint8_t reason = r.u8();
      p.rejectValue = r.u32();
      ok = reason >= (uint8_t)RejectReason::UnknownSession && reason <= (uint8_t)RejectReason::Replay;
      p.rejectReason = (RejectReason)reason;
      break;
    }
  }
  // Every non-Hello data message must carry a session and a sequence number.
  if (p.header.type != MsgType::Hello && p.header.type != MsgType::HelloAck && p.header.type != MsgType::SessionReject)
    ok = ok && p.header.sessionId != 0 && p.header.seq != 0;
  if (!r.ok()) return DecodeError::BadLength;
  if (!ok) return DecodeError::BadField;
  out = p;
  return DecodeError::Ok;
}

}  // namespace proto
}  // namespace vc
