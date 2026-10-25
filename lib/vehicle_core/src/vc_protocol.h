// Local sensor-node <-> gateway protocol carried inside encrypted ESP-NOW frames.
//
// Fixed-layout little-endian binary packets: header | body | 16-byte tag.
// Every message type has exactly one valid length; anything shorter, longer,
// of an unknown type/version, sent in the wrong direction, with a wrong
// authentication tag, or with out-of-range fields is rejected by decode()
// before the application sees it.
//
// Tag (version 2):
//   K_dir = HKDF-SHA256(salt = "vehicle-iot/espnow/v2", IKM = LMK,
//                       info = "auth sensor->gateway" | "auth gateway->sensor", L = 32)
//   tag   = first 16 bytes of HMAC-SHA256(K_dir, srcMAC | dstMAC | header | body)
// The MAC addresses are not transmitted inside the payload; both ends use the
// configured addresses. ESP-NOW's CCMP encryption provides confidentiality on
// the air; the tag makes authenticity independent of whether the radio driver
// ever delivers an unencrypted frame that spoofs the peer's MAC address.
// See docs/PROTOCOL.md for the field-by-field layout.
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace vc {
namespace proto {

const uint8_t kMagic = 0x56;  // 'V'
const uint8_t kVersion = 2;
const size_t kHeaderSize = 20;
const size_t kTagSize = 16;
// ESP-NOW v1 payload limit; also the size of every receive buffer.
const size_t kMaxPacketSize = 250;

enum class MsgType : uint8_t {
  Hello = 1,          // sensor -> gateway: start a session
  HelloAck = 2,       // gateway -> sensor: session granted
  Telemetry = 3,      // sensor -> gateway: latest readings (~every 500 ms)
  Event = 4,          // sensor -> gateway: impact / tilt event (acknowledged)
  EventAck = 5,       // gateway -> sensor: event accepted
  SessionReject = 6,  // gateway -> sensor: session unknown, re-handshake
};

enum class Direction : uint8_t { SensorToGateway, GatewayToSensor };
// The only direction in which a message type is valid.
Direction directionOf(MsgType type);

const uint8_t kFlagSynthetic = 0x01;  // readings come from synthetic-sensor mode

enum class SensorStatus : uint8_t {
  Ok = 0,           // fresh measurement
  Stale = 1,        // last good value, older than the freshness limit
  Error = 2,        // last read failed and no usable cached value
  Absent = 3,       // enabled, but never detected / not responding
  Calibrating = 4,  // IMU calibrating, values not yet meaningful
  Disabled = 5,     // not fitted: switched off in the build configuration
};
const uint8_t kSensorStatusMax = 5;

inline bool statusHasValue(SensorStatus s) { return s == SensorStatus::Ok || s == SensorStatus::Stale; }

// Telemetry "features" bits: which binary sensors are enabled in the build.
const uint8_t kFeaturePir = 0x01;
const uint8_t kFeatureShock = 0x02;
const uint8_t kFeatureMask = kFeaturePir | kFeatureShock;

enum class EventType : uint8_t { Impact = 1, Tilt = 2 };
enum class EventState : uint8_t { Detected = 0, Active = 1, Cleared = 2 };

// StaleBoot is also sent in answer to a refused Hello; its value is then the
// lowest boot id the gateway will accept (see vc_sensor_link.h).
enum class RejectReason : uint8_t { UnknownSession = 1, StaleBoot = 2, Replay = 3 };
enum class AckResult : uint8_t { Accepted = 0, Duplicate = 1 };

struct Header {
  MsgType type = MsgType::Hello;
  uint8_t flags = 0;
  uint32_t bootId = 0;     // sensor boot counter (persisted, increments every boot)
  uint32_t sessionId = 0;  // gateway-issued session; 0 only in Hello (and a Hello's reject)
  uint32_t seq = 0;        // per-session, per-direction packet counter starting at 1
  uint32_t uptimeMs = 0;   // sender millis() at transmission
};

struct DhtReading {
  SensorStatus status = SensorStatus::Absent;
  float tempC = 0;
  float humidityPct = 0;
  uint32_t ageMs = 0;  // time since the value was measured
  uint16_t consecutiveErrors = 0;
};

struct ImuReading {
  SensorStatus status = SensorStatus::Absent;
  float accelXG = 0, accelYG = 0, accelZG = 0;
  float accelMagG = 0;   // |a| of the latest sample
  float peakAccelG = 0;  // max |a| since the previous telemetry packet
  float rollDeg = 0;     // complementary-filter angle about X
  float pitchDeg = 0;    // complementary-filter angle about Y
  float rateHz = 0;      // measured update rate over the last second
  uint16_t maxGapMs = 0; // largest gap between updates since the previous packet
  uint32_t ageMs = 0;
};

// Cumulative since sensor boot, so a lost packet never loses counts.
struct Counters {
  uint32_t pirRises = 0;
  uint32_t impacts = 0;
  uint32_t shockEdges = 0;
  uint32_t impactOverflows = 0;  // ISR ring full: impact records lost
  uint32_t eventsDropped = 0;    // retransmit queue full: oldest event dropped
  uint32_t eventsExpired = 0;    // event never acknowledged within retry budget
  uint16_t linkRestarts = 0;
};

struct TelemetryBody {
  DhtReading dht[2];
  ImuReading imu;
  bool pirActive = false;
  bool tiltActive = false;
  uint8_t features = kFeatureMask;  // kFeature* bits of enabled binary sensors
  Counters counters;
};

struct EventBody {
  uint32_t eventCounter = 0;  // with header.bootId forms the stable event identity
  EventType type = EventType::Impact;
  EventState state = EventState::Detected;
  uint8_t attempt = 0;  // 1 = first transmission
  uint32_t detectedUptimeMs = 0;
  float angleDeg = 0;     // tilt: max(|roll|,|pitch|) at transition; impact: 0
  float peakAccelG = 0;   // impact: max |a| near detection
  uint16_t pulseCount = 0;  // impact: shock edges inside the refractory window
};

struct Packet {
  Header header;
  uint32_t nonce = 0;      // Hello / HelloAck: fresh per Hello, echoed in the HelloAck
  uint32_t bootNonce = 0;  // Hello: random per sensor boot, constant across re-handshakes
  TelemetryBody telemetry;
  EventBody event;
  uint32_t ackEventCounter = 0;  // EventAck
  AckResult ackResult = AckResult::Accepted;
  RejectReason rejectReason = RejectReason::UnknownSession;  // SessionReject
  // SessionReject: the rejected seq, or (StaleBoot answering a Hello) the
  // lowest boot id the gateway accepts.
  uint32_t rejectValue = 0;
};

enum class DecodeError : uint8_t {
  Ok = 0,
  TooShort,
  TooLong,
  BadMagic,
  UnsupportedVersion,
  UnknownType,
  WrongDirection,
  BadLength,
  BadTag,
  BadField,
};

// Per-direction packet-authentication keys derived from the shared LMK.
struct LinkKeys {
  uint8_t sensorToGateway[32];
  uint8_t gatewayToSensor[32];
};
LinkKeys deriveLinkKeys(const uint8_t lmk[16]);

// Everything a packet's tag is bound to besides its own bytes.
struct AuthContext {
  Direction direction = Direction::SensorToGateway;
  uint8_t key[32] = {0};
  uint8_t srcMac[6] = {0};
  uint8_t dstMac[6] = {0};
};
AuthContext makeAuthContext(const LinkKeys& keys, Direction dir, const uint8_t sensorMac[6],
                            const uint8_t gatewayMac[6]);

const char* decodeErrorName(DecodeError e);

// Body length for a message type (excluding header and tag); 0 if unknown.
size_t bodySize(MsgType type);
// Total on-air length for a message type; 0 if unknown.
size_t packetSize(MsgType type);

// Writes header, body and tag. Returns the number of bytes written, or 0 if
// `cap` is too small, the type is unknown or not valid in ctx.direction.
size_t encode(const Packet& p, const AuthContext& ctx, uint8_t* out, size_t cap);

// Validates framing and direction, then the tag (before any field is
// interpreted), then fields. `out` is only written on success.
DecodeError decode(const uint8_t* data, size_t len, const AuthContext& ctx, Packet& out);

// Recomputes the tag of an already-framed packet in place (test helper).
void sign(uint8_t* packet, size_t len, const AuthContext& ctx);

}  // namespace proto
}  // namespace vc
