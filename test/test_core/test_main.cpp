// Host unit tests for lib/vehicle_core. Run with: pio test -e native
#include <ArduinoJson.h>
#include <stdio.h>
#include <string.h>
#include <unity.h>

#include "vc_aggregate.h"
#include "vc_alarm.h"
#include "vc_cloud_link.h"
#include "vc_sensor_link.h"
#include "vc_backoff.h"
#include "vc_cloud_queue.h"
#include "vc_event_queue.h"
#include "vc_protocol.h"
#include "vc_session.h"
#include "vc_sha256.h"
#include "vc_telemetry.h"
#include "vc_timing.h"
#include "vc_util.h"

using namespace vc;
using namespace vc::proto;

static const uint8_t kLmk[16] = {0x10, 0x32, 0x54, 0x76, 0x98, 0xba, 0xdc, 0xfe,
                                 0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef};
static const LinkKeys kKeys = deriveLinkKeys(kLmk);
static const uint8_t kSensorMac[6] = {0x24, 0x6F, 0x28, 0x00, 0x00, 0x01};
static const uint8_t kGatewayMac[6] = {0x24, 0x6F, 0x28, 0x00, 0x00, 0x02};
static const AuthContext kKey = makeAuthContext(kKeys, Direction::SensorToGateway, kSensorMac, kGatewayMac);
static const AuthContext kG2S = makeAuthContext(kKeys, Direction::GatewayToSensor, kSensorMac, kGatewayMac);

void setUp() {}
void tearDown() {}

// ---------------------------------------------------------------- timing ----

static void test_elapsed_across_millis_wrap() {
  uint32_t since = 0xFFFFFF00u;
  TEST_ASSERT_FALSE(hasElapsed(0xFFFFFFF0u, since, 0x100));
  TEST_ASSERT_TRUE(hasElapsed(0x00000000u, since, 0x100));
  TEST_ASSERT_TRUE(hasElapsed(0x00000010u, since, 0x100));
  TEST_ASSERT_EQUAL_UINT32(0x110, elapsedMs(0x10, since));
}

static void test_periodic_timer_across_wrap_and_no_burst() {
  PeriodicTimer t(500);
  uint32_t now = 0xFFFFFE00u;  // 512 ms before wrap
  t.start(now);
  int fired = 0;
  for (uint32_t step = 0; step < 2000; step += 10) {
    if (t.due(now + step)) ++fired;
  }
  TEST_ASSERT_EQUAL(3, fired);  // at +500, +1000, +1500 despite wrapping at +512
  // A 10 s stall produces one tick, not twenty.
  uint32_t later = now + 2000 + 10000;
  TEST_ASSERT_TRUE(t.due(later));
  TEST_ASSERT_FALSE(t.due(later + 1));
  TEST_ASSERT_TRUE(t.skipped() > 0);
}

static void test_routine_report_interval_is_configured_period() {
  PeriodicTimer report(60000);
  report.start(1000);
  uint32_t fires[3];
  int n = 0;
  for (uint32_t ms = 1000; ms <= 1000 + 180000 && n < 3; ms += 100)
    if (report.due(ms)) fires[n++] = ms;
  TEST_ASSERT_EQUAL(3, n);
  TEST_ASSERT_EQUAL_UINT32(61000, fires[0]);
  TEST_ASSERT_EQUAL_UINT32(121000, fires[1]);
  TEST_ASSERT_EQUAL_UINT32(181000, fires[2]);
}

static void test_backoff_grows_caps_and_jitters() {
  Backoff b(1000, 30000, 25);
  uint32_t d0 = b.next(0);
  TEST_ASSERT_EQUAL_UINT32(1000, d0);
  TEST_ASSERT_EQUAL_UINT32(2000, b.next(0));
  TEST_ASSERT_EQUAL_UINT32(4000, b.next(0));
  for (int i = 0; i < 20; ++i) b.next(0);
  TEST_ASSERT_EQUAL_UINT32(30000, b.next(0));
  // Jitter removes at most 25%.
  for (uint32_t r = 0; r < 100000; r += 7919) {
    uint32_t d = b.next(r);
    TEST_ASSERT_TRUE(d <= 30000 && d >= 22500);
  }
  b.reset();
  TEST_ASSERT_EQUAL_UINT32(1000, b.next(0));
}

static void test_retry_timer_wrap_safe() {
  RetryTimer rt(1000, 8000, 0);
  uint32_t now = 0xFFFFFC00u;
  TEST_ASSERT_TRUE(rt.ready(now));
  rt.failed(now, 0);
  TEST_ASSERT_FALSE(rt.ready(now + 999));
  TEST_ASSERT_TRUE(rt.ready(now + 1000));  // crosses the wrap
  rt.failed(now + 1000, 0);
  TEST_ASSERT_FALSE(rt.ready(now + 2999));
  TEST_ASSERT_TRUE(rt.ready(now + 3000));
  rt.succeeded();
  TEST_ASSERT_TRUE(rt.ready(now + 3000));
}

static void test_sas_refresh_without_waiting_an_hour() {
  SasRefreshSchedule s;
  TEST_ASSERT_TRUE(s.due(0));  // no token yet
  uint32_t gen = 0xFFF00000u;  // near the millis wrap
  s.onGenerated(gen, 3600, 300);
  TEST_ASSERT_FALSE(s.due(gen + 3299u * 1000u));
  TEST_ASSERT_TRUE(s.due(gen + 3300u * 1000u));
  TEST_ASSERT_EQUAL_UINT32(1000, s.remainingMs(gen + 3299u * 1000u));
  // Margin larger than lifetime falls back to half the lifetime.
  s.onGenerated(0, 60, 120);
  TEST_ASSERT_FALSE(s.due(29999));
  TEST_ASSERT_TRUE(s.due(30000));
  s.invalidate();
  TEST_ASSERT_TRUE(s.due(1));
}

// ---------------------------------------------------------------- alarms ----

static void test_hysteresis_alarm_debounce_and_clear() {
  HysteresisAlarm a(45.0f, 35.0f, 500, 1000);
  TEST_ASSERT_EQUAL(AlarmTransition::None, a.update(50, true, 0));
  TEST_ASSERT_EQUAL(AlarmTransition::None, a.update(50, true, 499));
  TEST_ASSERT_EQUAL(AlarmTransition::Activated, a.update(52, true, 500));
  TEST_ASSERT_TRUE(a.active());
  // Between thresholds: stays active.
  TEST_ASSERT_EQUAL(AlarmTransition::None, a.update(40, true, 600));
  TEST_ASSERT_EQUAL(AlarmTransition::None, a.update(60, true, 700));
  TEST_ASSERT_EQUAL_FLOAT(60, a.peak());
  // Below off threshold must persist 1000 ms.
  TEST_ASSERT_EQUAL(AlarmTransition::None, a.update(30, true, 800));
  TEST_ASSERT_EQUAL(AlarmTransition::None, a.update(40, true, 1500));  // bounce cancels
  TEST_ASSERT_EQUAL(AlarmTransition::None, a.update(30, true, 1600));
  TEST_ASSERT_EQUAL(AlarmTransition::Cleared, a.update(30, true, 2600));
  TEST_ASSERT_EQUAL_FLOAT(60, a.peak());  // peak of the finished active period
}

static void test_hysteresis_alarm_ignores_invalid_samples() {
  HysteresisAlarm a(45.0f, 35.0f, 100, 100);
  a.update(50, true, 0);
  TEST_ASSERT_EQUAL(AlarmTransition::None, a.update(NAN, true, 200));
  TEST_ASSERT_EQUAL(AlarmTransition::None, a.update(50, false, 300));
  TEST_ASSERT_FALSE(a.active());
  TEST_ASSERT_EQUAL(AlarmTransition::None, a.update(50, true, 400));
  TEST_ASSERT_EQUAL(AlarmTransition::Activated, a.update(50, true, 500));
}

static void test_reminders_are_rate_limited() {
  ReminderTimer r(600000);
  TEST_ASSERT_FALSE(r.due(0));
  r.arm(0);
  int n = 0;
  for (uint32_t t = 0; t <= 1800000; t += 1000) n += r.due(t) ? 1 : 0;
  TEST_ASSERT_EQUAL(3, n);
  r.disarm();
  TEST_ASSERT_FALSE(r.due(5000000));
}

// -------------------------------------------------------------- protocol ----

static Packet sampleTelemetry() {
  Packet p;
  p.header.type = MsgType::Telemetry;
  p.header.bootId = 7;
  p.header.sessionId = 0xDEADBEEF;
  p.header.seq = 42;
  p.header.uptimeMs = 123456;
  TelemetryBody& t = p.telemetry;
  t.dht[0].status = SensorStatus::Ok;
  t.dht[0].tempC = 24.56f;
  t.dht[0].humidityPct = 51.2f;
  t.dht[0].ageMs = 1200;
  t.dht[1].status = SensorStatus::Error;
  t.dht[1].tempC = NAN;
  t.dht[1].humidityPct = NAN;
  t.dht[1].consecutiveErrors = 3;
  t.imu.status = SensorStatus::Ok;
  t.imu.accelXG = 0.01f;
  t.imu.accelYG = -0.02f;
  t.imu.accelZG = 1.0f;
  t.imu.accelMagG = 1.0f;
  t.imu.peakAccelG = 1.31f;
  t.imu.rollDeg = -3.25f;
  t.imu.pitchDeg = 12.5f;
  t.imu.rateHz = 99.7f;
  t.imu.maxGapMs = 14;
  t.pirActive = true;
  t.counters.pirRises = 5;
  t.counters.impacts = 2;
  t.counters.shockEdges = 9;
  return p;
}

static void test_protocol_roundtrip_and_size() {
  Packet p = sampleTelemetry();
  uint8_t buf[kMaxPacketSize];
  size_t n = encode(p, kKey, buf, sizeof(buf));
  TEST_ASSERT_TRUE(n > kHeaderSize);
  TEST_ASSERT_TRUE(n <= kMaxPacketSize);
  printf("[size] telemetry packet: %u bytes\n", (unsigned)n);
  Packet q;
  TEST_ASSERT_EQUAL(DecodeError::Ok, decode(buf, n, kKey, q));
  TEST_ASSERT_EQUAL_UINT32(42, q.header.seq);
  TEST_ASSERT_EQUAL(SensorStatus::Ok, q.telemetry.dht[0].status);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 24.56f, q.telemetry.dht[0].tempC);
  TEST_ASSERT_EQUAL(SensorStatus::Error, q.telemetry.dht[1].status);
  TEST_ASSERT_TRUE(isnan(q.telemetry.dht[1].tempC));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 12.5f, q.telemetry.imu.pitchDeg);
  TEST_ASSERT_TRUE(q.telemetry.pirActive);
  TEST_ASSERT_EQUAL_UINT32(9, q.telemetry.counters.shockEdges);
}

static void test_protocol_nonfinite_ok_reading_is_downgraded() {
  Packet p = sampleTelemetry();
  p.telemetry.dht[0].tempC = NAN;  // claims Ok but has no value
  uint8_t buf[kMaxPacketSize];
  size_t n = encode(p, kKey, buf, sizeof(buf));
  Packet q;
  TEST_ASSERT_EQUAL(DecodeError::Ok, decode(buf, n, kKey, q));
  TEST_ASSERT_EQUAL(SensorStatus::Error, q.telemetry.dht[0].status);
}

static void test_protocol_rejects_bad_framing() {
  Packet p = sampleTelemetry();
  uint8_t buf[kMaxPacketSize + 8];
  size_t n = encode(p, kKey, buf, sizeof(buf));
  Packet q;
  TEST_ASSERT_EQUAL(DecodeError::TooShort, decode(buf, 0, kKey, q));
  TEST_ASSERT_EQUAL(DecodeError::TooShort, decode(buf, kHeaderSize - 1, kKey, q));
  TEST_ASSERT_EQUAL(DecodeError::TooShort, decode(nullptr, 10, kKey, q));
  TEST_ASSERT_EQUAL(DecodeError::TooLong, decode(buf, kMaxPacketSize + 1, kKey, q));
  TEST_ASSERT_EQUAL(DecodeError::BadLength, decode(buf, n - 1, kKey, q));  // truncated
  buf[n] = 0;
  TEST_ASSERT_EQUAL(DecodeError::BadLength, decode(buf, n + 1, kKey, q));  // trailing byte

  uint8_t bad[kMaxPacketSize];
  memcpy(bad, buf, n);
  bad[0] = 0x00;
  TEST_ASSERT_EQUAL(DecodeError::BadMagic, decode(bad, n, kKey, q));
  memcpy(bad, buf, n);
  bad[1] = 1;  // protocol v1 (single key, no MAC binding) is no longer accepted
  TEST_ASSERT_EQUAL(DecodeError::UnsupportedVersion, decode(bad, n, kKey, q));
  memcpy(bad, buf, n);
  bad[2] = 99;
  TEST_ASSERT_EQUAL(DecodeError::UnknownType, decode(bad, n, kKey, q));
}

static void test_protocol_rejects_bad_fields() {
  Packet p = sampleTelemetry();
  uint8_t buf[kMaxPacketSize];
  size_t n = encode(p, kKey, buf, sizeof(buf));
  Packet q;
  uint8_t bad[kMaxPacketSize];

  memcpy(bad, buf, n);
  bad[3] = 0x80;  // unknown flag
  sign(bad, n, kKey);
  TEST_ASSERT_EQUAL(DecodeError::BadField, decode(bad, n, kKey, q));

  memcpy(bad, buf, n);
  bad[kHeaderSize] = 9;  // DHT status out of range
  sign(bad, n, kKey);
  TEST_ASSERT_EQUAL(DecodeError::BadField, decode(bad, n, kKey, q));

  memcpy(bad, buf, n);
  bad[kHeaderSize + 3] = 0x10;  // humidity 0x10xx/100 > 100 %
  bad[kHeaderSize + 4] = 0x30;
  sign(bad, n, kKey);
  TEST_ASSERT_EQUAL(DecodeError::BadField, decode(bad, n, kKey, q));

  memcpy(bad, buf, n);
  memset(bad + 8, 0, 4);  // sessionId = 0 on a data packet
  sign(bad, n, kKey);
  TEST_ASSERT_EQUAL(DecodeError::BadField, decode(bad, n, kKey, q));

  // Impact events are instantaneous: "active" is not a valid state.
  Packet e;
  e.header.type = MsgType::Event;
  e.header.bootId = 1;
  e.header.sessionId = 5;
  e.header.seq = 1;
  e.event.eventCounter = 1;
  e.event.attempt = 1;
  e.event.type = EventType::Impact;
  e.event.state = EventState::Active;
  n = encode(e, kKey, buf, sizeof(buf));
  TEST_ASSERT_EQUAL(DecodeError::BadField, decode(buf, n, kKey, q));
  e.event.state = EventState::Detected;
  n = encode(e, kKey, buf, sizeof(buf));
  TEST_ASSERT_EQUAL(DecodeError::Ok, decode(buf, n, kKey, q));

  // Hello must not carry a session; HelloAck must.
  Packet h;
  h.header.type = MsgType::Hello;
  h.header.bootId = 3;
  h.nonce = 77;
  h.bootNonce = 78;
  n = encode(h, kKey, buf, sizeof(buf));
  TEST_ASSERT_EQUAL(DecodeError::Ok, decode(buf, n, kKey, q));
  h.header.sessionId = 9;
  n = encode(h, kKey, buf, sizeof(buf));
  TEST_ASSERT_EQUAL(DecodeError::BadField, decode(buf, n, kKey, q));
}

static void test_protocol_every_type_fits_espnow() {
  const MsgType types[] = {MsgType::Hello,    MsgType::HelloAck, MsgType::Telemetry,
                           MsgType::Event,    MsgType::EventAck, MsgType::SessionReject};
  for (unsigned i = 0; i < sizeof(types) / sizeof(types[0]); ++i)
    TEST_ASSERT_TRUE(packetSize(types[i]) <= kMaxPacketSize);
  uint8_t small[10];
  Packet p = sampleTelemetry();
  TEST_ASSERT_EQUAL_UINT32(0, encode(p, kKey, small, sizeof(small)));
}

static void hexToBytes(const char* hex, uint8_t* out, size_t n) {
  for (size_t i = 0; i < n; ++i) {
    uint8_t hi = 0, lo = 0;
    hexNibble(hex[2 * i], hi);
    hexNibble(hex[2 * i + 1], lo);
    out[i] = (uint8_t)((hi << 4) | lo);
  }
}

static void assertDigest(const char* expectHex, const uint8_t* got, size_t n) {
  uint8_t expect[64];
  hexToBytes(expectHex, expect, n);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(expect, got, n);
}

// Published known-answer vectors (FIPS 180-2 examples, RFC 4231, RFC 5869),
// not values produced by this implementation.
static void test_sha256_and_hmac_vectors() {
  uint8_t out[32];
  Sha256 sh;
  sh.update((const uint8_t*)"abc", 3);
  sh.finish(out);
  assertDigest("ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad", out, 32);
  sh.reset();
  sh.finish(out);
  assertDigest("e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855", out, 32);
  const char* m448 = "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq";  // padding spills into a 2nd block
  sh.reset();
  sh.update((const uint8_t*)m448, 20);
  sh.update((const uint8_t*)m448 + 20, strlen(m448) - 20);
  sh.finish(out);
  assertDigest("248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1", out, 32);
  // RFC 4231 test case 2.
  const char* msg = "what do ya want for nothing?";
  hmacSha256((const uint8_t*)"Jefe", 4, (const uint8_t*)msg, strlen(msg), out);
  assertDigest("5bdcc146bf60754e6a042426089575c75a003f089d2739839dec58b964ec3843", out, 32);
  // RFC 4231 test case 6: a key longer than the block size is hashed first.
  uint8_t key[131];
  memset(key, 0xaa, sizeof(key));
  const char* m6 = "Test Using Larger Than Block-Size Key - Hash Key First";
  hmacSha256(key, sizeof(key), (const uint8_t*)m6, strlen(m6), out);
  assertDigest("60e431591ee0b67f0d8a26aacbf5b77f8e0bc6213728c5140546040f0ee37f54", out, 32);
}

static void test_hkdf_rfc5869_vectors() {
  uint8_t ikm[22];
  memset(ikm, 0x0b, sizeof(ikm));
  uint8_t salt[13], info[10], okm[42];
  for (int i = 0; i < 13; ++i) salt[i] = (uint8_t)i;
  for (int i = 0; i < 10; ++i) info[i] = (uint8_t)(0xf0 + i);
  // Test case 1.
  TEST_ASSERT_TRUE(hkdfSha256(salt, sizeof(salt), ikm, sizeof(ikm), info, sizeof(info), okm, sizeof(okm)));
  assertDigest("3cb25f25faacd57a90434f64d0362f2a2d2d0a90cf1a5a4c5db02d56ecc4c5bf34007208d5b887185865", okm, 42);
  // Test case 3: empty salt and info.
  TEST_ASSERT_TRUE(hkdfSha256(nullptr, 0, ikm, sizeof(ikm), nullptr, 0, okm, sizeof(okm)));
  assertDigest("8da4e775a563c18f715f802a063c5a31b8a11f5c5ee1879ec3454e5f3c738d2d9d201395faa4b61a96c8", okm, 42);
  TEST_ASSERT_FALSE(hkdfSha256(salt, sizeof(salt), ikm, sizeof(ikm), info, sizeof(info), okm, 255 * 32 + 1));
}

static void test_constant_time_equal() {
  uint8_t a[16] = {0}, b[16] = {0};
  TEST_ASSERT_TRUE(constantTimeEqual(a, b, 16));
  for (int i = 0; i < 16; ++i) {
    b[i] = 0x80;
    TEST_ASSERT_FALSE(constantTimeEqual(a, b, 16));  // a difference anywhere is found
    b[i] = 0;
  }
  TEST_ASSERT_TRUE(constantTimeEqual(a, b, 0));
}

// Vectors from tools/protocol_vectors.py, an independent implementation using
// Python's hashlib/hmac (run it to regenerate). LMK and MACs are the public
// test values kLmk / kSensorMac / kGatewayMac above.
static const uint8_t kKatKeyS2G[32] = {
    0x5b, 0x8b, 0x0e, 0xf9, 0x24, 0x83, 0xe0, 0xb4, 0xbf, 0x72, 0x11, 0x54,
    0xfb, 0x47, 0x65, 0xd4, 0x9f, 0x27, 0x14, 0xbe, 0xd2, 0x7d, 0x9f, 0xb9,
    0x25, 0xa1, 0xa5, 0x13, 0xb3, 0x16, 0x9d, 0x94};
static const uint8_t kKatKeyG2S[32] = {
    0x6b, 0x6f, 0xdf, 0xc9, 0x2b, 0x36, 0xb5, 0x81, 0x37, 0x0f, 0x04, 0x1e,
    0x76, 0x4b, 0xc4, 0x53, 0xea, 0x4a, 0x08, 0x8c, 0x91, 0x71, 0xd7, 0xdd,
    0x7f, 0x4e, 0xd6, 0x3b, 0x9f, 0xd3, 0x05, 0xde};
static const uint8_t kKatHello[44] = {
    0x56, 0x02, 0x01, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x40, 0xe2, 0x01, 0x00, 0x0d, 0xf0, 0xa5, 0xa5,
    0xd5, 0x71, 0x00, 0x0b, 0x97, 0xff, 0x93, 0xa6, 0x20, 0x6d, 0x0a, 0xba,
    0x56, 0x51, 0xfa, 0xbb, 0x88, 0xc9, 0x3e, 0x4e};
static const uint8_t kKatHelloAck[40] = {
    0x56, 0x02, 0x02, 0x00, 0x07, 0x00, 0x00, 0x00, 0xef, 0xbe, 0xad, 0xde,
    0x01, 0x00, 0x00, 0x00, 0x63, 0x00, 0x00, 0x00, 0x0d, 0xf0, 0xa5, 0xa5,
    0x4c, 0x7e, 0xa0, 0x2a, 0x7f, 0x75, 0x28, 0xeb, 0x8f, 0x06, 0xe1, 0x8c,
    0x7d, 0x8d, 0x80, 0x90};
static const uint8_t kKatTelemetry[110] = {
    0x56, 0x02, 0x03, 0x00, 0x07, 0x00, 0x00, 0x00, 0xef, 0xbe, 0xad, 0xde,
    0x2a, 0x00, 0x00, 0x00, 0x40, 0xe2, 0x01, 0x00, 0x00, 0x98, 0x09, 0x00,
    0x14, 0xb0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x05, 0x00, 0x80, 0xff, 0xff,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0a, 0x00, 0xec, 0xff, 0xe8,
    0x03, 0xe8, 0x03, 0x1e, 0x05, 0xbb, 0xfe, 0xe2, 0x04, 0xe5, 0x03, 0x0e,
    0x00, 0x03, 0x00, 0x00, 0x00, 0x01, 0x00, 0x03, 0x05, 0x00, 0x00, 0x00,
    0x02, 0x00, 0x00, 0x00, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0xab, 0xd1,
    0xe6, 0x8a, 0xa9, 0x6d, 0xa7, 0xe5, 0xfa, 0xf9, 0x68, 0x48, 0x37, 0x54,
    0x94, 0x1c};

static void test_link_keys_and_packets_match_reference() {
  TEST_ASSERT_EQUAL_HEX8_ARRAY(kKatKeyS2G, kKeys.sensorToGateway, 32);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(kKatKeyG2S, kKeys.gatewayToSensor, 32);

  Packet q;
  TEST_ASSERT_EQUAL(DecodeError::Ok, decode(kKatHello, sizeof(kKatHello), kKey, q));
  TEST_ASSERT_EQUAL(MsgType::Hello, q.header.type);
  TEST_ASSERT_EQUAL_UINT32(7, q.header.bootId);
  TEST_ASSERT_EQUAL_HEX32(0xA5A5F00D, q.nonce);
  TEST_ASSERT_EQUAL_HEX32(0x0B0071D5, q.bootNonce);
  uint8_t buf[kMaxPacketSize];
  TEST_ASSERT_EQUAL_UINT32(sizeof(kKatHello), encode(q, kKey, buf, sizeof(buf)));
  TEST_ASSERT_EQUAL_HEX8_ARRAY(kKatHello, buf, sizeof(kKatHello));

  TEST_ASSERT_EQUAL(DecodeError::Ok, decode(kKatHelloAck, sizeof(kKatHelloAck), kG2S, q));
  TEST_ASSERT_EQUAL_HEX32(0xDEADBEEF, q.header.sessionId);
  TEST_ASSERT_EQUAL_UINT32(sizeof(kKatHelloAck), encode(q, kG2S, buf, sizeof(buf)));
  TEST_ASSERT_EQUAL_HEX8_ARRAY(kKatHelloAck, buf, sizeof(kKatHelloAck));

  TEST_ASSERT_EQUAL(DecodeError::Ok, decode(kKatTelemetry, sizeof(kKatTelemetry), kKey, q));
  TEST_ASSERT_EQUAL(SensorStatus::Disabled, q.telemetry.dht[1].status);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 24.56f, q.telemetry.dht[0].tempC);
  TEST_ASSERT_EQUAL_UINT8(kFeaturePir | kFeatureShock, q.telemetry.features);
  TEST_ASSERT_EQUAL_UINT32(sizeof(kKatTelemetry), encode(q, kKey, buf, sizeof(buf)));
  TEST_ASSERT_EQUAL_HEX8_ARRAY(kKatTelemetry, buf, sizeof(kKatTelemetry));
}

static void test_tag_binds_direction_and_addresses() {
  Packet q;
  // A sensor->gateway frame offered as gateway->sensor (reflection) is refused
  // before the tag is even computed.
  TEST_ASSERT_EQUAL(DecodeError::WrongDirection, decode(kKatHello, sizeof(kKatHello), kG2S, q));
  TEST_ASSERT_EQUAL(DecodeError::WrongDirection, decode(kKatHelloAck, sizeof(kKatHelloAck), kKey, q));
  // Same key and direction, different claimed sender or receiver: bad tag.
  uint8_t otherMac[6] = {0x24, 0x6F, 0x28, 0x00, 0x00, 0x03};
  AuthContext otherSender = makeAuthContext(kKeys, Direction::SensorToGateway, otherMac, kGatewayMac);
  AuthContext otherReceiver = makeAuthContext(kKeys, Direction::SensorToGateway, kSensorMac, otherMac);
  TEST_ASSERT_EQUAL(DecodeError::BadTag, decode(kKatHello, sizeof(kKatHello), otherSender, q));
  TEST_ASSERT_EQUAL(DecodeError::BadTag, decode(kKatHello, sizeof(kKatHello), otherReceiver, q));
  // The two direction keys differ, and encode() refuses the wrong direction.
  TEST_ASSERT_FALSE(memcmp(kKeys.sensorToGateway, kKeys.gatewayToSensor, 32) == 0);
  Packet ack;
  ack.header.type = MsgType::EventAck;
  uint8_t buf[kMaxPacketSize];
  TEST_ASSERT_EQUAL_UINT32(0, encode(ack, kKey, buf, sizeof(buf)));
}

static void test_protocol_disabled_sensors_and_strict_fields() {
  Packet p = sampleTelemetry();
  p.telemetry.dht[1] = DhtReading();
  p.telemetry.dht[1].status = SensorStatus::Disabled;
  p.telemetry.dht[1].ageMs = 999;        // not encoded for a disabled sensor
  p.telemetry.features = kFeatureShock;  // PIR not fitted
  p.telemetry.pirActive = false;
  uint8_t buf[kMaxPacketSize];
  size_t n = encode(p, kKey, buf, sizeof(buf));
  Packet q;
  TEST_ASSERT_EQUAL(DecodeError::Ok, decode(buf, n, kKey, q));
  TEST_ASSERT_EQUAL(SensorStatus::Disabled, q.telemetry.dht[1].status);
  TEST_ASSERT_EQUAL_UINT32(0, q.telemetry.dht[1].ageMs);
  TEST_ASSERT_EQUAL_UINT8(kFeatureShock, q.telemetry.features);

  // A disabled PIR cannot be active; a disabled IMU cannot report tilt.
  p.telemetry.pirActive = true;
  n = encode(p, kKey, buf, sizeof(buf));
  TEST_ASSERT_EQUAL(DecodeError::BadField, decode(buf, n, kKey, q));
  p.telemetry.pirActive = false;
  p.telemetry.imu = ImuReading();
  p.telemetry.imu.status = SensorStatus::Disabled;
  p.telemetry.imu.rateHz = 50;  // encoded as 0
  p.telemetry.tiltActive = true;
  n = encode(p, kKey, buf, sizeof(buf));
  TEST_ASSERT_EQUAL(DecodeError::BadField, decode(buf, n, kKey, q));
  p.telemetry.tiltActive = false;
  n = encode(p, kKey, buf, sizeof(buf));
  TEST_ASSERT_EQUAL(DecodeError::Ok, decode(buf, n, kKey, q));
  TEST_ASSERT_EQUAL_FLOAT(0, q.telemetry.imu.rateHz);

  // A value-less IMU status must carry zero values; unknown feature bits are refused.
  const size_t imuOff = kHeaderSize + 2 * 11;
  uint8_t bad[kMaxPacketSize];
  memcpy(bad, buf, n);
  bad[imuOff + 1] = 5;  // accelX on a disabled IMU
  sign(bad, n, kKey);
  TEST_ASSERT_EQUAL(DecodeError::BadField, decode(bad, n, kKey, q));
  memcpy(bad, buf, n);
  bad[imuOff + 23 + 2] = 0x04;  // features
  sign(bad, n, kKey);
  TEST_ASSERT_EQUAL(DecodeError::BadField, decode(bad, n, kKey, q));
}

// Deterministic malformed-input run over the production decoder. Seeds are
// valid packets of every type; mutations are bit flips, byte overwrites,
// truncation, extension and random lengths. Half of the mutants are re-signed
// with the right key so the field parser (not just the tag check) is
// exercised. Properties: no crash / sanitizer report, and every accepted
// packet re-encodes to exactly the same bytes (the decoder accepts only
// canonical encodings). Reproduce with the same seed and iteration count.
static const uint32_t kFuzzSeed = 0x5EED2026u;
static const uint32_t kFuzzIterations = 200000;

static uint32_t xorshift32(uint32_t& s) {
  s ^= s << 13;
  s ^= s >> 17;
  s ^= s << 5;
  return s;
}

static void test_decoder_fuzz_corpus() {
  Packet seeds[6];
  seeds[0] = sampleTelemetry();
  seeds[1].header.type = MsgType::Hello;
  seeds[1].header.bootId = 3;
  seeds[1].nonce = 77;
  seeds[1].bootNonce = 0x1234;
  seeds[2].header.type = MsgType::Event;
  seeds[2].header.bootId = 3;
  seeds[2].header.sessionId = 9;
  seeds[2].header.seq = 4;
  seeds[2].event.eventCounter = 2;
  seeds[2].event.attempt = 1;
  seeds[2].event.type = EventType::Tilt;
  seeds[2].event.state = EventState::Active;
  seeds[2].event.angleDeg = 47.5f;
  seeds[3].header.type = MsgType::HelloAck;
  seeds[3].header.bootId = 3;
  seeds[3].header.sessionId = 9;
  seeds[3].nonce = 77;
  seeds[4].header.type = MsgType::EventAck;
  seeds[4].header.bootId = 3;
  seeds[4].header.sessionId = 9;
  seeds[4].header.seq = 2;
  seeds[4].ackEventCounter = 2;
  seeds[5].header.type = MsgType::SessionReject;
  seeds[5].header.bootId = 3;
  seeds[5].rejectReason = RejectReason::StaleBoot;
  seeds[5].rejectValue = 12;
  uint8_t corpus[6][kMaxPacketSize];
  size_t lens[6];
  const AuthContext* ctx[6];
  for (int i = 0; i < 6; ++i) {
    ctx[i] = directionOf(seeds[i].header.type) == Direction::SensorToGateway ? &kKey : &kG2S;
    lens[i] = encode(seeds[i], *ctx[i], corpus[i], sizeof(corpus[i]));
    TEST_ASSERT_TRUE(lens[i] > 0);
  }

  uint32_t rng = kFuzzSeed;
  uint32_t counts[10] = {0};
  uint8_t buf[kMaxPacketSize + 16];
  uint8_t re[kMaxPacketSize];
  for (uint32_t it = 0; it < kFuzzIterations; ++it) {
    int k = (int)(xorshift32(rng) % 6);
    size_t len = lens[k];
    memcpy(buf, corpus[k], len);
    uint32_t op = xorshift32(rng) % 6;
    if (op == 0) {  // a few random bit flips
      int flips = 1 + (int)(xorshift32(rng) % 4);
      // Each draw is a separate statement: the order of two calls inside one
      // expression differs between C++14 and C++17 compilers.
      for (int f = 0; f < flips; ++f) {
        size_t at = xorshift32(rng) % len;
        uint8_t bit = (uint8_t)(1u << (xorshift32(rng) % 8));
        buf[at] ^= bit;
      }
    } else if (op == 1) {  // overwrite one byte anywhere
      size_t at = xorshift32(rng) % len;
      buf[at] = (uint8_t)xorshift32(rng);
    } else if (op == 2) {  // truncate
      len = xorshift32(rng) % len;
    } else if (op == 3) {  // extend, possibly beyond the ESP-NOW limit
      size_t extra = 1 + xorshift32(rng) % 16;
      for (size_t e = 0; e < extra; ++e) buf[len + e] = (uint8_t)xorshift32(rng);
      len += extra;
    } else if (op == 4) {  // random content of a random length
      len = xorshift32(rng) % (kMaxPacketSize + 8);
      for (size_t e = 0; e < len; ++e) buf[e] = (uint8_t)xorshift32(rng);
    } else {  // overwrite a 16-bit chunk inside the body
      size_t at = kHeaderSize + xorshift32(rng) % (len - kHeaderSize - kTagSize);
      buf[at] = (uint8_t)xorshift32(rng);
      if (at + 1 < len - kTagSize) buf[at + 1] = (uint8_t)xorshift32(rng);
    }
    bool resign = (xorshift32(rng) & 1) != 0;
    if (resign && len >= kHeaderSize + kTagSize && len <= kMaxPacketSize) sign(buf, len, *ctx[k]);
    Packet q;
    DecodeError e = decode(buf, len, *ctx[k], q);
    ++counts[(int)e];
    if (e == DecodeError::Ok) {
      size_t m = encode(q, *ctx[k], re, sizeof(re));
      TEST_ASSERT_EQUAL_UINT32(len, m);
      TEST_ASSERT_EQUAL_HEX8_ARRAY(buf, re, len);
    }
  }
  printf("[fuzz] seed 0x%08lx, %lu inputs: ok=%lu too_short=%lu too_long=%lu bad_magic=%lu version=%lu "
         "type=%lu direction=%lu length=%lu tag=%lu field=%lu\n",
         (unsigned long)kFuzzSeed, (unsigned long)kFuzzIterations, (unsigned long)counts[0],
         (unsigned long)counts[1], (unsigned long)counts[2], (unsigned long)counts[3], (unsigned long)counts[4],
         (unsigned long)counts[5], (unsigned long)counts[6], (unsigned long)counts[7], (unsigned long)counts[8],
         (unsigned long)counts[9]);
  TEST_ASSERT_TRUE(counts[(int)DecodeError::BadField] > 1000);  // the field parser itself was reached
  TEST_ASSERT_TRUE(counts[(int)DecodeError::Ok] > 100);
}

static void test_protocol_rejects_forged_or_tampered_packets() {
  Packet p = sampleTelemetry();
  uint8_t buf[kMaxPacketSize];
  size_t n = encode(p, kKey, buf, sizeof(buf));
  Packet q;
  q.header.seq = 999;  // must stay untouched on failure
  // Any bit flip in header, body or tag is rejected before parsing.
  for (size_t i = 3; i < n; i += 7) {
    uint8_t bad[kMaxPacketSize];
    memcpy(bad, buf, n);
    bad[i] ^= 0x01;
    TEST_ASSERT_EQUAL(DecodeError::BadTag, decode(bad, n, kKey, q));
  }
  // A packet built with another key (e.g. a different installation) fails.
  uint8_t otherLmk[16] = {1};
  AuthContext other = makeAuthContext(deriveLinkKeys(otherLmk), Direction::SensorToGateway, kSensorMac, kGatewayMac);
  size_t m = encode(p, other, buf, sizeof(buf));
  TEST_ASSERT_EQUAL(DecodeError::BadTag, decode(buf, m, kKey, q));
  TEST_ASSERT_EQUAL_UINT32(999, q.header.seq);
}

// -------------------------------------------------------- replay/session ----

static void test_replay_window() {
  ReplayWindow w;
  TEST_ASSERT_FALSE(w.accept(0));
  TEST_ASSERT_TRUE(w.accept(1));
  TEST_ASSERT_FALSE(w.accept(1));
  TEST_ASSERT_TRUE(w.accept(3));
  TEST_ASSERT_TRUE(w.accept(2));  // reordered within window
  TEST_ASSERT_FALSE(w.accept(2));
  TEST_ASSERT_TRUE(w.accept(100));
  TEST_ASSERT_FALSE(w.accept(36));  // 64 behind
  TEST_ASSERT_TRUE(w.accept(37));   // 63 behind, unseen
  TEST_ASSERT_FALSE(w.accept(37));
}

static void test_session_policy_across_reboots() {
  const uint32_t n5 = 0xB005, n6 = 0xB006;  // per-boot nonces
  PeerSession gw;
  bool persist = false;
  TEST_ASSERT_EQUAL(PeerSession::Verdict::NoSession, gw.onData(5, 1, 1));
  uint32_t s1 = gw.onHello(5, n5, 0x1111, &persist);
  TEST_ASSERT_EQUAL_UINT32(0x1111, s1);
  TEST_ASSERT_TRUE(persist);
  TEST_ASSERT_EQUAL(PeerSession::Verdict::Accept, gw.onData(5, s1, 1));
  TEST_ASSERT_EQUAL(PeerSession::Verdict::Replay, gw.onData(5, s1, 1));
  TEST_ASSERT_EQUAL(PeerSession::Verdict::WrongSession, gw.onData(5, 0x2222, 2));

  // Sensor reboots: higher boot gets a new session; old-session frames rejected.
  uint32_t s2 = gw.onHello(6, n6, 0x3333, &persist);
  TEST_ASSERT_TRUE(persist);
  TEST_ASSERT_EQUAL(PeerSession::Verdict::StaleBoot, gw.onData(5, s1, 2));
  TEST_ASSERT_EQUAL(PeerSession::Verdict::Accept, gw.onData(6, s2, 1));

  // Replayed Hello from an older boot is refused without disturbing the session.
  TEST_ASSERT_EQUAL_UINT32(0, gw.onHello(5, n5, 0x4444, &persist));
  TEST_ASSERT_EQUAL(PeerSession::Verdict::Accept, gw.onData(6, s2, 2));

  // Same-boot re-handshake (e.g. channel change) is allowed, no NVS write.
  uint32_t s3 = gw.onHello(6, n6, 0x5555, &persist);
  TEST_ASSERT_FALSE(persist);
  TEST_ASSERT_EQUAL(PeerSession::Verdict::WrongSession, gw.onData(6, s2, 3));
  TEST_ASSERT_EQUAL(PeerSession::Verdict::Accept, gw.onData(6, s3, 1));

  // The same boot id from a different boot (sensor NVS erased) is refused
  // without disturbing the current session.
  TEST_ASSERT_EQUAL_UINT32(0, gw.onHello(6, 0xE4A5ED, 0x5656, &persist));
  TEST_ASSERT_EQUAL(PeerSession::Verdict::Accept, gw.onData(6, s3, 2));
  TEST_ASSERT_EQUAL_UINT32(0, gw.onHello(6, 0, 0x5757, &persist));  // nonce 0 never valid

  // Gateway reboots: persisted boot restored, but no session is active, so
  // frames captured before the reboot are rejected until a fresh Hello.
  PeerSession gw2;
  gw2.restorePersistedBoot(gw.persistedBoot(), gw.persistedNonce());
  TEST_ASSERT_EQUAL(PeerSession::Verdict::NoSession, gw2.onData(6, s3, 2));
  TEST_ASSERT_EQUAL_UINT32(0, gw2.onHello(5, n5, 0x6666, &persist));
  TEST_ASSERT_EQUAL_UINT32(0, gw2.onHello(6, 0xE4A5ED, 0x6767, &persist));
  TEST_ASSERT_EQUAL_UINT32(0x7777, gw2.onHello(6, n6, 0x7777, &persist));
  TEST_ASSERT_FALSE(persist);

  // Nonce unknown (e.g. its NVS write was lost): the next Hello is accepted
  // and its nonce recorded.
  PeerSession gw3;
  gw3.restorePersistedBoot(6, 0);
  TEST_ASSERT_EQUAL_UINT32(0x8888, gw3.onHello(6, n6, 0x8888, &persist));
  TEST_ASSERT_TRUE(persist);
  TEST_ASSERT_EQUAL_UINT32(n6, gw3.persistedNonce());
}

static void test_recent_event_ids_dedupe() {
  RecentEventIds<4> ids;
  ids.insert(1, 10);
  TEST_ASSERT_TRUE(ids.contains(1, 10));
  TEST_ASSERT_FALSE(ids.contains(2, 10));
  for (uint32_t i = 11; i < 15; ++i) ids.insert(1, i);
  TEST_ASSERT_FALSE(ids.contains(1, 10));  // evicted after capacity
  TEST_ASSERT_TRUE(ids.contains(1, 14));
}

// ---------------------------------------------------------------- queues ----

static EventBody makeEvent(uint32_t counter) {
  EventBody e;
  e.eventCounter = counter;
  e.type = EventType::Impact;
  e.state = EventState::Detected;
  e.detectedUptimeMs = 1000 * counter;
  return e;
}

static void test_retransmit_preserves_identity_and_acks() {
  RetransmitQueue<4> q;
  q.push(makeEvent(1));
  RetransmitQueue<4>::Entry* e = q.nextDue(0, 300);
  TEST_ASSERT_NOT_NULL(e);
  q.markSent(e, 0);
  TEST_ASSERT_NULL(q.nextDue(299, 300));
  e = q.nextDue(300, 300);
  TEST_ASSERT_NOT_NULL(e);
  TEST_ASSERT_EQUAL_UINT32(1, e->event.eventCounter);
  TEST_ASSERT_EQUAL_UINT32(1000, e->event.detectedUptimeMs);
  q.markSent(e, 300);
  TEST_ASSERT_EQUAL_UINT8(2, e->event.attempt);
  TEST_ASSERT_TRUE(q.ack(1));
  TEST_ASSERT_FALSE(q.ack(1));
  TEST_ASSERT_EQUAL(0, q.size());
}

static void test_retransmit_overflow_and_expiry() {
  RetransmitQueue<2> q;
  q.push(makeEvent(1));
  q.push(makeEvent(2));
  q.push(makeEvent(3));  // drops 1
  TEST_ASSERT_EQUAL_UINT32(1, q.dropped());
  TEST_ASSERT_FALSE(q.ack(1));
  RetransmitQueue<2>::Entry* e = q.nextDue(0, 100);
  TEST_ASSERT_EQUAL_UINT32(2, e->event.eventCounter);  // FIFO
  for (int i = 0; i < 3; ++i) {
    q.markSent(e, i * 100);
  }
  TEST_ASSERT_EQUAL(0u, q.expire(3, 250, 100));  // last try still in its window
  TEST_ASSERT_EQUAL(1u, q.expire(3, 300, 100));
  TEST_ASSERT_EQUAL_UINT32(1, q.expired());
  TEST_ASSERT_EQUAL(1, q.size());
}

typedef CloudOutbox<3, 128> Outbox;

static void test_cloud_queue_alert_priority_and_coalescing() {
  Outbox q;
  TEST_ASSERT_TRUE(q.pushSummary("s1", "{\"s\":1}", 7));
  TEST_ASSERT_TRUE(q.pushAlert("a1", "{\"a\":1}", 7, false));
  TEST_ASSERT_TRUE(q.pushSummary("s2", "{\"s\":2}", 7));  // replaces s1
  TEST_ASSERT_EQUAL_UINT32(1, q.stats().summariesCoalesced);
  Outbox::Message* m = q.nextToSend(4);
  TEST_ASSERT_EQUAL_STRING("a1", m->messageId);  // alert first
  q.markInFlight(m, 10, 0);
  m = q.nextToSend(4);
  TEST_ASSERT_EQUAL_STRING("s2", m->messageId);
  q.markInFlight(m, 11, 0);
  TEST_ASSERT_NULL(q.nextToSend(4));
  TEST_ASSERT_NULL(q.nextToSend(2));
  TEST_ASSERT_TRUE(q.onPublished(10));
  TEST_ASSERT_TRUE(q.onPublished(11));
  TEST_ASSERT_FALSE(q.onPublished(11));
  TEST_ASSERT_EQUAL(0, q.depth());
}

static void test_cloud_queue_overflow_policy() {
  Outbox q;
  q.pushAlert("a1", "x", 1, false);
  q.pushAlert("r1", "x", 1, true);
  q.pushAlert("a2", "x", 1, false);
  // Full: the reminder goes first.
  TEST_ASSERT_TRUE(q.pushAlert("a3", "x", 1, false));
  TEST_ASSERT_EQUAL_UINT32(1, q.stats().remindersDropped);
  // Full again, no reminders: oldest pending alert (a1) is dropped...
  Outbox::Message* first = q.nextToSend(1);
  TEST_ASSERT_EQUAL_STRING("a1", first->messageId);
  q.markInFlight(first, 1, 0);  // ...unless it is in flight: then a2 goes.
  TEST_ASSERT_TRUE(q.pushAlert("a4", "x", 1, false));
  TEST_ASSERT_EQUAL_UINT32(1, q.stats().alertsDropped);
  TEST_ASSERT_TRUE(q.onPublished(1));
  Outbox::Message* m = q.nextToSend(4);
  TEST_ASSERT_EQUAL_STRING("a3", m->messageId);
  // Summaries never take alert slots.
  TEST_ASSERT_TRUE(q.pushSummary("s", "y", 1));
  TEST_ASSERT_EQUAL(2, q.pendingAlerts());  // a3 (in flight) + a4
  // Oversized payloads are rejected, not truncated.
  char big[200];
  memset(big, 'z', sizeof(big));
  TEST_ASSERT_FALSE(q.pushAlert("big", big, sizeof(big), false));
}

static void test_cloud_queue_republish_after_disconnect_keeps_identity() {
  Outbox q;
  const char* body = "{\"eventId\":\"dev:1:7\"}";
  q.pushAlert("dev:1:7:detected", body, strlen(body), false);
  Outbox::Message* m = q.nextToSend(4);
  q.markInFlight(m, 5, 1000);
  q.onDisconnected();
  m = q.nextToSend(4);
  TEST_ASSERT_NOT_NULL(m);
  TEST_ASSERT_EQUAL_STRING("dev:1:7:detected", m->messageId);
  TEST_ASSERT_EQUAL_STRING("{\"eventId\":\"dev:1:7\"}", m->payload);
  q.markInFlight(m, 6, 2000);
  TEST_ASSERT_EQUAL_UINT32(1, q.stats().republished);
  TEST_ASSERT_FALSE(q.onPublished(5));  // stale msg id from the old connection
  TEST_ASSERT_EQUAL(0u, q.requeueTimedOut(2000 + 29999, 30000));
  TEST_ASSERT_EQUAL(1u, q.requeueTimedOut(2000 + 30000, 30000));
  m = q.nextToSend(4);
  q.markInFlight(m, 7, 40000);
  TEST_ASSERT_TRUE(q.onDeleted(7));
  TEST_ASSERT_NOT_NULL(q.nextToSend(4));
}

// ------------------------------------------------------------ aggregation ----

static void test_aggregator_counter_deltas_and_reboot() {
  SensorNodeAggregator agg;
  Packet p = sampleTelemetry();
  p.telemetry.counters.impacts = 10;
  agg.onTelemetry(p.header, p.telemetry);  // baseline only
  TEST_ASSERT_EQUAL_UINT32(0, agg.window().impacts);
  p.telemetry.counters.impacts = 12;
  agg.onTelemetry(p.header, p.telemetry);
  TEST_ASSERT_EQUAL_UINT32(2, agg.window().impacts);
  p.header.bootId = 8;  // sensor rebooted
  p.telemetry.counters.impacts = 1;
  agg.onTelemetry(p.header, p.telemetry);
  TEST_ASSERT_EQUAL_UINT32(3, agg.window().impacts);
  // Stale cached DHT values are not counted as new samples.
  uint32_t before = agg.window().dht[0].tempC.count;
  p.telemetry.dht[0].status = SensorStatus::Stale;
  agg.onTelemetry(p.header, p.telemetry);
  TEST_ASSERT_EQUAL_UINT32(before, agg.window().dht[0].tempC.count);
  TEST_ASSERT_EQUAL(SensorStatus::Stale, agg.window().dht[0].status);
  agg.resetWindow();
  TEST_ASSERT_EQUAL_UINT32(0, agg.window().impacts);
  // The same cached Ok value repeated with a growing age is one measurement;
  // a smaller age means a new measurement.
  p.telemetry.dht[0].status = SensorStatus::Ok;
  p.telemetry.dht[0].ageMs = 100;
  agg.onTelemetry(p.header, p.telemetry);
  p.telemetry.dht[0].ageMs = 600;
  agg.onTelemetry(p.header, p.telemetry);
  p.telemetry.dht[0].ageMs = 1100;
  agg.onTelemetry(p.header, p.telemetry);
  TEST_ASSERT_EQUAL_UINT32(1, agg.window().dht[0].samples);
  p.telemetry.dht[0].ageMs = 50;
  agg.onTelemetry(p.header, p.telemetry);
  TEST_ASSERT_EQUAL_UINT32(2, agg.window().dht[0].samples);
  TEST_ASSERT_TRUE(agg.window().pirActive);
}

// ------------------------------------------------------------------ json ----

static void assertStrictJson(const char* s, size_t n) {
  TEST_ASSERT_EQUAL(strlen(s), n);
  JsonDocument doc;
  TEST_ASSERT_EQUAL(DeserializationError::Ok, deserializeJson(doc, s, n).code());
  TEST_ASSERT_NULL(strstr(s, "nan"));
  TEST_ASSERT_NULL(strstr(s, "NaN"));
  TEST_ASSERT_NULL(strstr(s, "inf"));
}

static telemetry::Envelope sampleEnvelope() {
  telemetry::Envelope env;
  env.gatewayId = "vehicle-gateway-01";
  env.gatewayBootId = 12;
  env.seq = 345;
  env.uptimeMs = 3600000;
  env.unixMs = 1761393600123LL;
  env.timeValid = true;
  return env;
}

static void test_utc_formatting() {
  char buf[32];
  TEST_ASSERT_TRUE(formatUtcIso8601(0, buf, sizeof(buf)));
  TEST_ASSERT_EQUAL_STRING("1970-01-01T00:00:00.000Z", buf);
  TEST_ASSERT_TRUE(formatUtcIso8601(1761393600123LL, buf, sizeof(buf)));
  TEST_ASSERT_EQUAL_STRING("2025-10-25T12:00:00.123Z", buf);
  TEST_ASSERT_TRUE(formatUtcIso8601(951782400000LL, buf, sizeof(buf)));  // leap day
  TEST_ASSERT_EQUAL_STRING("2000-02-29T00:00:00.000Z", buf);
  TEST_ASSERT_FALSE(formatUtcIso8601(0, buf, 10));
}

static void test_summary_json_valid_when_readings_fail() {
  SensorNodeAggregator agg;
  Packet p = sampleTelemetry();
  for (int i = 0; i < 120; ++i) {
    p.header.seq++;
    p.telemetry.counters.pirRises += (i % 40 == 0);
    agg.onTelemetry(p.header, p.telemetry);
  }
  GatewayWindow gw;
  gw.dht22.seen = true;
  gw.dht22.status = SensorStatus::Error;
  gw.dht22.consecutiveErrors = 4;
  gw.flamePct.add(3.2f);
  gw.flamePct.add(NAN);
  gw.potPct.add(50.0f);

  telemetry::Summary s;
  s.windowMs = 60000;
  s.link.deviceId = "sensor-node-01";
  s.link.configured = true;
  s.link.up = true;
  s.link.everSeen = true;
  s.link.lastRxAgeMs = 230;
  s.sensor = &agg.window();
  s.gateway = &gw;
  s.health.wifi = "connected";
  s.health.rssi = -61;
  s.health.cloud = "connected";
  s.health.mqttConnects = 1;
  s.health.freeHeap = 181234;
  s.health.estMeterUnits = 42;

  char out[1536];
  size_t n = telemetry::serializeSummary(sampleEnvelope(), s, out, sizeof(out));
  TEST_ASSERT_TRUE(n > 0);
  assertStrictJson(out, n);
  printf("[example] summary (%u bytes): %s\n", (unsigned)n, out);
  JsonDocument doc;
  deserializeJson(doc, out);
  TEST_ASSERT_EQUAL(2, doc["schemaVersion"].as<int>());
  TEST_ASSERT_EQUAL_STRING("summary", doc["messageType"]);
  TEST_ASSERT_EQUAL_STRING("2025-10-25T12:00:00.123Z", doc["ts"]);
  TEST_ASSERT_TRUE(doc["sensorNode"]["dht11"][1]["tempC"].isNull());
  TEST_ASSERT_EQUAL_STRING("error", doc["sensorNode"]["dht11"][1]["status"]);
  TEST_ASSERT_TRUE(doc["gateway"]["dht22"]["tempC"].isNull());
  TEST_ASSERT_TRUE(doc["sensorNode"]["dht11"][0]["tempC"]["avg"].is<float>());

  // Too small a buffer: nothing (not a truncated document) is produced.
  char small[200];
  TEST_ASSERT_EQUAL_UINT32(0, telemetry::serializeSummary(sampleEnvelope(), s, small, sizeof(small)));
  TEST_ASSERT_EQUAL_UINT32(0, telemetry::serializeSummary(sampleEnvelope(), s, out, n));  // no room for NUL

  // Without a valid clock, ts is null.
  telemetry::Envelope env = sampleEnvelope();
  env.timeValid = false;
  n = telemetry::serializeSummary(env, s, out, sizeof(out));
  assertStrictJson(out, n);
  deserializeJson(doc, out);
  TEST_ASSERT_TRUE(doc["ts"].isNull());
}

static void test_summary_json_before_sensor_seen() {
  telemetry::Summary s;
  s.windowMs = 60000;
  s.link.deviceId = "sensor-node-01";
  s.link.configured = false;
  s.health.wifi = "unconfigured";
  s.health.cloud = "unconfigured";
  char out[1024];
  size_t n = telemetry::serializeSummary(sampleEnvelope(), s, out, sizeof(out));
  assertStrictJson(out, n);
}

static void test_alert_json_examples() {
  telemetry::Alert a;
  a.eventId = "sensor-node-01:7:3";
  a.alertType = "impact";
  a.state = "detected";
  a.sourceRole = "sensorNode";
  a.sourceDeviceId = "sensor-node-01";
  a.sourceBootId = 7;
  a.detectedUptimeMs = 3599750;
  a.eventUnixMs = 1761393599873LL;
  a.eventTsValid = true;
  a.eventTsSource = "gateway_estimate";
  a.receivedUnixMs = 1761393600100LL;
  a.receivedTsValid = true;
  a.peakAccelG = 2.41f;
  a.pulseCount = 3;
  char out[640];
  size_t n = telemetry::serializeAlert(sampleEnvelope(), a, out, sizeof(out));
  TEST_ASSERT_TRUE(n > 0);
  assertStrictJson(out, n);
  printf("[example] impact alert (%u bytes): %s\n", (unsigned)n, out);

  telemetry::Alert f;
  f.eventId = "vehicle-gateway-01:12:1";
  f.alertType = "flame";
  f.state = "active";
  f.reminder = 2;
  f.sourceRole = "gateway";
  f.sourceDeviceId = "vehicle-gateway-01";
  f.sourceBootId = 12;
  f.detectedUptimeMs = 2400000;
  f.eventUnixMs = 1761392400000LL;
  f.eventTsValid = true;
  f.eventTsSource = "gateway_clock";
  f.flamePct = NAN;  // sensor failed: omitted rather than "nan"
  n = telemetry::serializeAlert(sampleEnvelope(), f, out, sizeof(out));
  assertStrictJson(out, n);
  printf("[example] flame reminder (%u bytes): %s\n", (unsigned)n, out);

  telemetry::Alert l;
  l.eventId = "vehicle-gateway-01:12:2";
  l.alertType = "sensor_link_lost";
  l.state = "active";
  l.sourceRole = "gateway";
  l.sourceDeviceId = "sensor-node-01";
  l.sourceBootId = 12;
  l.detectedUptimeMs = 3605000;
  l.silentMs = 5000;
  n = telemetry::serializeAlert(sampleEnvelope(), l, out, sizeof(out));
  assertStrictJson(out, n);
  printf("[example] link-lost alert, event time unknown (%u bytes): %s\n", (unsigned)n, out);
}

// ------------------------------------------------------------------ util ----

static void test_config_validation() {
  uint8_t key[16];
  TEST_ASSERT_TRUE(isPlaceholder(""));
  TEST_ASSERT_TRUE(isPlaceholder("YOUR_WIFI_SSID"));
  TEST_ASSERT_FALSE(isPlaceholder("CarNet"));
  TEST_ASSERT_TRUE(parseHexKey("00112233445566778899aabbccddeeff", key, 16));
  TEST_ASSERT_EQUAL_HEX8(0xff, key[15]);
  TEST_ASSERT_FALSE(parseHexKey("00000000000000000000000000000000", key, 16));
  TEST_ASSERT_FALSE(parseHexKey("0011223344556677889", key, 16));
  TEST_ASSERT_FALSE(parseHexKey("zz112233445566778899aabbccddeeff", key, 16));
  uint8_t mac[6];
  TEST_ASSERT_TRUE(parseMac("24:6F:28:00:0A:b1", mac));
  TEST_ASSERT_EQUAL_HEX8(0x00, mac[3]);
  TEST_ASSERT_EQUAL_HEX8(0xb1, mac[5]);
  TEST_ASSERT_FALSE(parseMac("FF:FF:FF:FF:FF:FF", mac));  // broadcast
  TEST_ASSERT_FALSE(parseMac("00:00:00:00:00:00", mac));
  TEST_ASSERT_FALSE(parseMac("24-6F-28-00-0A-B1", mac));
}

static void test_gpio_constraints_and_metering() {
  TEST_ASSERT_TRUE(gpioUsableInput(34));
  TEST_ASSERT_FALSE(gpioUsableOutput(34));
  TEST_ASSERT_FALSE(gpioUsableInput(6));
  TEST_ASSERT_FALSE(gpioExists(20));
  TEST_ASSERT_TRUE(gpioIsAdc1(33));
  TEST_ASSERT_FALSE(gpioIsAdc1(25));
  TEST_ASSERT_TRUE(gpioIsStrapping(12));
  TEST_ASSERT_EQUAL_UINT32(1, meterUnits(512, 512));
  TEST_ASSERT_EQUAL_UINT32(2, meterUnits(513, 512));
  TEST_ASSERT_EQUAL_UINT32(1, meterUnits(1200, 4096));
}

struct MemCounter : public CounterStore {
  uint32_t value = 0;
  bool loadOk = true, storeOk = true;
  unsigned writes = 0;
  bool load(uint32_t& v) override {
    v = value;
    return loadOk;
  }
  bool store(uint32_t v) override {
    if (!storeOk) return false;  // e.g. NVS full, or power lost before the commit
    value = v;
    ++writes;
    return true;
  }
};

static void test_boot_counter_persistence_rules() {
  MemCounter c;
  TEST_ASSERT_EQUAL_UINT32(1, advanceBootCounter(c));
  TEST_ASSERT_EQUAL_UINT32(2, advanceBootCounter(c));
  // A failed write yields "no boot id" (the link stays off); an id that is
  // not durably stored is never handed out.
  c.storeOk = false;
  TEST_ASSERT_EQUAL_UINT32(0, advanceBootCounter(c));
  c.storeOk = true;
  TEST_ASSERT_EQUAL_UINT32(3, advanceBootCounter(c));
  c.loadOk = false;
  TEST_ASSERT_EQUAL_UINT32(0, advanceBootCounter(c));
  c.loadOk = true;
  // Exhausted counter: refuse instead of wrapping to an already granted id.
  c.value = 0xFFFFFFFFu;
  TEST_ASSERT_EQUAL_UINT32(0, advanceBootCounter(c));
  TEST_ASSERT_EQUAL_UINT32(0xFFFFFFFFu, c.value);
  TEST_ASSERT_EQUAL(3u, c.writes);  // exactly one NVS write per successful boot
}

static void test_url_encode_and_pin_checks() {
  char out[32];
  TEST_ASSERT_TRUE(urlEncode("gw-01:12:s3", out, sizeof(out)));
  TEST_ASSERT_EQUAL_STRING("gw-01%3A12%3As3", out);
  TEST_ASSERT_TRUE(urlEncode("a b/c~", out, sizeof(out)));
  TEST_ASSERT_EQUAL_STRING("a%20b%2Fc~", out);
  TEST_ASSERT_FALSE(urlEncode("::::::::::", out, 30));  // needs 31 bytes including the NUL
  TEST_ASSERT_TRUE(urlEncode("::::::::::", out, 31));
  static constexpr int defaults[] = {14, 27, 25, 26, 21, 22, -1, -1};  // sensor-node defaults
  static constexpr int clash[] = {14, 27, 25, 14};
  static_assert(pinsDistinct(defaults), "default sensor pins must be distinct");
  TEST_ASSERT_FALSE(pinsDistinct(clash));
  TEST_ASSERT_FALSE(gpioHasInternalPull(36));
  TEST_ASSERT_TRUE(gpioHasInternalPull(25));
  for (int pin : {14, 27, 25, 26, 21, 22, 4, 33, 32}) TEST_ASSERT_FALSE(gpioIsStrapping(pin));
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_elapsed_across_millis_wrap);
  RUN_TEST(test_periodic_timer_across_wrap_and_no_burst);
  RUN_TEST(test_routine_report_interval_is_configured_period);
  RUN_TEST(test_backoff_grows_caps_and_jitters);
  RUN_TEST(test_retry_timer_wrap_safe);
  RUN_TEST(test_sas_refresh_without_waiting_an_hour);
  RUN_TEST(test_hysteresis_alarm_debounce_and_clear);
  RUN_TEST(test_hysteresis_alarm_ignores_invalid_samples);
  RUN_TEST(test_reminders_are_rate_limited);
  RUN_TEST(test_protocol_roundtrip_and_size);
  RUN_TEST(test_protocol_nonfinite_ok_reading_is_downgraded);
  RUN_TEST(test_protocol_rejects_bad_framing);
  RUN_TEST(test_protocol_rejects_bad_fields);
  RUN_TEST(test_protocol_every_type_fits_espnow);
  RUN_TEST(test_sha256_and_hmac_vectors);
  RUN_TEST(test_hkdf_rfc5869_vectors);
  RUN_TEST(test_constant_time_equal);
  RUN_TEST(test_link_keys_and_packets_match_reference);
  RUN_TEST(test_tag_binds_direction_and_addresses);
  RUN_TEST(test_protocol_disabled_sensors_and_strict_fields);
  RUN_TEST(test_decoder_fuzz_corpus);
  RUN_TEST(test_protocol_rejects_forged_or_tampered_packets);
  RUN_TEST(test_replay_window);
  RUN_TEST(test_session_policy_across_reboots);
  RUN_TEST(test_recent_event_ids_dedupe);
  RUN_TEST(test_retransmit_preserves_identity_and_acks);
  RUN_TEST(test_retransmit_overflow_and_expiry);
  RUN_TEST(test_cloud_queue_alert_priority_and_coalescing);
  RUN_TEST(test_cloud_queue_overflow_policy);
  RUN_TEST(test_cloud_queue_republish_after_disconnect_keeps_identity);
  RUN_TEST(test_aggregator_counter_deltas_and_reboot);
  RUN_TEST(test_utc_formatting);
  RUN_TEST(test_summary_json_valid_when_readings_fail);
  RUN_TEST(test_summary_json_before_sensor_seen);
  RUN_TEST(test_alert_json_examples);
  RUN_TEST(test_config_validation);
  RUN_TEST(test_gpio_constraints_and_metering);
  RUN_TEST(test_boot_counter_persistence_rules);
  RUN_TEST(test_url_encode_and_pin_checks);
  return UNITY_END();
}
