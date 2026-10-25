// Cloud (Azure IoT Hub over MQTT 3.1.1) connection lifecycle, independent of
// the MQTT client library.
//
//   Unconfigured                      placeholder hub/device/key: never connects
//   WaitNetwork  -> (network up && UTC clock plausible && backoff elapsed)
//   Connecting   fresh SAS token, client started; waits for CONNACK
//   Connected    only after the broker accepted CONNECT
//   Backoff      after a transport/TLS error, refused CONNECT, timeout or drop
//
// The SAS token is regenerated refreshMarginSec before it expires (measured on
// the monotonic clock) by stopping the client and reconnecting with the new
// password. A refused CONNECT with "bad username/password" or "not authorized"
// backs off slowly (1 min doubling to 30 min); other failures back off from
// 2 s to 2 min. The transport backoff is reset only after a connection stayed
// up for stableMs, so a broker that accepts and immediately drops connections
// is not hammered.
//
// MQTT client events are delivered with the generation number of the client
// that produced them; events from a stopped client are ignored, so a late
// PUBACK can never acknowledge a message of a newer connection. PUBACK and
// outbox-deletion events are passed on through pollEvent(), followed by a
// LinkDown event whenever a connection ends, so the outbox can re-send
// everything that was not acknowledged.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "vc_backoff.h"
#include "vc_ports.h"
#include "vc_util.h"

namespace vc {

struct MqttEvent {
  enum class Kind : uint8_t { Connected, Disconnected, Refused, TransportError, Published, Deleted };
  uint32_t generation = 0;
  Kind kind = Kind::Disconnected;
  int msgId = -1;
  int refuseCode = 0;  // MQTT 3.1.1 CONNACK return code (1..5) for Refused
  int tlsErr = 0, tlsStackErr = 0, sockErrno = 0;  // TransportError details
};

// MQTT 3.1.1 CONNACK return codes.
const int kConnackBadCredentials = 4;
const int kConnackNotAuthorized = 5;

class MqttPort {
 public:
  virtual ~MqttPort() {}
  // Creates and starts a client for `generation` that authenticates with
  // `password`. The port must copy the password; the caller wipes its buffer
  // right after the call. Returns false on immediate failure.
  virtual bool start(uint32_t generation, const char* password) = 0;
  // Stops and destroys the current client (main-loop context).
  virtual void stop() = 0;
  // Enqueues a QoS 1 device-to-cloud message without blocking. Returns the
  // MQTT message id (> 0) or a value <= 0 on failure. `wireBytes` receives
  // the size that counts towards metering (body plus properties).
  virtual int publish(const char* messageId, const char* messageType, const char* payload, size_t len,
                      size_t* wireBytes) = 0;
};

class SasTokenSource {
 public:
  virtual ~SasTokenSource() {}
  // Writes a SharedAccessSignature valid for lifetimeSec from the current UTC
  // time into `out`. Returns false on failure. Never log the result.
  virtual bool generate(uint32_t lifetimeSec, char* out, size_t cap) = 0;
};

struct CloudLinkConfig {
  uint32_t sasLifetimeSec = 3600;
  uint32_t refreshMarginSec = 300;
  uint32_t connectTimeoutMs = 30000;
  uint32_t retryBaseMs = 2000, retryCapMs = 120000;
  uint32_t authRetryBaseMs = 60000, authRetryCapMs = 1800000;
  uint32_t stableMs = 60000;
};

class CloudLinkCore {
 public:
  enum class State : uint8_t { Unconfigured, WaitNetwork, Connecting, Connected, Backoff };

  struct Event {
    enum class Kind : uint8_t { Published, Deleted, LinkDown };
    Kind kind;
    int msgId;
  };

  bool begin(MqttPort* mqtt, SasTokenSource* tokens, RandomSource* rng, const CloudLinkConfig& cfg = CloudLinkConfig());

  // Feed client events in arrival order (from the loop, never the callback).
  void onMqttEvent(const MqttEvent& ev, uint32_t now);
  // The adapter could not queue some client events (its queue overflowed):
  // the connection state is no longer known, so it is restarted.
  void onEventsLost(uint32_t now);
  void tick(uint32_t now, bool networkUp, bool timeValid);

  // Accepted by the client != delivered: delivery is a Published event.
  int publish(const char* messageId, const char* messageType, const char* payload, size_t len, size_t* wireBytes);
  bool pollEvent(Event& out);

  bool connected() const { return state_ == State::Connected; }
  State state() const { return state_; }
  const char* stateName() const;
  const char* lastError() const { return lastError_; }
  uint32_t generation() const { return generation_; }
  uint32_t connects() const { return connects_; }
  uint32_t sasRenewals() const { return sasRenewals_; }
  uint32_t failures() const { return failures_; }
  uint32_t authFailures() const { return authFailures_; }
  uint32_t eventOverflows() const { return eventOverflows_; }
  uint32_t tokenRemainingMs(uint32_t now) const { return sas_.remainingMs(now); }
  uint32_t retryAt() const { return retryAt_; }

 private:
  void startConnect(uint32_t now);
  void teardown(const char* reason);
  void fail(uint32_t now, bool authFailure);
  void pushEvent(Event::Kind kind, int msgId);

  State state_ = State::Unconfigured;
  CloudLinkConfig cfg_;
  MqttPort* mqtt_ = nullptr;
  SasTokenSource* tokens_ = nullptr;
  RandomSource* rng_ = nullptr;
  SasRefreshSchedule sas_;
  RetryTimer retry_{2000, 120000, 25};
  RetryTimer authRetry_{60000, 1800000, 25};
  uint32_t generation_ = 0;
  uint32_t connectStart_ = 0;
  uint32_t connectedAt_ = 0;
  bool stableReached_ = false;
  bool clientRunning_ = false;
  bool refreshing_ = false;
  uint32_t retryAt_ = 0;
  uint32_t connects_ = 0, sasRenewals_ = 0, failures_ = 0, authFailures_ = 0, eventOverflows_ = 0;
  char lastError_[96] = "";
  char password_[512];
  static const unsigned kPending = 16;
  Event pending_[kPending];
  unsigned pendingHead_ = 0, pendingTail_ = 0;
  bool linkDownPending_ = false;
};

// Percent-encodes everything except RFC 3986 unreserved characters (used for
// MQTT topic property values). Returns false if `out` is too small.
bool urlEncode(const char* in, char* out, size_t cap);

}  // namespace vc
