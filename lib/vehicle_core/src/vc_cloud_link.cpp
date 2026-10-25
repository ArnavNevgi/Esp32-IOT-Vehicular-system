#include "vc_cloud_link.h"

#include <stdio.h>
#include <string.h>

#include "vc_log.h"
#include "vc_sha256.h"

namespace vc {

bool urlEncode(const char* in, char* out, size_t cap) {
  static const char hex[] = "0123456789ABCDEF";
  if (in == nullptr || out == nullptr || cap == 0) return false;
  size_t o = 0;
  for (const char* p = in; *p; ++p) {
    unsigned char c = (unsigned char)*p;
    bool plain = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '-' || c == '_' ||
                 c == '.' || c == '~';
    if (o + (plain ? 1 : 3) >= cap) return false;
    if (plain) {
      out[o++] = (char)c;
    } else {
      out[o++] = '%';
      out[o++] = hex[c >> 4];
      out[o++] = hex[c & 15];
    }
  }
  out[o] = '\0';
  return true;
}

const char* CloudLinkCore::stateName() const {
  switch (state_) {
    case State::Unconfigured: return "unconfigured";
    case State::WaitNetwork: return "waiting_for_network";
    case State::Connecting: return "connecting";
    case State::Connected: return "connected";
    case State::Backoff: return "backoff";
  }
  return "?";
}

bool CloudLinkCore::begin(MqttPort* mqtt, SasTokenSource* tokens, RandomSource* rng, const CloudLinkConfig& cfg) {
  state_ = State::Unconfigured;
  if (!mqtt || !tokens || !rng || cfg.sasLifetimeSec == 0) return false;
  mqtt_ = mqtt;
  tokens_ = tokens;
  rng_ = rng;
  cfg_ = cfg;
  retry_ = RetryTimer(cfg.retryBaseMs, cfg.retryCapMs, 25);
  authRetry_ = RetryTimer(cfg.authRetryBaseMs, cfg.authRetryCapMs, 25);
  state_ = State::WaitNetwork;
  return true;
}

void CloudLinkCore::pushEvent(Event::Kind kind, int msgId) {
  if (pendingHead_ - pendingTail_ >= kPending) {
    ++pendingTail_;  // full: oldest lost; that message's ack timeout re-sends it
    ++eventOverflows_;
  }
  pending_[pendingHead_ % kPending] = {kind, msgId};
  ++pendingHead_;
}

void CloudLinkCore::startConnect(uint32_t now) {
  if (!tokens_->generate(cfg_.sasLifetimeSec, password_, sizeof(password_))) {
    secureZero(password_, sizeof(password_));
    snprintf(lastError_, sizeof(lastError_), "SAS token generation failed");
    fail(now, false);
    return;
  }
  sas_.onGenerated(now, cfg_.sasLifetimeSec, cfg_.refreshMarginSec);
  ++generation_;
  bool started = mqtt_->start(generation_, password_);
  secureZero(password_, sizeof(password_));  // the client keeps its own copy
  clientRunning_ = true;
  if (!started) {
    snprintf(lastError_, sizeof(lastError_), "MQTT client start failed");
    teardown(nullptr);
    fail(now, false);
    return;
  }
  connectStart_ = now;
  state_ = State::Connecting;
  logf("[azure] connecting (client generation %lu)%s", (unsigned long)generation_,
       refreshing_ ? " for SAS renewal" : "");
}

void CloudLinkCore::teardown(const char* reason) {
  if (clientRunning_) {
    mqtt_->stop();
    clientRunning_ = false;
  }
  ++generation_;  // invalidate anything still queued from the old client
  sas_.invalidate();
  if (state_ == State::Connected || state_ == State::Connecting) linkDownPending_ = true;
  if (reason) logf("[azure] disconnected: %s", reason);
}

void CloudLinkCore::fail(uint32_t now, bool authFailure) {
  ++failures_;
  if (authFailure) ++authFailures_;
  refreshing_ = false;
  uint32_t wait = authFailure ? authRetry_.failed(now, rng_->random32()) : retry_.failed(now, rng_->random32());
  retryAt_ = now + wait;
  state_ = State::Backoff;
  logf("[azure] %s; retry in %lu s", lastError_, (unsigned long)(wait / 1000));
}

void CloudLinkCore::onEventsLost(uint32_t now) {
  ++eventOverflows_;
  if (state_ != State::Connected && state_ != State::Connecting) return;
  snprintf(lastError_, sizeof(lastError_), "MQTT event queue overflow; restarting connection");
  teardown(nullptr);
  fail(now, false);
}

void CloudLinkCore::onMqttEvent(const MqttEvent& ev, uint32_t now) {
  if (state_ == State::Unconfigured || ev.generation != generation_) return;  // from a stopped client
  switch (ev.kind) {
    case MqttEvent::Kind::Connected:
      if (state_ == State::Connecting) {
        state_ = State::Connected;
        ++connects_;
        if (refreshing_) ++sasRenewals_;
        refreshing_ = false;
        authRetry_.succeeded();  // the credentials work
        connectedAt_ = now;
        stableReached_ = false;
        lastError_[0] = '\0';
        logf("[azure] connected (MQTT CONNACK accepted); token refresh in %lu s",
             (unsigned long)(sas_.remainingMs(now) / 1000));
      }
      return;
    case MqttEvent::Kind::Refused: {
      bool auth = ev.refuseCode == kConnackBadCredentials || ev.refuseCode == kConnackNotAuthorized;
      static const char* const kNames[] = {"accepted", "protocol", "client id rejected", "server unavailable",
                                           "bad username/password", "not authorized"};
      const char* name = ev.refuseCode >= 0 && ev.refuseCode <= 5 ? kNames[ev.refuseCode] : "unknown";
      snprintf(lastError_, sizeof(lastError_), "CONNECT refused: %s%s", name,
               auth ? " (check device id/key and clock)" : "");
      teardown(nullptr);
      fail(now, auth);
      return;
    }
    case MqttEvent::Kind::TransportError:
      snprintf(lastError_, sizeof(lastError_), "transport/TLS error: esp_err=0x%x tls_stack=-0x%x errno=%d",
               (unsigned)ev.tlsErr, (unsigned)(-ev.tlsStackErr), ev.sockErrno);
      teardown(nullptr);
      fail(now, false);
      return;
    case MqttEvent::Kind::Disconnected:
      if (state_ == State::Connected || state_ == State::Connecting) {
        snprintf(lastError_, sizeof(lastError_), "broker connection closed");
        teardown(nullptr);
        fail(now, false);
      }
      return;
    case MqttEvent::Kind::Published:
      pushEvent(Event::Kind::Published, ev.msgId);
      return;
    case MqttEvent::Kind::Deleted:
      pushEvent(Event::Kind::Deleted, ev.msgId);
      return;
  }
}

void CloudLinkCore::tick(uint32_t now, bool networkUp, bool timeValid) {
  switch (state_) {
    case State::Unconfigured:
      return;
    case State::WaitNetwork:
      // Backoff timers still apply, so network flapping cannot bypass the
      // slow-down after an authentication failure.
      if (networkUp && timeValid && retry_.ready(now) && authRetry_.ready(now)) startConnect(now);
      return;
    case State::Backoff:
      if (!networkUp || !timeValid) {
        state_ = State::WaitNetwork;
      } else if (retry_.ready(now) && authRetry_.ready(now)) {
        startConnect(now);
      }
      return;
    case State::Connecting:
      if (!networkUp) {
        teardown("network lost");
        state_ = State::WaitNetwork;
      } else if (hasElapsed(now, connectStart_, cfg_.connectTimeoutMs)) {
        snprintf(lastError_, sizeof(lastError_), "no CONNACK within %lu ms", (unsigned long)cfg_.connectTimeoutMs);
        teardown(nullptr);
        fail(now, false);
      }
      return;
    case State::Connected:
      if (!networkUp) {
        teardown("network lost");
        state_ = State::WaitNetwork;
      } else if (sas_.due(now)) {
        teardown("SAS token renewal");
        refreshing_ = true;
        startConnect(now);
      } else if (!stableReached_ && hasElapsed(now, connectedAt_, cfg_.stableMs)) {
        stableReached_ = true;
        retry_.succeeded();
      }
      return;
  }
}

int CloudLinkCore::publish(const char* messageId, const char* messageType, const char* payload, size_t len,
                           size_t* wireBytes) {
  if (state_ != State::Connected) return -1;
  return mqtt_->publish(messageId, messageType, payload, len, wireBytes);
}

bool CloudLinkCore::pollEvent(Event& out) {
  // Acknowledgements received before a disconnect are delivered first, so a
  // message that did reach IoT Hub is not re-sent after the reconnect.
  if (pendingTail_ != pendingHead_) {
    out = pending_[pendingTail_ % kPending];
    ++pendingTail_;
    return true;
  }
  if (linkDownPending_) {
    linkDownPending_ = false;
    out = {Event::Kind::LinkDown, -1};
    return true;
  }
  return false;
}

}  // namespace vc
