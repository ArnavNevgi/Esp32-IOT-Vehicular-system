// Bounded gateway -> cloud outbox with alert priority and PUBACK tracking.
//
// Policy (documented in README "Queueing"):
//  * Alerts have their own AlertSlots entries; routine summaries can never
//    displace an alert. Summaries use 2 slots (one may be in flight).
//  * A new summary replaces a not-yet-sent summary (coalescing): after an
//    outage only the latest routine summary is published, never a backlog.
//  * When all alert slots are taken, the oldest pending *reminder* is dropped
//    first, then the oldest pending alert. In-flight messages are never dropped.
//  * A message keeps its payload and messageId until it is acknowledged
//    (MQTT PUBACK), so a republish after a disconnect is byte-identical.
//  * Everything is RAM only: queued messages do not survive a power loss.
#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "vc_timing.h"

namespace vc {

enum class CloudKind : uint8_t { Summary, Alert };

template <unsigned PayloadCap>
struct CloudMessage {
  enum class State : uint8_t { Free, Pending, InFlight };
  State state = State::Free;
  CloudKind kind = CloudKind::Summary;
  bool reminder = false;
  char messageId[64] = {0};
  char payload[PayloadCap] = {0};
  uint16_t len = 0;
  int mqttMsgId = -1;
  uint32_t sentAtMs = 0;
  uint32_t order = 0;
  uint8_t attempts = 0;
};

struct CloudQueueStats {
  uint32_t alertsDropped = 0;
  uint32_t remindersDropped = 0;
  uint32_t summariesCoalesced = 0;
  uint32_t rejected = 0;  // oversized, or no slot could be freed
  uint32_t acked = 0;
  uint32_t republished = 0;  // sent again after disconnect / ack timeout
};

template <unsigned AlertSlots, unsigned PayloadCap>
class CloudOutbox {
 public:
  typedef CloudMessage<PayloadCap> Message;
  static const unsigned kSummarySlots = 2;
  static const unsigned kSlots = AlertSlots + kSummarySlots;

  bool pushAlert(const char* messageId, const char* payload, size_t len, bool reminder) {
    if (!fits(messageId, len)) return false;
    Message* slot = freeSlot(CloudKind::Alert);
    if (!slot) slot = evictForAlert();
    if (!slot) {
      ++stats_.rejected;
      return false;
    }
    fill(slot, CloudKind::Alert, messageId, payload, len, reminder);
    return true;
  }

  bool pushSummary(const char* messageId, const char* payload, size_t len) {
    if (!fits(messageId, len)) return false;
    Message* slot = nullptr;
    for (unsigned i = 0; i < kSlots; ++i)
      if (msgs_[i].state == Message::State::Pending && msgs_[i].kind == CloudKind::Summary) {
        slot = &msgs_[i];
        ++stats_.summariesCoalesced;
        break;
      }
    if (!slot) slot = freeSlot(CloudKind::Summary);
    if (!slot) {
      // Both summary slots are in flight: forget the older one's tracking.
      slot = oldest(CloudKind::Summary, Message::State::InFlight);
      ++stats_.summariesCoalesced;
    }
    fill(slot, CloudKind::Summary, messageId, payload, len, false);
    return true;
  }

  // Next message to publish: oldest pending alert, else the pending summary.
  // Returns nullptr when nothing is pending or maxInFlight is reached.
  Message* nextToSend(unsigned maxInFlight) {
    if (inFlight() >= maxInFlight) return nullptr;
    Message* m = oldest(CloudKind::Alert, Message::State::Pending);
    if (!m) m = oldest(CloudKind::Summary, Message::State::Pending);
    return m;
  }

  void markInFlight(Message* m, int mqttMsgId, uint32_t now) {
    if (m->attempts > 0) ++stats_.republished;
    if (m->attempts < 255) ++m->attempts;
    m->state = Message::State::InFlight;
    m->mqttMsgId = mqttMsgId;
    m->sentAtMs = now;
  }

  // The client refused the message (e.g. outbox full); try again later.
  void markSendFailed(Message* m) {
    m->state = Message::State::Pending;
    m->mqttMsgId = -1;
  }

  // MQTT PUBACK received. Returns true if it matched an in-flight message.
  bool onPublished(int mqttMsgId) {
    for (unsigned i = 0; i < kSlots; ++i)
      if (msgs_[i].state == Message::State::InFlight && msgs_[i].mqttMsgId == mqttMsgId) {
        msgs_[i].state = Message::State::Free;
        ++stats_.acked;
        return true;
      }
    return false;
  }

  // The client dropped a message without PUBACK (e.g. outbox expiry).
  bool onDeleted(int mqttMsgId) {
    for (unsigned i = 0; i < kSlots; ++i)
      if (msgs_[i].state == Message::State::InFlight && msgs_[i].mqttMsgId == mqttMsgId) {
        markSendFailed(&msgs_[i]);
        return true;
      }
    return false;
  }

  // Connection lost or client recreated: unacknowledged messages become pending.
  void onDisconnected() {
    for (unsigned i = 0; i < kSlots; ++i)
      if (msgs_[i].state == Message::State::InFlight) markSendFailed(&msgs_[i]);
  }

  unsigned requeueTimedOut(uint32_t now, uint32_t timeoutMs) {
    unsigned n = 0;
    for (unsigned i = 0; i < kSlots; ++i)
      if (msgs_[i].state == Message::State::InFlight && hasElapsed(now, msgs_[i].sentAtMs, timeoutMs)) {
        markSendFailed(&msgs_[i]);
        ++n;
      }
    return n;
  }

  unsigned depth() const { return count(Message::State::Pending) + count(Message::State::InFlight); }
  unsigned inFlight() const { return count(Message::State::InFlight); }
  unsigned pendingAlerts() const {
    unsigned n = 0;
    for (unsigned i = 0; i < kSlots; ++i)
      n += (msgs_[i].kind == CloudKind::Alert && msgs_[i].state != Message::State::Free) ? 1 : 0;
    return n;
  }
  const CloudQueueStats& stats() const { return stats_; }

 private:
  bool fits(const char* messageId, size_t len) {
    if (len == 0 || len >= PayloadCap || strlen(messageId) >= sizeof(msgs_[0].messageId)) {
      ++stats_.rejected;
      return false;
    }
    return true;
  }

  Message* freeSlot(CloudKind kind) {
    unsigned used = 0;
    Message* free = nullptr;
    for (unsigned i = 0; i < kSlots; ++i) {
      if (msgs_[i].state == Message::State::Free) {
        if (!free) free = &msgs_[i];
      } else if (msgs_[i].kind == kind) {
        ++used;
      }
    }
    unsigned limit = kind == CloudKind::Alert ? AlertSlots : kSummarySlots;
    return used < limit ? free : nullptr;
  }

  Message* evictForAlert() {
    Message* victim = nullptr;
    for (unsigned i = 0; i < kSlots; ++i) {
      Message& m = msgs_[i];
      if (m.kind == CloudKind::Alert && m.state == Message::State::Pending && m.reminder &&
          (!victim || m.order < victim->order))
        victim = &m;
    }
    if (victim) {
      ++stats_.remindersDropped;
    } else {
      victim = oldest(CloudKind::Alert, Message::State::Pending);
      if (victim) ++stats_.alertsDropped;
    }
    if (victim) victim->state = Message::State::Free;
    return victim;
  }

  Message* oldest(CloudKind kind, typename Message::State state) {
    Message* best = nullptr;
    for (unsigned i = 0; i < kSlots; ++i) {
      Message& m = msgs_[i];
      if (m.kind == kind && m.state == state && (!best || m.order < best->order)) best = &m;
    }
    return best;
  }

  unsigned count(typename Message::State state) const {
    unsigned n = 0;
    for (unsigned i = 0; i < kSlots; ++i) n += msgs_[i].state == state ? 1 : 0;
    return n;
  }

  void fill(Message* m, CloudKind kind, const char* messageId, const char* payload, size_t len, bool reminder) {
    m->state = Message::State::Pending;
    m->kind = kind;
    m->reminder = reminder;
    strncpy(m->messageId, messageId, sizeof(m->messageId) - 1);
    m->messageId[sizeof(m->messageId) - 1] = '\0';
    memcpy(m->payload, payload, len);
    m->payload[len] = '\0';
    m->len = (uint16_t)len;
    m->mqttMsgId = -1;
    m->attempts = 0;
    m->order = nextOrder_++;
  }

  Message msgs_[kSlots];
  uint32_t nextOrder_ = 1;
  CloudQueueStats stats_;
};

}  // namespace vc
