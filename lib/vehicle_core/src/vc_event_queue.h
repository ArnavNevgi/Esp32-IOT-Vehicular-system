// Sensor-side queue of events awaiting an application-level EventAck.
//
// An event keeps its eventCounter for its whole life, so every retransmission
// (including ones sent in a new session after a re-handshake) is the same
// logical event. Attempts are only counted while the link is up.
#pragma once

#include <stdint.h>

#include "vc_protocol.h"
#include "vc_timing.h"

namespace vc {

template <unsigned N>
class RetransmitQueue {
 public:
  struct Entry {
    proto::EventBody event;
    uint32_t lastSendMs = 0;
    uint32_t order = 0;  // insertion order, for FIFO and drop-oldest
    bool used = false;
  };

  // Adds an event. If the queue is full the oldest entry is dropped (and
  // counted) to make room: newer state transitions supersede older ones.
  void push(const proto::EventBody& e) {
    unsigned slot = N;
    for (unsigned i = 0; i < N; ++i)
      if (!entries_[i].used) {
        slot = i;
        break;
      }
    if (slot == N) {
      slot = oldestIndex();
      ++dropped_;
    }
    entries_[slot].event = e;
    entries_[slot].event.attempt = 0;
    entries_[slot].lastSendMs = 0;
    entries_[slot].used = true;
    entries_[slot].order = nextOrder_++;
  }

  // Returns the oldest entry that has never been sent or whose retry interval
  // has elapsed; nullptr if none.
  Entry* nextDue(uint32_t now, uint32_t retryMs) {
    Entry* best = nullptr;
    for (unsigned i = 0; i < N; ++i) {
      Entry& en = entries_[i];
      if (!en.used) continue;
      if (en.event.attempt != 0 && !hasElapsed(now, en.lastSendMs, retryMs)) continue;
      if (!best || en.order < best->order) best = &en;
    }
    return best;
  }

  void markSent(Entry* e, uint32_t now) {
    if (!e) return;
    if (e->event.attempt < 255) ++e->event.attempt;
    e->lastSendMs = now;
  }

  // Removes entries that used up their attempt budget. Returns how many.
  unsigned expire(uint8_t maxAttempts, uint32_t now, uint32_t retryMs) {
    unsigned n = 0;
    for (unsigned i = 0; i < N; ++i) {
      Entry& en = entries_[i];
      if (en.used && en.event.attempt >= maxAttempts && hasElapsed(now, en.lastSendMs, retryMs)) {
        en.used = false;
        ++n;
      }
    }
    expired_ += n;
    return n;
  }

  bool ack(uint32_t eventCounter) {
    for (unsigned i = 0; i < N; ++i)
      if (entries_[i].used && entries_[i].event.eventCounter == eventCounter) {
        entries_[i].used = false;
        return true;
      }
    return false;
  }

  unsigned size() const {
    unsigned n = 0;
    for (unsigned i = 0; i < N; ++i) n += entries_[i].used ? 1 : 0;
    return n;
  }
  uint32_t dropped() const { return dropped_; }
  uint32_t expired() const { return expired_; }

 private:
  unsigned oldestIndex() const {
    unsigned idx = 0;
    for (unsigned i = 1; i < N; ++i)
      if (entries_[i].order < entries_[idx].order) idx = i;
    return idx;
  }
  Entry entries_[N];
  uint32_t nextOrder_ = 1;
  uint32_t dropped_ = 0;
  uint32_t expired_ = 0;
};

}  // namespace vc
