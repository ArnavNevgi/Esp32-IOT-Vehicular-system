// Capped exponential backoff with jitter, plus a small retry timer built on it.
#pragma once

#include <stdint.h>

#include "vc_timing.h"

namespace vc {

class Backoff {
 public:
  // jitterPct: up to this percentage of the computed delay is randomly removed
  // ("equal jitter"), so simultaneous retries from several devices spread out.
  Backoff(uint32_t baseMs, uint32_t capMs, uint8_t jitterPct = 25)
      : base_(baseMs), cap_(capMs < baseMs ? baseMs : capMs), jitter_(jitterPct > 100 ? 100 : jitterPct) {}

  // Delay to wait before the next attempt; advances the attempt counter.
  uint32_t next(uint32_t rand32) {
    uint32_t d = base_;
    for (uint32_t i = 0; i < attempts_ && d < cap_; ++i) {
      d = (d > cap_ / 2) ? cap_ : d * 2;
    }
    if (d > cap_) d = cap_;
    uint32_t span = (uint32_t)(((uint64_t)d * jitter_) / 100u);
    if (span > 0) d -= rand32 % (span + 1);
    if (attempts_ < 0xFFFFu) ++attempts_;
    return d;
  }

  void reset() { attempts_ = 0; }
  uint32_t attempts() const { return attempts_; }
  uint32_t cap() const { return cap_; }

 private:
  uint32_t base_;
  uint32_t cap_;
  uint8_t jitter_;
  uint32_t attempts_ = 0;
};

// Tracks when the next attempt is allowed.
class RetryTimer {
 public:
  RetryTimer(uint32_t baseMs, uint32_t capMs, uint8_t jitterPct = 25) : backoff_(baseMs, capMs, jitterPct) {}

  bool ready(uint32_t now) const { return !waiting_ || reached(now, retryAt_); }

  // Records a failure at `now` and schedules the next attempt. Returns the delay.
  uint32_t failed(uint32_t now, uint32_t rand32) {
    uint32_t d = backoff_.next(rand32);
    retryAt_ = now + d;
    waiting_ = true;
    return d;
  }

  void succeeded() {
    backoff_.reset();
    waiting_ = false;
  }

  uint32_t attempts() const { return backoff_.attempts(); }
  uint32_t retryAt() const { return retryAt_; }

 private:
  Backoff backoff_;
  uint32_t retryAt_ = 0;
  bool waiting_ = false;
};

}  // namespace vc
