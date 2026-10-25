// Wrap-safe monotonic scheduling helpers.
//
// All scheduling in this project uses a uint32_t millisecond counter (Arduino
// millis()) that wraps every ~49.7 days. Comparisons are done on the unsigned
// difference, which stays correct across the wrap as long as intervals are
// shorter than 2^31 ms (~24.8 days).
#pragma once

#include <stdint.h>

namespace vc {

inline uint32_t elapsedMs(uint32_t now, uint32_t since) { return now - since; }

inline bool hasElapsed(uint32_t now, uint32_t since, uint32_t interval) {
  return (uint32_t)(now - since) >= interval;
}

// True when `now` is at or after `deadline` (wrap-safe, deadline < 2^31 ms away).
inline bool reached(uint32_t now, uint32_t deadline) { return (int32_t)(now - deadline) >= 0; }

// Fixed-rate timer. If the caller falls behind by more than one period the
// schedule is re-anchored to `now` instead of firing a burst of catch-up ticks;
// the number of skipped periods is counted.
class PeriodicTimer {
 public:
  explicit PeriodicTimer(uint32_t periodMs = 1000) : period_(periodMs) {}

  void setPeriod(uint32_t periodMs) { period_ = periodMs; }
  uint32_t period() const { return period_; }

  // First tick fires one period after `now` (or immediately if fireNow).
  void start(uint32_t now, bool fireNow = false) {
    next_ = fireNow ? now : now + period_;
    started_ = true;
  }

  bool due(uint32_t now) {
    if (!started_) {
      start(now);
      return false;
    }
    if (!reached(now, next_)) return false;
    next_ += period_;
    if (reached(now, next_)) {
      // More than one period late: drop the backlog.
      uint32_t late = now - next_;
      skipped_ += 1 + late / (period_ ? period_ : 1);
      next_ = now + period_;
    }
    return true;
  }

  uint32_t skipped() const { return skipped_; }
  uint32_t nextDeadline() const { return next_; }

 private:
  uint32_t period_;
  uint32_t next_ = 0;
  uint32_t skipped_ = 0;
  bool started_ = false;
};

}  // namespace vc
