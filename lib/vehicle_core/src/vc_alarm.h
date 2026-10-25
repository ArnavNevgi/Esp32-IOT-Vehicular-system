// Debounced alarm state machines with hysteresis and controlled reminders.
#pragma once

#include <math.h>
#include <stdint.h>

#include "vc_timing.h"

namespace vc {

enum class AlarmTransition : uint8_t { None, Activated, Cleared };

// Boolean condition alarm: the condition must hold continuously for activateMs
// to activate, and be absent continuously for clearMs to clear.
class DebouncedAlarm {
 public:
  DebouncedAlarm(uint32_t activateMs, uint32_t clearMs) : activateMs_(activateMs), clearMs_(clearMs) {}

  AlarmTransition update(bool condition, uint32_t now) {
    if (condition == active_) {
      pending_ = false;
      return AlarmTransition::None;
    }
    if (!pending_) {
      pending_ = true;
      pendingSince_ = now;
    }
    if (!hasElapsed(now, pendingSince_, active_ ? clearMs_ : activateMs_)) return AlarmTransition::None;
    pending_ = false;
    active_ = !active_;
    changedAt_ = now;
    return active_ ? AlarmTransition::Activated : AlarmTransition::Cleared;
  }

  bool active() const { return active_; }
  uint32_t changedAt() const { return changedAt_; }

 private:
  uint32_t activateMs_;
  uint32_t clearMs_;
  bool active_ = false;
  bool pending_ = false;
  uint32_t pendingSince_ = 0;
  uint32_t changedAt_ = 0;
};

// High-level threshold alarm with hysteresis: activates at value >= onThreshold,
// clears at value <= offThreshold (offThreshold < onThreshold). Invalid samples
// (sensor failure, non-finite) freeze the state machine; they neither raise nor
// clear the alarm, and any pending transition restarts on the next valid sample.
class HysteresisAlarm {
 public:
  HysteresisAlarm(float onThreshold, float offThreshold, uint32_t activateMs, uint32_t clearMs)
      : on_(onThreshold), off_(offThreshold), debounced_(activateMs, clearMs) {}

  AlarmTransition update(float value, bool valid, uint32_t now) {
    if (!valid || !isfinite(value)) {
      // Re-arm the debounce so a pending transition needs fresh valid samples.
      debounced_.update(debounced_.active(), now);
      return AlarmTransition::None;
    }
    bool condition = debounced_.active() ? (value > off_) : (value >= on_);
    AlarmTransition t = debounced_.update(condition, now);
    if (t == AlarmTransition::Activated || (debounced_.active() && value > peak_)) peak_ = value;
    return t;
  }

  bool active() const { return debounced_.active(); }
  uint32_t changedAt() const { return debounced_.changedAt(); }
  // Largest value seen during the current (or most recent) active period.
  float peak() const { return peak_; }

 private:
  float on_;
  float off_;
  DebouncedAlarm debounced_;
  float peak_ = 0;
};

// Emits at most one reminder per interval while an alarm stays active.
class ReminderTimer {
 public:
  explicit ReminderTimer(uint32_t intervalMs) : interval_(intervalMs) {}
  void setInterval(uint32_t intervalMs) { interval_ = intervalMs; }
  void arm(uint32_t now) {
    last_ = now;
    count_ = 0;
    armed_ = true;
  }
  void disarm() { armed_ = false; }
  bool due(uint32_t now) {
    if (!armed_ || interval_ == 0 || !hasElapsed(now, last_, interval_)) return false;
    last_ = now;
    ++count_;
    return true;
  }
  uint32_t count() const { return count_; }

 private:
  uint32_t interval_;
  uint32_t last_ = 0;
  uint32_t count_ = 0;
  bool armed_ = false;
};

}  // namespace vc
