// Gateway upstream connectivity: Wi-Fi station management and NTP time.
// Neither class ever blocks the loop; failures back off with jitter.
#pragma once

#include <Arduino.h>

#include "vc_backoff.h"
#include "vc_ports.h"

class WifiLink {
 public:
  enum class State : uint8_t { Unconfigured, Connecting, Up, Backoff };

  // Returns false (state Unconfigured) for placeholder credentials.
  bool begin(const char* ssid, const char* password);
  void tick(uint32_t now);

  bool up() const { return state_ == State::Up; }
  State state() const { return state_; }
  const char* stateName() const;
  int rssi() const;
  uint8_t channel() const { return channel_; }
  uint32_t connects() const { return connects_; }
  uint32_t failures() const { return failures_; }

 private:
  void startAttempt(uint32_t now);
  State state_ = State::Unconfigured;
  const char* ssid_ = "";
  const char* password_ = "";
  uint32_t attemptStart_ = 0;
  uint8_t channel_ = 0;
  uint32_t connects_ = 0, failures_ = 0;
  vc::RetryTimer retry_{2000, 60000, 25};
};

class TimeSync : public vc::WallClock {
 public:
  void tick(uint32_t now, bool networkUp);
  // True once the system clock holds a plausible UTC time (>= 2025-01-01).
  bool valid() const override;
  int64_t unixMs() const override;
  const char* stateName() const;

 private:
  bool started_ = false;
  bool everValid_ = false;
  uint32_t startedAt_ = 0;
  vc::RetryTimer retry_{5000, 300000, 25};
};
