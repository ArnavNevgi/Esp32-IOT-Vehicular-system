#include "net.h"

#include <WiFi.h>
#include <sys/time.h>
#include <time.h>

#include "gateway_config.h"
#include "vc_util.h"

namespace {
const uint32_t kWifiAttemptTimeoutMs = 20000;
const time_t kMinPlausibleUnix = 1735689600;  // 2025-01-01T00:00:00Z
}  // namespace

bool WifiLink::begin(const char* ssid, const char* password) {
  WiFi.mode(WIFI_STA);
  // Modem sleep makes the station miss ESP-NOW frames between DTIM beacons.
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(false);  // reconnection is paced by this class
  WiFi.persistent(false);        // never write credentials to flash
  if (vc::isPlaceholder(ssid) || vc::isPlaceholder(password)) {
    state_ = State::Unconfigured;
    return false;
  }
  ssid_ = ssid;
  password_ = password;
  startAttempt(millis());
  return true;
}

const char* WifiLink::stateName() const {
  switch (state_) {
    case State::Unconfigured: return "unconfigured";
    case State::Connecting: return "connecting";
    case State::Up: return "connected";
    case State::Backoff: return "backoff";
  }
  return "?";
}

int WifiLink::rssi() const { return state_ == State::Up ? WiFi.RSSI() : 0; }

void WifiLink::startAttempt(uint32_t now) {
  Serial.printf("[wifi] connecting to \"%s\"\n", ssid_);  // SSID only, never the password
  WiFi.disconnect();
  WiFi.begin(ssid_, password_);
  attemptStart_ = now;
  state_ = State::Connecting;
}

void WifiLink::tick(uint32_t now) {
  wl_status_t st = WiFi.status();
  switch (state_) {
    case State::Unconfigured:
      return;
    case State::Connecting:
      if (st == WL_CONNECTED) {
        state_ = State::Up;
        ++connects_;
        retry_.succeeded();
        channel_ = WiFi.channel();
        Serial.printf("[wifi] connected, IP %s, channel %u, RSSI %d dBm\n", WiFi.localIP().toString().c_str(),
                      channel_, WiFi.RSSI());
      } else if (st == WL_CONNECT_FAILED || st == WL_NO_SSID_AVAIL ||
                 vc::hasElapsed(now, attemptStart_, kWifiAttemptTimeoutMs)) {
        ++failures_;
        uint32_t wait = retry_.failed(now, esp_random());
        WiFi.disconnect();
        state_ = State::Backoff;
        Serial.printf("[wifi] connect failed (status %d); retry in %lu ms\n", (int)st, (unsigned long)wait);
      }
      break;
    case State::Up:
      if (st != WL_CONNECTED) {
        Serial.printf("[wifi] connection lost (status %d)\n", (int)st);
        uint32_t wait = retry_.failed(now, esp_random());
        state_ = State::Backoff;
        Serial.printf("[wifi] reconnect in %lu ms\n", (unsigned long)wait);
      } else if (WiFi.channel() != channel_) {
        // The AP moved; the sensor node will lose radio ACKs and re-sweep.
        Serial.printf("[wifi] AP channel changed %u -> %u\n", (unsigned)channel_, (unsigned)WiFi.channel());
        channel_ = WiFi.channel();
      }
      break;
    case State::Backoff:
      if (retry_.ready(now)) startAttempt(now);
      break;
  }
}

bool TimeSync::valid() const { return time(nullptr) >= kMinPlausibleUnix; }

int64_t TimeSync::unixMs() const {
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  return (int64_t)tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

const char* TimeSync::stateName() const {
  if (valid()) return "valid";
  return started_ ? "syncing" : "waiting_for_network";
}

void TimeSync::tick(uint32_t now, bool networkUp) {
  if (valid()) {
    if (!everValid_) {
      everValid_ = true;
      retry_.succeeded();
      Serial.printf("[time] UTC clock synchronised\n");
    }
    return;  // SNTP keeps re-synchronising in the background
  }
  if (!networkUp) return;
  if (!started_) {
    if (!retry_.ready(now)) return;
    // UTC only: no timezone offset, timestamps are emitted with "Z".
    configTime(0, 0, "pool.ntp.org", "time.nist.gov", "time.google.com");
    started_ = true;
    startedAt_ = now;
    Serial.println("[time] SNTP started");
    return;
  }
  if (vc::hasElapsed(now, startedAt_, GW_TIME_SYNC_TIMEOUT_MS)) {
    uint32_t wait = retry_.failed(now, esp_random());
    started_ = false;
    Serial.printf("[time] no plausible time after %u ms; retry in %lu ms\n", (unsigned)GW_TIME_SYNC_TIMEOUT_MS,
                  (unsigned long)wait);
  }
}
