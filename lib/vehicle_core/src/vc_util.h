// Small pure helpers: statistics windows, SAS refresh scheduling, configuration
// validation, GPIO constraint checks, UTC formatting and metering estimates.
#pragma once

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "vc_timing.h"

namespace vc {

// Min / max / mean / latest over a reporting window. Non-finite samples are
// ignored (and counted) rather than poisoning the aggregate.
struct WindowStats {
  uint32_t count = 0;
  uint32_t invalid = 0;
  float minV = 0, maxV = 0, last = 0;
  double sum = 0;

  void add(float v) {
    if (!isfinite(v)) {
      ++invalid;
      return;
    }
    if (count == 0 || v < minV) minV = v;
    if (count == 0 || v > maxV) maxV = v;
    last = v;
    sum += v;
    ++count;
  }
  void addInvalid() { ++invalid; }
  bool empty() const { return count == 0; }
  float mean() const { return count ? (float)(sum / count) : NAN; }
  void reset() { *this = WindowStats(); }
};

// Decides when to regenerate an Azure SAS token. Uses the monotonic clock so a
// later NTP step cannot postpone the refresh past the token's real expiry.
class SasRefreshSchedule {
 public:
  void onGenerated(uint32_t nowMs, uint32_t lifetimeSec, uint32_t marginSec) {
    uint32_t refreshAfterSec = lifetimeSec > marginSec ? lifetimeSec - marginSec : lifetimeSec / 2;
    generatedMs_ = nowMs;
    refreshAfterMs_ = refreshAfterSec * 1000u;
    valid_ = true;
  }
  void invalidate() { valid_ = false; }
  bool valid() const { return valid_; }
  bool due(uint32_t nowMs) const { return !valid_ || hasElapsed(nowMs, generatedMs_, refreshAfterMs_); }
  uint32_t remainingMs(uint32_t nowMs) const {
    if (due(nowMs)) return 0;
    return refreshAfterMs_ - (nowMs - generatedMs_);
  }

 private:
  uint32_t generatedMs_ = 0;
  uint32_t refreshAfterMs_ = 0;
  bool valid_ = false;
};

// Configuration values shipped in secrets.example.h start with "YOUR_" or are
// empty; firmware refuses to use them so it never attempts a login with them.
inline bool isPlaceholder(const char* s) { return s == nullptr || s[0] == '\0' || strncmp(s, "YOUR_", 5) == 0; }

inline bool hexNibble(char c, uint8_t& v) {
  if (c >= '0' && c <= '9') v = (uint8_t)(c - '0');
  else if (c >= 'a' && c <= 'f') v = (uint8_t)(c - 'a' + 10);
  else if (c >= 'A' && c <= 'F') v = (uint8_t)(c - 'A' + 10);
  else return false;
  return true;
}

// Parses exactly 2*N hex digits (no separators). Rejects all-zero keys.
inline bool parseHexKey(const char* hex, uint8_t* out, size_t n) {
  if (hex == nullptr || strlen(hex) != n * 2) return false;
  bool nonZero = false;
  for (size_t i = 0; i < n; ++i) {
    uint8_t hi, lo;
    if (!hexNibble(hex[2 * i], hi) || !hexNibble(hex[2 * i + 1], lo)) return false;
    out[i] = (uint8_t)((hi << 4) | lo);
    nonZero = nonZero || out[i] != 0;
  }
  return nonZero;
}

// Parses "AA:BB:CC:DD:EE:FF". Rejects broadcast, multicast and all-zero.
inline bool parseMac(const char* s, uint8_t out[6]) {
  if (s == nullptr || strlen(s) != 17) return false;
  bool allZero = true;
  for (int i = 0; i < 6; ++i) {
    uint8_t hi, lo;
    if (!hexNibble(s[i * 3], hi) || !hexNibble(s[i * 3 + 1], lo)) return false;
    if (i < 5 && s[i * 3 + 2] != ':') return false;
    out[i] = (uint8_t)((hi << 4) | lo);
    allZero = allZero && out[i] == 0;
  }
  return !allZero && (out[0] & 0x01) == 0;
}

// --- Classic ESP32 (ESP32-D0WD, e.g. DevKitC / esp32dev) GPIO constraints ---
constexpr bool gpioExists(int pin) { return pin >= 0 && pin <= 39 && pin != 20 && pin != 24 && !(pin >= 28 && pin <= 31); }
// GPIO6-11 are wired to the module's SPI flash.
constexpr bool gpioIsFlash(int pin) { return pin >= 6 && pin <= 11; }
constexpr bool gpioInputOnly(int pin) { return pin >= 34 && pin <= 39; }
// Boot-mode strapping pins; avoid driving them from external circuits at reset.
constexpr bool gpioIsStrapping(int pin) { return pin == 0 || pin == 2 || pin == 5 || pin == 12 || pin == 15; }
// ADC2 cannot be read while Wi-Fi is running; only ADC1 (GPIO32-39) is usable.
constexpr bool gpioIsAdc1(int pin) { return pin >= 32 && pin <= 39; }

constexpr bool gpioUsableInput(int pin) { return gpioExists(pin) && !gpioIsFlash(pin); }
constexpr bool gpioUsableOutput(int pin) { return gpioUsableInput(pin) && !gpioInputOnly(pin); }
// GPIO34-39 have no internal pull-up/pull-down resistors.
constexpr bool gpioHasInternalPull(int pin) { return gpioUsableOutput(pin); }

// True if no two of the given pins (ignoring negative = "not used") are equal.
template <unsigned N>
constexpr bool pinsDistinct(const int (&pins)[N]) {
  for (unsigned i = 0; i < N; ++i)
    for (unsigned j = i + 1; j < N; ++j)
      if (pins[i] >= 0 && pins[i] == pins[j]) return false;
  return true;
}

// "2026-10-25T12:34:56.789Z" from Unix milliseconds (proleptic Gregorian, UTC).
inline bool formatUtcIso8601(int64_t unixMs, char* out, size_t cap) {
  if (cap < 25 || unixMs < 0) return false;
  int64_t secs = unixMs / 1000;
  int ms = (int)(unixMs % 1000);
  int64_t days = secs / 86400;
  int rem = (int)(secs % 86400);
  // Howard Hinnant's civil_from_days.
  int64_t z = days + 719468;
  int64_t era = z / 146097;
  unsigned doe = (unsigned)(z - era * 146097);
  unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
  int64_t y = (int64_t)yoe + era * 400;
  unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
  unsigned mp = (5 * doy + 2) / 153;
  unsigned d = doy - (153 * mp + 2) / 5 + 1;
  unsigned m = mp < 10 ? mp + 3 : mp - 9;
  if (m <= 2) ++y;
  if (y > 9999) return false;
  int hh = rem / 3600, mm = (rem % 3600) / 60, ss = rem % 60;
  // Manual formatting keeps this free of locale / printf differences.
  const int v[7] = {(int)y, (int)m, (int)d, hh, mm, ss, ms};
  const int w[7] = {4, 2, 2, 2, 2, 2, 3};
  const char sep[7] = {'-', '-', 'T', ':', ':', '.', 'Z'};
  size_t p = 0;
  for (int f = 0; f < 7; ++f) {
    int val = v[f];
    for (int k = w[f] - 1; k >= 0; --k) {
      out[p + k] = (char)('0' + val % 10);
      val /= 10;
    }
    p += w[f];
    out[p++] = sep[f];
  }
  out[p] = '\0';
  return true;
}

// Azure IoT Hub meters device-to-cloud messages in blocks (0.5 KB on the free
// tier, 4 KB on basic/standard tiers). Local estimate only.
inline uint32_t meterUnits(size_t bytes, size_t blockBytes) {
  if (blockBytes == 0) return 1;
  return bytes == 0 ? 1 : (uint32_t)((bytes + blockBytes - 1) / blockBytes);
}

}  // namespace vc
