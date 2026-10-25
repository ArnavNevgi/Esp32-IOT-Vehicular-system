// Small interfaces between the hardware-independent cores and the platform.
//
// The firmware implements them with ESP-NOW, NVS (Preferences), esp_random(),
// esp-mqtt and the SNTP clock; the host integration harness implements them
// with a simulated radio, an in-memory store, a seeded PRNG and a scripted
// MQTT broker. Keep them minimal: application logic belongs in the cores.
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace vc {

class RandomSource {
 public:
  virtual ~RandomSource() {}
  // Firmware: esp_random() (hardware RNG once the radio is on).
  virtual uint32_t random32() = 0;
  uint32_t nonZero() {
    uint32_t r;
    do r = random32();
    while (r == 0);
    return r;
  }
};

// Unicast to the one configured peer. Returns true if the radio accepted the
// frame for transmission (not that the peer received it).
class LinkRadio {
 public:
  virtual ~LinkRadio() {}
  virtual bool send(const uint8_t* data, size_t len) = 0;
  // Sensor node only: retune for the channel sweep.
  virtual void setChannel(uint8_t channel) { (void)channel; }
};

// Persistent single counter (NVS on the boards).
class CounterStore {
 public:
  virtual ~CounterStore() {}
  virtual bool load(uint32_t& value) = 0;
  // Must return true only once the value is durably committed.
  virtual bool store(uint32_t value) = 0;
};

// UTC wall clock (SNTP on the gateway).
class WallClock {
 public:
  virtual ~WallClock() {}
  virtual bool valid() const = 0;  // plausible synchronised UTC time
  virtual int64_t unixMs() const = 0;
};

}  // namespace vc
