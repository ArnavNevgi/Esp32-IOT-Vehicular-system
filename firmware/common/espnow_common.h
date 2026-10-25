// ESP-NOW / platform helpers shared by both boards: key/MAC configuration,
// bounded queues filled from Wi-Fi task callbacks, an NVS-backed counter and
// the hardware RNG, as implementations of the vc:: port interfaces.
#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include "vc_ports.h"
#include "vc_protocol.h"

struct LinkConfig {
  bool ok = false;
  const char* problem = "";
  uint8_t pmk[16] = {0};
  uint8_t lmk[16] = {0};
  uint8_t sensorMac[6] = {0};
  uint8_t gatewayMac[6] = {0};
};

// Validates the ESP-NOW keys and peer MACs from secrets.h. Refuses
// placeholders, malformed values, all-zero keys and PMK == LMK.
LinkConfig loadLinkConfig();

void formatMac(const uint8_t mac[6], char out[18]);

// Routes vc::logf() lines to the serial console.
void installSerialLog();

// One received ESP-NOW frame, copied out of the Wi-Fi task.
struct RxFrame {
  uint8_t mac[6];
  uint8_t len;
  int8_t rssi;
  uint8_t channel;
  uint8_t data[vc::proto::kMaxPacketSize];
};

class RxQueue {
 public:
  bool begin(unsigned depth);
  // Called from the ESP-NOW receive callback (Wi-Fi task). Never blocks;
  // oversize frames and a full queue are counted, not queued.
  void pushFromCallback(const uint8_t* mac, const uint8_t* data, int len, int8_t rssi, uint8_t channel);
  bool pop(RxFrame& out);
  uint32_t overflows() const { return overflows_; }
  uint32_t oversize() const { return oversize_; }
  unsigned highWater() const { return highWater_; }

 private:
  QueueHandle_t q_ = nullptr;
  volatile uint32_t overflows_ = 0;
  volatile uint32_t oversize_ = 0;
  volatile unsigned highWater_ = 0;
};

// Radio-level send results (ESP-NOW send callback, Wi-Fi task -> loop).
class SendResultQueue {
 public:
  bool begin(unsigned depth);
  void pushFromCallback(bool delivered);
  bool pop(bool& delivered);
  uint32_t overflows() const { return overflows_; }

 private:
  QueueHandle_t q_ = nullptr;
  volatile uint32_t overflows_ = 0;
};

// A uint32_t in NVS. store() returns true only after the value was written,
// committed and read back. NVS replaces an entry atomically, so a power loss
// leaves either the old or the new value.
class NvsCounterStore : public vc::CounterStore {
 public:
  NvsCounterStore(const char* nvsNamespace, const char* key) : ns_(nvsNamespace), key_(key) {}
  bool load(uint32_t& value) override;
  bool store(uint32_t value) override;

 private:
  const char* ns_;
  const char* key_;
};

class EspRandom : public vc::RandomSource {
 public:
  uint32_t random32() override { return esp_random(); }
};

// Increments the boot counter in NVS namespace `nvsNamespace` (key "boot");
// 0 if NVS failed or the counter is exhausted.
uint32_t nextBootCounter(const char* nvsNamespace);
