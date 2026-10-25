// ESP-NOW adapter for the sensor node's link (logic in vc::SensorLinkCore).
//
// The ESP-NOW callbacks run in the Wi-Fi task: they only copy frames and send
// results into bounded FreeRTOS queues. tick() drains both from the loop and
// then runs the core, so the core is never touched from two tasks.
#pragma once

#include <Arduino.h>
#include <esp_now.h>

#include "espnow_common.h"
#include "vc_sensor_link.h"

class LinkClient : public vc::LinkRadio {
 public:
  // Returns false (and the core stays Disabled) if keys/MACs are invalid or
  // the radio could not be initialised. Local sensing is unaffected.
  bool begin(const LinkConfig& cfg, uint32_t bootId, bool synthetic, vc::CounterStore* bootStore);
  void tick(uint32_t now);

  vc::SensorLinkCore& core() { return core_; }
  const vc::SensorLinkCore& core() const { return core_; }
  uint32_t rxQueueOverflows() const { return rx_.overflows() + rx_.oversize(); }
  uint32_t sendResultOverflows() const { return results_.overflows(); }

  // vc::LinkRadio
  bool send(const uint8_t* data, size_t len) override;
  void setChannel(uint8_t channel) override;

  static void onRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len);
  static void onSent(const wifi_tx_info_t* info, esp_now_send_status_t status);

 private:
  vc::SensorLinkCore core_;
  EspRandom rng_;
  uint8_t gatewayMac_[6] = {0};
  RxQueue rx_;
  SendResultQueue results_;
  bool radioUp_ = false;
};
