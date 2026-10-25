// ESP-NOW adapter for the gateway's link to the sensor node (logic in
// vc::GatewayLinkCore). The receive callback runs in the Wi-Fi task and only
// copies frames into a bounded queue; tick() drains it from the loop. The
// gateway follows its AP's channel and never changes channel for ESP-NOW.
#pragma once

#include <Arduino.h>
#include <esp_now.h>

#include "espnow_common.h"
#include "vc_gateway_link.h"

class SensorLink : public vc::LinkRadio {
 public:
  bool begin(const LinkConfig& cfg, vc::SensorEventSink* sink);
  void tick(uint32_t now);

  vc::GatewayLinkCore& core() { return core_; }
  const vc::GatewayLinkCore& core() const { return core_; }
  uint32_t rxQueueOverflows() const { return rx_.overflows() + rx_.oversize(); }
  unsigned rxQueueHighWater() const { return rx_.highWater(); }

  // vc::LinkRadio
  bool send(const uint8_t* data, size_t len) override;

  static void onRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len);

 private:
  vc::GatewayLinkCore core_;
  EspRandom rng_;
  NvsCounterStore bootStore_{"vc_gateway", "sensorBoot"};
  NvsCounterStore nonceStore_{"vc_gateway", "sensorNonce"};
  uint8_t sensorMac_[6] = {0};
  RxQueue rx_;
  bool radioUp_ = false;
};
