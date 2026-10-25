// Azure IoT Hub transport for the gateway: Azure SDK for C 1.1.8 (client id,
// username, SAS signature, telemetry topic + properties) on ESP-IDF 5.5
// esp-mqtt (MQTT 3.1.1 over TLS with DigiCert G2 / Microsoft 2017 roots and
// hostname verification). The connection lifecycle (SAS renewal, backoff,
// generations, PUBACK hand-off) lives in vc::CloudLinkCore.
//
// The MQTT event handler runs in the esp-mqtt task; it only copies a small
// record, tagged with the client generation, into a FreeRTOS queue. drain()
// feeds those records to the core from the loop. If the queue ever
// overflows, the core restarts the connection instead of guessing its state.
#pragma once

#include <Arduino.h>

#include "vc_cloud_link.h"

class AzureMqtt : public vc::MqttPort, public vc::SasTokenSource {
 public:
  // Validates the IoT Hub settings; false = placeholders / invalid (problem()).
  bool configure();
  const char* problem() const { return problem_; }
  // Moves queued client events into the core (loop context).
  void drain(vc::CloudLinkCore& core, uint32_t now);

  // vc::MqttPort
  bool start(uint32_t generation, const char* password) override;
  void stop() override;
  int publish(const char* messageId, const char* messageType, const char* payload, size_t len,
              size_t* wireBytes) override;
  // vc::SasTokenSource
  bool generate(uint32_t lifetimeSec, char* out, size_t cap) override;

 private:
  char problem_[80] = "";
};
