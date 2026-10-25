// Gateway: ESP-NOW from the sensor node + its own DHT22 / flame / potentiometer
// -> local alarms -> aggregated summaries and alerts -> Azure IoT Hub (MQTT/TLS).
//
// Everything runs from the Arduino loop without blocking on the network:
//   SensorLink  drains ESP-NOW frames (copied out of the Wi-Fi task)
//   sensors     DHT22 every 3 s, flame 100 ms, potentiometer 500 ms (if fitted)
//   app         vc::GatewayApp: alarms, reminders, 60 s summary, outbox service
//   cloud       vc::CloudLinkCore lifecycle over AzureMqtt (esp-mqtt)
// Local alarms and LED/buzzer keep working with no Wi-Fi or Azure.
#include <Arduino.h>
#include <WiFi.h>
#include <esp_heap_caps.h>

#include "alert_output.h"
#include "azure_link.h"
#include "dht_sampler.h"
#include "espnow_common.h"
#include "gateway_config.h"
#include "net.h"
#include "sensor_link.h"
#include "vc_cloud_link.h"
#include "vc_gateway_app.h"
#include "vc_secrets.h"
#include "vc_telemetry.h"
#include "vc_timing.h"
#include "vc_util.h"

using namespace vc;
using namespace vc::proto;

namespace {

const char* gatewayId() { return isPlaceholder(VC_IOTHUB_DEVICE_ID) ? "unconfigured-gateway" : VC_IOTHUB_DEVICE_ID; }

WifiLink wifi;
TimeSync clock_;
AzureMqtt azureMqtt;
EspRandom rng;
CloudLinkCore cloud;
SensorLink sensorLink;
GatewayApp app;
AlertOutput alerts;
DhtSampler dht22(GW_PIN_DHT22, DHT22);
PeriodicTimer flameTimer(GW_FLAME_SAMPLE_MS), potTimer(GW_POT_SAMPLE_MS), healthTimer(GW_HEALTH_LOG_MS);

class Platform : public PlatformStatus {
 public:
  const char* wifiState() const override { return wifi.stateName(); }
  int rssi() const override { return wifi.rssi(); }
  uint32_t freeHeap() const override { return ESP.getFreeHeap(); }
  uint32_t minFreeHeap() const override { return heap_caps_get_minimum_free_size(MALLOC_CAP_DEFAULT); }
  uint32_t radioRxOverflows() const override { return sensorLink.rxQueueOverflows(); }
} platform;

void sampleGatewaySensors(uint32_t now) {
  if (GW_ENABLE_DHT22 && dht22.poll(now)) app.onDht22Reading(dht22.reading(now), dht22.lastReadOk());
  if (GW_ENABLE_FLAME && flameTimer.due(now)) {
    float pct = analogRead(GW_PIN_FLAME) * 100.0f / 4095.0f;
    if (GW_FLAME_INVERTED) pct = 100.0f - pct;
    app.onFlameSample(pct, now);
  }
  if (GW_ENABLE_POT && potTimer.due(now)) app.onPotSample(analogRead(GW_PIN_POT) * 100.0f / 4095.0f);
}

void logHealth(uint32_t now) {
  const GatewayLinkCore& link = sensorLink.core();
  const GatewayLinkCore::Stats& ls = link.stats();
  const GatewayApp::Stats& as = app.stats();
  const char* dhtStatus = GW_ENABLE_DHT22 ? telemetry::statusName(dht22.reading(now).status) : "disabled";
  Serial.printf(
      "[health] up %lus wifi=%s ch=%u rssi=%d time=%s cloud=%s%s%s token=%lus | sensor link=%s lastRx=%lums "
      "ok=%lu tag=%lu malformed=%lu replay=%lu session=%lu peer=%lu dupEv=%lu rxOvf=%lu rxHw=%u | outbox=%u "
      "hw=%lu inflight=%u acked=%lu fail=%lu meter~%lu | dht22=%s flame=%s%.1f%%%s | heap=%lu min=%lu\n",
      (unsigned long)(now / 1000), wifi.stateName(), wifi.channel(), wifi.rssi(), clock_.stateName(),
      cloud.stateName(), cloud.lastError()[0] ? " last=" : "", cloud.lastError(),
      (unsigned long)(cloud.tokenRemainingMs(now) / 1000),
      !link.configured() ? "unconfigured" : (app.linkLost() ? "LOST" : (link.everSeen() ? "up" : "waiting")),
      (unsigned long)elapsedMs(now, link.lastRxMs()), (unsigned long)ls.accepted, (unsigned long)ls.badTag,
      (unsigned long)ls.malformed, (unsigned long)ls.replay, (unsigned long)ls.sessionRejected,
      (unsigned long)ls.unknownPeer, (unsigned long)ls.duplicateEvents, (unsigned long)sensorLink.rxQueueOverflows(),
      sensorLink.rxQueueHighWater(), app.outbox().depth(), (unsigned long)as.outboxHighWater, app.outbox().inFlight(),
      (unsigned long)as.publishedOk, (unsigned long)as.publishFailures, (unsigned long)as.estMeterUnits, dhtStatus,
      GW_ENABLE_FLAME ? "" : "disabled ", GW_ENABLE_FLAME ? app.lastFlamePct() : 0.0f,
      app.flameActive() ? " FLAME" : "", (unsigned long)ESP.getFreeHeap(),
      (unsigned long)heap_caps_get_minimum_free_size(MALLOC_CAP_DEFAULT));
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(200);
  installSerialLog();
  WiFi.mode(WIFI_STA);
  Serial.printf("\n=== Vehicle gateway === station MAC %s, secrets from %s\n", WiFi.macAddress().c_str(),
                VC_SECRETS_SOURCE);
  Serial.printf("fitted: dht22=%d flame=%d pot=%d led=%d buzzer=%d\n", GW_ENABLE_DHT22, GW_ENABLE_FLAME,
                GW_ENABLE_POT, GW_PIN_LED >= 0, GW_PIN_BUZZER >= 0);
  uint32_t gatewayBoot = nextBootCounter("vc_gw_boot");
  if (gatewayBoot == 0) {
    // Only message ids depend on it; without NVS they stay unique-ish.
    gatewayBoot = esp_random() | 0x80000000u;
    Serial.println("WARNING: NVS boot counter unavailable; using a random gateway boot id");
  }
  Serial.printf("gateway boot id %lu\n", (unsigned long)gatewayBoot);

  alerts.begin(GW_PIN_LED, GW_LED_ACTIVE_HIGH, GW_PIN_BUZZER, GW_BUZZER_ACTIVE_HIGH);
  analogReadResolution(12);
  uint32_t now = millis();
  if (GW_ENABLE_DHT22) dht22.begin(now, GW_DHT_INTERVAL_MS, GW_DHT_FRESH_MS, GW_DHT_MAX_CACHE_MS, 2000);

  if (!wifi.begin(VC_WIFI_SSID, VC_WIFI_PASSWORD))
    Serial.println("UNCONFIGURED: Wi-Fi credentials are placeholders; no upstream connection will be attempted.");
  CloudLinkConfig cc;
  cc.sasLifetimeSec = GW_SAS_LIFETIME_MIN * 60u;
  cc.refreshMarginSec = GW_SAS_REFRESH_MARGIN_S;
  cc.connectTimeoutMs = GW_MQTT_CONNECT_TIMEOUT_MS;
  if (!azureMqtt.configure() || !cloud.begin(&azureMqtt, &azureMqtt, &rng, cc))
    Serial.printf("UNCONFIGURED: Azure IoT Hub disabled (%s); no login will be attempted.\n", azureMqtt.problem());

  GatewayAppConfig ac;
  ac.gatewayId = gatewayId();
  ac.sensorNodeId = VC_SENSOR_NODE_ID;
  ac.gatewayBootId = gatewayBoot;
  ac.dht22Enabled = GW_ENABLE_DHT22;
  ac.flameEnabled = GW_ENABLE_FLAME;
  ac.potEnabled = GW_ENABLE_POT;
  ac.flameOnPct = GW_FLAME_ON_PCT;
  ac.flameOffPct = GW_FLAME_OFF_PCT;
  ac.flameActivateMs = GW_FLAME_ACTIVATE_MS;
  ac.flameClearMs = GW_FLAME_CLEAR_MS;
  ac.linkLossMs = GW_LINK_LOSS_MS;
  ac.linkRestoreMs = GW_LINK_RESTORE_MS;
  ac.linkBootGraceMs = GW_LINK_BOOT_GRACE_MS;
  ac.reminderMs = GW_REMINDER_MS;
  ac.summaryMs = GW_SUMMARY_MS;
  ac.cloudMinSpacingMs = GW_CLOUD_MIN_SPACING_MS;
  ac.cloudMaxInFlight = GW_CLOUD_MAX_IN_FLIGHT;
  ac.cloudAckTimeoutMs = GW_CLOUD_ACK_TIMEOUT_MS;
  ac.meterBlockBytes = GW_METER_BLOCK_BYTES;

  LinkConfig cfg = loadLinkConfig();
  if (!cfg.ok) Serial.printf("UNCONFIGURED: sensor link disabled (%s).\n", cfg.problem);
  else if (!sensorLink.begin(cfg, &app)) Serial.println("ERROR: ESP-NOW initialisation failed; sensor link disabled.");
  app.begin(ac, &clock_, &cloud, &sensorLink.core(), &platform, now);

  flameTimer.start(now);
  potTimer.start(now);
  healthTimer.start(now);
  enableLoopWDT();
}

void loop() {
  uint32_t now = millis();
  wifi.tick(now);
  clock_.tick(now, wifi.up());
  azureMqtt.drain(cloud, now);
  cloud.tick(now, wifi.up(), clock_.valid());

  sensorLink.tick(now);
  sampleGatewaySensors(now);
  app.tick(now);

  if (app.takeChirp()) alerts.chirp(3, now);
  alerts.setAlarm(app.alarmActive());
  alerts.setAttention(app.attention());
  alerts.update(now);

  if (healthTimer.due(now)) logHealth(now);
  delay(1);
}
