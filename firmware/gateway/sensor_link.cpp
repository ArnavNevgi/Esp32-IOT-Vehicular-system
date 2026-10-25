#include "sensor_link.h"

#include <WiFi.h>
#include <string.h>

namespace {
SensorLink* g_self = nullptr;
}  // namespace

void SensorLink::onRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (!g_self || !info) return;
  int8_t rssi = info->rx_ctrl ? (int8_t)info->rx_ctrl->rssi : 0;
  uint8_t ch = info->rx_ctrl ? (uint8_t)info->rx_ctrl->channel : 0;
  g_self->rx_.pushFromCallback(info->src_addr, data, len, rssi, ch);
}

bool SensorLink::begin(const LinkConfig& cfg, vc::SensorEventSink* sink) {
  if (!cfg.ok) return false;
  if (!rx_.begin(16)) return false;
  memcpy(sensorMac_, cfg.sensorMac, 6);

  // WiFi.mode(WIFI_STA) has already been called by WifiLink::begin().
  if (esp_now_init() != ESP_OK) return false;
  if (esp_now_set_pmk(cfg.pmk) != ESP_OK) return false;
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, cfg.sensorMac, 6);
  memcpy(peer.lmk, cfg.lmk, 16);
  peer.channel = 0;  // whatever channel the upstream AP dictates
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = true;
  if (esp_now_add_peer(&peer) != ESP_OK) return false;
  radioUp_ = true;

  vc::proto::LinkKeys keys = vc::proto::deriveLinkKeys(cfg.lmk);
  bool ok = core_.begin(keys, cfg.sensorMac, cfg.gatewayMac, this, &rng_, &bootStore_, &nonceStore_, sink);
  memset(&keys, 0, sizeof(keys));
  if (!ok) return false;
  g_self = this;
  esp_now_register_recv_cb(&SensorLink::onRecv);
  return true;
}

bool SensorLink::send(const uint8_t* data, size_t len) {
  return radioUp_ && esp_now_send(sensorMac_, data, len) == ESP_OK;
}

void SensorLink::tick(uint32_t now) {
  if (!core_.configured()) return;
  RxFrame f;
  for (int i = 0; i < 16 && rx_.pop(f); ++i) core_.onFrame(f.mac, f.data, f.len, now, f.channel, f.rssi);
}
