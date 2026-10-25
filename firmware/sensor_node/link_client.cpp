#include "link_client.h"

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <string.h>

#include "sensor_config.h"

namespace {
LinkClient* g_self = nullptr;
}  // namespace

void LinkClient::onRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (!g_self || !info) return;
  int8_t rssi = info->rx_ctrl ? (int8_t)info->rx_ctrl->rssi : 0;
  uint8_t ch = info->rx_ctrl ? (uint8_t)info->rx_ctrl->channel : 0;
  g_self->rx_.pushFromCallback(info->src_addr, data, len, rssi, ch);
}

void LinkClient::onSent(const wifi_tx_info_t*, esp_now_send_status_t status) {
  if (g_self) g_self->results_.pushFromCallback(status == ESP_NOW_SEND_SUCCESS);
}

bool LinkClient::begin(const LinkConfig& cfg, uint32_t bootId, bool synthetic, vc::CounterStore* bootStore) {
  if (!cfg.ok || bootId == 0) return false;
  if (!rx_.begin(8) || !results_.begin(16)) return false;
  memcpy(gatewayMac_, cfg.gatewayMac, 6);

  WiFi.mode(WIFI_STA);  // station interface, never associated with an AP
  WiFi.disconnect();
  if (esp_now_init() != ESP_OK) return false;
  if (esp_now_set_pmk(cfg.pmk) != ESP_OK) return false;
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, cfg.gatewayMac, 6);
  memcpy(peer.lmk, cfg.lmk, 16);
  peer.channel = 0;  // follow the current radio channel while sweeping
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = true;
  if (esp_now_add_peer(&peer) != ESP_OK) return false;
  g_self = this;
  esp_now_register_recv_cb(&LinkClient::onRecv);
  esp_now_register_send_cb(&LinkClient::onSent);
  radioUp_ = true;

  vc::SensorLinkConfig lc;
  lc.helloDwellMs = SN_LINK_HELLO_DWELL_MS;
  lc.maxChannel = SN_LINK_MAX_CHANNEL;
  lc.failsBeforeRescan = SN_LINK_FAILS_BEFORE_RESCAN;
  lc.eventRetryMs = SN_EVENT_RETRY_MS;
  lc.eventMaxAttempts = SN_EVENT_MAX_ATTEMPTS;
  vc::proto::LinkKeys keys = vc::proto::deriveLinkKeys(cfg.lmk);
  bool ok = core_.begin(keys, cfg.sensorMac, cfg.gatewayMac, bootId, synthetic, this, &rng_, bootStore, lc, millis());
  memset(&keys, 0, sizeof(keys));
  return ok;
}

bool LinkClient::send(const uint8_t* data, size_t len) {
  return radioUp_ && esp_now_send(gatewayMac_, data, len) == ESP_OK;
}

void LinkClient::setChannel(uint8_t channel) {
  if (radioUp_) esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
}

void LinkClient::tick(uint32_t now) {
  if (!radioUp_) return;
  bool delivered;
  for (int i = 0; i < 16 && results_.pop(delivered); ++i) core_.onSendResult(delivered);
  RxFrame f;
  for (int i = 0; i < 8 && rx_.pop(f); ++i) core_.onFrame(f.mac, f.data, f.len, now);
  core_.tick(now);
}
