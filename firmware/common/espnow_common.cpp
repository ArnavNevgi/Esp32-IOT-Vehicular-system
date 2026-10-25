#include "espnow_common.h"

#include <Preferences.h>
#include <string.h>

#include "vc_log.h"
#include "vc_secrets.h"
#include "vc_sensor_link.h"
#include "vc_util.h"

LinkConfig loadLinkConfig() {
  LinkConfig c;
  if (vc::isPlaceholder(VC_ESPNOW_PMK_HEX) || vc::isPlaceholder(VC_ESPNOW_LMK_HEX)) {
    c.problem = "ESP-NOW keys are placeholders";
    return c;
  }
  if (!vc::parseHexKey(VC_ESPNOW_PMK_HEX, c.pmk, 16) || !vc::parseHexKey(VC_ESPNOW_LMK_HEX, c.lmk, 16)) {
    c.problem = "ESP-NOW keys must be 32 hex digits and not all zero";
    return c;
  }
  if (memcmp(c.pmk, c.lmk, 16) == 0) {
    c.problem = "ESP-NOW PMK and LMK must differ";
    return c;
  }
  if (vc::isPlaceholder(VC_SENSOR_NODE_MAC) || vc::isPlaceholder(VC_GATEWAY_MAC)) {
    c.problem = "peer MAC addresses are placeholders";
    return c;
  }
  if (!vc::parseMac(VC_SENSOR_NODE_MAC, c.sensorMac) || !vc::parseMac(VC_GATEWAY_MAC, c.gatewayMac)) {
    c.problem = "peer MAC addresses must look like AA:BB:CC:DD:EE:FF (unicast)";
    return c;
  }
  if (memcmp(c.sensorMac, c.gatewayMac, 6) == 0) {
    c.problem = "sensor and gateway MAC addresses are identical";
    return c;
  }
  c.ok = true;
  return c;
}

void formatMac(const uint8_t mac[6], char out[18]) {
  snprintf(out, 18, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

namespace {
void serialSink(const char* line) { Serial.println(line); }
}  // namespace

void installSerialLog() { vc::setLogSink(serialSink); }

bool RxQueue::begin(unsigned depth) {
  q_ = xQueueCreate(depth, sizeof(RxFrame));
  return q_ != nullptr;
}

void RxQueue::pushFromCallback(const uint8_t* mac, const uint8_t* data, int len, int8_t rssi, uint8_t channel) {
  if (!q_ || mac == nullptr || data == nullptr) return;
  if (len <= 0 || (size_t)len > vc::proto::kMaxPacketSize) {
    oversize_ = oversize_ + 1;
    return;
  }
  RxFrame f;
  memcpy(f.mac, mac, 6);
  f.len = (uint8_t)len;
  f.rssi = rssi;
  f.channel = channel;
  memcpy(f.data, data, (size_t)len);
  if (xQueueSend(q_, &f, 0) != pdTRUE) {
    overflows_ = overflows_ + 1;
    return;
  }
  unsigned waiting = (unsigned)uxQueueMessagesWaiting(q_);
  if (waiting > highWater_) highWater_ = waiting;
}

bool RxQueue::pop(RxFrame& out) { return q_ && xQueueReceive(q_, &out, 0) == pdTRUE; }

bool SendResultQueue::begin(unsigned depth) {
  q_ = xQueueCreate(depth, sizeof(uint8_t));
  return q_ != nullptr;
}

void SendResultQueue::pushFromCallback(bool delivered) {
  uint8_t v = delivered ? 1 : 0;
  if (!q_ || xQueueSend(q_, &v, 0) != pdTRUE) overflows_ = overflows_ + 1;
}

bool SendResultQueue::pop(bool& delivered) {
  uint8_t v;
  if (!q_ || xQueueReceive(q_, &v, 0) != pdTRUE) return false;
  delivered = v != 0;
  return true;
}

bool NvsCounterStore::load(uint32_t& value) {
  Preferences prefs;
  // Read-write open: creates the namespace on first boot instead of failing.
  if (!prefs.begin(ns_, false)) return false;
  value = prefs.getUInt(key_, 0);
  prefs.end();
  return true;
}

bool NvsCounterStore::store(uint32_t value) {
  Preferences prefs;
  if (!prefs.begin(ns_, false)) return false;
  bool ok = prefs.putUInt(key_, value) == sizeof(uint32_t) && prefs.getUInt(key_, value + 1) == value;
  prefs.end();
  return ok;
}

uint32_t nextBootCounter(const char* nvsNamespace) {
  // Without NVS there is no monotonic boot id; the caller must not start the
  // link (a random id could be lower than the one the gateway remembers).
  NvsCounterStore store(nvsNamespace, "boot");
  return vc::advanceBootCounter(store);
}
