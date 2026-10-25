#include "azure_link.h"

#include <az_core.h>
#include <az_iot.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <mbedtls/base64.h>
#include <mbedtls/md.h>
#include <mbedtls/platform_util.h>
#include <mqtt_client.h>
#include <string.h>
#include <time.h>

#include "azure_root_ca.h"
#include "gateway_config.h"
#include "vc_secrets.h"
#include "vc_util.h"

namespace {

// Everything the MQTT client references lives for the whole program.
az_iot_hub_client hubClient;
char brokerUri[160];
char mqttClientId[128];
char mqttUsername[256];
char topicBuf[512];
esp_mqtt_client_handle_t client = nullptr;
QueueHandle_t mqttEvents = nullptr;
volatile uint32_t mqttEventsLost = 0;  // written by the MQTT task
uint32_t mqttEventsLostSeen = 0;

void mqttHandler(void* args, esp_event_base_t, int32_t eventId, void* data) {
  esp_mqtt_event_handle_t ev = (esp_mqtt_event_handle_t)data;
  vc::MqttEvent r;
  r.generation = (uint32_t)(uintptr_t)args;
  r.msgId = ev ? ev->msg_id : -1;
  switch ((esp_mqtt_event_id_t)eventId) {
    case MQTT_EVENT_CONNECTED: r.kind = vc::MqttEvent::Kind::Connected; break;
    case MQTT_EVENT_DISCONNECTED: r.kind = vc::MqttEvent::Kind::Disconnected; break;
    case MQTT_EVENT_PUBLISHED: r.kind = vc::MqttEvent::Kind::Published; break;
    case MQTT_EVENT_DELETED: r.kind = vc::MqttEvent::Kind::Deleted; break;
    case MQTT_EVENT_ERROR:
      if (!ev || !ev->error_handle) return;
      if (ev->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED) {
        r.kind = vc::MqttEvent::Kind::Refused;
        r.refuseCode = (int)ev->error_handle->connect_return_code;
      } else if (ev->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
        r.kind = vc::MqttEvent::Kind::TransportError;
        r.tlsErr = (int)ev->error_handle->esp_tls_last_esp_err;
        r.tlsStackErr = ev->error_handle->esp_tls_stack_err;
        r.sockErrno = ev->error_handle->esp_transport_sock_errno;
      } else {
        return;
      }
      break;
    default:
      return;
  }
  // Never block the MQTT task; a lost record is reported to the core.
  if (!mqttEvents || xQueueSend(mqttEvents, &r, 0) != pdTRUE) mqttEventsLost = mqttEventsLost + 1;
}

}  // namespace

bool AzureMqtt::configure() {
  if (vc::isPlaceholder(VC_IOTHUB_FQDN) || vc::isPlaceholder(VC_IOTHUB_DEVICE_ID) ||
      vc::isPlaceholder(VC_IOTHUB_DEVICE_KEY)) {
    snprintf(problem_, sizeof(problem_), "placeholder IoT Hub settings");
    return false;
  }
  az_iot_hub_client_options opts = az_iot_hub_client_options_default();
  opts.user_agent = AZ_SPAN_FROM_STR("c%2F" AZ_SDK_VERSION_STRING "(ard;esp32)");
  bool ok = az_result_succeeded(az_iot_hub_client_init(&hubClient, AZ_SPAN_FROM_STR(VC_IOTHUB_FQDN),
                                                       AZ_SPAN_FROM_STR(VC_IOTHUB_DEVICE_ID), &opts));
  size_t len = 0;
  ok = ok && az_result_succeeded(az_iot_hub_client_get_client_id(&hubClient, mqttClientId, sizeof(mqttClientId), &len));
  ok = ok && az_result_succeeded(az_iot_hub_client_get_user_name(&hubClient, mqttUsername, sizeof(mqttUsername), &len));
  int n = snprintf(brokerUri, sizeof(brokerUri), "mqtts://%s", VC_IOTHUB_FQDN);
  ok = ok && n > 0 && (size_t)n < sizeof(brokerUri);
  // A device key must be valid base64 of a key of at least 16 bytes.
  uint8_t probe[64];
  size_t keyLen = 0;
  ok = ok && mbedtls_base64_decode(probe, sizeof(probe), &keyLen, (const uint8_t*)VC_IOTHUB_DEVICE_KEY,
                                   strlen(VC_IOTHUB_DEVICE_KEY)) == 0 && keyLen >= 16;
  mbedtls_platform_zeroize(probe, sizeof(probe));
  if (!ok) {
    snprintf(problem_, sizeof(problem_), "invalid IoT Hub host name, device id or device key");
    return false;
  }
  mqttEvents = xQueueCreate(32, sizeof(vc::MqttEvent));
  if (!mqttEvents) {
    snprintf(problem_, sizeof(problem_), "out of memory");
    return false;
  }
  return true;
}

// SAS = "SharedAccessSignature sr=...&sig=...&se=..." where sig is the base64
// HMAC-SHA256 (keyed with the base64-decoded device key) of the SDK-built
// signature "<url-encoded resource>\n<expiry>". Key and MAC are wiped.
bool AzureMqtt::generate(uint32_t lifetimeSec, char* out, size_t cap) {
  uint64_t expiry = (uint64_t)time(nullptr) + lifetimeSec;
  uint8_t sigBuf[256];
  az_span sig;
  if (az_result_failed(az_iot_hub_client_sas_get_signature(&hubClient, expiry, AZ_SPAN_FROM_BUFFER(sigBuf), &sig)))
    return false;
  uint8_t key[64];
  size_t keyLen = 0;
  const char* b64 = VC_IOTHUB_DEVICE_KEY;
  bool ok = mbedtls_base64_decode(key, sizeof(key), &keyLen, (const uint8_t*)b64, strlen(b64)) == 0 && keyLen > 0;
  uint8_t mac[32];
  if (ok) {
    const mbedtls_md_info_t* md = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
    ok = mbedtls_md_hmac(md, key, keyLen, az_span_ptr(sig), (size_t)az_span_size(sig), mac) == 0;
  }
  char signedB64[64];
  size_t signedLen = 0;
  if (ok) ok = mbedtls_base64_encode((uint8_t*)signedB64, sizeof(signedB64), &signedLen, mac, sizeof(mac)) == 0;
  size_t pwLen = 0;
  if (ok)
    ok = az_result_succeeded(az_iot_hub_client_sas_get_password(
        &hubClient, expiry, az_span_create((uint8_t*)signedB64, (int32_t)signedLen), AZ_SPAN_EMPTY, out, cap, &pwLen));
  mbedtls_platform_zeroize(key, sizeof(key));
  mbedtls_platform_zeroize(mac, sizeof(mac));
  mbedtls_platform_zeroize(signedB64, sizeof(signedB64));
  return ok;
}

bool AzureMqtt::start(uint32_t generation, const char* password) {
  esp_mqtt_client_config_t cfg = {};
  cfg.broker.address.uri = brokerUri;  // port 8883 (mqtts default)
  cfg.broker.verification.certificate = kAzureRootCAs;
  cfg.broker.verification.certificate_len = sizeof(kAzureRootCAs);
  cfg.broker.verification.skip_cert_common_name_check = false;  // hostname is verified
  cfg.credentials.client_id = mqttClientId;
  cfg.credentials.username = mqttUsername;
  cfg.credentials.authentication.password = password;  // copied by esp_mqtt_client_init
  cfg.session.protocol_ver = MQTT_PROTOCOL_V_3_1_1;   // IoT Hub requires 3.1.1
  cfg.session.keepalive = GW_MQTT_KEEPALIVE_S;
  cfg.session.disable_clean_session = false;
  cfg.network.disable_auto_reconnect = true;  // lifecycle owned by vc::CloudLinkCore
  cfg.network.timeout_ms = 10000;
  cfg.buffer.size = 2048;
  cfg.buffer.out_size = 2048;
  cfg.outbox.limit = 16 * 1024;

  // Host and device id only; the SAS token is never printed.
  Serial.printf("[azure] TLS + MQTT 3.1.1 to %s:8883 as device %s\n", VC_IOTHUB_FQDN, VC_IOTHUB_DEVICE_ID);
  client = esp_mqtt_client_init(&cfg);
  if (!client) return false;
  esp_mqtt_client_register_event(client, MQTT_EVENT_ANY, mqttHandler, (void*)(uintptr_t)generation);
  return esp_mqtt_client_start(client) == ESP_OK;
}

void AzureMqtt::stop() {
  if (!client) return;
  // Main-loop context only: stop/destroy are not allowed in the handler.
  esp_mqtt_client_stop(client);
  esp_mqtt_client_destroy(client);
  client = nullptr;
}

void AzureMqtt::drain(vc::CloudLinkCore& core, uint32_t now) {
  vc::MqttEvent r;
  while (mqttEvents && xQueueReceive(mqttEvents, &r, 0) == pdTRUE) core.onMqttEvent(r, now);
  uint32_t lost = mqttEventsLost;
  if (lost != mqttEventsLostSeen) {
    mqttEventsLostSeen = lost;
    core.onEventsLost(now);
  }
}

int AzureMqtt::publish(const char* messageId, const char* messageType, const char* payload, size_t len,
                       size_t* wireBytes) {
  if (!client) return -1;
  uint8_t propBuf[256];
  az_iot_message_properties props;
  char midEnc[128];
  if (!vc::urlEncode(messageId, midEnc, sizeof(midEnc))) return -1;
  bool ok = az_result_succeeded(az_iot_message_properties_init(&props, AZ_SPAN_FROM_BUFFER(propBuf), 0));
  // System properties: content type/encoding let IoT Hub routing and
  // downstream consumers treat the body as JSON. $.mid is the stable id.
  ok = ok && az_result_succeeded(az_iot_message_properties_append(
                 &props, AZ_SPAN_FROM_STR(AZ_IOT_MESSAGE_PROPERTIES_CONTENT_TYPE), AZ_SPAN_FROM_STR("application%2Fjson")));
  ok = ok && az_result_succeeded(az_iot_message_properties_append(
                 &props, AZ_SPAN_FROM_STR(AZ_IOT_MESSAGE_PROPERTIES_CONTENT_ENCODING), AZ_SPAN_FROM_STR("utf-8")));
  ok = ok && az_result_succeeded(az_iot_message_properties_append(
                 &props, AZ_SPAN_FROM_STR(AZ_IOT_MESSAGE_PROPERTIES_MESSAGE_ID), az_span_create_from_str(midEnc)));
  ok = ok && az_result_succeeded(az_iot_message_properties_append(&props, AZ_SPAN_FROM_STR("messageType"),
                                                                  az_span_create_from_str((char*)messageType)));
  size_t topicLen = 0;
  ok = ok && az_result_succeeded(
                 az_iot_hub_client_telemetry_get_publish_topic(&hubClient, &props, topicBuf, sizeof(topicBuf), &topicLen));
  if (!ok) return -1;
  // Non-blocking: the message is stored in the client outbox and sent by the
  // MQTT task. Returns msg id > 0, -1 on failure, -2 when the outbox is full.
  int id = esp_mqtt_client_enqueue(client, topicBuf, payload, (int)len, 1, 0, true);
  // Size estimate for metering: body plus the topic's property bag (an upper
  // bound, the topic also contains the fixed devices/{id}/messages/events/).
  if (wireBytes) *wireBytes = len + topicLen;
  return id;
}
