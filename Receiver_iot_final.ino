#include <WiFi.h>
#include <WiFiUdp.h>
#include <mqtt_client.h>
#include <az_core.h>
#include <az_iot.h>
#include <azure_ca.h>
#include "DHT.h"
#include "AzIoTSasToken.h"
#include "SerialLogger.h"
#include "iot_configs.h"

// Wi-Fi credentials
const char* ssid = "YourSSID";
const char* password = "YourPassword";

// Azure IoT Hub settings
const char* host = IOT_CONFIG_IOTHUB_FQDN;
const char* mqtt_broker_uri = "mqtts://" IOT_CONFIG_IOTHUB_FQDN;
const char* device_id = IOT_CONFIG_DEVICE_ID;
static esp_mqtt_client_handle_t mqtt_client;
static az_iot_hub_client iot_hub_client;

#define SAS_TOKEN_DURATION_IN_MINUTES 60
#define TELEMETRY_FREQUENCY_MILLISECS 5000

// UDP Settings
WiFiUDP udp;
const int udpPort = 12345;
char incomingPacket[255]; // Buffer for incoming packets

// Sensor Configuration
#define DHT_PIN 4
#define FLAME_PIN 33
#define POTENTIOMETER_PIN 32
#define DHT_TYPE DHT22
DHT dht(DHT_PIN, DHT_TYPE);

// MQTT and telemetry buffers
static char telemetry_topic[128];
static char telemetry_payload[512];
static AzIoTSasToken sasToken(&iot_hub_client, AZ_SPAN_FROM_STR(IOT_CONFIG_DEVICE_KEY));

// Timing
unsigned long nextTelemetrySendTimeMs = 0;

// Forward declarations
void sendTelemetryToAzure(const char* message);
void initializeWiFi();
void initializeIoTHubClient();
void handleUdpPackets();

void setup() {
  Serial.begin(115200);

  // Initialize sensors
  dht.begin();
  pinMode(POTENTIOMETER_PIN, INPUT);
  pinMode(FLAME_PIN, INPUT);

  // Connect to Wi-Fi
  initializeWiFi();

  // Initialize Azure IoT Hub
  initializeIoTHubClient();

  // Start UDP
  udp.begin(udpPort);
  Serial.printf("UDP receiver started on port %d\n", udpPort);
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    initializeWiFi(); // Reconnect Wi-Fi if disconnected
  }

  handleUdpPackets(); // Listen for UDP packets

  // Send sensor telemetry to Azure IoT Hub periodically
  if (millis() > nextTelemetrySendTimeMs) {
    // Read sensor data
    float temperature = dht.readTemperature();
    float humidity = dht.readHumidity();
    uint16_t potentiometer = map(analogRead(POTENTIOMETER_PIN), 0, 4095, 0, 100);
    uint16_t flame = map(analogRead(FLAME_PIN), 0, 4095, 100, 0);

    // Prepare telemetry payload
    snprintf(telemetry_payload, sizeof(telemetry_payload),
             "{ \"Temperature\": %.2f, \"Humidity\": %.2f, \"Potentiometer\": %d, \"Flame\": %d }",
             temperature, humidity, potentiometer, flame);

    sendTelemetryToAzure(telemetry_payload);

    nextTelemetrySendTimeMs = millis() + TELEMETRY_FREQUENCY_MILLISECS;
  }

  delay(100);
}

// Initialize Wi-Fi connection
void initializeWiFi() {
  Serial.printf("Connecting to Wi-Fi: %s\n", ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.printf("\nConnected to Wi-Fi! IP: %s\n", WiFi.localIP().toString().c_str());
}

// Handle incoming UDP packets
void handleUdpPackets() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(incomingPacket, sizeof(incomingPacket) - 1);
    if (len > 0) {
      incomingPacket[len] = '\0'; // Null-terminate the string
    }
    Serial.printf("Received UDP packet: %s\n", incomingPacket);

    // Send the received UDP data to Azure IoT Hub
    sendTelemetryToAzure(incomingPacket);
  }
}

// Initialize IoT Hub Client
void initializeIoTHubClient() {
  az_iot_hub_client_options options = az_iot_hub_client_options_default();
  if (az_result_failed(az_iot_hub_client_init(&iot_hub_client,
                                              az_span_create((uint8_t*)host, strlen(host)),
                                              az_span_create((uint8_t*)device_id, strlen(device_id)),
                                              &options))) {
    Serial.println("Failed to initialize Azure IoT Hub client");
    while (1);
  }

  // Configure SAS token
  if (sasToken.Generate(SAS_TOKEN_DURATION_IN_MINUTES) != 0) {
    Serial.println("Failed to generate SAS token");
    while (1);
  }

  // Configure MQTT client
  esp_mqtt_client_config_t mqtt_config = {};
  mqtt_config.uri = mqtt_broker_uri;
  mqtt_config.client_id = device_id;
  mqtt_config.username = mqtt_client_id;
  mqtt_config.password = (const char*)az_span_ptr(sasToken.Get());
  mqtt_config.event_handle = [](esp_mqtt_event_handle_t event) -> esp_err_t {
    if (event->event_id == MQTT_EVENT_CONNECTED) {
      Serial.println("Connected to Azure IoT Hub");
    }
    return ESP_OK;
  };

  mqtt_client = esp_mqtt_client_init(&mqtt_config);
  if (esp_mqtt_client_start(mqtt_client) != ESP_OK) {
    Serial.println("Failed to start MQTT client");
    while (1);
  }
}

// Send telemetry to Azure IoT Hub
void sendTelemetryToAzure(const char* message) {
  if (az_result_failed(az_iot_hub_client_telemetry_get_publish_topic(
          &iot_hub_client, NULL, telemetry_topic, sizeof(telemetry_topic), NULL))) {
    Serial.println("Failed to get telemetry topic");
    return;
  }

  if (esp_mqtt_client_publish(mqtt_client, telemetry_topic, message, strlen(message), 1, 0) == 0) {
    Serial.println("Failed to publish telemetry");
  } else {
    Serial.println("Telemetry published successfully");
  }
}
