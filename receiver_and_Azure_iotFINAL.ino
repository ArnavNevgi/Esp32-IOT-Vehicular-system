#include <WiFi.h>
#include <WiFiUdp.h>
#include <AzureIoTHub.h>
#include <AzureIoTProtocol_MQTT.h>
#include <AzureIoTUtility.h>

const char* ssid = "IoTSender";       
const char* password = "12345555"; 
const int udpPort = 12345;

WiFiUDP udp;
char incomingPacket[255]; // Buffer for incoming packets

// Azure IoT Hub settings
const char* connectionString = "HostName=YourHostName.azure-devices.net;DeviceId=YourDeviceID;SharedAccessKey=YourPrimaryKey";
IOTHUB_DEVICE_CLIENT_LL_HANDLE deviceHandle;

// Timer for sending messages to Azure
unsigned long lastSendTime = 0;
const unsigned long sendInterval = 5000; // Send to Azure every 5 seconds

void sendMessageToAzure(const char* message) {
  IOTHUB_MESSAGE_HANDLE messageHandle = IoTHubMessage_CreateFromString(message);
  if (messageHandle == NULL) {
    Serial.println("Failed to create IoT Hub message.");
    return;
  }

  if (IoTHubDeviceClient_LL_SendEventAsync(deviceHandle, messageHandle, NULL, NULL) != IOTHUB_CLIENT_OK) {
    Serial.println("Failed to send message to IoT Hub.");
  } else {
    Serial.println("Message successfully sent to IoT Hub.");
  }

  IoTHubMessage_Destroy(messageHandle);
}

void setup() {
  Serial.begin(115200);

  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");

  // Initialize UDP for receiving data
  udp.begin(udpPort);
  Serial.printf("UDP receiver started on port %d\n", udpPort);

  // Initialize Azure IoT Hub
  IoTHub_Init();
  deviceHandle = IoTHubDeviceClient_LL_CreateFromConnectionString(connectionString, MQTT_Protocol);
  if (deviceHandle == NULL) {
    Serial.println("Failed to create IoT Hub client.");
    while (1);
  }
  Serial.println("Connected to Azure IoT Hub!");
}

void loop() {
  // Listen for UDP packets
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(incomingPacket, 255);
    if (len > 0) {
      incomingPacket[len] = 0; // Null-terminate the string
    }
    Serial.printf("Received UDP packet: %s\n", incomingPacket);

    // Send the received packet to Azure IoT Hub
    sendMessageToAzure(incomingPacket);
  }

  // Handle Azure IoT Hub events
  IoTHubDeviceClient_LL_DoWork(deviceHandle);

  // Limit the frequency of messages to Azure
  unsigned long currentTime = millis();
  if (currentTime - lastSendTime >= sendInterval) {
    IoTHubDeviceClient_LL_DoWork(deviceHandle);
    lastSendTime = currentTime;
  }

  delay(100); // delay
}
