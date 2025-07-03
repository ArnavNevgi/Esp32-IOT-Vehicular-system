#include <WiFi.h>
#include <WiFiUdp.h>
#include <AESLib.h>

const char* ssid = "IoTSender";
const char* password = "12345555";
const char* receiverIP = "192.168.1.224";
const int udpPort = 12345;

WiFiUDP udp;
AESLib aesLib;

byte aes_key[] = { '1','2','3','4','5','6','7','8','9','0','a','b','c','d','e','f' };

char encrypted[128];
char clear_text[] = "Temp: 27.5C, Humidity: 56%";

void encryptAndSend(const char* message) {
  int len = aesLib.encrypt(message, encrypted, aes_key, 128);
  if (len > 0) {
    udp.beginPacket(receiverIP, udpPort);
    udp.write((uint8_t*)encrypted, len);
    udp.endPacket();
    Serial.println("Encrypted message sent!");
  } else {
    Serial.println("Encryption failed");
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
  }
  Serial.println("\nWiFi connected.");
  udp.begin(udpPort);
}

void loop() {
  encryptAndSend(clear_text);
  delay(5000);
}
