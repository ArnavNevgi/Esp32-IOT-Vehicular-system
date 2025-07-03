#include <WiFi.h>
#include <WiFiUdp.h>
#include <AESLib.h>

const char* ssid = "IoTSender";
const char* password = "12345555";
const int udpPort = 12345;

WiFiUDP udp;
AESLib aesLib;

byte aes_key[] = { '1','2','3','4','5','6','7','8','9','0','a','b','c','d','e','f' };

char incomingPacket[128];
char decrypted[128];

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
  }
  Serial.println("\nWiFi connected.");
  udp.begin(udpPort);
  Serial.printf("Listening on UDP port %d\n", udpPort);
}

void loop() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(incomingPacket, sizeof(incomingPacket) - 1);
    incomingPacket[len] = '\0';

    int decrypted_len = aesLib.decrypt(incomingPacket, len, decrypted, aes_key, 128);
    if (decrypted_len > 0) {
      decrypted[decrypted_len] = '\0';
      Serial.print("Decrypted message: ");
      Serial.println(decrypted);
    } else {
      Serial.println("Decryption failed or wrong data.");
    }
  }
  delay(100);
}
