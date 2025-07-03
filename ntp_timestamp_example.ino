#include <WiFi.h>
#include <WiFiUdp.h>
#include <time.h>

const char* ssid = "IoTSender";
const char* password = "12345555";

void setupTime() {
  configTime(19800, 0, "pool.ntp.org", "time.nist.gov");
  Serial.print("Waiting for NTP time sync");
  time_t now = time(nullptr);
  while (now < 8 * 3600 * 2) {
    delay(500);
    Serial.print(".");
    now = time(nullptr);
  }
  Serial.println("\nTime synchronized!");
}

String getCurrentTime() {
  time_t now = time(nullptr);
  struct tm timeinfo;
  gmtime_r(&now, &timeinfo);

  char buf[32];
  strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &timeinfo);
  return String(buf);
}

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
  }
  Serial.println("\nWiFi connected.");

  setupTime();
}

void loop() {
  String timeStr = getCurrentTime();
  Serial.println("Current Timestamp: " + timeStr);
  delay(5000);
}
