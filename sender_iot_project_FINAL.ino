#include <WiFi.h>
#include <Wire.h>
#include <MPU6050.h>
#include <DHT.h>
#include <WiFiUdp.h>

#define MINUTE 60000
#define SECOND 1000

const char* ssid = "IoTSender";      
const char* password = "12345555";  
const char* receiverIP = "192.168.1.224"; 
const int udpPort = 12345;

WiFiUDP udp;
MPU6050 mpu;

int DHTPIN = 14;  
int DHT1PIN = 27;
int shockSensor = 25;
int pirPin = 26;

DHT dht(DHTPIN, DHT11);
DHT dht1(DHT1PIN, DHT11);

void setup() {
  Serial.begin(115200);
  
  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");

  udp.begin(udpPort);
  Serial.printf("UDP started on port %d\n", udpPort);

  // Initialize MPU6050
  Wire.begin();
  if (!mpu.begin()) {
    Serial.println("MPU6050 initialization failed!");
    while (1);
  }
  mpu.calcGyroOffsets(true);

  // Initialize sensors
  pinMode(shockSensor, INPUT);
  pinMode(pirPin, INPUT);
  dht.begin();
  dht1.begin();
  delay(2000);
}

void loop() {
  // Read sensor data
  float temp = dht.readTemperature();
  float humidity = dht.readHumidity();
  float temp1 = dht1.readTemperature();
  float humidity1 = dht1.readHumidity();
  int pirVal = digitalRead(pirPin);
  int shockSensorState = digitalRead(shockSensor);

  mpu.update();
  float ax = mpu.getAccX();
  float ay = mpu.getAccY();
  float az = mpu.getAccZ();
  float netAccel = sqrt(ax * ax + ay * ay + az * az);
  float roll = mpu.getAngleX();

  // Prepare data string
  String data = String("Temp1: ") + temp + "°C, Humidity1: " + humidity + "%, " +
                "Temp2: " + temp1 + "°C, Humidity2: " + humidity1 + "%, " +
                "PIR: " + (pirVal ? "Motion Detected" : "No Motion") + ", " +
                "Shock: " + (shockSensorState ? "YES" : "NO") + ", " +
                "Accel: " + netAccel + "g, Roll: " + roll + "°";

  // Send data via UDP
  udp.beginPacket(receiverIP, udpPort);
  udp.print(data);
  udp.endPacket();

  Serial.println(data); // For debugging
  
  delay(500); // delay
}
