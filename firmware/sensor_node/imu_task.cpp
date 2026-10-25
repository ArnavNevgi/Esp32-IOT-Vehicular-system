#include "imu_task.h"

#include <Arduino.h>
#include <MPU6050_light.h>
#include <Wire.h>
#include <esp_task_wdt.h>

#include "vc_timing.h"

using vc::proto::SensorStatus;

namespace imu {

namespace {

const uint8_t kAddr = MPU6050_ADDR;  // 0x68 (AD0 low)
const uint8_t kRegWhoAmI = 0x75;
const uint32_t kRetryMs = 5000;
const unsigned kFailuresBeforeAbsent = 5;
const unsigned kBuckets = 16;  // x 20 ms = 320 ms of peak history
const uint32_t kBucketMs = 20;

MPU6050 mpu(Wire);
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
ImuSnapshot shared;
float buckets[kBuckets];
uint32_t bucketStart[kBuckets];

int pinSda, pinScl;
uint32_t busHz, period;
bool doCalibrate;

// A cheap address ACK check before every update: MPU6050_light does not
// report read failures, so this keeps garbage out of the filter when the
// sensor is unplugged.
bool present() {
  Wire.beginTransmission(kAddr);
  return Wire.endTransmission() == 0;
}

bool initSensor() {
  if (!present()) return false;
  Wire.beginTransmission(kAddr);
  Wire.write(kRegWhoAmI);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom((int)kAddr, 1) != 1) return false;
  uint8_t who = Wire.read();
  Serial.printf("[imu] MPU6050 found, WHO_AM_I=0x%02X%s\n", who, who == 0x68 ? "" : " (not 0x68: compatible clone?)");
  // begin() returns the I2C status of its last register write: 0 = success.
  // gyro config 1 = +-500 deg/s, accel config 2 = +-8 g (headroom for impacts).
  if (mpu.begin(1, 2) != 0) return false;
  if (doCalibrate) {
    portENTER_CRITICAL(&mux);
    shared.status = SensorStatus::Calibrating;
    portEXIT_CRITICAL(&mux);
    Serial.println("[imu] calibrating: keep the vehicle still and level");
    esp_task_wdt_reset();
    mpu.calcOffsets(true, true);  // ~0.5-1 s, 500 samples
    esp_task_wdt_reset();
  }
  return true;
}

void recordBucket(uint32_t now, float mag) {
  unsigned i = (now / kBucketMs) % kBuckets;
  uint32_t start = now - (now % kBucketMs);
  if (bucketStart[i] != start) {
    bucketStart[i] = start;
    buckets[i] = 0;
  }
  if (mag > buckets[i]) buckets[i] = mag;
}

void task(void*) {
  esp_task_wdt_add(nullptr);
  Wire.begin(pinSda, pinScl, busHz);
  Wire.setTimeOut(5);  // ms; bounded bus stalls

  bool ready = false;
  unsigned failures = 0;
  uint32_t lastInitTry = 0;
  bool firstTry = true;
  uint32_t lastUpdateMs = 0;
  uint32_t secondStart = millis(), secondCount = 0;
  TickType_t wake = xTaskGetTickCount();

  for (;;) {
    esp_task_wdt_reset();
    uint32_t now = millis();

    if (!ready) {
      if (firstTry || vc::hasElapsed(now, lastInitTry, kRetryMs)) {
        firstTry = false;
        lastInitTry = now;
        ready = initSensor();
        portENTER_CRITICAL(&mux);
        shared.status = ready ? SensorStatus::Ok : SensorStatus::Absent;
        portEXIT_CRITICAL(&mux);
        if (!ready) Serial.printf("[imu] MPU6050 not detected; retrying every %u ms\n", (unsigned)kRetryMs);
        failures = 0;
        lastUpdateMs = millis();
        wake = xTaskGetTickCount();
      }
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    uint32_t t0 = micros();
    bool ok = present();
    if (ok) mpu.update();
    uint32_t dur = micros() - t0;
    now = millis();

    if (!ok) {
      portENTER_CRITICAL(&mux);
      ++shared.i2cErrors;
      if (++failures >= kFailuresBeforeAbsent) {
        shared.status = SensorStatus::Absent;
        ready = false;
      } else {
        shared.status = SensorStatus::Error;
      }
      portEXIT_CRITICAL(&mux);
    } else {
      failures = 0;
      float ax = mpu.getAccX(), ay = mpu.getAccY(), az = mpu.getAccZ();
      float mag = sqrtf(ax * ax + ay * ay + az * az);
      uint32_t gap = vc::elapsedMs(now, lastUpdateMs);
      lastUpdateMs = now;
      ++secondCount;
      portENTER_CRITICAL(&mux);
      shared.status = SensorStatus::Ok;
      shared.accelXG = ax;
      shared.accelYG = ay;
      shared.accelZG = az;
      shared.accelMagG = mag;
      shared.rollDeg = mpu.getAngleX();
      shared.pitchDeg = mpu.getAngleY();
      shared.sampleMs = now;
      if (gap > shared.maxGapMs) shared.maxGapMs = gap > 0xFFFF ? 0xFFFF : (uint16_t)gap;
      if (mag > shared.peakAccelG) shared.peakAccelG = mag;
      ++shared.samples;
      if (dur > shared.maxUpdateUs) shared.maxUpdateUs = dur;
      if (dur > period * 1000u) ++shared.overruns;
      recordBucket(now, mag);
      portEXIT_CRITICAL(&mux);
    }

    if (vc::hasElapsed(now, secondStart, 1000)) {
      float rate = secondCount * 1000.0f / (float)vc::elapsedMs(now, secondStart);
      portENTER_CRITICAL(&mux);
      shared.rateHz = rate;
      portEXIT_CRITICAL(&mux);
      secondStart = now;
      secondCount = 0;
    }

    // Fixed-rate wake-up; if we fell behind, resynchronise instead of bursting.
    if (xTaskDelayUntil(&wake, pdMS_TO_TICKS(period)) == pdFALSE) wake = xTaskGetTickCount();
  }
}

}  // namespace

bool start(int sda, int scl, uint32_t i2cHz, uint32_t periodMs, bool calibrate) {
  pinSda = sda;
  pinScl = scl;
  busHz = i2cHz;
  period = periodMs;
  doCalibrate = calibrate;
  // Core 1 (with the Arduino loop) at priority 5: above loop (1), far below
  // the Wi-Fi task (23, core 0). 4 KB stack: no large locals.
  return xTaskCreatePinnedToCore(task, "imu", 4096, nullptr, 5, nullptr, 1) == pdPASS;
}

ImuSnapshot peek() {
  portENTER_CRITICAL(&mux);
  ImuSnapshot s = shared;
  portEXIT_CRITICAL(&mux);
  return s;
}

ImuSnapshot take() {
  portENTER_CRITICAL(&mux);
  ImuSnapshot s = shared;
  shared.peakAccelG = 0;
  shared.maxGapMs = 0;
  portEXIT_CRITICAL(&mux);
  return s;
}

float recentPeakG(uint32_t windowMs) {
  uint32_t now = millis();
  float peak = 0;
  portENTER_CRITICAL(&mux);
  for (unsigned i = 0; i < kBuckets; ++i)
    if (vc::elapsedMs(now, bucketStart[i]) <= windowMs && buckets[i] > peak) peak = buckets[i];
  portEXIT_CRITICAL(&mux);
  return peak;
}

}  // namespace imu
