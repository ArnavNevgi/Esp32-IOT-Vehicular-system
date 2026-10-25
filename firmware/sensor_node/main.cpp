// Sensor node: 2x DHT11, MPU6050, shock sensor, PIR -> encrypted ESP-NOW -> gateway.
//
// Work is scheduled independently (see sensor_config.h for intervals):
//   imu task      ~100 Hz MPU6050 update (own FreeRTOS task, owns I2C)
//   shock ISR     edge capture -> impact records
//   loop          PIR poll 50 ms, DHT reads (one per call, staggered),
//                 tilt alarm 20 ms, telemetry 500 ms, link service, LED/buzzer
// Application logic (tilt, events, telemetry) lives in vc::SensorApp and the
// link protocol in vc::SensorLinkCore, both host-tested. Local alerts (tilt,
// impact chirp) work with or without the gateway.
#include <Arduino.h>
#include <WiFi.h>
#include <esp_heap_caps.h>

#include "alert_output.h"
#include "dht_sampler.h"
#include "espnow_common.h"
#include "imu_task.h"
#include "link_client.h"
#include "sensor_config.h"
#include "shock_capture.h"
#include "vc_secrets.h"
#include "vc_sensor_app.h"
#include "vc_telemetry.h"
#include "vc_timing.h"

#ifndef VC_SYNTHETIC_SENSORS
#define VC_SYNTHETIC_SENSORS 0
#endif

using namespace vc;
using namespace vc::proto;

namespace {

uint32_t bootId = 0;
NvsCounterStore bootStore("vc_sensor", "boot");
LinkClient gwLink;  // ("link" would clash with POSIX link())
SensorApp app;
AlertOutput alerts;
DhtSampler dhtA(SN_PIN_DHT11_A, DHT11);
DhtSampler dhtB(SN_PIN_DHT11_B, DHT11);
PeriodicTimer pirTimer(SN_PIR_POLL_MS), tiltTimer(20), telemetryTimer(SN_TELEMETRY_MS), healthTimer(SN_HEALTH_LOG_MS);
uint32_t telemetrySent = 0, telemetryRefused = 0;

#if VC_SYNTHETIC_SENSORS
// Deterministic fake readings for exercising scheduling, serialization and the
// cloud path without hardware. Every packet carries kFlagSynthetic and the
// gateway marks the data "synthetic": true. Never present these as real.
struct Synthetic {
  uint32_t lastImpactMs = 0;
  uint32_t impacts = 0;
  DhtReading dht(int i, uint32_t now) {
    DhtReading r;
    r.status = SensorStatus::Ok;
    r.tempC = 22.0f + i + 2.0f * sinf(now / 60000.0f);
    r.humidityPct = 45.0f + 5.0f * cosf(now / 90000.0f);
    r.ageMs = now % SN_DHT_INTERVAL_MS;
    return r;
  }
  ImuSample imu(uint32_t now) {
    ImuSample s;
    s.status = SensorStatus::Ok;
    // Tilts past the alarm threshold for 20 s every 5 minutes.
    float phase = (now % 300000u) / 1000.0f;
    s.rollDeg = phase > 200 && phase < 220 ? 50.0f : 5.0f * sinf(now / 3000.0f);
    s.pitchDeg = 2.0f;
    s.accelZG = 1.0f;
    s.accelMagG = 1.0f;
    s.peakAccelG = 1.05f;
    s.rateHz = 100.0f;
    s.maxGapMs = 10;
    s.sampleMs = now;
    return s;
  }
  bool impact(uint32_t now, ImpactRecord& out) {
    if (!hasElapsed(now, lastImpactMs, 90000)) return false;
    lastImpactMs = now;
    out.detectedMs = now;
    out.pulses = 2;
    ++impacts;
    return true;
  }
} synthetic;
#endif

#if !VC_SYNTHETIC_SENSORS
ImuSample toSample(const ImuSnapshot& s) {
  ImuSample o;
  o.status = s.status;
  o.accelXG = s.accelXG;
  o.accelYG = s.accelYG;
  o.accelZG = s.accelZG;
  o.accelMagG = s.accelMagG;
  o.peakAccelG = s.peakAccelG;
  o.rollDeg = s.rollDeg;
  o.pitchDeg = s.pitchDeg;
  o.rateHz = s.rateHz;
  o.maxGapMs = s.maxGapMs;
  o.sampleMs = s.sampleMs;
  return o;
}
#endif

ImuSample imuPeek(uint32_t now) {
#if VC_SYNTHETIC_SENSORS
  return synthetic.imu(now);
#else
  (void)now;
  return SN_ENABLE_MPU6050 ? toSample(imu::peek()) : ImuSample();
#endif
}

ShockCounters shockCounters() {
  ShockCounters c;
#if VC_SYNTHETIC_SENSORS
  c.impacts = synthetic.impacts;
  c.edges = synthetic.impacts * 2;
#else
  if (SN_ENABLE_SHOCK) {
    c.impacts = shock::impacts();
    c.edges = shock::edges();
    c.overflows = shock::overflows();
  }
#endif
  return c;
}

void handleImpacts(uint32_t now) {
  ImpactRecord rec;
#if VC_SYNTHETIC_SENSORS
  if (!synthetic.impact(now, rec)) return;
  float peak = 2.5f;
#else
  if (!SN_ENABLE_SHOCK || !shock::pop(rec, now)) return;
  float peak = SN_ENABLE_MPU6050 ? imu::recentPeakG(320) : 0;
#endif
  app.onImpact(rec.detectedMs, rec.pulses, peak);
  alerts.chirp(3, now);
}

void sendTelemetry(uint32_t now) {
#if VC_SYNTHETIC_SENSORS
  DhtReading a = synthetic.dht(0, now), b = synthetic.dht(1, now);
  ImuSample s = synthetic.imu(now);
#else
  DhtReading a = dhtA.reading(now), b = dhtB.reading(now);
  // take() also resets the per-packet peak and gap.
  ImuSample s = SN_ENABLE_MPU6050 ? toSample(imu::take()) : ImuSample();
#endif
  TelemetryBody t = app.buildTelemetry(now, a, b, s, shockCounters());
  if (gwLink.core().sendTelemetry(t, now)) ++telemetrySent;
  else ++telemetryRefused;
}

void logHealth(uint32_t now) {
  const SensorLinkCore& link = gwLink.core();
  ImuSnapshot s = SN_ENABLE_MPU6050 && !VC_SYNTHETIC_SENSORS ? imu::peek() : ImuSnapshot();
  TelemetryBody t = app.buildTelemetry(now, dhtA.reading(now), dhtB.reading(now), imuPeek(now), shockCounters());
  ShockCounters sc = shockCounters();
  Serial.printf(
      "[health] up %lus link=%s ch=%u restarts=%u lastGatewayRx=%lums | events pending=%u hw=%u dropped=%lu "
      "expired=%lu | imu=%s %.1fHz maxUpd=%luus overruns=%lu i2cErr=%lu | dht1=%s dht2=%s maxRead=%lu/%luus | "
      "shock edges=%lu impacts=%lu overflow=%lu | tx ok=%lu refused=%lu radioFail=%lu rxRej=%lu rxOvf=%lu "
      "resOvf=%lu | heap=%lu min=%lu\n",
      (unsigned long)(now / 1000), link.stateName(), link.channel(), link.linkRestarts(),
      (unsigned long)link.lastGatewayRxAgeMs(now), link.eventsPending(), link.eventsHighWater(),
      (unsigned long)link.eventsDropped(), (unsigned long)link.eventsExpired(),
      vc::telemetry::statusName(t.imu.status), s.rateHz, (unsigned long)s.maxUpdateUs, (unsigned long)s.overruns,
      (unsigned long)s.i2cErrors, vc::telemetry::statusName(t.dht[0].status),
      vc::telemetry::statusName(t.dht[1].status), (unsigned long)dhtA.maxReadDurationUs(),
      (unsigned long)dhtB.maxReadDurationUs(), (unsigned long)sc.edges, (unsigned long)sc.impacts,
      (unsigned long)sc.overflows, (unsigned long)telemetrySent, (unsigned long)telemetryRefused,
      (unsigned long)link.stats().radioSendFailures, (unsigned long)link.stats().rxRejected,
      (unsigned long)gwLink.rxQueueOverflows(), (unsigned long)gwLink.sendResultOverflows(),
      (unsigned long)ESP.getFreeHeap(), (unsigned long)heap_caps_get_minimum_free_size(MALLOC_CAP_DEFAULT));
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(200);
  installSerialLog();
  WiFi.mode(WIFI_STA);
  Serial.printf("\n=== Vehicle sensor node === station MAC %s, secrets from %s\n", WiFi.macAddress().c_str(),
                VC_SECRETS_SOURCE);
#if VC_SYNTHETIC_SENSORS
  Serial.println("!!! SYNTHETIC SENSOR MODE: readings are generated, NOT measured. Packets are flagged synthetic. !!!");
#endif
  Serial.printf("fitted: dht11_a=%d dht11_b=%d mpu6050=%d shock=%d pir=%d led=%d buzzer=%d\n", SN_ENABLE_DHT11_A,
                SN_ENABLE_DHT11_B, SN_ENABLE_MPU6050, SN_ENABLE_SHOCK, SN_ENABLE_PIR, SN_PIN_LED >= 0,
                SN_PIN_BUZZER >= 0);
  bootId = nextBootCounter("vc_sensor");
  Serial.printf("boot id %lu\n", (unsigned long)bootId);

  SensorAppConfig ac;
  ac.tiltOnDeg = SN_TILT_ON_DEG;
  ac.tiltOffDeg = SN_TILT_OFF_DEG;
  ac.tiltActivateMs = SN_TILT_ACTIVATE_MS;
  ac.tiltClearMs = SN_TILT_CLEAR_MS;
  ac.dhtEnabled[0] = VC_SYNTHETIC_SENSORS || SN_ENABLE_DHT11_A;
  ac.dhtEnabled[1] = VC_SYNTHETIC_SENSORS || SN_ENABLE_DHT11_B;
  ac.imuEnabled = VC_SYNTHETIC_SENSORS || SN_ENABLE_MPU6050;
  ac.shockEnabled = VC_SYNTHETIC_SENSORS || SN_ENABLE_SHOCK;
  ac.pirEnabled = !VC_SYNTHETIC_SENSORS && SN_ENABLE_PIR;
  app.begin(ac, &gwLink.core());

  alerts.begin(SN_PIN_LED, SN_LED_ACTIVE_HIGH, SN_PIN_BUZZER, SN_BUZZER_ACTIVE_HIGH);
  uint32_t now = millis();
#if !VC_SYNTHETIC_SENSORS
  if (SN_ENABLE_SHOCK) shock::begin(SN_PIN_SHOCK, SN_SHOCK_ACTIVE_HIGH, SN_IMPACT_REFRACTORY_MS);
  if (SN_ENABLE_PIR) pinMode(SN_PIN_PIR, INPUT);
  if (SN_ENABLE_DHT11_A) dhtA.begin(now, SN_DHT_INTERVAL_MS, SN_DHT_FRESH_MS, SN_DHT_MAX_CACHE_MS, 2000);
  if (SN_ENABLE_DHT11_B)
    dhtB.begin(now, SN_DHT_INTERVAL_MS, SN_DHT_FRESH_MS, SN_DHT_MAX_CACHE_MS, 2000 + SN_DHT_INTERVAL_MS / 2);
  if (SN_ENABLE_MPU6050 && !imu::start(SN_PIN_I2C_SDA, SN_PIN_I2C_SCL, SN_I2C_HZ, SN_IMU_PERIOD_MS, true))
    Serial.println("ERROR: could not start IMU task");
#endif

  LinkConfig cfg = loadLinkConfig();
  if (!cfg.ok) {
    Serial.printf("UNCONFIGURED: local link disabled (%s). Sensing and local alerts continue.\n", cfg.problem);
  } else if (bootId == 0) {
    Serial.println("ERROR: NVS boot counter unavailable; local link disabled to preserve replay protection.");
  } else if (!gwLink.begin(cfg, bootId, VC_SYNTHETIC_SENSORS, &bootStore)) {
    Serial.println("ERROR: ESP-NOW initialisation failed; local link disabled.");
  }

  pirTimer.start(now);
  tiltTimer.start(now);
  telemetryTimer.start(now);
  healthTimer.start(now);
  enableLoopWDT();  // the loop must come back within the task-watchdog timeout
}

void loop() {
  uint32_t now = millis();

#if !VC_SYNTHETIC_SENSORS
  if (SN_ENABLE_PIR && pirTimer.due(now)) app.onPirSample(digitalRead(SN_PIN_PIR) == (SN_PIR_ACTIVE_HIGH ? HIGH : LOW));
  // At most one (blocking, ~25 ms) DHT read per loop pass.
  bool dhtRead = SN_ENABLE_DHT11_A && dhtA.poll(now);
  if (!dhtRead && SN_ENABLE_DHT11_B) dhtB.poll(now);
#endif

  handleImpacts(now);
  if (tiltTimer.due(now)) app.evaluateTilt(imuPeek(now), now);
  if (telemetryTimer.due(now)) sendTelemetry(now);
  gwLink.tick(now);

  alerts.setAlarm(app.tiltActive());
  alerts.setAttention(!gwLink.core().linked());
  alerts.update(now);

  if (healthTimer.due(now)) logHealth(now);
  delay(1);  // yield; the IMU task and ISR keep their own timing
}
