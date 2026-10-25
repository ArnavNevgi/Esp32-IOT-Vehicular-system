// Sensor-node pins and timing. Every value can be overridden with a build flag,
// e.g. build_flags = -D SN_PIN_SHOCK=33. Pin defaults match the original
// sender_iot_project_FINAL.ino wiring.
#pragma once

#include "vc_util.h"

// ------------------------------------------------------ fitted hardware ----
// Set to 0 for a sensor that is not connected. A disabled sensor is never
// read (its pin is left untouched), raises no alarm, does not delay start-up,
// and is reported as "disabled" in telemetry. Defaults: the original wiring.
#ifndef SN_ENABLE_DHT11_A
#define SN_ENABLE_DHT11_A 1
#endif
#ifndef SN_ENABLE_DHT11_B
#define SN_ENABLE_DHT11_B 1
#endif
#ifndef SN_ENABLE_MPU6050
#define SN_ENABLE_MPU6050 1  // also required for tilt alarms and impact peak g
#endif
#ifndef SN_ENABLE_SHOCK
#define SN_ENABLE_SHOCK 1
#endif
#ifndef SN_ENABLE_PIR
#define SN_ENABLE_PIR 1
#endif

// ------------------------------------------------------------------ pins ----
#ifndef SN_PIN_DHT11_A
#define SN_PIN_DHT11_A 14  // DHT11 #1 data (original "DHTPIN"); needs a pull-up
#endif
#ifndef SN_PIN_DHT11_B
#define SN_PIN_DHT11_B 27  // DHT11 #2 data (original "DHT1PIN")
#endif
#ifndef SN_PIN_SHOCK
#define SN_PIN_SHOCK 25  // shock/vibration module digital output
#endif
#ifndef SN_SHOCK_ACTIVE_HIGH
#define SN_SHOCK_ACTIVE_HIGH 1  // original code treated HIGH as "shock"
#endif
#ifndef SN_PIN_PIR
#define SN_PIN_PIR 26  // PIR (e.g. HC-SR501) output, HIGH while motion
#endif
#ifndef SN_PIR_ACTIVE_HIGH
#define SN_PIR_ACTIVE_HIGH 1
#endif
#ifndef SN_PIN_I2C_SDA
#define SN_PIN_I2C_SDA 21  // MPU6050 SDA (ESP32 Arduino default)
#endif
#ifndef SN_PIN_I2C_SCL
#define SN_PIN_I2C_SCL 22  // MPU6050 SCL
#endif
#ifndef SN_I2C_HZ
#define SN_I2C_HZ 400000
#endif
// Optional outputs; -1 = not installed.
#ifndef SN_PIN_LED
#define SN_PIN_LED -1
#endif
#ifndef SN_LED_ACTIVE_HIGH
#define SN_LED_ACTIVE_HIGH 1
#endif
#ifndef SN_PIN_BUZZER
#define SN_PIN_BUZZER -1
#endif
#ifndef SN_BUZZER_ACTIVE_HIGH
#define SN_BUZZER_ACTIVE_HIGH 1
#endif

// ---------------------------------------------------------------- timing ----
#ifndef SN_IMU_PERIOD_MS
#define SN_IMU_PERIOD_MS 10  // ~100 Hz MPU6050 read + complementary filter
#endif
#ifndef SN_DHT_INTERVAL_MS
#define SN_DHT_INTERVAL_MS 4000  // per sensor; the two sensors are staggered by half
#endif
#ifndef SN_DHT_FRESH_MS
#define SN_DHT_FRESH_MS 9000  // older -> "stale"
#endif
#ifndef SN_DHT_MAX_CACHE_MS
#define SN_DHT_MAX_CACHE_MS 60000  // older -> value withheld
#endif
#ifndef SN_PIR_POLL_MS
#define SN_PIR_POLL_MS 50
#endif
#ifndef SN_TELEMETRY_MS
#define SN_TELEMETRY_MS 500  // local telemetry to the gateway
#endif
#ifndef SN_EVENT_RETRY_MS
#define SN_EVENT_RETRY_MS 300
#endif
#ifndef SN_EVENT_MAX_ATTEMPTS
#define SN_EVENT_MAX_ATTEMPTS 40  // ~12 s of link-up retries per event
#endif
#ifndef SN_HEALTH_LOG_MS
#define SN_HEALTH_LOG_MS 10000
#endif

// --------------------------------------------------------------- detection ----
#ifndef SN_IMPACT_REFRACTORY_MS
#define SN_IMPACT_REFRACTORY_MS 200  // edges closer than this belong to one impact
#endif
#ifndef SN_TILT_ON_DEG
#define SN_TILT_ON_DEG 45.0f  // max(|roll|,|pitch|) to raise the tilt alarm
#endif
#ifndef SN_TILT_OFF_DEG
#define SN_TILT_OFF_DEG 35.0f  // ...and to clear it (hysteresis)
#endif
#ifndef SN_TILT_ACTIVATE_MS
#define SN_TILT_ACTIVATE_MS 1000
#endif
#ifndef SN_TILT_CLEAR_MS
#define SN_TILT_CLEAR_MS 2000
#endif

// -------------------------------------------------------------------- link ----
#ifndef SN_LINK_HELLO_DWELL_MS
#define SN_LINK_HELLO_DWELL_MS 60  // wait for HelloAck on each channel
#endif
#ifndef SN_LINK_MAX_CHANNEL
#define SN_LINK_MAX_CHANNEL 13
#endif
#ifndef SN_LINK_FAILS_BEFORE_RESCAN
#define SN_LINK_FAILS_BEFORE_RESCAN 8  // consecutive unacknowledged radio sends
#endif

// ------------------------------------------------------ compile-time checks ----
// Classic ESP32 (esp32dev): GPIO6-11 are the flash bus, 34-39 are input-only
// without internal pulls, 0/2/5/12/15 are boot strapping pins that an external
// module must not drive during reset.
static_assert(!SN_ENABLE_DHT11_A || vc::gpioUsableOutput(SN_PIN_DHT11_A), "DHT data pin must be bidirectional");
static_assert(!SN_ENABLE_DHT11_B || vc::gpioUsableOutput(SN_PIN_DHT11_B), "DHT data pin must be bidirectional");
static_assert(!SN_ENABLE_SHOCK || vc::gpioHasInternalPull(SN_PIN_SHOCK),
              "shock pin needs an internal pull resistor (not GPIO34-39) to stay defined when the module floats");
static_assert(!SN_ENABLE_PIR || vc::gpioUsableInput(SN_PIN_PIR), "invalid PIR pin");
static_assert(!SN_ENABLE_MPU6050 || (vc::gpioUsableOutput(SN_PIN_I2C_SDA) && vc::gpioUsableOutput(SN_PIN_I2C_SCL)),
              "invalid I2C pins");
static_assert((!SN_ENABLE_DHT11_A || !vc::gpioIsStrapping(SN_PIN_DHT11_A)) &&
                  (!SN_ENABLE_DHT11_B || !vc::gpioIsStrapping(SN_PIN_DHT11_B)) &&
                  (!SN_ENABLE_SHOCK || !vc::gpioIsStrapping(SN_PIN_SHOCK)) &&
                  (!SN_ENABLE_PIR || !vc::gpioIsStrapping(SN_PIN_PIR)),
              "a sensor output on a strapping pin (0/2/5/12/15) can stop the board from booting");
namespace sn_pin_check {
constexpr int kPins[] = {SN_ENABLE_DHT11_A ? SN_PIN_DHT11_A : -1,
                         SN_ENABLE_DHT11_B ? SN_PIN_DHT11_B : -1,
                         SN_ENABLE_SHOCK ? SN_PIN_SHOCK : -1,
                         SN_ENABLE_PIR ? SN_PIN_PIR : -1,
                         SN_ENABLE_MPU6050 ? SN_PIN_I2C_SDA : -1,
                         SN_ENABLE_MPU6050 ? SN_PIN_I2C_SCL : -1,
                         SN_PIN_LED,
                         SN_PIN_BUZZER};
static_assert(vc::pinsDistinct(kPins), "two sensor-node functions share a GPIO");
}  // namespace sn_pin_check
static_assert(SN_PIN_LED < 0 || vc::gpioUsableOutput(SN_PIN_LED), "LED pin must be output-capable");
static_assert(SN_PIN_BUZZER < 0 || vc::gpioUsableOutput(SN_PIN_BUZZER), "buzzer pin must be output-capable");
static_assert(SN_DHT_INTERVAL_MS >= 2000, "the DHT driver needs >= 2 s between reads");
static_assert(SN_IMU_PERIOD_MS >= 2, "IMU period too short");
