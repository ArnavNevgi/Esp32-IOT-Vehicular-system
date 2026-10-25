// Gateway pins, intervals and limits. Override any value with a build flag.
// Pin defaults match the original Receiver_iot_final.ino wiring.
#pragma once

#include "vc_util.h"

// ------------------------------------------------------ fitted hardware ----
// Set to 0 for a sensor that is not connected. A disabled sensor's pin is
// never read (an unconnected analog input floats and would produce false
// flame alarms), it raises no alarm and is reported as "disabled".
#ifndef GW_ENABLE_DHT22
#define GW_ENABLE_DHT22 1
#endif
#ifndef GW_ENABLE_FLAME
#define GW_ENABLE_FLAME 1
#endif
#ifndef GW_ENABLE_POT
#define GW_ENABLE_POT 1
#endif

// ------------------------------------------------------------------ pins ----
#ifndef GW_PIN_DHT22
#define GW_PIN_DHT22 4  // DHT22 data; needs a pull-up (most modules have one)
#endif
#ifndef GW_PIN_FLAME
#define GW_PIN_FLAME 33  // flame module analog output (ADC1_CH5: usable with Wi-Fi)
#endif
#ifndef GW_FLAME_INVERTED
#define GW_FLAME_INVERTED 1  // module AO voltage drops with more IR (original map(...,100,0))
#endif
#ifndef GW_PIN_POT
#define GW_PIN_POT 32  // potentiometer wiper (ADC1_CH4)
#endif
#ifndef GW_PIN_LED
#define GW_PIN_LED -1  // optional, -1 = not installed
#endif
#ifndef GW_LED_ACTIVE_HIGH
#define GW_LED_ACTIVE_HIGH 1
#endif
#ifndef GW_PIN_BUZZER
#define GW_PIN_BUZZER -1  // optional active buzzer, -1 = not installed
#endif
#ifndef GW_BUZZER_ACTIVE_HIGH
#define GW_BUZZER_ACTIVE_HIGH 1
#endif

// --------------------------------------------------------------- sampling ----
#ifndef GW_DHT_INTERVAL_MS
#define GW_DHT_INTERVAL_MS 3000  // DHT22 needs >= 2 s
#endif
#ifndef GW_DHT_FRESH_MS
#define GW_DHT_FRESH_MS 7000
#endif
#ifndef GW_DHT_MAX_CACHE_MS
#define GW_DHT_MAX_CACHE_MS 60000
#endif
#ifndef GW_FLAME_SAMPLE_MS
#define GW_FLAME_SAMPLE_MS 100
#endif
#ifndef GW_POT_SAMPLE_MS
#define GW_POT_SAMPLE_MS 500
#endif

// ----------------------------------------------------------------- alarms ----
#ifndef GW_FLAME_ON_PCT
#define GW_FLAME_ON_PCT 60.0f  // uncalibrated IR intensity index, 0-100
#endif
#ifndef GW_FLAME_OFF_PCT
#define GW_FLAME_OFF_PCT 40.0f
#endif
#ifndef GW_FLAME_ACTIVATE_MS
#define GW_FLAME_ACTIVATE_MS 300
#endif
#ifndef GW_FLAME_CLEAR_MS
#define GW_FLAME_CLEAR_MS 3000
#endif
#ifndef GW_LINK_LOSS_MS
#define GW_LINK_LOSS_MS 5000  // no accepted sensor packet for this long -> alarm
#endif
#ifndef GW_LINK_RESTORE_MS
#define GW_LINK_RESTORE_MS 2000  // packets flowing again for this long -> clear
#endif
#ifndef GW_LINK_BOOT_GRACE_MS
#define GW_LINK_BOOT_GRACE_MS 30000  // after gateway boot, before "never seen" alarms
#endif
#ifndef GW_REMINDER_MS
#define GW_REMINDER_MS 600000  // reminder alerts for alarms still active (10 min)
#endif

// ------------------------------------------------------------------ cloud ----
#ifndef GW_SUMMARY_MS
#define GW_SUMMARY_MS 60000  // routine summary to Azure
#endif
#ifndef GW_SAS_LIFETIME_MIN
#define GW_SAS_LIFETIME_MIN 60
#endif
#ifndef GW_SAS_REFRESH_MARGIN_S
#define GW_SAS_REFRESH_MARGIN_S 300  // reconnect with a new token this early
#endif
#ifndef GW_MQTT_KEEPALIVE_S
#define GW_MQTT_KEEPALIVE_S 60
#endif
#ifndef GW_MQTT_CONNECT_TIMEOUT_MS
#define GW_MQTT_CONNECT_TIMEOUT_MS 30000
#endif
#ifndef GW_CLOUD_MIN_SPACING_MS
#define GW_CLOUD_MIN_SPACING_MS 1000  // gradual drain of a backlog after reconnect
#endif
#ifndef GW_CLOUD_MAX_IN_FLIGHT
#define GW_CLOUD_MAX_IN_FLIGHT 2
#endif
#ifndef GW_CLOUD_ACK_TIMEOUT_MS
#define GW_CLOUD_ACK_TIMEOUT_MS 60000
#endif
// GW_CLOUD_ALERT_SLOTS (default 16) sizes a class in lib/vehicle_core, so it
// may only be changed with a build flag (seen by the library too), never here.
#ifndef GW_METER_BLOCK_BYTES
#define GW_METER_BLOCK_BYTES 512  // Azure IoT Hub free tier; 4096 for B/S tiers
#endif
#ifndef GW_TIME_SYNC_TIMEOUT_MS
#define GW_TIME_SYNC_TIMEOUT_MS 30000
#endif
#ifndef GW_HEALTH_LOG_MS
#define GW_HEALTH_LOG_MS 30000
#endif

// ------------------------------------------------------ compile-time checks ----
static_assert(!GW_ENABLE_DHT22 || vc::gpioUsableOutput(GW_PIN_DHT22), "DHT data pin must be bidirectional");
static_assert(!GW_ENABLE_DHT22 || !vc::gpioIsStrapping(GW_PIN_DHT22),
              "a sensor output on a strapping pin (0/2/5/12/15) can stop the board from booting");
static_assert(!GW_ENABLE_FLAME || vc::gpioIsAdc1(GW_PIN_FLAME),
              "flame sensor must be on ADC1 (GPIO32-39): ADC2 is unusable with Wi-Fi");
static_assert(!GW_ENABLE_POT || vc::gpioIsAdc1(GW_PIN_POT),
              "potentiometer must be on ADC1 (GPIO32-39): ADC2 is unusable with Wi-Fi");
namespace gw_pin_check {
constexpr int kPins[] = {GW_ENABLE_DHT22 ? GW_PIN_DHT22 : -1, GW_ENABLE_FLAME ? GW_PIN_FLAME : -1,
                         GW_ENABLE_POT ? GW_PIN_POT : -1, GW_PIN_LED, GW_PIN_BUZZER};
static_assert(vc::pinsDistinct(kPins), "two gateway functions share a GPIO");
}  // namespace gw_pin_check
static_assert(GW_PIN_LED < 0 || vc::gpioUsableOutput(GW_PIN_LED), "LED pin must be output-capable");
static_assert(GW_PIN_BUZZER < 0 || vc::gpioUsableOutput(GW_PIN_BUZZER), "buzzer pin must be output-capable");
static_assert(GW_FLAME_OFF_PCT < GW_FLAME_ON_PCT, "flame hysteresis: off threshold must be below on threshold");
static_assert(GW_SAS_REFRESH_MARGIN_S < GW_SAS_LIFETIME_MIN * 60, "SAS refresh margin must be shorter than the lifetime");
static_assert(GW_DHT_INTERVAL_MS >= 2000, "the DHT driver needs >= 2 s between reads");
