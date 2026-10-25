// Example configuration with PLACEHOLDER values only.
//
//   cp config/secrets.example.h config/secrets.h   (secrets.h is git-ignored)
//
// Firmware built with these placeholders still compiles and runs its local
// sensing and alerts, but reports UNCONFIGURED and never attempts a Wi-Fi,
// Azure or ESP-NOW connection with them. Any value beginning with "YOUR_" is
// treated as unset. Both boards are built from the same secrets.h.
#pragma once

// ---------------------------------------------------------------- gateway ----
// Upstream Wi-Fi access point the gateway joins for internet access.
#define VC_WIFI_SSID "YOUR_WIFI_SSID"
#define VC_WIFI_PASSWORD "YOUR_WIFI_PASSWORD"

// Azure IoT Hub device identity of the GATEWAY (symmetric-key authentication).
// Hub host name only, e.g. "my-hub.azure-devices.net" (no scheme, no port).
#define VC_IOTHUB_FQDN "YOUR_HUB_NAME.azure-devices.net"
#define VC_IOTHUB_DEVICE_ID "YOUR_GATEWAY_DEVICE_ID"
// Base64 primary (or secondary) key of that device. Never log or commit it.
#define VC_IOTHUB_DEVICE_KEY "YOUR_DEVICE_KEY_BASE64"

// ----------------------------------------------------- local ESP-NOW link ----
// Identical on both boards. Generate each with, for example:
//   python -c "import secrets; print(secrets.token_hex(16))"
// PMK: primary master key (encrypts the LMK). LMK: per-peer local master key.
#define VC_ESPNOW_PMK_HEX "YOUR_32_HEX_DIGITS_PMK"
#define VC_ESPNOW_LMK_HEX "YOUR_32_HEX_DIGITS_LMK"

// Wi-Fi station MAC addresses, "AA:BB:CC:DD:EE:FF". Each board prints its own
// at boot ("station MAC ...").
#define VC_SENSOR_NODE_MAC "YOUR_SENSOR_NODE_STA_MAC"
#define VC_GATEWAY_MAC "YOUR_GATEWAY_STA_MAC"

// Logical id of the sensor node, used in cloud messages and event ids.
#define VC_SENSOR_NODE_ID "sensor-node-01"
