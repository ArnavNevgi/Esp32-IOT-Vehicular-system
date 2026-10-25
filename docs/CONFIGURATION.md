# Configuration, first flash and troubleshooting

## Secrets (`config/secrets.h`)

Both boards build from the same file. It is git-ignored; start from the placeholder example:

```sh
cp config/secrets.example.h config/secrets.h
```

| Setting | Used by | How to get it |
|---|---|---|
| `VC_WIFI_SSID`, `VC_WIFI_PASSWORD` | gateway | your 2.4 GHz access point |
| `VC_IOTHUB_FQDN` | gateway | `<hub-name>.azure-devices.net` (host only) |
| `VC_IOTHUB_DEVICE_ID`, `VC_IOTHUB_DEVICE_KEY` | gateway | an **existing** device identity (the gateway) with symmetric-key auth, e.g. `az iot hub device-identity show -n <hub> -d <id> --query authentication.symmetricKey.primaryKey` (prints a secret) |
| `VC_ESPNOW_PMK_HEX`, `VC_ESPNOW_LMK_HEX` | both | two **different** random 16-byte keys: `python -c "import secrets; print(secrets.token_hex(16))"` |
| `VC_SENSOR_NODE_MAC`, `VC_GATEWAY_MAC` | both | flash once, read `station MAC ...` from each board's boot log |
| `VC_SENSOR_NODE_ID` | gateway | logical name in cloud messages (default `sensor-node-01`) |

Any value that still starts with `YOUR_` is treated as unset. The board then prints
`UNCONFIGURED: ...` and **never attempts** a Wi-Fi join, Azure login or ESP-NOW session with it,
while local sensing and alarms keep working. Keys, passwords and tokens are never logged.

## Build flags

Set them in `platformio.ini`, for example
`build_flags = ${esp32_common.build_flags} -D GW_SUMMARY_MS=120000 -D GW_ENABLE_FLAME=0`.

### Sensor node

| Flag | Default | Meaning |
|---|---|---|
| `SN_ENABLE_DHT11_A/B`, `SN_ENABLE_MPU6050`, `SN_ENABLE_SHOCK`, `SN_ENABLE_PIR` | 1 | fitted hardware |
| `SN_PIN_DHT11_A/B`, `SN_PIN_SHOCK`, `SN_PIN_PIR`, `SN_PIN_I2C_SDA/SCL` | 14/27, 25, 26, 21/22 | pins |
| `SN_SHOCK_ACTIVE_HIGH`, `SN_PIR_ACTIVE_HIGH`, `SN_I2C_HZ` | 1, 1, 400000 | signal polarity, I²C clock |
| `SN_PIN_LED`, `SN_PIN_BUZZER` (+ `_ACTIVE_HIGH`) | -1 | optional outputs (-1 = none; active buzzer only) |
| `SN_IMU_PERIOD_MS` | 10 | MPU update period (~100 Hz) |
| `SN_DHT_INTERVAL_MS` / `SN_DHT_FRESH_MS` / `SN_DHT_MAX_CACHE_MS` | 4000 / 9000 / 60000 | DHT read period; older → `stale`; older → withheld |
| `SN_PIR_POLL_MS`, `SN_TELEMETRY_MS` | 50, 500 | PIR poll; telemetry to the gateway |
| `SN_TILT_ON_DEG` / `SN_TILT_OFF_DEG` | 45 / 35 | tilt hysteresis |
| `SN_TILT_ACTIVATE_MS` / `SN_TILT_CLEAR_MS` | 1000 / 2000 | tilt debounce |
| `SN_IMPACT_REFRACTORY_MS` | 200 | edges closer than this are one impact |
| `SN_EVENT_RETRY_MS` / `SN_EVENT_MAX_ATTEMPTS` | 300 / 40 | event retransmission |
| `SN_LINK_HELLO_DWELL_MS` / `SN_LINK_MAX_CHANNEL` / `SN_LINK_FAILS_BEFORE_RESCAN` | 60 / 13 / 8 | channel sweep |
| `SN_HEALTH_LOG_MS` | 10000 | `[health]` line period |

### Gateway

| Flag | Default | Meaning |
|---|---|---|
| `GW_ENABLE_DHT22`, `GW_ENABLE_FLAME`, `GW_ENABLE_POT` | 1 | fitted hardware |
| `GW_PIN_DHT22`, `GW_PIN_FLAME`, `GW_PIN_POT` | 4, 33, 32 | pins (analog pins must be ADC1, GPIO 32–39) |
| `GW_FLAME_INVERTED` | 1 | module output falls with more IR |
| `GW_PIN_LED`, `GW_PIN_BUZZER` (+ `_ACTIVE_HIGH`) | -1 | optional outputs |
| `GW_DHT_INTERVAL_MS` / `GW_DHT_FRESH_MS` / `GW_DHT_MAX_CACHE_MS` | 3000 / 7000 / 60000 | DHT22 timing |
| `GW_FLAME_SAMPLE_MS`, `GW_POT_SAMPLE_MS` | 100, 500 | analog sampling |
| `GW_FLAME_ON_PCT` / `GW_FLAME_OFF_PCT` | 60 / 40 | flame hysteresis (uncalibrated index) |
| `GW_FLAME_ACTIVATE_MS` / `GW_FLAME_CLEAR_MS` | 300 / 3000 | flame debounce |
| `GW_LINK_LOSS_MS` / `GW_LINK_RESTORE_MS` / `GW_LINK_BOOT_GRACE_MS` | 5000 / 2000 / 30000 | sensor-link alarm |
| `GW_REMINDER_MS` | 600000 | reminders for still-active alarms |
| `GW_SUMMARY_MS` | 60000 | routine summary (120000 recommended on the free tier) |
| `GW_SAS_LIFETIME_MIN` / `GW_SAS_REFRESH_MARGIN_S` | 60 / 300 | token lifetime; renew this early |
| `GW_MQTT_KEEPALIVE_S` / `GW_MQTT_CONNECT_TIMEOUT_MS` | 60 / 30000 | MQTT |
| `GW_CLOUD_MIN_SPACING_MS` / `GW_CLOUD_MAX_IN_FLIGHT` / `GW_CLOUD_ACK_TIMEOUT_MS` | 1000 / 2 / 60000 | outbox drain |
| `GW_CLOUD_ALERT_SLOTS` | 16 | alert slots in the outbox (**build flag only**: it sizes a library class) |
| `GW_METER_BLOCK_BYTES` | 512 | block size for `estMeterUnits` (4096 on B/S tiers) |
| `GW_TIME_SYNC_TIMEOUT_MS`, `GW_HEALTH_LOG_MS` | 30000, 30000 | SNTP attempt; `[health]` period |

A sensor built with its `*_ENABLE_*` flag set to `0` is never initialised or read, raises no
alarm, and appears in telemetry with `"status": "disabled"` and `null` values. Pin choices are
checked at compile time (flash pins, strapping pins, input-only pins, ADC1, duplicates).

## First flash, step by step

1. `python tools/verify.py` → `ALL CHECKS PASSED`.
2. Flash the gateway with placeholders (one board connected):
   `pio run -e gateway -t upload && pio device monitor -e gateway`. Note its `station MAC`.
3. Flash the sensor node the same way (`-e sensor_node`). Note its `station MAC`. Keep it still
   and level during the ~1 s MPU calibration after every boot.
4. Create `config/secrets.h`: both MACs, two new ESP-NOW keys, then Wi-Fi and the gateway's IoT
   Hub identity.
5. Re-flash **both** boards (the ESP-NOW settings must match). Use `--upload-port` if both are
   connected.
6. Follow [HARDWARE_TESTS.md](HARDWARE_TESTS.md) stages 0–7 in order.

### Expected boot messages

Gateway, configured, first boot (values vary):

```
=== Vehicle gateway === station MAC 24:6F:28:xx:xx:xx, secrets from config/secrets.h
fitted: dht22=1 flame=1 pot=1 led=0 buzzer=0
gateway boot id 1
[wifi] connecting to "<ssid>"
[wifi] connected, IP 192.168.x.x, channel 6, RSSI -55 dBm
[time] SNTP started
[time] UTC clock synchronised
[azure] TLS + MQTT 3.1.1 to <hub>.azure-devices.net:8883 as device <gateway-id>
[azure] connecting (client generation 1)
[azure] connected (MQTT CONNACK accepted); token refresh in 3299 s
[link] session 5a1c09e2 for sensor boot 3 on channel 6 (RSSI -48)
[health] up 30s wifi=connected ch=6 rssi=-55 time=valid cloud=connected token=3274s | sensor link=up ...
```

Sensor node:

```
=== Vehicle sensor node === station MAC 24:6F:28:xx:xx:xx, secrets from config/secrets.h
fitted: dht11_a=1 dht11_b=1 mpu6050=1 shock=1 pir=1 led=0 buzzer=0
boot id 3
[imu] MPU6050 found, WHO_AM_I=0x68
[imu] calibrating: keep the vehicle still and level
[link] linked on channel 6, session 5a1c09e2
[health] up 10s link=linked ch=6 restarts=0 lastGatewayRx=... | events pending=0 hw=0 ... | imu=ok 100.0Hz ...
```

### Health output

Each board prints a `[health]` line (sensor every 10 s, gateway every 30 s):

* **Sensor:** link state and channel, restarts, last gateway packet age; event queue `pending`,
  high-water `hw`, `dropped`, `expired`; IMU status, rate, longest update `maxUpd`, `overruns`,
  `i2cErr`; DHT statuses and longest read; shock `edges`/`impacts`/`overflow`; telemetry
  sent/refused, radio send failures, rejected frames, queue overflows; free and minimum heap.
* **Gateway:** Wi-Fi, time, cloud state and last error, token time left; sensor-link counters
  (`ok`, `tag`, `malformed`, `replay`, `session`, `peer`, `dupEv`, `rxOvf`, `rxHw`); outbox depth,
  high-water `hw`, in flight, acked, failures, metering estimate; DHT22 / flame; free and
  minimum heap.

## Calibration

* **MPU6050:** calibrates for about 1 s at every boot and after a reconnect. Keep the vehicle
  **stationary and level, chip Z axis up**; that orientation then reads roll = pitch = 0.
* **Flame:** the index is uncalibrated; measure your module and set `GW_FLAME_ON_PCT` /
  `GW_FLAME_OFF_PCT`.
* **Shock:** set the module's sensitivity potentiometer; `SN_IMPACT_REFRACTORY_MS` groups chatter.

## Troubleshooting

| Symptom | Check |
|---|---|
| `UNCONFIGURED: ...` at boot | `config/secrets.h` missing or still has `YOUR_` values; rebuild and re-flash |
| `ERROR: NVS boot counter unavailable` (sensor) | erase flash (`pio run -e sensor_node -t erase`) and re-flash; the gateway then moves the sensor past its old boot ids automatically |
| Sensor `no gateway answered on channels 1-13` | MACs swapped or mistyped, keys differ between boards, gateway not running, or AP on channel 14 |
| Gateway `tag=` count rising | the boards were built with different `VC_ESPNOW_LMK_HEX`; different firmware generations show as `malformed` |
| `refused Hello from sensor boot ...` once, then linked | expected after a sensor flash erase |
| `CONNECT refused: not authorized` | wrong device id or key, disabled device, or clock far off; next attempt waits ≥ 60 s |
| `transport/TLS error ... tls_stack=...` | outbound 8883 blocked, wrong hub host name, or a TLS-inspecting proxy |
| `time=syncing` for a long time | UDP 123 blocked; NTP servers unreachable |
| No summaries in Azure but `cloud=connected` | `az iot hub monitor-events --properties all`; check `fail`/`outbox` in the health line |
| `imu=absent` | wiring, 3.3 V supply, AD0 low (address 0x68) |
| Random flame alarms with no module | build with `-D GW_ENABLE_FLAME=0` |
