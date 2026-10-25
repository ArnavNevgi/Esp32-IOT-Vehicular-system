# Staged hardware and Azure validation

**Status: none of these checks has been performed.** Everything so far is verified only by
compiling the firmware and by host tests (README → *Verification status*). The host
integration harness runs the production logic against a *simulated* radio, clock, NVS and
broker; it cannot show sensor accuracy, radio reliability, ESP32 heap behaviour, real task
timing, esp-mqtt/TLS behaviour on the board or IoT Hub behaviour.

Work through the stages in order; each one assumes the previous passed. Record date, board
revision, firmware commit and the result next to each item. Results marked **pending** in the
README are measured here.

Equipment: two ESP32 DevKit boards (`esp32dev`, ESP32-D0WD), the sensors wired as in the README,
USB cables, a 2.4 GHz access point you can reconfigure (channel, power, uplink), and for stage 5
an existing test IoT Hub with the gateway device registered plus Azure CLI with the `azure-iot`
extension. A second microcontroller or signal generator for pulse tests is optional.

Serial monitor: `pio device monitor -e sensor_node` / `-e gateway` (115200 baud, exception
decoder enabled).

## Stage 0: first flash with placeholders (one board at a time)

- [ ] `python tools/verify.py` passes on the PC first.
- [ ] Flash the **gateway** with no `config/secrets.h`. Expected log:
      `=== Vehicle gateway === station MAC 24:..., secrets from config/secrets.example.h (placeholders)`,
      `fitted: dht22=1 flame=1 pot=1 ...`, `gateway boot id 1`, then three `UNCONFIGURED:` lines
      (Wi-Fi, Azure, sensor link). No `[wifi] connecting`, no `[azure]` line ever appears.
- [ ] Flash the **sensor node** likewise: `UNCONFIGURED: local link disabled (ESP-NOW keys are
      placeholders)`, `[imu] MPU6050 found, WHO_AM_I=0x68`, `[imu] calibrating ...`.
- [ ] Reset each board three times: `boot id` increments by one per reset (NVS works).
- [ ] Note both station MACs for `config/secrets.h`.

## Stage 1: sensor-node sensors (no gateway needed)

The `[health]` line appears every 10 s. Record values in the table at the end.

- [ ] **MPU cadence**: `imu=ok`, rate ≈ 100 Hz. Record min/max over 10 min, `maxUpd` (longest
      update, µs), `overruns` (expect 0 at 400 kHz), `i2cErr`.
- [ ] **Tilt calibration**: boot level and still; tilt by known angles (protractor / phone
      inclinometer) and compare `rollDeg`/`pitchDeg` at 0°, ±30°, ±60°.
- [ ] **Tilt alarm**: > 45° for > 1 s → `[event] tilt ACTIVE`; < 35° for > 2 s → `tilt cleared`;
      hovering at 40° produces no chatter.
- [ ] **DHT11 ×2**: `dht1=ok dht2=ok`; `maxRead` ≈ 25,000 µs per sensor. Unplug one: `stale`
      after 9 s, `error` after 60 s; replug: `ok` without reboot. Boot with one missing: `absent`.
- [ ] **Shock**: one tap → one `[event] impact #n ... pulse(s)` and `shock edges` grows. Bursts
      < 200 ms apart count as one impact with more pulses.
- [ ] **Short pulses** (optional generator, 3.3 V logic on GPIO25): 1 ms, 100 µs, 10 µs pulses at
      1 Hz for 60 s, with and without DHT reads in progress; compare `shock edges` with the pulse
      count. A width that is missed needs a hardware latch/one-shot.
- [ ] **PIR**: motion → `pir` rises counted (visible in stage 3 summaries).
- [ ] **Unplug the MPU6050**: `imu=absent` within ~50 ms, no tilt from stale data; replug →
      detected and recalibrated within 5 s (keep still).
- [ ] **Disabled sensors**: build with e.g. `-D SN_ENABLE_PIR=0 -D SN_ENABLE_DHT11_B=0`, leave
      them unplugged: `fitted: ... pir=0`, no errors for them, the rest unchanged.

## Stage 2: gateway sensors (no Wi-Fi needed)

- [ ] **DHT22**: `dht22=ok` in `[health]` (every 30 s); unplug/replug as above.
- [ ] **Flame**: record the `flame=` index in normal light, sunlight, and with a lighter at
      10/30/50 cm. Adjust `GW_FLAME_ON_PCT`/`GW_FLAME_OFF_PCT`. Activation and clearing produce
      exactly one `[alert] flame active` / `cleared` each.
- [ ] **Floating input**: disconnect the flame AO wire and record the readings — this is why an
      unfitted module must be built with `-D GW_ENABLE_FLAME=0`. With that flag: `flame=disabled`,
      no flame alerts with the wire floating.
- [ ] **Potentiometer**: index follows the knob 0–100.

## Stage 3: local link (both boards, gateway Wi-Fi may stay unconfigured)

Fill in `config/secrets.h` with the ESP-NOW keys and both MACs (Wi-Fi/Azure may stay
placeholders: the gateway's radio then stays on its default channel, normally 1, and the
sensor's sweep finds it there). Rebuild and flash **both** boards.

- [ ] Sensor: `[link] linked on channel <n>, session ...`; gateway:
      `[link] session ... for sensor boot <b> on channel <n>`. Gateway `[health]` shows
      `sensor link=up`, `ok` increasing ~2 per second, `tag=0 malformed=0 replay=0`.
- [ ] No key, password, token or full connection string in either log.
- [ ] **Sensor reboot**: new session for boot b+1, no tag errors.
- [ ] **Gateway reboot**: sensor logs `session rejected by gateway (reason 1)` and relinks within
      ~1 s of the gateway's ESP-NOW start.
- [ ] **Power the gateway off**: within ~4 s (8 failed sends) the sensor logs a rescan, then
      `no gateway answered ... next sweep in` with growing delays (≤ 30 s). An impact during
      the outage is delivered after power returns (gateway `[alert] impact detected`).
- [ ] **Event retransmission**: with the gateway off right after an impact, the event reaches the
      gateway once (`dupEv` may grow, a second `[alert]` for the same id must not appear).
- [ ] **NVS erase recovery**: `pio run -e sensor_node -t erase`, re-flash the sensor. Expected:
      gateway `refused Hello from sensor boot 1 (highest granted b)`, sensor
      `gateway has granted boot b; advancing boot id 1 -> b+1`, then linked.
- [ ] **Replay (optional, needs a third ESP32 or an SDR)**: replay captured frames; gateway
      counts them under `replay`/`session` and accepts nothing; frames from another MAC count
      under `peer`.
- [ ] **Range**: walk the sensor away; record RSSI at which `radioFail` starts and the link drops.

## Stage 4: Wi-Fi and time (gateway, Azure still placeholders)

Add Wi-Fi credentials, rebuild the gateway (the sensor needs no rebuild for Wi-Fi changes).

- [ ] `[wifi] connected, IP ..., channel <n>` then `[time] SNTP started`,
      `[time] UTC clock synchronised`. The sensor relinks on channel `<n>`.
- [ ] Wrong Wi-Fi password: `[wifi] connect failed`, retries 2 s → 60 s with jitter; local
      alarms and the ESP-NOW link keep working.
- [ ] Block UDP 123 at the router: `time=syncing`, retries back off; no Azure attempt.
- [ ] Move the AP to another channel: `[wifi] AP channel changed`, sensor re-sweeps and relinks;
      record the outage.

## Stage 5: Azure IoT Hub (existing test hub only)

First, optionally, check the hub from the PC with the same roots and SAS scheme
([AZURE_SMOKE_TEST.md](AZURE_SMOKE_TEST.md)). Then add the hub host, gateway device id and key.

- [ ] Gateway: `[azure] TLS + MQTT 3.1.1 to <hub>:8883 as device <id>`, then
      `[azure] connected (MQTT CONNACK accepted); token refresh in ~3300 s`. (This also confirms
      that esp-mqtt copied the SAS password before the gateway wiped its buffer.)
- [ ] `az iot hub monitor-events -n <hub> -d <gateway-id> --properties all` shows a `summary`
      every 60 s with `$.ct application/json`, `$.ce utf-8`, `$.mid` and `messageType`;
      the JSON matches docs/TELEMETRY.md and has `"timeValid": true`.
- [ ] An impact / tilt / flame produces one `alert` each with the expected `eventId`.
- [ ] Wrong device key: one `CONNECT refused: not authorized`, next attempt ≥ 60 s later,
      doubling to 30 min. Fix the key: connects after the current backoff.
- [ ] Hub host typo: `transport/TLS error ...`, backoff 2 s → 120 s. No insecure fallback.

## Stage 6: recovery under load

- [ ] Remove the AP's internet uplink for 15 min while generating alarms: alerts queue (≤ 16),
      only the newest summary is kept (`summariesCoalesced` grows); after restoration at most
      one publish per second drains the backlog, alerts first, with unchanged `$.mid`.
- [ ] SAS renewal: build with `-D GW_SAS_LIFETIME_MIN=10 -D GW_SAS_REFRESH_MARGIN_S=120`.
      Every 8 min `disconnected: SAS token renewal` → `connected`, `sasRenewals` increments, no
      queued message lost, no duplicate except re-sends with identical `$.mid`.
- [ ] Hold the board's EN button / brown-out the gateway during traffic: it reboots, queued
      messages are lost (documented), the summary `$.mid` carries the new boot id.

## Stage 7: endurance and metering

- [ ] Default build, ≥ 3 h (well past the 60-min token lifetime): summaries every 60 s ± 1 s,
      `heap`/`min` in `[health]` stable, `hw` (outbox high-water) small.
- [ ] 24 h: compare the portal's daily message count with docs/TELEMETRY.md (6,270 free-tier /
      1,696 B1-S1 units in the simulated day) and with `health.estMeterUnits`.

## Measurements (pending)

| Quantity | Where to read it | Host-model value (not a measurement) | Board result |
|---|---|---|---|
| IMU update rate, min/max | sensor `[health]` `imu=ok <x>Hz`, summary `imu.rateHzMin` | 100 Hz configured | pending |
| IMU longest update / overruns | `maxUpd`, `overruns` | — | pending |
| IMU max gap during DHT reads | summary `imu.maxGapMs` | — | pending |
| DHT11 / DHT22 read time | sensor `maxRead`, — | ~25 ms / ~5 ms (driver estimate) | pending |
| Shortest reliably captured shock pulse | stage 1 pulse test | — | pending |
| Event queue high-water / drops / expiries | sensor `events pending/hw/dropped/expired` | 0 drops at 5 % loss (24 h harness) | pending |
| ESP-NOW receive-queue overflows / high-water | gateway `rxOvf`, `rxHw`; summary `health.rx.overflows` | n/a | pending |
| Outbox high-water | gateway `hw`; summary `health.queueHighWater` | ≤ 18 by construction | pending |
| Free / minimum heap after 3 h | `heap=`, `min=` | n/a | pending |
| Link drop RSSI | stage 3 range test | n/a | pending |
| Recovery time after gateway reboot | stage 3 | ~1 s in the harness model | pending |
| Daily metered messages | Azure portal | 6,270 free / 1,696 S1 (simulated day) | pending |
