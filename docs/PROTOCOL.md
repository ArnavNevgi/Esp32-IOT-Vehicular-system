# Local link protocol (sensor node ⇄ gateway), version 2

Transport: **ESP-NOW unicast with peer encryption** (CCMP, per-peer LMK, non-default PMK)
**plus an application-layer authentication tag** on every packet. There is no broadcast and no
plaintext fallback. If the keys or MACs are not configured, the link does not start.

Source of truth: [`vc_protocol.h/.cpp`](../lib/vehicle_core/src/vc_protocol.h) (framing, tag),
[`vc_session.h`](../lib/vehicle_core/src/vc_session.h) (sessions, replay window),
[`vc_sensor_link.cpp`](../lib/vehicle_core/src/vc_sensor_link.cpp) and
[`vc_gateway_link.cpp`](../lib/vehicle_core/src/vc_gateway_link.cpp) (the two endpoints). The
firmware adapters in `firmware/*/link_client.cpp` / `sensor_link.cpp` only move frames between
ESP-NOW callbacks and these classes.

Version 1 (single HMAC key, no address binding, 4-byte Hello) is rejected as
`unsupported_version`. Both boards must run the same firmware generation.

## 1. Authentication

### Why a second layer

ESP-NOW's CCMP keeps payloads confidential on the air. Espressif's documentation does not say
that the driver discards an *unencrypted* frame that spoofs an encrypted peer's MAC address
(an unpaired receiver accepts unencrypted unicast). Authenticity therefore does not rely on the
radio layer.

### Algorithm (exact)

Standard primitives only: SHA-256 (FIPS 180-4), HMAC-SHA256 (RFC 2104), HKDF-SHA256 (RFC 5869).

```
LMK         16-byte shared secret from config (VC_ESPNOW_LMK_HEX); also the ESP-NOW peer LMK
K_s2g       = HKDF-SHA256(salt = "vehicle-iot/espnow/v2", IKM = LMK, info = "auth sensor->gateway", L = 32)
K_g2s       = HKDF-SHA256(salt = "vehicle-iot/espnow/v2", IKM = LMK, info = "auth gateway->sensor", L = 32)

tag (16 B)  = first 16 bytes of HMAC-SHA256(K_dir, srcMAC(6) | dstMAC(6) | header(20) | body)
packet      = header | body | tag
```

* **Identity:** `srcMAC` and `dstMAC` are the *configured* station MACs of the two boards
  (not taken from the frame). A frame replayed from or to any other address fails the tag. The
  gateway additionally drops frames whose radio source MAC is not the configured sensor
  (`unknownPeer`).
* **Direction / message type:** every type is valid in exactly one direction (Hello, Telemetry,
  Event: sensor→gateway; HelloAck, EventAck, SessionReject: gateway→sensor). `decode()` rejects
  a type that does not match the receiver's direction (`wrong_direction`) before computing the
  tag, and each direction has its own key, so a reflected frame can never verify.
* **Session, sequence, payload:** the header (type, flags, bootId, sessionId, seq, uptimeMs)
  and the whole body are inside the MAC.
* **Key separation:** the tag keys are derived with HKDF and distinct `info` labels; neither is
  ever used as a radio key, and the PMK is not used for anything but ESP-NOW. (The LMK itself
  is both the CCMP peer key and the HKDF input. HKDF output is computationally independent of
  its input, which is the standard way to derive several keys from one secret.)
* **Comparison:** `vc::constantTimeEqual` ORs the XOR of every byte (no early exit, no
  data-dependent branch).
* **Order of checks:** length → magic → version → type → direction → exact length → **tag** →
  field validation. Nothing in a packet is interpreted, and no session or replay state changes,
  before the tag verified.

### Evidence

| Check | Where |
|---|---|
| SHA-256 FIPS 180-2 examples, HMAC RFC 4231 cases 2 and 6, HKDF RFC 5869 cases 1 and 3 | `test_sha256_and_hmac_vectors`, `test_hkdf_rfc5869_vectors` |
| Link keys and complete Hello / HelloAck / Telemetry packets equal those produced by an **independent** implementation (Python `hashlib`/`hmac`, [`tools/protocol_vectors.py`](../tools/protocol_vectors.py)); decoding them and re-encoding gives the same bytes | `test_link_keys_and_packets_match_reference` |
| Reflection, other sender/receiver MAC, wrong key, every single-bit flip, wrong direction | `test_tag_binds_direction_and_addresses`, `test_protocol_rejects_forged_or_tampered_packets` |
| Tampered / foreign-key / junk / foreign-MAC frames leave the session id and replay window untouched (production gateway code) | integration `test_malformed_unauthenticated_and_replayed_frames` |
| 200,000 seeded malformed inputs (seed `0x5EED2026`), half re-signed so the field parser is reached; every accepted input re-encodes byte-identically | `test_decoder_fuzz_corpus` |

## 2. Packet layout (little-endian)

```
| header 20 B | body (fixed per type) | tag 16 B |      total <= 250 B (ESP-NOW v1 limit)
```

| Offset | Size | Field | Notes |
|---|---|---|---|
| 0 | 1 | magic | `0x56` |
| 1 | 1 | version | `2` |
| 2 | 1 | type | see below |
| 3 | 1 | flags | bit0 = synthetic readings; other bits must be 0 |
| 4 | 4 | bootId | sensor boot counter (≠ 0). Gateway packets echo the sensor boot they address. |
| 8 | 4 | sessionId | gateway-issued; 0 in Hello and in a SessionReject that answers a Hello |
| 12 | 4 | seq | per session and direction, starts at 1 (Hello: 0) |
| 16 | 4 | uptimeMs | sender `millis()` at transmission |

| Type | Dir | Body | Total | Body fields |
|---|---|---|---|---|
| 1 Hello | S→G | 8 | 44 | nonce u32 (≠0, fresh per Hello), bootNonce u32 (≠0, random per sensor boot) |
| 2 HelloAck | G→S | 4 | 40 | echoed nonce; header carries the new sessionId |
| 3 Telemetry | S→G | 74 | 110 | 2× DHT (status, temp c°C i16, RH c% u16, ageMs u32, errors u16); IMU (status, ax/ay/az mg i16, \|a\| mg u16, peak mg u16, roll/pitch c° i16, rate d-Hz u16, maxGap ms u16, ageMs u32); PIR, tilt (u8 0/1); features u8 (bit0 PIR fitted, bit1 shock fitted); cumulative counters (PIR rises, impacts, shock edges, impact overflows, events dropped, events expired u32; link restarts u16) |
| 4 Event | S→G | 17 | 53 | eventCounter u32 (≠0), type u8 (1 impact, 2 tilt), state u8 (0 detected, 1 active, 2 cleared), attempt u8 (≥1), detectedUptimeMs u32, angle c° i16, peak mg u16, pulses u16 |
| 5 EventAck | G→S | 5 | 41 | eventCounter u32, result u8 (0 accepted, 1 duplicate) |
| 6 SessionReject | G→S | 5 | 41 | reason u8 (1 unknown session, 2 stale boot, 3 replay), value u32 (rejected seq; for a refused Hello: the highest boot the gateway has granted) |

Sensor status: 0 ok, 1 stale, 2 error, 3 absent (enabled but not responding), 4 calibrating
(IMU), 5 **disabled** (not fitted, switched off in the build). Readings without a value use
sentinels (`INT16_MIN` temperature, `0xFFFF` humidity) or zeros. `decode()` accepts only the
canonical encoding: status/sentinel mismatches, non-zero values behind a value-less status,
non-zero rate/age for a disabled sensor, an active PIR that is not fitted, tilt reported by a
disabled IMU, unknown flag or feature bits, unknown enum values, out-of-range values
(−40..85 °C, 0..100 %, |a| ≤ 16 g per axis, ±180°), a wrong length and trailing bytes are all
rejected.

## 3. Sessions, replay and reboots

### Handshake

1. The sensor sends **Hello** (fresh random `nonce`, its `bootId` and per-boot `bootNonce`) on
   each channel in turn (§5).
2. The gateway grants a session unless the Hello is **stale** (below). It answers with
   **HelloAck** echoing the nonce and carrying a fresh hardware-random 32-bit `sessionId`, and
   resets the 64-entry replay window.
3. The sensor accepts a HelloAck only while scanning, only with its outstanding nonce, and only
   addressed to its current boot.
4. Telemetry / Event frames must carry the current `(bootId, sessionId)` and a `seq` that passes
   the 64-entry sliding window (exact duplicates and frames ≥ 64 behind are dropped and counted
   as `replay`). Frames for an unknown session get a rate-limited (500 ms) **SessionReject**
   that echoes their session id; the sensor re-handshakes only if that id is its *current*
   session, at most once per 2 s.

### Stale-boot rule and recovery

The gateway persists the highest sensor boot it has granted and that boot's `bootNonce`.
A Hello is refused if `bootId < highest`, or `bootId == highest` with a different
`bootNonce` (the same id reused by a different boot, e.g. after the sensor's NVS was erased).
A refused Hello is answered with an authenticated `SessionReject(StaleBoot, value = highest)`
addressed to that boot; the sensor then persists `highest + 1` and says Hello again. A sensor
boot id is therefore never reused, so event ids (`sensor-node-01:<boot>:<counter>`) never
collide. (Integration test: `test_sensor_nvs_erased_recovers_without_reusing_ids`.)

### What each situation does

| Situation | Result | Tested in |
|---|---|---|
| Sensor-only reboot | boot counter +1 → new session; packets of the old boot get `StaleBoot` | `test_sensor_and_gateway_reboots` |
| Gateway-only reboot | no session in RAM: the sensor's next frame gets `SessionReject(UnknownSession)` echoing its session → re-handshake within ~1 s | same |
| Simultaneous reboot | sensor boot > persisted → new session as usual | same |
| Delayed packets from an earlier session or boot | rejected (`WrongSession` / `StaleBoot`); the resulting reject names the old session/boot and the sensor ignores it | `test_delayed_and_replayed_session_establishment` |
| Replayed Hello of an older boot | refused; current session untouched; the hint is addressed to the old boot and ignored | same |
| Replayed Hello of the **current** boot | the gateway starts a new session (it cannot tell); the real sensor's next frame is rejected and it re-handshakes. **Availability only**: no data is accepted under the replayed session (the attacker cannot produce a valid tag) | same |
| Replayed SessionReject | ignored unless it names the current session; at most one re-handshake per 2 s | rejects naming an old session: `test_delayed_and_replayed_session_establishment`; a replay *within* the current session is not separately tested |
| Replayed EventAck | acknowledges an event the gateway already accepted (harmless) | not separately tested |
| Invalid tag, wrong key, foreign MAC, junk | counted, no state change | `test_malformed_unauthenticated_and_replayed_frames` |
| Event retransmitted after a lost EventAck (same or new session) | processed once, acknowledged again (`duplicate`) via a 32-entry recent-event cache | `test_loss_duplication_reordering_events_exactly_once` |
| Event retransmitted after a **gateway** reboot (cache lost) | may be forwarded again; the cloud copy has the same `eventId`/`$.mid` (at-least-once) | — |

### Persistent state

| Board | NVS key | Written | Write frequency |
|---|---|---|---|
| Sensor | `vc_sensor/boot` | at every boot, before the link starts; and when the gateway's stale-boot hint moves it forward | once per boot |
| Gateway | `vc_gateway/sensorBoot`, `vc_gateway/sensorNonce` | when it grants a session to a new sensor boot (not on same-boot re-handshakes) | once per sensor boot |
| Gateway | `vc_gw_boot/boot` | at every gateway boot (only for message ids) | once per boot |

A few writes per boot are far below NVS flash endurance. NVS replaces a key atomically, so a
power loss during a write leaves the old or the new value:

* The sensor uses a new boot id only after `store()` returned success and a read-back matched
  (`advanceBootCounter`). An interrupted write can at worst skip an id, never reuse one. If NVS
  is unavailable, or the counter would wrap past `0xFFFFFFFF`, the sensor **does not start the
  link** (reported at boot) instead of guessing an id.
* On the gateway, an interrupted `(boot, nonce)` update leaves a pair that does not match the
  next Hello; the sensor resolves it by advancing its boot id (above). A failed gateway write is
  counted (`persistFailures`); the session is still granted. Consequence: after a later gateway
  reboot, a replayed older Hello could again start a session — availability only, as above.
* Erased gateway NVS: all boots are accepted again (still never reusing the sensor's own ids).
  Erasing **both** boards restarts sensor boots at 1, so event ids repeat ids from before the
  erase; downstream de-duplication should also key on `gatewayBootId` / time if that matters.

### Sequence exhaustion and wraparound

`seq` is 32-bit per session and direction. The sensor starts a new session when its counter
reaches `0xFFFFFF00` (at 2 Hz that is ~68 years), so the replay window never sees a wrap. Boot
ids are 32-bit and never wrap (see above). All timers compare `millis()` differences and work
across its 49.7-day wrap (`test_millis_wrap_on_both_devices`).

## 4. Events and acknowledgements

The sensor keeps each impact/tilt event in an 8-entry retransmit queue and sends it every
300 ms, keeping the same `eventCounter` on every attempt and across re-handshakes, until an
**EventAck** arrives: up to 40 attempts while the link is up (attempts are not consumed while
it is down). If the queue is full, the oldest event is dropped and counted. The gateway
acknowledges an event only **after** it has been queued for the cloud, and acknowledges
duplicates again without reprocessing them. An EventAck means "the gateway accepted it", not
"Azure received it". A radio-level send success proves neither.

## 5. Channel coordination

The gateway's channel is set by its upstream access point; the gateway never changes it. The
sensor is not associated with any AP. It sweeps channels 1–13, dwelling 60 ms per channel for a
HelloAck, and stays on the channel that answered. If the AP moves the gateway to another channel,
the sensor's sends stop being ACKed at the radio level; after 8 consecutive failures it sweeps
again, starting from the last good channel. A sweep that finds nothing backs off from 1 s up to
30 s, with jitter (`test_channel_change_resweep`, `test_link_loss_alarm_and_recovery_...`).
While the gateway itself is reconnecting to Wi-Fi its radio scans channels, so the local link can
drop for a few seconds and recovers the same way. ESP-NOW peers use channel 0 ("current
channel") on both sides, and the gateway disables Wi-Fi modem sleep so it does not miss frames.

The local link needs no router, but the gateway still needs an internet uplink to reach Azure.

## 6. Residual risks

* A replayed current-boot Hello or a jammer can interrupt the link (availability).
* Both boards share one LMK. Anyone who extracts it from either board's flash can impersonate
  either side; flash encryption and secure boot are not enabled.
* Only one sensor node per gateway is implemented.
