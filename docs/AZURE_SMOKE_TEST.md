# Optional Azure smoke test from a PC

[`tools/azure_smoke.py`](../tools/azure_smoke.py) checks the gateway's cloud path from a PC
before (or instead of) flashing a board: TLS with the firmware's own root certificates and
hostname verification, SAS device authentication, QoS 1 JSON telemetry with the same
properties, the PUBACK, and reconnecting with a freshly generated token.

* **Separate from offline verification.** `tools/verify.py` and CI run only its offline
  `self-test` (fake broker on a local socket pair). Nothing contacts Azure unless you run one
  of the commands below yourself.
* **Existing resources only.** It never creates, changes or deletes Azure resources. Use a
  test hub and a test device identity, not production.
* **Credentials** come from environment variables or the git-ignored `config/secrets.h`. The
  device key, SAS tokens and signatures are redacted from all output.
* Standard library only (Python ≥ 3.8). No `pip install` needed.

## Status

| Check | Status |
|---|---|
| Offline self-test (`python tools/azure_smoke.py self-test`) | **passed** locally: CA bundle (2 roots, no Baltimore), TLS context requires certificate + hostname, MQTT 3.1.1 CONNECT/PUBLISH/PUBACK framing, SAS layout and signature input, redaction |
| TLS-only check against a public Azure IoT endpoint (`tls --host global.azure-devices-provisioning.net`) | **passed** on 2026-10-25: TLS 1.3, chain issued by `MSFT RS256 CA-1`, verified against only the firmware's two roots, hostname matched. This is the Device Provisioning Service, not an IoT Hub; no credentials were used. |
| `tls`, `publish`, `refresh` against a test IoT Hub | **pending**: no hub credentials were available |

## Procedure

1. Pick an existing test hub and device (symmetric key). If you need the key:
   `az iot hub device-identity show -n <hub> -d <device> --query authentication.symmetricKey.primaryKey -o tsv`
   (the command prints a secret: run it in a private terminal and do not paste the output into
   tickets or chat).
2. Provide the settings, either in `config/secrets.h` (`VC_IOTHUB_FQDN`, `VC_IOTHUB_DEVICE_ID`,
   `VC_IOTHUB_DEVICE_KEY`) or as environment variables:

   ```powershell
   # PowerShell 7+ (-MaskInput); in Windows PowerShell 5.1 set the variable from a file you delete afterwards
   $env:VC_IOTHUB_FQDN = "<hub>.azure-devices.net"
   $env:VC_IOTHUB_DEVICE_ID = "<device>"
   $env:VC_IOTHUB_DEVICE_KEY = Read-Host -MaskInput "device key"
   ```

   ```sh
   export VC_IOTHUB_FQDN=<hub>.azure-devices.net VC_IOTHUB_DEVICE_ID=<device>
   read -rs VC_IOTHUB_DEVICE_KEY && export VC_IOTHUB_DEVICE_KEY
   ```

3. In a second terminal, watch the hub:
   `az iot hub monitor-events -n <hub> -d <device> --properties all`
4. Run, in this order:

   ```sh
   python tools/azure_smoke.py tls --host <hub>.azure-devices.net   # TLS + hostname only
   python tools/azure_smoke.py publish                               # or: --secrets-h config/secrets.h
   python tools/azure_smoke.py refresh --lifetime 120                # token 1, wait ~90 s, token 2
   ```

5. Expected output (values vary):

   ```
   TLS ok: TLSv1.2|TLSv1.3, <cipher>; verified against 2 firmware root(s); hostname <hub> matches
   [publish] SAS token generated (expires hh:mm:ss UTC, value redacted)
   [publish] CONNACK: 0 (accepted)
   [publish] PUBACK for $.mid=<device>:smoke:<time>:1 after <n> ms
   ```

   and in `monitor-events`: a message with `"messageType": "smoke_test"` (it contains no sensor
   data), properties `contentType: application/json`, `contentEncoding: utf-8`, `messageId`
   and the application property `messageType`.

| Symptom | Meaning |
|---|---|
| `CONNACK: 5 (not authorized)` or `4` | wrong device id/key, device disabled, or PC clock far off |
| `SSLCertVerificationError` | the hub's chain does not lead to the firmware's roots (re-check Microsoft's TLS guidance before changing `azure_root_ca.h`), or a TLS-inspecting proxy |
| timeout / connection refused | outbound port 8883 blocked |

## What a pass does and does not show

A pass validates **this PC client path and the hub/device configuration**: the bundled root
certificates, hostname, SAS scheme, MQTT 3.1.1 login, property names and PUBACK behaviour. It
does **not** test the ESP32's TLS stack (mbedTLS), esp-mqtt, the Azure SDK for C build on the
board, the board's SNTP clock, heap use or Wi-Fi. ESP32-side Azure integration still requires
a board (HARDWARE_TESTS.md stage 5).
