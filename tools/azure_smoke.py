#!/usr/bin/env python3
"""Optional PC smoke test of the gateway's Azure IoT Hub client path.

NOT part of the default offline verification, never run by CI, and it never
creates or changes Azure resources: point it at an EXISTING test IoT Hub and an
EXISTING device identity. Standard library only (ssl, socket, hmac).

It exercises the same choices the gateway firmware makes:
  * TLS to <hub>:8883, verified against the firmware's own root bundle
    (firmware/gateway/azure_root_ca.h), hostname checked, TLS >= 1.2
  * MQTT 3.1.1, client id = device id,
    username "<hub>/<device>/?api-version=2020-09-30&DeviceClientType=...",
    password = SAS token (HMAC-SHA256 of "<url-encoded hub/devices/device>\\n<expiry>")
  * QoS 1 telemetry on devices/<id>/messages/events/ with $.ct=application/json,
    $.ce=utf-8, $.mid and messageType properties, waiting for the PUBACK
  * reconnect with a freshly generated token (credential refresh)

Commands
  self-test            offline checks against an in-process fake broker (no network)
  tls                  TLS handshake + certificate/hostname verification only (no credentials)
  publish              connect, publish one JSON smoke-test message, wait for PUBACK
  refresh              publish, then reconnect with a NEW token and publish again

Credentials (never printed; SAS tokens and keys are redacted from all output):
  environment  VC_IOTHUB_FQDN, VC_IOTHUB_DEVICE_ID, VC_IOTHUB_DEVICE_KEY
  or           --secrets-h config/secrets.h   (the git-ignored firmware config)

Examples (PowerShell: $env:VC_IOTHUB_FQDN="..." etc.)
  python tools/azure_smoke.py tls --host my-test-hub.azure-devices.net
  python tools/azure_smoke.py publish --secrets-h config/secrets.h
  python tools/azure_smoke.py refresh --lifetime 120
Then watch the hub:  az iot hub monitor-events -n <hub> -d <device> --properties all

A successful run validates THIS PC client path and the hub/device configuration
(roots, hostname, SAS scheme, MQTT properties). It does not test the ESP32's
TLS stack, esp-mqtt, the Azure SDK for C build or the board's clock/heap.
"""
import argparse
import base64
import hashlib
import hmac
import json
import os
import re
import socket
import ssl
import struct
import sys
import threading
import time
import urllib.parse

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CA_HEADER = os.path.join(ROOT, "firmware", "gateway", "azure_root_ca.h")
# The firmware's Azure SDK for C 1.1.8 sends exactly this api-version.
API_VERSION = "2020-09-30"
CLIENT_TYPE = "vc-azure-smoke%2F1.0"  # firmware: c%2F<sdk version>(ard;esp32)
PORT = 8883

_secrets = []  # values that must never appear in output


def redact(text):
    s = str(text)
    for v in _secrets:
        if v:
            s = s.replace(v, "<redacted>")
    s = re.sub(r"(sig=)[^&\s\"']+", r"\1<redacted>", s)
    s = re.sub(r"(SharedAccessKey=)[^;\s\"']+", r"\1<redacted>", s)
    return s


def say(*parts):
    print(redact(" ".join(str(p) for p in parts)), flush=True)


# ------------------------------------------------------------------ config ---

def load_credentials(args):
    vals = {
        "fqdn": os.environ.get("VC_IOTHUB_FQDN", ""),
        "device": os.environ.get("VC_IOTHUB_DEVICE_ID", ""),
        "key": os.environ.get("VC_IOTHUB_DEVICE_KEY", ""),
    }
    if args.secrets_h:
        with open(args.secrets_h, encoding="utf-8") as f:
            text = f.read()
        for name, field in (("VC_IOTHUB_FQDN", "fqdn"), ("VC_IOTHUB_DEVICE_ID", "device"), ("VC_IOTHUB_DEVICE_KEY", "key")):
            m = re.search(r'#define\s+%s\s+"([^"]*)"' % name, text)
            if m:
                vals[field] = m.group(1)
    _secrets.append(vals["key"])
    missing = [k for k, v in vals.items() if not v or v.startswith("YOUR_")]
    if missing:
        say("credentials missing or placeholders:", ", ".join(missing), "- nothing was sent.")
        sys.exit(2)
    try:
        if len(base64.b64decode(vals["key"], validate=True)) < 16:
            raise ValueError
    except ValueError:
        say("device key is not valid base64 of >= 16 bytes - nothing was sent.")
        sys.exit(2)
    if "/" in vals["fqdn"] or ":" in vals["fqdn"]:
        say("VC_IOTHUB_FQDN must be a bare host name (no scheme, path or port).")
        sys.exit(2)
    return vals


def firmware_ca_bundle():
    """The PEM roots compiled into the gateway firmware."""
    with open(CA_HEADER, encoding="utf-8") as f:
        text = f.read()
    body = text[text.index("kAzureRootCAs[] =") :]
    pem = "".join(re.findall(r'"((?:[^"\\]|\\.)*)"', body[: body.index(";")]))
    pem = pem.replace("\\n", "\n")
    certs = re.findall(r"-----BEGIN CERTIFICATE-----.*?-----END CERTIFICATE-----", pem, re.S)
    return pem, certs


def tls_context():
    pem, certs = firmware_ca_bundle()
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)  # verifies certificates and host names
    ctx.minimum_version = ssl.TLSVersion.TLSv1_2
    ctx.load_verify_locations(cadata=pem)  # ONLY the firmware's roots, not the OS store
    assert ctx.verify_mode == ssl.CERT_REQUIRED and ctx.check_hostname
    return ctx, len(certs)


def sas_token(fqdn, device, key_b64, lifetime_s, now=None):
    expiry = int(now if now is not None else time.time()) + int(lifetime_s)
    sr = urllib.parse.quote("%s/devices/%s" % (fqdn, device), safe="")
    to_sign = ("%s\n%d" % (sr, expiry)).encode()
    sig = base64.b64encode(hmac.new(base64.b64decode(key_b64), to_sign, hashlib.sha256).digest()).decode()
    token = "SharedAccessSignature sr=%s&sig=%s&se=%d" % (sr, urllib.parse.quote(sig, safe=""), expiry)
    _secrets.append(token)
    _secrets.append(sig)
    return token, expiry


# ------------------------------------------------------------ MQTT 3.1.1 ---

def _varint(n):
    out = bytearray()
    while True:
        b = n % 128
        n //= 128
        out.append(b | (0x80 if n else 0))
        if not n:
            return bytes(out)


def _str(s):
    b = s.encode("utf-8")
    return struct.pack("!H", len(b)) + b


CONNACK_CODES = {0: "accepted", 1: "unacceptable protocol version", 2: "identifier rejected",
                 3: "server unavailable", 4: "bad user name or password", 5: "not authorized"}


class Mqtt:
    def __init__(self, sock, timeout=20):
        self.sock = sock
        self.sock.settimeout(timeout)
        self.buf = b""

    def _recv_exact(self, n):
        while len(self.buf) < n:
            chunk = self.sock.recv(4096)
            if not chunk:
                raise ConnectionError("connection closed by broker")
            self.buf += chunk
        data, self.buf = self.buf[:n], self.buf[n:]
        return data

    def read_packet(self):
        head = self._recv_exact(1)[0]
        mult, length = 1, 0
        for _ in range(4):
            b = self._recv_exact(1)[0]
            length += (b & 0x7F) * mult
            mult *= 128
            if not b & 0x80:
                break
        return head >> 4, head & 0x0F, self._recv_exact(length)

    def send(self, ptype, flags, body):
        self.sock.sendall(bytes([(ptype << 4) | flags]) + _varint(len(body)) + body)

    def connect(self, client_id, username, password, keepalive=60):
        var = _str("MQTT") + bytes([4, 0x80 | 0x40 | 0x02]) + struct.pack("!H", keepalive)
        self.send(1, 0, var + _str(client_id) + _str(username) + _str(password))
        ptype, _, body = self.read_packet()
        if ptype != 2 or len(body) != 2:
            raise ConnectionError("expected CONNACK, got packet type %d" % ptype)
        return body[1]

    def publish_qos1(self, topic, payload, packet_id):
        self.send(3, 0x02, _str(topic) + struct.pack("!H", packet_id) + payload)
        while True:
            ptype, _, body = self.read_packet()
            if ptype == 4 and struct.unpack("!H", body[:2])[0] == packet_id:
                return True
            # Anything else (e.g. a PINGRESP) is ignored while waiting.

    def disconnect(self):
        try:
            self.send(14, 0, b"")
        except OSError:
            pass


def telemetry_topic(device, message_id, message_type):
    props = "$.ct=%s&$.ce=utf-8&$.mid=%s&messageType=%s" % (
        urllib.parse.quote("application/json", safe=""),
        urllib.parse.quote(message_id, safe=""),
        urllib.parse.quote(message_type, safe=""),
    )
    return "devices/%s/messages/events/%s" % (device, props)


def smoke_payload(device, seq):
    # Deliberately contains NO sensor readings: it must never be mistaken for real data.
    return json.dumps({
        "schemaVersion": 2, "messageType": "smoke_test", "gatewayId": device, "seq": seq,
        "ts": time.strftime("%Y-%m-%dT%H:%M:%S.000Z", time.gmtime()), "timeValid": True,
        "note": "PC smoke test from tools/azure_smoke.py; not vehicle data",
    }, separators=(",", ":")).encode()


# ---------------------------------------------------------------- commands ---

def open_tls(host, timeout=20):
    ctx, n = tls_context()
    raw = socket.create_connection((host, PORT), timeout=timeout)
    tls = ctx.wrap_socket(raw, server_hostname=host)
    cert = tls.getpeercert()
    subject = dict(x[0] for x in cert.get("subject", ()))
    issuer = dict(x[0] for x in cert.get("issuer", ()))
    say("TLS ok: %s, %s; verified against %d firmware root(s); hostname %s matches" % (
        tls.version(), tls.cipher()[0], n, host))
    say("  server certificate subject CN=%s, issuer CN=%s, expires %s" % (
        subject.get("commonName"), issuer.get("commonName"), cert.get("notAfter")))
    return tls


def cmd_tls(args):
    host = args.host or os.environ.get("VC_IOTHUB_FQDN", "")
    if not host:
        say("give --host <hub>.azure-devices.net (no credentials are needed for this check)")
        return 2
    open_tls(host).close()
    return 0


def session(creds, lifetime, seq, label):
    token, expiry = sas_token(creds["fqdn"], creds["device"], creds["key"], lifetime)
    say("[%s] SAS token generated (expires %s UTC, value redacted)" % (
        label, time.strftime("%H:%M:%S", time.gmtime(expiry))))
    tls = open_tls(creds["fqdn"])
    m = Mqtt(tls)
    username = "%s/%s/?api-version=%s&DeviceClientType=%s" % (creds["fqdn"], creds["device"], API_VERSION, CLIENT_TYPE)
    rc = m.connect(creds["device"], username, token)
    say("[%s] CONNACK: %d (%s)" % (label, rc, CONNACK_CODES.get(rc, "unknown")))
    if rc != 0:
        tls.close()
        return False
    mid = "%s:smoke:%d:%d" % (creds["device"], int(time.time()), seq)
    t0 = time.time()
    m.publish_qos1(telemetry_topic(creds["device"], mid, "smoke_test"), smoke_payload(creds["device"], seq), seq)
    say("[%s] PUBACK for $.mid=%s after %.0f ms" % (label, mid, (time.time() - t0) * 1000))
    m.disconnect()
    tls.close()
    return True


def cmd_publish(args):
    creds = load_credentials(args)
    say("hub %s, device %s" % (creds["fqdn"], creds["device"]))
    return 0 if session(creds, args.lifetime, 1, "publish") else 1


def cmd_refresh(args):
    creds = load_credentials(args)
    say("hub %s, device %s; token lifetime %d s" % (creds["fqdn"], creds["device"], args.lifetime))
    if not session(creds, args.lifetime, 1, "token 1"):
        return 1
    wait = max(0, args.lifetime - args.margin)
    say("waiting %d s (lifetime - margin), then reconnecting with a NEW token ..." % wait)
    time.sleep(wait)
    return 0 if session(creds, args.lifetime, 2, "token 2") else 1


# --------------------------------------------------------------- self-test ---

def _fake_broker(sock, connack_rc, seen):
    m = Mqtt(sock, timeout=5)
    ptype, _, body = m.read_packet()
    seen["connect_type"] = ptype
    proto_len = struct.unpack("!H", body[:2])[0]
    seen["proto"] = body[2 : 2 + proto_len].decode()
    seen["level"] = body[2 + proto_len]
    seen["flags"] = body[3 + proto_len]
    pos = 6 + proto_len
    fields = []
    for _ in range(3):
        n = struct.unpack("!H", body[pos : pos + 2])[0]
        fields.append(body[pos + 2 : pos + 2 + n].decode())
        pos += 2 + n
    seen["client_id"], seen["username"], seen["password"] = fields
    m.send(2, 0, bytes([0, connack_rc]))
    if connack_rc:
        return
    ptype, flags, body = m.read_packet()
    tlen = struct.unpack("!H", body[:2])[0]
    seen["topic"] = body[2 : 2 + tlen].decode()
    seen["qos"] = (flags >> 1) & 3
    pid = struct.unpack("!H", body[2 + tlen : 4 + tlen])[0]
    seen["payload"] = body[4 + tlen :]
    m.send(13, 0, b"")  # an unrelated PINGRESP first: the client must keep waiting
    m.send(4, 0, struct.pack("!H", pid))
    ptype, _, _ = m.read_packet()
    seen["last"] = ptype


def cmd_self_test(_args):
    failures = []

    def check(cond, what):
        print(("  ok   " if cond else "  FAIL ") + what)
        if not cond:
            failures.append(what)

    print("azure_smoke.py offline self-test (no network)")
    # 1) The firmware's CA bundle parses and loads as the only trust anchors.
    pem, certs = firmware_ca_bundle()
    check(len(certs) == 2, "firmware root bundle has 2 certificates")
    check(all(len(ssl.PEM_cert_to_DER_cert(c)) > 500 for c in certs), "each root parses as DER")
    ctx, _ = tls_context()
    check(ctx.verify_mode == ssl.CERT_REQUIRED and ctx.check_hostname, "TLS context verifies certificate + hostname")
    check("Baltimore" not in pem, "expired Baltimore CyberTrust root is not trusted")
    # 2) MQTT remaining-length encoding at the boundaries.
    check([_varint(n) for n in (0, 127, 128, 16383, 16384)] ==
          [b"\x00", b"\x7f", b"\x80\x01", b"\xff\x7f", b"\x80\x80\x01"], "remaining-length encoding")
    # 3) SAS token structure and signature input.
    key = base64.b64encode(bytes(range(32))).decode()
    _secrets.append(key)
    tok, exp = sas_token("test-hub.azure-devices.net", "gw-01", key, 3600, now=1761393600)
    sr = "test-hub.azure-devices.net%2Fdevices%2Fgw-01"
    expect_sig = base64.b64encode(hmac.new(bytes(range(32)), ("%s\n%d" % (sr, exp)).encode(), hashlib.sha256).digest())
    check(tok.startswith("SharedAccessSignature sr=%s&sig=" % sr) and tok.endswith("&se=%d" % exp), "SAS token layout")
    check(urllib.parse.quote(expect_sig.decode(), safe="") in tok, "SAS signature over '<sr>\\n<expiry>'")
    check(redact("password " + tok) == "password <redacted>" and key not in redact("k=" + key),
          "generated tokens and keys are redacted from output")
    check(redact("...&sig=abc%2Bdef&se=1") == "...&sig=<redacted>&se=1" and
          redact("HostName=h;SharedAccessKey=xyz=") == "HostName=h;SharedAccessKey=<redacted>",
          "unknown signatures / connection-string keys are redacted by pattern")
    # 4) CONNECT / PUBLISH / PUBACK against a fake broker on a local socket pair.
    for rc in (0, 5):
        a, b = socket.socketpair()
        seen = {}
        t = threading.Thread(target=_fake_broker, args=(b, rc, seen))
        t.start()
        m = Mqtt(a, timeout=5)
        got = m.connect("gw-01", "test-hub.azure-devices.net/gw-01/?api-version=%s" % API_VERSION, tok)
        if rc == 0:
            ok = m.publish_qos1(telemetry_topic("gw-01", "gw-01:1:s7", "summary"), b'{"a":1}', 7)
            m.disconnect()
        t.join(5)
        a.close()
        b.close()
        if rc == 0:
            check(got == 0 and ok, "CONNACK accepted, PUBACK matched by packet id (PINGRESP skipped)")
            check(seen.get("proto") == "MQTT" and seen.get("level") == 4, "MQTT 3.1.1 CONNECT")
            check(seen.get("flags") == 0xC2, "CONNECT flags: username + password + clean session")
            check(seen.get("password") == tok and seen.get("client_id") == "gw-01", "client id and SAS password sent")
            check(seen.get("qos") == 1, "telemetry published with QoS 1")
            check(seen.get("topic") == "devices/gw-01/messages/events/$.ct=application%2Fjson&$.ce=utf-8"
                  "&$.mid=gw-01%3A1%3As7&messageType=summary", "topic carries content type/encoding, $.mid, messageType")
            check(seen.get("last") == 14, "DISCONNECT sent")
        else:
            check(got == 5, "a refused CONNECT is reported with its return code (5 not authorized)")
    payload = json.loads(smoke_payload("gw-01", 1))
    check(payload["messageType"] == "smoke_test" and "sensorNode" not in payload, "smoke payload carries no sensor data")
    print("self-test %s" % ("PASSED" if not failures else "FAILED (%d)" % len(failures)))
    return 0 if not failures else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("self-test")
    p = sub.add_parser("tls")
    p.add_argument("--host", help="IoT Hub host name, e.g. my-hub.azure-devices.net")
    for name in ("publish", "refresh"):
        p = sub.add_parser(name)
        p.add_argument("--secrets-h", help="read VC_IOTHUB_* from this (git-ignored) header")
        p.add_argument("--lifetime", type=int, default=3600 if name == "publish" else 120, help="SAS lifetime, s")
        p.add_argument("--margin", type=int, default=30, help="refresh this long before expiry (refresh only)")
    args = ap.parse_args()
    try:
        return {"self-test": cmd_self_test, "tls": cmd_tls, "publish": cmd_publish, "refresh": cmd_refresh}[args.cmd](args)
    except (OSError, ssl.SSLError, ConnectionError) as e:
        say("FAILED: %s: %s" % (type(e).__name__, e))
        return 1


if __name__ == "__main__":
    sys.exit(main())
