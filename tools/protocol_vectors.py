#!/usr/bin/env python3
"""Independent reference implementation of the local-link authentication.

Uses only Python's standard hmac/hashlib modules (not the firmware's C++ SHA-256)
to produce the known-answer vectors hard-coded in test/test_core/test_main.cpp:

  * RFC 5869 HKDF-SHA256 test case 1 (checks this script's HKDF itself)
  * the two per-direction link keys derived from a fixed test LMK
  * the complete on-air bytes and tag of a fixed Hello, HelloAck and Telemetry packet

Run:  python tools/protocol_vectors.py          (prints the vectors)
      python tools/protocol_vectors.py --check  (exit 1 if RFC 5869 fails)

The test LMK/MACs below are public test values, never real keys.
"""
import hashlib
import hmac
import struct
import sys


def hkdf_sha256(salt: bytes, ikm: bytes, info: bytes, length: int) -> bytes:
    prk = hmac.new(salt or b"\0" * 32, ikm, hashlib.sha256).digest()
    out, t, i = b"", b"", 1
    while len(out) < length:
        t = hmac.new(prk, t + info + bytes([i]), hashlib.sha256).digest()
        out += t
        i += 1
    return out[:length]


def rfc5869_case1_ok() -> bool:
    ikm = bytes.fromhex("0b" * 22)
    salt = bytes.fromhex("000102030405060708090a0b0c")
    info = bytes.fromhex("f0f1f2f3f4f5f6f7f8f9")
    okm = hkdf_sha256(salt, ikm, info, 42)
    return okm.hex() == (
        "3cb25f25faacd57a90434f64d0362f2a2d2d0a90cf1a5a4c5db02d56ecc4c5bf34007208d5b887185865"
    )


LMK = bytes.fromhex("1032547698badcfe0123456789abcdef")
SENSOR_MAC = bytes.fromhex("246f28000001")
GATEWAY_MAC = bytes.fromhex("246f28000002")
SALT = b"vehicle-iot/espnow/v2"
K_S2G = hkdf_sha256(SALT, LMK, b"auth sensor->gateway", 32)
K_G2S = hkdf_sha256(SALT, LMK, b"auth gateway->sensor", 32)


def header(msg_type: int, flags: int, boot: int, session: int, seq: int, uptime: int) -> bytes:
    return struct.pack("<BBBBIIII", 0x56, 2, msg_type, flags, boot, session, seq, uptime)


def tag(key: bytes, src: bytes, dst: bytes, data: bytes) -> bytes:
    return hmac.new(key, src + dst + data, hashlib.sha256).digest()[:16]


def s2g(data: bytes) -> bytes:
    return data + tag(K_S2G, SENSOR_MAC, GATEWAY_MAC, data)


def g2s(data: bytes) -> bytes:
    return data + tag(K_G2S, GATEWAY_MAC, SENSOR_MAC, data)


def hello() -> bytes:
    # nonce 0xA5A5F00D, boot nonce 0x0B0071D5
    return s2g(header(1, 0, 7, 0, 0, 123456) + struct.pack("<II", 0xA5A5F00D, 0x0B0071D5))


def hello_ack() -> bytes:
    return g2s(header(2, 0, 7, 0xDEADBEEF, 1, 99) + struct.pack("<I", 0xA5A5F00D))


def telemetry() -> bytes:
    # DHT #1 ok 24.56 C / 51.20 %, age 1200 ms; DHT #2 disabled (sentinels, zeros)
    dht1 = struct.pack("<BhHIH", 0, 2456, 5120, 1200, 0)
    dht2 = struct.pack("<BhHIH", 5, -32768, 0xFFFF, 0, 0)
    # IMU ok: ax 10 mg, ay -20 mg, az 1000 mg, |a| 1000, peak 1310, roll -325, pitch 1250 c-deg,
    # 99.7 Hz -> 997, max gap 14 ms, age 3 ms
    imu = struct.pack("<BhhhHHhhHHI", 0, 10, -20, 1000, 1000, 1310, -325, 1250, 997, 14, 3)
    flags = struct.pack("<BBB", 1, 0, 0x03)  # PIR active, tilt inactive, features PIR|shock
    counters = struct.pack("<IIIIIIH", 5, 2, 9, 0, 0, 0, 1)
    body = dht1 + dht2 + imu + flags + counters
    return s2g(header(3, 0, 7, 0xDEADBEEF, 42, 123456) + body)


def c_array(name: str, data: bytes) -> str:
    rows = [", ".join("0x%02x" % b for b in data[i : i + 12]) for i in range(0, len(data), 12)]
    return "static const uint8_t %s[%d] = {\n    %s};" % (name, len(data), ",\n    ".join(rows))


def main() -> int:
    ok = rfc5869_case1_ok()
    print("RFC 5869 test case 1:", "OK" if ok else "FAILED")
    if "--check" in sys.argv:
        return 0 if ok else 1
    print(c_array("kKatKeyS2G", K_S2G))
    print(c_array("kKatKeyG2S", K_G2S))
    print(c_array("kKatHello", hello()))
    print(c_array("kKatHelloAck", hello_ack()))
    print(c_array("kKatTelemetry", telemetry()))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
