#!/usr/bin/env python3
"""Complete offline verification of this repository in one command.

    python tools/verify.py              firmware builds + host unit/integration tests
    python tools/verify.py --sanitize   ... plus the tests under ASan/UBSan
                                        (natively on Linux/macOS, else in Docker)
    python tools/verify.py --clean      delete previous build outputs first
    python tools/verify.py --skip-firmware   host checks only (fast)

Every firmware build uses the PLACEHOLDER configuration
(-D VC_FORCE_PLACEHOLDER_SECRETS=1) even if config/secrets.h exists, so the
result never depends on, and the binaries never contain, real credentials.
Nothing here talks to Azure, flashes a board or needs hardware.
"""
import argparse
import os
import platform
import shutil
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FIRMWARE_ENVS = ["sensor_node", "sensor_node_synthetic", "gateway"]
PIO_VERSION = "6.1.19"
DOCKER_IMAGE = "python:3.12-slim"

results = []


def run(name, cmd, env=None, cwd=ROOT):
    print("\n=== %s\n$ %s" % (name, " ".join(cmd)), flush=True)
    t0 = time.time()
    rc = subprocess.call(cmd, cwd=cwd, env=env)
    results.append((name, rc == 0, time.time() - t0))
    return rc == 0


def check_secret_hygiene():
    print("\n=== secret hygiene", flush=True)
    ok = True
    tracked = subprocess.run(["git", "ls-files", "config/secrets.h"], cwd=ROOT, capture_output=True, text=True)
    if tracked.stdout.strip():
        print("FAIL: config/secrets.h is tracked by git")
        ok = False
    ignored = subprocess.run(["git", "check-ignore", "-q", "config/secrets.h"], cwd=ROOT)
    if ignored.returncode != 0:
        print("FAIL: config/secrets.h is not git-ignored")
        ok = False
    if os.path.exists(os.path.join(ROOT, "config", "secrets.h")):
        print("note: config/secrets.h exists locally; builds below still use the placeholders")
    if ok:
        print("ok: config/secrets.h is ignored and untracked")
    results.append(("secret hygiene", ok, 0.0))
    return ok


def pio():
    exe = shutil.which("pio") or shutil.which("platformio")
    return [exe] if exe else [sys.executable, "-m", "platformio"]


def sanitize_cmd():
    if platform.system() in ("Linux", "Darwin"):
        return pio() + ["test", "-e", "native_sanitize"]
    docker = shutil.which("docker")
    if not docker:
        return None
    # Copy the tree (without build outputs or secrets) into the container so
    # nothing is written back to the host checkout.
    script = (
        "set -e; apt-get update -qq && apt-get install -y -qq --no-install-recommends g++ git >/dev/null; "
        "pip install -q platformio==%s; mkdir -p /work/p; "
        "tar -C /src --exclude=./.pio --exclude=./config/secrets.h -cf - . | tar -C /work/p -xf -; "
        "cd /work/p && pio test -e native_sanitize" % PIO_VERSION
    )
    return [docker, "run", "--rm", "-v", "%s:/src:ro" % ROOT, DOCKER_IMAGE, "bash", "-c", script]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sanitize", action="store_true", help="also run the tests with ASan + UBSan")
    ap.add_argument("--clean", action="store_true", help="remove previous build outputs first")
    ap.add_argument("--skip-firmware", action="store_true", help="skip the ESP32 firmware builds")
    args = ap.parse_args()

    if args.clean:
        build = os.path.join(ROOT, ".pio", "build")
        print("removing", build)
        shutil.rmtree(build, ignore_errors=True)

    env = dict(os.environ)
    env["PLATFORMIO_BUILD_FLAGS"] = "-D VC_FORCE_PLACEHOLDER_SECRETS=1"

    check_secret_hygiene()
    run("protocol reference vectors (RFC 5869 self-check)", [sys.executable, "-I", "tools/protocol_vectors.py", "--check"])
    run("azure_smoke.py offline self-test", [sys.executable, "-I", "tools/azure_smoke.py", "self-test"])
    if not args.skip_firmware:
        cmd = pio() + ["run"]
        for e in FIRMWARE_ENVS:
            cmd += ["-e", e]
        run("firmware builds (placeholder config)", cmd, env=env)
    run("host unit tests + integration harness", pio() + ["test", "-e", "native"], env=env)
    if args.sanitize:
        cmd = sanitize_cmd()
        if cmd is None:
            print("\n=== sanitizer run: SKIPPED (needs Linux/macOS or Docker)")
            results.append(("host tests under ASan/UBSan", False, 0.0))
        else:
            run("host tests under ASan/UBSan", cmd)

    print("\n=== summary")
    for name, ok, secs in results:
        print("  %-52s %s  (%.0f s)" % (name, "PASS" if ok else "FAIL", secs))
    failed = [r for r in results if not r[1]]
    print("\n%s" % ("ALL CHECKS PASSED" if not failed else "%d CHECK(S) FAILED" % len(failed)))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
