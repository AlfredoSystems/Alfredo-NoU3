#!/usr/bin/env python3
"""
Auto-flasher for the Alfredo NoU3 (ESP32-S3).

Watches the USB serial ports. As soon as an ESP32-S3 in boot/download mode is
plugged in, it uploads the MotorParty example to it. Compile happens once at
startup; every board after that is just a fast flash.

Workflow (one board at a time):
    1. Run this script and leave it running.
    2. Plug in a board that is in boot mode  -> it flashes automatically.
    3. When it says "DONE", unplug the board.
    4. Plug in the next one. Repeat for as many boards as you like.

Requirements (all already present on this machine):
    - Arduino IDE 2.x (for the bundled arduino-cli) or arduino-cli on PATH
    - esp32 core 3.x installed (provides esptool + the alfredo-nou3 board)
    - Python 3 with pyserial  (pip install pyserial)

Usage:
    python flash_motorparty.py                 # normal run
    python flash_motorparty.py --recompile      # force a fresh compile
    python flash_motorparty.py --any-vid        # flash ANY new serial port,
                                                #   not just Espressif 0x303A
    python flash_motorparty.py --list           # list current ports and exit
"""

import argparse
import ctypes
import os
import shutil
import subprocess
import sys
import time
from datetime import datetime

import serial.tools.list_ports as list_ports

# --------------------------------------------------------------------------
# Configuration
# --------------------------------------------------------------------------

FQBN = "esp32:esp32:alfredo-nou3"

# Espressif's USB vendor ID. An ESP32-S3 in ROM download (boot) mode, or running
# TinyUSB, enumerates under this VID. Use --any-vid to ignore this filter.
ESPRESSIF_VID = 0x303A

# This script lives in <repo>/tools/ ; the sketch and libraries are found
# relative to it, so you can move the whole library folder anywhere.
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_DIR = os.path.dirname(SCRIPT_DIR)                       # .../Alfredo-NoU3
LIBRARIES_DIR = os.path.dirname(REPO_DIR)                    # .../libraries
SKETCH_DIR = os.path.join(REPO_DIR, "examples", "MotorParty")

# Precompiled binaries are cached here so every board after the first is a
# fast flash (no recompile).
BUILD_DIR = os.path.join(SCRIPT_DIR, ".build_motorparty")

# The sketch compiles WITHOUT FVT mode by default (a normal user upload). This
# flasher is the production QA tool, so it injects -DFVT_MODE at compile time to
# turn on Functional Verification Testing behavior. compiler.cpp.extra_flags is
# empty by default in the esp32 core, so this does not clobber the board's USB
# build flags.
FVT_BUILD_PROPERTY = "compiler.cpp.extra_flags=-DFVT_MODE"

POLL_SECONDS = 0.4


# --------------------------------------------------------------------------
# Helpers
# --------------------------------------------------------------------------

def log(msg, tag=""):
    stamp = datetime.now().strftime("%H:%M:%S")
    print(f"[{stamp}] {tag}{msg}", flush=True)


def find_arduino_cli():
    """Locate arduino-cli: PATH first, then the bundled Arduino IDE copy."""
    found = shutil.which("arduino-cli")
    if found:
        return found
    candidates = [
        r"C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe",
        os.path.expandvars(r"%LOCALAPPDATA%\Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe"),
    ]
    for c in candidates:
        if os.path.isfile(c):
            return c
    return None


def target_ports(any_vid):
    """Return the set of COM port device names that look like a target board."""
    ports = set()
    for p in list_ports.comports():
        if any_vid or (p.vid == ESPRESSIF_VID):
            ports.add(p.device)
    return ports


def describe_port(device):
    for p in list_ports.comports():
        if p.device == device:
            vid = f"{p.vid:04X}" if p.vid is not None else "----"
            pid = f"{p.pid:04X}" if p.pid is not None else "----"
            return f"{device} (VID:PID {vid}:{pid} - {p.description})"
    return device


def compile_sketch(cli, force):
    """Compile MotorParty once into BUILD_DIR. Returns True on success."""
    binary = os.path.join(BUILD_DIR, "MotorParty.ino.bin")
    if os.path.isfile(binary) and not force:
        log(f"Using cached build in {BUILD_DIR}", tag="OK  ")
        return True

    if force and os.path.isdir(BUILD_DIR):
        shutil.rmtree(BUILD_DIR, ignore_errors=True)

    log("Compiling MotorParty in FVT mode (one time)...", tag="... ")
    cmd = [
        cli, "compile",
        "-b", FQBN,
        "--libraries", LIBRARIES_DIR,
        "--build-property", FVT_BUILD_PROPERTY,
        "--output-dir", BUILD_DIR,
        SKETCH_DIR,
    ]
    result = subprocess.run(cmd)
    if result.returncode != 0:
        log("Compile FAILED. Fix the errors above and rerun.", tag="ERR ")
        return False
    log("Compile succeeded.", tag="OK  ")
    return True


def flash_port(cli, device, attempts=3):
    """Upload the precompiled binaries to one board. Returns True on success.

    A freshly enumerated USB serial port is often momentarily locked by Windows
    (or by another process still releasing it), which shows up as a "port is
    busy / Access is denied" error. That is usually transient, so retry a few
    times before giving up.
    """
    log(f"Flashing {describe_port(device)}", tag=">>> ")
    cmd = [
        cli, "upload",
        "-b", FQBN,
        "-p", device,
        "--input-dir", BUILD_DIR,
    ]
    start = time.time()
    for attempt in range(1, attempts + 1):
        # Bail out early if the board was unplugged mid-retry.
        if device not in target_ports_all():
            log("Board disappeared before it could be flashed.", tag="ERR ")
            return False
        result = subprocess.run(cmd)
        if result.returncode == 0:
            log(f"DONE in {time.time() - start:0.1f}s -- unplug this board.", tag="OK  ")
            return True
        if attempt < attempts:
            log(f"Attempt {attempt}/{attempts} failed, retrying in 1.5s "
                f"(if this keeps failing, close the Arduino IDE Serial Monitor)...", tag="... ")
            time.sleep(1.5)
    log(f"Upload FAILED after {attempts} attempts ({time.time() - start:0.1f}s).", tag="ERR ")
    return False


def target_ports_all():
    """All currently connected serial port device names (any VID)."""
    return {p.device for p in list_ports.comports()}


def is_admin():
    """pnputil device removal requires an elevated (admin) process."""
    try:
        return ctypes.windll.shell32.IsUserAnAdmin() != 0
    except Exception:
        return False


def remove_ghost_ports():
    """Remove disconnected (non-present) Espressif COM ports from Windows so the
    device list doesn't fill up with a phantom port per board.

    Only touches Ports-class devices that are BOTH non-present (unplugged) AND
    Espressif (VID_303A), so a currently connected board or your other serial
    hardware is never removed. Returns the number of ports removed, or -1 if the
    cleanup could not run.
    """
    ps = (
        "$removed = 0; "
        "Get-PnpDevice -Class Ports -ErrorAction SilentlyContinue | "
        "Where-Object { -not $_.Present -and $_.InstanceId -match 'VID_303A' } | "
        "ForEach-Object { "
        "  pnputil /remove-device $_.InstanceId | Out-Null; "
        "  if ($LASTEXITCODE -eq 0) { $removed++ } "
        "}; "
        "Write-Output $removed"
    )
    try:
        result = subprocess.run(
            ["powershell", "-NoProfile", "-ExecutionPolicy", "Bypass", "-Command", ps],
            capture_output=True, text=True, timeout=60,
        )
    except Exception as e:
        log(f"Ghost-port cleanup could not run: {e}", tag="ERR ")
        return -1
    try:
        return int(result.stdout.strip() or "0")
    except ValueError:
        return 0


# --------------------------------------------------------------------------
# Main watch loop
# --------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description="Auto-flash MotorParty to NoU3 boards in boot mode.")
    ap.add_argument("--recompile", action="store_true", help="force a fresh compile before watching")
    ap.add_argument("--any-vid", action="store_true", help="flash any new serial port, not just Espressif (0x303A)")
    ap.add_argument("--list", action="store_true", help="list current serial ports and exit")
    ap.add_argument("--no-cleanup", action="store_true",
                    help="do NOT remove ghost COM ports after a board is unplugged")
    ap.add_argument("--cleanup-now", action="store_true",
                    help="remove all disconnected Espressif ghost COM ports and exit")
    args = ap.parse_args()

    if args.list:
        for p in list_ports.comports():
            print(describe_port(p.device))
        return 0

    if args.cleanup_now:
        if not is_admin():
            log("Removing ports needs admin. Re-run this terminal as Administrator.", tag="ERR ")
            return 1
        n = remove_ghost_ports()
        log(f"Removed {n} ghost Espressif COM port(s).", tag="OK  ")
        return 0

    # Ghost-port cleanup runs after each unplug, but only if we're elevated.
    cleanup = not args.no_cleanup
    if cleanup and not is_admin():
        log("Not running as Administrator -- ghost COM ports will NOT be removed.", tag="WARN")
        log("To auto-clean the port list, re-run this terminal as Administrator.", tag="WARN")
        cleanup = False

    cli = find_arduino_cli()
    if not cli:
        log("Could not find arduino-cli. Install Arduino IDE 2.x or put arduino-cli on PATH.", tag="ERR ")
        return 1
    log(f"arduino-cli: {cli}")

    if not os.path.isdir(SKETCH_DIR):
        log(f"Sketch folder not found: {SKETCH_DIR}", tag="ERR ")
        return 1

    if not compile_sketch(cli, args.recompile):
        return 1

    filt = "any serial port" if args.any_vid else f"Espressif VID 0x{ESPRESSIF_VID:04X}"
    print("-" * 64)
    log(f"Watching for boards in boot mode ({filt}).")
    log("Plug one in to flash it. Ctrl+C to quit.")
    print("-" * 64)

    ok_count = 0
    fail_count = 0

    # Ports present at startup are ignored until unplugged (they may be running
    # boards, not ones you want to flash right now).
    known = target_ports(args.any_vid)
    if known:
        log(f"Ignoring {len(known)} port(s) already connected: {', '.join(sorted(known))}")

    try:
        while True:
            current = target_ports(args.any_vid)

            # Drop remembered ports that are gone, so their slot can re-arm.
            known &= current

            new_ports = current - known
            if new_ports:
                device = sorted(new_ports)[0]
                # Small settle delay so the OS finishes enumerating the device.
                time.sleep(0.3)
                if flash_port(cli, device):
                    ok_count += 1
                else:
                    fail_count += 1
                log(f"Totals -> flashed OK: {ok_count}   failed: {fail_count}")

                # Mark done and wait for physical unplug before arming again.
                # This avoids re-flashing the board when it re-enumerates after
                # reset (download-mode port -> running-app port).
                known.add(device)
                print("-" * 64)
                log("Waiting for you to unplug the board...", tag="... ")
                while device in target_ports(args.any_vid):
                    time.sleep(POLL_SECONDS)
                known.discard(device)
                log("Unplugged.")

                # The just-removed board is now a non-present ghost; purge it
                # (and any earlier ones) so the COM list stays clean.
                if cleanup:
                    time.sleep(0.5)  # let Windows mark the device non-present
                    n = remove_ghost_ports()
                    if n > 0:
                        log(f"Removed {n} ghost COM port(s) from the device list.", tag="OK  ")
                log("Ready for the next board.")
                print("-" * 64)

            time.sleep(POLL_SECONDS)
    except KeyboardInterrupt:
        print()
        log(f"Stopped. Flashed OK: {ok_count}   Failed: {fail_count}")
        return 0


if __name__ == "__main__":
    sys.exit(main())
