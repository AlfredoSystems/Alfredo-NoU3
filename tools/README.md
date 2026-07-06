# NoU3 Production Flasher

`flash_motorparty.py` auto-flashes the **MotorParty** example onto Alfredo NoU3
(ESP32-S3) boards for QA/FVT testing. It watches your USB serial ports and, the
moment a board in boot mode is plugged in, uploads the sketch to it. Compile
happens once at startup; every board after that is just a fast flash.

It is built for flashing **hundreds of boards, one at a time**:
plug in → it flashes → unplug → plug in the next → repeat.

## What makes it different from a normal upload

- **FVT mode is on.** The flasher injects `-DFVT_MODE` at compile time, so the
  board runs in Functional Verification Testing mode. A normal upload of
  MotorParty (from the Arduino IDE) is *not* in FVT mode. Nothing in
  `MotorParty.ino` is hardcoded — the mode is decided by *how* it's flashed.
- **Ghost COM ports are cleaned up.** Each ESP32-S3 has a unique USB serial, so
  Windows assigns a new COMx per board and never reclaims it. After you unplug a
  board, the flasher removes that dead port so your device list stays clean.

## Requirements

Everything below is already installed on the QA machine:

- Arduino IDE 2.x (provides the bundled `arduino-cli`) — or `arduino-cli` on PATH
- esp32 Arduino core 3.x (provides esptool + the `alfredo-nou3` board)
- Python 3 with `pyserial` (`pip install pyserial`)
- The `Alfredo-NoU3` library and its dependencies in your Arduino `libraries`
  folder (this repo's normal location)

## Running it

**Run from an Administrator terminal** so ghost-port cleanup can work:

1. Right-click Windows Terminal / PowerShell → **Run as administrator**
2. ```
   cd "C:\Users\Jacob\Documents\Arduino\libraries\Alfredo-NoU3\tools"
   python flash_motorparty.py
   ```

Or double-click **`flash_motorparty.bat`** (won't be elevated, so port cleanup
is skipped — flashing still works). To elevate it, right-click the .bat →
Run as administrator.

### Normal session

1. First run compiles MotorParty in FVT mode once (~30–60 s).
2. It prints `Watching for boards in boot mode...`.
3. Put a board in **boot mode** (hold BOOT, tap RESET, release BOOT) and plug it
   in → it flashes automatically.
4. It prints `OK  DONE ... -- unplug this board.` → unplug it.
5. It removes that board's ghost COM port and re-arms for the next one.
6. `Ctrl+C` to quit; it prints the final `flashed OK / failed` totals.

## Options

| Flag | What it does |
|------|--------------|
| *(none)* | Compile once (cached), then watch and flash boards as they're plugged in |
| `--recompile` | Force a fresh compile first. **Use this after editing `MotorParty.ino`** — otherwise it reuses the cached binary |
| `--cleanup-now` | Remove all disconnected Espressif (VID 0x303A) ghost COM ports, then exit |
| `--no-cleanup` | Flash normally but leave COM ports alone |
| `--any-vid` | Flash any newly appeared serial port, not just Espressif (0x303A). Use only if a board shows up under a different USB bridge |
| `--list` | Print the serial ports currently seen (and their VID:PID) and exit |

## Housekeeping

Clean up phantom ports that have already piled up (run once, as admin):

```
python flash_motorparty.py --cleanup-now
```

## Troubleshooting

- **`Access is denied` / `port is busy`** — another program owns the COM port.
  Close the **Arduino IDE Serial Monitor** (or the whole IDE) and any other
  serial terminal (PuTTY, etc.). The flasher retries 3× automatically for brief
  locks.
- **Board never detected** — with it plugged in, run `python flash_motorparty.py
  --list`. If it doesn't appear as Espressif `303A`, note the VID and either use
  `--any-vid` or tell the maintainer. Also confirm the board is actually in boot
  mode.
- **Ports not being removed** — you're not running as Administrator. The flasher
  prints a `WARN` when that's the case. Re-launch the terminal elevated.
- **Compile fails** — make sure the `Alfredo-NoU3` library and its dependencies
  are installed in your Arduino `libraries` folder.

## Notes / limitations

- Designed for **one board at a time**. It waits for you to unplug the current
  board before arming for the next (this also prevents a re-flash when the board
  resets and re-enumerates).
- Cleanup removes the dead **port entries** from the device list. Windows keeps a
  separate internal COM-number reservation, so new boards may still get climbing
  COM numbers — but your list won't fill with dead entries.
- Build output is cached in `tools/.build_motorparty/` (gitignored).
