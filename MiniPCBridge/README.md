# Mini PC Bridge

Runs on the backstage mini PC. Bridges each Arduino Mega's USB serial port
to a TCP port that the main PC (running TouchDesigner) connects to. See
`mega_serial_bridge.py`'s module docstring for how it fits into the overall
architecture.

## Getting the code onto the mini PC

Whichever is easiest given your setup:

- **OneDrive**: this repo lives under `OneDrive\Documents\FeelFestivalStageRepo`
  on the dev machine. If the mini PC is signed into the same OneDrive
  account, it may already be syncing there automatically — check before
  copying anything by hand.
- **USB drive**: copy the whole `MiniPCBridge` folder over. It's small and
  self-contained — only `mega_serial_bridge.py`, `test_client.py`, and this
  README.
- **Git**: this repo has no remote configured yet (it's local-only). If you
  want to `git pull` updates onto the mini PC going forward instead of
  re-copying by hand, that'd need a remote (e.g. a private GitHub repo) set
  up first — ask if you want that done.

## Setup

1. Install Python 3 on the mini PC if it isn't already there
   (python.org — check "Add Python to PATH" during install).
2. `pip install pyserial`
3. Find the actual COM ports for the two Megas: Device Manager → Ports
   (COM & LPT), plug in one Mega at a time if you're not sure which is
   which (the new entry that appears when you plug it in is that one).
4. Open `mega_serial_bridge.py` and edit the `SIDES` list with those port
   numbers — they're very likely different from whatever's configured now,
   since COM numbering is per-machine, not tied to the physical Mega.
5. Run it:
   ```
   python mega_serial_bridge.py
   ```
   You should see `Opened COM<n> @ 115200` for each side, and
   `Listening for main PC on 0.0.0.0:9000` / `:9001`. If a Mega isn't
   plugged in yet, that side logs a retry warning every 2s instead —
   expected, not an error.
6. Find the mini PC's IP address (`ipconfig`, look for the IPv4 address on
   the adapter connected to the main PC) — you'll need it both for testing
   below and later for `SerialRelayEXT`'s `Serverhost` parameter in TD.
7. **Windows Firewall**: the first time it binds a listening port, Windows
   may prompt "Allow python.exe to communicate on these networks?" — allow
   it (at least for Private networks). If you don't get prompted and the
   test below can't connect, check Windows Defender Firewall → Allowed apps
   for `python.exe`.

Leave it running for the duration of the show/rehearsal. It reconnects
automatically if a Mega is unplugged/replugged or the main PC disconnects —
you shouldn't need to restart it for either of those.

## Testing the bridge without TD

The bridge does no protocol translation — it just forwards raw ASCII
lines — so you can test it directly, the same way the Arduino's own serial
link was tested with the Serial Monitor. Windows doesn't ship a telnet
client by default, so use the included `test_client.py` instead (works
from the mini PC itself, or any other machine on the same network):

```
python test_client.py <mini-pc-ip> 9000
> STATUS
< STATUS 0 0 0 0 0 0.0000
< STATUS 1 0 0 0 0 0.0000
...
```

(7 `STATUS ...` lines, assuming the LEFT Mega is running the firmware and
connected.) Try `HOME 0`, `SETPOS 0 200`, etc. too. This confirms the full
path — network → bridge → serial → Mega → serial → bridge → network — is
healthy before worrying about whether a problem (if any) is on the TD side.

If you have a telnet client available, that works too (`telnet <ip> 9000`,
same commands) — `test_client.py` just doesn't depend on it being
installed/enabled.

## Auto-starting the bridge

Not set up here — for a permanent installation, consider Windows Task
Scheduler (run at login) or a scheduled/startup script, so the bridge
comes back up automatically if the mini PC reboots. Out of scope for now;
ask if you want this wired up.
