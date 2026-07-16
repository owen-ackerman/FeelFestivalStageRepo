# Mini PC Bridge

Runs on the backstage mini PC. Bridges each Arduino Mega's USB serial port
to a TCP port that the main PC (running TouchDesigner) connects to. See
`mega_serial_bridge.py`'s module docstring for how it fits into the overall
architecture.

## Setup

1. Install Python 3 on the mini PC if it isn't already there.
2. `pip install pyserial`
3. Open `mega_serial_bridge.py` and edit the `SIDES` list — set
   `serial_port` to the actual COM port (Windows) or device path (e.g.
   `/dev/ttyACM0` on Linux) for each Mega. Confirm in Device Manager /
   `ls /dev/tty*` which port is LEFT vs RIGHT.
4. Run it:
   ```
   python mega_serial_bridge.py
   ```
   You should see `Opened COM3 @ 115200` (or similar) for each side, and
   `Listening for main PC on 0.0.0.0:9000` / `:9001`.

Leave it running for the duration of the show/rehearsal. It reconnects
automatically if a Mega is unplugged/replugged or the main PC disconnects —
you shouldn't need to restart it for either of those.

## Testing the bridge without TD

Since the bridge does no protocol translation — it just forwards raw
ASCII lines — you can test it with any raw TCP client, the same way the
Arduino's own serial link was tested with the Serial Monitor:

```
telnet <mini-pc-ip> 9000
STATUS
```

You should see 7 `STATUS ...` lines echo back (assuming the LEFT Mega is
running the firmware and connected). This is a good way to confirm the
mini PC → Mega link is healthy before worrying about whether the problem
(if any) is on the TD side.

## Auto-starting the bridge

Not set up here — for a permanent installation, consider Windows Task
Scheduler (run at login) or a scheduled/startup script, so the bridge
comes back up automatically if the mini PC reboots. Out of scope for now;
ask if you want this wired up.
