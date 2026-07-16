# TD Network Setup — Wave Motor Control

How to build the actual TouchDesigner node network on the main PC. All the
Python behind this already exists in `extensions/` and `dats/` — this is
about wiring those into a real `.toe` file.

One general caveat up front: I don't have a live TD environment to verify
exact UI labels (parameter names, dialog tab names) against your installed
version — the mechanics below (Customize Component, custom parameters,
Callbacks DAT) are stable, long-standing TD idioms I'm confident in, but if
a specific label doesn't match what you see, hover the parameter for its
tooltip or check the operator's own help page ('?' icon).

This guide covers the four pieces needed for the "home, then slow
continuous wave" test: `base_serial_left`, `base_serial_right`,
`base_motor_controller`, `base_choreography`. `base_osc_handler`,
`base_pid_tuning`, and `base_visualization` aren't covered yet — ask when
you're ready to wire those up.

---

## Custom parameters reference

Quick lookup for every COMP that needs a Custom parameter page, pulled
directly from what each extension actually reads via `.par.X` — parameter
**Name** fields are case-sensitive and must match exactly, since that's
what `.par.X` in the code looks up (the Label next to it is just display
text and can be anything).

| COMP | Extension | Parameter | Type | Default |
|---|---|---|---|---|
| wherever `SerialEXT` is attached (local bench-test transport) | `SerialEXT` | `Serialport` | Str | e.g. `'COM3'` |
| | | `Baudrate` | Int | `115200` |
| | | `Motoroffset` | Int | `0` |
| `base_serial_left` | `SerialRelayEXT` | `Serverhost` | Str | mini PC IP |
| | | `Serverport` | Int | `9000` |
| | | `Motoroffset` | Int | `0` |
| `base_serial_right` | `SerialRelayEXT` | `Serverhost` | Str | mini PC IP |
| | | `Serverport` | Int | `9001` |
| | | `Motoroffset` | Int | `7` |
| `base_motor_controller` | `MotorControllerEXT` | `Autorehomedrift` | Int | `200` |
| | | `Pid_kp` | Float | `0.8` |
| | | `Pid_ki` | Float | `0.01` |
| | | `Pid_kd` | Float | `0.1` |
| `base_choreography` | `ChoreographyEXT` | `Waveamplitude` | Int | `800` |
| | | `Wavefrequency` | Float | `0.2` |
| | | `Wavephaseoffset` | Float | `0.4` |
| | | `Wavemode` | Menu | items exactly `SINE`, `TRIANGLE`, `CUSTOM` (case-sensitive — code does exact string comparison) — default `SINE` |
| | | `Playback` | Toggle | off |
| | | `Activecue` | Int | `0` |

**`OSCHandlerEXT` needs none of its own** — it never reads
`self.ownerComp.par.anything`; it only reads/writes other COMPs'
parameters (e.g. `base_choreography`'s `Waveamplitude`) by looking them
up. Its COMP just needs the extension attached, no custom page. (The OSC
In DAT itself has its own built-in parameters — Port, Active, etc. — but
those are the operator's native parameters, not a custom page.)

`PIDTuningEXT` and `base_visualization` aren't written yet, so no
parameter list for those until they're built.

---

## 0. Prerequisites

- `mega_serial_bridge.py` running on the mini PC, both sides showing
  `Opened COM<n>` and `Listening for main PC on 0.0.0.0:900x`.
- The mini PC's IP address (from `ipconfig` there).

## 1. Top-level container

At the root of your network (or wherever you want this to live within
`WaveMotorController.toe`):

1. Add Operator → COMP → **Base COMP**. Rename it `base_motor_system`.
2. Double-click to dive inside — everything below lives in here.

## 2. The `extensions` folder

This holds one canonical copy of each class, referenced by everything else.

1. Inside `base_motor_system`, add a **Base COMP** named `extensions`. It
   doesn't need any extension attached to itself — just a container.
2. Dive inside. For each class below, add a **Text DAT**, rename it to
   match exactly, open it, and paste the full contents of the
   corresponding file from `extensions/`:
   - `SerialProtocolBase`
   - `SerialEXT`
   - `SerialRelayEXT`
   - `MotorControllerEXT`
   - `ChoreographyEXT`
3. (Optional, cosmetic) On each Text DAT's Text page, set **Language** to
   `Python` for syntax highlighting.
4. Back out to `base_motor_system` (Backspace, or click the parent
   breadcrumb at the top of the network view).

## 3. `base_serial_left`

1. Add a **Base COMP** named `base_serial_left`.
2. Right-click it → **Customize Component...** → **Extensions** page.
   Set `Extension 1` to point at `../extensions/SerialRelayEXT` (a
   relative path to that Text DAT). TD reads the class name
   (`SerialRelayEXT`) directly from the file, so you don't type it
   separately — it's just the path that matters.
3. Still in Customize Component, add three **Custom Parameters**:
   | Name | Type | Default |
   |---|---|---|
   | `Serverhost` | Str | the mini PC's IP, e.g. `192.168.1.50` |
   | `Serverport` | Int | `9000` |
   | `Motoroffset` | Int | `0` |
4. Close the dialog, dive inside `base_serial_left`:
   - Add a **TCP/IP DAT**, rename it `tcpip_left`.
   - Add a **Text DAT**, rename it `tcp_callback`, paste the contents of
     `dats/tcp_callback.py`.
   - On `tcpip_left`'s parameters, find **Callbacks DAT** and point it at
     `tcp_callback`.
5. Back out to `base_motor_system`.

## 4. `base_serial_right` — mirror of the above

Fastest way: select `base_serial_left`, copy/paste it, rename the copy to
`base_serial_right`. Then:

- Change its custom parameters: `Serverport` = `9001`, `Motoroffset` = `7`
  (`Serverhost` stays the same mini PC IP).
- Rename the inner TCP/IP DAT to `tcpip_right` if you want the naming to
  stay consistent (cosmetic only).

## 5. `base_motor_controller`

1. Add a **Base COMP** named `base_motor_controller`.
2. Customize Component → Extensions: `Extension 1` = `../extensions/MotorControllerEXT`.
3. Custom Parameters:
   | Name | Type | Default |
   |---|---|---|
   | `Autorehomedrift` | Int | `200` |
   | `Pid_kp` | Float | `0.8` |
   | `Pid_ki` | Float | `0.01` |
   | `Pid_kd` | Float | `0.1` |
4. Dive inside, add two **Table DAT**s: `motor_state_table` and
   `event_log`. Leave both empty — the code populates them at runtime.

## 6. `base_choreography`

1. Add a **Base COMP** named `base_choreography`.
2. Customize Component → Extensions: `Extension 1` = `../extensions/ChoreographyEXT`.
3. Custom Parameters:
   | Name | Type | Default |
   |---|---|---|
   | `Waveamplitude` | Int | `800` |
   | `Wavefrequency` | Float | `0.2` |
   | `Wavephaseoffset` | Float | `0.4` |
   | `Wavemode` | Menu | items: `SINE`, `TRIANGLE`, `CUSTOM` — default `SINE` |
   | `Playback` | Toggle | off |
   | `Activecue` | Int | `0` |
4. Dive inside, add an **Execute DAT**, rename it `choreo_execute`. Open
   it and paste the contents of `dats/choreo_execute.py`.
5. On `choreo_execute`'s parameters, enable **Frame Start** (this is what
   makes `onFrameStart` fire every frame). Confirm **Active** is also on.

---

## 7. First test

From TD's **Textport** (Alt+T, or Dialogs → Textport):

```python
# Connect both sides to the mini PC
op('base_serial_left').ext.SerialRelayEXT.Connect()
op('base_serial_right').ext.SerialRelayEXT.Connect()
```

Check the textport for `[base_serial_left] Mega READY` / same for right —
confirms the TCP link to the bridge and the serial link to each Mega are
both up. If you don't see it, work backwards: is `mega_serial_bridge.py`
still running, does `Serverhost` match its actual IP, is the firewall
prompt handled.

```python
# Home everything
op('base_motor_controller').ext.MotorControllerEXT.HomeAll()
```

Watch `motor_state_table` (or textport `HOMED <id>` lines) until all
connected motors show homed. Then:

```python
# Start conservative -- lower amplitude than the WAVE_SLOW default for a
# first pass with everything connected at once
op('base_choreography').par.Waveamplitude = 200

op('base_choreography').par.Playback = 1
op('base_choreography').ext.ChoreographyEXT.GoCueByName('WAVE_SLOW')
```

Emergency stop, any time:

```python
op('base_motor_controller').ext.MotorControllerEXT.StopAll()
```

Once that's confirmed working, `op('base_choreography').par.Waveamplitude`
can be raised back toward the cue's normal 800-step default.

## Not yet covered here

`base_osc_handler` (external control surface input), `base_pid_tuning`
(live gain-tuning panel), and `base_visualization` (3D preview) all have
their Python written already but aren't detailed in this build guide yet —
ask when you're ready to wire one of them up.
