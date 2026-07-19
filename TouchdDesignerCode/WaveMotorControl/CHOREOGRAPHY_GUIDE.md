# ChoreographyEXT — Complete Usage Guide

`ChoreographyEXT` is the motion-design layer. Every frame it decides "what
should each of the 14 motors be doing right now" and pushes that to
`MotorControllerEXT`, which handles the serial/network plumbing. It never
talks to the Megas directly and runs no PID — it only computes targets.

---

## 1. How it fits together

```
Execute DAT (Frame Start)  --calls every frame-->  ChoreographyEXT.Update()
                                                          |
                                    SetIdealPos() / SetSpeed() per motor
                                                          v
                                                 MotorControllerEXT
                                                          |
                                              SETPOS / SETSPEED over serial
                                                          v
                                                    the Megas
```

- Lives on the `base_choreography` COMP.
- **Requires** `base_motor_controller` to exist as a sibling COMP — the
  internal `_motorController()` lookup is `self.ownerComp.parent().op('base_motor_controller')`.
- Driven by an **Execute DAT** with **Frame Start** enabled, containing
  `dats/choreo_execute.py`, which just calls `parent().ext.ChoreographyEXT.Update()`.

---

## 2. Prerequisites checklist

**Custom parameters** on `base_choreography` (exact Names, case-sensitive):

| Name | Type | Purpose |
|---|---|---|
| `Waveamplitude` | Int | wave peak displacement, steps |
| `Wavefrequency` | Float | wave rate, Hz |
| `Wavephaseoffset` | Float | spatial phase between motors, radians |
| `Wavemode` | Menu | `SINE`, `TRIANGLE`, `CUSTOM` (exact strings) |
| `Constantspeed` | Float | continuous-rotation speed, signed steps/sec |
| `Playback` | Toggle | master on/off for per-frame output |
| `Activecue` | Int | current cue index (set by GoCue; don't edit by hand) |

**Child operators:**
- An Execute DAT (Frame Start on, Active on) running `choreo_execute.py` — **required**.
- `custom_wave` CHOP — *optional*, only for `Wavemode = CUSTOM`.
- `custom_speed` CHOP — *optional*, only for `constant` cues / audio reactivity.

If any custom parameter is missing you'll get
`'td.ParCollection' object has no attribute 'X'` the moment a cue needs it.

---

## 3. The core model: cues + Playback

Two independent switches decide what happens:

1. **`Activecue`** — *which* cue is loaded (set only via `GoCue`/`GoCueByName`/`NextCue`/`PrevCue`, never by editing the parameter directly).
2. **`Playback`** — the master gate. `Update()` returns immediately if it's off.

> **The #1 gotcha:** `GoCueByName('WAVE_SLOW')` succeeds and loads the cue,
> but **nothing moves until `Playback = 1`**. Loading a cue and enabling
> output are two separate steps.

Only `wave` and `constant` cues do per-frame work in `Update()`. The
`home`, `hold`, and `absolute` cues are **one-shot** — they act once inside
`GoCue()` and then need no per-frame updates.

---

## 4. The built-in cues

| Name | Type | What it does | maxspeed / accel |
|---|---|---|---|
| `HOME` | home | Homes all 14 motors (`HomeAll`) | — |
| `FREEZE` | hold | Freezes: stops any spinning motor, holds positions | — |
| `WAVE_SLOW` | wave | Sine wave, amp 800, 0.2 Hz, phase 0.4 | 2000 / 800 |
| `WAVE_FAST` | wave | Sine wave, amp 400, 0.8 Hz, phase 0.6 | 6000 / 3000 |
| `UNISON_UP` | absolute | All motors to +800 steps | 4000 / 1500 |
| `UNISON_DOWN` | absolute | All motors to −800 steps | 4000 / 1500 |
| `SPIN` | constant | Continuous rotation at `Constantspeed` | 8000 / — |

---

## 5. Running it — textport quickstart

Adjust the `/project1/base_choreography` path to wherever your COMP lives.

**Home, then a slow wave:**
```python
op('/project1/base_choreography').ext.ChoreographyEXT.GoCueByName('HOME')
# wait for all motors to report homed, then:
op('/project1/base_choreography').par.Playback = 1
op('/project1/base_choreography').ext.ChoreographyEXT.GoCueByName('WAVE_SLOW')
```

**Continuous spin:**
```python
op('/project1/base_choreography').par.Constantspeed = 400   # steps/sec, sign = direction
op('/project1/base_choreography').par.Playback = 1
op('/project1/base_choreography').ext.ChoreographyEXT.GoCueByName('SPIN')
```

**Stop / freeze:**
```python
op('/project1/base_choreography').ext.ChoreographyEXT.GoCueByName('FREEZE')  # holds in place
# or a full emergency stop (also pauses Playback):
op('/project1/base_motor_controller').ext.MotorControllerEXT.StopAll()
```

---

## 6. Wave cues in depth

A wave cue computes each motor's position every frame as a function of
**absolute time** (`absTime.seconds`), so it stays smooth across cook rate
changes and never accumulates drift.

`ComputeWavePos(motor_index, t)`:
```
spatial_phase = MOTOR_POSITION[motor_index] * Wavephaseoffset
phase         = 2*pi * Wavefrequency * t + spatial_phase
normalized    = shape(phase)          # per Wavemode
return int(normalized * Waveamplitude)
```

**Live tuning:** once a wave cue is running, changing `Waveamplitude`,
`Wavefrequency`, `Wavephaseoffset`, or `Wavemode` takes effect on the next
frame — they're read fresh every `Update()`. Bind sliders to them.

> **But note:** calling `GoCue` on a wave cue **overwrites** amplitude/
> frequency/phase from the cue's stored values. So if you hand-tune
> `Waveamplitude` and then re-select `WAVE_SLOW`, it snaps back to 800.
> Tune *after* selecting the cue, or edit the cue's stored values (§9).

**Spatial phase & the MOTOR_POSITION table:** phase per motor is driven by
`MOTOR_POSITION`, currently set for a **mirrored-from-center** layout
(motors 0 and 7 nearest center, 6 and 13 outermost). This is a best-guess
default — **verify it against your physical rig** and edit the table at the
top of the file if a side is reversed.

### Wave modes
- `SINE` — smooth sinusoid.
- `TRIANGLE` — linear triangle wave.
- `CUSTOM` — reads channel `motor_index` of a **`custom_wave` CHOP** as the
  normalized (−1..1) shape, then scales by `Waveamplitude`. This lets you
  design motion with native CHOP tools (LFO, Wave, audio-reactive, noise…)
  instead of code. Silently outputs 0 if no `custom_wave` CHOP exists.

---

## 7. Constant (SPIN) cue + audio reactivity

The `constant` cue commands continuous rotation via `SetSpeed` (the
firmware's SETSPEED mode) — **not** position. There's no position target
tracked on the TD side while spinning.

Each frame, for every motor:
```
speed = custom_speed[i]   if a custom_speed CHOP with channel i exists
        Constantspeed     otherwise
SetSpeed(i, speed)
```
`SetSpeed` change-detects, so calling every frame only actually sends a
command when a motor's speed changes.

**Audio reactivity:** wire an audio-analysis CHOP into a **`custom_speed`
CHOP** child of `base_choreography` — e.g. `Audio Device In → Analyze` (or a
Spectrum band split), scaled to steps/sec, one channel per motor. While a
`constant` cue is active with Playback on, each motor's rotation speed
follows its channel live.

> Speed changes in SETSPEED mode are **instant** — the firmware does not
> ramp them (acceleration only applies to SETPOS moves). If audio slams the
> speed frame-to-frame, put a **Lag/Filter CHOP** before `custom_speed` to
> smooth it, or a heavy NEMA 34 can lurch / lose steps.

---

## 8. Per-cue speed & acceleration limits

Each cue may carry optional `maxspeed` (steps/sec) and `accel` (steps/sec²).
When the cue activates, `GoCue` sends these to the Megas *first* (before any
motion), so a cue can raise its own governor for fast motion or lower it for
gentle motion — no separate manual command.

- Omit the keys to leave the current limits untouched.
- `SPIN` sets `maxspeed 8000` (no `accel` — acceleration is irrelevant to
  continuous rotation). This also **caps `Constantspeed`** — you can't spin
  faster than the ceiling.
- **These only work on the PID-free firmware** (`motor_controller_without_pid.ino`),
  which is the build that accepts `SETMAXSPEED`/`SETACCEL`. On the original
  PID build and the simple build they're silently ignored (`ERR unknown
  command`, harmless). On the simple build, max speed/accel are compile-time
  only.

---

## 9. Adding or editing cues

Cues are plain dicts in `self.cues` (in `__init__`). Edit the list directly.

```python
# A fast triangle wave with its own limits:
{'name': 'RIPPLE', 'type': 'wave', 'amplitude': 600, 'frequency': 0.5,
 'phase': 0.8, 'maxspeed': 5000, 'accel': 2500},

# Send specific motors to specific positions:
{'name': 'STAGGER', 'type': 'absolute',
 'positions': [i * 100 for i in range(14)], 'maxspeed': 4000, 'accel': 1500},

# A gentle reverse spin:
{'name': 'DRIFT', 'type': 'constant', 'maxspeed': 3000},
```

Required keys per type:
- `home` / `hold` — just `name` + `type`.
- `wave` — `amplitude`, `frequency`, `phase`.
- `absolute` — `positions` (list of 14 step targets).
- `constant` — none beyond `name`/`type` (speed comes from `Constantspeed`/`custom_speed`).
- `maxspeed` / `accel` — optional on any cue.

Cue **order** matters for `NextCue`/`PrevCue` and for the numeric
`Activecue` index used by OSC `/motor/cue`.

---

## 10. Navigation & OSC

```python
ext.GoCue(index)            # by numeric index
ext.GoCueByName('WAVE_FAST')# by name (preferred — order-independent)
ext.NextCue()               # clamps at the last cue
ext.PrevCue()               # clamps at the first cue
```

Via OSC (`OSCHandlerEXT`, if wired): `/motor/cue <i>`, `/motor/next`,
`/motor/prev`, plus `/motor/wave/amp|freq|phase` to tweak wave params live.

---

## 11. Method reference

| Method | Use |
|---|---|
| `Update()` | Called by the Execute DAT every frame. Don't call manually. |
| `GoCue(i)` | Activate cue by index. Applies its limits, runs its one-shot action. |
| `GoCueByName(name)` | Same, by name. |
| `NextCue()` / `PrevCue()` | Step through the cue list (clamped). |
| `ComputeWavePos(i, t)` | Position of motor `i` at time `t` for the active wave params. |
| `_applyCueLimits(cue, ctrl)` | Internal — pushes a cue's maxspeed/accel. |
| `_motorController()` | Internal — resolves the sibling MotorControllerEXT. |

---

## 12. Troubleshooting

| Symptom | Cause |
|---|---|
| Cue selected but nothing moves | `Playback` is off. Set it to 1. |
| `AttributeError: ... 'Constantspeed'` (or another par) | That custom parameter isn't on `base_choreography` yet. Add it (§2). |
| `Update` never runs / prints | Execute DAT missing, or Frame Start / Active off, or `choreo_execute.py` not pasted into it. |
| Wave params snap back when I re-trigger the cue | `GoCue` overwrites amp/freq/phase from the cue dict. Tune after selecting, or edit the cue. |
| SPIN won't exceed a speed | `Constantspeed` is capped by the cue's `maxspeed` (8000), and only the without_pid firmware honors it — on other builds it's the compile-time ceiling. |
| Per-cue maxspeed/accel do nothing | You're not on `motor_controller_without_pid.ino`. Only that build accepts SETMAXSPEED/SETACCEL. |
| Motors home but wave looks wrong across the rig | `MOTOR_POSITION` layout doesn't match your physical wiring. Edit the table. |
| Spin lurches with audio | No smoothing on `custom_speed`. Add a Lag/Filter CHOP. |

---

## 13. Firmware compatibility at a glance

| Feature | PID build | without_pid | simple |
|---|---|---|---|
| `wave` / `absolute` cues (SETPOS) | ✓ | ✓ | ✓ |
| `SPIN` / `constant` cues (SETSPEED) | ✓ | ✓ | ✓ |
| Per-cue `maxspeed` / `accel` | ignored | ✓ | ignored |
| Continuous-rotation drift correction | ✓ (resync) | ✓ (resync) | ✗ (open-loop) |

ChoreographyEXT itself is firmware-agnostic — it sends the same commands
regardless; the firmware decides what it honors.
