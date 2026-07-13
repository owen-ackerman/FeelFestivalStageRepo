# Festival Stage Motor Control System
## Implementation Specification for Claude Code

---

## Project Overview

A festival stage installation with **14 NEMA 34 stepper motors** arranged in two symmetric groups of 7, spanning ~30m across a stage. Motors are controlled from a **Front of House (FOH) laptop** via a **backstage mini PC** running TouchDesigner. Each group of 7 motors is managed by a dedicated **Arduino Mega 2560**, receiving commands over USB serial and executing motion autonomously. Each motor has a **homing sensor** (limit switch via PC817 optocoupler breakout board) at θ=0.

**Control architecture:** TouchDesigner computes and streams ideal motor positions to each Mega. The **Arduino runs a PID controller** that continuously drives each motor's actual position toward the ideal position. The Mega reports actual positions back to TD for monitoring only — TD does not close the control loop.

---

## Hardware Architecture

```
FOH Laptop (OSC/UDP)
    │
    │ 30m Cat6 Ethernet
    │
Mini PC — backstage (TouchDesigner)
    │                        │
    │ USB 20m active          │ USB 20m active
    │                        │
Arduino Mega LEFT           Arduino Mega RIGHT
7× DM860I drivers           7× DM860I drivers
7× NEMA 34 motors           7× NEMA 34 motors
7× homing sensors           7× homing sensors
(optocoupler isolated)      (optocoupler isolated)
```

### Responsibility Split

| Responsibility | Owner |
|---|---|
| Wave patterns, cue sequencing, show control | TouchDesigner |
| Streaming ideal positions to Megas | TouchDesigner |
| PID control loop (ideal → actual) | **Arduino Mega** |
| Step generation, acceleration profiles | **Arduino Mega** |
| Homing sequences, sensor ISRs | **Arduino Mega** |
| Position reporting (monitoring only) | Arduino → TouchDesigner |
| Re-home trigger on catastrophic drift | TouchDesigner (safety fallback) |

### Key Hardware Facts
- **Motors:** NEMA 34 steppers, up to 7.8A peak, driven at 48V
- **Drivers:** DM860I — STEP/DIR/EN interface, internal to each box (<20cm runs)
- **Homing sensors:** Limit switches wired through PC817 optocoupler breakout boards. Output is **active LOW** (pin reads LOW when sensor triggered). Wired to interrupt-capable pins where possible.
- **Motor phase cables:** Up to 10m from driver to motor, 4-core shielded 1.5mm²
- **Sensor cables:** Up to 10m from motor to box, 3-core shielded LiYCY, powered at 12V sensor side, optocoupler output goes to Arduino 5V logic side
- **Position unit:** All positions expressed in **steps from home (θ=0)**. Sign convention: positive = forward from home.
- **Microstepping:** Set on DM860I DIP switches — confirm setting before coding (determines steps/revolution). Assume 1600 steps/rev (1/8 microstepping) unless specified otherwise.

---

## File Structure to Implement

```
project/
├── arduino/
│   └── motor_controller/
│       └── motor_controller.ino       # Single sketch, runs on both Megas
│
└── touchdesigner/
    ├── extensions/
    │   ├── SerialEXT.py               # Serial I/O handler (one instance per Mega)
    │   ├── MotorControllerEXT.py      # State model + monitoring (no PID)
    │   ├── ChoreographyEXT.py         # Ideal position + show cue management
    │   ├── OSCHandlerEXT.py           # FOH OSC input receiver
    │   └── PIDTuningEXT.py            # Live PID tuning panel logic
    └── README_TD_SETUP.md             # How to wire up the TD network
```

---

## Part 1: Arduino Firmware

### File: `arduino/motor_controller/motor_controller.ino`

#### Dependencies
- `AccelStepper` library (install via Arduino Library Manager)
- Standard Arduino libraries only (no additional dependencies)

#### Constants to Define at Top of File

```cpp
// Number of motors this Mega controls
#define NUM_MOTORS 7

// Reporting interval (milliseconds) — monitoring only, not control
#define REPORT_INTERVAL_MS 50

// Homing speed (steps/sec) — slow and safe
#define HOMING_SPEED 400

// Homing timeout (milliseconds) — abort if sensor not found
#define HOMING_TIMEOUT_MS 15000

// Serial baud rate
#define BAUD_RATE 115200

// Report deadband (steps) — suppress POS messages for tiny changes
#define REPORT_DEADBAND 2

// PID deadband (steps) — error smaller than this is ignored by PID
#define PID_DEADBAND 5

// PID integral clamp — prevents windup
#define INTEGRAL_CLAMP 500.0

// Auto re-home threshold (steps) — if drift exceeds this, request re-home
// Note: re-home is triggered by TD via HOME command, not autonomously
#define REHOME_THRESHOLD 200

// Homing direction: +1 or -1 — set per installation
// Defines which direction the motor moves to find the sensor
#define HOMING_DIR -1

// Max choreography speed (steps/sec) — used for SETPOS moves
#define MAX_SPEED 4000

// Acceleration (steps/sec²)
#define ACCELERATION 800

// PID correction max speed (steps/sec) — gentler than choreography moves
#define PID_MAX_SPEED 800
```

#### PID Tuning Constants

```cpp
// PID gains — can be updated at runtime via SETPID serial command
float Kp = 0.8;
float Ki = 0.01;
float Kd = 0.1;
```

#### Pin Assignments

```cpp
// STEP pins — digital output
const int STEP_PIN[NUM_MOTORS] = {2, 4, 6, 8, 10, 12, 24};

// DIR pins
const int DIR_PIN[NUM_MOTORS]  = {3, 5, 7, 9, 11, 13, 25};

// EN pins (active LOW on DM860 — LOW = enabled, HIGH = disabled)
const int EN_PIN[NUM_MOTORS]   = {26, 27, 28, 29, 30, 31, 32};

// Homing sensor pins — optocoupler board provides pull-up, use INPUT not INPUT_PULLUP
// Interrupt-capable pins on Mega: 2, 3, 18, 19, 20, 21
// Remaining sensors polled in loop()
const int SENSOR_PIN[NUM_MOTORS] = {18, 19, 20, 21, 22, 23, 33};
```

> **Note for Claude Code:** Assign pin numbers carefully. STEP/DIR/EN must not conflict with sensor pins. Pins 0/1 are reserved for Serial. Pins 20/21 are SDA/SCL — avoid if I2C used later. Verify all assignments against the Mega 2560 pinout before finalising.

#### State Variables

```cpp
AccelStepper stepper[NUM_MOTORS];

// Position tracking
volatile int32_t actual_pos[NUM_MOTORS];  // steps from home — volatile: ISR writes it
int32_t ideal_pos[NUM_MOTORS];            // target set by TD via SETPOS
bool homed[NUM_MOTORS];                   // true after first successful home
bool homing_active[NUM_MOTORS];           // currently executing home sequence
unsigned long homing_start_time[NUM_MOTORS];

// PID state — one set per motor
float pid_integral[NUM_MOTORS];
float pid_prev_error[NUM_MOTORS];

// ISR flags — set in ISR, cleared in loop()
volatile bool homed_flag[NUM_MOTORS];

// Reporting
int32_t last_reported_pos[NUM_MOTORS];
unsigned long last_report_time = 0;

// Serial input buffer
String input_buffer = "";
```

#### Homing Sensor ISRs

Create one ISR per motor. ISRs must be minimal — no Serial calls, no String operations.

```cpp
// Motor 0 example — repeat pattern for motors 1–3 (hardware interrupt pins)
void sensor_isr_0() {
    actual_pos[0] = 0;
    stepper[0].setCurrentPosition(0);
    ideal_pos[0] = 0;          // resync ideal to home on crossing
    pid_integral[0] = 0;       // reset integral on home
    pid_prev_error[0] = 0;
    homed[0] = true;
    homing_active[0] = false;
    homed_flag[0] = true;      // loop() sends HOMED message
}
// sensor_isr_1, sensor_isr_2, sensor_isr_3 — identical pattern
```

> **ISR Safety:** `Serial.println()` must NOT be called inside an ISR. Use `volatile bool homed_flag[]`. The main loop checks flags each iteration and sends HOMED messages.

For sensors on non-hardware-interrupt pins (22, 23, 33), implement `pollSensors()` in the main loop with software debounce (10ms minimum debounce time).

#### `setup()`

```cpp
void setup() {
    Serial.begin(BAUD_RATE);

    for (int i = 0; i < NUM_MOTORS; i++) {
        stepper[i] = AccelStepper(AccelStepper::DRIVER, STEP_PIN[i], DIR_PIN[i]);
        stepper[i].setMaxSpeed(MAX_SPEED);
        stepper[i].setAcceleration(ACCELERATION);
        stepper[i].setEnablePin(EN_PIN[i]);
        stepper[i].setPinsInverted(false, false, true); // EN active LOW
        stepper[i].enableOutputs();

        pinMode(SENSOR_PIN[i], INPUT); // optocoupler provides pull-up

        // Init all state
        actual_pos[i]      = 0;
        ideal_pos[i]       = 0;
        last_reported_pos[i] = 0;
        homed[i]           = false;
        homing_active[i]   = false;
        homed_flag[i]      = false;
        pid_integral[i]    = 0.0;
        pid_prev_error[i]  = 0.0;
    }

    // Attach hardware interrupts (motors 0–3)
    attachInterrupt(digitalPinToInterrupt(SENSOR_PIN[0]), sensor_isr_0, FALLING);
    attachInterrupt(digitalPinToInterrupt(SENSOR_PIN[1]), sensor_isr_1, FALLING);
    attachInterrupt(digitalPinToInterrupt(SENSOR_PIN[2]), sensor_isr_2, FALLING);
    attachInterrupt(digitalPinToInterrupt(SENSOR_PIN[3]), sensor_isr_3, FALLING);
    // Motors 4–6: polled in loop() via pollSensors()

    Serial.println("READY");
}
```

#### `loop()`

No `delay()` calls anywhere. Must execute as fast as possible.

```cpp
void loop() {
    // 1. Run PID for each motor — updates stepper target
    for (int i = 0; i < NUM_MOTORS; i++) {
        runPID(i);
    }

    // 2. Execute stepper motion (must run every loop)
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (homing_active[i]) {
            stepper[i].runSpeed();   // constant speed during homing
        } else {
            stepper[i].run();        // acceleration-managed motion otherwise
        }
        actual_pos[i] = stepper[i].currentPosition();
    }

    // 3. Handle homed flags set by ISRs
    checkHomedFlags();

    // 4. Poll sensors for motors 4–6
    pollSensors();

    // 5. Check homing timeouts
    checkHomingTimeouts();

    // 6. Send periodic position reports to TD (monitoring)
    sendPositionReports();

    // 7. Parse incoming serial commands from TD
    parseSerial();
}
```

#### PID Controller — `runPID(int i)`

This is the core control function. Called every loop iteration per motor.

```cpp
void runPID(int i) {
    // Guards
    if (!homed[i])        return;  // no reference until homed
    if (homing_active[i]) return;  // don't interfere with homing

    float error = (float)(ideal_pos[i] - actual_pos[i]);

    // Dead band — ignore noise, bleed integral
    if (abs(error) < PID_DEADBAND) {
        pid_integral[i] *= 0.95;   // gentle integral decay in dead band
        return;
    }

    // Integral with anti-windup clamp
    pid_integral[i] += error;
    if (pid_integral[i] >  INTEGRAL_CLAMP) pid_integral[i] =  INTEGRAL_CLAMP;
    if (pid_integral[i] < -INTEGRAL_CLAMP) pid_integral[i] = -INTEGRAL_CLAMP;

    // Derivative
    float derivative = error - pid_prev_error[i];
    pid_prev_error[i] = error;

    // PID output
    float output = Kp * error + Ki * pid_integral[i] + Kd * derivative;

    // Convert output to corrected target position
    int32_t corrected_target = ideal_pos[i] + (int32_t)output;

    // Command AccelStepper — use PID_MAX_SPEED for gentle corrections
    stepper[i].setMaxSpeed(PID_MAX_SPEED);
    stepper[i].moveTo(corrected_target);
}
```

> **Design note:** `PID_MAX_SPEED` limits correction velocity so drift correction is invisible to the audience. When a new SETPOS arrives from a choreography cue, the firmware temporarily sets `MAX_SPEED` for the initial move, then hands control back to the PID at `PID_MAX_SPEED`. See SETPOS handling below.

#### Serial Command Parser

Read one character at a time into `input_buffer`. On `'\n'`, tokenise and dispatch.

```cpp
void parseSerial() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n') {
            input_buffer.trim();
            if (input_buffer.length() > 0) {
                handleCommand(input_buffer);
            }
            input_buffer = "";
        } else {
            input_buffer += c;
        }
    }
}
```

#### Command Handlers

**`SETPOS <id> <ideal_steps>`**
The primary TD → Mega command. Sets the PID target for one motor.
```cpp
// On large choreography moves: briefly allow full speed, PID resumes correction
ideal_pos[id] = target_steps;
stepper[id].setMaxSpeed(MAX_SPEED);
stepper[id].moveTo(target_steps);
// PID will take over refinement on next loop iterations
```

**`HOME <id>`**
```cpp
homing_active[id] = true;
homed[id] = false;
homing_start_time[id] = millis();
pid_integral[id] = 0;
pid_prev_error[id] = 0;
stepper[id].setSpeed(HOMING_SPEED * HOMING_DIR);
// runSpeed() called in loop() while homing_active
```

**`HOMEALL`**
Execute HOME for all 7 motors simultaneously.

**`STOP <id>`**
```cpp
stepper[id].stop();
homing_active[id] = false;
```

**`STOPALL`**
Stop all 7 motors immediately.

**`ENABLE <id>` / `DISABLE <id>`**
Call `stepper[id].enableOutputs()` / `stepper[id].disableOutputs()`.

**`SETPID <Kp> <Ki> <Kd>`** — Global (all motors)
Update PID gains for all 7 motors on this Mega at runtime without reflashing.
```cpp
Kp = new_kp;
Ki = new_ki;
Kd = new_kd;
// Also reset integral and derivative state on all motors to prevent
// windup carryover from old gains causing a jerk on gain change
for (int i = 0; i < NUM_MOTORS; i++) {
    pid_integral[i]   = 0.0;
    pid_prev_error[i] = 0.0;
}
Serial.println("PID_UPDATED");
```

**`SETPID <id> <Kp> <Ki> <Kd>`** — Per-motor
Update PID gains for one specific motor. Requires per-motor gain arrays:
```cpp
float motor_kp[NUM_MOTORS];
float motor_ki[NUM_MOTORS];
float motor_kd[NUM_MOTORS];
```
Initialise all to global Kp/Ki/Kd defaults in `setup()`. When per-motor SETPID received, update only `motor_kp[id]`, `motor_ki[id]`, `motor_kd[id]` and reset that motor's PID state. `runPID()` uses per-motor arrays instead of globals.

Reply: `PID_UPDATED <id>` (with motor id) to distinguish from global update.

**`RESETPID`** — Reset PID state without changing gains
```cpp
for (int i = 0; i < NUM_MOTORS; i++) {
    pid_integral[i]   = 0.0;
    pid_prev_error[i] = 0.0;
}
Serial.println("PID_RESET");
```
Useful after manual repositioning or re-home, to prevent integral windup from old error history causing a jerk.

**`STATUS`**
Send full state for all 7 motors, one line each:
```
STATUS <id> <actual_pos> <ideal_pos> <homed> <homing_active> <pid_integral>
```

#### Position Reporting — `sendPositionReports()`

Every `REPORT_INTERVAL_MS`, for each motor whose position has changed by more than `REPORT_DEADBAND` steps:

```
POS <id> <actual_steps>
```

This is for TD monitoring only — TD does not use these values to close a control loop.

#### Homing Timeout — `checkHomingTimeouts()`

```cpp
for (int i = 0; i < NUM_MOTORS; i++) {
    if (homing_active[i]) {
        if (millis() - homing_start_time[i] > HOMING_TIMEOUT_MS) {
            stepper[i].stop();
            homing_active[i] = false;
            Serial.print("FAULT ");
            Serial.print(i);
            Serial.println(" TIMEOUT");
        }
    }
}
```

#### Messages Sent by Mega → TD

```
READY                                              # once on boot
POS <id> <steps>                                   # periodic, monitoring only
HOMED <id>                                         # sensor crossed, position reset
FAULT <id> <code>                                  # TIMEOUT
STATUS <id> <actual> <ideal> <homed> <homing> <integral>  # response to STATUS
PID_UPDATED                                        # confirmation after SETPID
```

---

## Part 2: TouchDesigner Python Extensions

TouchDesigner version: **2023.x or later**. All extensions use the standard TD Python Extension pattern — attached to a Base COMP, accessed via `op('base_name').ext.ExtensionName`.

### File: `touchdesigner/extensions/SerialEXT.py`

Manages raw serial communication with one Arduino Mega. Two instances — one for LEFT box (motors 0–6), one for RIGHT box (motors 7–13).

#### Custom Parameters on COMP
- `Serialport` — COM port string e.g. `COM3`
- `Baudrate` — 115200
- `Motoroffset` — `0` for LEFT, `7` for RIGHT

#### Key Methods

```python
class SerialEXT:
    def __init__(self, ownerComp):
        self.ownerComp = ownerComp
        self.motor_offset = ownerComp.par.Motoroffset.eval()
        self.rx_buffer = ""
        self.connected = False

    def Connect(self):
        """Open serial port."""

    def Disconnect(self):
        """Close serial port cleanly."""

    def onSerialReceive(self, dat, rowIndex, cellIndex, prev):
        """
        Called by TD Serial DAT callback (Line mode).
        Parses one complete line per call.
        Dispatches to MotorControllerEXT.
        """

    def ParseLine(self, line):
        """
        Dispatch table:
          'POS'    → MotorControllerEXT.UpdateActualPos(global_id, steps)
          'HOMED'  → MotorControllerEXT.OnHomed(global_id)
          'FAULT'  → MotorControllerEXT.OnFault(global_id, code)
          'READY'  → log connection confirmed
          'STATUS' → MotorControllerEXT.OnStatusReport(data)
        global_id = local_id + self.motor_offset
        """

    def SendSetPos(self, global_motor_id, ideal_steps):
        """Send: SETPOS <local_id> <steps>\n"""

    def SendHome(self, global_motor_id):
        """Send: HOME <local_id>\n"""

    def SendHomeAll(self):
        """Send: HOMEALL\n"""

    def SendStop(self, global_motor_id):
        """Send: STOP <local_id>\n"""

    def SendStopAll(self):
        """Send: STOPALL\n"""

    def SendEnable(self, global_motor_id):
        """Send: ENABLE <local_id>\n"""

    def SendDisable(self, global_motor_id):
        """Send: DISABLE <local_id>\n"""

    def SendSetPID(self, kp, ki, kd):
        """Send: SETPID <kp> <ki> <kd>\n — applies to all motors on this Mega"""

    def RequestStatus(self):
        """Send: STATUS\n"""
```

---

### File: `touchdesigner/extensions/MotorControllerEXT.py`

Central state manager for all 14 motors. **Does not run PID** — that lives on the Arduino. Holds ideal positions, streams them to the Megas, receives actual positions for monitoring, and provides safety fallbacks.

#### State Model

```python
NUM_MOTORS = 14

self.ideal_pos   = [0] * NUM_MOTORS   # steps — set by ChoreographyEXT, sent to Mega
self.actual_pos  = [0] * NUM_MOTORS   # steps — received from Mega (monitoring only)
self.homed       = [False] * NUM_MOTORS
self.homing      = [False] * NUM_MOTORS
self.faults      = [None] * NUM_MOTORS
```

#### Custom Parameters on COMP

```
Autorehomedrift     int     default: 200   # steps of drift that triggers TD-side re-home command

# PID display parameters — read-only, reflect last values sent to Arduino via SETPID
# These are NOT the control source — they mirror what was last sent
Pid_kp              float   default: 0.8
Pid_ki              float   default: 0.01
Pid_kd              float   default: 0.1
```

> **PID parameters are not the control source.** Kp/Ki/Kd are owned by the Arduino firmware. These TD parameters mirror the last values sent via `SETPID` and are used for display in the tuning panel. The Arduino's actual gains only change when a `SETPID` serial command is received.

#### Key Methods

```python
def SetIdealPos(self, motor_id, steps):
    """
    Called by ChoreographyEXT every cook to update ideal position.
    Stores value locally.
    Sends SETPOS to the appropriate Mega via SerialEXT.
    Only sends if value has changed (avoid flooding serial with identical values).
    """

def UpdateActualPos(self, motor_id, steps):
    """
    Called by SerialEXT when POS message received.
    Updates self.actual_pos[motor_id].
    Updates motor_state_table DAT for monitoring UI.
    Checks if abs(ideal - actual) > Autorehomedrift — if so, sends HOME as safety fallback.
    Does NOT run PID — that is Arduino's job.
    """

def OnHomed(self, motor_id):
    """
    Called by SerialEXT when HOMED received.
    Sets self.homed[motor_id] = True.
    Resets self.actual_pos[motor_id] = 0.
    Resets self.ideal_pos[motor_id] = 0 (resync choreography reference).
    Logs event with timestamp.
    Updates monitoring table.
    """

def OnFault(self, motor_id, code):
    """
    Records fault. Logs. Flags for operator attention.
    On TIMEOUT: mark motor not homed.
    """

def OnStatusReport(self, motor_id, actual, ideal, homed, homing, integral):
    """Full state sync. Update all local state from Mega STATUS response."""

def HomeAll(self):
    """Send HOMEALL to both Megas. Reset all homed flags locally."""

def HomeMotor(self, motor_id):
    """Send HOME for one motor."""

def StopAll(self):
    """Emergency stop — STOPALL to both Megas."""

def EnableAll(self):
    """Enable all driver outputs."""

def DisableAll(self):
    """Disable all driver outputs."""

def SetPIDGains(self, kp, ki, kd, motor_id=None):
    """
    Send SETPID to both Megas simultaneously (or one motor if motor_id specified).
    Stores last-sent values locally so the TD panel can display them.
    Allows live PID tuning without reflashing Arduino.

    If motor_id is None: sends SETPID <kp> <ki> <kd> to both Megas (applies to all 14 motors).
    If motor_id is given: sends SETPID <local_id> <kp> <ki> <kd> to the correct Mega only.

    Also updates self.pid_kp, self.pid_ki, self.pid_kd for UI display.
    Logs the update with timestamp.
    """

def GetPIDGains(self):
    """Return (kp, ki, kd) currently stored — for panel display."""

def ResetPIDState(self, motor_id=None):
    """
    Send SETPID with current gains to force Arduino to reset integral/derivative state.
    Use after a re-home or when a motor has been manually moved.
    If motor_id is None, resets all motors.
    """

def GetSerialEXT(self, motor_id):
    """Route command to correct Mega based on motor_id."""
    if motor_id < 7:
        return op('serial_left').ext.SerialEXT
    else:
        return op('serial_right').ext.SerialEXT

def Startup(self):
    """
    Full startup sequence:
    1. Connect both serial ports
    2. Wait for READY from both Megas (timeout 5s)
    3. DisableAll()
    4. Wait 500ms
    5. EnableAll()
    6. HomeAll()
    7. Wait for all 14 HOMED messages (timeout 30s)
    8. Log 'System ready'
    9. Activate default cue via ChoreographyEXT
    """
```

#### Monitoring Table

`motor_state_table` — Table DAT, 15 rows (header + 14 motors), updated on every `UpdateActualPos()` call:

```
motor_id | ideal_pos | actual_pos | error | homed | homing | fault
```

`error = ideal_pos - actual_pos` — computed in TD, reflects Arduino PID performance. Drive a Table TOP from this for the operator UI.

> **Note:** There is no `pid_integral` column here — that lives on the Arduino. The STATUS command can pull it for diagnostics but it is not in the live monitoring table.

---

### File: `touchdesigner/extensions/ChoreographyEXT.py`

Computes ideal motor positions each cook and pushes them to `MotorControllerEXT`. Runs on a CHOP Execute DAT at TD cook rate (typically 60fps).

#### Design

Each cook:
1. Read current show parameters
2. Compute `ideal_pos[i]` for each motor (0–13)
3. Call `MotorControllerEXT.SetIdealPos(i, pos)` for each motor

#### Custom Parameters

```
Waveamplitude     int     steps — peak displacement from home
Wavefrequency     float   Hz
Wavephaseoffset   float   radians — spatial phase between adjacent motors
Wavemode          menu    SINE | TRIANGLE | CUSTOM
Playback          toggle  enable/disable output
Activecue         int     current cue index
```

#### Wave Computation

```python
def ComputeWavePos(self, motor_index, t):
    spatial_phase = motor_index * self.ownerComp.par.Wavephaseoffset.eval()
    amplitude     = self.ownerComp.par.Waveamplitude.eval()
    frequency     = self.ownerComp.par.Wavefrequency.eval()
    normalized    = math.sin(2 * math.pi * frequency * t + spatial_phase)
    return int(normalized * amplitude)
```

#### Cue System

```python
cues = [
    {'name': 'HOME',        'type': 'home'},
    {'name': 'FREEZE',      'type': 'hold'},
    {'name': 'WAVE_SLOW',   'type': 'wave', 'amplitude': 800,  'frequency': 0.2, 'phase': 0.4},
    {'name': 'WAVE_FAST',   'type': 'wave', 'amplitude': 400,  'frequency': 0.8, 'phase': 0.6},
    {'name': 'UNISON_UP',   'type': 'absolute', 'positions': [800]*14},
    {'name': 'UNISON_DOWN', 'type': 'absolute', 'positions': [-800]*14},
]

def GoCue(self, cue_index): ...
def NextCue(self): ...
def PrevCue(self): ...
```

---

### File: `touchdesigner/extensions/OSCHandlerEXT.py`

Receives OSC from FOH laptop, translates to ChoreographyEXT / MotorControllerEXT calls.

#### OSC Address Map

```
/motor/cue          i   cue_index           → ChoreographyEXT.GoCue(i)
/motor/next                                 → ChoreographyEXT.NextCue()
/motor/prev                                 → ChoreographyEXT.PrevCue()
/motor/wave/amp     f   amplitude_steps     → set wave amplitude
/motor/wave/freq    f   frequency_hz        → set wave frequency
/motor/wave/phase   f   phase_offset_rad    → set spatial phase offset
/motor/homeall                              → MotorControllerEXT.HomeAll()
/motor/stopall                              → MotorControllerEXT.StopAll()
/motor/enable                               → MotorControllerEXT.EnableAll()
/motor/disable                              → MotorControllerEXT.DisableAll()
/motor/pid/kp           f       value               → update Kp for all motors, send SETPID to both Megas
/motor/pid/ki           f       value               → update Ki for all motors
/motor/pid/kd           f       value               → update Kd for all motors
/motor/pid/set          f f f   kp ki kd            → set all three gains in one message
/motor/pid/motor        i f f f motor_id kp ki kd   → per-motor gain update
/motor/pid/reset                                    → MotorControllerEXT.ResetPIDState() — send RESETPID to both Megas
/motor/setpos           i i     motor_id steps      → MotorControllerEXT.SetIdealPos(id, steps)
```

> **Note:** All `/motor/pid/*` commands route through `MotorControllerEXT` which forwards `SETPID` or `RESETPID` to the correct Mega(s) over serial. PID tuning is fully live from FOH without reflashing the Arduino.

---

### File: `touchdesigner/extensions/PIDTuningEXT.py`

Dedicated extension for the PID tuning panel. Separates tuning UI logic from `MotorControllerEXT` to keep concerns clean. Attached to a `base_pid_tuning` COMP that contains the panel UI.

#### Purpose

Provides a live tuning interface that:
- Displays current Kp/Ki/Kd values (last sent to Arduino)
- Lets the operator change gains and send them immediately via serial
- Shows per-motor error in real time to judge whether tuning is working
- Provides preset gain profiles for quick switching
- Logs all gain changes with timestamps for post-show analysis

#### Custom Parameters on COMP (the tuning panel controls)

```
Kp              float   default: 0.8    — global Kp slider
Ki              float   default: 0.01   — global Ki slider
Kd              float   default: 0.1    — global Kd slider
Selectedmotor   int     -1 to 13        — -1 = all motors, 0-13 = single motor
Applymode       menu    GLOBAL | PERMOTOR
Autosend        toggle  if ON, sends SETPID on every parameter change (live knob mode)
                        if OFF, requires explicit SendGains() call (safe mode)
```

#### Preset Profiles

Store as a dict of named gain sets. Operator can select a preset and apply it instantly:

```python
presets = {
    'default':      {'kp': 0.8,  'ki': 0.01,  'kd': 0.1},
    'aggressive':   {'kp': 1.5,  'ki': 0.02,  'kd': 0.05},
    'gentle':       {'kp': 0.3,  'ki': 0.005, 'kd': 0.2},
    'p_only':       {'kp': 0.8,  'ki': 0.0,   'kd': 0.0},
    'high_damping': {'kp': 0.6,  'ki': 0.01,  'kd': 0.4},
}
```

#### Key Methods

```python
def SendGains(self):
    """
    Read Kp/Ki/Kd from COMP parameters.
    If Applymode == GLOBAL or Selectedmotor == -1:
        call MotorControllerEXT.SetPIDGains(kp, ki, kd)  → SETPID to both Megas
    If Applymode == PERMOTOR and Selectedmotor >= 0:
        call MotorControllerEXT.SetPIDGains(kp, ki, kd, motor_id=Selectedmotor)
    Log: "PID gains sent: Kp={kp} Ki={ki} Kd={kd} [motor={id or 'all'}]"
    """

def ApplyPreset(self, preset_name):
    """
    Load preset gains into COMP parameters.
    If Autosend is ON, call SendGains() immediately.
    """

def ResetPIDState(self):
    """
    Send RESETPID to both Megas via MotorControllerEXT.ResetPIDState().
    Does not change gains — only clears integral/derivative accumulators.
    Use after a re-home or after manually repositioning a motor.
    Log: "PID state reset — integral and derivative cleared"
    """

def onParValueChange(self, par, prev):
    """
    Called when any COMP parameter changes.
    If Autosend is ON and par.name in ['Kp', 'Ki', 'Kd']:
        call SendGains()
    """

def GetErrorSummary(self):
    """
    Pull actual_pos and ideal_pos from MotorControllerEXT for all 14 motors.
    Return dict: {'max_error': int, 'mean_error': float, 'worst_motor': int}
    Used to display tuning quality in the panel.
    """

def LogGainChange(self, kp, ki, kd, motor_id, source):
    """
    Append to a log DAT inside the COMP:
    timestamp | kp | ki | kd | motor_id | source (PANEL/OSC/PRESET)
    Useful for post-show analysis and repeatability.
    """
```

#### TD Panel UI Elements to Build

Implement as a Container COMP with the following controls wired to `PIDTuningEXT`:

```
[Kp slider 0.0–3.0]         [Ki slider 0.0–0.1]         [Kd slider 0.0–1.0]
[Kp numeric field]           [Ki numeric field]           [Kd numeric field]

[Motor selector: ALL | 0 | 1 | 2 ... 13]
[Apply mode: GLOBAL / PER-MOTOR]
[Auto-send toggle]

[SEND GAINS button]          [RESET PID STATE button]

[Preset selector dropdown]   [APPLY PRESET button]

[Error monitor table — motor_id | ideal | actual | error — live 20Hz refresh]
[Max error display]          [Worst motor display]

[Gain change log DAT — scrolling, last 50 entries]
```

#### Panel Wiring in TD

- Sliders and numeric fields: bind to `ownerComp.par.Kp`, `par.Ki`, `par.Kd`
- SEND GAINS button: `op('base_pid_tuning').ext.PIDTuningEXT.SendGains()`
- RESET PID STATE button: `op('base_pid_tuning').ext.PIDTuningEXT.ResetPIDState()`
- Error monitor: Table DAT refreshed from `MotorControllerEXT.GetStateTable()` at 20Hz via a Timer CHOP execute
- Auto-send: implement via `onParValueChange` callback on the COMP

---

## Part 3: TouchDesigner Network Setup

### Operator Tree

```
/project1
├── base_motor_system
│   ├── base_serial_left          ← SerialEXT (Motoroffset=0)
│   │   ├── serial_left           (Serial DAT, Line mode, COM port LEFT Mega)
│   │   └── serial_callback       (DAT Execute → SerialEXT.onSerialReceive)
│   │
│   ├── base_serial_right         ← SerialEXT (Motoroffset=7)
│   │   ├── serial_right          (Serial DAT, Line mode, COM port RIGHT Mega)
│   │   └── serial_callback
│   │
│   ├── base_motor_controller     ← MotorControllerEXT
│   │   ├── motor_state_table     (Table DAT — 15 rows × 7 cols, live monitoring)
│   │   └── custom params         (Autorehomedrift)
│   │
│   ├── base_choreography         ← ChoreographyEXT
│   │   └── chop_execute          (CHOP Execute DAT — runs every cook)
│   │
│   ├── base_pid_tuning           ← PIDTuningEXT
│   │   ├── pid_panel             (Container COMP — tuning UI)
│   │   ├── pid_log               (Table DAT — gain change history)
│   │   └── error_monitor         (Table DAT — live error per motor, 20Hz)
│   │
│   └── base_osc_handler          ← OSCHandlerEXT
│       ├── osc_in                (OSC In DAT — port 7000)
│       └── osc_callback          (DAT Execute → OSCHandlerEXT.OnOSCMessage)
```

### Startup Procedure

Implement as `MotorControllerEXT.Startup()` — callable from a TD panel button on show launch:

```
1. Connect both serial ports
2. Wait for READY from both Megas (timeout 5s, log error if not received)
3. DisableAll — drivers off
4. Wait 500ms
5. EnableAll — drivers on
6. HomeAll — both Megas start homing all 14 motors simultaneously
7. Wait for 14× HOMED messages (timeout 30s per motor)
8. Log "System ready — all motors homed"
9. ChoreographyEXT.GoCue(0)  →  FREEZE or idle wave
```

---

## Part 4: Protocol Reference

### TD → Mega (ASCII, newline terminated)

| Command | Format | Description |
|---|---|---|
| SETPOS | `SETPOS <id> <steps>` | Set PID ideal target for one motor. Arduino drives to this position. |
| HOME | `HOME <id>` | Home single motor |
| HOMEALL | `HOMEALL` | Home all 7 motors simultaneously |
| STOP | `STOP <id>` | Stop single motor |
| STOPALL | `STOPALL` | Emergency stop all motors |
| ENABLE | `ENABLE <id>` | Enable driver output |
| DISABLE | `DISABLE <id>` | Disable driver output |
| SETPID | `SETPID <Kp> <Ki> <Kd>` | Update PID gains for all motors. Resets integral/derivative state. |
| SETPID | `SETPID <id> <Kp> <Ki> <Kd>` | Update PID gains for one motor only. |
| RESETPID | `RESETPID` | Reset integral and derivative state without changing gains. |
| STATUS | `STATUS` | Request full status for all 7 motors |

### Mega → TD (ASCII, newline terminated)

| Message | Format | Description |
|---|---|---|
| READY | `READY` | Sent once on boot |
| POS | `POS <id> <steps>` | Periodic position report — monitoring only |
| HOMED | `HOMED <id>` | Sensor crossed, position and ideal reset to 0 |
| FAULT | `FAULT <id> <code>` | Error. Codes: `TIMEOUT` |
| STATUS | `STATUS <id> <actual> <ideal> <homed> <homing> <integral>` | Response to STATUS query |
| PID_UPDATED | `PID_UPDATED` | Confirmation after global SETPID received |
| PID_UPDATED | `PID_UPDATED <id>` | Confirmation after per-motor SETPID received |
| PID_RESET | `PID_RESET` | Confirmation after RESETPID received |

### Parameter Types
- `<id>`: integer 0–6 (local to this Mega — SerialEXT translates to/from global 0–13)
- `<steps>`: int32, signed, steps from home (positive = forward from home)
- `<homed>`: 0 or 1
- `<homing>`: 0 or 1
- `<integral>`: float, current PID integral accumulator value

---

## Part 5: PID Tuning Guide

The PID runs on the Arduino at the step loop rate. It is a **position correction loop**, not a high-speed servo. Expected error magnitude at steady state: 0–20 steps. Expected correction timescale: sub-second.

### Default Starting Values (in firmware constants)
```
Kp = 0.8        proportional — primary driver of correction
Ki = 0.01       integral — eliminates steady-state offset
Kd = 0.1        derivative — damps overshoot
Integral clamp = 500 steps
PID dead band = 5 steps
PID max speed = 800 steps/sec (gentle — invisible to audience)
Auto re-home threshold (TD) = 200 steps
```

### How to Send Gains Without Reflashing

Three routes — all equivalent, all take effect immediately on the Arduino:

**1. TD Tuning Panel (recommended during commissioning)**
Open `base_pid_tuning` panel. Adjust Kp/Ki/Kd sliders. Press SEND GAINS or enable Auto-send for live knob mode. Error monitor shows correction quality in real time.

**2. Serial command directly (raw debugging)**
Open the TD serial monitor DAT or any terminal connected to the Mega's COM port:
```
SETPID 1.2 0.01 0.1       ← applies to all 7 motors on this Mega
SETPID 3 1.2 0.01 0.1     ← applies to motor 3 only
RESETPID                   ← clear integral/derivative without changing gains
STATUS                     ← read back current state including integral value
```

**3. OSC from FOH (during show, remote adjustment)**
```
/motor/pid/set  1.2  0.01  0.1          ← all motors
/motor/pid/motor  3  1.2  0.01  0.1     ← motor 3 only
/motor/pid/reset                        ← reset state
```

### Step-by-Step Tuning Procedure

**Phase 1 — Proportional only**
```
SETPID 0.0 0.0 0.0   ← zero everything
SETPID 0.3 0.0 0.0   ← start low
```
Run wave motion. Watch error column. Increase Kp in steps of 0.2 until motors correct drift crisply. Stop before oscillation appears.

**Phase 2 — Add integral**
```
SETPID <kp> 0.005 0.0
```
Watch for steady-state offset (motor rests a few steps away from ideal). Increase Ki in steps of 0.005 until offset disappears. Stay below 0.05 — high Ki causes windup.

**Phase 3 — Add derivative if needed**
Only add Kd if corrections visibly overshoot. Start at 0.05, increase in steps of 0.05:
```
SETPID <kp> <ki> 0.05
```

**Phase 4 — Reset and verify**
After settling on gains, send `RESETPID` to clear any accumulated integral from the tuning process, then let the system run for 60 seconds and check max error stays below 10 steps.

**Phase 5 — Save to firmware**
Once happy, update `Kp`, `Ki`, `Kd` constants in `motor_controller.ino` and reflash so the tuned values survive a power cycle.

### Symptoms and Fixes

| Symptom | Cause | Fix |
|---|---|---|
| Motors oscillate around target | Kp too high | Reduce Kp |
| Correction too slow | Kp too low | Increase Kp |
| Steady-state offset remains | Ki too low | Increase Ki |
| Growing overcorrection over time | Ki too high or clamp too small | Reduce Ki or tighten `INTEGRAL_CLAMP` |
| Motor chatter / rapid small steps | Dead band too small | Increase `PID_DEADBAND` in firmware and reflash |
| Corrections visible as audience-noticeable jerks | `PID_MAX_SPEED` too high | Reduce in firmware and reflash |
| Jerk on gain change | Old integral carried over | Send `RESETPID` after every gain change during tuning |
| One motor behaves differently from others | Mechanical friction difference | Use per-motor SETPID to tune that motor independently |

---

## Part 6: Key Implementation Notes for Claude Code

1. **No `delay()` on the Arduino** — ever. `stepper[i].run()` or `stepper[i].runSpeed()` must execute every loop iteration without interruption. Any blocking call causes missed steps.

2. **Serial parsing on Arduino** — read character by character into a `String` buffer, parse on `'\n'`. Do not use `Serial.readString()` — it blocks for its full timeout period.

3. **ISR safety** — no `Serial` calls, no `malloc`, no `String` operations inside ISRs. Only write to `volatile` primitives and set `volatile bool` flags. Handle all messaging in `loop()`.

4. **AccelStepper modes during homing** — use `stepper.setSpeed()` + `stepper.runSpeed()` during homing (constant speed, no target). Switch back to `stepper.moveTo()` + `stepper.run()` after homing completes. The loop() switches modes based on `homing_active[i]`.

5. **PID feeds AccelStepper, not pins directly** — the PID output sets `stepper[i].moveTo(corrected_target)`. AccelStepper handles acceleration and step timing. Never toggle STEP pins manually.

6. **SETPOS vs PID interaction** — when a new `SETPOS` arrives from a choreography cue, briefly allow `MAX_SPEED` for the initial move so the motor reaches its new target quickly. The PID then takes over at `PID_MAX_SPEED` for fine correction. Do not let the PID fight a large choreography move at correction speed.

7. **TD extension init** — TD calls `__init__` before the network is fully cooked. Defer any operator access using `run("op('x').method()", delayFrames=1)` if needed.

8. **Serial DAT in TD** — set to `Line` mode so each callback fires on one newline-terminated message. Set `Active` to 1 and `Port` to the correct COM string.

9. **Motor ID mapping** — Arduino always uses local IDs 0–6. TD always uses global IDs 0–13. `SerialEXT.motor_offset` (0 for LEFT, 7 for RIGHT) translates in both directions. Never pass global IDs directly to the Mega.

10. **SETPOS rate limiting** — ChoreographyEXT runs at 60fps and may compute identical ideal_pos values many consecutive frames. `SetIdealPos()` in `MotorControllerEXT` should only send `SETPOS` when the value has actually changed, to avoid flooding serial with redundant commands.

13. **Send RESETPID after re-home** — when a motor homes mid-show (crossing θ=0 during wave motion), the integral accumulator may hold residual error from before the crossing. `OnHomed()` in `MotorControllerEXT` should call `ResetPIDState(motor_id)` automatically to prevent a correction jerk after the home event.

14. **SETPID resets integral/derivative** — the Arduino firmware resets PID state on every `SETPID` command. This is intentional — it prevents old accumulated integral from causing a jerk when gains change. The TD panel does not need to send a separate `RESETPID` after `SETPID`.

15. **Auto-send mode caution** — `PIDTuningEXT.Autosend` is convenient but can cause rapid repeated `SETPID` commands while a slider is being dragged. Rate-limit outgoing SETPID messages to no faster than one per 100ms when in auto-send mode.

11. **Step direction** — `HOMING_DIR` must be set correctly for the physical installation. If a motor moves away from its sensor on HOME, flip the sign. Verify before enabling all 14 motors.

12. **First run checklist:**
    - Verify DM860 microstepping DIP switch setting on all drivers — must match `steps/rev` assumption in TD choreography math
    - Test one motor in isolation before enabling all 14
    - Manually trigger each homing sensor (hand-press the switch) to verify ISR fires, HOMED message arrives in TD, and position resets to 0
    - Watch `error` column in monitoring table at rest — if it's noisy, increase `PID_DEADBAND`
    - Run `SETPID 0 0 0` to zero all gains and verify motors hold position open-loop before enabling PID

---

## Part 7: Testing Sequence

Implement as callable methods on `MotorControllerEXT` or TD panel buttons:

```python
def TestSingleMotor(self, motor_id):
    """
    Send SETPOS <id> 200 to one motor.
    Wait 3s. Send SETPOS <id> 0.
    Watch actual_pos in monitoring table.
    Pass if motor moves and returns cleanly.
    """

def TestAllSensors(self):
    """
    Send STATUS to both Megas.
    Print homed state for all 14 motors.
    Pass if all 14 report correctly.
    """

def TestPIDTracking(self, motor_id):
    """
    Set ideal_pos to +100 steps via SETPOS.
    Poll actual_pos via POS messages for 5s.
    Log error over time.
    Pass if error converges to < PID_DEADBAND within 2s.
    """

def RunWaveTest(self, duration_seconds=10):
    """
    Run slow sine wave (0.2Hz, amplitude 400 steps) on all 14 motors.
    Log max absolute error across all motors over duration.
    Pass if max error stays below 20 steps during steady motion.
    """

def TestHomingSequence(self):
    """
    Send HOMEALL.
    Wait for 14× HOMED messages.
    Verify all actual_pos report 0 after homing.
    Log time taken per motor.
    """
```

---

*Document version: 2.1 | PID on Arduino | Live serial tuning via SETPID/RESETPID | PIDTuningEXT panel*
*Hardware: 14× NEMA 34 | 2× Arduino Mega 2560 | Mini PC running TouchDesigner*
