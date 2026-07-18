// Festival Stage Wave Motor Control — Arduino Mega 2560 firmware (NO PID)
// One sketch, flashed to both the LEFT and RIGHT Megas (7 motors each).
//
// This is the PID-FREE variant of motor_controller.ino. Motors are driven
// directly by the commands TD sends -- no closed-loop position correction:
//   SETPOS   -> AccelStepper moveTo() + run(), using built-in trapezoidal
//               acceleration toward the target. That's it, no PID trim.
//   SETSPEED -> continuous rotation at a fixed speed via runSpeed().
// Position accuracy over long runs comes instead from the homing sensor:
// during continuous rotation the count is resynced to the true reference
// once per revolution (resyncContinuousPosition), same as the PID build.
//
// New in this variant: TD can set max speed and acceleration at runtime
// via SETMAXSPEED / SETACCEL (per-motor or all), so those no longer require
// a reflash to tune.
//
// Wire protocol is otherwise IDENTICAL to motor_controller.ino so the same
// TD side works unchanged, EXCEPT: this firmware never sends PID_UPDATED/
// PID_RESET, ignores SETPID/RESETPID (they'll come back as "ERR unknown
// command"), and the 6th field of STATUS -- which was pid_integral -- is
// always 0.0000 here (TD parses but discards that field, so this stays a
// drop-in replacement).

#include <AccelStepper.h>
#include <string.h>
#include <stdlib.h>

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

#define NUM_MOTORS 7

#define REPORT_INTERVAL_MS 50      // monitoring position reports
#define HOMING_SPEED 400           // steps/sec — slow and safe
#define HOMING_TIMEOUT_MS 15000
#define BAUD_RATE 115200

#define REPORT_DEADBAND 2          // steps — suppress POS messages for tiny changes

// ============================================================================
// !!! HOMING_DIR MUST BE VERIFIED PER MEGA BEFORE FLASHING !!!
// LEFT and RIGHT motor groups are physically mirrored. The same sign here
// can mean "toward home" on one side and "away from home" on the other.
// Flash, jog/hand-trigger each sensor to confirm every motor homes toward
// its switch, THEN re-check -- possibly with the opposite sign -- before
// flashing the other Mega.
// ============================================================================
#define HOMING_DIR -1

// Startup DEFAULTS for max speed / acceleration. Both are now adjustable at
// runtime from TD via SETMAXSPEED / SETACCEL without reflashing -- these are
// just the values used until TD changes them.
#define DEFAULT_MAX_SPEED 4000     // steps/sec
#define DEFAULT_ACCEL 800          // steps/sec^2

// Steps per full revolution -- MUST match the DM860I driver's microstepping
// DIP switch (e.g. 800 or 1600). Used only for continuous-rotation drift
// resync against the reference sensor.
#define STEPS_PER_REV 1600

// ---------------------------------------------------------------------------
// Pin assignments
// ---------------------------------------------------------------------------

const int STEP_PIN[NUM_MOTORS] = {30, 33, 36, 39, 42, 45, 48};
const int DIR_PIN[NUM_MOTORS]  = {31, 34, 37, 40, 43, 46, 49};
const int EN_PIN[NUM_MOTORS]   = {32, 35, 38, 41, 44, 47, 50};

// All motors polled in loop() via pollSensors() -- no ISRs. Active HIGH:
// digitalRead(SENSOR_PIN[i]) == HIGH means triggered.
const int SENSOR_PIN[NUM_MOTORS] = {22, 23, 24, 25, 26, 27, 28};

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

AccelStepper stepper[NUM_MOTORS];

volatile int32_t actual_pos[NUM_MOTORS];  // steps from home — access via readActualPos()/writeActualPos()
int32_t ideal_pos[NUM_MOTORS];            // target set by TD via SETPOS
volatile bool homed[NUM_MOTORS];          // true after first successful home
volatile bool homing_active[NUM_MOTORS];  // currently executing home sequence
unsigned long homing_start_time[NUM_MOTORS];

// Emergency-stop latch. Set by STOPALL; cleared by SETPOS/SETSPEED/HOME/
// HOMEALL/ENABLE for that motor.
bool estopped[NUM_MOTORS];

// Continuous-speed mode (SETSPEED). Mutually exclusive with position mode.
bool speed_mode[NUM_MOTORS];
long commanded_speed[NUM_MOTORS];  // steps/sec, signed, as given to SETSPEED

// Per-motor max speed / acceleration, runtime-adjustable via SETMAXSPEED /
// SETACCEL. max_speed caps BOTH SETPOS-move speed and SETSPEED continuous
// speed (AccelStepper's setSpeed() is clamped to setMaxSpeed()).
long motor_max_speed[NUM_MOTORS];
long motor_accel[NUM_MOTORS];

// Previous raw sensor reading per motor, for direction-aware edge detection.
bool last_sensor_state[NUM_MOTORS];

volatile bool homed_flag[NUM_MOTORS];     // set by pollSensors(), reported in loop()

int32_t last_reported_pos[NUM_MOTORS];
unsigned long last_report_time = 0;

bool status_pending = false;
uint8_t status_next_motor = 0;

String input_buffer = "";

// ---------------------------------------------------------------------------
// Interrupt-safe access to actual_pos[] (no ISRs write it anymore, but the
// guards are harmless and cheap, kept for parity with the PID build).
// ---------------------------------------------------------------------------

int32_t readActualPos(int i) {
    int32_t v;
    noInterrupts();
    v = actual_pos[i];
    interrupts();
    return v;
}

void writeActualPos(int i, int32_t v) {
    noInterrupts();
    actual_pos[i] = v;
    interrupts();
}

// ---------------------------------------------------------------------------
// Homing / sensor
// ---------------------------------------------------------------------------

void resetHomePosition(int i) {
    writeActualPos(i, 0);
    stepper[i].setCurrentPosition(0);
    ideal_pos[i] = 0;          // resync choreography reference to home
    homed[i] = false;
    homing_active[i] = false;
    homed_flag[i] = true;      // loop() sends HOMED message
}

// Periodic drift correction during continuous rotation (speed_mode). Snaps
// the running step count onto the nearest true multiple of STEPS_PER_REV,
// bounding worst-case error to under one revolution. Does not exit
// speed_mode. This is the accuracy mechanism that replaces PID here.
void resyncContinuousPosition(int i) {
    int32_t raw_pos = stepper[i].currentPosition();
    int32_t remainder = raw_pos % STEPS_PER_REV;
    if (remainder < 0) remainder += STEPS_PER_REV;  // C++ % keeps the dividend's sign
    int32_t error = (remainder > STEPS_PER_REV / 2) ? (remainder - STEPS_PER_REV) : remainder;

    int32_t corrected = raw_pos - error;
    writeActualPos(i, corrected);
    stepper[i].setCurrentPosition(corrected);

    Serial.print("RESYNC ");
    Serial.print(i);
    Serial.print(' ');
    Serial.println(error);
}

// Direction-aware reference edge: the sensor has physical width, so
// position 0 is the rising edge when moving forward and the falling edge
// when moving reverse -- using one edge for both directions would resync to
// two different physical angles. Direction from HOMING_DIR while homing, or
// the sign of commanded_speed[i] in continuous mode.
bool checkReferenceEdge(int i) {
    bool state = digitalRead(SENSOR_PIN[i]);  // active HIGH
    bool prev = last_sensor_state[i];
    last_sensor_state[i] = state;

    bool moving_forward;
    if (homing_active[i]) {
        moving_forward = (HOMING_DIR > 0);
    } else if (speed_mode[i]) {
        moving_forward = (commanded_speed[i] > 0);
    } else {
        return false;
    }

    if (moving_forward) {
        return (prev == LOW && state == HIGH);   // rising edge
    } else {
        return (prev == HIGH && state == LOW);    // falling edge
    }
}

void pollSensors() {
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (!checkReferenceEdge(i)) continue;

        if (homing_active[i]) {
            resetHomePosition(i);          // one-time calibration
        } else {
            resyncContinuousPosition(i);   // periodic drift correction
        }
    }
}

void checkHomedFlags() {
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (homed_flag[i]) {
            homed_flag[i] = false;
            Serial.print("HOMED ");
            Serial.println(i);
        }
    }
}

void checkHomingTimeouts() {
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (homing_active[i] && (millis() - homing_start_time[i] > HOMING_TIMEOUT_MS)) {
            stepper[i].stop();
            homing_active[i] = false;
            Serial.print("FAULT ");
            Serial.print(i);
            Serial.println(" TIMEOUT");
        }
    }
}

// ---------------------------------------------------------------------------
// Command handlers
// ---------------------------------------------------------------------------

void cmdSetPos(int id, long steps) {
    if (id < 0 || id >= NUM_MOTORS) return;
    estopped[id] = false;
    speed_mode[id] = false;    // leave continuous-speed mode
    ideal_pos[id] = steps;
    stepper[id].moveTo(steps);
    // No PID -- AccelStepper's run() carries it to target using the motor's
    // current max speed / acceleration.
}

void cmdSetSpeed(int id, long speed) {
    if (id < 0 || id >= NUM_MOTORS) return;
    estopped[id] = false;
    speed_mode[id] = true;
    homing_active[id] = false;
    commanded_speed[id] = speed;
    stepper[id].setSpeed(speed);   // clamped to motor_max_speed[id]
}

void cmdHome(int id) {
    if (id < 0 || id >= NUM_MOTORS) return;
    estopped[id] = false;
    speed_mode[id] = false;
    homing_active[id] = true;
    homed[id] = true;
    homing_start_time[id] = millis();
    stepper[id].setSpeed(HOMING_SPEED * HOMING_DIR);
}

void cmdHomeAll() {
    for (int i = 0; i < NUM_MOTORS; i++) cmdHome(i);
}

void cmdStop(int id) {
    if (id < 0 || id >= NUM_MOTORS) return;
    if (speed_mode[id]) {
        stepper[id].setSpeed(0);   // runSpeed() won't run a decel profile
        commanded_speed[id] = 0;
    } else {
        stepper[id].stop();        // decelerate to stop — not latched
    }
    homing_active[id] = false;
}

void cmdStopAll() {
    for (int i = 0; i < NUM_MOTORS; i++) {
        int32_t pos = stepper[i].currentPosition();
        stepper[i].moveTo(pos);    // distanceToGo() = 0 — no further steps
        stepper[i].setSpeed(0);
        homing_active[i] = false;
        estopped[i] = true;        // latched until explicitly cleared
    }
}

void cmdEnable(int id) {
    if (id < 0 || id >= NUM_MOTORS) return;
    stepper[id].enableOutputs();
    estopped[id] = false;
}

void cmdDisable(int id) {
    if (id < 0 || id >= NUM_MOTORS) return;
    stepper[id].disableOutputs();
}

void cmdSetMaxSpeed(int id, long v) {
    if (id < 0 || id >= NUM_MOTORS || v <= 0) return;
    motor_max_speed[id] = v;
    stepper[id].setMaxSpeed(v);
    // If this motor is spinning, re-apply the commanded speed so a raised
    // ceiling takes effect immediately (setSpeed re-clamps to the new max).
    if (speed_mode[id]) stepper[id].setSpeed(commanded_speed[id]);
    Serial.print("MAXSPEED_UPDATED ");
    Serial.println(id);
}

void cmdSetMaxSpeedAll(long v) {
    if (v <= 0) return;
    for (int i = 0; i < NUM_MOTORS; i++) {
        motor_max_speed[i] = v;
        stepper[i].setMaxSpeed(v);
        if (speed_mode[i]) stepper[i].setSpeed(commanded_speed[i]);
    }
    Serial.println("MAXSPEED_UPDATED");
}

void cmdSetAccel(int id, long v) {
    if (id < 0 || id >= NUM_MOTORS || v <= 0) return;
    motor_accel[id] = v;
    stepper[id].setAcceleration(v);
    Serial.print("ACCEL_UPDATED ");
    Serial.println(id);
}

void cmdSetAccelAll(long v) {
    if (v <= 0) return;
    for (int i = 0; i < NUM_MOTORS; i++) {
        motor_accel[i] = v;
        stepper[i].setAcceleration(v);
    }
    Serial.println("ACCEL_UPDATED");
}

void cmdRequestStatus() {
    status_pending = true;
    status_next_motor = 0;
}

// ---------------------------------------------------------------------------
// Serial command parser
// ---------------------------------------------------------------------------

void handleCommand(String &line) {
    char buf[64];
    line.toCharArray(buf, sizeof(buf));

    char* cmd = strtok(buf, " ");
    if (cmd == NULL) return;

    if (strcmp(cmd, "SETPOS") == 0) {
        char* a1 = strtok(NULL, " ");
        char* a2 = strtok(NULL, " ");
        if (a1 && a2) cmdSetPos(atoi(a1), atol(a2));

    } else if (strcmp(cmd, "SETSPEED") == 0) {
        char* a1 = strtok(NULL, " ");
        char* a2 = strtok(NULL, " ");
        if (a1 && a2) cmdSetSpeed(atoi(a1), atol(a2));

    } else if (strcmp(cmd, "SETMAXSPEED") == 0) {
        // 2 args -> per-motor (id value); 1 arg -> all motors (value)
        char* a1 = strtok(NULL, " ");
        char* a2 = strtok(NULL, " ");
        if (a1 && a2)      cmdSetMaxSpeed(atoi(a1), atol(a2));
        else if (a1)       cmdSetMaxSpeedAll(atol(a1));

    } else if (strcmp(cmd, "SETACCEL") == 0) {
        // 2 args -> per-motor (id value); 1 arg -> all motors (value)
        char* a1 = strtok(NULL, " ");
        char* a2 = strtok(NULL, " ");
        if (a1 && a2)      cmdSetAccel(atoi(a1), atol(a2));
        else if (a1)       cmdSetAccelAll(atol(a1));

    } else if (strcmp(cmd, "HOME") == 0) {
        char* a1 = strtok(NULL, " ");
        if (a1) cmdHome(atoi(a1));

    } else if (strcmp(cmd, "HOMEALL") == 0) {
        cmdHomeAll();

    } else if (strcmp(cmd, "STOP") == 0) {
        char* a1 = strtok(NULL, " ");
        if (a1) cmdStop(atoi(a1));

    } else if (strcmp(cmd, "STOPALL") == 0) {
        cmdStopAll();

    } else if (strcmp(cmd, "ENABLE") == 0) {
        char* a1 = strtok(NULL, " ");
        if (a1) cmdEnable(atoi(a1));

    } else if (strcmp(cmd, "DISABLE") == 0) {
        char* a1 = strtok(NULL, " ");
        if (a1) cmdDisable(atoi(a1));

    } else if (strcmp(cmd, "STATUS") == 0) {
        cmdRequestStatus();

    } else {
        // Echo unknown commands rather than silently dropping. NOTE: SETPID
        // and RESETPID land here in this PID-free build -- expected.
        Serial.print("ERR unknown command: ");
        Serial.println(cmd);
    }
}

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
            if (input_buffer.length() > 64) input_buffer = "";
        }
    }
}

// ---------------------------------------------------------------------------
// Reporting
// ---------------------------------------------------------------------------

void sendPositionReports() {
    unsigned long now = millis();
    if (now - last_report_time < REPORT_INTERVAL_MS) return;
    last_report_time = now;

    for (int i = 0; i < NUM_MOTORS; i++) {
        int32_t pos = readActualPos(i);
        if (labs(pos - last_reported_pos[i]) >= REPORT_DEADBAND) {
            last_reported_pos[i] = pos;
            Serial.print("POS ");
            Serial.print(i);
            Serial.print(' ');
            Serial.println(pos);
        }
    }
}

// Same STATUS wire format as the PID build for TD compatibility. The 6th
// field was pid_integral there; here there is no PID, so it's always
// 0.0000 (TD parses but discards that field).
void sendStatusChunk() {
    if (!status_pending) return;

    int i = status_next_motor;
    Serial.print("STATUS ");
    Serial.print(i);
    Serial.print(' ');
    Serial.print(readActualPos(i));
    Serial.print(' ');
    Serial.print(ideal_pos[i]);
    Serial.print(' ');
    Serial.print(homed[i] ? 1 : 0);
    Serial.print(' ');
    Serial.print(homing_active[i] ? 1 : 0);
    Serial.print(' ');
    Serial.println("0.0000");   // was pid_integral in the PID build

    status_next_motor++;
    if (status_next_motor >= NUM_MOTORS) status_pending = false;
}

// ---------------------------------------------------------------------------
// Setup / loop
// ---------------------------------------------------------------------------

void setup() {
    Serial.begin(BAUD_RATE);

    for (int i = 0; i < NUM_MOTORS; i++) {
        stepper[i] = AccelStepper(AccelStepper::DRIVER, STEP_PIN[i], DIR_PIN[i]);

        motor_max_speed[i] = DEFAULT_MAX_SPEED;
        motor_accel[i]     = DEFAULT_ACCEL;
        stepper[i].setMaxSpeed(motor_max_speed[i]);
        stepper[i].setAcceleration(motor_accel[i]);
        stepper[i].setEnablePin(EN_PIN[i]);
        stepper[i].setPinsInverted(false, false, true);  // EN active LOW
        stepper[i].enableOutputs();

        // INPUT_PULLUP, active HIGH: unconnected channels read LOW (not
        // triggered), so bench testing with fewer motors wired times out on
        // the unconnected ones rather than reporting a false HOMED.
        pinMode(SENSOR_PIN[i], INPUT_PULLUP);
        last_sensor_state[i] = LOW;

        actual_pos[i]        = 0;
        ideal_pos[i]         = 0;
        last_reported_pos[i] = 0;
        homed[i]             = true;
        homing_active[i]     = false;
        homed_flag[i]        = false;
        estopped[i]          = false;
        speed_mode[i]        = false;
        commanded_speed[i]   = 0;
    }

    Serial.println("READY");
}

void loop() {
    // 1. Execute stepper motion (must run every loop, never blocked). No PID
    //    pass -- position moves ride AccelStepper's own acceleration.
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (homing_active[i] || speed_mode[i]) {
            stepper[i].runSpeed();   // constant speed -- homing or continuous rotation
        } else {
            stepper[i].run();        // acceleration-managed move toward a SETPOS target
        }
        writeActualPos(i, stepper[i].currentPosition());
    }

    // 2. Handle homed flags set by pollSensors()
    checkHomedFlags();

    // 3. Poll all 7 sensors (homing calibration + continuous-mode resync)
    pollSensors();

    // 4. Check homing timeouts
    checkHomingTimeouts();

    // 5. Periodic position reports to TD (monitoring)
    sendPositionReports();

    // 6. One STATUS line per iteration if a request is pending
    sendStatusChunk();

    // 7. Parse incoming serial commands from TD
    parseSerial();
}
