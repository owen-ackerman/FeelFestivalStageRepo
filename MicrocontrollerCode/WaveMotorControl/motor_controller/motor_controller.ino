// Festival Stage Wave Motor Control — Arduino Mega 2560 firmware
// One sketch, flashed to both the LEFT and RIGHT Megas (7 motors each).
// See motor_control_implementation_spec.md at the repo root for the full protocol.
//
// Architecture: TouchDesigner streams "ideal" target positions (SETPOS). This
// firmware runs a PID loop that continuously drives each motor's actual
// position toward its ideal position. Position reports sent back to TD are
// for monitoring only — TD does not close the control loop.

#include <AccelStepper.h>
#include <string.h>
#include <stdlib.h>

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

#define NUM_MOTORS 7

#define REPORT_INTERVAL_MS 50      // monitoring position reports, not control
#define HOMING_SPEED 400           // steps/sec — slow and safe
#define HOMING_TIMEOUT_MS 15000
#define BAUD_RATE 115200

#define REPORT_DEADBAND 2          // steps — suppress POS messages for tiny changes
#define PID_DEADBAND 5             // steps — error smaller than this is ignored by PID
#define INTEGRAL_CLAMP 500.0
#define REHOME_THRESHOLD 200       // steps — TD-side re-home trigger, not used here directly

// ============================================================================
// !!! HOMING_DIR MUST BE VERIFIED PER MEGA BEFORE FLASHING !!!
// LEFT and RIGHT motor groups are physically mirrored. The same sign here
// can easily mean "toward home" on one side and "away from home" on the
// other. Flash this file to one Mega, jog/hand-trigger each sensor to
// confirm every motor homes toward its switch (not away from it), THEN
// check again — possibly with the opposite sign — before flashing the
// other Mega. Do not assume the same value is correct for both.
// ============================================================================
#define HOMING_DIR -1

#define MAX_SPEED 4000             // steps/sec — choreography moves
#define ACCELERATION 800           // steps/sec^2
#define PID_MAX_SPEED 800          // steps/sec — gentle PID correction speed

// Above this error (steps), a choreography move (SETPOS) is still in
// progress and PID stands down rather than fighting it at PID_MAX_SPEED.
// Below it, PID takes over for fine correction. See runPID().
#define CHOREO_ERROR_THRESHOLD 50

// Steps per full revolution -- must match microstepping config. Used only
// for continuous-rotation (speed_mode) drift resync against the reference
// sensor; position-mode motion doesn't need this at all.
#define STEPS_PER_REV 1600

// ---------------------------------------------------------------------------
// PID gains — per-motor, initialised from these defaults in setup().
// Can be updated at runtime via SETPID (global or per-motor) without
// reflashing.
// ---------------------------------------------------------------------------
float Kp = 0.8;
float Ki = 0.01;
float Kd = 0.1;

float motor_kp[NUM_MOTORS];
float motor_ki[NUM_MOTORS];
float motor_kd[NUM_MOTORS];

// ---------------------------------------------------------------------------
// Pin assignments
// ---------------------------------------------------------------------------

const int STEP_PIN[NUM_MOTORS] = {30, 33, 36, 39, 42, 45, 48};
const int DIR_PIN[NUM_MOTORS]  = {31, 34, 37, 40, 43, 46, 49};
const int EN_PIN[NUM_MOTORS]   = {32, 35, 38, 41, 44, 47, 50};

// All motors polled in loop() via pollSensors() -- no ISRs. Active HIGH:
// digitalRead(SENSOR_PIN[i]) == HIGH means triggered (confirmed against
// the working Motor_driver_circular_ribbon_box_ctrl.ino sketch, same pins,
// same hardware -- its HOME_ACTIVE = HIGH).
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

// Emergency-stop latch. Set by STOPALL; cleared by SETPOS/HOME/HOMEALL/ENABLE
// for that motor. While set, runPID() will not resume motion — otherwise an
// "immediate" stop would only last until the next loop iteration.
bool estopped[NUM_MOTORS];

// True while a motor is in continuous-speed mode (SETSPEED) rather than
// position mode (SETPOS/PID). Mutually exclusive with position control --
// cleared by SETPOS/HOME, set by SETSPEED. No absolute target while true;
// the motor just spins at commanded_speed[i] via runSpeed() until told
// otherwise.
bool speed_mode[NUM_MOTORS];
long commanded_speed[NUM_MOTORS];  // steps/sec, signed, as given to SETSPEED

// Previous raw sensor reading per motor, for direction-aware edge
// detection (see checkReferenceEdge()). Kept fresh even when a motor is
// neither homing_active nor speed_mode, so the first comparison made once
// either mode starts is never against a stale reading.
bool last_sensor_state[NUM_MOTORS];

float pid_integral[NUM_MOTORS];
float pid_prev_error[NUM_MOTORS];

volatile bool homed_flag[NUM_MOTORS];     // set by pollSensors(), cleared+reported in loop()

int32_t last_reported_pos[NUM_MOTORS];
unsigned long last_report_time = 0;

bool status_pending = false;
uint8_t status_next_motor = 0;

String input_buffer = "";

// ---------------------------------------------------------------------------
// Interrupt-safe access to actual_pos[]
// No ISRs write to actual_pos anymore (sensor handling is now pure polling
// in pollSensors(), called from loop() like everything else), so this
// guarding is no longer strictly necessary -- left in place since it's
// harmless and cheap. Safe to simplify to plain reads/writes later if
// wanted; not doing that now since it wasn't part of what was asked.
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
// Homing
// ---------------------------------------------------------------------------

void resetHomePosition(int i) {
    writeActualPos(i, 0);
    stepper[i].setCurrentPosition(0);
    ideal_pos[i] = 0;          // resync choreography reference to home
    pid_integral[i] = 0;
    pid_prev_error[i] = 0;
    homed[i] = false;
    homing_active[i] = false;
    homed_flag[i] = true;      // loop() sends HOMED message
}

// Periodic drift correction during continuous rotation (speed_mode) --
// unlike resetHomePosition(), this does NOT touch ideal_pos/PID state or
// exit speed_mode. It just snaps the stepper's own running step count back
// onto the nearest true multiple of STEPS_PER_REV, bounding worst-case
// error to under one revolution no matter how long the motor spins.
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

// The sensor has physical width, so the exact step count at which it first
// reads triggered depends on which side you approach from -- naively using
// the same edge for both directions would resync to two DIFFERENT physical
// angles depending on which way the motor was travelling. Direction-aware:
// moving forward, position 0 = the rising edge (entering the sensor's
// zone); moving in reverse, position 0 = the falling edge (exiting the
// zone from the other side). Used for both homing_active (direction fixed
// by HOMING_DIR) and speed_mode (direction = sign of commanded_speed[i]).
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
        return false;  // not currently seeking a reference
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
            resetHomePosition(i);          // one-time calibration: zero everything, exit homing, report HOMED
        } else {
            resyncContinuousPosition(i);   // periodic drift correction, stays in speed_mode
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
// PID controller
// ---------------------------------------------------------------------------

void runPID(int i) {
    if (!homed[i])        return;  // no reference until homed
    if (homing_active[i]) return;  // don't interfere with homing
    if (speed_mode[i])    return;  // continuous-speed motors have no position target
    if (estopped[i])      return;  // latched by STOPALL until explicitly resumed

    int32_t actual = readActualPos(i);
    float error = (float)(ideal_pos[i] - actual);

    // Large error: a choreography move (SETPOS) already commanded the
    // stepper toward ideal_pos at MAX_SPEED — don't fight it. Just keep PID
    // state fresh so correction is smooth once the move settles into range.
    if (abs(error) >= CHOREO_ERROR_THRESHOLD) {
        pid_integral[i] = 0;
        pid_prev_error[i] = error;
        return;
    }

    // Dead band — ignore noise, bleed integral
    if (abs(error) < PID_DEADBAND) {
        pid_integral[i] *= 0.95;
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
    float output = motor_kp[i] * error + motor_ki[i] * pid_integral[i] + motor_kd[i] * derivative;

    int32_t corrected_target = ideal_pos[i] + (int32_t)output;

    stepper[i].setMaxSpeed(PID_MAX_SPEED);
    stepper[i].moveTo(corrected_target);
}

// ---------------------------------------------------------------------------
// Command handlers
// ---------------------------------------------------------------------------

void cmdSetPos(int id, long steps) {
    if (id < 0 || id >= NUM_MOTORS) return;
    estopped[id] = false;
    speed_mode[id] = false;    // leave continuous-speed mode -- mutually exclusive
    ideal_pos[id] = steps;
    stepper[id].setMaxSpeed(MAX_SPEED);
    stepper[id].moveTo(steps);
    // PID takes over fine correction once within CHOREO_ERROR_THRESHOLD.
}

void cmdSetSpeed(int id, long speed) {
    if (id < 0 || id >= NUM_MOTORS) return;
    estopped[id] = false;
    speed_mode[id] = true;
    homing_active[id] = false;  // supersedes an in-progress homing search, if any
    commanded_speed[id] = speed;
    stepper[id].setSpeed(speed);
    // No moveTo(), no target -- loop() routes speed_mode motors through
    // runSpeed() indefinitely until told otherwise (SETSPEED/SETPOS/HOME/STOP).
}

void cmdHome(int id) {
    if (id < 0 || id >= NUM_MOTORS) return;
    estopped[id] = false;
    speed_mode[id] = false;    // leave continuous-speed mode -- mutually exclusive
    homing_active[id] = true;
    homed[id] = true;
    homing_start_time[id] = millis();
    pid_integral[id] = 0;
    pid_prev_error[id] = 0;
    stepper[id].setSpeed(HOMING_SPEED * HOMING_DIR);
}

void cmdHomeAll() {
    for (int i = 0; i < NUM_MOTORS; i++) cmdHome(i);
}

void cmdStop(int id) {
    if (id < 0 || id >= NUM_MOTORS) return;
    if (speed_mode[id]) {
        // stepper.stop() computes a deceleration profile that only ever
        // gets executed by run() -- speed_mode motors are driven by
        // runSpeed() instead, which would never process it. Just zero the
        // speed directly; runSpeed() stops issuing steps immediately.
        stepper[id].setSpeed(0);
        commanded_speed[id] = 0;
    } else {
        stepper[id].stop();        // decelerate to stop — not latched
    }
    homing_active[id] = false;
}

void cmdStopAll() {
    for (int i = 0; i < NUM_MOTORS; i++) {
        int32_t pos = stepper[i].currentPosition();
        stepper[i].moveTo(pos);    // distanceToGo() becomes 0 — no further steps issued
        stepper[i].setSpeed(0);
        homing_active[i] = false;
        estopped[i] = true;        // latched: PID will not resume until explicitly cleared
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

void cmdSetPidGlobal(float kp, float ki, float kd) {
    for (int i = 0; i < NUM_MOTORS; i++) {
        motor_kp[i] = kp;
        motor_ki[i] = ki;
        motor_kd[i] = kd;
        pid_integral[i] = 0.0;
        pid_prev_error[i] = 0.0;
    }
    Kp = kp; Ki = ki; Kd = kd;
    Serial.println("PID_UPDATED");
}

void cmdSetPidMotor(int id, float kp, float ki, float kd) {
    if (id < 0 || id >= NUM_MOTORS) return;
    motor_kp[id] = kp;
    motor_ki[id] = ki;
    motor_kd[id] = kd;
    pid_integral[id] = 0.0;
    pid_prev_error[id] = 0.0;
    Serial.print("PID_UPDATED ");
    Serial.println(id);
}

void cmdResetPid() {
    for (int i = 0; i < NUM_MOTORS; i++) {
        pid_integral[i] = 0.0;
        pid_prev_error[i] = 0.0;
    }
    Serial.println("PID_RESET");
}

void cmdRequestStatus() {
    status_pending = true;
    status_next_motor = 0;
}

// ---------------------------------------------------------------------------
// Serial command parser
// Reads one character at a time, dispatches a full line on '\n'.
// Never uses Serial.readString() — it blocks for its full timeout.
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

    } else if (strcmp(cmd, "SETPID") == 0) {
        // 3 args after SETPID -> global; 4 args -> per-motor (id kp ki kd)
        char* a1 = strtok(NULL, " ");
        char* a2 = strtok(NULL, " ");
        char* a3 = strtok(NULL, " ");
        char* a4 = strtok(NULL, " ");
        if (a1 && a2 && a3 && !a4) {
            cmdSetPidGlobal(atof(a1), atof(a2), atof(a3));
        } else if (a1 && a2 && a3 && a4) {
            cmdSetPidMotor(atoi(a1), atof(a2), atof(a3), atof(a4));
        }

    } else if (strcmp(cmd, "RESETPID") == 0) {
        cmdResetPid();

    } else if (strcmp(cmd, "STATUS") == 0) {
        cmdRequestStatus();

    } else {
        // Echo back rather than silently dropping — this is the only signal
        // that a line arrived at all, which matters when diagnosing serial
        // terminal issues (wrong line ending, case mismatch, etc).
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
            // Guard against a runaway buffer if '\n' is never seen.
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

// Sends one STATUS line per loop() iteration rather than all 7 at once, so a
// STATUS request can never stall step generation waiting on the TX buffer.
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
    Serial.println(pid_integral[i], 4);

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
        stepper[i].setMaxSpeed(MAX_SPEED);
        stepper[i].setAcceleration(ACCELERATION);
        stepper[i].setEnablePin(EN_PIN[i]);
        stepper[i].setPinsInverted(false, false, true);  // EN active LOW
        stepper[i].enableOutputs();

        // INPUT_PULLDOWN, active HIGH: unconnected sensor channels float
        // toward LOW (not triggered) via the pull-down, so bench testing
        // with fewer than 7 motors wired correctly times out on the
        // unconnected ones instead of reporting a false HOMED.
        pinMode(SENSOR_PIN[i], INPUT_PULLDOWN);
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
        pid_integral[i]      = 0.0;
        pid_prev_error[i]    = 0.0;

        motor_kp[i] = Kp;
        motor_ki[i] = Ki;
        motor_kd[i] = Kd;
    }

    Serial.println("READY");
}

void loop() {
    // 1. Run PID for each motor — updates stepper target
    for (int i = 0; i < NUM_MOTORS; i++) {
        //runPID(i);
    }

    // 2. Execute stepper motion (must run every loop, never blocked)
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (homing_active[i] || speed_mode[i]) {
            stepper[i].runSpeed();   // constant speed -- homing search or continuous rotation
        } else {
            stepper[i].run();        // acceleration-managed motion toward a SETPOS target
        }
        writeActualPos(i, stepper[i].currentPosition());
    }

    // 3. Handle homed flags set by pollSensors()
    checkHomedFlags();

    // 4. Poll all 7 sensors
    pollSensors();

    // 5. Check homing timeouts
    checkHomingTimeouts();

    // 6. Send periodic position reports to TD (monitoring only)
    sendPositionReports();

    // 7. Send at most one STATUS line this iteration, if a request is pending
    sendStatusChunk();

    // 8. Parse incoming serial commands from TD
    parseSerial();
}
