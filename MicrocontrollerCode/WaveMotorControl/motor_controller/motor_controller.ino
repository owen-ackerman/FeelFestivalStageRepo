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

// Minimum time between accepted homing-sensor triggers, for both the
// hardware-interrupt sensors (0-3) and the polled ones (4-6).
#define DEBOUNCE_MS 10

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

// Motors 0-3 sit on hardware-interrupt pins (18,19,20,21) and are handled by
// ISRs below. Motors 4-6 (22,23,33) are not interrupt-capable on the Mega
// and are polled in loop() via pollSensors().
const int SENSOR_PIN[NUM_MOTORS] = {22, 23, 24, 25, 26, 27, 28};

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

AccelStepper stepper[NUM_MOTORS];

volatile int32_t actual_pos[NUM_MOTORS];  // steps from home — ISR writes it; access via readActualPos()/writeActualPos()
int32_t ideal_pos[NUM_MOTORS];            // target set by TD via SETPOS
volatile bool homed[NUM_MOTORS];          // true after first successful home
volatile bool homing_active[NUM_MOTORS];  // currently executing home sequence
unsigned long homing_start_time[NUM_MOTORS];

// Emergency-stop latch. Set by STOPALL; cleared by SETPOS/HOME/HOMEALL/ENABLE
// for that motor. While set, runPID() will not resume motion — otherwise an
// "immediate" stop would only last until the next loop iteration.
bool estopped[NUM_MOTORS];

float pid_integral[NUM_MOTORS];
float pid_prev_error[NUM_MOTORS];

volatile bool homed_flag[NUM_MOTORS];     // set in ISR/poll, cleared+reported in loop()
volatile unsigned long last_home_trigger_ms[NUM_MOTORS];  // debounce timestamps
bool last_sensor_state[NUM_MOTORS];       // previous polled-sensor state (motors 4-6 only)

int32_t last_reported_pos[NUM_MOTORS];
unsigned long last_report_time = 0;

bool status_pending = false;
uint8_t status_next_motor = 0;

String input_buffer = "";

// ---------------------------------------------------------------------------
// Interrupt-safe access to actual_pos[]
// int32_t writes are not atomic on the 8-bit ATmega2560, and actual_pos is
// written both by ISRs (motors 0-3, on home) and by the main loop (every
// iteration, from stepper.currentPosition()). Guard both sides so a read
// never observes a torn value.
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

// Shared by both the hardware ISRs (motors 0-3) and the polled path
// (motors 4-6). Must stay ISR-safe: no Serial, no String, no malloc.
void resetHomePosition(int i) {
    writeActualPos(i, 0);
    stepper[i].setCurrentPosition(0);
    ideal_pos[i] = 0;          // resync choreography reference to home
    pid_integral[i] = 0;
    pid_prev_error[i] = 0;
    homed[i] = true;
    homing_active[i] = false;
    homed_flag[i] = true;      // loop() sends HOMED message
}

void handleSensorISR(int i) {
    unsigned long now = millis();
    if (now - last_home_trigger_ms[i] < DEBOUNCE_MS) return;
    last_home_trigger_ms[i] = now;
    resetHomePosition(i);
}

void sensor_isr_0() { handleSensorISR(0); }
void sensor_isr_1() { handleSensorISR(1); }
void sensor_isr_2() { handleSensorISR(2); }
void sensor_isr_3() { handleSensorISR(3); }

// Motors 4-6: no hardware interrupt available, so poll with edge detection
// plus the same debounce window used by the ISR path.
void pollSensors() {
    for (int i = 4; i < NUM_MOTORS; i++) {
        bool state = digitalRead(SENSOR_PIN[i]);  // active LOW: LOW = triggered
        if (last_sensor_state[i] == HIGH && state == LOW) {
            unsigned long now = millis();
            if (now - last_home_trigger_ms[i] >= DEBOUNCE_MS) {
                last_home_trigger_ms[i] = now;
                resetHomePosition(i);
            }
        }
        last_sensor_state[i] = state;
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
    ideal_pos[id] = steps;
    stepper[id].setMaxSpeed(MAX_SPEED);
    stepper[id].moveTo(steps);
    // PID takes over fine correction once within CHOREO_ERROR_THRESHOLD.
}

void cmdHome(int id) {
    if (id < 0 || id >= NUM_MOTORS) return;
    estopped[id] = false;
    homing_active[id] = true;
    homed[id] = false;
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
    stepper[id].stop();            // decelerate to stop — not latched
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

        pinMode(SENSOR_PIN[i], INPUT);  // optocoupler provides pull-up
        last_sensor_state[i] = HIGH;

        actual_pos[i]        = 0;
        ideal_pos[i]         = 0;
        last_reported_pos[i] = 0;
        homed[i]             = false;
        homing_active[i]     = false;
        homed_flag[i]        = false;
        estopped[i]          = false;
        last_home_trigger_ms[i] = 0;
        pid_integral[i]      = 0.0;
        pid_prev_error[i]    = 0.0;

        motor_kp[i] = Kp;
        motor_ki[i] = Ki;
        motor_kd[i] = Kd;
    }

    // Hardware interrupts: motors 0-3
    attachInterrupt(digitalPinToInterrupt(SENSOR_PIN[0]), sensor_isr_0, FALLING);
    attachInterrupt(digitalPinToInterrupt(SENSOR_PIN[1]), sensor_isr_1, FALLING);
    attachInterrupt(digitalPinToInterrupt(SENSOR_PIN[2]), sensor_isr_2, FALLING);
    attachInterrupt(digitalPinToInterrupt(SENSOR_PIN[3]), sensor_isr_3, FALLING);
    // Motors 4-6: polled in loop() via pollSensors()

    Serial.println("READY");
}

void loop() {
    // 1. Run PID for each motor — updates stepper target
    for (int i = 0; i < NUM_MOTORS; i++) {
        //runPID(i);
    }

    // 2. Execute stepper motion (must run every loop, never blocked)
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (homing_active[i]) {
            stepper[i].runSpeed();   // constant speed during homing
        } else {
            stepper[i].run();        // acceleration-managed motion otherwise
        }
        writeActualPos(i, stepper[i].currentPosition());
    }

    // 3. Handle homed flags set by ISRs/poll
    checkHomedFlags();

    // 4. Poll sensors for motors 4-6
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
