// Festival Stage Wave Motor Control — Arduino Mega 2560 firmware (SPEED-ONLY)
// One sketch, flashed to both the LEFT and RIGHT Megas (7 motors each).
//
// Motors are controlled by VELOCITY only -- there is no SETPOS / position
// targeting at all. TD streams SETSPEED values; the firmware just runs each
// motor at its commanded speed via runSpeed(). This avoids the choppiness of
// streaming discrete position targets: the motor cruises at a velocity and a
// slightly-late speed update is seamless (no position jump).
//
// All motion "intelligence" -- waves, phase offsets, drift recentering --
// lives in TD (ChoreographySpeedEXT). Any position feedback correction is
// computed there from the POS reports below and folded into the SETSPEED
// value. This firmware stays deliberately dumb: home, then spin at whatever
// speed you're told.
//
// The homing sensor is used ONLY during a HOME sequence, to establish a
// known zero. It is NOT read during the run (no per-revolution resync).
//
// Commands: SETSPEED, HOME, HOMEALL, STOP, STOPALL, ENABLE, DISABLE, STATUS.
// STATUS wire format matches the other builds (0.0000 in the old
// pid_integral slot) so the same TD side works unchanged.

#include <AccelStepper.h>
#include <string.h>
#include <stdlib.h>

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

#define NUM_MOTORS 7

#define HOMING_SPEED 400           // steps/sec — slow and safe
#define HOMING_TIMEOUT_MS 15000
#define BAUD_RATE 115200

// POS reports are for TD-side feedback (drift recentering) and monitoring.
// Slower than the position-mode builds -- velocity control doesn't need
// fast position feedback, and this keeps serial traffic down while TD is
// also streaming SETSPEED the other direction.
#define REPORT_INTERVAL_MS 100
#define REPORT_DEADBAND 2          // steps — suppress POS for tiny changes

// ============================================================================
// !!! HOMING_DIR MUST BE VERIFIED PER MEGA BEFORE FLASHING !!!
// LEFT and RIGHT groups are mirrored -- the same sign can mean "toward home"
// on one side and "away" on the other. Confirm each motor homes toward its
// switch before flashing the other Mega.
// ============================================================================
#define HOMING_DIR -1

// Max speed cap. AccelStepper clamps setSpeed() to setMaxSpeed(), so this is
// the ceiling for every SETSPEED value -- keep it generous.
#define MAX_SPEED 8000             // steps/sec

// ---------------------------------------------------------------------------
// Pin assignments
// ---------------------------------------------------------------------------

const int STEP_PIN[NUM_MOTORS] = {30, 33, 36, 39, 42, 45, 48};
const int DIR_PIN[NUM_MOTORS]  = {31, 34, 37, 40, 43, 46, 49};
const int EN_PIN[NUM_MOTORS]   = {32, 35, 38, 41, 44, 47, 50};
const int SENSOR_PIN[NUM_MOTORS] = {22, 23, 24, 25, 26, 27, 28};  // active HIGH

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

AccelStepper stepper[NUM_MOTORS];

bool homed[NUM_MOTORS];                   // true after a successful home
bool homing_active[NUM_MOTORS];           // currently seeking the sensor
unsigned long homing_start_time[NUM_MOTORS];
bool estopped[NUM_MOTORS];                // latched by STOPALL

long commanded_speed[NUM_MOTORS];         // last SETSPEED value (signed)

bool last_sensor_state[NUM_MOTORS];       // for homing edge detection
bool homed_flag[NUM_MOTORS];              // set on home, reported in loop()

int32_t last_reported_pos[NUM_MOTORS];
unsigned long last_report_time = 0;

bool status_pending = false;
uint8_t status_next_motor = 0;

String input_buffer = "";

// ---------------------------------------------------------------------------
// Sensor polling
// ---------------------------------------------------------------------------

// Reads every motor's sensor every loop. Two behaviors depending on state:
//
//   homing_active[i]  -> HOME: on the reference edge, zero the position and
//                        finish homing (the motor DOES stop -- that's homing).
//
//   homed[i], moving  -> RUN: on the reference edge, REPORT the crossing
//                        ("ZERO <id> <count>") but do NOT touch position or
//                        speed. This is the absolute-position reference TD's
//                        feedback loop uses to re-anchor. Modifying the
//                        stepper here (as the resync builds did) resets its
//                        speed to 0 and stops the motor at the sensor -- so
//                        this build only reports and lets TD do the math.
//
// Direction-aware edge in both cases: the sensor has physical width, so the
// true zero is the rising edge moving forward and the falling edge moving
// reverse. Direction is HOMING_DIR while homing, or the sign of the
// commanded speed while running. last_sensor_state is refreshed every call.
void pollSensors() {
    for (int i = 0; i < NUM_MOTORS; i++) {
        bool state = digitalRead(SENSOR_PIN[i]);
        bool prev = last_sensor_state[i];
        last_sensor_state[i] = state;

        if (homing_active[i]) {
            bool fwd = (HOMING_DIR > 0);
            bool edge = fwd ? (prev == LOW && state == HIGH)
                            : (prev == HIGH && state == LOW);
            if (edge) {
                stepper[i].setCurrentPosition(0);
                stepper[i].setSpeed(0);      // setCurrentPosition zeroes speed anyway; make it explicit
                commanded_speed[i] = 0;
                homed[i] = true;
                homing_active[i] = false;
                homed_flag[i] = true;        // loop() sends HOMED

            }
        } else if (homed[i] && commanded_speed[i] != 0) {
            bool fwd = (commanded_speed[i] > 0);
            bool edge = fwd ? (prev == LOW && state == HIGH)
                            : (prev == HIGH && state == LOW);
            if (edge) {
                // Report only -- physical position is 0 right now; the count
                // tells TD how far the open-loop count has drifted from it.
                Serial.print("ZERO ");
                Serial.print(i);
                Serial.print(' ');
                Serial.println(stepper[i].currentPosition());
            }
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
            stepper[i].setSpeed(0);
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

void cmdSetSpeed(int id, long speed) {
    if (id < 0 || id >= NUM_MOTORS) return;
    estopped[id] = false;
    homing_active[id] = false;
    commanded_speed[id] = speed;
    stepper[id].setSpeed(speed);   // clamped to MAX_SPEED
}

void cmdHome(int id) {
    if (id < 0 || id >= NUM_MOTORS) return;
    estopped[id] = false;
    homed[id] = false;
    homing_active[id] = true;
    homing_start_time[id] = millis();
    stepper[id].setSpeed(HOMING_SPEED * HOMING_DIR);
}

void cmdHomeAll() {
    for (int i = 0; i < NUM_MOTORS; i++) cmdHome(i);
}

void cmdStop(int id) {
    if (id < 0 || id >= NUM_MOTORS) return;
    stepper[id].setSpeed(0);
    commanded_speed[id] = 0;
    homing_active[id] = false;
}

void cmdStopAll() {
    for (int i = 0; i < NUM_MOTORS; i++) {
        stepper[i].setSpeed(0);
        commanded_speed[i] = 0;
        homing_active[i] = false;
        estopped[i] = true;   // latched until a new SETSPEED/HOME/ENABLE clears it
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

void cmdRequestStatus() {
    status_pending = true;
    status_next_motor = 0;
}

// ---------------------------------------------------------------------------
// Serial parser
// ---------------------------------------------------------------------------

void handleCommand(String &line) {
    char buf[64];
    line.toCharArray(buf, sizeof(buf));

    char* cmd = strtok(buf, " ");
    if (cmd == NULL) return;

    if (strcmp(cmd, "SETSPEED") == 0) {
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

    } else if (strcmp(cmd, "STATUS") == 0) {
        cmdRequestStatus();

    } else {
        Serial.print("ERR unknown command: ");
        Serial.println(cmd);
    }
}

void parseSerial() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n') {
            input_buffer.trim();
            if (input_buffer.length() > 0) handleCommand(input_buffer);
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

// POS reports only for motors that have been homed -- an un-homed motor has
// no meaningful zero reference, so its position is noise for TD's feedback
// loop. This also keeps a fresh boot (nothing homed) from flooding serial.
void sendPositionReports() {
    unsigned long now = millis();
    if (now - last_report_time < REPORT_INTERVAL_MS) return;
    last_report_time = now;

    for (int i = 0; i < NUM_MOTORS; i++) {
        if (!homed[i]) continue;
        int32_t pos = stepper[i].currentPosition();
        if (labs(pos - last_reported_pos[i]) >= REPORT_DEADBAND) {
            last_reported_pos[i] = pos;
            Serial.print("POS ");
            Serial.print(i);
            Serial.print(' ');
            Serial.println(pos);
        }
    }
}

// STATUS format matches the other builds. Fields 3 (ideal) and 6 (integral)
// are fixed placeholders here -- this build has neither a position target
// nor PID. TD parses both but only uses actual/homed/homing.
void sendStatusChunk() {
    if (!status_pending) return;

    int i = status_next_motor;
    Serial.print("STATUS ");
    Serial.print(i);
    Serial.print(' ');
    Serial.print(stepper[i].currentPosition());
    Serial.print(' ');
    Serial.print(0);                       // ideal — n/a in speed-only
    Serial.print(' ');
    Serial.print(homed[i] ? 1 : 0);
    Serial.print(' ');
    Serial.print(homing_active[i] ? 1 : 0);
    Serial.print(' ');
    Serial.println("0.0000");              // pid_integral slot

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
        stepper[i].setEnablePin(EN_PIN[i]);
        stepper[i].setPinsInverted(false, false, true);  // EN active LOW
        stepper[i].enableOutputs();

        pinMode(SENSOR_PIN[i], INPUT_PULLDOWN);  // active HIGH; unconnected reads LOW
        last_sensor_state[i] = LOW;

        last_reported_pos[i] = 0;
        homed[i]             = false;
        homing_active[i]     = false;
        homed_flag[i]        = false;
        estopped[i]          = false;
        commanded_speed[i]   = 0;
    }

    Serial.println("READY");
}

void loop() {
    // Every motor is always in speed mode -- just run it. Never blocked.
    for (int i = 0; i < NUM_MOTORS; i++) {
        stepper[i].runSpeed();
    }

    checkHomedFlags();
    pollSensors();           // homing edge -> HOMED; run-time zero crossing -> ZERO report
    checkHomingTimeouts();
    sendPositionReports();
    sendStatusChunk();
    parseSerial();
}
