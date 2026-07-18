// Festival Stage Wave Motor Control — Arduino Mega 2560 firmware (SIMPLE)
// One sketch, flashed to both the LEFT and RIGHT Megas (7 motors each).
//
// The minimal variant. Only three things:
//   HOME / HOMEALL  -> drive to the homing sensor once and zero position
//   SETPOS <id> <n> -> move to an absolute step position (with acceleration)
//   SETSPEED <id> <sps> -> spin continuously at a signed speed
//
// The homing sensor is used ONLY during a HOME sequence. Once a motor is
// running (SETPOS or SETSPEED) the sensor is completely ignored -- there is
// no per-revolution resync, so a continuous spin is never interrupted.
// Trade-off vs. motor_controller_without_pid.ino: no drift correction over
// long runs; position can slowly wander from truth until the next HOME.
//
// No PID. STOP/STOPALL/ENABLE/DISABLE/STATUS kept for safety and monitoring.
// STATUS wire format matches the other builds (0.0000 in the old
// pid_integral slot) so the same TD side works unchanged.

#include <AccelStepper.h>
#include <string.h>
#include <stdlib.h>

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

#define NUM_MOTORS 7

#define REPORT_INTERVAL_MS 50      // position reports to TD (monitoring)
#define HOMING_SPEED 400           // steps/sec — slow and safe
#define HOMING_TIMEOUT_MS 15000
#define BAUD_RATE 115200
#define REPORT_DEADBAND 2          // steps — suppress POS messages for tiny changes

// ============================================================================
// !!! HOMING_DIR MUST BE VERIFIED PER MEGA BEFORE FLASHING !!!
// LEFT and RIGHT groups are mirrored -- the same sign can mean "toward home"
// on one side and "away" on the other. Confirm each motor homes toward its
// switch before flashing the other Mega.
// ============================================================================
#define HOMING_DIR -1

// Compile-time speed/accel limits (no runtime SETMAXSPEED/SETACCEL in this
// build). Max speed caps both SETPOS moves and SETSPEED continuous rotation
// (AccelStepper clamps setSpeed() to setMaxSpeed()), so keep it generous.
#define MAX_SPEED 8000             // steps/sec
#define ACCELERATION 1500          // steps/sec^2

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

int32_t ideal_pos[NUM_MOTORS];            // last SETPOS target (for STATUS)
bool homed[NUM_MOTORS];                   // true after a successful home
bool homing_active[NUM_MOTORS];           // currently seeking the sensor
unsigned long homing_start_time[NUM_MOTORS];
bool estopped[NUM_MOTORS];                // latched by STOPALL

bool speed_mode[NUM_MOTORS];              // SETSPEED vs SETPOS
long commanded_speed[NUM_MOTORS];

bool last_sensor_state[NUM_MOTORS];       // for homing edge detection
bool homed_flag[NUM_MOTORS];              // set on home, reported in loop()

int32_t last_reported_pos[NUM_MOTORS];
unsigned long last_report_time = 0;

bool status_pending = false;
uint8_t status_next_motor = 0;

String input_buffer = "";

// ---------------------------------------------------------------------------
// Homing (the only place the sensor is read)
// ---------------------------------------------------------------------------

// Reads every motor's sensor, but only ACTS on it while that motor is
// homing. Direction-aware edge: since the sensor has physical width, "home"
// is the rising edge when the motor moves forward and the falling edge when
// it moves reverse -- HOMING_DIR fixes which. last_sensor_state is refreshed
// every call so the first comparison after HOME starts is never stale.
void pollHomingSensors() {
    bool moving_forward = (HOMING_DIR > 0);
    for (int i = 0; i < NUM_MOTORS; i++) {
        bool state = digitalRead(SENSOR_PIN[i]);
        if (homing_active[i]) {
            bool edge = moving_forward ? (last_sensor_state[i] == LOW && state == HIGH)
                                       : (last_sensor_state[i] == HIGH && state == LOW);
            if (edge) {
                stepper[i].setCurrentPosition(0);
                ideal_pos[i] = 0;
                homed[i] = true;
                homing_active[i] = false;
                homed_flag[i] = true;   // loop() sends HOMED
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

void cmdSetPos(int id, long steps) {
    if (id < 0 || id >= NUM_MOTORS) return;
    estopped[id] = false;
    speed_mode[id] = false;
    homing_active[id] = false;
    ideal_pos[id] = steps;
    stepper[id].moveTo(steps);
}

void cmdSetSpeed(int id, long speed) {
    if (id < 0 || id >= NUM_MOTORS) return;
    estopped[id] = false;
    speed_mode[id] = true;
    homing_active[id] = false;
    commanded_speed[id] = speed;
    stepper[id].setSpeed(speed);   // clamped to MAX_SPEED
}

void cmdHome(int id) {
    if (id < 0 || id >= NUM_MOTORS) return;
    estopped[id] = false;
    speed_mode[id] = false;
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
    if (speed_mode[id]) {
        stepper[id].setSpeed(0);
        commanded_speed[id] = 0;
    } else {
        stepper[id].stop();
    }
    homing_active[id] = false;
}

void cmdStopAll() {
    for (int i = 0; i < NUM_MOTORS; i++) {
        stepper[i].moveTo(stepper[i].currentPosition());  // no further steps
        stepper[i].setSpeed(0);
        homing_active[i] = false;
        estopped[i] = true;
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

void sendPositionReports() {
    unsigned long now = millis();
    if (now - last_report_time < REPORT_INTERVAL_MS) return;
    last_report_time = now;

    for (int i = 0; i < NUM_MOTORS; i++) {
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

// Same STATUS format as the other builds (6th field is a fixed 0.0000
// placeholder for the pid_integral slot TD parses but discards).
void sendStatusChunk() {
    if (!status_pending) return;

    int i = status_next_motor;
    Serial.print("STATUS ");
    Serial.print(i);
    Serial.print(' ');
    Serial.print(stepper[i].currentPosition());
    Serial.print(' ');
    Serial.print(ideal_pos[i]);
    Serial.print(' ');
    Serial.print(homed[i] ? 1 : 0);
    Serial.print(' ');
    Serial.print(homing_active[i] ? 1 : 0);
    Serial.print(' ');
    Serial.println("0.0000");

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

        pinMode(SENSOR_PIN[i], INPUT_PULLDOWN);  // active HIGH; unconnected reads LOW
        last_sensor_state[i] = LOW;

        ideal_pos[i]         = 0;
        last_reported_pos[i] = 0;
        homed[i]             = false;
        homing_active[i]     = false;
        homed_flag[i]        = false;
        estopped[i]          = false;
        speed_mode[i]        = false;
        commanded_speed[i]   = 0;
    }

    Serial.println("READY");
}

void loop() {
    // Execute motion: constant speed while homing or in SETSPEED mode,
    // acceleration-managed otherwise. Never blocked.
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (homing_active[i] || speed_mode[i]) {
            stepper[i].runSpeed();
        } else {
            stepper[i].run();
        }
    }

    checkHomedFlags();
    pollHomingSensors();     // acts only on motors currently homing
    checkHomingTimeouts();
    sendPositionReports();
    sendStatusChunk();
    parseSerial();
}
