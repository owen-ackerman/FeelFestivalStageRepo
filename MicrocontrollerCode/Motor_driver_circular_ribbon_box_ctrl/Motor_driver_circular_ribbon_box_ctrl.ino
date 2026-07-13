#include <AccelStepper.h>

const int stepPins[6] = { 25, 26, 27, 14, 12, 13 };
const int dirPins[6]  = { 21, 19, 18, 17, 16,  4 };
const int homingPins[6] = {33, 32, 35, 34, 39, 36};  // swap for your actual pins
const float SPEED      = 500.0; // positive = forward, negative = reverse

const float HOMING_SPEED = 500.0;
const int HOMING_DIR = -1;
AccelStepper steppers[6] = {
  AccelStepper(AccelStepper::DRIVER, stepPins[0], dirPins[0]),
  AccelStepper(AccelStepper::DRIVER, stepPins[1], dirPins[1]),
  AccelStepper(AccelStepper::DRIVER, stepPins[2], dirPins[2]),
  AccelStepper(AccelStepper::DRIVER, stepPins[3], dirPins[3]),
  AccelStepper(AccelStepper::DRIVER, stepPins[4], dirPins[4]),
  AccelStepper(AccelStepper::DRIVER, stepPins[5], dirPins[5]),
};

void setup() {
  Serial.begin(250000);
  while (!Serial);

  for (int i = 0; i < 6; i++) {
    steppers[i].setMaxSpeed(2400.0);
    steppers[i].setSpeed(0);
  }

  Serial.println("Hello Serial Setup");
}
void loop() {
  // Run all steppers on every loop — this must not be blocked
  for (int i = 0; i < 6; i++) {
    steppers[i].runSpeed();
  }

  // Scan for the two-byte sync header 0xAA 0x55.
  // We only scan while >= 8 bytes are available so we never consume the
  // header without its payload — doing so would orphan the payload bytes
  // and stall communication permanently.
  bool headerFound = false;
  while (Serial.available() >= 8) {
    if (Serial.peek() == 0xAA) {
      Serial.read();               // tentatively consume 0xAA
      if (Serial.peek() == 0x55) {
        Serial.read();             // consume 0x55 — full header confirmed
        headerFound = true;
        break;
      }
      // 0xAA was a data byte, not a real header — keep scanning
    } else {
      Serial.read();               // discard non-sync byte
    }
  }

  if (!headerFound) return;

  // Header consumed; 6 payload bytes are guaranteed available (we had >= 8)
  int robotNum = Serial.read() - 1;  // convert 1-based to 0-based
  int motorNum = Serial.read();
  int b0       = Serial.read();
  int b1       = Serial.read();
  int b2       = Serial.read();
  int b3       = Serial.read();

  // Validate after reading all bytes so nothing is left stranded in the buffer
  if (robotNum < 0 || robotNum > 5) return;
  if (motorNum != 1 && motorNum != 2 && motorNum != 3) return;

  if (motorNum == 2) {
    int stepperSPS = (b0 << 16) | (b1 << 8) | b2;
    if (b3 == 1) stepperSPS = -stepperSPS;
    steppers[robotNum].setSpeed(stepperSPS);
  }
}