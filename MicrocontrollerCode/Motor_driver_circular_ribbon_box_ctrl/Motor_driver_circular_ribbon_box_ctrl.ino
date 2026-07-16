#include <AccelStepper.h>
// motor driver 4 produces CW rotation instead of CCW
// drivers 1&2, CW rotation
const int stepPins[7] =   {30, 33, 36, 39, 42, 45, 48};// FOR old box{ 25, 26, 27, 14, 12, 13 };
const int dirPins[7]  =   {31, 34, 37, 40, 43, 46, 49};// FOR old box{ 21, 19, 18, 17, 16,  4 };
const int enPins[7]   =   {32, 35, 38, 41, 44, 47, 50};
const int homingPins[7] = {22, 23, 24, 25, 26, 27, 28};// FOR old box{33, 32, 35, 34, 39, 36};  // swap for your actual pins
const float SPEED      = 500.0; // positive = forward , negative = reverse

const float HOMING_SPEED = 800.0;
const int HOMING_DIR = -1;
AccelStepper steppers[7] = {
  AccelStepper(AccelStepper::DRIVER, stepPins[0], dirPins[0]),
  AccelStepper(AccelStepper::DRIVER, stepPins[1], dirPins[1]),
  AccelStepper(AccelStepper::DRIVER, stepPins[2], dirPins[2]),
  AccelStepper(AccelStepper::DRIVER, stepPins[3], dirPins[3]),
  AccelStepper(AccelStepper::DRIVER, stepPins[4], dirPins[4]),
  AccelStepper(AccelStepper::DRIVER, stepPins[5], dirPins[5]),
  AccelStepper(AccelStepper::DRIVER, stepPins[6], dirPins[6]),
};
const unsigned long HOMING_TIMEOUT_MS = 3000;
const int HOME_ACTIVE = HIGH;   // PC817 pulls the pin down when triggered

bool homed[7];

void homeAll() {
  for (int i = 0; i < 7; i++) {
    pinMode(homingPins[i], INPUT_PULLUP);
    homed[i] = false;
    steppers[i].setSpeed(HOMING_SPEED * HOMING_DIR);
  }

  unsigned long start = millis();
  int remaining = 7;

  while (remaining > 0 && (millis() - start) < HOMING_TIMEOUT_MS) {
    for (int i = 0; i < 7; i++) {
      if (homed[i]) continue;

      if (digitalRead(homingPins[i]) == HOME_ACTIVE) {
        steppers[i].setSpeed(0);
        steppers[i].setCurrentPosition(0);   // also zeroes internal speed
        homed[i] = true;
        remaining--;
        Serial.print("Homed motor "); Serial.println(i);
      } else {
        steppers[i].runSpeed();
      }
    }
  }

  // Timeout — stop whatever never found its switch
  for (int i = 0; i < 7; i++) {
    if (!homed[i]) {
      steppers[i].setSpeed(0);
      Serial.print("HOMING TIMEOUT on motor "); Serial.println(i);
    }
  }
}

void setup() {
  Serial.begin(250000);
  while (!Serial);

  for (int i = 0; i < 7; i++) {
  pinMode(enPins[i], OUTPUT);
  digitalWrite(enPins[i], LOW);
}
  for (int i = 0; i < 7; i++) {
    steppers[i].setMaxSpeed(2400.0);
    steppers[i].setSpeed(0);
  }

  Serial.println("Hello Serial Setup");
  homeAll();
  
}
void loop() {
  /*int sensorNum = 0;
  Serial.print("Homing sensor ");
  Serial.print(sensorNum);
  Serial.print(": ");
  Serial.println(digitalRead(homingPins[sensorNum]));
  */
  // Run all steppers on every loop — this must not be blocked
  for (int i = 0; i < 7; i++) {
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
  if (robotNum < 0 || robotNum > 6) return;
  if (motorNum != 1 && motorNum != 2 && motorNum != 3) return;

  if (motorNum == 2) {
    int stepperSPS = (b0 << 16) | (b1 << 8) | b2;
    if (b3 == 1) stepperSPS = -stepperSPS;
    for (int i = 0; i < 7; i++) {
    steppers[i].setSpeed(stepperSPS);
    }
    Serial.print("Stepper Speed: ");
    Serial.println(stepperSPS);

   
  }
}