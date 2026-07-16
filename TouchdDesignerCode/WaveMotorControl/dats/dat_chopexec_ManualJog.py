"""
CHOP Execute DAT callback — wires a CHOP channel straight into one motor's
SETPOS target, for bench-testing a single motor by hand (Slider, LFO, or
Constant CHOP feeding a target step count).

Setup in TD:
  1. Create a CHOP Execute DAT (e.g. 'chopexec_manual_jog') directly inside
     base_motor_system, as a SIBLING of base_motor_controller — not nested
     inside base_motor_controller or any other COMP. This file uses the
     relative lookup op('base_motor_controller'), which resolves relative
     to this script's own parent, so base_motor_controller must be a
     sibling for that lookup to find it.
  2. Set its 'CHOP' parameter to the CHOP driving the test (one channel —
     if the CHOP has multiple channels, only the first is used below).
  3. Set its 'Callbacks DAT' parameter to this file.
  4. Edit MOTOR_ID below to the global motor id (0-13) you're testing.

The channel's value is used directly as a target step count, no scaling —
e.g. a Slider CHOP ranged -800..800 maps straight to SETPOS steps.

Routed through MotorControllerEXT.SetIdealPos() rather than straight to the
serial DAT, so it gets the same change-detection the rest of the system
uses (won't flood serial on every frame) and the monitoring table stays
accurate. Requires base_motor_controller to exist in the network — see
README_TD_SETUP.md for the operator tree.

Does NOT require homing first: SETPOS drives AccelStepper's own position
counter regardless of the firmware's homed[] state (see cmdSetPos() in
motor_controller.ino).
"""

# Global motor id (0-13) this CHOP drives. Change before testing a
# different motor.
MOTOR_ID = 0


def onValueChange(channel, sampleIndex, val, prev):
    controller = op('base_motor_controller')
    if controller is None:
        debug("dat_chopexec_ManualJog: base_motor_controller not found — is the network set up yet?")
        return
    controller.ext.MotorControllerEXT.SetIdealPos(MOTOR_ID, int(val))
    return


def onOffToOn(channel, sampleIndex, val, prev):
    return


def whileOn(channel, sampleIndex, val, prev):
    return


def onOnToOff(channel, sampleIndex, val, prev):
    return


def whileOff(channel, sampleIndex, val, prev):
    return
