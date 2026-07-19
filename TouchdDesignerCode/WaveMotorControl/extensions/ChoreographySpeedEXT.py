"""
ChoreographySpeedEXT — speed-only motion design for the 14 motors.

The velocity-control counterpart to ChoreographyEXT. Instead of streaming
position targets (SETPOS), it computes a SPEED per motor every frame and
sends it via MotorControllerEXT.SetSpeed (SETSPEED). This is smoother for
continuous/wave motion -- the motor cruises at a velocity rather than
chasing a stream of discrete positions -- and is meant to run against
speed_motor_controller.ino.

Coexists with ChoreographyEXT (does not replace it). Lives on its own COMP
(e.g. base_choreography_speed), a sibling of base_motor_controller. Driven
by an Execute DAT with Frame Start, same as ChoreographyEXT -- see
dats/choreo_speed_execute.py.

Three modes (custom parameter Mode: CONSTANT | DIRECTION | WAVE):

  CONSTANT   every motor spins at the same Basespeed (steps/sec, signed).

  DIRECTION  every motor spins at |Basespeed| but its own direction, from a
             per-motor sign. Signs come from a 'direction' CHOP (channel i,
             sign of its value) if present, else all forward.

  WAVE       a travelling sine. Each motor's DESIRED position is
                 desired_i(t) = Waveamp * sin(2*pi*Wavefreq*t + phase_i)
             and we command the velocity that produces it -- the derivative
             (feedforward) plus a gentle proportional pull back toward the
             desired position using the POS feedback we already receive:
                 v_i = Waveamp*2*pi*Wavefreq*cos(...) + Kp*(desired_i - actual_i)
             Kp=0 is pure open-loop (may drift); a small Kp self-centers
             using MotorControllerEXT.actual_pos, no firmware feedback loop
             needed. Only WAVE uses feedback; CONSTANT/DIRECTION are meant to
             spin freely and stay pure open-loop.

Phase pattern (custom parameter Phasepattern: LINEAR | SYMMETRIC):
  LINEAR     phase_i = Wavephaseoffset * i           (wave travels 0 -> 13)
  SYMMETRIC  phase_i = Wavephaseoffset * dist_from_center(i)
             (wave emanates from center 6&7 outward to the ends 0&13)

Custom parameters on the owning COMP:
    Mode            menu    CONSTANT | DIRECTION | WAVE
    Basespeed       float   steps/sec, signed -- CONSTANT/DIRECTION speed
    Waveamp         float   steps -- WAVE peak position swing
    Wavefreq        float   Hz -- WAVE oscillation rate
    Wavephaseoffset float   radians per position step -- WAVE spatial phase
    Phasepattern    menu    LINEAR | SYMMETRIC
    Wavekp          float   position-correction gain for WAVE (0 = open-loop)
    Playback        toggle  enable/disable output

Optional child operator:
    direction       CHOP — DIRECTION mode: channel i's sign sets motor i's
                    direction. Missing -> all forward.
"""

import math

NUM_MOTORS = 14

# Reject dt gaps larger than this (seconds) in the WAVE phase accumulator --
# a paused/resumed project or a frame hitch would otherwise advance the
# phase by a huge step. ~0.1s comfortably exceeds a normal frame (16-50ms)
# while rejecting pause gaps.
MAX_DT = 0.1

# Distance from center stage per motor id, for SYMMETRIC phase. Motors 6 & 7
# are the center (phase 0), 0 & 13 the outer ends (phase 6). Matches the
# MOTOR_POSITION table in ChoreographyEXT -- verify against the real rig.
DIST_FROM_CENTER = [
    6, 5, 4, 3, 2, 1, 0,   # LEFT:  0-6
    0, 1, 2, 3, 4, 5, 6,   # RIGHT: 7-13
]


class ChoreographySpeedEXT:
    def __init__(self, ownerComp):
        self.ownerComp = ownerComp
        # WAVE phase is ACCUMULATED, not computed from absolute time, so a
        # live Wavefreq change only alters the rate of advance -- never the
        # phase value -- and the motion doesn't jump. See _updateWave.
        self._phase_accum = 0.0
        self._last_time = absTime.seconds

    # -- per-frame update --------------------------------------------------

    def Update(self):
        """Called every frame by the Execute DAT. Computes a speed per motor
        for the active mode and streams it via SetSpeed (change-detected, so
        an unchanged speed sends nothing)."""
        if not self.ownerComp.par.Playback.eval():
            return

        mode = self.ownerComp.par.Mode.eval()
        controller = self._motorController()

        if mode == 'CONSTANT':
            speed = self.ownerComp.par.Basespeed.eval()
            for i in range(NUM_MOTORS):
                if controller.homing[i]:
                    continue   # don't override an in-progress home -- its per-frame
                               # SETSPEED would cancel homing_active on the firmware
                controller.SetSpeed(i, speed)

        elif mode == 'DIRECTION':
            mag = abs(self.ownerComp.par.Basespeed.eval())
            direction = self.ownerComp.op('direction')
            for i in range(NUM_MOTORS):
                if controller.homing[i]:
                    continue
                sign = self._chanSign(direction, i)
                controller.SetSpeed(i, sign * mag)

        elif mode == 'WAVE':
            self._updateWave(controller)

    def _updateWave(self, controller):
        # Advance the shared phase accumulator by this frame's elapsed time.
        # Because phase is integrated (not phase = w*t), changing Wavefreq
        # changes only w here -- the accumulated phase stays continuous, so
        # no jump/jitter. A pause/resume or frame hitch (dt > MAX_DT) is
        # rejected so it can't lurch the phase forward.
        now = absTime.seconds
        dt = now - self._last_time
        self._last_time = now
        if dt < 0 or dt > MAX_DT:
            dt = 0.0

        freq = self.ownerComp.par.Wavefreq.eval()
        w = 2.0 * math.pi * freq
        self._phase_accum += w * dt

        amp = self.ownerComp.par.Waveamp.eval() / 10
        kp = self.ownerComp.par.Wavekp.eval()

        for i in range(NUM_MOTORS):
            if controller.homing[i]:
                continue   # don't override an in-progress home
            phase = self._phase_accum + self._phase(i)
            desired = amp * math.sin(phase)          # where motor i should be
            feedforward = amp * w * math.cos(phase)  # d/dt of desired (uses current freq)
            error = desired - controller.actual_pos[i]
            controller.SetSpeed(i, feedforward + kp * error)

    # -- helpers -------------------------------------------------------

    def _phase(self, motor_index):
        offset = self.ownerComp.par.Wavephaseoffset.eval()
        if self.ownerComp.par.Phasepattern.eval() == 'SYMMETRIC':
            return offset * DIST_FROM_CENTER[motor_index]
        return offset * motor_index  # LINEAR

    def _chanSign(self, chop, motor_index):
        """+1 / -1 from channel motor_index of a direction CHOP; +1 if the
        CHOP is missing or lacks that channel. Zero counts as forward."""
        if chop is None or motor_index >= chop.numChans:
            return 1
        return -1 if chop[motor_index][0] < 0 else 1

    # -- controls ------------------------------------------------------

    def ResetWaveClock(self):
        """Restart the WAVE phase at 0 (call when starting a fresh wave so
        it begins from phase 0)."""
        self._phase_accum = 0.0
        self._last_time = absTime.seconds

    def StopAll(self):
        """Stop output and all motors."""
        self.ownerComp.par.Playback = 0
        self._motorController().StopAll()

    # -- internals -----------------------------------------------------

    def _motorController(self):
        return self.ownerComp.parent().op('base_motor_controller').ext.MotorControllerEXT
