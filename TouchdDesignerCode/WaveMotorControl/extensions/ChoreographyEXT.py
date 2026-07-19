"""
ChoreographyEXT — computes ideal motor positions every frame and pushes
them to MotorControllerEXT. Does NOT talk to serial/network directly, and
does NOT run PID — purely "what should each motor's target be right now."

Driven by an Execute DAT (Frame Start enabled) calling Update() every
frame — see dats/choreo_execute.py. The spec called for a "CHOP Execute DAT
at cook rate," but TD's actual mechanism for "run this every frame,
unconditionally" is a plain Execute DAT with Frame Start, not a CHOP
Execute watching some channel — same category of correction as the Serial
DAT callback earlier in this project.

Custom parameters on the owning COMP:
    Waveamplitude     int     steps -- peak displacement from home
    Wavefrequency     float   Hz
    Wavephaseoffset   float   radians -- spatial phase between adjacent motors
    Wavemode          menu    SINE | TRIANGLE | CUSTOM
    Constantspeed     float   steps/sec, signed -- global speed for the 'constant'
                              cue type (continuous rotation). Live-adjustable. Used
                              as a fallback when no per-motor 'custom_speed' CHOP is
                              present (see Update()).
    Playback          toggle  enable/disable output
    Activecue         int     current cue index (read-only display; set via GoCue)

Optional child operator:
    custom_speed      CHOP — if present, its channel i drives motor i's speed
                      during a 'constant' cue instead of the global Constantspeed
                      param. This is the audio-reactivity hook: wire an audio
                      analysis CHOP (Audio Device In -> Analyze/Spectrum, scaled
                      to steps/sec) into it and continuous rotation follows the
                      music per-motor. Falls back to Constantspeed for any channel
                      it doesn't provide.

Note on 'constant' cues: these use SETSPEED (the firmware's continuous-
rotation mode), NOT SETPOS. There is no position target tracked on the TD
side at all while spinning -- the motor just runs at the commanded speed,
and drift is corrected entirely on the Arduino by resyncing against the
homing sensor once per revolution (see resyncContinuousPosition in
motor_controller.ino). This sidesteps the position-wraparound problem an
absolute-position approach would have had.
"""

import math

NUM_MOTORS = 14

# Physical distance from center stage, one entry per global motor id
# (0-13). Motors 0-6 = LEFT, 7-13 = RIGHT, mirrored from center outward:
# index 0 and index 7 are both treated as nearest-center (same phase),
# index 6 and 13 as both outermost. VERIFY THIS against your actual rig --
# I don't know which physical end each Mega's local id 0 is wired to, so
# this is a best-guess default, not a measurement. Edit directly if a side
# is backwards (e.g. reverse one half's ordering).
MOTOR_POSITION = [
    6, 5, 4, 3, 2, 1, 0,   # LEFT:  0-6  (6 = outermost, 0 = nearest center)
    0, 1, 2, 3, 4, 5, 6,   # RIGHT: 7-13 (7 = nearest center, 13 = outermost)
]


class ChoreographyEXT:
    def __init__(self, ownerComp):
        self.ownerComp = ownerComp
        self._cue_start_time = 0.0

        # Each cue may optionally carry 'maxspeed' (steps/sec) and 'accel'
        # (steps/sec^2). When present, GoCue pushes them to the Megas via
        # SETMAXSPEED/SETACCEL as the cue activates -- so a cue can raise its
        # own governor for fast motion, or lower it for gentle motion,
        # without a separate manual command. Only effective on the PID-free
        # firmware (motor_controller_without_pid.ino), which is the one that
        # accepts SETMAXSPEED/SETACCEL. Omit these keys to leave the current
        # limits untouched when the cue activates.
        self.cues = [
            {'name': 'HOME',        'type': 'home'},
            {'name': 'FREEZE',      'type': 'hold'},
            {'name': 'WAVE_SLOW',   'type': 'wave', 'amplitude': 800, 'frequency': 0.2, 'phase': 0.4,
                                    'maxspeed': 2000, 'accel': 800},
            {'name': 'WAVE_FAST',   'type': 'wave', 'amplitude': 400, 'frequency': 0.8, 'phase': 0.6,
                                    'maxspeed': 6000, 'accel': 3000},
            {'name': 'UNISON_UP',   'type': 'absolute', 'positions': [800] * NUM_MOTORS,
                                    'maxspeed': 4000, 'accel': 1500},
            {'name': 'UNISON_DOWN', 'type': 'absolute', 'positions': [-800] * NUM_MOTORS,
                                    'maxspeed': 4000, 'accel': 1500},
            {'name': 'SPIN',        'type': 'constant', 'maxspeed': 8000},
        ]

    # -- per-frame update --------------------------------------------------

    def Update(self):
        """Called every frame by the Execute DAT. Streams the active cue's
        per-frame output. Only 'wave' and 'constant' cues need per-frame
        work; 'home'/'hold'/'absolute' are one-shot (handled in GoCue)."""
        if not self.ownerComp.par.Playback.eval():
            return

        cue = self.cues[self.ownerComp.par.Activecue.eval()]
        controller = self._motorController()

        if cue['type'] == 'wave':
            t = absTime.seconds - self._cue_start_time
            for i in range(NUM_MOTORS):
                controller.SetIdealPos(i, self.ComputeWavePos(i, t))

        elif cue['type'] == 'constant':
            # Per-motor speed from the custom_speed CHOP if present (audio
            # reactivity hook), else the global Constantspeed slider.
            # SetSpeed change-detects, so calling every frame only actually
            # sends SETSPEED when a motor's speed changes.
            custom = self.ownerComp.op('custom_speed')
            default_speed = self.ownerComp.par.Constantspeed.eval()
            for i in range(NUM_MOTORS):
                speed = custom[i][0] if (custom and i < custom.numChans) else default_speed
                controller.SetSpeed(i, speed)

    # -- wave computation --------------------------------------------------

    def ComputeWavePos(self, motor_index, t):
        spatial_phase = MOTOR_POSITION[motor_index] * self.ownerComp.par.Wavephaseoffset.eval()
        amplitude = self.ownerComp.par.Waveamplitude.eval()
        frequency = self.ownerComp.par.Wavefrequency.eval()
        phase = 2 * math.pi * frequency * t + spatial_phase
        mode = self.ownerComp.par.Wavemode.eval()

        if mode == 'SINE':
            normalized = math.sin(phase)
        elif mode == 'TRIANGLE':
            cycle = (phase / (2 * math.pi)) % 1.0
            normalized = 2 * abs(2 * cycle - 1) - 1
        elif mode == 'CUSTOM':
            # Sample channel `motor_index` of a CHOP named 'custom_wave', if
            # present, so motion can be designed with TD's native CHOP tools
            # (Wave CHOP, LFO CHOP, audio-reactive, etc.) instead of only
            # Python-coded shapes. Silently 0 if not wired up yet.
            custom = self.ownerComp.op('custom_wave')
            normalized = custom[motor_index][0] if custom else 0.0
        else:
            normalized = 0.0

        return int(normalized * amplitude)

    # -- cue system ----------------------------------------------------

    def GoCue(self, cue_index):
        if cue_index < 0 or cue_index >= len(self.cues):
            debug(f"[ChoreographyEXT] GoCue: index {cue_index} out of range")
            return

        cue = self.cues[cue_index]
        self.ownerComp.par.Activecue = cue_index
        self._cue_start_time = absTime.seconds

        controller = self._motorController()

        # Apply this cue's speed/accel limits first, so the governor is in
        # place before any motion is commanded below (matters for SPIN in
        # particular -- the ceiling must be up before Update() commands the
        # speed, or the first SETSPEED would be clamped low).
        self._applyCueLimits(cue, controller)

        if cue['type'] == 'home':
            controller.HomeAll()
        elif cue['type'] == 'wave':
            self.ownerComp.par.Waveamplitude = cue['amplitude']
            self.ownerComp.par.Wavefrequency = cue['frequency']
            self.ownerComp.par.Wavephaseoffset = cue['phase']
        elif cue['type'] == 'absolute':
            for i, pos in enumerate(cue['positions']):
                controller.SetIdealPos(i, pos)
        elif cue['type'] == 'hold':
            # Freeze in place. A position-mode motor already holds its last
            # SETPOS, so nothing to do -- but a motor still in continuous
            # rotation would keep spinning, so explicitly command speed 0
            # (stays in speed mode, just stopped).
            for i in range(NUM_MOTORS):
                if controller.speed_mode[i]:
                    controller.SetSpeed(i, 0)
        # 'constant' needs no GoCue action -- Update() drives it every frame
        # from Constantspeed / the custom_speed CHOP.

        controller.LogEvent(f"Cue -> {cue['name']}")

    def _applyCueLimits(self, cue, controller):
        """Push a cue's optional maxspeed/accel to all motors as it activates.
        Silently does nothing for keys the cue omits, leaving those limits
        as they were. No-op on the PID firmware (it ignores SETMAXSPEED/
        SETACCEL)."""
        if 'maxspeed' in cue:
            controller.SetMaxSpeed(cue['maxspeed'])
        if 'accel' in cue:
            controller.SetAccel(cue['accel'])

    def NextCue(self):
        self.GoCue(min(self.ownerComp.par.Activecue.eval() + 1, len(self.cues) - 1))

    def PrevCue(self):
        self.GoCue(max(self.ownerComp.par.Activecue.eval() - 1, 0))

    def GoCueByName(self, name):
        for i, cue in enumerate(self.cues):
            if cue['name'] == name:
                self.GoCue(i)
                return
        debug(f"[ChoreographyEXT] GoCueByName: unknown cue '{name}'")

    # -- internals -----------------------------------------------------

    def _motorController(self):
        return self.ownerComp.parent().op('base_motor_controller').ext.MotorControllerEXT
