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
    Constantspeed     float   steps/sec, signed -- slider from -limit to +limit for
                              the 'constant' cue type (continuous unbounded rotation,
                              see the 'SPIN' cue and Update()/GoCue() below). Live --
                              can be adjusted while a constant-motion cue is playing.
    Playback          toggle  enable/disable output
    Activecue         int     current cue index (read-only display; set via GoCue)

Note on 'constant' cues: ideal_pos is deliberately left UNBOUNDED (never
wrapped with modulo) for this cue type. SETPOS is an absolute-position
command, and AccelStepper's moveTo() always takes the shortest arithmetic
path to a new absolute target -- wrapping the position with modulo would
make the target jump from e.g. 1599 back to 0 every revolution, and the
firmware would drive the motor BACKWARD almost a full turn to reach "0"
the short way, instead of continuing forward by the 1 step that actually
completes the revolution. Letting ideal_pos climb without bound avoids
this entirely and matches how SETPOS already works; int32 has room for
well over a million revolutions before it would matter.
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
        self._constant_start_pos = [0] * NUM_MOTORS  # snapshot taken in GoCue() when entering a 'constant' cue

        self.cues = [
            {'name': 'HOME',        'type': 'home'},
            {'name': 'FREEZE',      'type': 'hold'},
            {'name': 'WAVE_SLOW',   'type': 'wave', 'amplitude': 800, 'frequency': 0.2, 'phase': 0.4},
            {'name': 'WAVE_FAST',   'type': 'wave', 'amplitude': 400, 'frequency': 0.8, 'phase': 0.6},
            {'name': 'UNISON_UP',   'type': 'absolute', 'positions': [800] * NUM_MOTORS},
            {'name': 'UNISON_DOWN', 'type': 'absolute', 'positions': [-800] * NUM_MOTORS},
            {'name': 'SPIN',        'type': 'constant'},
        ]

    # -- per-frame update --------------------------------------------------

    def Update(self):
        """Called every frame by the Execute DAT. Recomputes and streams
        ideal positions for whichever cue is active."""
        if not self.ownerComp.par.Playback.eval():
            return

        cue = self.cues[self.ownerComp.par.Activecue.eval()]
        if cue['type'] != 'wave':
            return  # 'home'/'hold'/'absolute' need no per-frame recompute

        t = absTime.seconds - self._cue_start_time
        controller = self._motorController()
        for i in range(NUM_MOTORS):
            controller.SetIdealPos(i, self.ComputeWavePos(i, t))
     


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

        if cue['type'] == 'home':
            controller.HomeAll()
        elif cue['type'] == 'wave':
            self.ownerComp.par.Waveamplitude = cue['amplitude']
            self.ownerComp.par.Wavefrequency = cue['frequency']
            self.ownerComp.par.Wavephaseoffset = cue['phase']
        elif cue['type'] == 'absolute':
            for i, pos in enumerate(cue['positions']):
                controller.SetIdealPos(i, pos)
        # 'hold' needs no action here -- Update() already skips recomputing
        # positions for any non-'wave' cue, so motors just stay wherever
        # they are.

        controller.LogEvent(f"Cue -> {cue['name']}")

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
