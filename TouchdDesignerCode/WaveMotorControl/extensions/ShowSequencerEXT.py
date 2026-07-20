import time


class ShowSequencerEXT:
    """
    Table-driven show sequencer for the SPEED choreography stack. Reads a
    Table DAT where each row is a timed segment and, each frame, drives the
    show by writing the custom parameters on base_choreography_speed (and
    calling base_motor_controller for home/stop actions). ChoreographySpeedEXT
    then reads those params and produces the motion -- so this owns "what the
    show should be doing at time t", the choreography owns "turn that into
    motor speeds".

    Table columns (header row must use these exact names):
      time_start   -- float seconds; when this segment begins
      duration     -- float seconds; over which numeric params interpolate
      behavior     -- the Mode (CONSTANT | DIRECTION | WAVE) OR an action
                      keyword: home / stop
      params_start -- space-separated Name=value pairs, applied at segment
                      entry (Names are ChoreographySpeedEXT custom params:
                      Basespeed, Waveamp, Wavefreq, Wavephaseoffset, Wavekp,
                      Constphaseoffset, Phasepattern)
      params_end   -- same; NUMERIC params interpolate start->end over
                      duration. Menu/string params (Phasepattern) are set
                      once at entry, not interpolated.

    Unlike the RobotRibbon original, ChoreographySpeedEXT runs one Mode at a
    time with no weight blending, so a 'behavior' (Mode) change is an INSTANT
    switch at segment entry -- there is no fadeTo crossfade. Smooth
    transitions come from interpolating numeric params within a segment (and
    the choreography's own phase accumulator keeps Wavefreq sweeps jump-free).

    Execute DAT order (one Execute DAT, in this order, so params land the
    same frame the choreography reads them):
        op('base_sequencer').ext.ShowSequencerEXT.Update()
        op('base_choreography_speed').ext.ChoreographySpeedEXT.Update()
    """

    def __init__(self, ownerComp):
        self.ownerComp = ownerComp

        self.seq_time      = 0.0
        self._prev_wall    = None
        self.playing       = False
        self.loop          = False
        self.duration      = 0.0

        self._segments     = []
        self._dat_name     = 'sequencer_table'
        self._last_seg_idx = -1
        self._debug        = False  # SetDebug(True) for verbose per-frame tracing

    # -------------------------
    # Update loop
    # -------------------------

    def Update(self):
        now = time.time()
        if self._prev_wall is None:
            self._prev_wall = now
            return
        dt = now - self._prev_wall
        if dt < 0.001:
            return
        self._prev_wall = now

        if not self.playing:
            return

        self.seq_time += dt

        if self.duration > 0 and self.seq_time >= self.duration:
            if self.loop:
                self.seq_time      %= self.duration
                self._last_seg_idx  = -1
            else:
                self.seq_time  = self.duration
                self.playing   = False

        self._evaluateSegments()

    # -------------------------
    # Playback controls
    # -------------------------

    def play(self):
        """Start or resume sequencer playback. Rewinds to t=0 if already at the end."""
        if self.duration > 0 and self.seq_time >= self.duration:
            self.seek(0.0)
        self._prev_wall = time.time()
        self.playing    = True

    def resume(self):
        """Resume playback from the current playhead position without rewinding."""
        self._prev_wall = time.time()
        self.playing    = True

    def pause(self):
        """Freeze the playhead. Motors hold whatever the current segment last
        commanded (velocity control -- they keep doing it). Use stop() to
        actually halt the rig."""
        self.playing = False

    def stop(self):
        """Pause, rewind to t=0, and stop all motion."""
        self.playing       = False
        self.seq_time      = 0.0
        self._last_seg_idx = -1
        ctrl = self._controllerExt()
        if ctrl:
            ctrl.StopAll()

    def seek(self, seconds):
        """Jump the playhead to an arbitrary time and immediately apply that segment."""
        self.seq_time      = max(0.0, float(seconds))
        self._last_seg_idx = -1
        self._evaluateSegments()

    def setLoop(self, loop):
        self.loop = bool(loop)

    def returnTime(self):
        return self.seq_time

    # -------------------------
    # Table loading
    # -------------------------

    def setDATName(self, name):
        """Set the op name of the Table DAT to read. Default: 'sequencer_table'."""
        self._dat_name = name

    def loadFromDAT(self, dat_op_name=None):
        """
        Parse the Table DAT and rebuild the segment list.
        Safe to call every frame for live editing.
        """
        name = dat_op_name or self._dat_name
        dat  = self.ownerComp.op(name) or self.ownerComp.parent().op(name)
        if not dat:
            print(f'[Sequencer] Table DAT "{name}" not found.')
            return

        segments = []
        for row in range(1, dat.numRows):
            def cell(col):
                try:
                    v = str(dat[row, col])
                    return v if v not in ('', 'None') else ''
                except Exception:
                    return ''

            time_start_s = cell('time_start')
            behavior_s   = cell('behavior').strip()

            if not time_start_s or not behavior_s:
                continue

            try:
                time_start = float(time_start_s)
                duration_s = cell('duration')
                duration   = float(duration_s) if duration_s else 0.0
            except ValueError:
                print(f'[Sequencer] Row {row}: invalid time_start or duration -- skipped.')
                continue

            segments.append({
                'time_start':   time_start,
                'duration':     max(0.0, duration),
                'behavior':     behavior_s,
                'params_start': self._parseParams(cell('params_start')),
                'params_end':   self._parseParams(cell('params_end')),
            })

        self._segments     = sorted(segments, key=lambda s: s['time_start'])
        self._last_seg_idx = -1

        if self._segments:
            last = self._segments[-1]
            self.duration = last['time_start'] + last['duration']
        else:
            self.duration = 0.0

        print(f'[Sequencer] Loaded {len(self._segments)} segment(s), '
              f'duration={self.duration:.2f}s')

    def reloadDAT(self):
        """Re-parse the table DAT without changing playback state."""
        self.loadFromDAT()

    # -------------------------
    # Internal -- evaluation
    # -------------------------

    def _evaluateSegments(self):
        if not self._segments:
            if self._debug:
                print('[Sequencer] _evaluateSegments: NO SEGMENTS LOADED '
                      '(call loadFromDAT()).')
            return
        idx = self._findActiveSegment(self.seq_time)
        if idx < 0:
            if self._debug:
                print(f'[Sequencer] no active segment at t={self.seq_time:.2f} '
                      f'(first segment starts at {self._segments[0]["time_start"]}s).')
            return

        seg      = self._segments[idx]
        elapsed  = self.seq_time - seg['time_start']
        progress = min(1.0, elapsed / seg['duration']) if seg['duration'] > 0 else 1.0

        if idx != self._last_seg_idx:
            self._transitionToSegment(seg)
            self._last_seg_idx = idx

        self._updateParams(seg, progress)

    def _findActiveSegment(self, t):
        active = -1
        for i, seg in enumerate(self._segments):
            if seg['time_start'] <= t:
                active = i
            else:
                break
        return active

    def _transitionToSegment(self, seg):
        """
        Called once on entering a new segment. Action keywords fire on the
        motor controller; otherwise 'behavior' is a Mode -- set it, apply
        params_start, and enable Playback so the choreography produces output.
        """
        behavior = seg['behavior']

        # Action keywords (accept the RobotRibbon aliases too).
        if behavior in ('home', 'motor_homing'):
            ctrl = self._controllerExt()
            if ctrl:
                ctrl.HomeAll()
            print(f'[Sequencer] -> home  t={self.seq_time:.2f}s')
            return
        if behavior in ('stop', 'zeroAll'):
            ctrl = self._controllerExt()
            if ctrl:
                ctrl.StopAll()
            print(f'[Sequencer] -> stop  t={self.seq_time:.2f}s')
            return

        # Otherwise behavior is a Mode (CONSTANT / DIRECTION / WAVE).
        choreo = self._choreoComp()
        if choreo is None:
            print(f'[Sequencer] !! base_choreography_speed NOT FOUND (looked for a '
                  f'sibling of {self.ownerComp.path}) -- cannot drive "{behavior}"')
            return
        print(f'[Sequencer] -> Mode={behavior}  t={self.seq_time:.2f}s  '
              f'writing to {choreo.path}')
        self._setPar(choreo, 'Mode', behavior)
        for key, value in seg['params_start'].items():
            self._setPar(choreo, key, value)
        # StopAll (from a previous 'stop' segment) turns Playback off, so
        # re-enable it whenever a motion segment begins.
        self._setPar(choreo, 'Playback', 1)

    def _updateParams(self, seg, progress):
        """Every frame: interpolate NUMERIC params_start -> params_end over the
        segment. Menu/string params were set once at entry."""
        if seg['behavior'] in ('home', 'motor_homing', 'stop', 'zeroAll'):
            return
        choreo = self._choreoComp()
        if choreo is None:
            return
        p_start  = seg['params_start']
        p_end    = seg['params_end']
        for key in set(p_start.keys()) | set(p_end.keys()):
            v_start = p_start.get(key, p_end.get(key, 0.0))
            v_end   = p_end.get(key,   p_start.get(key, 0.0))
            if isinstance(v_start, str) or isinstance(v_end, str):
                continue  # menu/string (e.g. Phasepattern) -- set once at entry
            self._setPar(choreo, key, v_start + (v_end - v_start) * progress)

    @staticmethod
    def _parseParams(s):
        """Parse 'key=value key2=value2 ...' into {key: float | str}."""
        if not s or not s.strip():
            return {}
        result = {}
        for pair in s.split():
            if '=' in pair:
                key, _, val = pair.partition('=')
                key = key.strip()
                val = val.strip()
                try:
                    result[key] = float(val)
                except ValueError:
                    result[key] = val  # non-numeric (e.g. Phasepattern=SYMMETRIC)
        return result

    # -------------------------
    # Diagnostics
    # -------------------------

    def listSegments(self):
        for i, s in enumerate(self._segments):
            print(f'  [{i}] t={s["time_start"]}  dur={s["duration"]}  '
                  f'behavior={s["behavior"]}')

    # -------------------------
    # Op references / helpers
    # -------------------------

    def _setPar(self, comp, key, value):
        """Set a custom parameter by name, skipping (with a warning) any the
        COMP doesn't have -- a table typo can never crash Update()."""
        par = getattr(comp.par, key, None)
        if par is None:
            print(f'[Sequencer] !! parameter "{key}" MISSING on {comp.path} -- skipped')
            return
        par.val = value
        if self._debug:
            print(f'[Sequencer]    set {comp.name}.par.{key} = {value}')

    # -------------------------
    # Debugging
    # -------------------------

    def SetDebug(self, on=True):
        """Toggle verbose per-frame tracing (param writes, empty-segment /
        no-active-segment notices). Off by default to avoid textport spam."""
        self._debug = bool(on)

    def Diagnose(self):
        """One-shot state dump -- call from the textport to see why the show
        isn't reaching base_choreography_speed."""
        print('=== ShowSequencer Diagnose ===')
        print(f'  ownerComp (this ext)  : {self.ownerComp.path}')
        parent = self.ownerComp.parent()
        print(f'  ownerComp.parent()    : {parent.path if parent else None}')

        choreo = self._choreoComp()
        print(f'  base_choreography_speed: {choreo.path if choreo else "!! NOT FOUND (must be a sibling of base_sequencer)"}')
        if choreo:
            for p in ('Mode', 'Playback', 'Basespeed', 'Constphaseoffset',
                      'Waveamp', 'Wavefreq', 'Wavephaseoffset', 'Wavekp', 'Phasepattern'):
                par = getattr(choreo.par, p, None)
                print(f'      par.{p:16}: {"!! MISSING" if par is None else par.eval()}')

        ctrl = self.ownerComp.parent().op('base_motor_controller')
        print(f'  base_motor_controller : {ctrl.path if ctrl else "!! NOT FOUND"}')

        print(f'  segments loaded       : {len(self._segments)}'
              + ('  (call loadFromDAT())' if not self._segments else ''))
        print(f'  playing               : {self.playing}')
        print(f'  seq_time / duration   : {self.seq_time:.2f} / {self.duration:.2f}')
        if self._segments:
            idx = self._findActiveSegment(self.seq_time)
            if idx >= 0:
                print(f'  active segment        : [{idx}] behavior={self._segments[idx]["behavior"]}')
            else:
                print(f'  active segment        : none yet (first at {self._segments[0]["time_start"]}s)')
        print('==============================')

    def _choreoComp(self):
        return self.ownerComp.parent().op('base_choreography_speed')

    def _controllerExt(self):
        c = self.ownerComp.parent().op('base_motor_controller')
        if not c:
            print('[Sequencer] base_motor_controller not found')
            return None
        return c.ext.MotorControllerEXT
