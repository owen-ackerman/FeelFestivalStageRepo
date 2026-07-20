import time


class ShowLooperEXT:
    """
    Timed show looper for the SPEED stack. Drives the motor controller and the
    sequencer through a fixed repeating schedule so an installation can run on
    a loop unattended:

      t = HOMING_TIME  ->  home:  stop all, (re)load the show table, home motors
      t = PLAY_TIME    ->  play:  sequencer starts driving the choreography
      t = STOP_TIME    ->  stop:  sequencer stops, all motors stop
      t = LOOP_PERIOD  ->  loop:  restart the cycle from t=0

    (Audio and lighting are intentionally NOT handled here -- this build only
    orchestrates motion.)

    Setup:
      1. Attach this extension to a COMP (e.g. base_looper), a sibling of
         base_motor_controller, base_choreography_speed, and base_sequencer.
      2. Call Update() every frame from an Execute DAT:
             op('base_looper').ext.ShowLooperEXT.Update()
      3. start(show_table_name) to begin, stop() to halt.

    The times below are placeholders -- set them to your show length. Make
    PLAY_TIME comfortably longer than a full home so the show doesn't begin
    mid-homing (the choreography skips still-homing motors regardless, but a
    clean start is nicer).
    """

    HOMING_TIME = 0
    PLAY_TIME   = 10
    STOP_TIME   = 475
    LOOP_PERIOD = 595  # show length + cooldown before the next cycle

    def __init__(self, ownerComp):
        self.ownerComp      = ownerComp
        self._running       = False
        self._show_name     = 'sequencer_table'  # Table DAT the sequencer loads
        self._looper_time   = 0.0
        self._prev_wall     = None
        self._current_phase = None  # 'homing' | 'playing' | 'cooling'
        self._time_chop     = 'const_looper_time'  # optional Constant CHOP inside ownerComp

    # -------------------------
    # Update loop
    # -------------------------

    def Update(self):
        if not self._running:
            return

        now = time.time()
        if self._prev_wall is None:
            self._prev_wall = now
            return

        dt = now - self._prev_wall
        if dt < 0.001:
            return
        self._prev_wall    = now
        self._looper_time += dt

        if self._looper_time >= self.LOOP_PERIOD:
            self._looper_time   = 0.0
            self._current_phase = None  # allow re-entry into homing on next eval

        t = self._looper_time
        if t < self.PLAY_TIME:
            new_phase = 'homing'
        elif t < self.STOP_TIME:
            new_phase = 'playing'
        else:
            new_phase = 'cooling'

        if new_phase != self._current_phase:
            self._current_phase = new_phase
            if new_phase == 'homing':
                self._onHoming()
            elif new_phase == 'playing':
                self._onPlay()
            elif new_phase == 'cooling':
                self._onStop()

        self._pushToCHOP()

    # -------------------------
    # Public controls
    # -------------------------

    def start(self, show_name=None):
        """Begin the loop from t=0. show_name is the sequencer's Table DAT op name."""
        if show_name:
            self._show_name = show_name
        self._looper_time   = 0.0
        self._prev_wall     = None
        self._current_phase = None
        self._running       = True
        print(f'[ShowLooper] Starting -- show: {self._show_name}')

    def stop(self):
        """Halt the looper and stop all motion immediately."""
        self._running = False
        self._onStop()
        print('[ShowLooper] Stopped.')

    def pause(self):
        """Freeze the looper clock. Does not stop motion on its own -- call
        stop() for that."""
        self._running = False
        print(f'[ShowLooper] Paused at t={self._looper_time:.1f}s.')

    def resume(self):
        """Resume the looper clock from the current position."""
        self._prev_wall = None
        self._running   = True
        print(f'[ShowLooper] Resumed at t={self._looper_time:.1f}s.')

    def setShowName(self, show_name):
        """Change the sequencer Table DAT name loaded on the next homing phase."""
        self._show_name = show_name

    def setTimeCHOP(self, chop_name):
        """Set the Constant CHOP (inside ownerComp) that receives elapsed time."""
        self._time_chop = chop_name

    @property
    def elapsed(self):
        """Current position within the loop cycle (seconds)."""
        return self._looper_time

    # -------------------------
    # Phase handlers
    # -------------------------

    def _onHoming(self):
        print(f'[ShowLooper] t={self._looper_time:.1f}s -- homing')
        seq  = self._seq()
        ctrl = self._ctrl()
        if seq:
            seq.stop()                        # resets the playhead + issues StopAll
            seq.loadFromDAT(self._show_name)
        elif ctrl:
            ctrl.StopAll()                    # no sequencer -> still stop before homing
        if ctrl:
            ctrl.HomeAll()                    # LAST, so a StopAll above can't cancel it

    def _onPlay(self):
        print(f'[ShowLooper] t={self._looper_time:.1f}s -- play')
        seq = self._seq()
        if seq:
            seq.play()                        # sequencer drives the choreography (enables Playback per segment)

    def _onStop(self):
        print(f'[ShowLooper] t={self._looper_time:.1f}s -- stop')
        seq  = self._seq()
        ctrl = self._ctrl()
        if seq:
            seq.stop()                        # also StopAll
        elif ctrl:
            ctrl.StopAll()

    # -------------------------
    # CHOP output
    # -------------------------

    def _pushToCHOP(self):
        chop = self.ownerComp.op(self._time_chop)
        if chop:
            par = getattr(chop.par, 'const0value', None)
            if par is not None:
                par.val = self._looper_time

    # -------------------------
    # Op helpers
    # -------------------------

    def _ctrl(self):
        c = self.ownerComp.parent().op('base_motor_controller')
        if not c:
            print('[ShowLooper] base_motor_controller not found')
            return None
        return c.ext.MotorControllerEXT

    def _seq(self):
        s = self.ownerComp.parent().op('base_sequencer')
        if not s:
            print('[ShowLooper] base_sequencer not found')
            return None
        return s.ext.ShowSequencerEXT
