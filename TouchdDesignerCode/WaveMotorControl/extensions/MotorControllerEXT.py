"""
MotorControllerEXT — central state manager for all 14 motors.

Does NOT run PID — that lives entirely on the Arduino (see
motor_controller.ino). This holds ideal positions, streams them to the
Megas, receives actual positions for monitoring, and provides safety
fallbacks (drift-triggered re-home, emergency stop).

Expected to live on base_motor_controller, a sibling of base_serial_left
and base_serial_right under base_motor_system (see README_TD_SETUP.md for
the full operator tree).

Custom parameters on the owning COMP:
    Autorehomedrift   int    default 200   — steps of drift that triggers a TD-side re-home
    Pidkp             float  default 0.8   — display mirror of the last Kp sent via SETPID
    Pidki             float  default 0.01  — display mirror of the last Ki sent via SETPID
    Pidkd             float  default 0.1   — display mirror of the last Kd sent via SETPID
    (TD's custom parameter Name field doesn't allow underscores -- these are
    Pidkp/Pidki/Pidkd, not Pid_kp/Pid_ki/Pid_kd; give them whatever Label
    you like in the UI, e.g. "Pid Kp", the Name is what code reads)

Child operator (optional but recommended):
    motor_state_table   Table DAT — 15 rows (header + 14 motors), rebuilt by
                         _updateStateTable() as reports come in.
    event_log           Table DAT — timestamp | message, appended to by
                         LogEvent(). If absent, LogEvent() just falls back
                         to debug() console output.
    error_chop          Constant CHOP — 14 channels, one per motor, holding
                         the live error (ideal - actual) for visualization.
                         Pre-configure it in TD with 14 channels (name them
                         however you like, e.g. m0..m13); _updateErrorCHOP()
                         only writes the values via the const{N}value params.
                         If absent or with fewer channels, writing is a
                         no-op — nothing breaks, that motor just isn't shown.
"""

NUM_MOTORS = 14

# Must match the firmware's STEPS_PER_REV #define. Used to interpret a
# zero-crossing during continuous rotation: the count is a whole-revolution
# multiple, so the real per-rev drift is the remainder (see OnZeroCross).
STEPS_PER_REV = 1600

# TD-side homing watchdog. If a motor's homing flag is set but no HOMED/FAULT
# reply arrives within this many seconds, clear it automatically (see
# IsHoming) -- a lost reply (link drop, offline test) must never leave the
# choreography's homing-skip guard blocking the motor forever. Must exceed
# the firmware's HOMING_TIMEOUT_MS (15s) plus reply latency.
HOMING_TIMEOUT_S = 20.0


class MotorControllerEXT:
    def __init__(self, ownerComp):
        self.ownerComp = ownerComp

        self.ideal_pos  = [0] * NUM_MOTORS    # steps — set by ChoreographyEXT/manual jog, sent to Mega
        self.actual_pos = [0] * NUM_MOTORS    # steps — received from Mega (monitoring only)
        self.homed      = [False] * NUM_MOTORS
        self.homing     = [False] * NUM_MOTORS
        self._homing_start = [0.0] * NUM_MOTORS   # absTime when each home began, for the watchdog
        self.faults     = [None] * NUM_MOTORS

        # Continuous-rotation (SETSPEED) tracking, mirroring the firmware's
        # per-motor speed_mode. speed_mode[i] True means this motor is
        # spinning under SETSPEED, NOT tracking a position target -- which
        # matters for two things: change-detection on the commanded speed,
        # and suppressing the drift-triggered re-home in UpdateActualPos
        # (in speed mode actual_pos climbs without bound while ideal_pos is
        # stale, so the drift check would otherwise fire a spurious re-home
        # almost immediately).
        self.motor_speed = [0] * NUM_MOTORS   # signed steps/sec last sent via SETSPEED
        self.motor_speed_pos = [0] * NUM_MOTORS   # signed steps/sec last sent via SETSPEED
        self.speed_mode  = [False] * NUM_MOTORS

    # -- logging -------------------------------------------------------

    def LogEvent(self, message):
        debug(f"[MotorControllerEXT] {message}")
        log_dat = self.ownerComp.op('event_log')
        if log_dat is not None:
            log_dat.appendRow([str(absTime.seconds), message])

    # -- routing -------------------------------------------------------

    def GetSerialEXT(self, motor_id):
        """
        Route command to correct Mega based on motor_id.

        Uses comp.extensions[0] rather than a hardcoded .ext.SerialEXT so
        this works whether base_serial_left/right is running SerialEXT
        (local Serial DAT — direct bench testing) or SerialRelayEXT (TCP to
        the mini PC bridge — full show rig). Both expose the same public
        interface via SerialProtocolBase, so callers here don't need to
        know which one is attached.
        """
        parent = self.ownerComp.parent()
        comp_name = 'base_serial_left' if motor_id < 7 else 'base_serial_right'
        return parent.op(comp_name).extensions[0]

    # -- position streaming ---------------------------------------------

    def SetIdealPos(self, motor_id, steps):
        """
        Called every cook by ChoreographyEXT (or manually, e.g. from a CHOP
        jog script) to update one motor's ideal position. Only sends SETPOS
        if the value actually changed, to avoid flooding serial with
        identical values from a 60fps cook.
        """
        steps = int(steps)
        # Leaving continuous-speed mode -- mirrors the firmware's cmdSetPos,
        # which clears speed_mode on any SETPOS.
        self.speed_mode[motor_id] = False
        if self.ideal_pos[motor_id] == steps:
            return
        self.ideal_pos[motor_id] = steps
        self.GetSerialEXT(motor_id).SendSetPos(motor_id, steps)
        self._updateStateTable(motor_id)

    def SetSpeed(self, motor_id, steps_per_sec):
        """
        Put one motor into continuous-rotation mode at a signed speed
        (steps/sec; sign = direction, 0 = stop but stay in speed mode).
        Only sends SETSPEED when the value actually changes, so this is safe
        to call every frame from a live slider / audio CHOP. Mutually
        exclusive with position mode -- mirrors the firmware.
        """
        steps_per_sec = int(steps_per_sec)
        self.speed_mode[motor_id] = True
        if self.motor_speed[motor_id] == steps_per_sec:
            return
        self.motor_speed[motor_id] = steps_per_sec
        self.GetSerialEXT(motor_id).SendSetSpeed(motor_id, steps_per_sec)
        self._updateStateTable(motor_id)

    def SetSpeedPos(self, motor_id, pos):
        """
        Put one motor into continuous-rotation mode at a signed speed
        (steps/sec; sign = direction, 0 = stop but stay in speed mode).
        Only sends SETSPEED when the value actually changes, so this is safe
        to call every frame from a live slider / audio CHOP. Mutually
        exclusive with position mode -- mirrors the firmware.
        """
        
        
        if self.motor_speed_pos[motor_id] == pos:
            return
        self.motor_speed_pos[motor_id] = pos
        self._updateStateTable(motor_id)




    def SetMaxSpeed(self, value, motor_id=None):
        """
        Set max speed (steps/sec) on the Megas. Only the PID-free firmware
        (motor_controller_without_pid.ino) acts on this; the PID build
        ignores it. motor_id=None applies to all 14 motors (one command per
        Mega); otherwise just that motor's Mega. Not change-detected -- send
        it when the operator changes the value, e.g. from a slider callback,
        not every frame.
        """
        if motor_id is None:
            self.GetSerialEXT(0).SendSetMaxSpeed(value)
            self.GetSerialEXT(7).SendSetMaxSpeed(value)
            self.LogEvent(f"SETMAXSPEED {value} sent to all motors")
        else:
            self.GetSerialEXT(motor_id).SendSetMaxSpeed(value, motor_id)
            self.LogEvent(f"SETMAXSPEED {value} sent to motor {motor_id}")

    def SetAccel(self, value, motor_id=None):
        """
        Set max acceleration (steps/sec^2) on the Megas. PID-free firmware
        only. motor_id=None applies to all 14 motors; otherwise just that
        motor's Mega. Acceleration only affects SETPOS moves (run()), not
        SETSPEED continuous rotation (runSpeed() ignores acceleration).
        """
        if motor_id is None:
            self.GetSerialEXT(0).SendSetAccel(value)
            self.GetSerialEXT(7).SendSetAccel(value)
            self.LogEvent(f"SETACCEL {value} sent to all motors")
        else:
            self.GetSerialEXT(motor_id).SendSetAccel(value, motor_id)
            self.LogEvent(f"SETACCEL {value} sent to motor {motor_id}")

    def UpdateActualPos(self, motor_id, steps):
        """
        Called by SerialEXT when a POS message is received. Updates
        monitoring state and checks the drift safety fallback. Does NOT run
        PID — that's the Arduino's job.
        """
        self.actual_pos[motor_id] = steps
        self._updateStateTable(motor_id)

        # In continuous-speed mode there is no position target -- actual_pos
        # climbs without bound, so the ideal-vs-actual drift check is
        # meaningless and would fire a spurious re-home every report.
        if self.speed_mode[motor_id]:
            return

        threshold = self.ownerComp.par.Autorehomedrift.eval()
        drift = abs(self.ideal_pos[motor_id] - steps)
        if drift > threshold and not self.homing[motor_id]:
            self.LogEvent(
                f"Motor {motor_id} drift {drift} exceeds Autorehomedrift "
                f"{threshold} — triggering safety re-home"
            )
            self.HomeMotor(motor_id)

    def OnHomed(self, motor_id):
        """Sensor crossed, position and ideal reset to 0."""
        self.homed[motor_id] = True
        self.homing[motor_id] = False
        self.actual_pos[motor_id] = 0
        self.ideal_pos[motor_id] = 0
        # Firmware zeroed this motor's speed when it homed. Reset our
        # SetSpeed change-detection cache to match, or the next SetSpeed with
        # the pre-home value would be suppressed as "unchanged" and never
        # reach the now-stopped firmware -- i.e. motors that won't restart
        # after homing.
        self.motor_speed[motor_id] = 0
        self.faults[motor_id] = None
        self.LogEvent(f"Motor {motor_id} HOMED")
        self._updateStateTable(motor_id)
        # NOTE: do NOT send RESETPID/SETPID here. The PID firmware already
        # zeroes its own integral/derivative on the home crossing
        # (resetHomePosition), so this was always redundant -- and on the
        # non-PID firmwares (without_pid / simple / speed) it just produces
        # "ERR unknown command: SETPID" noise. Firmware owns its own PID
        # state reset on home.

    def OnZeroCross(self, motor_id, firmware_count):
        """
        Called by SerialEXT on a ZERO message from speed_motor_controller:
        the motor physically crossed its home sensor mid-run, so its TRUE
        position is 0 right now. Re-anchor our estimate to that physical
        zero, correcting whatever open-loop drift had accumulated. This is
        the absolute reference that closes ChoreographySpeedEXT's WAVE
        feedback loop -- between crossings it relies on POS + the Kp term;
        each crossing gives it an exact fix.

        firmware_count is the Arduino's open-loop step count at the crossing.
        The firmware deliberately did NOT reset its own count (that would
        stop the motor). In WAVE the count stays near 0; in CONSTANT/
        DIRECTION it climbs by STEPS_PER_REV each revolution, so at a
        crossing it's a whole-revolution multiple plus the real drift. The
        true drift is therefore the signed remainder within one revolution,
        NOT the raw count -- e.g. 1601 -> +1, 3203 -> +3, 1599 -> -1.
        """
        half = STEPS_PER_REV // 2
        drift = ((firmware_count + half) % STEPS_PER_REV) - half
        # Anchor to the nearest clean revolution multiple. In WAVE that's 0
        # (count near 0); in continuous rotation it matches the climbing POS
        # frame instead of flickering against it.
        self.actual_pos[motor_id] = firmware_count - drift
        self._updateStateTable(motor_id)
        if drift != 0:
            self.LogEvent(f"Motor {motor_id} zero-cross; drift corrected {drift}")

    def OnFault(self, motor_id, code):
        """Records fault. On TIMEOUT, marks the motor not homed."""
        self.faults[motor_id] = code
        self.LogEvent(f"FAULT motor {motor_id}: {code}")
        if code == 'TIMEOUT':
            self.homed[motor_id] = False
            self.homing[motor_id] = False
        self._updateStateTable(motor_id)

    def OnStatusReport(self, motor_id, actual, ideal, homed, homing, integral):
        """Full state sync from a Mega STATUS response."""
        self.actual_pos[motor_id] = actual
        self.ideal_pos[motor_id] = ideal
        self.homed[motor_id] = homed
        self.homing[motor_id] = homing
        self._updateStateTable(motor_id)

    # -- motion / homing / stop -----------------------------------------

    def _homingDisabled(self):
        """Debug switch: custom parameter 'Disablehoming' (Toggle) on this
        COMP. When on, HomeAll/HomeMotor are suppressed -- no serial send AND
        no homing[] flags set. That matters because a set homing[] flag is
        only cleared by a HOMED/FAULT reply; with no rig connected the reply
        never comes, the flags stick True, and ChoreographySpeedEXT's
        homing-skip guard then blocks EVERY motor (sequencer and manual
        control alike). This switch lets you drive the choreography offline.
        Defensive getattr so it works whether or not the param exists yet."""
        par = getattr(self.ownerComp.par, 'Disablehoming', None)
        return bool(par.eval()) if par is not None else False

    def HomeAll(self):
        """Send HOMEALL to both Megas. Reset all homed flags locally."""
        if self._homingDisabled():
            self.LogEvent("HOMEALL suppressed (Disablehoming debug switch is ON)")
            return
        for i in range(NUM_MOTORS):
            self.homed[i] = False
            self.homing[i] = True
            self._homing_start[i] = absTime.seconds
            #self.speed_mode[i] = False   # homing leaves speed mode (mirrors firmware)
        self.GetSerialEXT(0).SendHomeAll()
        self.GetSerialEXT(7).SendHomeAll()
        self.LogEvent("HOMEALL sent to both Megas")

    def HomeMotor(self, motor_id):
        """Send HOME for one motor."""
        if self._homingDisabled():
            self.LogEvent(f"HOME {motor_id} suppressed (Disablehoming debug switch is ON)")
            return
        self.homed[motor_id] = False
        self.homing[motor_id] = True
        self._homing_start[motor_id] = absTime.seconds
        self.speed_mode[motor_id] = False   # homing leaves speed mode (mirrors firmware)
        self.GetSerialEXT(motor_id).SendHome(motor_id)
        self.LogEvent(f"HOME sent for motor {motor_id}")

    def IsHoming(self, motor_id):
        """True while motor_id is homing -- but SELF-CLEARS if no HOMED/FAULT
        reply arrived within HOMING_TIMEOUT_S. Callers (the choreography's
        homing-skip guard) should use this instead of reading self.homing[]
        directly, so a lost reply can't freeze the rig forever."""
        if self.homing[motor_id] and (absTime.seconds - self._homing_start[motor_id]) > HOMING_TIMEOUT_S:
            self.homing[motor_id] = False
            self.LogEvent(f"Motor {motor_id} homing watchdog fired (no HOMED/FAULT in "
                          f"{HOMING_TIMEOUT_S:.0f}s) -- flag cleared so motion can resume")
        return self.homing[motor_id]

    def ClearHoming(self, motor_id=None):
        """Manually clear stuck homing flag(s) -- immediate un-block after a
        lost home reply. motor_id=None clears all 14."""
        for i in (range(NUM_MOTORS) if motor_id is None else [motor_id]):
            self.homing[i] = False
        self.LogEvent(f"Homing flag(s) cleared: {'all' if motor_id is None else motor_id}")

    def StopAll(self):
        """
        Emergency stop — STOPALL to both Megas.

        Pauses every motion source first (position AND speed choreographers).
        Without this, if any source is still playing, its next-frame Update()
        would push a new SETPOS/SETSPEED -- and the firmware clears its estop
        latch on any such command (meant for a human re-issuing after
        investigating a stop, not an automated loop that has no idea a stop
        just happened). Silencing the source, not just the symptom, is what
        makes this an actual emergency stop rather than a stop-for-one-frame.
        """
        self._pauseChoreography()
        self.GetSerialEXT(0).SendStopAll()
        self.GetSerialEXT(7).SendStopAll()
        # Firmware zeroed each motor's speed on STOPALL, so sync our cached
        # commanded speed to 0. Otherwise SetSpeed's change-detection could
        # suppress a later SETSPEED with the same value the motor had before
        # the stop, and it would never resume (the firmware also latches
        # estopped, which SETSPEED clears -- but only if the send happens).
        # speed_mode is left as-is so UpdateActualPos keeps suppressing the
        # drift re-home for a motor that was spinning.
        for i in range(NUM_MOTORS):
            self.motor_speed[i] = 0
        self.LogEvent("STOPALL sent to both Megas — emergency stop")

    def _pauseChoreography(self):
        # Pause EVERY motion source, not just the position choreographer.
        # Any source left with Playback on would re-issue a command the next
        # frame and defeat the stop -- for the speed choreographer that's a
        # SETSPEED which also clears the firmware's estop latch. Add new
        # choreography COMP names here as more motion sources are created.
        for name in ('base_choreography', 'base_choreography_speed'):
            comp = self.ownerComp.parent().op(name)
            if comp is not None and hasattr(comp.par, 'Playback'):
                comp.par.Playback = 0

    def EnableAll(self):
        """Enable all driver outputs. Protocol has no bulk ENABLEALL, so this sends per-motor."""
        for i in range(NUM_MOTORS):
            self.GetSerialEXT(i).SendEnable(i)
        self.LogEvent("ENABLE sent for all motors")

    def DisableAll(self):
        """Disable all driver outputs. Protocol has no bulk DISABLEALL, so this sends per-motor."""
        for i in range(NUM_MOTORS):
            self.GetSerialEXT(i).SendDisable(i)
        self.LogEvent("DISABLE sent for all motors")

    # -- PID tuning -------------------------------------------------------

    def SetPIDGains(self, kp, ki, kd, motor_id=None):
        """
        Send SETPID to both Megas (or one motor's Mega if motor_id given).
        Stores last-sent values on the Pidkp/Pidki/Pidkd custom parameters so the
        TD panel can display them — those parameters ARE the display mirror,
        not the control source (the Arduino's actual gains only change when
        it receives a SETPID command).
        """
        if motor_id is None:
            self.GetSerialEXT(0).SendSetPID(kp, ki, kd)
            self.GetSerialEXT(7).SendSetPID(kp, ki, kd)
        else:
            self.GetSerialEXT(motor_id).SendSetPID(kp, ki, kd, motor_id)

        self.ownerComp.par.Pidkp = kp
        self.ownerComp.par.Pidki = ki
        self.ownerComp.par.Pidkd = kd
        target = motor_id if motor_id is not None else 'all'
        self.LogEvent(f"SETPID sent: Kp={kp} Ki={ki} Kd={kd} [motor={target}]")

    def GetPIDGains(self):
        """Return (kp, ki, kd) currently stored — for panel display."""
        return (
            self.ownerComp.par.Pidkp.eval(),
            self.ownerComp.par.Pidki.eval(),
            self.ownerComp.par.Pidkd.eval(),
        )

    def ResetPIDState(self, motor_id=None):
        """
        Reset integral/derivative accumulators without changing gains.

        The firmware's RESETPID command has no per-motor form — it always
        clears every motor on the receiving Mega (see cmdResetPid() in
        motor_controller.ino). To reset just one motor without disturbing
        its neighbours, re-send that motor's current gains via the
        per-motor SETPID path instead, which resets PID state on the
        Arduino as an explicit side effect (cmdSetPidMotor()).
        """
        if motor_id is None:
            self.GetSerialEXT(0).SendResetPID()
            self.GetSerialEXT(7).SendResetPID()
            self.LogEvent("RESETPID sent to both Megas")
        else:
            kp, ki, kd = self.GetPIDGains()
            self.GetSerialEXT(motor_id).SendSetPID(kp, ki, kd, motor_id)
            self.LogEvent(f"PID state reset for motor {motor_id} via per-motor SETPID")

    # -- monitoring table --------------------------------------------------

    def _updateStateTable(self, motor_id):
        # Refresh the error CHOP too -- both are "this motor's state changed,
        # update its displays", and all callers already funnel through here.
        # Done first, and independently guarded, so the CHOP still updates
        # even when no motor_state_table exists (and vice versa).
        self._updateErrorCHOP(motor_id)

        table = self.ownerComp.op('motor_state_table')
        if table is None:
            return
        if table.numRows == 0:
            table.appendRow(['motor_id', 'ideal_pos', 'actual_pos', 'error', 'homed', 'homing', 'fault'])
        while table.numRows <= motor_id + 1:
            table.appendRow([''] * 7)

        error = self.ideal_pos[motor_id] - self.actual_pos[motor_id]
        table.replaceRow(motor_id + 1, [
            motor_id,
            self.ideal_pos[motor_id],
            self.actual_pos[motor_id],
            error,
            int(self.homed[motor_id]),
            int(self.homing[motor_id]),
            self.faults[motor_id] or '',
        ])

    def _updateErrorCHOP(self, motor_id):
        """
        Write motor_id's live error (ideal - actual) into channel motor_id
        of a Constant CHOP named 'error_chop', for real-time visualization.

        Uses the const{N}value parameter pattern proven in the RobotRibbon
        project's RobotEXT.PushToCHOP(). getattr(..., None) makes this
        robust to the CHOP not existing yet, or being configured with fewer
        than NUM_MOTORS channels -- in either case the write is silently
        skipped rather than raising, so this never interferes with the
        control path. Channel names are set on the CHOP itself in TD.
        """
        # Each CHOP is looked up and written independently: a missing op, or
        # one configured with fewer than NUM_MOTORS channels, silently skips
        # only its own write instead of crashing or blocking the others.
        # (The previous version only null-checked error_chop, so a missing
        # ideal_chop/actual_chop would raise 'NoneType has no attribute par'.)
        self._writeChopChannel('error_chop', motor_id,
                               self.ideal_pos[motor_id] - self.actual_pos[motor_id])
        self._writeChopChannel('ideal_chop', motor_id, self.ideal_pos[motor_id])
        self._writeChopChannel('actual_chop', motor_id, self.actual_pos[motor_id])
        self._writeChopChannel('speed_chop', motor_id, self.motor_speed[motor_id])
        self._writeChopChannel('speed_pos_chop', motor_id, self.motor_speed_pos[motor_id])


    def _writeChopChannel(self, chop_name, motor_id, value):
        chop = self.ownerComp.op(chop_name)
        if chop is None:
            return
        par = getattr(chop.par, f'const{motor_id}value', None)
        if par is not None:
            par.val = value

    def GetStateTable(self):
        return self.ownerComp.op('motor_state_table')

    # -- startup -------------------------------------------------------

    def Startup(self):
        """
        Full startup sequence (non-blocking — TD can't sleep on the main
        thread, so waits are implemented as re-scheduled checks via run()):
          1. Connect both serial ports
          2. Wait for READY from both Megas (timeout 5s)
          3. DisableAll()
          4. Wait 500ms
          5. EnableAll()
          6. HomeAll()
          7. Wait for HOMED from every motor (timeout 30s total)
          8. Log 'System ready' — proceeding with whatever homed
             successfully; anything still unhomed at the deadline is
             flagged as faulted for manual retry rather than aborting.
          9. Activate default cue via ChoreographyEXT
        """
        
        self.LogEvent("Startup initiated")
        self.GetSerialEXT(0).Connect()
        self.GetSerialEXT(7).Connect()
        self._startup_ready_deadline = absTime.seconds + 5
        self._awaitReady()

    def _awaitReady(self):
        left_ready = self.GetSerialEXT(0).connected
        right_ready = self.GetSerialEXT(7).connected

        if left_ready and right_ready:
            self.LogEvent("Startup: both Megas READY")
            self.DisableAll()
            run("args[0]._startupEnableAndHome()", self, delayMilliSeconds=500)
            return

        if absTime.seconds >= self._startup_ready_deadline:
            missing = [name for name, ok in (('LEFT', left_ready), ('RIGHT', right_ready)) if not ok]
            self.LogEvent(f"Startup FAILED: no READY from {missing} within 5s")
            return

        run("args[0]._awaitReady()", self, delayMilliSeconds=100)

    def _startupEnableAndHome(self):
        self.EnableAll()
        self._startup_home_deadline = absTime.seconds + 30
        self.HomeAll()
        self._awaitHomed()

    def _awaitHomed(self):
        pending = [i for i in range(NUM_MOTORS) if not self.homed[i]]

        if not pending:
            self.LogEvent("Startup: all motors homed — system ready")
            self._startupActivateDefaultCue()
            return

        if absTime.seconds >= self._startup_home_deadline:
            self.LogEvent(
                f"Startup: homing timeout — proceeding with {NUM_MOTORS - len(pending)}/"
                f"{NUM_MOTORS} homed. Faulted, needs manual retry: {pending}"
            )
            for i in pending:
                self.faults[i] = 'HOME_TIMEOUT'
                self.homing[i] = False
                self._updateStateTable(i)
            self._startupActivateDefaultCue()
            return

        run("args[0]._awaitHomed()", self, delayMilliSeconds=250)

    def _startupActivateDefaultCue(self):
        choreo = self.ownerComp.parent().op('base_choreography_speed')
        debug(choreo)
        if choreo is None:
            self.LogEvent("Startup: base_choreography_speed not found — skipping default cue")
            return
        choreo.ext.ChoreographySpeedEXT.GoCue(0)

    # -- bench testing -------------------------------------------------

    def TestSingleMotor(self, motor_id, distance=200, hold_seconds=3):
        """
        Bench-test one motor in isolation. Sends SETPOS directly, which
        works against AccelStepper's own position counter regardless of
        homed[] state — no homing sequence (or the other 13 motors) needs
        to be wired up first. Watch actual_pos in the monitoring table, or
        via 'STATUS' / 'POS' lines on the Arduino's serial console.
        """
        self.LogEvent(f"TestSingleMotor({motor_id}): moving to {distance}, will return in {hold_seconds}s")
        self.SetIdealPos(motor_id, distance)
        run("args[0].SetIdealPos(args[1], 0)", self, motor_id, delayMilliSeconds=hold_seconds * 1000)
