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
"""

NUM_MOTORS = 14


class MotorControllerEXT:
    def __init__(self, ownerComp):
        self.ownerComp = ownerComp

        self.ideal_pos  = [0] * NUM_MOTORS    # steps — set by ChoreographyEXT/manual jog, sent to Mega
        self.actual_pos = [0] * NUM_MOTORS    # steps — received from Mega (monitoring only)
        self.homed      = [False] * NUM_MOTORS
        self.homing     = [False] * NUM_MOTORS
        self.faults     = [None] * NUM_MOTORS

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
        if self.ideal_pos[motor_id] == steps:
            return
        self.ideal_pos[motor_id] = steps
        self.GetSerialEXT(motor_id).SendSetPos(motor_id, steps)
        self._updateStateTable(motor_id)

    def UpdateActualPos(self, motor_id, steps):
        """
        Called by SerialEXT when a POS message is received. Updates
        monitoring state and checks the drift safety fallback. Does NOT run
        PID — that's the Arduino's job.
        """
        self.actual_pos[motor_id] = steps
        self._updateStateTable(motor_id)

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
        self.faults[motor_id] = None
        self.LogEvent(f"Motor {motor_id} HOMED")
        self._updateStateTable(motor_id)
        # Clear any integral/derivative accumulated before the home crossing
        # so the first move afterward doesn't jerk (spec note 13).
        self.ResetPIDState(motor_id)

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

    def HomeAll(self):
        """Send HOMEALL to both Megas. Reset all homed flags locally."""
        for i in range(NUM_MOTORS):
            self.homed[i] = False
            self.homing[i] = True
        self.GetSerialEXT(0).SendHomeAll()
        self.GetSerialEXT(7).SendHomeAll()
        self.LogEvent("HOMEALL sent to both Megas")

    def HomeMotor(self, motor_id):
        """Send HOME for one motor."""
        self.homed[motor_id] = False
        self.homing[motor_id] = True
        self.GetSerialEXT(motor_id).SendHome(motor_id)
        self.LogEvent(f"HOME sent for motor {motor_id}")

    def StopAll(self):
        """
        Emergency stop — STOPALL to both Megas.

        Also pauses ChoreographyEXT first. Without this, if a wave cue is
        still playing, the very next frame's Update() would push a new
        SETPOS -- and the firmware's estop latch is deliberately cleared by
        any new SETPOS (that's meant for a human re-issuing a command after
        investigating a stop, not an automated loop that has no idea a stop
        just happened). Silencing the source, not just the symptom, is what
        makes this an actual emergency stop rather than a stop-for-one-frame.
        """
        self._pauseChoreography()
        self.GetSerialEXT(0).SendStopAll()
        self.GetSerialEXT(7).SendStopAll()
        self.LogEvent("STOPALL sent to both Megas — emergency stop")

    def _pauseChoreography(self):
        choreo = self.ownerComp.parent().op('base_choreography')
        if choreo is None:
            return
        choreo.par.Playback = 0

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
        choreo = self.ownerComp.parent().op('base_choreography')
        if choreo is None:
            self.LogEvent("Startup: base_choreography not found — skipping default cue")
            return
        choreo.ext.ChoreographyEXT.GoCue(0)

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
