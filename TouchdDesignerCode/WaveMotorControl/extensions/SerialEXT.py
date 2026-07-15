"""
SerialEXT — raw serial transport for one Arduino Mega (LEFT or RIGHT).

One instance of this extension lives on each of base_serial_left and
base_serial_right (see README_TD_SETUP.md for the full operator tree).
Two per-COMP things are expected to exist for this extension to work:

Custom parameters on the owning COMP:
    Serialport   str    e.g. 'COM3'
    Baudrate     int    115200
    Motoroffset  int    0 for LEFT (motors 0-6), 7 for RIGHT (motors 7-13)

Child operator:
    A Serial DAT (Line mode) somewhere under the owning COMP. Found by type
    rather than a hardcoded name, since base_serial_left/base_serial_right
    both use this same extension class.

Wiring: the Serial DAT's line-mode output feeds a 'serial_callback' DAT
Execute (watching for new rows) whose onRowChange() calls
onSerialReceive(dat, rowIndex) once per newly arrived line.

Protocol (see motor_control_implementation_spec.md, Part 4) uses LOCAL motor
ids (0-6) on the wire. This class converts to/from the GLOBAL ids (0-13)
that the rest of the TD network uses, via Motoroffset.
"""


class SerialEXT:
    def __init__(self, ownerComp):
        self.ownerComp = ownerComp
        self.motor_offset = int(ownerComp.par.Motoroffset.eval())
        self.rx_buffer = ""  # reserved; unused while the Serial DAT runs in Line mode
        self.connected = False

    # -- lifecycle -----------------------------------------------------

    def Connect(self):
        """Open serial port."""
        dat = self._serialDat()
        if dat is None:
            debug(f"[{self.ownerComp.name}] Connect() failed — no Serial DAT child found")
            return
        dat.par.port = self.ownerComp.par.Serialport.eval()
        dat.par.baudrate = self.ownerComp.par.Baudrate.eval()
        dat.par.active = True
        self.connected = False  # flips to True once a READY message is received

    def Disconnect(self):
        """Close serial port cleanly."""
        dat = self._serialDat()
        if dat is not None:
            dat.par.active = False
        self.connected = False

    # -- receive ---------------------------------------------------------

    def onSerialReceive(self, dat, rowIndex, cellIndex=None, prev=None):
        """
        Called once per newly-arrived line (by the serial_callback DAT
        Execute watching this COMP's Serial DAT in Line mode). Extracts the
        line text from the row and dispatches it.
        """
        try:
            line = dat[rowIndex, 0].val
        except Exception:
            return
        self.ParseLine(line)

    def ParseLine(self, line):
        """
        Dispatch table:
          'POS'    -> MotorControllerEXT.UpdateActualPos(global_id, steps)
          'HOMED'  -> MotorControllerEXT.OnHomed(global_id)
          'FAULT'  -> MotorControllerEXT.OnFault(global_id, code)
          'READY'  -> log connection confirmed
          'STATUS' -> MotorControllerEXT.OnStatusReport(...)
        global_id = local_id + self.motor_offset
        """
        line = line.strip()
        if not line:
            return

        tokens = line.split()
        msg_type = tokens[0]

        if msg_type == 'READY':
            self.connected = True
            debug(f"[{self.ownerComp.name}] Mega READY")
            return

        if msg_type == 'POS' and len(tokens) >= 3:
            local_id = int(tokens[1])
            steps = int(tokens[2])
            self._motorController().UpdateActualPos(local_id + self.motor_offset, steps)
            return

        if msg_type == 'HOMED' and len(tokens) >= 2:
            local_id = int(tokens[1])
            self._motorController().OnHomed(local_id + self.motor_offset)
            return

        if msg_type == 'FAULT' and len(tokens) >= 3:
            local_id = int(tokens[1])
            code = tokens[2]
            self._motorController().OnFault(local_id + self.motor_offset, code)
            return

        if msg_type == 'STATUS' and len(tokens) >= 7:
            local_id = int(tokens[1])
            actual = int(tokens[2])
            ideal = int(tokens[3])
            homed = bool(int(tokens[4]))
            homing = bool(int(tokens[5]))
            integral = float(tokens[6])
            self._motorController().OnStatusReport(
                local_id + self.motor_offset, actual, ideal, homed, homing, integral
            )
            return

        if msg_type in ('PID_UPDATED', 'PID_RESET'):
            debug(f"[{self.ownerComp.name}] {line}")
            return

        debug(f"[{self.ownerComp.name}] Unrecognized message: {line}")

    # -- send: motion / homing / stop -------------------------------------

    def SendSetPos(self, global_motor_id, ideal_steps):
        """Send: SETPOS <local_id> <steps>"""
        self._send(f"SETPOS {self._toLocal(global_motor_id)} {int(ideal_steps)}")

    def SendHome(self, global_motor_id):
        """Send: HOME <local_id>"""
        self._send(f"HOME {self._toLocal(global_motor_id)}")

    def SendHomeAll(self):
        """Send: HOMEALL"""
        self._send("HOMEALL")

    def SendStop(self, global_motor_id):
        """Send: STOP <local_id>"""
        self._send(f"STOP {self._toLocal(global_motor_id)}")

    def SendStopAll(self):
        """Send: STOPALL"""
        self._send("STOPALL")

    def SendEnable(self, global_motor_id):
        """Send: ENABLE <local_id>"""
        self._send(f"ENABLE {self._toLocal(global_motor_id)}")

    def SendDisable(self, global_motor_id):
        """Send: DISABLE <local_id>"""
        self._send(f"DISABLE {self._toLocal(global_motor_id)}")

    # -- send: PID ---------------------------------------------------------

    def SendSetPID(self, kp, ki, kd, global_motor_id=None):
        """
        Send: SETPID <kp> <ki> <kd>            (global_motor_id is None — all motors on this Mega)
        Send: SETPID <local_id> <kp> <ki> <kd> (global_motor_id given — one motor only)
        """
        if global_motor_id is None:
            self._send(f"SETPID {kp} {ki} {kd}")
        else:
            self._send(f"SETPID {self._toLocal(global_motor_id)} {kp} {ki} {kd}")

    def SendResetPID(self):
        """Send: RESETPID"""
        self._send("RESETPID")

    def RequestStatus(self):
        """Send: STATUS"""
        self._send("STATUS")

    # -- internals -----------------------------------------------------

    def _toLocal(self, global_motor_id):
        return global_motor_id - self.motor_offset

    def _serialDat(self):
        children = self.ownerComp.findChildren(type=serialDAT)
        return children[0] if children else None

    def _motorController(self):
        # Fixed sibling path per the network tree in README_TD_SETUP.md.
        return self.ownerComp.parent().op('base_motor_controller').ext.MotorControllerEXT

    def _send(self, command):
        dat = self._serialDat()
        if dat is None:
            debug(f"[{self.ownerComp.name}] Cannot send '{command}' — no Serial DAT child found")
            return
        dat.send(command, terminator='\n')
