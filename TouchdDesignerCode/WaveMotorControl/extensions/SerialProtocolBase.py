"""
SerialProtocolBase — transport-agnostic motor command protocol.

Holds everything about the wire protocol (motor id translation, command
formatting, inbound message parsing/dispatch) that doesn't care HOW bytes
actually reach a Mega. Two concrete subclasses provide the transport:

  SerialEXT       — local Serial DAT. Use for direct bench testing with a
                    laptop plugged straight into one Mega's USB port.
  SerialRelayEXT  — TCP/IP DAT talking to MiniPCBridge/mega_serial_bridge.py
                    on the backstage mini PC. Use for the full show rig,
                    where TD runs on a separate main PC from the Megas.

Both live in the same 'extensions' folder and reference this class via
mod('SerialProtocolBase').SerialProtocolBase — this file must be a sibling
Text DAT of both for that relative path to resolve.

Subclasses must implement _send(command) and call self.ParseLine(message)
whenever a complete line arrives from their transport.
"""


class SerialProtocolBase:
    def __init__(self, ownerComp):
        self.ownerComp = ownerComp
        self.motor_offset = int(ownerComp.par.Motoroffset.eval())
        self.connected = False

    # -- receive ---------------------------------------------------------

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

    def _motorController(self):
        # Fixed sibling path per the network tree in README_TD_SETUP.md.
        return self.ownerComp.parent().op('base_motor_controller').ext.MotorControllerEXT

    def _send(self, command):
        raise NotImplementedError("Subclasses must implement _send()")
