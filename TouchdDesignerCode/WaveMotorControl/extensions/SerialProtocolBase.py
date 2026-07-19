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

Both reference this class via
mod(me.parent().parent().path + '/SerialProtocolBase').SerialProtocolBase
-- i.e. this file must be a child of the GRANDPARENT of whichever COMP
owns SerialEXT/SerialRelayEXT (their COMP's parent), not a direct sibling
of the SerialEXT/SerialRelayEXT Text DAT itself. Match your actual network
layout on both sides.

Subclasses must implement _send(command) and call self.ParseLine(message)
whenever a complete line arrives from their transport.
"""


class SerialProtocolBase:
    def __init__(self, ownerComp):
        self.ownerComp = ownerComp
        self.motor_offset = int(ownerComp.par.Motoroffset.eval())
        self.connected = False          # True only once the Mega itself has said READY
        self.bridge_serial_up = False   # True once the bridge confirms its serial link to the Mega is up
                                         # (SerialEXT/local transport has no bridge, so this just stays
                                         # False unused there -- only SerialRelayEXT's BRIDGE messages set it)

    # -- receive ---------------------------------------------------------

    def ParseLine(self, line):
        """
        Dispatch table:
          'BRIDGE' -> bridge self-status (SerialRelayEXT/mega_serial_bridge.py only,
                      see mega_serial_bridge.py's module docstring for the message list)
          'POS'    -> MotorControllerEXT.UpdateActualPos(global_id, steps)
          'HOMED'  -> MotorControllerEXT.OnHomed(global_id)
          'ZERO'   -> MotorControllerEXT.OnZeroCross(global_id, count)
          'FAULT'  -> MotorControllerEXT.OnFault(global_id, code)
          'READY'  -> Mega itself confirms alive -- the real end-to-end handshake
          'STATUS' -> MotorControllerEXT.OnStatusReport(...)
        global_id = local_id + self.motor_offset
        """
        line = line.strip()
        if not line:
            return

        tokens = line.split()
        msg_type = tokens[0]

        if msg_type == 'BRIDGE':
            self._handleBridgeMessage(tokens[1:])
            return

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

        if msg_type == 'ZERO' and len(tokens) >= 2:
            # speed_motor_controller: motor physically crossed its home
            # sensor mid-run. Optional 3rd token is the firmware's open-loop
            # count at the crossing (drift diagnostic).
            local_id = int(tokens[1])
            count = int(tokens[2]) if len(tokens) >= 3 else 0
            self._motorController().OnZeroCross(local_id + self.motor_offset, count)
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

        if msg_type in ('PID_UPDATED', 'PID_RESET', 'MAXSPEED_UPDATED', 'ACCEL_UPDATED', 'RESYNC'):
            debug(f"[{self.ownerComp.name}] {line}")
            return

        if msg_type == 'ERR':
            # The firmware echoes commands it doesn't recognize. Expected
            # when TD talks to a firmware that lacks a feature (e.g. SETPID
            # on a non-PID build) -- log as a firmware notice, not a TD
            # parse failure.
            debug(f"[{self.ownerComp.name}] Firmware {line}")
            return

        debug(f"[{self.ownerComp.name}] Unrecognized message: {line}")

    # -- send: motion / homing / stop -------------------------------------

    def SendSetPos(self, global_motor_id, ideal_steps):
        """Send: SETPOS <local_id> <steps>"""
        self._send(f"SETPOS {self._toLocal(global_motor_id)} {int(ideal_steps)}")

    def SendSetSpeed(self, global_motor_id, steps_per_sec):
        """
        Send: SETSPEED <local_id> <signed_steps_per_sec>
        Puts the motor into continuous-rotation mode on the firmware side
        (see cmdSetSpeed in motor_controller.ino). Sign is direction; 0
        stops it while staying in speed mode.
        """
        self._send(f"SETSPEED {self._toLocal(global_motor_id)} {int(steps_per_sec)}")

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

    # -- send: motion limits (motor_controller_without_pid.ino only) -------

    def SendSetMaxSpeed(self, value, global_motor_id=None):
        """
        Send: SETMAXSPEED <steps_per_sec>            (all motors on this Mega)
        Send: SETMAXSPEED <local_id> <steps_per_sec> (one motor only)
        Only the PID-free firmware handles this; the PID build replies
        "ERR unknown command" (harmless -- just logged).
        """
        if global_motor_id is None:
            self._send(f"SETMAXSPEED {int(value)}")
        else:
            self._send(f"SETMAXSPEED {self._toLocal(global_motor_id)} {int(value)}")

    def SendSetAccel(self, value, global_motor_id=None):
        """
        Send: SETACCEL <steps_per_sec2>            (all motors on this Mega)
        Send: SETACCEL <local_id> <steps_per_sec2> (one motor only)
        PID-free firmware only; PID build replies "ERR unknown command".
        """
        if global_motor_id is None:
            self._send(f"SETACCEL {int(value)}")
        else:
            self._send(f"SETACCEL {self._toLocal(global_motor_id)} {int(value)}")

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

    def _handleBridgeMessage(self, args):
        """
        Handles mega_serial_bridge.py's self-status messages -- distinct
        from anything the Mega itself sends. See that file's module
        docstring for the exact message list. These give visibility a bare
        TCP connect doesn't: 'connected to the bridge' is not the same as
        'the bridge's serial link to the Mega is up', which is not the same
        as 'the Mega itself has confirmed alive' (that last one is READY,
        handled separately in ParseLine).
        """
        if not args:
            return
        kind = args[0]

        if kind == 'CONNECTED':
            serial_state = args[1] if len(args) > 1 else 'UNKNOWN'
            self.bridge_serial_up = (serial_state == 'SERIAL_UP')
            debug(
                f"[{self.ownerComp.name}] Bridge TCP link confirmed — "
                f"Mega serial link is {'UP' if self.bridge_serial_up else 'DOWN'}"
            )
        elif kind == 'SERIAL_UP':
            self.bridge_serial_up = True
            debug(f"[{self.ownerComp.name}] Bridge reports Mega serial link back UP")
        elif kind == 'SERIAL_DOWN':
            self.bridge_serial_up = False
            self.connected = False
            debug(f"[{self.ownerComp.name}] Bridge reports Mega serial link DOWN — no longer connected")
        elif kind == 'DISCONNECTING':
            reason = ' '.join(args[1:]) if len(args) > 1 else 'unknown reason'
            self.bridge_serial_up = False
            self.connected = False
            debug(f"[{self.ownerComp.name}] Bridge is disconnecting us: {reason}")
        else:
            debug(f"[{self.ownerComp.name}] Unrecognized BRIDGE message: {kind}")

    def _toLocal(self, global_motor_id):
        return global_motor_id - self.motor_offset

    def _motorController(self):
        # Fixed sibling path per the network tree in README_TD_SETUP.md.
        return self.ownerComp.parent().op('base_motor_controller').ext.MotorControllerEXT

    def _send(self, command):
        raise NotImplementedError("Subclasses must implement _send()")
