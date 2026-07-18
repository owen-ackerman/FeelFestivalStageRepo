"""
OSCHandlerEXT — receives OSC from an external control surface (FOH device,
lighting desk, QLab, a TouchOSC layout, etc.) and translates it into
ChoreographyEXT / MotorControllerEXT calls. Optional: the system works
standalone via direct Python calls (textport, UI buttons) without this at
all — this is only needed if you want a physical/external control surface
triggering cues or tweaking PID live during a show.

Wiring: an OSC In DAT (default port 7000) with its Callbacks DAT pointed
at dats/osc_callback.py, which forwards into OnOSCMessage() below. Must be
placed as base_osc_handler, a sibling of base_motor_controller and
base_choreography under base_motor_system, for the fixed relative lookups
below to resolve.

CAVEAT: same situation as SerialRelayEXT/TCP earlier in this project — I
don't have a live TD environment to confirm the OSC In DAT Callbacks DAT's
exact signature. onReceiveOSC(dat, rowIndex, message, bytes, timeStamp,
address, args, peer) is my best-effort match to TD's documented API, not
verified against a running instance. If it doesn't fire, or errors on
argument count, check the OSC In DAT's own help page for your TD version
and adjust dats/osc_callback.py accordingly.

Address map (see motor_control_implementation_spec.md Part 2):
    /motor/cue          i   cue_index
    /motor/next
    /motor/prev
    /motor/wave/amp     f   amplitude_steps
    /motor/wave/freq    f   frequency_hz
    /motor/wave/phase   f   phase_offset_rad
    /motor/homeall
    /motor/stopall
    /motor/enable
    /motor/disable
    /motor/pid/kp           f       value
    /motor/pid/ki           f       value
    /motor/pid/kd           f       value
    /motor/pid/set          f f f   kp ki kd
    /motor/pid/motor        i f f f motor_id kp ki kd
    /motor/pid/reset
    /motor/setpos           i i     motor_id steps
"""


class OSCHandlerEXT:
    def __init__(self, ownerComp):
        self.ownerComp = ownerComp

    def OnOSCMessage(self, address, args):
        """
        Dispatch one OSC message. address is a string like '/motor/cue',
        args is a list of already-typed values (int/float), as TD's OSC In
        DAT parses them from the incoming packet.
        """
        try:
            self._dispatch(address, args)
        except (ValueError, IndexError, TypeError) as e:
            debug(f"[OSCHandlerEXT] Malformed OSC message {address} {args}: {e}")

    def _dispatch(self, address, args):
        choreo = self._choreography()
        controller = self._motorController()

        if address == '/motor/cue' and len(args) >= 1:
            choreo.GoCue(int(args[0]))
        elif address == '/motor/next':
            choreo.NextCue()
        elif address == '/motor/prev':
            choreo.PrevCue()
        elif address == '/motor/wave/amp' and len(args) >= 1:
            self._choreographyComp().par.Waveamplitude = float(args[0])
        elif address == '/motor/wave/freq' and len(args) >= 1:
            self._choreographyComp().par.Wavefrequency = float(args[0])
        elif address == '/motor/wave/phase' and len(args) >= 1:
            self._choreographyComp().par.Wavephaseoffset = float(args[0])
        elif address == '/motor/homeall':
            controller.HomeAll()
        elif address == '/motor/stopall':
            controller.StopAll()
        elif address == '/motor/enable':
            controller.EnableAll()
        elif address == '/motor/disable':
            controller.DisableAll()
        elif address == '/motor/pid/kp' and len(args) >= 1:
            self._setPidComponent(kp=float(args[0]))
        elif address == '/motor/pid/ki' and len(args) >= 1:
            self._setPidComponent(ki=float(args[0]))
        elif address == '/motor/pid/kd' and len(args) >= 1:
            self._setPidComponent(kd=float(args[0]))
        elif address == '/motor/pid/set' and len(args) >= 3:
            controller.SetPIDGains(float(args[0]), float(args[1]), float(args[2]))
        elif address == '/motor/pid/motor' and len(args) >= 4:
            controller.SetPIDGains(float(args[1]), float(args[2]), float(args[3]), motor_id=int(args[0]))
        elif address == '/motor/pid/reset':
            controller.ResetPIDState()
        elif address == '/motor/setpos' and len(args) >= 2:
            controller.SetIdealPos(int(args[0]), int(args[1]))
        elif address == '/motor/setspeed' and len(args) >= 2:
            controller.SetSpeed(int(args[0]), int(args[1]))
        else:
            debug(f"[OSCHandlerEXT] Unhandled OSC message: {address} {args}")

    def _setPidComponent(self, kp=None, ki=None, kd=None):
        """/motor/pid/kp|ki|kd update just one gain component, keeping the
        other two at their last-known values."""
        controller = self._motorController()
        cur_kp, cur_ki, cur_kd = controller.GetPIDGains()
        controller.SetPIDGains(
            kp if kp is not None else cur_kp,
            ki if ki is not None else cur_ki,
            kd if kd is not None else cur_kd,
        )

    # -- internals -----------------------------------------------------

    def _motorController(self):
        return self.ownerComp.parent().op('base_motor_controller').ext.MotorControllerEXT

    def _choreographyComp(self):
        return self.ownerComp.parent().op('base_choreography')

    def _choreography(self):
        return self._choreographyComp().ext.ChoreographyEXT
