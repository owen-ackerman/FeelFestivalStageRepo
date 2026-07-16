"""
SerialEXT — local Serial DAT transport for one Arduino Mega.

Use this for DIRECT bench testing: a laptop plugged straight into one
Mega's USB port, no network involved. For the full show rig, where TD runs
on a separate main PC from the Megas (see the architecture notes in
README_TD_SETUP.md — main PC <-Ethernet-> mini PC <-USB serial-> Megas),
use SerialRelayEXT instead, which talks to the mini PC bridge
(MiniPCBridge/mega_serial_bridge.py) over TCP.

Both share the same protocol logic (command formatting, motor id
translation, message parsing) via SerialProtocolBase — this file only
implements the transport: opening/closing a local Serial DAT and reading/
writing it.

Custom parameters on the owning COMP:
    Serialport   str    e.g. 'COM3'
    Baudrate     int    115200
    Motoroffset  int    0 for LEFT (motors 0-6), 7 for RIGHT (motors 7-13)

Child operator:
    A Serial DAT (Line mode) somewhere under the owning COMP. Found by type
    rather than a hardcoded name, since base_serial_left/base_serial_right
    both use this same extension class.

Wiring: the Serial DAT's own 'Callbacks DAT' parameter (not a separate DAT
Execute watching row changes) points at a small Text DAT — see
dats/serial_callback.py — whose onReceive(dat, rowIndex, message) calls
onSerialReceive(dat, rowIndex, message) once per received line.
"""

SerialProtocolBase = mod('SerialProtocolBase').SerialProtocolBase


class SerialEXT(SerialProtocolBase):
    def __init__(self, ownerComp):
        super().__init__(ownerComp)
        self.rx_buffer = ""  # reserved; unused while the Serial DAT runs in Line mode

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

    def onSerialReceive(self, dat, rowIndex, message):
        """
        Called once per received line, via the Serial DAT's own Callbacks
        DAT (dats/serial_callback.py) forwarding its onReceive(dat,
        rowIndex, message). 'message' is the raw line text — the Serial
        DAT itself supplies it, no need to re-read the row.
        """
        self.ParseLine(message)

    # -- internals -----------------------------------------------------

    def _serialDat(self):
        children = self.ownerComp.findChildren(type=serialDAT)
        return children[0] if children else None

    def _send(self, command):
        dat = self._serialDat()
        if dat is None:
            debug(f"[{self.ownerComp.name}] Cannot send '{command}' — no Serial DAT child found")
            return
        dat.send(command, terminator='\n')
