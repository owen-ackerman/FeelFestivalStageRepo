"""
SerialRelayEXT — networked transport for one Arduino Mega, via the mini PC
bridge (MiniPCBridge/mega_serial_bridge.py) over a plain TCP socket.

Use this on the main PC for the full show rig: main PC (this) <-Ethernet->
mini PC (bridge script) <-USB serial-> Megas. For direct local bench
testing with a laptop plugged straight into a Mega, use SerialEXT instead.
Both share the same protocol logic — see SerialProtocolBase.py.

The wire protocol to the bridge is intentionally the exact same newline-
terminated ASCII lines the Mega itself speaks (see
motor_control_implementation_spec.md, Part 4) — the bridge does no
translation, it just forwards bytes between this TCP socket and the Mega's
COM port. That means it's testable by hand with any raw TCP client (e.g.
`telnet <mini-pc-ip> <port>` or `nc <mini-pc-ip> <port>`), independent of
TD entirely — the same way the Arduino's own serial link was tested via
the Serial Monitor. Useful for telling apart a TD-side problem from a
bridge-side one.

Custom parameters on the owning COMP:
    Serverhost   str    the mini PC's IP address, e.g. '192.168.1.50'
    Serverport   int    e.g. 9000 for LEFT / 9001 for RIGHT — must match
                         the corresponding entry in mega_serial_bridge.py's
                         SIDES config
    Motoroffset  int    0 for LEFT (motors 0-6), 7 for RIGHT (motors 7-13)

Child operator:
    A TCP/IP DAT (client mode) somewhere under the owning COMP, found by
    type. Its 'Callbacks DAT' parameter should point at dats/tcp_callback.py.

CAVEAT: I don't have a live TD environment to confirm the exact TCP/IP DAT
Python API (parameter names, send method, callback signature) the way
Serial DAT's was verified earlier in this project — the parameter names
and send() call below are my best-effort match to TD's documented API, not
tested against a running instance. If Connect()/_send() errors out, check
the TCP/IP DAT's own parameter names in your TD version and adjust
_tcpDat()/_send() accordingly; see also the caveat in dats/tcp_callback.py
for the receive side.
"""

SerialProtocolBase = mod('SerialProtocolBase').SerialProtocolBase


class SerialRelayEXT(SerialProtocolBase):
    def __init__(self, ownerComp):
        super().__init__(ownerComp)

    # -- lifecycle -----------------------------------------------------

    def Connect(self):
        """Open the TCP connection to the mini PC bridge."""
        dat = self._tcpDat()
        if dat is None:
            debug(f"[{self.ownerComp.name}] Connect() failed — no TCP/IP DAT child found")
            return
        dat.par.netaddress = self.ownerComp.par.Serverhost.eval()
        dat.par.port = self.ownerComp.par.Serverport.eval()
        dat.par.active = True
        self.connected = False  # flips to True once a READY message is received

    def Disconnect(self):
        """Close the TCP connection cleanly."""
        dat = self._tcpDat()
        if dat is not None:
            dat.par.active = False
        self.connected = False

    # -- receive ---------------------------------------------------------

    def onNetworkReceive(self, dat, message):
        """
        Called once per received line, via the TCP/IP DAT's Callbacks DAT
        (dats/tcp_callback.py). 'message' is the raw line text as relayed
        verbatim by mega_serial_bridge.py from the Mega's serial output.
        """
        self.ParseLine(message)

    # -- internals -----------------------------------------------------

    def _tcpDat(self):
        children = self.ownerComp.findChildren(type=tcpipDAT)
        return children[0] if children else None

    def _send(self, command):
        dat = self._tcpDat()
        if dat is None:
            debug(f"[{self.ownerComp.name}] Cannot send '{command}' — no TCP/IP DAT child found")
            return
        dat.sendText(command + '\n')
