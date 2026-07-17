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

CONFIRMED LIVE (2026-07-17): Connect()/_send() verified against a running
instance — dat.par.address (not netaddress), dat.par.port, dat.par.active
are all correct as written. Mode must be set to Client on the TCP/IP DAT
itself (not a code concern, a DAT parameter to check by hand). The receive
side is also confirmed — see dats/tcp_callback.py for the real onConnect/
onClose/onReceive signatures (onClose, not onDisconnect; onReceive, not
onReceiveText).

send() must be called with terminator= explicitly (send(command,
terminator='\n'), not send(command + '\n')) — per TD's own help(dat.send):
"If no append terminator is specified, a null character will automatically
be appended to the message." That auto-null doesn't corrupt the message it
was appended to -- it becomes the LEADING byte of the NEXT one, since TCP
is a byte stream and the bridge just buffers and splits on '\n'. This
caused a real bug: HOMEALL arriving as '\x00HOMEALL' and being silently
dropped by the Arduino's parser. format ('perbyte'/'perline'/'all') is a
receive-side chunking setting, unrelated to this — don't confuse the two.
"""

SerialProtocolBase = mod(me.parent().parent().path + '/SerialProtocolBase').SerialProtocolBase


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
        dat.par.address = self.ownerComp.par.Serverhost.eval()
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
        # Must use terminator= explicitly -- send()'s default behavior when
        # terminator isn't specified is to auto-append a null character.
        # That null doesn't corrupt the current message; it becomes the
        # LEADING byte of the next one (confirmed live: TCP is a byte
        # stream, and the bridge just buffers everything and splits on
        # '\n' -- the previous message's trailing \x00 sits in that buffer
        # until the next '\n' arrives, then prefixes whatever follows it).
        dat.send(command, terminator='\n')
