"""
TCP/IP DAT 'Callbacks DAT' script for SerialRelayEXT.

Set this as the Callbacks DAT parameter on the TCP/IP DAT (client mode)
living inside base_serial_left / base_serial_right on the main PC. Must be
a sibling of that TCP/IP DAT, since parent() below resolves to whatever
COMP contains this script.

CONFIRMED LIVE (2026-07-17), from TD's own callback template: the real
names/signatures are onConnect(dat, peer), onClose(dat, peer), and
onReceive(dat, rowIndex, message, byteData, peer) -- NOT
onConnect(dat)/onDisconnect(dat)/onReceiveText(dat, rowIndex, message) as
originally guessed. 'peer' is a Peer object describing the connection;
unused here but required in the signature since TD calls these
positionally. 'message' is the ASCII representation of the received data;
'byteData' is the same data as raw bytes, unused here since the wire
protocol is plain ASCII lines.
"""


def onConnect(dat, peer):
    # This is just the TCP handshake to the BRIDGE succeeding -- it says
    # nothing about whether the bridge's serial link to the Mega is up, or
    # whether the Mega itself is alive. Real confirmation follows as
    # separate messages: a "BRIDGE CONNECTED SERIAL_UP/DOWN" line arrives
    # within the same instant (see SerialProtocolBase._handleBridgeMessage),
    # then "READY" once the Mega itself checks in.
    debug(f"[{parent().name}] TCP link to mini PC bridge established -- waiting for bridge/Mega confirmation...")
    return


def onClose(dat, peer):
    debug(f"[{parent().name}] Disconnected from mini PC bridge")
    parent().ext.SerialRelayEXT.connected = False
    parent().ext.SerialRelayEXT.bridge_serial_up = False
    op('const_connect').par.const0value = 0
    return


def onReceive(dat: tcpipDAT, rowIndex: int, message: str, byteData: bytes, 
              peer: Peer):
    parent().ext.SerialRelayEXT.onNetworkReceive(dat, message)
    if (message == "BRIDGE CONNECTED SERIAL_UP"):
        op('const_connect').par.const0value = 1

    """
    Called when TCP data is received.

    Args:
        dat: The DAT that received the data
        rowIndex: The row number the data was placed into
        message: ASCII representation of the data
        byteData: A byte array of the data received
        peer: A Peer object describing the originating data
    """
    return

