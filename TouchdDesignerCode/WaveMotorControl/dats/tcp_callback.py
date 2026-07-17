"""
TCP/IP DAT 'Callbacks DAT' script for SerialRelayEXT.

Set this as the Callbacks DAT parameter on the TCP/IP DAT (client mode)
living inside base_serial_left / base_serial_right on the main PC. Must be
a sibling of that TCP/IP DAT, since parent() below resolves to whatever
COMP contains this script.

CAVEAT: unlike serial_callback.py (verified against TD's actual Serial DAT
API earlier in this project), this is my best-effort guess at TCP/IP DAT's
callback signature — I don't have a live TD environment to confirm it for
your version. onConnect/onDisconnect are now confirmed live to take a
second positional argument beyond `dat` (likely a connection/peer
reference — the name doesn't matter, only the count, since TD calls these
positionally). onReceiveText's signature is still an unverified guess; if
it errors the same way, add positional parameters to match whatever count
the TypeError reports, same fix as was just applied here. The bridge
script itself (MiniPCBridge/mega_serial_bridge.py) is independently
testable via `telnet <mini-pc-ip> <port>`, which helps tell apart a
TD-side wiring issue from a bridge/Mega-side one.
"""


def onConnect(dat, connection):
    debug(f"[{parent().name}] Connected to mini PC bridge")
    return


def onDisconnect(dat, connection):
    debug(f"[{parent().name}] Disconnected from mini PC bridge")
    parent().ext.SerialRelayEXT.connected = False
    return


def onReceiveText(dat, rowIndex, message):
    parent().ext.SerialRelayEXT.onNetworkReceive(dat, message)
    return
