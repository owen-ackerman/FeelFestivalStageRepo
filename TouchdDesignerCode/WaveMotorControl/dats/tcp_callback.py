"""
TCP/IP DAT 'Callbacks DAT' script for SerialRelayEXT.

Set this as the Callbacks DAT parameter on the TCP/IP DAT (client mode)
living inside base_serial_left / base_serial_right on the main PC. Must be
a sibling of that TCP/IP DAT, since parent() below resolves to whatever
COMP contains this script.

CAVEAT: unlike serial_callback.py (verified against TD's actual Serial DAT
API earlier in this project), this is my best-effort guess at TCP/IP DAT's
callback signature — I don't have a live TD environment to confirm it for
your version. If onReceiveText never fires, or fires with a different
argument count, open the TCP/IP DAT's own help (the '?' on the DAT) for
the actual callback names/signatures it exposes and adjust below. The
bridge script itself (MiniPCBridge/mega_serial_bridge.py) is independently
testable via `telnet <mini-pc-ip> <port>`, which helps tell apart a
TD-side wiring issue from a bridge/Mega-side one.
"""


def onConnect(dat):
    debug(f"[{parent().name}] Connected to mini PC bridge")
    return


def onDisconnect(dat):
    debug(f"[{parent().name}] Disconnected from mini PC bridge")
    parent().ext.SerialRelayEXT.connected = False
    return


def onReceiveText(dat, rowIndex, message):
    parent().ext.SerialRelayEXT.onNetworkReceive(dat, message)
    return
