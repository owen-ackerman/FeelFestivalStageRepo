"""
Serial DAT 'Callbacks DAT' script.

Set this as the Callbacks DAT parameter on the Serial DAT living inside
base_serial_left (or base_serial_right). Must be placed as a direct child
of that same COMP — a sibling of the Serial DAT — since parent() below
resolves to whatever COMP contains this script, and that COMP is expected
to have the SerialEXT extension attached.

See README_TD_SETUP.md for the full operator tree.
"""


def onConnect(dat):
    debug(f"[{parent().name}] Serial port opened")
    return


def onDisconnect(dat):
    debug(f"[{parent().name}] Serial port closed")
    parent().ext.SerialEXT.connected = False
    return


def onReceive(dat, rowIndex, message):
    parent().ext.SerialEXT.onSerialReceive(dat, rowIndex, message)
    return
