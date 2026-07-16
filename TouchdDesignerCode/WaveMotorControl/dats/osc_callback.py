"""
OSC In DAT 'Callbacks DAT' script for OSCHandlerEXT.

Set this as the Callbacks DAT parameter on the OSC In DAT living inside
base_osc_handler. Must be a sibling of that OSC In DAT, since parent()
below resolves to whatever COMP contains this script.

CAVEAT: best-effort guess at OSC In DAT's callback signature, not verified
against a live TD instance -- if onReceiveOSC never fires, or fires with a
different argument count, check the OSC In DAT's own help ('?' on the DAT)
for the actual signature in your TD version and adjust below.
"""


def onReceiveOSC(dat, rowIndex, message, bytes, timeStamp, address, args, peer):
    parent().ext.OSCHandlerEXT.OnOSCMessage(address, args)
    return
