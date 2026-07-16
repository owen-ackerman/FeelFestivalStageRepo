"""
Execute DAT callback -- drives ChoreographyEXT.Update() every frame.

Set this as the Callbacks DAT (or the script directly, if using a plain
Text DAT-based Execute DAT) on an Execute DAT with its 'Frame Start'
parameter enabled, placed as a direct child of base_choreography (so
parent() below resolves to that COMP).
"""


def onFrameStart(frame):
    parent().ext.ChoreographyEXT.Update()
    return


def onStart():
    return


def onCreate():
    return


def onExit():
    return


def onFrameEnd(frame):
    return


def onPlayStateChange(state):
    return


def onDeviceChange():
    return


def onProjectPreSave():
    return


def onProjectPostSave():
    return
