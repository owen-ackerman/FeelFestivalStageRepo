"""
Execute DAT callback -- drives ChoreographySpeedEXT.Update() every frame.

Same setup as dats/choreo_execute.py: paste this file's contents INTO an
Execute DAT (Execute DATs hold their own callback code, they have no
separate Callbacks DAT), placed as a direct child of the COMP that has
ChoreographySpeedEXT attached (e.g. base_choreography_speed). Enable both
'Frame Start' and 'Active' on its parameters.
"""


def onFrameStart(frame):
    parent().ext.ChoreographySpeedEXT.Update()
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
