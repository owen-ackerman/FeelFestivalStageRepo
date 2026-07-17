"""
Execute DAT callback -- drives ChoreographyEXT.Update() every frame.

Unlike Serial DAT/TCP/IP DAT/OSC In DAT, an Execute DAT has no separate
'Callbacks DAT' parameter pointing elsewhere -- it holds its own callback
code directly in its own body. Paste this file's full contents INTO the
Execute DAT itself (open it and edit its text, same as a Text DAT).

Setup:
  1. Create an Execute DAT as a direct child of base_choreography (so
     parent() below resolves to that COMP).
  2. Open it and paste this entire file's contents into it.
  3. Enable both 'Frame Start' and 'Active' on its parameters -- Frame
     Start alone isn't enough; Active has to be on too.

onFrameStart is the only thing that actually calls into ChoreographyEXT --
nothing about an Execute DAT is aware of any extension automatically, this
one explicit call is the entire bridge between the two.
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
