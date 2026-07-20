"""
Execute DAT callback -- drives the speed show stack every frame, in order:
looper -> sequencer -> choreography (so params a sequencer writes land the
same frame the choreography reads them).

Paste this file's contents INTO an Execute DAT (Execute DATs hold their own
callback code, they have no separate Callbacks DAT). Enable both 'Frame
Start' and 'Active' on its parameters. The COMP names below are resolved
relative to this DAT's location -- if the DAT is at /project1/Execute_dat,
they must be /project1/base_looper etc.

Each stage is called independently via _run(): a MISSING or ERRORING looper
or sequencer can never stop the choreography from updating (the original
version ran all three as one expression, so one missing COMP threw and the
choreography Update never executed -- which looks exactly like "nothing
responds, even manual control"). Errors are logged to the textport instead.

base_looper and base_sequencer are OPTIONAL -- if you're driving the
choreography directly (no show), just don't create them; _run() skips a
COMP that doesn't exist without complaint.
"""


def _run(comp_name, ext_name):
    comp = op(comp_name)
    if comp is None:
        return  # optional stage not present -- skip silently
    try:
        getattr(comp.ext, ext_name).Update()
    except Exception as e:
        debug(f'[Execute] {comp_name}.{ext_name}.Update() error: {e}')


def onFrameStart(frame):
    _run('base_looper', 'ShowLooperEXT')
    _run('base_sequencer', 'ShowSequencerEXT')
    _run('base_choreography_speed', 'ChoreographySpeedEXT')
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
