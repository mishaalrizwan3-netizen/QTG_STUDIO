"""Compatibility entry for the simulator position helper."""

import sys
from interfaces import position_setter as _setter

sys.modules[__name__] = _setter


if __name__ == "__main__":
    import runpy

    runpy.run_module("interfaces.position_setter", run_name="__main__")
