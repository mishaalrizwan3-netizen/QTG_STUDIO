"""Compatibility entry for the standalone legacy engine GUI."""
import sys
from frontend import SMK_Engine_From_Climb as _implementation
sys.modules[__name__] = _implementation

if __name__ == "__main__":
    _implementation.main()
