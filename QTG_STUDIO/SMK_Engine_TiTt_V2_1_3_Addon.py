"""Compatibility import and command-line entry for Ti/Tt reporting."""
import sys
from reports import SMK_Engine_TiTt_V2_1_3_Addon as _implementation
sys.modules[__name__] = _implementation

if __name__ == "__main__":
    raise SystemExit(_implementation.main())
