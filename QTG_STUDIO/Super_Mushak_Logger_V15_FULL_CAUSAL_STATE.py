"""Compatibility entry for the standalone legacy climb logger GUI."""
import sys
from frontend import Super_Mushak_Logger_V15_FULL_CAUSAL_STATE as _implementation
sys.modules[__name__] = _implementation

if __name__ == "__main__":
    _implementation.main()
