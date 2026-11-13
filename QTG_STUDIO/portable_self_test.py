"""Compatibility import for the portable release self-test."""
import sys
from tools import portable_self_test as _implementation
sys.modules[__name__] = _implementation

if __name__ == "__main__":
    _implementation.run_self_test()
