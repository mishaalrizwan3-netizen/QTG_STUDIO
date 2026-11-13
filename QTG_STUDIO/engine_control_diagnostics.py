"""Compatibility import for backend engine diagnostics."""
import sys
from backend import engine_control_diagnostics as _implementation
sys.modules[__name__] = _implementation
