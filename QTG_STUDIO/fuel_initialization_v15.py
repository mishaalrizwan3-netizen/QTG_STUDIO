"""Compatibility import for backend fuel initialization."""
import sys
from backend import fuel_initialization_v15 as _implementation
sys.modules[__name__] = _implementation
