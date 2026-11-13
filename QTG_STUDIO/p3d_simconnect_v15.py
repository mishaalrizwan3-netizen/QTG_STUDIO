"""Compatibility import for the legacy SimConnect adapter."""
import sys
from interfaces import p3d_simconnect_v15 as _implementation
sys.modules[__name__] = _implementation
