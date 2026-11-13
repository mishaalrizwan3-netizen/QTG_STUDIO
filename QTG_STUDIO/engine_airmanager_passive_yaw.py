"""Compatibility import for the backend Air Manager helper."""
import sys
from backend import engine_airmanager_passive_yaw as _implementation
sys.modules[__name__] = _implementation
