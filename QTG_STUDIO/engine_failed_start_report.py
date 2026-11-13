"""Compatibility import for backend failed-start reporting."""
import sys
from backend import engine_failed_start_report as _implementation
sys.modules[__name__] = _implementation
