"""Compatibility import for the engine report generator."""
import sys
from reports import engine_report_v2 as _implementation

sys.modules[__name__] = _implementation
