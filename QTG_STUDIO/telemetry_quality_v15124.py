"""Compatibility import for backend telemetry quality checks."""
import sys
from backend import telemetry_quality_v15124 as _implementation
sys.modules[__name__] = _implementation
