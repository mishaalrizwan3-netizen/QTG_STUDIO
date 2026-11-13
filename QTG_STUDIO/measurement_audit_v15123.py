"""Compatibility import for backend measurement audit."""
import sys
from backend import measurement_audit_v15123 as _implementation
sys.modules[__name__] = _implementation
