"""Compatibility import for backend throttle timeline handling."""
import sys
from backend import throttle_timeline as _implementation
sys.modules[__name__] = _implementation
