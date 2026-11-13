"""Compatibility import for the climb controller interface."""
import sys
from interfaces import climb_brunner as _implementation
sys.modules[__name__] = _implementation
