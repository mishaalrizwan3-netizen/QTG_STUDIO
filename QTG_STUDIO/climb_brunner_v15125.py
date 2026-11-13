"""Compatibility import for the v15.12.5 climb controller interface."""
import sys
from interfaces import climb_brunner_v15125 as _implementation
sys.modules[__name__] = _implementation
