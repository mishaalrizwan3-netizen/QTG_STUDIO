"""Compatibility import for the climb BRUNNER transport."""
import sys
from interfaces import climb_brunner_transport as _implementation
sys.modules[__name__] = _implementation
