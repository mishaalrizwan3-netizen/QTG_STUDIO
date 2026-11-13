"""Compatibility import for engine response timing rules."""
import sys
from reports import engine_rules_v2 as _implementation

sys.modules[__name__] = _implementation
