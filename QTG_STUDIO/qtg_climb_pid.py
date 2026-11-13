"""Compatibility import for the Normal Climb procedure module."""

import sys
from test_suites.normal_climb import procedure as _procedure

# Preserve monkeypatching and state shared with existing imports.
sys.modules[__name__] = _procedure
