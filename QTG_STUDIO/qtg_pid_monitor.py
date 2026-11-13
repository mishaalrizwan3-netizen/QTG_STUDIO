"""Compatibility import for the existing PID monitor UI."""

import sys
from frontend import pid_monitor as _monitor

sys.modules[__name__] = _monitor
