"""Compatibility import for backend engine preparation."""
import sys
from backend import engine_preparation_v15 as _implementation
sys.modules[__name__] = _implementation
