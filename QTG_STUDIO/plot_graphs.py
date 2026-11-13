"""Compatibility import for the engine report renderer."""
import sys
from reports import plot_graphs as _implementation

sys.modules[__name__] = _implementation
