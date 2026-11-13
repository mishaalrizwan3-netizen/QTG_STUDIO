"""Source and packaged entry point for the reorganized QTG Studio."""

import sys
from frontend import main as _frontend

# Existing imports of ``main`` and patching of its functions use the same module.
sys.modules[__name__] = _frontend

if __name__ == "__main__":
    _frontend.cli()
