"""The demonstration deployment always starts with Universal Initialization off."""
import os

for name in ('QTG_UNIVERSAL_ENGINE_ACCELERATION', 'QTG_UNIVERSAL_ENGINE_DECELERATION',
             'QTG_UNIVERSAL_INITIALIZATION_ONLY', 'QTG_UNIVERSAL_MANUAL_PLANS',
             'QTG_UNIVERSAL_AUTOMATIC_PLANS'):
    os.environ.pop(name, None)
