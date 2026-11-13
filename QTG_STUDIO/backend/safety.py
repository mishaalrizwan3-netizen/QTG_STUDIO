"""Shared validation of operator-set run limits."""

import math


def validate_duration(value):
    number = float(value)
    if not math.isfinite(number) or number != int(number) or not 1 <= number <= 3600:
        raise ValueError("Duration must be a whole number from 1 to 3600 seconds.")
    return int(number)
