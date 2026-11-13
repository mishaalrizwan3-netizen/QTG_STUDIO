"""Input validation shared by Manual, Recorded Replay and PID runs."""

import csv
import math
from pathlib import Path


def validate_reference(path):
    path = Path(path).resolve()
    aliases = [
        ("Pitch_Manual_Pos", "Pitch_Man_Pos", "Pitch_Pos", "Joystick Pitch Position (mm)"),
        ("Roll_Manual_Pos", "Roll_Man_Pos", "Roll_Pos", "Joystick Roll Position (mm)"),
        ("Rudder_Manual_Pos", "Rudder_Man_Pos", "Rudder_Pos", "Rudder Pedal Position (mm)"),
        ("Throttle 1 Position (%)", "Throttle_Manual", "Throttle"),
    ]
    with path.open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        names = set(reader.fieldnames or [])
        if "Time (s)" not in names or any(not names.intersection(group) for group in aliases):
            raise ValueError("Choose a Manual recording from the BRUNNER base, containing time, pitch, roll, rudder and throttle measurements.")
        previous = None
        count = 0
        for row in reader:
            stamp = float(row["Time (s)"])
            values = [float(next(row[key] for key in group if key in row and row[key].strip())) for group in aliases]
            if not all(math.isfinite(value) for value in [stamp] + values) or (previous is not None and stamp < previous):
                raise ValueError("Manual reference contains invalid measurements or backwards timestamps.")
            if not 0 <= values[-1] <= 100:
                raise ValueError("Manual reference throttle must be between 0 and 100 percent.")
            previous = stamp
            count += 1
    if count < 2:
        raise ValueError("Manual reference needs at least two recorded samples.")
    return str(path)


def validate_conditions(values):
    if values is None:
        return None
    result = dict(values)
    result.setdefault("heading_deg", 0.0)
    result.setdefault("pitch_deg", 0.0)
    result.setdefault("bank_deg", 0.0)
    limits = {"alt_ft": (-1500, 60000), "ias_kt": (0, 400), "lat": (-90, 90),
              "lon": (-180, 180), "heading_deg": (0, 360), "pitch_deg": (-90, 90),
              "bank_deg": (-90, 90), "flaps": (0, 10), "throttle": (0, 100),
              "temperature_c": (-100, 60)}
    for key, (low, high) in limits.items():
        value = float(result[key])
        if not math.isfinite(value) or not low <= value <= high:
            raise ValueError("Invalid initial condition: " + key)
        result[key] = value
    if result["flaps"] != int(result["flaps"]):
        raise ValueError("Flaps must be a whole-number handle index.")
    result["on_ground"] = bool(result.get("on_ground", False))
    return result
