"""Read-only measurement companion. No simulator or controller interfaces."""
import copy
import csv
import json
import math
import threading
import time
from pathlib import Path
from telemetry_quality_v15124 import track_diagnostics


def clean(value):
    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {str(k): clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(v) for v in value]
    return str(value)


def number(value):
    try:
        result = float(value)
        return result if math.isfinite(result) else None
    except (ValueError, TypeError):
        return None


# Native names remain present in JSONL and CSV. These readable columns never
# substitute an unrelated channel or a guessed historical value.
CHANNELS = {
    "Indicated Airspeed (kt)": "airspeed_indicated",
    "Indicated Altitude (ft)": "indicated_altitude",
    "Geometric Altitude MSL (ft)": "pressure_alt",
    "Radar Altitude (ft)": "radar_alt",
    "Terrain Altitude (ft)": "ground_alt",
    "Ground Speed (kt)": "ground_speed",
    "Ground True Track (deg)": "track_angle",
    "Ground Track from World Velocity (deg; calculated)": "track_world_velocity_deg",
    "GPS minus Velocity Track (deg)": "track_gps_minus_velocity_deg",
    "Wind Direction (deg as reported)": "ambient_wind_dir",
    "Wind Speed (kt)": "ambient_wind_spd",
    "Elevator Trim (rad)": "elevator_trim",
    "Elevator Trim (percent as reported)": "elevator_trim_pct",
    "CG Longitudinal (percent as reported)": "cg_percent",
    "CG Lateral (percent as reported)": "cg_percent_lateral",
    "Body X Acceleration (ft/s2)": "accelX",
    "Body Y Acceleration (ft/s2)": "accelY",
    "Body Z Acceleration (ft/s2)": "accelZ",
    "Fuel Flow Engine 1 (pph as reported)": "fuelFlow",
    "Weight (kg)": "total_weight",
    "Fuel Weight (kg)": "total_fuel_quantity",
    "Pitch (deg as reported)": "pitch_angle",
    "Bank (deg wrapper convention)": "bank_angle",
    "Heading True (deg)": "true_heading",
    "Heading Magnetic (deg)": "magnetic_heading",
    "Angle of Attack (deg)": "aoa",
    "Sideslip (deg)": "sideslip",
}


class MeasurementAudit:
    def __init__(self):
        self._lock = threading.RLock()
        self._records = []
        self._start = None
        self._mode = None

    def begin(self, mode, start_wall, primary_t0, cls_t0):
        with self._lock:
            self._start = float(start_wall)
            self._mode = str(mode)
            self._records = []
            self.record(primary_t0, cls_t0, "accepted_t0")

    def record(self, primary, cls, kind="observation"):
        with self._lock:
            if self._start is None:
                return
            observed = time.time()
            p = clean(copy.deepcopy(dict(primary or {})))
            p.update(track_diagnostics(p))
            hardware = clean(copy.deepcopy(dict(cls or {})))
            updated = number(p.get("last_updated"))
            seq = number(p.get("bridge_telemetry_sequence"))
            previous = self._records[-1] if self._records else None
            repeated = bool(previous and seq is not None and
                            seq == number(previous["p3d"].get("bridge_telemetry_sequence")))
            self._records.append({
                "kind": kind, "mode": self._mode,
                "observed_host_wall_s": observed,
                "observation_elapsed_s": observed - self._start,
                "primary_wrapper_elapsed_s": updated - self._start if updated is not None else None,
                "primary_wrapper_age_s": observed - updated if updated is not None else None,
                "repeated_primary_sequence": repeated,
                "p3d": p, "cls_cached": hardware,
            })

    def save(self, legacy_csv):
        with self._lock:
            records = copy.deepcopy(self._records)
            start = self._start
            mode = self._mode
        if not records:
            raise RuntimeError("No measurement observations captured; do not use this run as a verified recording")
        base = Path(legacy_csv)
        # Prefix avoids the legacy Manual-catalog filename patterns.
        prefix = base.with_name("Measured_" + base.stem)
        jsonl = prefix.with_suffix(".jsonl")
        table = prefix.with_suffix(".csv")
        meta = prefix.with_suffix(".metadata.json")
        with jsonl.open("w", encoding="utf-8") as handle:
            for record in records:
                handle.write(json.dumps(record, allow_nan=False, separators=(",", ":")) + "\n")
        flat = []
        for record in records:
            row = {k: v for k, v in record.items() if k not in ("p3d", "cls_cached")}
            for label, key in CHANNELS.items():
                row[label] = number(record["p3d"].get(key))
            # No actual PRESSURE ALTITUDE channel exists in this wrapper layout.
            row["Pressure Altitude (ft; unavailable)"] = None
            for group in ("p3d", "cls_cached"):
                for key, value in record[group].items():
                    row[group + "." + key] = json.dumps(value, allow_nan=False) if isinstance(value, (list, dict)) else value
            flat.append(row)
        fields = list(dict.fromkeys(key for row in flat for key in row))
        with table.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            writer.writerows(flat)
        known = list(CHANNELS.values())
        summary = {}
        for key in known:
            values = [number(r["p3d"].get(key)) for r in records]
            available = [v for v in values if v is not None]
            summary[key] = {"finite_observations": len(available),
                            "missing_observations": len(values) - len(available),
                            "constant": len(set(available)) == 1 if available else None,
                            "minimum": min(available) if available else None,
                            "maximum": max(available) if available else None}
        metadata = {
            "schema": "V15.12.4-measurement-companion-1", "mode": mode,
            "start_host_wall_s": start, "observation_count": len(records),
            "legacy_csv": base.name,
            "measurement_jsonl": jsonl.name, "measurement_csv": table.name,
            "readable_columns_to_native_keys": CHANNELS,
            "channel_summary": summary,
            "repeated_primary_sequence_observations": sum(r["repeated_primary_sequence"] for r in records),
            "primary_age_over_0_2s_observations": sum((r["primary_wrapper_age_s"] or 0) > 0.2 for r in records),
            "gps_track_disagreement_observations": sum(r["p3d"].get("track_gps_disagreement_over_5deg") is True for r in records),
            "cg_percent_outside_0_to_100_observations": sum((number(r["p3d"].get("cg_percent")) is not None and not 0 <= number(r["p3d"]["cg_percent"]) <= 100) for r in records),
            "limits": [
                "Legacy CSV/PDF retains compatibility substitutions and unit-label defects. Use measured exports for engineering review.",
                "These are host observation and wrapper-update timestamps, not simulator frame timestamps.",
                "Sequence is read separately by the existing wrapper. Sequence and data atomicity are not established.",
                "CLS values are cached observations. Their independent acquisition timestamps are not provided here.",
                "Extended state and engine support have independent update sequences. One row does not prove simultaneous acquisition.",
                "Zero and constant values are preserved. They do not prove a channel is valid or supported by the aircraft.",
                "Blank/null means unavailable. No missing historical value or raw force is reconstructed.",
                "pressure_alt is the legacy wrapper name for PLANE ALTITUDE (geometric MSL), not pressure altitude.",
                "CLS engineering-force units require movement-type metadata: translation N, rotation Nm. Raw position is not millimetres.",
                "Only wrapper-exposed fields are retained. No guarantee of all QTG channels, sampling suitability, or qualification acceptance.",
                "No replay, initialization, controller, bank policy, or hardware calibration changes are made by this companion.",
                "The application V15.12.4 adds Manual aircraft-wind settling and a final aircraft-wind comparison. This companion itself only records.",
                "Negative/out-of-range CG percent is flagged without clamping or converting it to a guessed aircraft value.",
            ],
        }
        meta.write_text(json.dumps(metadata, indent=2, allow_nan=False), encoding="utf-8")
        return table, jsonl, meta
