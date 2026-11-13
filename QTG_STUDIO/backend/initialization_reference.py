"""Immutable recording-start evidence for the existing initialization manager.

No event alignment, GUI defaults, interpolation, or legacy flap reinterpretation.
CSV values are checked against their full-precision first-sample sidecar values.
"""
import csv
from dataclasses import dataclass
from decimal import Decimal, InvalidOperation
import hashlib
import io
import json
import math
from pathlib import Path
from types import MappingProxyType

from backend.initialization_plan_safety import PlanBlocked


# (CSV column, snapshot alias, native/recorded telemetry key). Historical heading
# column is written from true_heading by this logger despite its display label.
SOURCES = {
    'alt_ft': ('Altitude MSL (ft)', 'altitude_msl', 'pressure_alt'),
    'ias_kt': ('Indicated Airspeed (kts)', 'airspeed', 'airspeed_indicated'),
    'lat': (None, 'lat', 'plane_latitude'), 'lon': (None, 'lon', 'plane_longitude'),
    'heading_deg': ('Magnetic Heading (deg)', 'heading', 'true_heading'),
    'pitch_deg': ('Pitch Angle (deg)', 'pitch_ang', 'pitch_angle'),
    'bank_deg': ('Bank Angle (deg)', 'bank_ang', 'bank_angle'),
    'throttle': ('Throttle 1 Position (%)', 'throttle', 'engine_throttle_percent'),
    'engine_rpm': ('Engine 1 RPM', 'rpm', 'engine_rpm'),
    'prop_rpm': ('Propeller 1 RPM', None, 'prop_rpms'),
    'on_ground': (None, None, 'sim_on_ground'),
    'flaps': (None, None, 'measured_flaps_handle_index'),
    'ground_speed': ('Ground Speed (kt)', None, 'ground_speed'),
    'temperature_c': ('OAT (deg C)', 'oat', 'ambient_temp'),
    'wind_speed': ('Wind Spd (kt)', 'wind_spd', 'ambient_wind_spd'),
    'wind_direction': ('Wind Dir (mag,deg)', 'wind_dir', 'ambient_wind_dir'),
    'barometer': ('Baro Press (hPa)', 'baro_press', 'baro_pressure'),
    'mixture': ('Mixture 1 Position (%)', None, 'engine_mixture_percent'),
    'propeller_lever': ('Propeller 1 Lever (%)', None, 'engine_propeller_percent'),
    'manifold_pressure': ('Engine Manifold Pressure (psi)', None, 'engine_manifold_pressure_psi_1'),
    'weight': ('Gross Wt (kg)', None, 'total_weight'),
    'cg': (None, None, 'cg_percent'),
    'elevator_trim': ('Elevator Trim Angle (rad)', None, 'trim_elevator_angle_rad'),
    'aileron_trim': ('Aileron Trim (%)', None, 'trim_aileron_percent'),
    'rudder_trim': ('Rudder Trim (%)', None, 'trim_rudder_percent'),
}


def finite(value, name):
    try:
        result = float(value)
        if not math.isfinite(result):
            raise ValueError('nonfinite')
        return result
    except (TypeError, ValueError) as exc:
        raise PlanBlocked(name + ': missing/nonfinite recording-start measurement') from exc


@dataclass(frozen=True)
class ReferenceSnapshot:
    path: str
    identity: object
    values: object
    provenance: object
    sample_index: int
    time_s: float
    snapshot_json: str

    def snapshot(self):
        return json.loads(self.snapshot_json)

    def conditions(self):
        from backend.verifier import validate_conditions
        keys = ('alt_ft', 'ias_kt', 'lat', 'lon', 'heading_deg', 'pitch_deg',
                'bank_deg', 'throttle', 'flaps', 'temperature_c', 'on_ground')
        missing = set(keys) - self.values.keys()
        if missing:
            raise PlanBlocked('Recorded preparation conditions unavailable: ' + ', '.join(sorted(missing)))
        return validate_conditions({key: self.values[key] for key in keys})


def capture_reference(path, required):
    if not path:
        raise PlanBlocked('Select a Manual reference CSV for Universal Automatic initialization; GUI targets cannot substitute')
    selected = Path(path).resolve()
    sidecar = selected.with_name(selected.stem + '_init_conditions.json')
    try:
        csv_bytes, sidecar_bytes = selected.read_bytes(), sidecar.read_bytes()
        document = json.loads(sidecar_bytes.decode('utf-8-sig'))
        snapshot = document['manual']
        if not isinstance(snapshot, dict) or not isinstance(snapshot.get('telemetry'), dict):
            raise ValueError('Manual first-sample telemetry metadata required')
        rows = list(csv.DictReader(io.StringIO(csv_bytes.decode('utf-8-sig'))))
    except (OSError, ValueError, KeyError, TypeError) as exc:
        raise PlanBlocked('Invalid Manual CSV/matching sidecar: ' + str(exc)) from exc
    if not rows:
        raise PlanBlocked('Manual CSV contains no recorded samples')
    for column in ('Time (s)', 'Altitude MSL (ft)', 'Indicated Airspeed (kts)', 'Throttle 1 Position (%)'):
        if column not in rows[0]:
            raise PlanBlocked('Missing Manual recording-start CSV column: ' + column)
    # Essential preparation fields cannot be supplied by a GUI or a plan literal.
    required = set(required) | {'alt_ft', 'ias_kt', 'throttle'}
    if required - SOURCES.keys():
        raise PlanBlocked('Unsupported recording-start sources: ' + ', '.join(sorted(required-SOURCES.keys())))
    state = snapshot['telemetry']
    first = None
    last_error = None
    for index, row in enumerate(rows):
        try:
            stamp = finite(row.get('Time (s)'), 'Time (s)')
            if stamp < 0:
                raise PlanBlocked('Negative recording time; event-aligned CSV is not a recording-start reference')
            for key in required:
                column = SOURCES[key][0]
                if column and column in row:
                    finite(row[column], column)
            first = index, stamp, row
            break
        except PlanBlocked as exc:
            last_error = exc
    if first is None:
        raise PlanBlocked('No valid recording-start sample: ' + str(last_error))
    index, stamp, row = first
    # The working replay consumes every recorded row. Do not silently trim or
    # shift its time axis, or pair later data with a t=0 sidecar.
    if index:
        raise PlanBlocked('First valid sample is CSV row ' + str(index+2)
                          + '; leading invalid samples cannot be paired with the recording-start sidecar or silently retimed')
    values, provenance = {}, {}
    for key, (column, alias, channel) in SOURCES.items():
        value = snapshot.get(alias) if alias else None
        native = state.get(channel)
        if key == 'prop_rpm':
            native = native[0] if isinstance(native, (list, tuple)) and native else None
        if value is None:
            value = native
        if value is None and key not in required:
            continue
        try:
            value = finite(value, key)
            if native is not None and finite(native, key) != value:
                raise PlanBlocked(key + ': inconsistent sidecar snapshot/telemetry')
            if column and column in row:
                measured = Decimal(row[column])
                exact = Decimal(str(value))
                if not measured.is_finite() or exact.quantize(measured) != measured:
                    raise PlanBlocked(key + ': Manual CSV/sidecar recording-start mismatch')
            if key == 'on_ground':
                if value not in (0., 1.):
                    raise PlanBlocked('Invalid genuine ground-contact state')
                value = bool(value)
            if key == 'flaps' and (value < 0 or value != int(value)):
                raise PlanBlocked('Invalid genuine flap-handle index')
        except (PlanBlocked, InvalidOperation) as exc:
            # Present contradictory data is never silently accepted even if the
            # channel is not currently a mandatory readiness check.
            if key in required or value is not None:
                raise PlanBlocked(str(exc)) from exc
            continue
        values[key] = value
        provenance[key] = str(sidecar) + ': manual.' + (alias or 'telemetry.' + channel)
        if column and column in row:
            provenance[key] += '; CSV first sample: ' + column
    missing = required-values.keys()
    if missing:
        raise PlanBlocked('Unavailable reference measurements: ' + ', '.join(sorted(missing)))
    for axis in ('pitch', 'roll', 'rudder'):
        if axis + '_pos' in snapshot:
            value = finite(snapshot[axis+'_pos'], axis+'_pos')
            if not -1 <= value <= 1:
                raise PlanBlocked('Invalid recorded physical ' + axis + ' position')
            column = axis.title() + '_Manual_Pos'
            if column in row and finite(row[column], column) != value:
                raise PlanBlocked(axis + ': physical position CSV/sidecar mismatch')
    identities = dict(csv_sha256=hashlib.sha256(csv_bytes).hexdigest(),
                      sidecar_sha256=hashlib.sha256(sidecar_bytes).hexdigest())
    if selected.read_bytes() != csv_bytes or sidecar.read_bytes() != sidecar_bytes:
        raise PlanBlocked('Reference changed during snapshot capture')
    return ReferenceSnapshot(str(selected), MappingProxyType(identities),
        MappingProxyType(values), MappingProxyType(provenance), index, stamp, json.dumps(snapshot))
