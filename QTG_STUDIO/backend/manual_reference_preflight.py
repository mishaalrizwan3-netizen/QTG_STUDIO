"""Read-only reference diagnostics. Not imported by initialization or execution.

Ranges are diagnostic proposals, not qualification or runtime tolerances.
Values labelled CSV-recorded are not proof of native telemetry provenance:
legacy exporters can insert defaults (notably weather/weight/ground speed).
"""
import argparse
import csv
import json
import math
from pathlib import Path
from statistics import median


# CSV column, unit, sidecar telemetry key, proposed maximum window range.
CHANNELS = {
    'engine_rpm': ('Engine 1 RPM', 'rpm', 'engine_rpm', 30.),
    'prop_rpm': ('Propeller 1 RPM', 'rpm', 'prop_rpms', 30.),
    'throttle': ('Throttle 1 Position (%)', '%', 'engine_throttle_percent', 1.),
    'map': ('Engine Manifold Pressure (psi)', 'psi', 'engine_manifold_pressure_psi_1', .2),
    'ias': ('Indicated Airspeed (kts)', 'kt', 'airspeed_indicated', 1.),
    'ground_speed': ('Ground Speed (kt)', 'kt', 'ground_speed', .5),
    'altitude_msl': ('Altitude MSL (ft)', 'ft MSL', 'pressure_alt', 5.),
    'pitch': ('Pitch Angle (deg)', 'deg', 'pitch_angle', .5),
    'bank': ('Bank Angle (deg)', 'deg', 'bank_angle', .5),
    'heading': ('Magnetic Heading (deg)', 'deg (CSV labelled magnetic)', None, 1.),
    'temperature': ('OAT (deg C)', 'deg C', 'ambient_temp', 1.),
    'wind_speed': ('Wind Spd (kt)', 'kt', 'ambient_wind_spd', 1.),
    'wind_direction': ('Wind Dir (mag,deg)', 'deg (CSV labelled magnetic)', None, 5.),
    'barometer': ('Baro Press (hPa)', 'hPa', 'baro_pressure', .2),
    'weight': ('Gross Wt (kg)', 'kg', 'total_weight', 1.),
}


def number(value):
    try:
        result = float(value)
        return result if math.isfinite(result) else None
    except (TypeError, ValueError):
        return None


def analyze_reference(csv_path, *, mode='deceleration', window_s=2.,
                      sidecar_path=None, range_limits=None, max_gap_s=.25):
    """Return JSON-safe diagnostics; never write, reject a run, or fill gaps.

    Onset follows measure() in SMK_Engine_TiTt_V2_1_3_Addon: initial/final
    throttle medians, directional 2%-of-sweep (minimum 1%) threshold, then
    interpolated crossing. Explicit percent column only; no unit guessing,
    MAP dependency, companion substitution, or dropping invalid rows.
    """
    if mode not in ('acceleration', 'deceleration'):
        raise ValueError('Unknown engine mode')
    if not math.isfinite(window_s) or window_s <= 0 or not math.isfinite(max_gap_s) or max_gap_s <= 0:
        raise ValueError('Window and maximum gap must be positive and finite')
    limits = {key: value[3] for key, value in CHANNELS.items()}
    for key, value in (range_limits or {}).items():
        if key not in limits or number(value) is None or value < 0:
            raise ValueError('Invalid diagnostic range limit: ' + key)
        limits[key] = value
    path = Path(csv_path)
    report = dict(csv=str(path), mode=mode, diagnostic_only=True, issues=[],
                  recording_start={}, pre_event={}, sidecar_start={}, discrepancies={},
                  event_time_s=None, stable=None, window_s=window_s,
                  provenance_warning='CSV values are recorded values; legacy exporter defaults cannot be distinguished from native measurements here.')
    issues = report['issues']
    with path.open(newline='', encoding='utf-8-sig') as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        issues.append('Empty CSV')
        return report
    times = [number(row.get('Time (s)')) for row in rows]
    throttle = [number(row.get(CHANNELS['throttle'][0])) for row in rows]
    sidepath = Path(sidecar_path) if sidecar_path else path.with_name(path.stem + '_init_conditions.json')
    snapshot = {}
    try:
        document = json.loads(sidepath.read_text(encoding='utf-8'))
        snapshot = document.get('manual', document)
        if not isinstance(snapshot, dict) or not isinstance(snapshot.get('telemetry', {}), dict):
            raise ValueError('Invalid snapshot object')
    except (OSError, ValueError, AttributeError) as exc:
        issues.append('Unavailable/invalid sidecar: ' + str(exc))
        snapshot = {}
    telemetry = snapshot.get('telemetry', {})
    for key, (column, unit, telekey, _) in CHANNELS.items():
        value = number(rows[0].get(column))
        report['recording_start'][key] = dict(value=value, unit=unit,
            source=f'{path}: CSV row 2, {column}', time_s=times[0])
        raw = telemetry.get(telekey) if telekey else None
        if key == 'prop_rpm':
            raw = raw[0] if isinstance(raw, list) and raw else None
        side = number(raw)
        report['sidecar_start'][key] = dict(value=side, unit=unit,
            source=f'{sidepath}: telemetry.{telekey}' if telekey else None)
        if value is not None and side is not None and abs(value-side) > limits[key]:
            issues.append('CSV/sidecar disagreement: ' + key)
        if key == 'engine_rpm':
            target = number(snapshot.get('rpm'))
            report['snapshot_rpm_target'] = dict(value=target, unit='rpm', source=f'{sidepath}: rpm')
            if target is not None and any(v is not None and abs(target-v) > limits[key] for v in (value, side)):
                issues.append('Replay snapshot RPM target disagrees with recorded RPM')
    if any(t is None for t in times) or any(b <= a for a, b in zip(times, times[1:])):
        issues.append('Invalid or non-increasing timestamps; event analysis unavailable')
        return report
    if len(rows) < 8 or any(h is None or not 0 <= h <= 100 for h in throttle):
        issues.append('Insufficient/invalid percent throttle samples; event analysis unavailable')
        return report
    first = max(2, min(5, len(rows)//50))
    last = max(3, min(len(rows)//5, len(rows)//8))
    h0, hf = median(throttle[:first]), median(throttle[-last:])
    sign = 1 if mode == 'acceleration' else -1
    if sign*(hf-h0) < 20:
        issues.append('Small or wrong-direction throttle sweep; event analysis unavailable')
        return report
    threshold = max(1., .02*abs(hf-h0))
    onset = next((i for i in range(1, len(rows)) if sign*(throttle[i]-h0) >= threshold), None)
    if onset is None:
        issues.append('No measured throttle onset')
        return report
    before = sign*(throttle[onset-1]-h0)
    after = sign*(throttle[onset]-h0)
    event = times[onset-1] + (times[onset]-times[onset-1])*(threshold-before)/(after-before)
    report['event_time_s'] = event
    # Last measured sample before onset, not an interpolated aircraft state.
    report['event_start'] = {
        key: dict(value=number(rows[onset-1].get(column)), unit=unit,
                  time_s=times[onset-1], source=f'{path}: CSV row {onset+1}, {column}')
        for key, (column, unit, _, _) in CHANNELS.items()}
    report['event_source'] = dict(path=str(path), column=CHANNELS['throttle'][0],
                                 threshold_pct=threshold, method='interpolated directional throttle threshold')
    indices = [i for i in range(onset) if event-window_s <= times[i] < event]
    complete = (len(indices) >= 5 and times[0] <= event-window_s
                and all(b-a <= max_gap_s for a, b in zip(times[indices[0]:onset+1], times[indices[0]+1:onset+1])))
    if not complete:
        issues.append('Incomplete or sparse pre-event window')
    stable = complete
    for key, (column, unit, _, _) in CHANNELS.items():
        values = [number(rows[i].get(column)) for i in indices]
        valid = [v for v in values if v is not None]
        # Circular spans relative to the first measured direction avoid 359/0 jumps.
        span_values = ([(v-valid[0]+180) % 360-180 for v in valid]
                       if valid and key in ('heading', 'wind_direction') else valid)
        span = max(span_values)-min(span_values) if valid else None
        available = bool(valid) and len(valid) == len(indices)
        channel_stable = available and span <= limits[key]
        stable = stable and channel_stable
        centre = ((valid[0]+median(span_values)) % 360 if key in ('heading', 'wind_direction') and valid
                  else median(valid) if valid else None)
        report['pre_event'][key] = dict(value=centre, unit=unit, statistic='window median',
            source=f'{path}: {column}', samples=len(valid), expected_samples=len(indices),
            range=span, range_limit=limits[key], stable=channel_stable if available else None)
        if not available:
            issues.append('Unavailable/incomplete pre-event measurement: ' + key)
        elif not channel_stable:
            issues.append('Unstable pre-event measurement: ' + key)
        start = report['recording_start'][key]['value']
        if start is not None and centre is not None:
            delta = centre-start
            if key in ('heading', 'wind_direction'):
                delta = (delta+180) % 360-180
            report['discrepancies'][key] = dict(pre_event_minus_start=delta, unit=unit)
            if abs(delta) > limits[key]:
                issues.append('Transient recording-start condition: ' + key)
    report['stable'] = stable
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv')
    parser.add_argument('--mode', choices=('acceleration', 'deceleration'), default='deceleration')
    parser.add_argument('--window-s', type=float, default=2.)
    args = parser.parse_args()
    print(json.dumps(analyze_reference(args.csv, mode=args.mode, window_s=args.window_s), indent=2, allow_nan=False))
