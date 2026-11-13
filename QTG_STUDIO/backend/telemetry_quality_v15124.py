"""Read-only telemetry calculations and pre-capture settling checks."""
import math
import time


def finite(value):
    try:
        result = float(value)
        return result if math.isfinite(result) else None
    except (TypeError, ValueError):
        return None


def track_diagnostics(data):
    x, z = finite(data.get('velocity_world_x')), finite(data.get('velocity_world_z'))
    gps = finite(data.get('track_angle'))
    result = {'track_world_velocity_deg': None, 'track_gps_minus_velocity_deg': None,
              'track_gps_disagreement_over_5deg': None}
    if x is None or z is None or math.hypot(x, z) < 1.0:
        return result
    track = math.degrees(math.atan2(x, z)) % 360.0
    result['track_world_velocity_deg'] = track
    if gps is not None:
        delta = (gps - track + 180.0) % 360.0 - 180.0
        result['track_gps_minus_velocity_deg'] = delta
        result['track_gps_disagreement_over_5deg'] = abs(delta) > 5.0
    return result


def wind_ready(data, tolerance_kt=0.25):
    values = [finite(data.get('aircraft_wind_' + axis + '_kt')) for axis in 'xyz']
    ambient = finite(data.get('ambient_wind_spd'))
    if ambient is None or any(v is None for v in values):
        return False, 'aircraft/ambient wind measurement missing'
    effective = math.sqrt(sum(v*v for v in values))
    difference = abs(effective - abs(ambient))
    return difference <= tolerance_kt, ('effective wind %.3f kt; ambient %.3f kt; '
                                        'magnitude difference %.3f kt' % (effective, ambient, difference))


def wait_wind_ready(client, timeout=20.0, hold_seconds=0.5,
                    clock=time.monotonic, pause=time.sleep, cancelled=lambda: False):
    """No simulator writes. Require changing primary sequences during settling.

    Magnitude agreement is an initialization-transient check, not proof of full
    wind direction/atmosphere equivalence. Auto compares the vector separately.
    """
    deadline = clock() + timeout
    stable_since = None
    previous = client.get_telemetry_sequence()
    latest = None
    detail = 'no fresh wind frame'
    while clock() < deadline:
        if cancelled():
            return None, 'wind settling cancelled'
        if not client.request_telemetry_once():
            return None, 'wind settling telemetry request rejected'
        sample = client.wait_for_fresh_telemetry(previous, 0.3)
        seq = finite((sample or {}).get('bridge_telemetry_sequence'))
        if sample is None or seq is None or seq <= previous:
            stable_since = None
            pause(0.01)
            continue
        previous = seq
        latest = sample
        ready, detail = wind_ready(sample)
        now = clock()
        if ready:
            if stable_since is None:
                stable_since = now
            if now - stable_since >= hold_seconds:
                return latest, detail
        else:
            stable_since = None
        pause(0.04)
    return None, 'wind did not settle within %.1f s: %s' % (timeout, detail)


def wind_matches_reference(data, state, tolerance_kt=0.25):
    details = []
    ok = True
    for axis in 'xyz':
        key = 'aircraft_wind_' + axis + '_kt'
        live, reference = finite(data.get(key)), finite(state.get(key))
        if live is None or reference is None:
            ok = False
            details.append(key + '=missing')
        else:
            difference = live - reference
            ok = ok and abs(difference) <= tolerance_kt
            details.append('%s delta=%+.3f kt' % (axis, difference))
    return ok, '; '.join(details)
