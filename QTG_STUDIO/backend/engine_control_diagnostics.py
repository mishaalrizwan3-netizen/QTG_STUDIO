"""Read-only control-handover diagnostics for V2.1.3 BRUNNER-enabled engine testing.

This module NEVER writes flight controls. It separates a known P3D control
mapping mismatch from unrelated start failures during *diagnostic* runs.
"""

CONTROL_FIELDS = frozenset({
    'ElevInput', 'AilInput', 'RudInput', 'ElevSurface',
    'AilLSurface', 'AilRSurface', 'RudSurface',
})


def classify_handover(detail):
    """Return (mapping_mismatch, other_failures, control_failures).

    Fail closed for unknown/missing fields. Classification uses the same
    descriptive fields emitted by the verified V15.12.5 check.
    """
    control_failures, other_failures = [], []
    if not isinstance(detail, str) or not detail.strip():
        return False, ['missing handover detail'], []
    segments = [s.strip() for s in detail.split('|') if s.strip()]
    if not segments:
        return False, ['empty control handover'], []
    for segment in segments:
        if '(FAIL)' not in segment and 'MISSING(' not in segment:
            continue
        label = segment.split('=', 1)[0].strip()
        # Missing control feedback is never a permissible diagnostic mismatch.
        (control_failures if label in CONTROL_FIELDS and '(FAIL)' in segment and 'MISSING(' not in segment else other_failures).append(segment)
    return bool(control_failures), other_failures, control_failures


def stable_rpm_sample(data, min_rpm=2660, max_rpm=2740, max_split=60):
    """Verify a fresh P3D frame has stable-power prerequisites (not a timer)."""
    if not isinstance(data, dict):
        return False, 'No fresh engine telemetry'
    try:
        if not data.get('bridge_telemetry_sequence') and not data.get('last_updated'):
            return False, 'Telemetry sequence/time unavailable'
        rpm = float((data.get('eng_rpms') or [None])[0])
        prop = float((data.get('prop_rpms') or [None])[0])
        throttle = float((data.get('throttles') or [None])[0])
        import math
        if not all(math.isfinite(x) for x in (rpm, prop, throttle)):
            return False, 'Invalid RPM/throttle telemetry'
        if data.get('sim_on_ground', 0) < .5:
            return False, 'Aircraft is not on the ground'
        if data.get('brake_parking_position', 0) < .9:
            return False, 'Parking brake not applied'
        if throttle < 95:
            return False, f'Throttle only {throttle:.1f}%'
        if not (min_rpm <= rpm <= max_rpm and min_rpm <= prop <= max_rpm):
            return False, f'Engine {rpm:.0f} / prop {prop:.0f} RPM; waiting for ~2700'
        if abs(rpm - prop) > max_split:
            return False, f'Engine/prop RPM difference {abs(rpm-prop):.0f} exceeds {max_split}'
        return True, f'Stable power candidate: engine={rpm:.1f}, prop={prop:.1f}, throttle={throttle:.1f}%'
    except (TypeError, ValueError, IndexError):
        return False, 'Engine/prop RPM channel missing'
