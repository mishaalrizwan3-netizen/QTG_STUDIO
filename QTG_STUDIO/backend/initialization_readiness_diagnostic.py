"""Read-only rendering of per-parameter evidence; never authorizes readiness."""
import math

from backend.initialization_requirements import REQUIREMENTS


def finite(value):
    return isinstance(value, (int, float)) and math.isfinite(value)


def blocked_rows(context, reason):
    configured = context.get('initial_conditions') or {}
    criteria = context.get('initialization_attempt', {}).get('reviewed_criteria', {})
    policy = REQUIREMENTS.get(context.get('test_reference'))
    return [dict(parameter=key, configured_target=configured.get(key), actual_p3d=None,
        unit=item.unit, tolerance=criteria.get(key, {}).get('tolerance'), status='UNAVAILABLE', freshness='UNVERIFIED',
        age_s=None, reason=reason, provenance=None)
        for key, item in (policy.requirements.items() if policy else [])]


def comparison_rows(plan, frame, timing, now):
    age = now-frame.measured_at if frame is not None and finite(frame.measured_at) else None
    freshness = 'UNVERIFIED'
    if frame is not None and not frame.connected:
        freshness = 'DISCONNECTED'
    elif age is not None and age < 0:
        freshness = 'FUTURE_TIMESTAMP'
    elif age is not None and age > plan.max_age_s:
        freshness = 'STALE_NATIVE_RECEIPT'
    elif (frame is not None and age is not None and timing.get('sequence') == frame.sequence
          and timing.get('running') is False
          and all(type(timing.get(k)) is int and timing[k] > 0 for k in ('generation','request_id'))):
        freshness = 'FRESH_NATIVE_RECEIPT'
    fresh = freshness == 'FRESH_NATIVE_RECEIPT'
    result = []
    for key, check in plan.checks.items():
        value = frame.values.get(key) if frame is not None else None
        valid = (value is not None and finite(value.value)
                 and value.unit == check.unit and value.channel == check.telemetry)
        status, reason = 'UNAVAILABLE', 'Missing, nonfinite or mismatched-unit/channel telemetry'
        if not fresh:
            reason = 'Disconnected, stale/future, uncorrelated or final pause unverified'
        elif valid:
            delta = abs(value.value-check.value)
            if key in ('heading_deg', 'wind_direction'):
                delta = abs((value.value-check.value+180) % 360-180)
            status = 'PASS' if delta <= check.tolerance else 'FAIL'
            reason = 'Within tolerance' if status == 'PASS' else 'Outside reviewed tolerance'
        result.append(dict(parameter=key, configured_target=check.value,
            actual_p3d=value.value if value is not None and finite(value.value) else None,
            unit=check.unit, tolerance=check.tolerance, status=status, reason=reason,
            freshness=freshness, age_s=age,
            measured_at=frame.measured_at if frame is not None else None,
            assessed_at=now,
            sequence=frame.sequence if frame is not None else None,
            provenance=timing.get('provenance'), generation=timing.get('generation'),
            request_id=timing.get('request_id')))
    return result


def text_table(rows):
    def display(value):
        return f'{value:g}' if finite(value) else 'Not available'
    lines = ['Parameter | Configured target | Actual P3D | Tolerance | Result | Freshness']
    for row in rows:
        age = f" ({row['age_s']:.3f}s)" if row.get('age_s') is not None else ''
        lines.append(f"{row['parameter']} | {display(row['configured_target'])} {row['unit']} | "
                     f"{display(row['actual_p3d'])} | {display(row['tolerance'])} | "
                     f"{row['status']} | {row['freshness']}{age}")
    return '\n'.join(lines)


def report_blocked(window, reason):
    context = window._session_context
    record = context.get('initialization_attempt', {})
    rows = blocked_rows(context, reason)
    record['readiness_diagnostics'] = rows
    record['readiness_diagnostic_text'] = text_table(rows)
    print('[Universal readiness BLOCKED] ' + reason + '\n' + text_table(rows), flush=True)
