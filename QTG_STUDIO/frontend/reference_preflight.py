"""Presentation of optional engine-reference diagnostics; no runtime commands."""
from backend.manual_reference_preflight import analyze_reference


MODES = {'1.f.(1)': 'acceleration', '1.f.(2)': 'deceleration'}


def diagnostic_text(reference, csv_path):
    if reference not in MODES:
        return 'Reference diagnostics are available for Engine Acceleration and Engine Deceleration only.'
    if not csv_path:
        return 'INSUFFICIENT DATA: Select a Manual reference CSV first.'
    try:
        report = analyze_reference(csv_path, mode=MODES[reference])
    except (OSError, ValueError, TypeError) as exc:
        return f'INSUFFICIENT DATA: Unable to analyze selected reference: {exc}'
    return format_report(report)


def format_report(report):
    pre = report['pre_event']
    insufficient = (report['stable'] is None or not pre
                    or any(item['stable'] is None for item in pre.values())
                    or 'Incomplete or sparse pre-event window' in report['issues'])
    status = 'INSUFFICIENT DATA' if insufficient else 'STABLE' if report['stable'] else 'UNSTABLE'
    lines = [f'Manual Reference Preflight: {status}',
             'Informational only. Initialization targets and readiness are unchanged.',
             f"CSV: {report['csv']}",
             f"Measured throttle onset: {report['event_time_s']} s",
             f"Stability window: {report['window_s']:g} seconds before onset (proposed diagnostic limits).",
             'Event-start values below are the last measured sample BEFORE onset, not an exact interpolated state.',
             'Pre-event median describes the window; it is not a separate measured sample.', '',
             report['provenance_warning'], '']
    def display(value):
        return 'unavailable' if value is None else f'{value:.6g}'
    for key, start in report['recording_start'].items():
        event = report.get('event_start', {}).get(key, {})
        window = pre.get(key, {})
        delta = report['discrepancies'].get(key, {}).get('pre_event_minus_start')
        label = 'unavailable' if window.get('stable') is None else 'stable' if window['stable'] else 'unstable'
        lines.extend([
            f"{key} [{start['unit']}]: recording start={display(start['value'])}; "
            f"event-start sample={display(event.get('value'))}; pre-event median={display(window.get('value'))}",
            f"  Window: {label}; range={display(window.get('range'))}; limit={display(window.get('range_limit'))}; "
            f"median minus recording start={display(delta)}",
            f"  Start source: {start['source']} (t={start['time_s']} s)",
            f"  Event source: {event.get('source', 'unavailable')} (t={event.get('time_s')} s)",
        ])
        side = report['sidecar_start'].get(key, {})
        if side.get('value') is not None:
            lines.append(f"  Sidecar start={display(side['value'])}; source: {side['source']}")
    lines.extend(['', 'Findings:'] + (report['issues'] or ['No issues detected under the proposed diagnostic limits.']))
    return '\n'.join(lines)
