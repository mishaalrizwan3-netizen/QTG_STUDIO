"""Presentation helpers for the operator-controlled initialization panel."""


def display_state(state):
    attempt = state.get('initialization_attempt') or {}
    raw = attempt.get('status')
    if raw in ('READY',):
        status = 'READY'
    elif raw in ('BLOCKED', 'FAILED', 'CANCELLED'):
        status = 'FAILED'
    elif raw in ('INITIALIZING', 'VALIDATING', 'PREPARING', 'VERIFYING'):
        status = 'INITIALIZING'
    else:
        status = 'IDLE'
    locked = state.get('initialization_only') is True and state.get('can_record') is False
    lock = 'LOCKED' if locked else 'UNVERIFIED'
    error = attempt.get('failure_reason') or (state.get('details') if status == 'FAILED' else '') or ''
    required = set((attempt.get('reviewed_criteria') or {}).keys())
    rows = [row for row in attempt.get('readiness_diagnostics', [])
            if row.get('parameter') in required]
    return status, lock, error, rows
