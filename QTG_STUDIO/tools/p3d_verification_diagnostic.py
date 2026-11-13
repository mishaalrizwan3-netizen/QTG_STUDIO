"""Console display/IPC only. The existing worker owns all native snapshot reads."""
import argparse
import json
from pathlib import Path
import sys
import time
from uuid import uuid4

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from backend.p3d_verification_diagnostic import START, STOP, status_path


def load_context(path, *, require_live=True, wall_clock=time.time):
    context = json.loads(Path(path).read_text(encoding='utf-8-sig'))
    directory = Path(context['command_directory'])
    status = Path(context['status_file'])
    if not directory.is_dir() or not status.is_file():
        raise ValueError('An existing worker command directory and status file are required; no worker will be launched.')
    if require_live:
        worker = json.loads(status.read_text(encoding='utf-8-sig'))
        age = wall_clock()-worker.get('updated', 0)
        if not 0 <= age <= 5:
            raise ValueError('Worker heartbeat is not current (5-second transport check); no command queued.')
        if worker.get('command_directory') != context['command_directory']:
            raise ValueError('Worker status does not match the selected context')
    return context


def send(context, action, **values):
    if context.get('session_kind') == 'p3d_verification_only':
        from backend.p3d_diagnostic_only import enqueue
        return enqueue(context, action, **values)
    if action not in (START, STOP):
        raise ValueError('Only verification diagnostic start/stop commands are allowed')
    from qtg_common_engine import send_command
    send_command(context, action, expires_at=time.time()+5., **values)


def render(payload):
    running = payload.get('running')
    state = 'Running' if running is True else 'Paused' if running is False else 'Unknown'
    lines = [f"Last observation UTC: {payload.get('observed_utc', 'Not available')}",
        f"Observed P3D transport: {'Connected' if payload.get('connected') else 'Disconnected/unknown'} | {payload.get('state')}",
        f"Running state: {state} | Simulation time: {payload.get('simulator_time_progression', 'UNVERIFIED')}",
        f"Request {payload.get('request_id', '--')} | Generation {payload.get('generation', '--')} | Native sequence {payload.get('sequence', '--')}",
        f"Native receipt age at observation: {payload.get('native_receipt_age_s', '--')} s | Validity mask: {payload.get('valid_mask', '--')}",
        'Native response freshness is not proof of physics progression or sensor accuracy. Internal acquisition time unavailable.']
    if payload.get('library'):
        lines.append(f"Worker PID {payload.get('worker_pid')} | DLL: {payload['library'].get('path')} | On-disk SHA256: {payload['library'].get('file_sha256')}")
    if payload.get('reason'): lines.append('Note: '+payload['reason'])
    if payload.get('supersession_probe'):
        lines.append('Read-only supersession probe: '+json.dumps(payload['supersession_probe']))
    for item in payload.get('measurements', []):
        display = str(item.get('display', 'Not available'))
        lines.append(f"{item['channel']:22} {display:20} {item['unit']:12} {item['validity']} | {item['source']}")
    if payload.get('active') is False:
        lines.append('The readings above are historical; monitoring is stopped.')
        lines.append('Monitoring stopped: '+str(payload.get('end_reason', payload.get('reason', ''))))
        for name, path in payload.get('outputs', {}).items():
            lines.append(f"{name}: {path} [{'FILE EXISTS' if Path(path).is_file() else 'NOT CONFIRMED SAVED'}]")
        if payload.get('summary_save_error'): lines.append(payload['summary_save_error'])
    return '\n'.join(lines)


def follow(context, session_id, duration_s, *, clock=time.monotonic, sleep=time.sleep, output=print):
    destination = status_path(context)
    deadline, acknowledgement_deadline = clock()+duration_s+8., clock()+8.
    previous = None
    acknowledged = False
    try:
        while clock() < deadline:
            try:
                payload = json.loads(destination.read_text(encoding='utf-8'))
            except (OSError, ValueError):
                payload = {}
            if payload.get('session_id') == session_id:
                acknowledged = True
                encoded = json.dumps(payload, sort_keys=True)
                if encoded != previous:
                    output(render(payload))
                    previous = encoded
                if payload.get('active') is False:
                    return 1 if (payload.get('state') == 'REJECTED' or payload.get('summary_save_error')
                        or str(payload.get('end_reason', '')).startswith('Diagnostic/logging failure')) else 0
            if not acknowledged and clock() >= acknowledgement_deadline:
                raise RuntimeError('No matching worker acknowledgement; check the selected context and diagnostic log.')
            sleep(.2)
        raise RuntimeError('Diagnostic status stopped updating or did not finish; diagnostic-only stop requested.')
    finally:
        # Idempotent and session-scoped: never sends ordinary QTG stop/shutdown.
        send(context, STOP, session_id=session_id)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--context', help='Exact context JSON printed by diagnostic-only startup or an existing idle worker')
    sub = parser.add_subparsers(dest='action', required=True)
    start = sub.add_parser('start')
    start.add_argument('--max-age-s', type=float, required=True, help='Diagnostic age criterion only; not a readiness limit')
    start.add_argument('--duration-s', type=float, default=60.)
    start.add_argument('--probe', choices=('cache', 'superseded'), help='Read-only cache or request supersession check')
    sub.add_parser('status')
    sub.add_parser('stop')
    sub.add_parser('close', help='Close an owned diagnostic-only session, never a normal QTG worker')
    connection = sub.add_parser('diagnostic-only', help='Explicit foreground P3D-only connection; no aircraft/test initialization')
    connection.add_argument('--session-root', default=str(ROOT/'build'/'p3d_verification_sessions'))
    connection.add_argument('--connection-timeout-s', type=float, default=15.)
    connection.add_argument('--session-timeout-s', type=float, default=300.)
    args = parser.parse_args(argv)
    try:
        if args.action == 'diagnostic-only':
            if args.context: raise ValueError('diagnostic-only creates and prints its own context; omit --context')
            from backend.p3d_diagnostic_only import run
            return run(args.session_root, connection_timeout_s=args.connection_timeout_s,
                       session_timeout_s=args.session_timeout_s)
        if not args.context: raise ValueError('--context is required for start/status/stop/close')
        context = load_context(args.context, require_live=args.action != 'status')
        if args.action == 'close':
            from backend.p3d_diagnostic_only import CLOSE, KIND
            if context.get('session_kind') != KIND:
                raise ValueError('close is allowed only for an owned diagnostic-only session')
            send(context, CLOSE)
            deadline = time.monotonic()+8.
            while time.monotonic() < deadline:
                try:
                    owner = json.loads(Path(context['status_file']).read_text(encoding='utf-8'))
                except (OSError, ValueError): owner = {}
                if owner.get('phase') in ('CLOSED', 'FAILED'):
                    print('Diagnostic-only session '+owner['phase']+': '+str(owner.get('reason')))
                    return 0 if owner['phase'] == 'CLOSED' else 1
                time.sleep(.1)
            raise RuntimeError('Diagnostic-only shutdown was not confirmed; inspect its foreground terminal. No QTG shutdown command sent.')
        if args.action == 'start':
            from backend.p3d_verification_diagnostic import finite
            if not all(finite(v) and v > 0 for v in (args.max_age_s, args.duration_s)):
                raise ValueError('Positive finite diagnostic age/duration required')
            session = uuid4().hex
            send(context, START, session_id=session, max_age_s=args.max_age_s,
                 duration_s=args.duration_s, probe=args.probe)
            return follow(context, session, args.duration_s)
        payload = json.loads(status_path(context).read_text(encoding='utf-8'))
        if args.action == 'status':
            print('Stored diagnostic status only; this command requests no native sample.')
            print(render(payload))
        else:
            send(context, STOP, session_id=payload['session_id'])
            print('Diagnostic-only stop requested. QTG worker and simulator connection are unchanged.')
        return 0
    except KeyboardInterrupt:
        print('Console cancelled; diagnostic-only stop requested.')
        return 130
    except (OSError, ValueError, KeyError, RuntimeError) as exc:
        print('Verification diagnostic: '+str(exc), file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
