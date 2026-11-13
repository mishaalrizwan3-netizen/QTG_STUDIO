"""Explicitly started native snapshot monitor on a borrowed worker connection.

No connection creation, simulator writes, BRUNNER calls or readiness authority.
The age limit is a diagnostic display criterion, never a readiness tolerance.
"""
import csv
import ctypes
import hashlib
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import threading
import time
from uuid import uuid4

from interfaces.brunner_base.src.verification_snapshot import CHANNELS, VerificationSnapshot


START = 'verification_diagnostic_start'
STOP = 'verification_diagnostic_stop'
SWITCHES = ('QTG_UNIVERSAL_ENGINE_ACCELERATION', 'QTG_UNIVERSAL_ENGINE_DECELERATION')


def status_path(context):
    return Path(context['status_file']).with_suffix('.verification.json')


def idle_reason(window):
    """Observe ownership/activity only. Never stop or release another owner."""
    if (any(os.environ.get(key) == '1' for key in SWITCHES)
            or window._session_context.get('universal_initialization_enabled') is True):
        return 'Both Universal Initialization switches must remain disabled.'
    for name in ('_shutting_down', '_preparing', 'is_recording',
                 'pid_controller_active', '_external_control_active'):
        if getattr(window, name, False):
            return 'Worker is busy: ' + name
    for name in ('preparation_thread', 'worker_thread'):
        thread = getattr(window, name, None)
        if thread is not None and thread.is_alive():
            return 'Worker is busy: ' + name
    adapter = getattr(window, '_engine_initialization_adapter', None)
    reader = getattr(adapter, 'thread', None)
    if reader is not None and reader.is_alive():
        return 'Universal verification reader is still active.'
    return None


def publish(path, payload):
    """Dedicated destination and unique temporary file, never QTG worker status."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + '.' + uuid4().hex + '.tmp')
    try:
        temporary.write_text(json.dumps(payload, indent=2, allow_nan=False), encoding='utf-8')
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


class SnapshotDiagnostic:
    def __init__(self, client, directory, destination, *, max_age_s, duration_s=60.,
                 interval_s=.5, allowed=lambda: None, clock=time.monotonic, probe=None):
        if (not all(finite(v) and v > 0 for v in (max_age_s, duration_s, interval_s))
                or interval_s < .1):
            raise ValueError('Positive diagnostic age/duration and interval >= 0.1 s required')
        if probe not in (None, 'cache', 'superseded'):
            raise ValueError('Unknown read-only probe')
        self.client, self.directory, self.destination = client, Path(directory), Path(destination)
        self.max_age_s, self.duration_s, self.interval_s = max_age_s, duration_s, interval_s
        self.allowed, self.clock = allowed, clock
        self.cancelled = threading.Event()
        self.thread = None
        self.previous = None
        self.reason = None
        self.outputs = {}
        self.counts = {}
        self.channel_summary = {}
        self.transitions = []
        self.publication_errors = []
        self.session_id = uuid4().hex
        self.probe, self._seeded = probe, False
        path = getattr(client, '_dll_path', None)
        self.library = dict(path=str(path) if isinstance(path, (str, Path)) else None,
            file_sha256=None, note='Client-selected library path and on-disk hash, not a hash of loaded memory')
        if self.library['path']:
            try:
                self.library['file_sha256'] = hashlib.sha256(Path(path).read_bytes()).hexdigest()
            except OSError as exc:
                self.library['error'] = str(exc)

    def _native_copy(self):
        getter = self.client._bridge.GetVerificationV1
        getter.argtypes = [ctypes.POINTER(VerificationSnapshot), ctypes.c_uint32]
        getter.restype = ctypes.c_int
        raw = VerificationSnapshot()
        started = self.clock()
        if not getter(ctypes.byref(raw), ctypes.sizeof(raw)):
            raise RuntimeError('Native snapshot copy unavailable or unsupported')
        if raw.size != ctypes.sizeof(raw) or raw.version != 1 or not raw.connected:
            raise RuntimeError('Native verification ABI/connection unavailable')
        raw._host_copy_started = started
        return raw

    def _cached_values(self, raw):
        if not raw.frequency or raw.copied_ticks < raw.receipt_ticks:
            raise RuntimeError('Invalid native receipt clock')
        age = (raw.copied_ticks-raw.receipt_ticks)/raw.frequency
        return dict(version=raw.version, generation=raw.generation, request_id=raw.request_id,
            sequence=raw.sequence, receipt_ticks=raw.receipt_ticks, copied_ticks=raw.copied_ticks,
            frequency=raw.frequency, measured_at=raw._host_copy_started-age, valid_mask=raw.valid_mask,
            running={-1: None, 0: False, 1: True}.get(raw.running),
            values={key:raw.values[i] for i,(key,_,_) in enumerate(CHANNELS)
                    if raw.valid_mask & (1 << i) and math.isfinite(raw.values[i])},
            provenance='Native snapshot export copy; copy alone is not a new simulator response')

    def _read(self):
        if self.probe is None or (self.probe == 'cache' and not self._seeded):
            raw = self.client.verification_snapshot()
            self._seeded = True
            return raw
        lock = self.client.__dict__.setdefault('_verification_lock', threading.Lock())
        with lock:
            if self.probe == 'cache':
                result = self._cached_values(self._native_copy())  # No new request.
                result['read_without_request'] = True
                return result
            generation = self._native_copy().generation
            request = self.client._bridge.RequestVerificationV1
            request.argtypes, request.restype = [], ctypes.c_uint32
            if self.cancelled.is_set() or self.allowed(): raise RuntimeError('Probe cancelled/busy')
            old = request()
            if self.cancelled.is_set() or self.allowed(): raise RuntimeError('Probe cancelled/busy')
            newest = request()  # Read-only pending request supersession.
            if not old or newest <= old: raise RuntimeError('Supersession requests unavailable')
            deadline = self.clock()+1.5
            ignored = 0
            while self.clock() < deadline:
                if self.cancelled.is_set() or self.allowed(): raise RuntimeError('Probe cancelled/busy')
                raw = self._native_copy()
                if raw.generation != generation: raise RuntimeError('Connection changed during supersession probe')
                if raw.request_id == newest:
                    result = self._cached_values(raw)
                    result['supersession_probe'] = dict(older_request_id=old, newest_request_id=newest,
                        accepted_request_id=raw.request_id, ignored_old_export_copies=ignored,
                        obsolete_callback_delivery='Not observable; no claim it arrived after supersession')
                    return result
                if raw.request_id: ignored += 1
                self.cancelled.wait(.01)
            raise RuntimeError('No correlated newest response during supersession probe')

    def sample(self):
        observed = datetime.now(timezone.utc).isoformat()
        payload = dict(session_id=self.session_id, observed_utc=observed,
            state='UNAVAILABLE', reason=None, connected=False, measurements=[],
            native_acquired_at=None, simulator_time_progression='UNVERIFIED',
            max_age_s=self.max_age_s, recording_authorization=False)
        payload.update(worker_pid=os.getpid(), library=self.library)
        raw = None
        try:
            blocked = self.allowed()
            if self.cancelled.is_set() or blocked:
                raise RuntimeError(blocked or 'Diagnostic cancelled')
            if not self.client.is_connected():
                payload['state'] = 'DISCONNECTED'
                raise RuntimeError('Existing worker simulator connection is disconnected')
            payload['connected'] = True
            # The established helper binds RequestVerificationV1/GetVerificationV1
            # on this exact client. It never falls back to its legacy cache.
            raw = self._read()
            blocked = self.allowed()
            if self.cancelled.is_set() or blocked:
                raise RuntimeError(blocked or 'Diagnostic cancelled during read')
            if not self.client.is_connected():
                payload.update(connected=False, state='DISCONNECTED')
                raise RuntimeError('Connection lost during snapshot read')
            for key in ('generation', 'request_id', 'sequence'):
                if type(raw.get(key)) is not int or raw[key] <= 0:
                    raise ValueError('Invalid native identity: ' + key)
            if raw.get('version') != 1:
                raise ValueError('Unsupported snapshot version')
            ticks, copied, frequency = (raw.get(k) for k in ('receipt_ticks', 'copied_ticks', 'frequency'))
            if any(type(v) is not int or v <= 0 for v in (ticks, copied, frequency)) or copied < ticks:
                raise ValueError('Invalid native receipt timing')
            mask = raw.get('valid_mask')
            if type(mask) is not int or mask < 0 or mask >> len(CHANNELS):
                raise ValueError('Invalid native validity mask')
            stamp = raw.get('measured_at')
            if not finite(stamp) or stamp > self.clock():
                raise ValueError('Invalid/future host monotonic receipt estimate')
            age = max((copied-ticks)/frequency, self.clock()-stamp)
            payload.update({key: raw[key] for key in ('generation', 'request_id', 'sequence',
                'receipt_ticks', 'copied_ticks', 'frequency', 'valid_mask')})
            running = raw.get('running') if type(raw.get('running')) is bool else None
            payload.update(native_receipt_age_s=age, measured_at=stamp,
                running=running, provenance=raw.get('provenance'),
                state='STALE_NATIVE_RESPONSE' if age > self.max_age_s else 'FRESH_NATIVE_RESPONSE')
            payload['receipt_within_diagnostic_age'] = age <= self.max_age_s
            payload['probe'] = self.probe
            if raw.get('supersession_probe'): payload['supersession_probe'] = raw['supersession_probe']
            previous = self.previous
            values = raw.get('values')
            if not isinstance(values, dict):
                raise ValueError('Invalid measurement payload')
            if previous:
                same_connection = raw['generation'] == previous['generation']
                if (raw['generation'] < previous['generation']
                        or raw['request_id'] <= previous['request_id']
                        or (same_connection and (raw['sequence'] <= previous['sequence']
                            or ticks <= previous['receipt_ticks']))):
                    payload.update(state='CACHED_OR_SUPERSEDED_RESPONSE',
                        reason='Repeated, old or out-of-order native identity/timing; not a new sample.')
                elif not same_connection:
                    payload['connection_transition'] = 'RECONNECTED'
                else:
                    old, new = previous.get('sim_time'), values.get('sim_time')
                    if finite(old) and finite(new):
                        payload['simulator_time_progression'] = (
                            'ADVANCED' if new > old else 'UNCHANGED' if new == old else 'RESET')
            if payload['state'] != 'CACHED_OR_SUPERSEDED_RESPONSE':
                self.previous = dict(generation=raw['generation'], request_id=raw['request_id'],
                    sequence=raw['sequence'], receipt_ticks=ticks, sim_time=values.get('sim_time'))
            if payload['state'] == 'STALE_NATIVE_RESPONSE':
                payload['reason'] = 'Native receipt exceeds the operator-selected diagnostic age limit.'
            elif raw.get('running') is True and payload['simulator_time_progression'] == 'UNCHANGED':
                payload['reason'] = 'Native response received; simulation-time progression not demonstrated.'
            elif raw.get('running') not in (True, False):
                payload['reason'] = 'Pause/running state unknown. Packets do not prove physics progression.'
            if raw.get('read_without_request'):
                payload.update(state='CACHED_COPY_NOT_REQUESTED',
                    reason='No request issued for this export copy; it cannot count as a new correlated diagnostic sample.')
        except Exception as exc:
            # A malformed frame is never displayed with a fresh classification.
            if payload['state'] not in ('DISCONNECTED',):
                payload['state'] = 'UNAVAILABLE'
            payload['reason'] = str(exc)
        measurements = raw.get('values', {}) if isinstance(raw, dict) else {}
        mask = raw.get('valid_mask', 0) if isinstance(raw, dict) else 0
        for i, (name, simvar, unit) in enumerate(CHANNELS):
            value = measurements.get(name) if isinstance(measurements, dict) else None
            valid = type(mask) is int and bool(mask & (1 << i)) and finite(value)
            if name == 'on_ground': valid = valid and value in (0, 1)
            if name == 'flaps': valid = valid and value >= 0 and value == int(value)
            payload['measurements'].append(dict(channel=name, source=simvar, unit=unit,
                value=value if finite(value) else None, validity='VALID_NATIVE_VALUE' if valid else 'MISSING_OR_INVALID',
                display=value if valid else 'Not available',
                status=payload['state'], raw_value_repr=repr(value) if value is not None and not finite(value) else None))
        return payload

    def start(self):
        if self.thread is not None:
            raise RuntimeError('Diagnostic instances are single use')
        self.directory.mkdir(parents=True, exist_ok=False)
        self.outputs = {name: str(self.directory/filename) for name, filename in
            [('jsonl', 'snapshots.jsonl'), ('csv', 'measurements.csv'), ('summary', 'summary.json')]}
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def stop(self, reason='operator_stop'):
        self.reason = reason
        self.cancelled.set()  # Borrowed client is never stopped/closed/released.

    def _publish(self, payload):
        try:
            publish(self.destination, payload)
        except OSError as exc:
            self.publication_errors.append(str(exc))

    def _run(self):
        started = self.clock()
        payload = dict(session_id=self.session_id, state='WAITING', measurements=[])
        samples = 0
        try:
            with Path(self.outputs['jsonl']).open('x', encoding='utf-8') as evidence, \
                    Path(self.outputs['csv']).open('x', newline='', encoding='utf-8') as table:
                writer = csv.DictWriter(table, fieldnames=['observed_utc', 'channel', 'source', 'unit',
                    'value', 'validity', 'status', 'generation', 'request_id', 'sequence', 'native_receipt_age_s'])
                writer.writeheader()
                while not self.cancelled.is_set() and self.clock()-started < self.duration_s:
                    blocked = self.allowed()
                    if blocked:
                        self.reason = blocked
                        break
                    payload = self.sample()
                    samples += 1
                    self.counts[payload['state']] = self.counts.get(payload['state'], 0)+1
                    transition = {key:payload.get(key) for key in ('state','connected','generation','running')}
                    if not self.transitions or self.transitions[-1]['observation'] != transition:
                        self.transitions.append(dict(observed_utc=payload['observed_utc'], observation=transition))
                    for item in payload['measurements']:
                        aggregate = self.channel_summary.setdefault(item['channel'], dict(
                            unit=item['unit'], source=item['source'], finite_valid_samples=0,
                            fresh_response_samples=0, missing_or_invalid_samples=0,
                            first_valid_reading=None, last_valid_reading=None))
                        if item['validity'] == 'VALID_NATIVE_VALUE':
                            aggregate['finite_valid_samples'] += 1
                            aggregate['fresh_response_samples'] += int(payload['state'] == 'FRESH_NATIVE_RESPONSE')
                            reading = dict(value=item['value'], observed_utc=payload['observed_utc'],
                                state=payload['state'], request_id=payload.get('request_id'),
                                generation=payload.get('generation'), sequence=payload.get('sequence'))
                            if aggregate['first_valid_reading'] is None: aggregate['first_valid_reading'] = reading
                            aggregate['last_valid_reading'] = reading
                        else:
                            aggregate['missing_or_invalid_samples'] += 1
                    payload.update(active=True, outputs=self.outputs, samples=samples)
                    evidence.write(json.dumps(payload, allow_nan=False)+'\n')
                    evidence.flush()
                    for item in payload['measurements']:
                        row = {key: item[key] for key in ('channel', 'source', 'unit', 'value', 'validity', 'status')}
                        row.update({key: payload.get(key) for key in ('observed_utc', 'generation',
                            'request_id', 'sequence', 'native_receipt_age_s')})
                        writer.writerow(row)
                    table.flush()
                    self._publish(payload)
                    self.cancelled.wait(self.interval_s)
        except Exception as exc:
            self.reason = 'Diagnostic/logging failure: ' + str(exc)
        finally:
            payload.update(active=False, session_id=self.session_id, outputs=self.outputs,
                samples=samples, end_reason=self.reason or 'diagnostic_duration_reached',
                counts=self.counts, publication_errors=self.publication_errors,
                channels=self.channel_summary, connection_transitions=self.transitions,
                note='Diagnostic only. Native receipt is not internal sensor acquisition time; no recording authorization.')
            try:
                Path(self.outputs['summary']).write_text(json.dumps(payload, indent=2, allow_nan=False), encoding='utf-8')
            except OSError as exc:
                payload['summary_save_error'] = str(exc)
            errors_before = len(self.publication_errors)
            self._publish(payload)
            if len(self.publication_errors) != errors_before:
                # Keep final publication failures visible in the independent log.
                try:
                    Path(self.outputs['summary']).write_text(json.dumps(payload, indent=2, allow_nan=False), encoding='utf-8')
                except OSError:
                    pass  # First save failure is already retained in the status payload.


def handle_command(window, context, command):
    action = command.get('action')
    if action not in (START, STOP):
        return False
    monitor = getattr(window, '_p3d_verification_diagnostic', None)
    expires = command.get('expires_at')
    if expires is not None and (not finite(expires) or time.time() > expires):
        try:
            publish(status_path(context), dict(active=False, state='REJECTED',
                reason='Diagnostic command expired; no simulator read performed.',
                session_id=command.get('session_id'), measurements=[]))
        except OSError as error:
            print('[P3D Verification] Diagnostic status publication failed: '+str(error))
        return True
    if action == STOP:
        if monitor is not None and command.get('session_id', monitor.session_id) == monitor.session_id:
            monitor.stop()
        return True
    try:
        blocked = idle_reason(window)
        if blocked: raise RuntimeError(blocked)
        if monitor is not None and monitor.thread is not None and monitor.thread.is_alive():
            raise RuntimeError('Verification diagnostic already running')
        destination = status_path(context)
        directory = destination.parent/'p3d_verification'/uuid4().hex
        monitor = SnapshotDiagnostic(window.p3d, directory, destination,
            max_age_s=command['max_age_s'], duration_s=command.get('duration_s', 60.),
            allowed=lambda: idle_reason(window), probe=command.get('probe'))
        if command.get('session_id'):
            monitor.session_id = str(command['session_id'])
        window._p3d_verification_diagnostic = monitor
        monitor.start()
    except Exception as exc:
        try:
            publish(status_path(context), dict(active=False, state='REJECTED', reason=str(exc),
                session_id=command.get('session_id'), measurements=[]))
        except OSError as error:
            # Diagnostic publication must not escape into QTG command failure handling.
            print('[P3D Verification] Diagnostic status publication failed: '+str(error))
    return True


def stop_for_activity(window, action):
    monitor = getattr(window, '_p3d_verification_diagnostic', None)
    if monitor is not None and action in ('initialize', 'record', 'stop', 'shutdown', 'pid_engage', 'pid_tune'):
        monitor.stop('QTG action: '+action)
