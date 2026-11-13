"""Operator-started cache monitor. No connect/request/set/control methods used."""
import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import re
import threading
import time
from uuid import uuid4

from backend.initialization_requirements import FIELDS
from backend.telemetry_verification import ReadOnlyTelemetry


class SessionSummary:
    """Aggregate recorded evidence without replacing or backfilling any sample."""
    def __init__(self):
        self.snapshots = self.p3d_available_snapshots = 0
        self.channels = {}
        self.transitions = []
        self.previous = None

    def observe(self, records, source_state, context=None):
        self.snapshots += 1
        stamp = records[0]['observed_utc'] if records else None
        context = context or {}
        state = {'source_state': source_state, 'worker_stage': context.get('worker_stage'),
                 'shutdown_requested': context.get('shutdown_requested', False)}
        if state != self.previous:
            self.transitions.append(dict(observed_utc=stamp, **state))
            self.previous = state
        p3d_available = False
        for row in records:
            key = row['channel']
            entry = self.channels.setdefault(key, dict(unit=row['unit'], available_sample_count=0,
                missing_sample_count=0, verified_sample_count=0, first_valid_reading=None,
                last_valid_reading=None, missing_intervals=[]))
            value = row['value']
            try: available = value is not None and math.isfinite(float(value))
            except (TypeError, ValueError): available = False
            if available:
                entry['available_sample_count'] += 1
                evidence = dict(row)
                if entry['first_valid_reading'] is None: entry['first_valid_reading'] = evidence
                entry['last_valid_reading'] = evidence
                if entry['missing_intervals'] and entry['missing_intervals'][-1]['open']:
                    entry['missing_intervals'][-1]['open'] = False
                if not key.startswith('brunner.'): p3d_available = True
            else:
                entry['missing_sample_count'] += 1
                intervals = entry['missing_intervals']
                if not intervals or not intervals[-1]['open']:
                    intervals.append(dict(first_missing_utc=stamp,last_missing_utc=stamp,samples=0,open=True))
                intervals[-1]['last_missing_utc'] = stamp
                intervals[-1]['samples'] += 1
        self.p3d_available_snapshots += int(p3d_available)

    def document(self):
        return dict(snapshot_count=self.snapshots, p3d_available_snapshots=self.p3d_available_snapshots,
            p3d_missing_snapshots=self.snapshots-self.p3d_available_snapshots,
            available_channels=[k for k,v in self.channels.items() if v['available_sample_count']],
            channels=self.channels, connection_and_availability_transitions=self.transitions,
            validity_note='Valid reading means finite recorded value only. Native freshness and BRUNNER physical freshness remain unverified; verified_sample_count is zero.')


class DiagnosticMonitor:
    def __init__(self, client, directory, status_file, *, display_cache=lambda: '', interval_s=.5, context_cache=lambda: {}):
        if interval_s < .1:
            raise ValueError('Diagnostic interval must be at least 0.1 seconds')
        self.reader = ReadOnlyTelemetry(client)
        self.directory, self.status_file = Path(directory), Path(status_file)
        self.display_cache, self.interval_s = display_cache, interval_s
        self.cancelled = threading.Event()
        self.thread = None
        self.rows = 0
        self.issues = set()
        self.outputs = {}
        self.context_cache = context_cache
        self.summary = SessionSummary()
        self.end_reason = None
        self.source_state = 'NOT_SAMPLED'
        self.publication_failures = 0
        self.publication_retries = 0
        self._publication_id = uuid4().hex

    def sample(self):
        timestamp = datetime.now(timezone.utc).isoformat()
        try:
            frame = self.reader.diagnostic_feedback()
            values = frame.values
            status = self.reader.diagnostic_state if frame.connected else 'DISCONNECTED'
            self.source_state = status
            issues = list(self.reader.issues)
            provenance = getattr(self.reader.client, 'diagnostic_provenance', None)
            if isinstance(provenance, str): issues.append(provenance)
            if not frame.connected:
                issues.append('P3D disconnected; no current simulator measurements')
            stamp, sequence = frame.measured_at, frame.sequence
        except Exception as exc:
            values, stamp, sequence = {}, None, None
            status, issues = 'UNAVAILABLE', [str(exc)]
            self.source_state = 'STALE_OR_REPEATED_CACHE' if 'repeated' in str(exc) or 'stale' in str(exc) else 'UNAVAILABLE'
        records = []
        channels = {key: unit for key, (unit, _) in FIELDS.items()}
        channels.update(total_fuel_quantity='kg', vertical_speed='ft/min',
                        indicated_altitude='ft indicated', ambient_pressure_mb='mbar',
                        pressure_altitude='ft pressure altitude', sim_paused='bool')
        for key, unit in channels.items():
            item = values.get(key)
            records.append(dict(observed_utc=timestamp, channel=key,
                value=item.value if item else None, unit=item.unit if item else unit,
                source=item.channel if item else 'No validated source',
                status=status if item else 'MISSING_OR_UNVALIDATED',
                sequence=sequence, cache_copy_monotonic=stamp,
                native_acquired_at=None, freshness='STALE' if status == 'STALE_CACHE' else 'UNVERIFIED',
                source_timestamp=self.reader.diagnostic_timing.get('source_timestamp'),
                source_clock=self.reader.diagnostic_timing.get('source_clock')))
        # Existing UI text is rounded and may be stale. Never claim native validity
        # or use this representation to authorize the manager.
        display = self.display_cache()
        for axis in ('Pitch', 'Roll', 'Rudder'):
            match = re.search(axis+r':\s*([+-]?\d+(?:\.\d+)?)', display)
            records.append(dict(observed_utc=timestamp, channel='brunner.'+axis.lower(),
                value=float(match.group(1)) if match else None, unit='normalized (rounded display)',
                source='Existing CLS2Sim UI cache; no additional read_axis calls',
                status='DISPLAY_ONLY_UNVERIFIED_POTENTIALLY_STALE' if match else 'MISSING_OR_UNVALIDATED',
                sequence=None, cache_copy_monotonic=None, native_acquired_at=None, freshness='UNVERIFIED',
                source_timestamp=None, source_clock=None))
        return records, issues

    def start(self):
        if self.thread is not None:
            raise RuntimeError('Monitor instances are single-use')
        self.directory.mkdir(parents=True, exist_ok=True)
        token = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_'+uuid4().hex[:8]
        self.outputs = {'csv': str(self.directory/(token+'.csv')),
                        'summary': str(self.directory/(token+'_summary.json'))}
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def stop(self, reason='operator_stop'):
        if not self.cancelled.is_set(): self.end_reason = reason
        self.cancelled.set()  # No GUI-thread join or hardware cleanup.

    def _publish(self, payload):
        # Private staging file: never share the ordinary QTG status writer's temp.
        temporary = self.status_file.with_name(self.status_file.name+'.'+self._publication_id+'.tmp')
        try:
            temporary.write_text(json.dumps(payload, allow_nan=False, indent=2), encoding='utf-8')
            for attempt in range(3):
                try:
                    temporary.replace(self.status_file)
                    return
                except PermissionError:
                    if attempt == 2: raise
                    self.publication_retries += 1
                    self.cancelled.wait(.02)
        finally:
            try: temporary.unlink(missing_ok=True)
            except OSError: pass

    def _safe_publish(self, payload):
        try: self._publish(payload)
        except OSError as exc:
            self.publication_failures += 1
            self.issues.add('Status publication failed (diagnostic only): '+str(exc))

    def _run(self):
        payload = dict(active=True, outputs=self.outputs, samples=0, measurements=[], issues=[])
        try:
            with Path(self.outputs['csv']).open('x', newline='', encoding='utf-8') as stream:
                writer = None
                while not self.cancelled.is_set():
                    records, issues = self.sample()
                    if writer is None:
                        writer = csv.DictWriter(stream, fieldnames=list(records[0]))
                        writer.writeheader()
                    writer.writerows(records)
                    stream.flush()
                    self.rows += 1
                    self.issues.update(issues)
                    self.summary.observe(records, self.source_state, self.context_cache())
                    payload.update(samples=self.rows, measurements=records, issues=issues, source_state=self.source_state)
                    self._safe_publish(payload)
                    self.cancelled.wait(self.interval_s)
        except Exception as exc:
            self.issues.add('Diagnostic failed: '+str(exc))
            self.end_reason = 'diagnostic_error'
        finally:
            payload.update(active=False, samples=self.rows, issues=sorted(self.issues),
                end_reason=self.end_reason or 'monitor_finished', session=self.summary.document(),
                status_publication_failures=self.publication_failures, status_publication_retries=self.publication_retries)
            # Final publication failure must not prevent saving the session summary.
            self._safe_publish(payload)
            payload.update(issues=sorted(self.issues), status_publication_failures=self.publication_failures,
                           status_publication_retries=self.publication_retries)
            try:
                Path(self.outputs['summary']).write_text(json.dumps(payload, indent=2, allow_nan=False), encoding='utf-8')
            except OSError as exc:
                payload['issues'].append('Summary save failed: '+str(exc))
                self._safe_publish(payload)
