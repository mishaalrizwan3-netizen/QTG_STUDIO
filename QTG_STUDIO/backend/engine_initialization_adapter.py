"""Default-off engine adapters around the existing preparation dispatcher.

No positioning, weather, engine, PID or actuator writes are implemented here.
Reviewed plans and authoritative feedback are requirements, not inferred approval.
"""
import hashlib
from dataclasses import replace
import json
import math
import os
from pathlib import Path
import threading
import time
from uuid import uuid4

from backend.initialization_requirements import REQUIREMENTS
from backend.telemetry_verification import ReadOnlyTelemetry, TelemetryUnavailable
from backend.universal_initialization import InitializationManager, Plan
from backend.initialization_plan_safety import (PlanBlocked, identity, verify_document,
                                                plan_identity)
from backend.initialization_configuration import (manual_plans, resolve_configuration,
    automatic_plans, ResolvedConfiguration, reviewed_manual_policy, manual_registry,
    automatic_registry)
from backend.initialization_reference import capture_reference


ENV = {'1.f.(2)': 'QTG_UNIVERSAL_ENGINE_DECELERATION',
       '1.f.(1)': 'QTG_UNIVERSAL_ENGINE_ACCELERATION'}
INITIALIZATION_ONLY = 'QTG_UNIVERSAL_INITIALIZATION_ONLY'
ATTEMPT_KEYS = ('manual_airflow_preparation', 'weather_diagnostics',
                'rpm_startup_readiness', 'engine_rpm_initialization',
                'manual_pose_readback', 'engine_altimeter_readiness', 'initialization_source')


def requested(context):
    if context.get('mode') == 'manual' and context.get('control_method', 'replay') == 'replay':
        if context.get('test_reference') in manual_plans():
            return True
    if context.get('mode') == 'auto':
        mode = 'pid' if context.get('control_method', 'replay') == 'pid' else 'recorded_replay'
        if mode in automatic_plans().get(context.get('test_reference'), {}):
            return True
    return (context.get('test_reference') in ENV
            and context.get('mode') in ('manual', 'auto')
            and context.get('control_method', 'replay') == 'replay'
            and os.environ.get(ENV[context['test_reference']]) == '1')


def initialization_only(window):
    """Latch the supervised trial interlock; changing the environment cannot unlock it."""
    return (getattr(window, '_initialization_only', False) is True
            or os.environ.get(INITIALIZATION_ONLY) == '1')


def validate_trial(window, mode):
    if not initialization_only(window):
        return
    window._initialization_only = True
    context = window._session_context
    if (mode == context.get('mode') == 'manual' and context.get('test_reference') in manual_plans()
            and requested(context) and os.environ.get('QTG_DECELERATION_BRAKE_CAPTURE') != '1'
            and all(os.environ.get(flag) != '1' for flag in ENV.values())):
        return  # Explicit per-test trial, independently reviewed; recording/PID lock is unchanged.
    if (context.get('test_reference') != '1.f.(2)' or mode != 'manual'
            or context.get('mode') != 'manual' or not requested(context)
            or os.environ.get(ENV['1.f.(1)']) == '1'
            or os.environ.get('QTG_DECELERATION_BRAKE_CAPTURE') == '1'):
        raise RuntimeError('Initialization-only trial requires Universal Deceleration Manual only; Acceleration and brake capture must be disabled')


def begin_attempt(window):
    context = window._session_context
    for key in ATTEMPT_KEYS:
        context.pop(key, None)
    window._manual_airflow_ready = None
    context['initialization_attempt'] = dict(
        attempt_id=uuid4().hex, test_reference=context.get('test_reference'),
        mode=context.get('mode'), selected_reference=context.get('reference_csv'),
        status='INITIALIZING', manager_enabled=False,
        verified_measurements={}, failure_reason=None)
    context['initialization_attempt']['initialization_only'] = initialization_only(window)
    try:
        enabled = requested(context)
    except PlanBlocked:
        # A malformed explicit opt-in is a blocked enabled request, never legacy.
        context['manual_manager_active'] = context.get('mode') == 'manual'
        context['universal_manager_active'] = True
        raise
    context['manual_manager_active'] = context.get('mode') == 'manual' and enabled
    context['universal_manager_active'] = enabled
    context['initialization_attempt']['manager_enabled'] = enabled


def mark(window, status, reason=None):
    record = getattr(window, '_session_context', {}).get('initialization_attempt')
    if record is not None:
        record['status'] = status
        if status == 'COMPLETE':
            manager = getattr(window, '_universal_manager', None)
            if manager is not None:
                manager.complete(record['attempt_id'])
        if reason:
            record['failure_reason'] = str(reason)


def configuration_snapshot(context):
    """Compatibility API for the shared reference-aware resolved configuration."""
    conditions, configuration, _ = resolve_configuration(context)
    return conditions, configuration


class ReviewedInputs:
    """Pin plan bytes and the final Manual configuration for the whole attempt."""
    def __init__(self, context, path):
        self.path = Path(path)
        self.plan_bytes = self.path.read_bytes()
        self.selection = {k: context.get(k) for k in ('test_reference', 'mode', 'control_method', 'reference_csv', 'configuration_revision')}
        self.conditions = self.configuration = None
        self.reference_snapshot = None
        self.reference_files = {}
        if context.get('mode') == 'manual':
            self.snapshot = ResolvedConfiguration.capture(context)
            self.conditions, self.configuration = self.snapshot.conditions, dict(self.snapshot.configuration)
            context['initial_conditions'] = dict(self.conditions)
        self.document = json.loads(self.plan_bytes.decode('utf-8-sig'))
        verify_document(self.document)
        if context.get('mode') == 'auto':
            self.reference_snapshot = capture_reference(context.get('reference_csv'), self.document['checks'])
        if context.get('reference_csv'):
            selected = Path(context['reference_csv']).resolve()
            for file in (selected, selected.with_name(selected.stem + '_init_conditions.json')):
                self.reference_files[file] = hashlib.sha256(file.read_bytes()).hexdigest()
        if self.configuration is not None and self.document.get('configuration_identity') != self.configuration:
            raise PlanBlocked('Reviewed Manual configuration identity mismatch')
        self.digest = identity(dict(plan_sha256=hashlib.sha256(self.plan_bytes).hexdigest(),
                                    configuration=self.configuration))
        self.manual_selection = selected_plan(context)
        self.check(context)

    def check(self, context):
        try:
            self._check(context)
        except PlanBlocked:
            raise
        except Exception as exc:
            raise PlanBlocked('Reviewed inputs unavailable or invalid: ' + str(exc)) from exc

    def _check(self, context):
        if self.manual_selection is not None and selected_plan(context) != self.manual_selection:
            raise PlanBlocked('Reviewed initialization plan selection changed')
        if self.path.read_bytes() != self.plan_bytes:
            raise PlanBlocked('Approved plan file changed; new review/attempt required')
        if {k: context.get(k) for k in self.selection} != self.selection:
            raise PlanBlocked('Reviewed test/mode/reference selection changed')
        for file, digest in self.reference_files.items():
            if hashlib.sha256(file.read_bytes()).hexdigest() != digest:
                raise PlanBlocked('Reviewed reference contents changed; new review/attempt required')
        if self.configuration is not None:
            _, current = configuration_snapshot(context)
            if current != self.configuration or identity(context.get('initial_conditions')) != identity(dict(self.conditions)):
                raise PlanBlocked('Reviewed configuration or operator overrides changed')


def load_plan(context, path, *, resolved_inputs=None):
    try:
        return _load_plan(context, path, resolved_inputs=resolved_inputs)
    except PlanBlocked:
        raise
    except Exception as exc:
        raise PlanBlocked('Incomplete or invalid initialization plan: ' + str(exc)) from exc


def _load_plan(context, path, *, resolved_inputs=None):
    """Use reviewed tolerances; obtain Replay targets only from its selected sidecar.

    No pre-event substitution or fallback to configured Manual Replay targets.
    The unchanged legacy loader still performs its own full reference validation.
    """
    document = json.loads(Path(path).read_text(encoding='utf-8-sig'))
    approval = verify_document(document)
    reference = context['test_reference']
    mode = ('manual' if context['mode'] == 'manual' else
            'pid' if context.get('control_method') == 'pid' else 'recorded_replay')
    if document.get('test_reference') != reference or document.get('mode') != mode:
        raise ValueError('Reviewed engine plan does not match test/mode')
    if mode == 'manual':
        return _load_manual_plan(context, document, approval, resolved_inputs)
    from backend.verifier import validate_reference
    captured = (resolved_inputs.reference_snapshot if resolved_inputs is not None
                else capture_reference(context.get('reference_csv'), document['checks']))
    selected = Path(validate_reference(captured.path)).resolve()
    if Path(document['reference_csv']).resolve() != selected:
        raise PlanBlocked('Reviewed plan selected Manual reference mismatch')
    if document.get('reference_identity') != dict(captured.identity):
        raise PlanBlocked('Reviewed reference contents identity mismatch')
    captured.conditions()  # Missing command inputs block before any preparation.
    registry, mandatory = automatic_registry(reference, mode, captured.values, document)
    policy = registry[reference]
    from interfaces.brunner_base.src.verification_snapshot import CHANNELS
    supported = {name for name, _, _ in CHANNELS}
    checks = {}
    for key, spec in document['checks'].items():
        if key not in supported:
            raise PlanBlocked(key + ': authoritative V1 verification unavailable; approved exclusion or validated source required')
        original = policy.requirements[key]
        value = captured.values.get(key)
        if 'value' in spec and spec['value'] != value:
            raise PlanBlocked(key + ': reviewed target differs from selected reference')
        if 'value' not in spec and spec.get('target_source') != 'manual_recording_t0':
            raise PlanBlocked(key + ': target_source=manual_recording_t0 required')
        if value is None:
            raise ValueError(f'{key}: no measured/established recording-start target')
        checks[key] = replace(original, value=value, source=captured.provenance[key],
            missing_reason=None, tolerance=spec['tolerance'],
            tolerance_source=spec['tolerance_source'], scope=mode)
    if not mandatory <= checks.keys():
        raise ValueError('Missing required engine readiness checks: ' + ', '.join(sorted(mandatory-checks.keys())))
    if reference == '1.f.(1)' and checks['ias_kt'].tolerance > 2.:
        raise ValueError('Acceleration IAS tolerance cannot exceed the existing 2 kt limit')
    plan = Plan(reference, mode, checks, document['exclusions'], document['review_source'],
                document['timeout_s'], document['max_age_s'], document['stable_samples'])
    if plan.stable_samples < 3:
        raise ValueError('Engine verification requires at least three samples')
    plan = replace(plan, binding=identity(dict(document=document, reference=dict(captured.identity),
        sample_index=captured.sample_index, recording_time_s=captured.time_s)))
    plan = replace(plan, approval=replace(approval, identity=plan_identity(plan)))
    validator = (InitializationManager(None) if registry is REQUIREMENTS
                 else InitializationManager(None, registry=registry))
    validator._validate(plan)  # No preparation before approval.
    return plan


def _load_manual_plan(context, document, approval, bound):
    if bound is not None:
        bound.check(context)
        conditions, configuration = dict(bound.conditions), bound.configuration
    else:
        conditions, configuration = configuration_snapshot(context)
    if document.get('configuration_identity') != configuration:
        raise PlanBlocked('Reviewed Manual configuration identity mismatch')
    context['initial_conditions'] = dict(conditions)
    reference = context['test_reference']
    policy = reviewed_manual_policy(document, reference, conditions)
    registry = manual_registry(reference, conditions, policy)
    from interfaces.brunner_base.src.verification_snapshot import CHANNELS
    supported = {name for name, _, _ in CHANNELS}
    checks, decisions = {}, []
    record = context.get('initialization_attempt')
    if isinstance(record, dict):
        record['reviewed_criteria'] = {key: dict(tolerance=spec.get('tolerance'),
            tolerance_source=spec.get('tolerance_source')) for key, spec in document['checks'].items()}
    for key, spec in document['checks'].items():
        if key not in REQUIREMENTS[reference].requirements or key not in supported:
            raise PlanBlocked(key + ': authoritative V1 verification unavailable; approved exclusion or validated source required')
        if spec.get('target_source') != 'resolved_initial_conditions' or 'value' in spec or 'value_source' in spec:
            decisions.append(key + ': use target_source=resolved_initial_conditions; duplicate plan target values are forbidden')
        if conditions.get(key) is None:
            decisions.append(key + ': no configured paused-initialization target in resolved QTG initial conditions')
        if spec.get('tolerance') is None or not spec.get('tolerance_source'):
            decisions.append(key + ': no approved tolerance/source')
        if decisions:
            continue
        checks[key] = replace(registry[reference].requirements[key], value=conditions[key],
            source='resolved QTG initial_conditions.' + key + ' (configuration identity bound)',
            missing_reason=None, tolerance=spec['tolerance'],
            tolerance_source=spec['tolerance_source'], scope='manual')
    if decisions:
        raise PlanBlocked('Missing Manual readiness decisions: ' + '; '.join(sorted(decisions)))
    if not policy.mandatory <= checks.keys():
        raise PlanBlocked('Missing required Manual readiness checks: ' + ', '.join(sorted(policy.mandatory-checks.keys())))
    if reference == '1.f.(1)' and checks['ias_kt'].tolerance > 2.:
        raise PlanBlocked('Acceleration IAS tolerance cannot exceed the existing 2 kt limit')
    if type(document.get('stable_samples')) is not int or document['stable_samples'] < 3:
        raise PlanBlocked('Manual verification requires at least three samples')
    plan = Plan(reference, 'manual', checks, document['exclusions'], document['review_source'],
        document['timeout_s'], document['max_age_s'], document['stable_samples'])
    plan = replace(plan, binding=identity(dict(document=document, configuration=conditions)))
    plan = replace(plan, approval=replace(approval, identity=plan_identity(plan)))
    validator = (InitializationManager(None) if registry is REQUIREMENTS
                 else InitializationManager(None, registry=registry))
    validator._validate(plan)
    return plan


class EngineAdapter:
    cleanup_owner = 'legacy_runtime'

    def __init__(self, window, dispatch, mode, reference, *, clock=time.monotonic):
        self.window, self.dispatch = window, dispatch
        self.mode, self.reference, self.clock = mode, reference, clock
        self.attempt_id = None
        self.preparation_started = False
        self.legacy_failure_cleanup_completed = False
        self.cancelled = threading.Event()
        self.thread = None
        self._lock = threading.Lock()
        self._frame = self._error = None
        self._delivered = None
        self._delivered_frame = None
        self._latest_observation = None
        self._diagnostic_signature = None
        self.reader = ReadOnlyTelemetry(window.p3d, clock=clock)

    def _check_owner(self):
        device = self.window.brunner
        if (getattr(self.window, 'pid_controller_active', False)
                or getattr(self.window, '_external_control_active', False)
                or getattr(getattr(device, '_pid_transport', None), 'owns_overrides', False) is True):
            raise RuntimeError('BRUNNER ownership conflict: another controller is active')

    def prepare(self, attempt_id, plan):
        if self.attempt_id is not None:
            raise RuntimeError('Engine adapter preparation already dispatched')
        self._check_owner()
        self.check_integrity()
        if getattr(self.window, 'is_recording', False):
            raise RuntimeError('Cannot initialize during recording')
        self.attempt_id = attempt_id
        self.plan = plan
        self.reader.max_age_s = plan.max_age_s
        record = self.window._session_context['initialization_attempt']
        if self.window._session_context.get('operator_initiated_universal_manual') is True:
            if (getattr(self.window, '_prepared', False)
                    or getattr(self.window, '_universal_recording_authorized', False)):
                raise RuntimeError('Recording authorization must be locked before initialization')
            self.reader.feedback(attempt_id, _authoritative=True)
            timing = dict(self.reader.native_timing)
            if timing.get('running') is not False:
                raise RuntimeError('P3D must be confirmed paused before initialization')
            self._check_owner()
            physical = self._physical_readback()
            self.check_integrity()
            if self.cancelled.is_set() or self.window.stop_worker_event.is_set():
                raise RuntimeError('Initialization cancelled during preflight')
            record['operator_preflight'] = dict(paused=True, native_verification=timing,
                physical_controls=physical, recording_locked=True,
                ownership='No in-process PID/replay ownership conflict; strict axis reads succeeded')
        record.update(attempt_id=attempt_id, status='PREPARING',
                      review_source=plan.review_source, exclusions=dict(plan.exclusions),
                      selected_system='Universal Initialization Manager',
                      preparation_dispatch='qtg_common_engine._initialize_mode',
                      command_acknowledgement='Not independently established; readiness requires measured verification')
        if getattr(self, 'reviewed_inputs', None) is not None:
            record['reviewed_input_identity'] = self.reviewed_inputs.digest
            record['configuration_identity'] = self.reviewed_inputs.configuration
            if hasattr(self.reviewed_inputs, 'snapshot'):
                record['configuration_provenance'] = {
                    key: dict(value) for key, value in self.reviewed_inputs.snapshot.provenance.items()}
            captured = self.reviewed_inputs.reference_snapshot
            if captured is not None:
                record['reference_identity'] = dict(captured.identity)
                record['reference_start'] = dict(sample_index=captured.sample_index,
                    recording_time_s=captured.time_s, alignment='recording start; not throttle event',
                    provenance=dict(captured.provenance))
        record['approved_plan_identity'] = plan.approval.identity
        record['planned_checks'] = {key: dict(target=check.value, unit=check.unit,
            channel=check.telemetry, target_source=check.source,
            tolerance=check.tolerance, tolerance_source=check.tolerance_source,
            scope=check.scope) for key, check in plan.checks.items()}
        self._diagnose(None, {})
        # Exactly one invocation. That dispatcher owns all existing simulator/hardware work.
        self.preparation_started = True
        record['preparation_dispatch_started'] = True
        self.dispatch(self.window, self.mode, self.reference)
        record['preparation_dispatch_returned'] = True

    def check_integrity(self):
        bound = getattr(self, 'reviewed_inputs', None)
        if bound is not None:
            bound.check(self.window._session_context)

    def bound_conditions(self):
        self.check_integrity()
        return dict(self.reviewed_inputs.conditions)

    def _diagnose(self, frame, timing):
        from backend.initialization_readiness_diagnostic import comparison_rows, text_table
        record = self.window._session_context.get('initialization_attempt', {})
        if record.get('attempt_id') != self.attempt_id:
            return
        rows = comparison_rows(self.plan, frame, timing, self.clock())
        record['readiness_diagnostics'] = rows
        record['readiness_diagnostic_text'] = text_table(rows)
        record['diagnostic_scope'] = 'Per-channel evidence only; READY also requires approval, consecutive samples, pause, ownership and lifecycle checks'
        signature = tuple((r['parameter'], r['status'], r['reason']) for r in rows)
        if signature != self._diagnostic_signature:
            print('[Universal readiness]\n' + record['readiness_diagnostic_text'], flush=True)
            self._diagnostic_signature = signature

    def _sample(self):
        frame = self._read_native()
        if self.reader.native_timing.get('running') is not False:
            raise TelemetryUnavailable('Final P3D pause not verified: simulator is running or state is unknown')
        return frame, self._physical_readback()

    def _read_native(self):
        self._latest_observation = None
        if self.cancelled.is_set() or self.window.stop_worker_event.is_set():
            raise RuntimeError('Engine verification cancelled')
        self._check_owner()
        self.check_integrity()
        # Read requests reuse the worker client, including while paused. No state writes.
        frame = self.reader.feedback(self.attempt_id, _authoritative=True)
        self._latest_observation = frame, dict(self.reader.native_timing)
        self._diagnose(*self._latest_observation)
        if not frame.connected:
            raise TelemetryUnavailable('P3D verification feedback disconnected')
        # Unsupported native DLLs never fall back to the legacy cache stream.
        gaps = [issue for issue in self.reader.issues if 'cache-copy time' in issue]
        gaps.extend(key + ': authoritative telemetry unavailable'
                    for key in self.plan.checks if key not in frame.values)
        if gaps:
            raise TelemetryUnavailable('; '.join(gaps))
        return frame

    def _physical_readback(self):
        device = self.window.brunner
        physical = {}
        with device._io_lock:
            if self.cancelled.is_set() or self.window.stop_worker_event.is_set():
                raise RuntimeError('Engine verification cancelled before hardware read')
            previous = getattr(device, 'strict_io', False)
            try:
                device.strict_io = True
                for name, axis in (('pitch',device.AXIS_PITCH), ('roll',device.AXIS_ROLL), ('rudder',device.AXIS_RUDDER)):
                    value = device.read_axis(axis)[0]
                    if not math.isfinite(value):
                        raise RuntimeError('Unavailable physical BRUNNER ' + name)
                    if self.mode == 'auto' and self.plan.mode != 'pid':
                        target = self.window.ref_manual_snapshot.get(name + '_pos')
                        if target is None or not math.isfinite(float(target)) or abs(value-float(target)) > .005:
                            raise RuntimeError('Physical BRUNNER ' + name + ' differs from selected Manual (>0.005 normalized, existing settle criterion)')
                    physical[name] = dict(value=value, unit='normalized position',
                                          source='BRUNNER direct read_axis', observed_at=self.clock())
            finally:
                device.strict_io = previous
        return physical

    def _collect(self):
        while not self.cancelled.is_set() and not self.window.stop_worker_event.is_set():
            try:
                frame, physical = self._sample()
                if frame is None:
                    self.cancelled.wait(.25)
                    continue
                with self._lock:
                    self._frame = frame
                    self.physical = physical
                    self.native_timing = dict(getattr(self.reader, 'native_timing', {}))
            except Exception as exc:
                with self._lock:
                    self._error = exc if isinstance(exc, PlanBlocked) else str(exc)
                if self._latest_observation is None:
                    self._diagnose(None, {})
                return
            self.cancelled.wait(.25)

    def feedback(self, attempt_id):
        if attempt_id != self.attempt_id or self.cancelled.is_set():
            raise RuntimeError('Superseded/cancelled engine feedback')
        try:
            self.check_integrity()
        except PlanBlocked:
            self._diagnose(None, {})
            raise
        if self.thread is None:
            self.thread = threading.Thread(target=self._collect, daemon=True)
            self.thread.start()
        with self._lock:
            if self._error:
                if isinstance(self._error, PlanBlocked):
                    raise self._error
                raise TelemetryUnavailable(self._error)
            frame = self._frame
            self._diagnose(frame, dict(getattr(self, 'native_timing', {})))
            if frame is None or frame.sequence == self._delivered:
                return None  # Nonblocking pending read; never authorizes recording.
            self._delivered = frame.sequence
            self._delivered_frame = frame
            timing = dict(getattr(self, 'native_timing', {}))
            if timing.get('sequence') != frame.sequence or timing.get('running') is not False:
                raise TelemetryUnavailable('Final P3D pause metadata missing or not correlated with verification frame')
            record = self.window._session_context['initialization_attempt']
            if record['attempt_id'] != attempt_id:
                raise RuntimeError('Superseded attempt metadata')
            record['observed_measurements'] = {key: dict(value=value.value, unit=value.unit,
                source=value.channel, measured_at=frame.measured_at, sequence=frame.sequence)
                for key, value in frame.values.items()}
            record['physical_controls'] = self.physical
            record['native_verification'] = timing
            return frame

    def confirm_verified(self):
        record = self.window._session_context['initialization_attempt']
        if record['attempt_id'] != self.attempt_id:
            raise RuntimeError('Superseded verification metadata')
        record['verified_measurements'] = {key: record['observed_measurements'][key]
            for key in self.plan.checks}

    def cleanup(self, attempt_id):
        if attempt_id == self.attempt_id:
            self.cancelled.set()
        # No pause, release, axis command, socket close or shared connection cleanup.

    def note_legacy_cleanup(self):
        """Called only after the unchanged Automatic failure cleanup returns."""
        self.legacy_failure_cleanup_completed = True
        record = self.window._session_context.get('initialization_attempt', {})
        if record.get('attempt_id') == self.attempt_id:
            record['cleanup_owner'] = 'legacy Automatic failure handler'
            record['cleanup_confirmation'] = 'Existing cleanup returned; physical release not independently verified'

    def handoff(self, *, for_recording=False):
        self.cancelled.set()  # Stop readiness sampling; recording owns the existing path.
        if self.thread is not None:
            self.thread.join(timeout=1.5)
            if self.thread.is_alive():
                raise RuntimeError('Verification reader has not stopped; recording/next initialization blocked')
        if for_recording:
            self.check_integrity()
            if self.window.stop_worker_event.is_set():
                raise RuntimeError('Recording handoff cancelled')
            if self._error:
                raise RuntimeError('Verification reader failed during handoff: ' + str(self._error))
            if self._delivered_frame is None or not 0 <= self.clock()-self._delivered_frame.measured_at <= self.plan.max_age_s:
                raise RuntimeError('Verification snapshot expired during recording handoff')
            self.confirm_verified()


class ManualPreparationAdapter(EngineAdapter):
    """Same dispatcher/reader/cleanup; optional approved running-window evidence."""
    def __init__(self, *args, policy, **kwargs):
        super().__init__(*args, **kwargs)
        self.policy = policy
        self.preparation_complete = threading.Event()
        self._verification_after = None
        self._running_verified_at = None
        self._final_stability_at = None
        self._window = None
        if policy.stability_kind == 'running_window':
            from backend.telemetry_verification import StabilityWindow
            self._window = StabilityWindow(dict(policy.limits), duration_s=policy.duration_s,
                min_samples=policy.min_samples, max_gap_s=policy.max_gap_s)

    def prepare(self, attempt_id, plan):
        super().prepare(attempt_id, plan)
        record = self.window._session_context['initialization_attempt']
        record['stability_scope'] = self.policy.stability_kind
        record['running_stability_verified'] = False
        if self._window is not None and not self.cancelled.is_set():
            self.thread = threading.Thread(target=self._collect, daemon=True)
            self.thread.start()

    def _physical_readback(self):
        return super()._physical_readback() if self.policy.physical_controls else {}

    def preparation_finished(self):
        if not self.preparation_complete.is_set():
            self._verification_after = self.clock()
            self.preparation_complete.set()

    def _sample(self):
        if self._window is None:
            return super()._sample()
        frame = self._read_native()
        running = self.reader.native_timing.get('running')
        if not self.preparation_complete.is_set():
            if running is not True:
                # Paused responses cannot establish or extend a running-flight window.
                if (self._running_verified_at is None
                        or frame.measured_at-self._running_verified_at > self.policy.max_gap_s):
                    self._window.frames.clear()
                    self._running_verified_at = None
            elif self.reader.native_timing.get('simulator_time_advanced') is not True:
                self._window.frames.clear()
                self._running_verified_at = None
            else:
                in_band = True
                for key in self.policy.limits:
                    measurement, check = frame.values[key], self.plan.checks[key]
                    error = abs(measurement.value-check.value)
                    if key in ('heading_deg', 'wind_direction'):
                        error = abs((measurement.value-check.value+180) % 360-180)
                    in_band &= error <= check.tolerance
                if not in_band:
                    self._window.frames.clear()
                    self._running_verified_at = None
                elif self._window.observe(frame):
                    self._running_verified_at = frame.measured_at
            return None, {}
        if self._verification_after is None:
            raise TelemetryUnavailable('Preparation completion timing unavailable')
        if frame.measured_at < self._verification_after:
            return None, {}  # A request crossing the preparation/verification boundary.
        if running is not False:
            raise TelemetryUnavailable('Final P3D pause not verified: simulator is running or state is unknown')
        if self._final_stability_at is None and (self._running_verified_at is None
                or not 0 <= frame.measured_at-self._running_verified_at <= self.policy.max_gap_s):
            raise TelemetryUnavailable('Reviewed running stabilization not established or expired; paused snapshots are not stabilized flight. A validated preparation method is required')
        if self._final_stability_at is None:
            self._final_stability_at = frame.measured_at
        record = self.window._session_context['initialization_attempt']
        record['running_stability_verified'] = True
        record['stability_evidence'] = dict(kind='running_window',
            last_running_measured_at=self._running_verified_at, final_paused_measured_at=frame.measured_at,
            duration_s=self.policy.duration_s, samples=len(self._window.frames),
            source='authoritative native feedback while simulator running')
        return frame, self._physical_readback()


def selected_plan(context):
    path = manual_plans().get(context['test_reference']) if context.get('mode') == 'manual' else None
    if context.get('mode') == 'auto':
        mode = 'pid' if context.get('control_method') == 'pid' else 'recorded_replay'
        path = automatic_plans().get(context['test_reference'], {}).get(mode)
    if path is None and context['test_reference'] in ENV:
        path = os.environ.get(ENV[context['test_reference']] + '_PLAN')
    return path


def install(window, mode, reference, dispatch):
    context = window._session_context
    path = selected_plan(context)
    if not path:
        raise PlanBlocked('Engine Universal initialization requires a reviewed test/mode plan file; no baseline auto-approval')
    try:
        bound = ReviewedInputs(context, path)
    except Exception as exc:
        raise PlanBlocked(str(exc)) from exc
    plan = load_plan(context, path, resolved_inputs=bound)
    bound.check(context)
    if mode == 'manual':
        policy = reviewed_manual_policy(bound.document, context['test_reference'], bound.conditions)
        adapter = ManualPreparationAdapter(window, dispatch, mode, reference, policy=policy)
    elif plan.mode == 'pid':
        adapter = PidPreparationAdapter(window, dispatch, mode, reference)
    else:
        adapter = EngineAdapter(window, dispatch, mode, reference)
    adapter.reviewed_inputs = bound
    window._engine_initialization_adapter = adapter
    if mode == 'manual' and REQUIREMENTS[context['test_reference']].status != 'CONFIGURED':
        window._universal_manager = InitializationManager(adapter,
            registry=manual_registry(context['test_reference'], bound.conditions, policy))
    elif mode == 'auto' and REQUIREMENTS[context['test_reference']].status != 'CONFIGURED':
        registry, _ = automatic_registry(context['test_reference'], plan.mode,
                                         bound.reference_snapshot.values, bound.document)
        window._universal_manager = InitializationManager(adapter, registry=registry)
    else:
        window._universal_manager = InitializationManager(adapter)
    window._universal_plan = plan


class PidPreparationAdapter(EngineAdapter):
    """Paused reference preparation -> single authorized handoff to existing PID."""
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.execution_allowed = threading.Event()

    def prepare_conditions(self):
        self.check_integrity()
        self._check_owner()
        captured = self.reviewed_inputs.reference_snapshot
        from backend.initializer import apply_manual_conditions
        conditions = captured.conditions()
        self.window.ref_manual_snapshot = captured.snapshot()
        actual = apply_manual_conditions(self.window, conditions)
        self.window._restore_reference_weather()
        self.window._restore_reference_state()
        self.check_integrity()
        return actual

    def wait_for_execution(self):
        window = self.window
        window.sig_update_stage.emit(window._base_module.STAGE_CONNECTED_READY,
            'Reference conditions applied; awaiting Universal verification before PID handoff.')
        while not self.execution_allowed.wait(.05):
            if window.stop_worker_event.is_set():
                raise RuntimeError('Universal PID preparation cancelled')
            self.check_integrity()
            manager = window._universal_manager
            if manager.attempt_id != self.attempt_id or manager.state.value in ('BLOCKED', 'FAILED', 'CANCELLED'):
                raise RuntimeError('Universal PID preparation failed or was superseded')
            if self.clock() >= manager._deadline:
                manager.fail(self.attempt_id, 'Initialization timeout before PID handoff')
                raise RuntimeError('Initialization timeout before PID handoff')
        if window.stop_worker_event.is_set() or window._universal_attempt_id != self.attempt_id:
            raise RuntimeError('Universal PID handoff cancelled or superseded')

def authorize_pid_recording(window):
    """Guard the PID's internal transition; never recheck t=0 targets in flight."""
    from backend.universal_initialization import State
    adapter = getattr(window, '_engine_initialization_adapter', None)
    manager = getattr(window, '_universal_manager', None)
    attempt = getattr(window, '_universal_attempt_id', None)
    if (initialization_only(window) or not isinstance(adapter, PidPreparationAdapter)
            or manager is None or manager.attempt_id != attempt or adapter.attempt_id != attempt
            or manager.state != State.EXECUTING or not adapter.execution_allowed.is_set()
            or window.stop_worker_event.is_set() or not window.p3d.is_connected()
            or not window.brunner.sock):
        raise RuntimeError('Universal authorization required before PID recording')
    adapter.check_integrity()
    if not manager.recording_started(attempt):
        raise RuntimeError('Universal PID recording authorization invalidated')
    mark(window, 'RECORDING')
