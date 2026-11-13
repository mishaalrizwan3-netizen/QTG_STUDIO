"""Standalone, caller-driven initialization core. No live adapters or runtime wiring.

Adapters must be nonblocking and serialize their own I/O. The owner must call
poll() regularly to enforce deadlines; this core creates no threads or timers.
Reviewed plans are explicit inputs, not approvals inferred from CONFIGURED.
"""
from dataclasses import dataclass, replace
from enum import Enum
import math
import threading
import time
from types import MappingProxyType
from typing import Mapping, Protocol
from uuid import uuid4

from backend.initialization_requirements import REQUIREMENTS, Requirement, validate_policy
from backend.initialization_plan_safety import PlanApproval, PlanBlocked, plan_identity, validate_approval_record


class State(str, Enum):
    VALIDATING = 'VALIDATING'
    BLOCKED = 'BLOCKED'
    PREPARING = 'PREPARING'
    VERIFYING = 'VERIFYING'
    READY = 'READY'
    EXECUTING = 'EXECUTING'  # Verified initialization handed to an existing PID workflow.
    RECORDING = 'RECORDING'
    COMPLETE = 'COMPLETE'
    FAILED = 'FAILED'
    CANCELLED = 'CANCELLED'


@dataclass(frozen=True)
class Plan:
    reference: str
    mode: str
    checks: Mapping[str, Requirement]
    exclusions: Mapping[str, str]
    review_source: str
    timeout_s: float
    max_age_s: float
    stable_samples: int
    approval: PlanApproval | None = None
    binding: str = ''


@dataclass(frozen=True)
class Measurement:
    value: float
    unit: str
    channel: str


@dataclass(frozen=True)
class Feedback:
    connected: bool
    sequence: int
    measured_at: float  # Same monotonic clock domain as manager.
    values: Mapping[str, Measurement]


class Adapter(Protocol):
    def prepare(self, attempt_id: str, plan: Plan) -> None:
        """Start preparation; completion calls manager.prepared(attempt_id)."""
    def feedback(self, attempt_id: str) -> Feedback | None:
        """Return measured feedback, or None while awaiting a new frame; never ACKs."""
    def cleanup(self, attempt_id: str) -> None:
        """Cancel producers and request pause/release; raise on incomplete cleanup."""


def _positive(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value) and value > 0


class InitializationManager:
    def __init__(self, adapter: Adapter, *, registry=None, clock=time.monotonic):
        self.adapter = adapter
        self.registry = dict(REQUIREMENTS if registry is None else registry)
        self.clock = clock
        self._lock = threading.RLock()
        self.attempt_id = None
        self.state = None
        self.reason = ''
        self.cleanup_errors = []
        self.history = []
        self._plan = None
        self._deadline = None
        self._last_sequence = None
        self._last_time = None
        self._matches = 0
        self._feedback_after = None
        self._authorized_sequence = None

    def _state(self, state):
        self.state = state
        self.history.append((self.attempt_id, state))

    def _terminate(self, state, reason):
        self._state(state)
        self.reason = reason
        self._matches = 0
        try:
            self.adapter.cleanup(self.attempt_id)
        except Exception as exc:
            self.cleanup_errors.append(f'{self.attempt_id}: {exc}')

    def _validate(self, plan):
        validate_approval_record(plan.approval)
        if plan.approval.identity != plan_identity(plan):
            raise PlanBlocked('Approved compiled plan identity mismatch')
        policy = self.registry.get(plan.reference)
        if policy is None:
            raise ValueError('Unknown test reference')
        if validate_policy(policy):
            raise ValueError('Invalid registry policy')
        if plan.mode not in policy.modes:
            raise ValueError('Unsupported initialization mode')
        if policy.status != 'CONFIGURED':
            raise ValueError('Policy is UNCONFIGURED')
        if not isinstance(plan.review_source, str) or not plan.review_source.strip():
            raise ValueError('Explicit reviewed plan required; baseline is not authorization')
        if not _positive(plan.timeout_s) or not _positive(plan.max_age_s):
            raise ValueError('Explicit finite positive deadlines required')
        if type(plan.stable_samples) is not int or plan.stable_samples < 2:
            raise ValueError('At least two independent verification samples required')
        keys = set(policy.requirements)
        if not plan.checks or set(plan.checks) & set(plan.exclusions) or set(plan.checks) | set(plan.exclusions) != keys:
            raise ValueError('Every requirement needs a check or explicit reviewed exclusion')
        if any(not isinstance(reason, str) or not reason.strip() for reason in plan.exclusions.values()):
            raise ValueError('Exclusions require justification')
        for key, check in plan.checks.items():
            original = policy.requirements[key]
            if (not isinstance(check.value, (int, float)) or not math.isfinite(check.value)
                    or (isinstance(check.value, bool) and original.unit != 'bool')
                    or not isinstance(check.source, str) or not check.source.strip()
                    or check.missing_reason or check.scope != plan.mode):
                raise ValueError(f'{key}: missing/unsupported target, source or mode scope')
            if check.unit != original.unit or check.telemetry != original.telemetry:
                raise ValueError(f'{key}: units/channel/altitude reference mismatch')
            if (not isinstance(check.tolerance, (int, float)) or isinstance(check.tolerance, bool)
                    or not math.isfinite(check.tolerance) or check.tolerance < 0
                    or not isinstance(check.tolerance_source, str) or not check.tolerance_source.strip()):
                raise ValueError(f'{key}: explicit justified tolerance required')
            replay_only = key == 'engine_rpm' and plan.reference in ('1.f.(1)', '1.f.(2)')
            if original.tolerance is not None and (not replay_only or plan.mode == 'recorded_replay') and check.tolerance != original.tolerance:
                raise ValueError(f'{key}: existing test-specific tolerance must be preserved')

    def start(self, plan: Plan, *, attempt_id=None):
        with self._lock:
            if attempt_id is not None and (not isinstance(attempt_id, str) or not attempt_id.strip()
                    or any(item[0] == attempt_id for item in self.history)):
                raise ValueError('Initialization attempt ID must be unique and nonempty')
            if self.attempt_id and self.state not in (State.BLOCKED, State.FAILED, State.CANCELLED, State.COMPLETE):
                self._terminate(State.CANCELLED, 'Superseded by new attempt')
            self.attempt_id = attempt_id or uuid4().hex
            self._state(State.VALIDATING)
            self.reason = ''
            self._plan = None
            self._deadline = self._last_sequence = self._last_time = None
            self._matches = 0
            self._authorized_sequence = None
            try:
                if self.cleanup_errors:
                    raise ValueError('Previous cleanup failed; use a new manager after external recovery')
                # Snapshot caller mappings so subsequent edits cannot change checks.
                plan = replace(plan, checks=MappingProxyType(dict(plan.checks)),
                               exclusions=MappingProxyType(dict(plan.exclusions)))
                try:
                    self._validate(plan)
                except Exception as exc:
                    raise PlanBlocked(str(exc)) from exc
                self._plan = plan
                self._deadline = self.clock() + plan.timeout_s
                self._state(State.PREPARING)
                self.adapter.prepare(self.attempt_id, plan)
            except PlanBlocked as exc:
                self._terminate(State.BLOCKED, str(exc))
            except Exception as exc:
                self._terminate(State.FAILED, str(exc))
            return self.attempt_id

    def prepared(self, attempt_id, *, success=True, reason='Preparation failed'):
        with self._lock:
            if attempt_id != self.attempt_id or self.state != State.PREPARING:
                return False
            if self.clock() >= self._deadline:
                self._terminate(State.FAILED, 'Initialization timeout')
            elif not success:
                self._terminate(State.FAILED, reason)
            else:
                self._feedback_after = self.clock()
                self._state(State.VERIFYING)
                return True
            return False

    def cancel(self, attempt_id):
        with self._lock:
            if attempt_id is None or attempt_id != self.attempt_id or self.state in (State.BLOCKED, State.FAILED, State.CANCELLED, State.COMPLETE):
                return False
            self._terminate(State.CANCELLED, 'Cancelled')
            return True

    def fail(self, attempt_id, reason):
        """Attempt-tagged preparation/runtime failure; never revive a terminal run."""
        with self._lock:
            if attempt_id is None or attempt_id != self.attempt_id or self.state in (State.BLOCKED, State.FAILED, State.CANCELLED, State.COMPLETE):
                return False
            self._terminate(State.FAILED, reason)
            return True

    def poll(self, attempt_id):
        with self._lock:
            if attempt_id != self.attempt_id or self.state not in (State.PREPARING, State.VERIFYING, State.READY):
                return False
            if self.state != State.READY and self.clock() >= self._deadline:
                self._terminate(State.FAILED, 'Initialization timeout')
                return False
            if self.state == State.PREPARING:
                return False
            try:
                frame = self.adapter.feedback(attempt_id)
                if frame is None:
                    if self._last_time is not None and self.clock()-self._last_time > self._plan.max_age_s:
                        raise ValueError('Stale telemetry while waiting for a new snapshot')
                    return False  # A nonblocking adapter has no new frame yet.
                now = self.clock()
                if self.state != State.READY and now >= self._deadline:
                    raise ValueError('Initialization timeout')
                if not frame.connected:
                    raise ValueError('Simulator feedback disconnected')
                if (not math.isfinite(frame.measured_at) or frame.measured_at < self._feedback_after
                        or not 0 <= now-frame.measured_at <= self._plan.max_age_s
                        or (self._last_time is not None and frame.measured_at <= self._last_time)
                        or type(frame.sequence) is not int
                        or (self._last_sequence is not None and frame.sequence <= self._last_sequence)):
                    raise ValueError('Stale or invalid telemetry')
                for key, check in self._plan.checks.items():
                    value = frame.values.get(key)
                    if value is None or not math.isfinite(value.value):
                        raise ValueError(f'{key}: unavailable/nonfinite telemetry')
                    if value.unit != check.unit or value.channel != check.telemetry:
                        raise ValueError(f'{key}: feedback units/channel mismatch')
                    error = abs(value.value-check.value)
                    if key in ('heading_deg', 'wind_direction'):
                        error = abs((value.value-check.value+180) % 360-180)
                    if error > check.tolerance:
                        raise ValueError(f'{key}: measured value outside verified tolerance '
                                         f'(target={check.value:g}, actual={value.value:g}, '
                                         f'tolerance={check.tolerance:g} {check.unit}, '
                                         f'sequence={frame.sequence})')
                self._last_sequence, self._last_time = frame.sequence, frame.measured_at
                self._matches += 1
                if self._matches >= self._plan.stable_samples and self.state != State.READY:
                    self._state(State.READY)
            except PlanBlocked as exc:
                self._terminate(State.BLOCKED, str(exc))
            except Exception as exc:
                self._terminate(State.FAILED, str(exc))
            return self.state == State.READY

    def authorize_recording(self, attempt_id):
        """Authorization requires READY plus another fresh, connected readback."""
        with self._lock:
            if attempt_id != self.attempt_id or self.state != State.READY or self.cleanup_errors:
                return False
            authorized = self.poll(attempt_id)
            self._authorized_sequence = self._last_sequence if authorized else None
            return authorized

    def recording_started(self, attempt_id):
        """Consume fresh authorization once; cease initialization target monitoring."""
        with self._lock:
            if (attempt_id == self.attempt_id and self.state == State.EXECUTING
                    and self._plan.mode == 'pid'):
                self._state(State.RECORDING)
                return True
            if (attempt_id != self.attempt_id or self.state != State.READY
                    or self._authorized_sequence is None or self._authorized_sequence != self._last_sequence
                    or not 0 <= self.clock()-self._last_time <= self._plan.max_age_s):
                return False
            self._authorized_sequence = None
            self._state(State.RECORDING)
            return True

    def execution_started(self, attempt_id):
        """Consume fresh paused authorization before handing control to PID.

        Later stabilization uses the existing PID algorithm, not frozen t=0 checks.
        Cancellation/failure still invalidates the attempt before recording.
        """
        with self._lock:
            if (attempt_id != self.attempt_id or self.state != State.READY
                    or self._plan.mode != 'pid' or self._authorized_sequence is None
                    or self._authorized_sequence != self._last_sequence
                    or not 0 <= self.clock()-self._last_time <= self._plan.max_age_s):
                return False
            self._authorized_sequence = None
            self._state(State.EXECUTING)
            return True

    def complete(self, attempt_id):
        with self._lock:
            if attempt_id != self.attempt_id or self.state != State.RECORDING:
                return False
            self._state(State.COMPLETE)
            return True
