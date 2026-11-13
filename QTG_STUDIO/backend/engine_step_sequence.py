"""Offline engine-step state machine. It does not connect to hardware.

The caller must serialize samples, cancellation and dispatch of returned actions.
Receipt timestamps must be in the same monotonic clock domain as ``now``.
This module is not wired into the live test worker.
"""
from dataclasses import dataclass
import math

ENGINE_TARGET_RPM = {'1.f.(1)': 2700.0, '1.f.(2)': 972.0}


@dataclass(frozen=True)
class Settings:
    target_rpm: float
    initial_throttle: float
    final_throttle: float
    starting_throttle_tolerance: float
    throttle_event_delta: float
    engine_response_delta: float
    max_receipt_age: float
    baseline_seconds: float = 5.0
    rpm_tolerance: float = 50.0
    stable_seconds: float = 3.0
    max_post_command_seconds: float = 30.0

    def __post_init__(self):
        values = tuple(vars(self).values())
        if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in values):
            raise ValueError('All sequence settings must be finite numbers')
        if not 3 <= self.baseline_seconds <= 5:
            raise ValueError('Baseline must be between 3 and 5 seconds')
        if not 0 <= self.initial_throttle <= 100 or not 0 <= self.final_throttle <= 100:
            raise ValueError('Throttle must be between 0 and 100 percent')
        if self.initial_throttle == self.final_throttle or self.target_rpm <= 0:
            raise ValueError('A valid throttle step and RPM target are required')
        if min(self.starting_throttle_tolerance, self.throttle_event_delta, self.engine_response_delta, self.max_receipt_age,
               self.rpm_tolerance, self.stable_seconds, self.max_post_command_seconds) <= 0:
            raise ValueError('Detection thresholds and durations must be positive')
        if self.throttle_event_delta > abs(self.final_throttle - self.initial_throttle):
            raise ValueError('Throttle event threshold exceeds the step')


class EngineStepSequence:
    """Accept fresh measurements and return at most one throttle action."""
    TERMINAL = frozenset(('COMPLETE', 'ABORTED', 'FAILED', 'NOT STABILIZED'))

    def __init__(self, settings):
        self.settings = settings
        self.phase = 'IDLE'
        self.events = {}
        self.reason = ''
        self.last_sequence = None
        self.last_receipt = None
        self.stable_since = None
        self.initial_rpm = None
        self._responded = False

    def start(self, now):
        if self.phase != 'IDLE' or not math.isfinite(now):
            raise ValueError('Cannot start this attempt')
        self.phase = 'RECORDING BASELINE'
        self.events['recording_start'] = now

    def abort(self, now, reason='Operator aborted'):
        if self.phase not in self.TERMINAL:
            self.phase, self.reason = 'ABORTED', reason
            self.events['recording_stop'] = now

    def fail(self, now, reason):
        if self.phase not in self.TERMINAL:
            self.phase, self.reason = 'FAILED', reason
            self.events['recording_stop'] = now

    def tick(self, now):
        """Enforce deadlines even if no new telemetry arrives."""
        if self.phase in self.TERMINAL or self.phase == 'IDLE':
            return
        last = self.last_receipt if self.last_receipt is not None else self.events['recording_start']
        if now - last > self.settings.max_receipt_age:
            self.fail(now, 'Native telemetry stopped advancing')
        elif ('throttle_command' in self.events and
              now - self.events['throttle_command'] >= self.settings.max_post_command_seconds):
            self.phase, self.reason = 'NOT STABILIZED', 'RPM did not stabilize before the deadline'
            self.events['recording_stop'] = now

    def sample(self, *, now, sequence, receipt_time, rpm, throttle, connected=True):
        if self.phase == 'IDLE' or self.phase in self.TERMINAL:
            return None
        if not connected:
            self.fail(now, 'P3D disconnected')
            return None
        if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in (now, receipt_time, rpm, throttle)):
            self.fail(now, 'Missing or nonfinite telemetry')
            return None
        if (not isinstance(sequence, int) or sequence < 0 or receipt_time > now
                or now - receipt_time > self.settings.max_receipt_age
                or (self.last_sequence is not None and sequence <= self.last_sequence)
                or (self.last_receipt is not None and receipt_time <= self.last_receipt)
                or receipt_time - (self.last_receipt if self.last_receipt is not None else self.events['recording_start']) > self.settings.max_receipt_age
                or now < self.events['recording_start']):
            self.fail(now, 'Stale, duplicate, future or out-of-order telemetry')
            return None
        self.last_sequence, self.last_receipt = sequence, receipt_time
        if self.phase == 'RECORDING BASELINE':
            if abs(throttle - self.settings.initial_throttle) > self.settings.starting_throttle_tolerance:
                self.fail(now, 'Starting throttle does not match the configured condition')
                return None
            self.initial_rpm = rpm
            if now - self.events['recording_start'] >= self.settings.baseline_seconds:
                self.events['throttle_command'] = now
                self.phase = 'THROTTLE COMMAND'
                return {'type': 'set_throttle', 'percent': self.settings.final_throttle}
            return None
        if now - self.events['throttle_command'] >= self.settings.max_post_command_seconds:
            self.phase, self.reason = 'NOT STABILIZED', 'RPM did not stabilize before the deadline'
            self.events['recording_stop'] = now
            return None
        direction = 1 if self.settings.final_throttle > self.settings.initial_throttle else -1
        if 'throttle_onset' not in self.events and direction * (throttle - self.settings.initial_throttle) >= self.settings.throttle_event_delta:
            self.events['throttle_onset'] = now
            self.phase = 'WAITING FOR RPM'
        if direction * (rpm - self.initial_rpm) >= self.settings.engine_response_delta:
            self._responded = True
        eligible = 'throttle_onset' in self.events and self._responded
        if eligible and abs(rpm - self.settings.target_rpm) <= self.settings.rpm_tolerance:
            if self.stable_since is None:
                self.stable_since = now
                self.events['stabilization_start'] = now
            self.phase = 'STABILIZING'
            if now - self.stable_since >= self.settings.stable_seconds:
                self.phase = 'COMPLETE'
                self.events['stabilization_confirmed'] = now
                self.events['recording_stop'] = now
        else:
            self.stable_since = None
            self.events.pop('stabilization_start', None)
            if eligible:
                self.phase = 'WAITING FOR RPM'
        return None
