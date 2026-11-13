"""Read-only telemetry adapters; never connect or write simulator state.

Uses an already-running client's is_connected/get_telemetry_sequence/get_telemetry.
Cache-copy freshness is not proof of native acquisition time; see source audit.
BRUNNER must be supplied as a separately timestamped cache, never read_axis().
The explicit authoritative path requests a verification-only native snapshot.
"""
from dataclasses import dataclass
import math
import time
from types import MappingProxyType

from backend.initialization_requirements import FIELDS
from backend.universal_initialization import Feedback, Measurement


@dataclass(frozen=True)
class CachedSource:
    connected: bool
    sequence: int
    measured_at: float
    clock: str
    values: dict


class TelemetryUnavailable(ValueError):
    pass


def finite(value):
    return isinstance(value, (int, float)) and math.isfinite(value)


class ReadOnlyTelemetry:
    """Feedback provider only, not a preparation/cleanup or ownership adapter."""
    def __init__(self, client, *, brunner_cache=None, max_age_s=1.,
                 clock=time.monotonic, wall_clock=time.time, clock_skew_s=.25):
        if not finite(max_age_s) or max_age_s <= 0 or not finite(clock_skew_s) or clock_skew_s < 0:
            raise ValueError('Invalid freshness limits')
        self.client = client
        self.brunner_cache = brunner_cache
        self.max_age_s, self.clock_skew_s = max_age_s, clock_skew_s
        self.clock, self.wall_clock = clock, wall_clock
        self._offset = wall_clock()-clock()
        self._seen = {}
        self.issues = []
        self.brunner = None
        self.diagnostic_state = 'NOT_SAMPLED'
        self.diagnostic_timing = {}
        self._diagnostic_previous = None
        self._diagnostic_sequence = None

    def diagnostic_feedback(self):
        """Display-only observations; never use these to authorize initialization."""
        return self.feedback(_diagnostic=True)

    def _diagnostic_stamp(self, raw, sequence):
        now, wall = self.clock(), self.wall_clock()
        raw_stamp = raw.get('last_updated')
        self.diagnostic_timing = {'source_timestamp': raw_stamp if finite(raw_stamp) else None,
            'source_clock': 'unix_seconds (cache copy / dashboard receive-age estimate)',
            'observed_wall_seconds': wall, 'observed_monotonic_seconds': now,
            'native_acquired_at': None}
        self.diagnostic_state = 'CONNECTED_FRESHNESS_UNVERIFIED'
        stamp = None
        # Compare wall timestamps directly in their domain; do not subtract a
        # long-lived epoch offset with nanosecond-level acceptance tolerance.
        if finite(raw_stamp):
            age = wall-raw_stamp
            self.diagnostic_timing['wall_age_seconds'] = age
            if abs((wall-now)-self._offset) > self.clock_skew_s:
                self.diagnostic_state = 'CLOCK_UNCERTAIN_UNVERIFIED'
            elif abs(age) > 86400:
                self.diagnostic_state = 'CLOCK_DOMAIN_UNCERTAIN_UNVERIFIED'
            elif age < -self.clock_skew_s:
                self.diagnostic_state = 'FUTURE_TIMESTAMP_UNVERIFIED'
            elif age > self.max_age_s:
                self.diagnostic_state = 'STALE_CACHE'
            elif age >= 0:
                stamp = now-age
            # Small negative age remains unverified, not a fabricated timestamp.
        previous = self._diagnostic_previous
        if previous and sequence == previous[0] and raw_stamp == previous[1]:
            if now-previous[2] > self.max_age_s:
                self.diagnostic_state = 'STALE_CACHE'
            elif self.diagnostic_state == 'CONNECTED_FRESHNESS_UNVERIFIED':
                self.diagnostic_state = 'UNCHANGED_FRESHNESS_UNVERIFIED'
        elif previous and type(sequence) is int and type(previous[0]) is int and sequence < previous[0]:
            self.diagnostic_state = 'OUT_OF_SEQUENCE_UNVERIFIED'
        else:
            self._diagnostic_previous = (sequence, raw_stamp, now)
        if self._diagnostic_sequence and sequence == self._diagnostic_sequence[0]:
            if now-self._diagnostic_sequence[1] > self.max_age_s:
                self.diagnostic_state = 'STALE_CACHE'
            elif self.diagnostic_state == 'CONNECTED_FRESHNESS_UNVERIFIED':
                self.diagnostic_state = 'UNCHANGED_FRESHNESS_UNVERIFIED'
        else:
            self._diagnostic_sequence = (sequence, now)
        if self.diagnostic_state != 'CONNECTED_FRESHNESS_UNVERIFIED':
            self.issues.append('Telemetry timing: '+self.diagnostic_state)
        return stamp

    def _stamp(self, name, source):
        now, wall = self.clock(), self.wall_clock()
        if abs((wall-now)-self._offset) > self.clock_skew_s:
            raise TelemetryUnavailable('Host wall/monotonic clock mapping changed')
        if source.clock not in ('unix_seconds', 'host_monotonic'):
            raise TelemetryUnavailable(f'{name}: unsupported telemetry clock')
        stamp = source.measured_at
        if not finite(stamp):
            raise TelemetryUnavailable(f'{name}: missing/nonfinite timestamp')
        if source.clock == 'unix_seconds':
            stamp -= self._offset
        if not -1e-9 <= now-stamp <= self.max_age_s:
            raise TelemetryUnavailable(f'{name}: stale/future timestamp or mismatched clock')
        stamp = min(stamp, now)  # Floating-point clock-offset cancellation only.
        if type(source.sequence) is not int or source.sequence <= 0:
            raise TelemetryUnavailable(f'{name}: invalid sequence')
        previous = self._seen.get(name)
        if previous and (source.sequence <= previous[0] or stamp <= previous[1]):
            raise TelemetryUnavailable(f'{name}: repeated/out-of-order snapshot')
        self._seen[name] = (source.sequence, stamp)
        return stamp

    def _authoritative_feedback(self, attempt_id):
        self.issues = []
        if not self.client.is_connected():
            raise TelemetryUnavailable('P3D disconnected')
        getter = getattr(self.client, 'verification_snapshot', None)
        if not callable(getter):
            raise TelemetryUnavailable('Native verification unsupported; cache-copy time is insufficient')
        try:
            raw = getter()
        except Exception as exc:
            raise TelemetryUnavailable(str(exc)) from exc
        if not self.client.is_connected() or raw.get('version') != 1:
            raise TelemetryUnavailable('Disconnected or unsupported verification snapshot')
        generation, request, sequence = (raw.get(k) for k in ('generation', 'request_id', 'sequence'))
        if any(type(v) is not int or v <= 0 for v in (generation, request, sequence)):
            raise TelemetryUnavailable('Invalid native verification identity')
        previous = getattr(self, '_native_previous', None)
        if previous and (generation != previous[0] or request <= previous[1] or sequence <= previous[2]):
            raise TelemetryUnavailable('Reconnected, repeated or out-of-order verification response; new attempt required')
        stamp = self._stamp('native', CachedSource(True, sequence, raw.get('measured_at'), 'host_monotonic', {}))
        self._native_previous = generation, request, sequence
        measurements = raw.get('values', {})
        sim_time = measurements.get('sim_time')
        if not finite(sim_time):
            raise TelemetryUnavailable('Simulator elapsed time unavailable')
        old_time = getattr(self, '_native_sim_time', None)
        if old_time is not None and sim_time < old_time:
            raise TelemetryUnavailable('Simulator time moved backwards; new attempt required')
        # A newly served paused snapshot can be unchanged. Callback freshness is
        # not proof of physics progression, even when simulator time advances.
        self.native_timing = dict(generation=generation, request_id=request, sequence=sequence,
            running=raw.get('running'), sim_time=sim_time,
            simulator_time_advanced=old_time is not None and sim_time > old_time,
            provenance=raw.get('provenance'), receipt_ticks=raw.get('receipt_ticks'),
            copied_ticks=raw.get('copied_ticks'), frequency=raw.get('frequency'),
            receipt_age_s=raw.get('receipt_age_s'), valid_mask=raw.get('valid_mask'),
            source_acquisition_time=None, sources=raw.get('sources', {}))
        self._native_sim_time = sim_time
        values = {}
        for key, (unit, channel) in FIELDS.items():
            value = measurements.get(key)
            if not finite(value) or (key == 'on_ground' and value not in (0, 1)) or (
                    key == 'flaps' and (value < 0 or value != int(value))):
                self.issues.append(key + ': native measurement missing, unsupported or invalid')
                continue
            values[key] = Measurement(value, unit, channel)
        for key, unit in (('ambient_pressure_mb', 'mbar'), ('indicated_altitude', 'ft indicated')):
            if finite(measurements.get(key)):
                values[key] = Measurement(measurements[key], unit, key)
        return Feedback(True, sequence, stamp, MappingProxyType(values))

    def feedback(self, attempt_id=None, *, _diagnostic=False, _authoritative=False):
        if _authoritative:
            return self._authoritative_feedback(attempt_id)

        self.issues = []
        if _diagnostic: self.diagnostic_timing = {}
        self.brunner = None
        if not self.client.is_connected():
            self.diagnostic_state = 'DISCONNECTED'
            return Feedback(False, 0, self.clock(), {})
        before = self.client.get_telemetry_sequence()
        raw = self.client.get_telemetry()
        after = self.client.get_telemetry_sequence()
        if before != after or raw.get('bridge_telemetry_sequence') != after:
            raise TelemetryUnavailable('P3D cache/bridge sequence mismatch; snapshot not coherent')
        if not self.client.is_connected():
            return Feedback(False, 0, self.clock(), {})
        stamp = (self._diagnostic_stamp(raw, after) if _diagnostic else
                 self._stamp('p3d', CachedSource(True, after, raw.get('last_updated'), 'unix_seconds', raw)))
        values = {}
        for key, (unit, channel) in FIELDS.items():
            # Ground contact is currently merged from the separately sequenced
            # extended block, with no own acquisition timestamp. Do not relabel it.
            if key in ('on_ground', 'flaps'):
                self.issues.append(f'{key}: independently timestamped/validated native source unavailable')
                continue
            source = 'eng_rpms' if key == 'engine_rpm' else 'prop_rpms' if key == 'prop_rpm' else channel
            if key == 'alt_ft':
                source = 'pressure_alt'  # Native PLANE ALTITUDE, NOT pressure altitude.
            value = raw.get(source)
            if key in ('engine_rpm', 'prop_rpm'):
                value = value[0] if isinstance(value, (list, tuple)) and value else None
            if not finite(value):
                self.issues.append(f'{key}: missing/nonfinite native measurement')
                continue
            values[key] = Measurement(value, unit, channel)
        native = values.get('engine_rpm')
        if native and finite(raw.get('engine_rpm')) and native.value != raw['engine_rpm']:
            self.issues.append('engine_rpm alias conflicts with native eng_rpms[0]; native retained')
        # Extra native main-block measurements, kept outside registry targets.
        for key, unit in (('total_fuel_quantity', 'kg'), ('vertical_speed', 'ft/min'),
                          ('indicated_altitude', 'ft indicated'), ('ambient_pressure_mb', 'mbar')):
            if finite(raw.get(key)):
                values[key] = Measurement(raw[key], unit, key)
        self.issues.append('P3D timestamp is Python cache-copy time, not native acquisition time')
        if self.brunner_cache is not None:
            source = self.brunner_cache()
            if not source.connected:
                self.issues.append('BRUNNER cache disconnected')
            else:
                try:
                    hardware_stamp = self._stamp('brunner', source)
                    hardware = {}
                    for key, value in source.values.items():
                        if not isinstance(value, Measurement) or not finite(value.value):
                            self.issues.append(f'BRUNNER {key}: missing/nonfinite or untyped reading')
                            continue
                        # Never merge physical axes into P3D surface/controller keys.
                        hardware['brunner.'+key] = value
                    self.brunner = Feedback(True, source.sequence, hardware_stamp, MappingProxyType(hardware))
                except TelemetryUnavailable as exc:
                    self.issues.append(str(exc))
        return Feedback(True, after, stamp, MappingProxyType(values))


class StabilityWindow:
    """Diagnostic range gate; combine with manager target/tolerance verification.

    Callers explicitly choose units-compatible range limits and time coverage.
    Gaps, missing channels and changed connectivity invalidate the whole window.
    """
    def __init__(self, limits, *, duration_s, min_samples, max_gap_s):
        if (not limits or any(not finite(v) or v < 0 for v in limits.values())
                or not finite(duration_s) or duration_s <= 0
                or type(min_samples) is not int or min_samples < 2
                or not finite(max_gap_s) or max_gap_s <= 0):
            raise ValueError('Explicit valid stability criteria required')
        self.limits = dict(limits)
        self.duration_s, self.min_samples, self.max_gap_s = duration_s, min_samples, max_gap_s
        self.frames = []

    def observe(self, frame):
        if (not frame.connected or not finite(frame.measured_at)
                or any(k not in frame.values or not finite(frame.values[k].value) for k in self.limits)):
            self.frames.clear()
            return False
        if self.frames:
            previous = self.frames[-1]
            if (frame.sequence <= previous.sequence or not 0 < frame.measured_at-previous.measured_at <= self.max_gap_s
                    or any((frame.values[k].unit, frame.values[k].channel) !=
                           (previous.values[k].unit, previous.values[k].channel) for k in self.limits)):
                self.frames.clear()
                return False
        self.frames.append(frame)
        while len(self.frames) > 1 and frame.measured_at-self.frames[1].measured_at >= self.duration_s:
            self.frames.pop(0)
        for key, limit in self.limits.items():
            values = [f.values[key].value for f in self.frames]
            if key in ('heading_deg', 'wind_direction'):
                values = [(v-values[0]+180) % 360-180 for v in values]
            if max(values)-min(values) > limit:
                self.frames = [frame]
                return False
        return len(self.frames) >= self.min_samples and frame.measured_at-self.frames[0].measured_at >= self.duration_s
