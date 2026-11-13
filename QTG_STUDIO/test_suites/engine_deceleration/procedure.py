"""Test-specific replay behavior; BRUNNER source remains unchanged."""
from backend.deceleration_config import display_defaults, load_initialization, load_input

TEST_REFERENCE = '1.f.(2)'
USE_SETTLED_REFERENCE_ALTIMETER = True
DEFAULT_CONDITIONS = display_defaults()


def initial_conditions():
    return load_initialization()
# A recorded throttle step is a discrete controller action, not a ramp.
# Leave gradual Manual throttle changes interpolated, and keep this policy
# confined to the deceleration replay pathway.
def make_step_throttle_replay(base_class, interpolate, enabled):
    import bisect
    import math

    class DecelerationThrottleReplay(base_class):
        _qtg_deceleration_step_replay = True

        def command(self, elapsed):
            if not enabled():
                return super().command(elapsed)
            if not math.isfinite(elapsed) or (self.last_elapsed is not None and elapsed < self.last_elapsed):
                raise ValueError('Throttle replay clock must be finite and monotonic.')
            self.last_elapsed = elapsed
            target = interpolate(self.times, self.values, elapsed)
            next_index = bisect.bisect_right(self.times, elapsed)
            if 0 < next_index < len(self.times):
                before = next_index - 1
                if abs(self.values[next_index] - self.values[before]) >= 20.0:
                    midpoint = (self.times[before] + self.times[next_index]) / 2.0
                    target = self.values[before] if elapsed < midpoint else self.values[next_index]
            if self.last_sent is None or abs(target - self.last_command) >= 0.05 or elapsed - self.last_sent >= 0.2:
                self.last_sent = elapsed
                self.last_command = target
                return target
            return None

    return DecelerationThrottleReplay

DEFAULT_DURATION_S = int(load_input()['duration_seconds'])

DURATION_SECONDS = DEFAULT_DURATION_S
DEFAULT_INITIAL_CONDITIONS = initial_conditions()
