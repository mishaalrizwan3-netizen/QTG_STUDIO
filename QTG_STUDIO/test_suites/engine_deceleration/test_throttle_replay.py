"""Offline checks for the deceleration-only throttle replay policy."""
import importlib.util
from pathlib import Path
import unittest


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


ROOT = Path(__file__).resolve().parents[2]
POLICY = load('deceleration_policy', Path(__file__).with_name('procedure.py'))
REPLAY = load('brunner_replay_support', ROOT / 'interfaces' / 'brunner_base' / 'src' / 'replay_support.py')


class ThrottleStepTests(unittest.TestCase):
    def make_replay(self, values, enabled=True):
        cls = POLICY.make_step_throttle_replay(
            REPLAY.ThrottleReplay, REPLAY.interpolate_reference, lambda: enabled)
        return cls([0.0, 1.0, 1.02, 2.0], values)

    def test_abrupt_deceleration_is_a_step(self):
        replay = self.make_replay([100.0, 100.0, 5.0, 5.0])
        self.assertEqual(replay.command(0.0), 100.0)
        self.assertEqual(replay.command(1.005), 100.0)
        self.assertEqual(replay.command(1.011), 5.0)
        self.assertIsNone(replay.command(1.015))

    def test_gradual_change_keeps_interpolation(self):
        replay = self.make_replay([100.0, 100.0, 90.0, 90.0])
        replay.command(0.0)
        self.assertAlmostEqual(replay.command(1.01), 95.0)

    def test_other_tests_keep_base_interpolation(self):
        replay = self.make_replay([100.0, 100.0, 5.0, 5.0], enabled=False)
        replay.command(0.0)
        self.assertAlmostEqual(replay.command(1.005), 76.25)

    def test_clock_must_remain_monotonic(self):
        replay = self.make_replay([100.0, 100.0, 5.0, 5.0])
        replay.command(1.011)
        with self.assertRaisesRegex(ValueError, 'monotonic'):
            replay.command(1.0)


if __name__ == '__main__':
    unittest.main()
