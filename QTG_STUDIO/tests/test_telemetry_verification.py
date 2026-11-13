import unittest
from dataclasses import replace
from unittest.mock import Mock

from backend.telemetry_verification import CachedSource, ReadOnlyTelemetry, StabilityWindow, TelemetryUnavailable
from backend.universal_initialization import Feedback, Measurement


class TelemetryVerificationTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.
        self.wall = 1000.
        self.client = Mock(spec=['is_connected', 'get_telemetry_sequence', 'get_telemetry'])
        self.client.is_connected.return_value = True
        self.client.get_telemetry_sequence.return_value = 1
        self.raw = dict(bridge_telemetry_sequence=1, last_updated=1000., eng_rpms=[0.],
                        prop_rpms=[983.], engine_rpm=983., pressure_alt=755.,
                        engine_throttle_percent=0., ground_speed=0.)
        self.client.get_telemetry.return_value = self.raw
        self.reader = ReadOnlyTelemetry(self.client, clock=lambda: self.now, wall_clock=lambda: self.wall)

    def test_native_rpm_zero_and_msl_preserved_without_fallback(self):
        frame = self.reader.feedback('attempt')
        self.assertEqual(frame.values['engine_rpm'].value, 0.)
        self.assertEqual(frame.values['prop_rpm'].value, 983.)
        self.assertEqual(frame.values['alt_ft'].unit, 'ft MSL')
        self.assertEqual(frame.values['throttle'].value, 0.)
        self.assertTrue(any('conflicts' in issue for issue in self.reader.issues))
        self.assertEqual(frame.measured_at, 100.)

    def test_missing_and_nonfinite_native_values_are_omitted(self):
        self.raw.update(eng_rpms=[float('nan')], pressure_alt=float('inf'))
        frame = self.reader.feedback()
        self.assertNotIn('engine_rpm', frame.values)
        self.assertNotIn('alt_ft', frame.values)
        self.assertNotIn('on_ground', frame.values)

    def test_disconnection_never_returns_default_measurements(self):
        self.client.is_connected.return_value = False
        frame = self.reader.feedback()
        self.assertFalse(frame.connected)
        self.assertEqual(frame.values, {})
        self.client.get_telemetry.assert_not_called()

    def test_invalid_timestamps_and_clock_domains(self):
        for stamp in (None, float('nan'), 990., 1001., 100.):
            with self.subTest(stamp=stamp):
                self.raw['last_updated'] = stamp
                with self.assertRaises(TelemetryUnavailable):
                    self.reader.feedback()

    def test_host_clock_jump_rejected(self):
        self.wall += 2.
        with self.assertRaisesRegex(TelemetryUnavailable, 'mapping changed'):
            self.reader.feedback()

    def test_repeated_reads_and_new_sequence_with_repeated_timestamp(self):
        self.reader.feedback()
        with self.assertRaisesRegex(TelemetryUnavailable, 'repeated'):
            self.reader.feedback()
        self.raw['bridge_telemetry_sequence'] = 2
        self.client.get_telemetry_sequence.return_value = 2
        with self.assertRaisesRegex(TelemetryUnavailable, 'repeated'):
            self.reader.feedback()
        self.now += .1
        self.wall += .1
        self.raw['last_updated'] = self.wall
        self.assertEqual(self.reader.feedback().sequence, 2)

    def test_bridge_cache_race_rejected(self):
        self.client.get_telemetry_sequence.side_effect = [1, 2]
        with self.assertRaisesRegex(TelemetryUnavailable, 'coherent'):
            self.reader.feedback()

    def test_brunner_is_separate_and_never_overwrites_simulator_state(self):
        cached = Mock(return_value=CachedSource(True, 1, 100., 'host_monotonic',
            {'pitch_position': Measurement(.25, 'normalized', 'CLS2Sim 0x11')}))
        self.reader.brunner_cache = cached
        frame = self.reader.feedback()
        self.assertNotIn('brunner.pitch_position', frame.values)
        self.assertEqual(self.reader.brunner.values['brunner.pitch_position'].value, .25)
        cached.assert_called_once_with()

    def test_stale_or_disconnected_brunner_does_not_poison_p3d(self):
        for source in (CachedSource(False, 1, 100., 'host_monotonic', {}),
                       CachedSource(True, 1, 99., 'sim_seconds', {})):
            self.setUp()
            self.reader.brunner_cache = Mock(return_value=source)
            self.assertTrue(self.reader.feedback().connected)
            self.assertIsNone(self.reader.brunner)
            self.assertTrue(any('BRUNNER' in issue or 'brunner' in issue for issue in self.reader.issues))

    def test_stability_needs_duration_distinct_frames_and_consistent_units(self):
        gate = StabilityWindow({'ias_kt': 1.}, duration_s=1., min_samples=3, max_gap_s=.6)
        def frame(seq, stamp, value=0., unit='kt'):
            return Feedback(True, seq, stamp, {'ias_kt': Measurement(value, unit, 'airspeed_indicated')})
        self.assertFalse(gate.observe(frame(1, 0)))
        self.assertFalse(gate.observe(frame(2, .5)))
        self.assertTrue(gate.observe(frame(3, 1.)))
        self.assertFalse(gate.observe(frame(3, 1.)))
        self.assertFalse(gate.observe(frame(4, 1.5)))
        self.assertFalse(gate.observe(frame(5, 2., unit='m/s')))

    def test_gap_missing_and_drifting_values_reset_stability(self):
        gate = StabilityWindow({'ias_kt': 1.}, duration_s=1., min_samples=2, max_gap_s=.6)
        frame = Feedback(True, 1, 0., {'ias_kt': Measurement(0., 'kt', 'ias')})
        gate.observe(frame)
        self.assertFalse(gate.observe(replace(frame, sequence=2, measured_at=2.)))
        self.assertFalse(gate.observe(replace(frame, sequence=3, measured_at=2.5, values={})))
        gate.observe(replace(frame, sequence=4, measured_at=3.))
        self.assertFalse(gate.observe(replace(frame, sequence=5, measured_at=3.5,
            values={'ias_kt': Measurement(5., 'kt', 'ias')})))


if __name__ == '__main__':
    unittest.main()
