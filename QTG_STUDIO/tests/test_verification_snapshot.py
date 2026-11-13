"""Offline ABI/reader regressions. No DLL loading or simulator connection."""
import ctypes
import re
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from interfaces.brunner_base.src.verification_snapshot import CHANNELS, VerificationSnapshot, acquire
from backend.telemetry_verification import ReadOnlyTelemetry, TelemetryUnavailable


class Function:
    def __init__(self, call): self.call = call
    def __call__(self, *args): return self.call(*args)


class SnapshotTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.
        self.raw = VerificationSnapshot()
        self.raw.size = ctypes.sizeof(self.raw)
        self.raw.version = self.raw.connected = self.raw.generation = 1
        self.raw.request_id = 1001
        self.raw.sequence = 7
        self.raw.receipt_ticks = 5000
        self.raw.copied_ticks = 5010
        self.raw.frequency = 1000
        self.raw.running = 0
        self.raw.valid_mask = (1 << len(CHANNELS))-1
        self.raw.values[0], self.raw.values[1] = 1., 0.
        self.raw.values[18] = 50.
        self.responses = []
        def copy(pointer, size):
            action = self.responses.pop(0) if self.responses else None
            if action: action()
            ctypes.memmove(pointer, ctypes.byref(self.raw), size)
            return 1
        self.client = SimpleNamespace(is_connected=lambda: True, _bridge=SimpleNamespace(
            RequestVerificationV1=Function(lambda: 1001), GetVerificationV1=Function(copy)))

    def read(self):
        def sleep(_): self.now += .01
        return acquire(self.client, .05, clock=lambda: self.now, sleep=sleep)

    def reader(self):
        self.client.verification_snapshot = self.read
        return ReadOnlyTelemetry(self.client, clock=lambda: self.now, wall_clock=lambda: 10000.)

    def test_packed_abi_size_and_separate_ground_flap_fields(self):
        self.assertEqual(ctypes.sizeof(self.raw), 72+len(CHANNELS)*8)
        frame = self.reader().feedback('a', _authoritative=True)
        self.assertEqual(frame.values['on_ground'].value, 1)
        self.assertEqual(frame.values['flaps'].value, 0)
        self.assertEqual(frame.values['flaps'].unit, 'handle index')

    def test_native_registration_order_and_units_match_python_abi(self):
        source = (Path(__file__).parents[1]/'interfaces/brunner_base/native/src/verification_snapshot_v1.h').read_text()
        registrations = re.findall(r'TrackedAddToDataDefinition\(hSimConnect, VERIFICATION_DEFINITION, "([^"]+)", "([^"]+)"', source)
        self.assertEqual(registrations, [(simvar, unit) for _, simvar, unit in CHANNELS])

    def test_native_age_is_duration_not_epoch_conversion(self):
        self.assertAlmostEqual(self.read()['measured_at'], 99.99)

    def test_old_request_is_not_accepted(self):
        self.raw.request_id = 999
        with self.assertRaisesRegex(RuntimeError, 'correlated'): self.read()

    def test_old_callback_then_correct_response(self):
        self.raw.request_id = 999
        self.responses = [lambda: None, lambda: setattr(self.raw, 'request_id', 1001)]
        self.assertEqual(self.read()['request_id'], 1001)

    def test_reconnect_while_waiting_is_rejected(self):
        self.raw.request_id = 999
        self.responses = [lambda: None, lambda: setattr(self.raw, 'generation', 2)]
        with self.assertRaisesRegex(RuntimeError, 'generation changed'): self.read()

    def test_disconnected_response(self):
        self.raw.connected = 0
        with self.assertRaisesRegex(RuntimeError, 'connection'): self.read()

    def test_unsupported_old_dll(self):
        self.client._bridge = SimpleNamespace()
        with self.assertRaisesRegex(RuntimeError, 'unsupported'): self.read()

    def test_bad_abi_or_clock_is_rejected(self):
        for name, value in [('version', 2), ('size', 0), ('frequency', 0), ('receipt_ticks', 9999)]:
            old = getattr(self.raw, name)
            setattr(self.raw, name, value)
            with self.subTest(name=name), self.assertRaises(RuntimeError): self.read()
            setattr(self.raw, name, old)

    def test_incomplete_nonfinite_measurements_are_not_zero_filled(self):
        self.raw.valid_mask &= ~(1 << 1)
        self.raw.values[3] = float('nan')
        frame = self.reader().feedback('a', _authoritative=True)
        self.assertNotIn('flaps', frame.values)
        self.assertNotIn('ias_kt', frame.values)
        self.assertIn('on_ground', frame.values)

    def test_invalid_ground_and_flap_domains(self):
        self.raw.values[0], self.raw.values[1] = 2, .5
        frame = self.reader().feedback('a', _authoritative=True)
        self.assertNotIn('on_ground', frame.values)
        self.assertNotIn('flaps', frame.values)

    def test_cached_native_response_is_not_another_sample(self):
        reader = self.reader()
        reader.feedback('a', _authoritative=True)
        with self.assertRaisesRegex(TelemetryUnavailable, 'repeated'): reader.feedback('a', _authoritative=True)

    def test_fresh_paused_identical_values_are_accepted(self):
        reader = self.reader()
        first = reader.feedback('a', _authoritative=True)
        self.now += .1
        self.raw.sequence += 1
        self.raw.request_id += 1
        self.client._bridge.RequestVerificationV1.call = lambda: self.raw.request_id
        second = reader.feedback('a', _authoritative=True)
        self.assertEqual(first.values, second.values)
        self.assertFalse(reader.native_timing['running'])
        self.assertFalse(reader.native_timing['simulator_time_advanced'])

    def test_running_sim_time_progress_is_separate_evidence(self):
        reader = self.reader()
        reader.feedback('a', _authoritative=True)
        self.now += .1
        self.raw.sequence += 1
        self.raw.request_id += 1
        self.raw.running = 1
        self.raw.values[18] += .1
        self.client._bridge.RequestVerificationV1.call = lambda: self.raw.request_id
        reader.feedback('a', _authoritative=True)
        self.assertTrue(reader.native_timing['simulator_time_advanced'])

    def test_unknown_pause_state_is_explicit(self):
        self.raw.running = -1
        reader = self.reader()
        reader.feedback('a', _authoritative=True)
        self.assertIsNone(reader.native_timing['running'])

    def test_stale_native_receipt(self):
        self.raw.copied_ticks = 8000
        with self.assertRaisesRegex(TelemetryUnavailable, 'stale'): self.reader().feedback('a', _authoritative=True)

    def test_simulator_clock_reset_requires_new_attempt(self):
        reader = self.reader()
        reader.feedback('a', _authoritative=True)
        self.now += .1
        self.raw.sequence += 1
        self.raw.request_id += 1
        self.raw.values[18] -= 1
        self.client._bridge.RequestVerificationV1.call = lambda: self.raw.request_id
        with self.assertRaisesRegex(TelemetryUnavailable, 'backwards'): reader.feedback('a', _authoritative=True)

    def test_reconnect_between_frames_requires_new_attempt(self):
        reader = self.reader()
        reader.feedback('a', _authoritative=True)
        self.raw.generation += 1
        with self.assertRaisesRegex(TelemetryUnavailable, 'Reconnected'): reader.feedback('a', _authoritative=True)

    def test_no_legacy_cache_or_aircraft_commands(self):
        self.client.get_telemetry = Mock(side_effect=AssertionError('legacy cache'))
        self.client.set_pause = Mock(side_effect=AssertionError('aircraft write'))
        self.reader().feedback('a', _authoritative=True)
        self.client.get_telemetry.assert_not_called()
        self.client.set_pause.assert_not_called()
        text = (Path(__file__).parents[1]/'interfaces/brunner_base/native/src/verification_snapshot_v1.h').read_text()
        for forbidden in ('SetDataOnSimObject', 'TransmitClientEvent', 'SimConnect_Open', 'WeatherSet'):
            self.assertNotIn(forbidden, text)


if __name__ == '__main__': unittest.main()
