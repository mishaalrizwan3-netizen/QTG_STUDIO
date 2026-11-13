import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

from backend.telemetry_diagnostic import DiagnosticMonitor


class TelemetryDiagnosticTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.client = Mock(spec=['is_connected', 'get_telemetry_sequence', 'get_telemetry'])
        self.client.is_connected.return_value = True
        self.client.get_telemetry_sequence.return_value = 1
        import time
        self.client.get_telemetry.return_value = dict(bridge_telemetry_sequence=1,
            last_updated=time.time(), eng_rpms=[983.], prop_rpms=[2700.])
        self.monitor = DiagnosticMonitor(self.client, self.root, self.root/'status.json',
            display_cache=lambda:'Brunner Controls: Pitch: +0.25 | Roll: -0.10 | Rudder: +0.00')

    def test_values_independent_sources_and_unverified_timing(self):
        rows, _ = self.monitor.sample()
        values = {row['channel']: row for row in rows}
        self.assertEqual(values['engine_rpm']['value'], 983.)
        self.assertEqual(values['prop_rpm']['value'], 2700.)
        self.assertEqual(values['brunner.pitch']['value'], .25)
        self.assertEqual(values['brunner.pitch']['status'], 'DISPLAY_ONLY_UNVERIFIED_POTENTIALLY_STALE')
        self.assertTrue(all(row['native_acquired_at'] is None and row['freshness'] == 'UNVERIFIED' for row in rows))

    def test_frozen_cache_and_repeated_samples_are_not_relabelled_fresh(self):
        self.monitor.sample()
        rows, issues = self.monitor.sample()
        self.assertEqual(self.monitor.source_state, 'UNCHANGED_FRESHNESS_UNVERIFIED')
        engine=next(row for row in rows if row['channel']=='engine_rpm')
        self.assertEqual(engine['value'],983.)
        self.assertEqual(engine['freshness'],'UNVERIFIED')

    def test_connection_loss(self):
        self.client.is_connected.return_value = False
        rows, _ = self.monitor.sample()
        self.assertTrue(all(row['value'] is None for row in rows if not row['channel'].startswith('brunner.')))

    def test_malformed_data_is_reported_without_crashing(self):
        self.client.get_telemetry.return_value = None
        rows, issues = self.monitor.sample()
        self.assertTrue(issues)
        self.assertTrue(rows)

    def test_missing_channels_remain_unavailable(self):
        rows, _ = self.monitor.sample()
        values = {row['channel']: row for row in rows}
        for key in ('flaps', 'sim_paused', 'pressure_altitude', 'on_ground'):
            self.assertIsNone(values[key]['value'])
            self.assertEqual(values[key]['status'], 'MISSING_OR_UNVALIDATED')

    def test_cancellation_saves_csv_and_summary_without_hardware_cleanup(self):
        self.monitor.cancelled.set()
        self.monitor.start()
        self.monitor.thread.join(2)
        self.assertFalse(self.monitor.thread.is_alive())
        self.assertTrue(Path(self.monitor.outputs['csv']).exists())
        report = json.loads(Path(self.monitor.outputs['summary']).read_text())
        self.assertFalse(report['active'])
        self.assertEqual(report['samples'], 0)

    def test_save_one_sample_and_stop(self):
        original = self.monitor.sample
        def sample():
            rows = original()
            self.monitor.stop()
            return rows
        self.monitor.sample = sample
        self.monitor.start()
        self.monitor.thread.join(2)
        self.assertFalse(self.monitor.thread.is_alive())
        self.assertEqual(json.loads((self.root/'status.json').read_text())['samples'], 1)
        self.assertIn('UNVERIFIED', Path(self.monitor.outputs['csv']).read_text())
        with self.assertRaises(RuntimeError): self.monitor.start()

    def test_worker_diagnostic_commands_do_not_dispatch_recording_or_control(self):
        import qtg_common_engine as engine
        base = Mock()
        base.SuperMushakLoggerApp = object
        context = {'status_file': str(self.root/'worker.json')}
        worker = object.__new__(engine.worker_class(base, context, Mock()))
        worker.p3d = self.client
        worker.arm_recording = Mock()
        worker.stop_test = Mock()
        with patch('backend.telemetry_diagnostic.DiagnosticMonitor') as factory:
            monitor = factory.return_value
            monitor.thread.is_alive.return_value = True
            worker.handle_command({'action': 'diagnostic_start'})
            worker.handle_command({'action': 'diagnostic_start'})
            worker.handle_command({'action': 'diagnostic_stop'})
            factory.assert_called_once()
            self.assertIs(factory.call_args.args[0], self.client)
            monitor.start.assert_called_once()
            monitor.stop.assert_called_once()
        worker.arm_recording.assert_not_called()
        worker.stop_test.assert_not_called()


if __name__ == '__main__':
    unittest.main()
