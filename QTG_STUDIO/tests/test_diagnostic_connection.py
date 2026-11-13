from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import Mock, patch

from backend.diagnostic_connection import DashboardCache, DiagnosticSession, run_standalone


class DiagnosticConnectionTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def dashboard(self):
        client = Mock()
        client.running = True
        client.snapshot.return_value = SimpleNamespace(connected=1, telemetry_valid=1,
            telemetry_age_ms=0, pitch_deg=0., bank_deg=0., ias_kt=0., altitude_ft=755.,
            throttle_percent=100., weight_kg=850., temperature_c=15., engine_rpm=983., prop_rpm=984.)
        return client

    def test_shared_dashboard_is_borrowed_never_started_or_stopped(self):
        client = self.dashboard()
        spawn = Mock()
        session = DiagnosticSession(self.root, dashboard=client, popen=spawn)
        session.connect()
        self.assertIn('borrowed dashboard', session.kind)
        session.command('close')
        client.start.assert_not_called()
        client.stop.assert_not_called()
        spawn.assert_not_called()

    def test_dashboard_preserves_independent_rpm_and_omits_unknowns(self):
        view = DashboardCache(self.dashboard())
        self.assertTrue(view.is_connected())
        self.assertEqual(view.get_telemetry()['eng_rpms'], [983.])
        self.assertEqual(view.get_telemetry()['prop_rpms'], [984.])
        self.assertNotIn('engine_manifold_pressure_psi_1', view.get_telemetry())

    def test_active_worker_preferred_over_dashboard_and_standalone(self):
        worker = Mock()
        worker.poll.return_value = None
        spawn = Mock()
        client = self.dashboard()
        session = DiagnosticSession(self.root, dashboard=client, worker=worker,
            worker_context={'status_file': str(self.root/'worker.json')}, popen=spawn)
        session.connect()
        with patch('qtg_common_engine.send_command') as send:
            session.command('start')
            session.command('close')
            self.assertEqual([call.args[1] for call in send.call_args_list], ['diagnostic_start', 'diagnostic_stop'])
        spawn.assert_not_called()
        client.snapshot.assert_not_called()
        worker.terminate.assert_not_called()

    def test_standalone_uses_separate_process_and_repeated_connect_is_idempotent(self):
        spawn = Mock()
        session = DiagnosticSession(self.root, popen=spawn)
        session.connect()
        session.connect()
        spawn.assert_called_once()
        self.assertIn('--diagnostic-connection', spawn.call_args.args[0])
        session.command('start')
        session.command('stop')
        session.command('close')
        self.assertEqual(sorted(p.read_text() for p in self.root.glob('*.command')), ['close', 'start', 'stop'])

    def test_standalone_process_only_stops_its_own_client_and_no_placement(self):
        client = Mock()
        client.last_error = ''
        # Close immediately after connection launch, with no monitoring or control.
        (self.root/'1_close.command').write_text('close')
        shared = self.dashboard()
        run_standalone(self.root, factory=Mock(return_value=client))
        client.connect_and_run.assert_called_once_with(place_at_rahwali=False, place_airborne=False)
        client.stop.assert_called_once()
        shared.stop.assert_not_called()
        client.pause_sim.assert_not_called()
        client.unpause_sim.assert_not_called()

    def test_disconnected_dashboard_falls_back_without_restarting_shared_transport(self):
        client = self.dashboard()
        client.snapshot.return_value.connected = 0
        spawn = Mock()
        session = DiagnosticSession(self.root, dashboard=client, popen=spawn)
        session.connect()
        spawn.assert_called_once()
        client.start.assert_not_called()
        client.stop.assert_not_called()


if __name__ == '__main__': unittest.main()
