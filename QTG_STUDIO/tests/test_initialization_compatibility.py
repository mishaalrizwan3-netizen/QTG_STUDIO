"""Offline compatibility checks; never load or connect hardware drivers.

Safety assertions intentionally fail when existing production behavior violates
the contract. No expected-failure markers hide those findings.
"""
import csv
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import qtg_common_engine as engine
from backend import initializer
from backend.registry import engine_test_policy
from backend.verifier import validate_reference

spec = importlib.util.spec_from_file_location(
    'compat_engine_adapter', ROOT / 'test_suites/_shared/engine_initialization.py')
adapter = importlib.util.module_from_spec(spec)
spec.loader.exec_module(adapter)


class InitializationCompatibilityTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / 'manual.csv'
        self.cfg = dict(alt_ft=755., ias_kt=0., throttle=100., flaps=0.,
                        lat=32.23366667, lon=74.1235, heading_deg=48.4)
        self.policy = engine_test_policy('1.f.(2)')
        self.base = SimpleNamespace(startup_rpm_tolerance=lambda ref: 10.,
                                    validate_initial_airspeed=Mock())
        self.host = SimpleNamespace(
            p3d=Mock(), test_mode='auto', stop_worker_event=threading.Event(),
            _session_context={'test_reference': '1.f.(2)'},
            _base_module=self.base, ref_manual_snapshot={'airspeed': 0},
            selected_ref_csv=self.path, _commit_initial_position=Mock(),
            _refresh_restored_airdata=Mock(), _refresh_paused_telemetry=Mock())
        self.host.p3d._qtg_ground_position_wrapped = False
        # install() patches named modules; isolate these from the interpreter.
        self.modules = patch.dict(sys.modules, {
            'replay_support': SimpleNamespace(),
            'qtg_brunner_base': SimpleNamespace()})
        self.modules.start()
        self.addCleanup(self.modules.stop)

    def reference(self, sidecar=True, **changes):
        row = {'Time (s)': 0, 'Pitch_Manual_Pos': 0, 'Roll_Manual_Pos': 0,
               'Rudder_Manual_Pos': 0, 'Throttle 1 Position (%)': 100,
               'Altitude MSL (ft)': 755, 'Indicated Airspeed (kts)': 0}
        row.update(changes)
        with self.path.open('w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(row))
            writer.writeheader()
            writer.writerow(row)
            writer.writerow(dict(row, **{'Time (s)': 1}))
        if sidecar:
            self.sidecar().write_text(json.dumps({'manual': {
                'lat': self.cfg['lat'], 'lon': self.cfg['lon'],
                'heading': self.cfg['heading_deg']}}), encoding='utf-8')

    def sidecar(self):
        return self.path.with_name('manual_init_conditions.json')

    def run_replay(self, entry):
        with patch('backend.deceleration_config.resolve_initial_conditions',
                   return_value=dict(self.cfg)):
            adapter.run_initialization(self.host, 'auto', entry, self.policy)

    def test_ground_zero_ias_routes_to_ground_queue(self):
        original = self.host.p3d.set_position_airborne
        adapter.install(self.host)
        self.host.p3d.set_position_airborne(alt_ft=755, speed_kts=0,
                                           lat=32.23366667, lon=74.1235, hdg_deg=48.4)
        self.host.p3d.queue_position_airborne.assert_called_once_with(
            alt_ft=755, speed_kts=0., pitch_deg=0., bank_deg=0.,
            hdg_deg=48.4, lat=32.23366667, lon=74.1235, on_ground=True)
        original.assert_not_called()
        self.host.p3d.pause_sim.assert_called_once()

    def test_ground_position_rejection_propagates(self):
        adapter.install(self.host)
        self.host.p3d.queue_position_airborne.return_value = False
        with self.assertRaisesRegex(RuntimeError, 'ground position rejected'):
            self.host.p3d.set_position_airborne(alt_ft=755, speed_kts=0)

    def test_ground_refresh_never_runs_live_airdata_recovery(self):
        original = self.host._refresh_restored_airdata
        adapter.install(self.host)
        self.host._refresh_restored_airdata()
        original.assert_not_called()
        self.host._refresh_paused_telemetry.assert_called_once()
        self.host.p3d.unpause_sim.assert_not_called()

    def test_deceleration_startup_tolerance_is_60_rpm(self):
        adapter.install(self.host)
        self.assertEqual(self.base.startup_rpm_tolerance('1.f.(2)'), 60.)
        self.assertEqual(self.base.startup_rpm_tolerance('other'), 10.)
        self.host.ref_manual_snapshot = {'rpm': 2700., 'telemetry': {'prop_rpms': [2700.]}}
        self.host._refresh_paused_telemetry.return_value = {'engine_rpm': 2760., 'prop_rpms': [2640.]}
        adapter.engine_restore_rpm(self.host)
        self.assertEqual(self.host._session_context['rpm_startup_readiness']['tolerance_rpm'], 60.)
        self.host.p3d.unpause_sim.assert_not_called()

    def test_missing_sidecar_rejected_before_entry(self):
        self.reference(sidecar=False)
        entry = Mock()
        with self.assertRaisesRegex(ValueError, 'matching _init_conditions.json'):
            self.run_replay(entry)
        entry.assert_not_called()

    def test_mismatched_or_incomplete_sidecar_rejected(self):
        self.reference()
        for pose in ({'lat': 0, 'lon': 0, 'heading': 0}, {'lat': self.cfg['lat']}):
            with self.subTest(pose=pose):
                self.sidecar().write_text(json.dumps({'manual': pose}), encoding='utf-8')
                entry = Mock()
                with self.assertRaises(ValueError):
                    self.run_replay(entry)
                entry.assert_not_called()

    def test_csv_mismatch_and_invalid_samples_rejected(self):
        for changes in ({'Altitude MSL (ft)': 3000},
                        {'Indicated Airspeed (kts)': 'nan'},
                        {'Throttle 1 Position (%)': 0}):
            with self.subTest(changes=changes):
                self.reference(**changes)
                entry = Mock()
                with self.assertRaises(ValueError):
                    self.run_replay(entry)
                entry.assert_not_called()
        self.reference(**{'Throttle 1 Position (%)': 'nan'})
        with self.assertRaises(ValueError):
            validate_reference(self.path)

    def test_matching_reference_selects_discrete_policy_only_once(self):
        class Replay:
            pass
        self.base.ThrottleReplay = Replay
        self.base.interpolate_reference = Mock(return_value=50.)
        self.reference()
        self.assertEqual(validate_reference(self.path), str(self.path.resolve()))
        entry = Mock()
        self.run_replay(entry)
        selected = self.base.ThrottleReplay
        self.assertTrue(selected._qtg_deceleration_step_replay)
        self.run_replay(entry)
        self.assertIs(self.base.ThrottleReplay, selected)
        replay = selected()
        replay.times = [0., 1.]
        replay.values = [100., 0.]
        replay.last_elapsed = replay.last_sent = replay.last_command = None
        self.assertEqual(replay.command(.25), 100.)
        self.assertEqual(replay.command(.75), 0.)
        self.assertEqual(entry.call_count, 2)

    def test_dispatch_uses_deceleration_adapter_and_base_replay_entry(self):
        def start_auto_test_only(window):
            raise AssertionError('adapter must control invocation')
        self.base.SuperMushakLoggerApp = SimpleNamespace(start_auto_test_only=start_auto_test_only)
        self.base.__file__ = str(ROOT / 'interfaces/brunner_base/Super_Mushak_Logger_GUI.py')
        self.host.brunner = Mock()
        self.host._engine_test_adapter = Mock()
        with patch.object(engine, 'configure_test_extension'), \
                patch.object(engine, 'validate_reference', return_value=str(self.path)), \
                patch('backend.deceleration_config.resolve_initial_conditions', return_value=self.cfg):
            engine.initialize_mode(self.host, 'auto', str(self.path))
        args = self.host._engine_test_adapter.run_initialization.call_args.args
        self.assertEqual(args[:3], (self.host, 'auto', start_auto_test_only))
        self.assertEqual(args[3].TEST_REFERENCE, '1.f.(2)')
        self.assertFalse(self.host._prepared)

    def worker(self):
        class Base:
            def _update_stage_ui(self, stage, details=''):
                self.current_stage = stage
        base = SimpleNamespace(SuperMushakLoggerApp=Base,
            STAGE_CONNECTED_READY='Ready', STAGE_AUTO_BLOCKED='Blocked',
            STAGE_CONNECTION_ERROR='Failed', STAGE_STOPPED_BY_USER='Stopped')
        worker = object.__new__(engine.worker_class(base, {}, lambda state: None))
        worker._control_method = 'replay'
        worker._prepared = False
        worker._preparing = True
        worker.current_stage = 'Initializing'
        worker.is_recording = False
        worker._publish = Mock()
        worker.arm_recording = Mock()
        worker.p3d = Mock()
        worker.brunner = Mock()
        worker.stop_worker_event = threading.Event()
        return worker

    def test_recording_requires_successful_ready_stage(self):
        worker = self.worker()
        worker.handle_command({'action': 'record'})
        worker.arm_recording.assert_not_called()
        worker._update_stage_ui('Failed', 'initialization failed')
        worker.handle_command({'action': 'record'})
        worker.arm_recording.assert_not_called()
        worker._preparing = True
        worker._update_stage_ui('Ready')
        worker.handle_command({'action': 'record'})
        worker.arm_recording.assert_called_once()

    def test_cancelled_engine_preparation_rejected_before_commands(self):
        self.host.stop_worker_event.set()
        with self.assertRaisesRegex(RuntimeError, 'cancelled'):
            adapter.engine_restore_rpm(self.host)
        self.host.p3d.unpause_sim.assert_not_called()
        self.host.p3d.set_replay_throttle.assert_not_called()

    def test_unavailable_engine_feedback_rejected(self):
        self.host.ref_manual_snapshot = {'rpm': 2700., 'telemetry': {'prop_rpms': [2700.]}}
        self.host._refresh_paused_telemetry.return_value = {}
        with self.assertRaisesRegex(RuntimeError, 'RPM is unavailable'):
            adapter.engine_restore_rpm(self.host)
        self.host.p3d.unpause_sim.assert_not_called()

    def test_late_ready_after_cancellation_must_not_enable_recording(self):
        worker = self.worker()
        worker.stop_worker_event.set()
        worker._update_stage_ui('Ready')
        worker.handle_command({'action': 'record'})
        worker.arm_recording.assert_not_called()

    def test_late_ready_after_failure_does_not_change_failed_stage(self):
        worker = self.worker()
        worker._update_stage_ui('Failed', 'preparation failed')
        worker._update_stage_ui('Ready')
        self.assertFalse(worker._prepared)
        self.assertEqual(worker.current_stage, 'Failed')
        worker.handle_command({'action': 'record'})
        worker.arm_recording.assert_not_called()

    def test_cancellation_after_ready_still_blocks_recording(self):
        worker = self.worker()
        worker._update_stage_ui('Ready')
        worker.stop_worker_event.set()
        worker.handle_command({'action': 'record'})
        worker.arm_recording.assert_not_called()

    def test_superseded_queued_callback_is_ignored_but_current_ready_works(self):
        worker = self.worker()
        worker._initialization_generation = 2
        worker._initialization_stage(1, 'Ready')
        self.assertFalse(worker._prepared)
        self.assertEqual(worker.current_stage, 'Initializing')
        worker._initialization_stage(1, 'Failed', 'old failure')
        self.assertTrue(worker._preparing)
        worker._initialization_stage(2, 'Ready')
        self.assertTrue(worker._prepared)
        worker.handle_command({'action': 'record'})
        worker.arm_recording.assert_called_once()

    def test_begin_initialization_keeps_generation_on_queued_receiver(self):
        worker = self.worker()
        worker.sig_update_stage = Mock()
        qt = SimpleNamespace(ConnectionType=SimpleNamespace(QueuedConnection=object()))
        with patch.dict(sys.modules, {'PySide6.QtCore': SimpleNamespace(Qt=qt)}):
            worker._begin_initialization()
            first = worker._initialization_stage_receiver
            worker._begin_initialization()
        worker.sig_update_stage.disconnect.assert_called_with(first)
        self.assertIs(worker.sig_update_stage.connect.call_args.args[1],
                      qt.ConnectionType.QueuedConnection)
        worker._preparing = True
        first('Ready')
        self.assertFalse(worker._prepared)
        worker._initialization_stage_receiver('Ready')
        self.assertTrue(worker._prepared)

    def test_new_attempt_rejected_while_previous_thread_can_emit(self):
        worker = self.worker()
        worker.preparation_thread = Mock()
        worker.preparation_thread.is_alive.return_value = True
        with self.assertRaisesRegex(RuntimeError, 'previous initialization'):
            worker._begin_initialization()

    def test_synchronous_initialization_failure_invalidates_readiness(self):
        worker = self.worker()
        worker._begin_initialization = Mock()
        with patch.object(engine, '_initialize_mode', side_effect=ValueError('invalid settings')):
            with self.assertRaisesRegex(ValueError, 'invalid settings'):
                engine.initialize_mode(worker, 'auto')
        worker._update_stage_ui('Ready')
        worker.handle_command({'action': 'record'})
        worker.arm_recording.assert_not_called()

    def test_stop_invalidates_queued_ready_even_after_event_is_cleared(self):
        worker = self.worker()
        worker._initialization_generation = 1
        worker.stop_test = Mock(side_effect=worker.stop_worker_event.set)
        worker.handle_command({'action': 'stop'})
        worker.stop_worker_event.clear()
        worker._initialization_stage(1, 'Ready')
        worker.handle_command({'action': 'record'})
        worker.arm_recording.assert_not_called()

    def test_repeated_configuration_restores_window_methods(self):
        original = self.host._commit_initial_position
        initializer.configure_test_extension(self.host, '1.f.(2)')
        wrapped_position = self.host.p3d.set_position_airborne
        initializer.configure_test_extension(self.host, '1.f.(2)')
        self.assertIs(self.host._test_extension_originals['_commit_initial_position'], original)
        self.assertIs(self.host.p3d.set_position_airborne, wrapped_position)
        snapshot = {'telemetry': {}}
        self.host._commit_initial_position(reference_snapshot=snapshot)
        original.assert_called_once_with(reference_snapshot=snapshot)

    def test_repeated_configuration_must_not_accumulate_global_wrappers(self):
        initializer.configure_test_extension(self.host, '1.f.(2)')
        first = self.base.startup_rpm_tolerance
        initializer.configure_test_extension(self.host, '1.f.(2)')
        second = self.base.startup_rpm_tolerance
        # A newly allocated wrapper is acceptable; wrapping the prior wrapper is not.
        captured = [cell.cell_contents for cell in (second.__closure__ or ())]
        self.assertNotIn(first, captured, 'startup RPM wrappers accumulate on repeated initialization')

    def test_rpm_policy_survives_test_switches_and_adapter_reload(self):
        originals = [Mock(return_value=value) for value in (17., 23., 31.)]
        modules = [SimpleNamespace(startup_rpm_tolerance=original)
                   for original in originals]
        self.host._base_module = modules[2]
        with patch.dict(sys.modules, dict(replay_support=modules[0], qtg_brunner_base=modules[1])):
            initializer.configure_test_extension(self.host, '1.f.(2)')
            installed = [mod.startup_rpm_tolerance for mod in modules]
            for ref in ('1.f.(1)', '1.c.(1)', '1.f.(2)', '1.f.(1)'):
                initializer.configure_test_extension(self.host, ref)
                for mod, original, wrapped in zip(modules, originals, installed):
                    self.assertIs(mod.startup_rpm_tolerance, wrapped)
                    self.assertEqual(wrapped('1.f.(1)'), 400.)
                    self.assertEqual(wrapped('1.f.(2)'), 60.)
                    original.reset_mock()
                    self.assertEqual(wrapped('1.c.(1)'), original.return_value)
                    original.assert_called_once_with('1.c.(1)')

    def test_rpm_policy_installed_once_when_base_module_is_global_alias(self):
        original = Mock(return_value=19.)
        self.base.startup_rpm_tolerance = original
        with patch.dict(sys.modules, {'qtg_brunner_base': self.base}):
            initializer.configure_test_extension(self.host, '1.f.(2)')
            first = self.base.startup_rpm_tolerance
            initializer.configure_test_extension(self.host, '1.f.(1)')
            self.assertIs(self.base.startup_rpm_tolerance, first)
            self.assertEqual(first('other'), 19.)
            original.assert_called_once_with('other')


if __name__ == '__main__':
    unittest.main()
