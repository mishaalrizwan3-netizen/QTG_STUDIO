"""Offline preparation and recording-boundary regressions; no hardware clients."""
import ast
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest
from unittest.mock import Mock, patch

from backend import airborne_preparation as preparation
from backend.initializer import apply_manual_conditions

ROOT = Path(__file__).resolve().parents[1]


class AirbornePreparationTests(unittest.TestCase):
    def setUp(self):
        spec = importlib.util.spec_from_file_location('replay_support', ROOT / 'interfaces/brunner_base/src/replay_support.py')
        replay = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(replay)
        self.modules = patch.dict(sys.modules, replay_support=replay)
        self.modules.start()
        self.addCleanup(self.modules.stop)
        self.clock = 0.
        self.sequence = 0
        self.cfg = dict(alt_ft=2000., ias_kt=80., lat=32., lon=74., heading_deg=35.,
                        pitch_deg=0., bank_deg=0., throttle=5., flaps=0.,
                        temperature_c=15., on_ground=False)
        self.sample = dict(airspeed_indicated=80., pressure_alt=2000., indicated_altitude=2000.,
                           ambient_wind_spd=8., aircraft_wind_x_kt=8., aircraft_wind_y_kt=0.,
                           aircraft_wind_z_kt=0., baro_pressure=1013.25,
                           plane_latitude=32., plane_longitude=74., engine_throttle_percent=5.)
        sim = Mock()
        self.running = False
        sim.simulation_running.side_effect = lambda: self.running
        sim.unpause_sim.side_effect = lambda: setattr(self, 'running', True)
        sim.pause_sim.side_effect = lambda: setattr(self, 'running', False)
        sim.get_telemetry_sequence.side_effect = lambda: self.sequence
        sim.wait_for_fresh_telemetry.side_effect = self.fresh
        sim.get_global_weather_metar.return_value = 'GLOB 00000KT 9999 15/12 Q1013'
        event = Mock()
        event.is_set.return_value = False
        event.wait.side_effect = self.advance
        self.window = SimpleNamespace(p3d=sim, stop_worker_event=event, test_mode='manual',
            _base_module=SimpleNamespace(STAGE_INITIALIZING='Initializing aircraft'), sig_update_stage=Mock(),
            _session_context={'test_reference':'1.f.(1)', 'acceleration_manual_airflow_preparation':True},
            manual_initial_conditions=self.cfg, _refresh_paused_telemetry=Mock(side_effect=lambda: dict(self.sample)),
            _wait_for_initial_altitude=Mock())

    def advance(self, seconds):
        self.clock += seconds

    def fresh(self, sequence, timeout):
        self.sequence += 1
        return dict(self.sample)

    def prepare(self):
        return preparation.prepare(self.window, self.cfg, dict(self.sample), clock=lambda: self.clock)

    def test_ready_requires_elapsed_time_and_consecutive_advancing_frames(self):
        self.prepare()
        self.assertGreaterEqual(self.clock, 2.5)
        self.assertEqual(self.window._session_context['manual_airflow_preparation']['consecutive_samples'], 3)
        preparation.verify_recording_sample(self.window, self.sample)
        self.window.p3d.unpause_sim.assert_called_once()
        self.window.p3d.queue_position_airborne.assert_not_called()
        self.window.p3d.set_global_weather_metar.assert_not_called()
        messages = [call.args[1] for call in self.window.sig_update_stage.emit.call_args_list]
        self.assertTrue(any('Recovering airflow' in text and 'IAS 80.00 / target 80.00 kt' in text
                            and 'MSL altitude 2000.0 / target 2000.0 ft' in text
                            and 'local wind 8.00 / target 8.00 kt' in text for text in messages))
        self.assertTrue(all(call.args[0] == 'Initializing aircraft'
                            for call in self.window.sig_update_stage.emit.call_args_list))
        self.assertFalse(hasattr(self.window, 'rec_time'))
        self.assertFalse(hasattr(self.window, 'brunner'))

    def test_changing_airflow_resets_consecutive_count(self):
        def changing(sequence, timeout):
            sample = self.fresh(sequence, timeout)
            sample['aircraft_wind_x_kt'] = 0. if self.sequence in (1, 10) else 8.
            return sample
        self.window.p3d.wait_for_fresh_telemetry.side_effect = changing
        self.prepare()
        self.assertGreaterEqual(self.sequence, 13)

    def test_descent_and_ias_excursions_fail_without_flight_corrections(self):
        for channel, value, reason in [('pressure_alt', 1974., 'MSL altitude'),
                                       ('airspeed_indicated', 82.63, 'IAS')]:
            with self.subTest(channel=channel):
                original = self.sample[channel]
                self.sample[channel] = value
                with self.assertRaisesRegex(RuntimeError, reason): self.prepare()
                self.assertIsNone(self.window._manual_airflow_ready)
                self.sample[channel] = original
        self.window.p3d.queue_position_airborne.assert_not_called()
        self.window.p3d.pause_sim.assert_called()

    def test_stale_frame_and_frozen_sequence_are_rejected(self):
        for result in (None, dict(self.sample)):
            self.window.p3d.wait_for_fresh_telemetry.side_effect = None
            self.window.p3d.wait_for_fresh_telemetry.return_value = result
            with self.assertRaisesRegex(RuntimeError, 'advancing'): self.prepare()

    def test_cancellation_and_disconnection_block_recording(self):
        self.window.stop_worker_event.is_set.return_value = True
        with self.assertRaisesRegex(RuntimeError, 'cancelled'): self.prepare()
        self.window.p3d.unpause_sim.assert_not_called()
        self.window.stop_worker_event.is_set.return_value = False
        self.window.p3d.is_connected.return_value = False
        with self.assertRaisesRegex(RuntimeError, 'disconnected'): self.prepare()
        with self.assertRaisesRegex(RuntimeError, 'not ready'):
            preparation.verify_recording_sample(self.window, self.sample)
        self.assertFalse(self.window._prepared)
        self.window.p3d.pause_sim.assert_called()
        self.window.p3d.wait_for_paused.assert_called_with(3.)

    def test_cancellation_during_live_recovery_invalidates_readiness_and_pauses(self):
        def cancel_during_wait(seconds):
            self.advance(seconds)
            self.window.stop_worker_event.is_set.return_value = True
        self.window.stop_worker_event.wait.side_effect = cancel_during_wait
        with self.assertRaisesRegex(RuntimeError, 'cancelled'): self.prepare()
        self.assertIsNone(self.window._manual_airflow_ready)
        self.assertFalse(self.window._prepared)
        self.window.p3d.unpause_sim.assert_called_once()
        self.window.p3d.pause_sim.assert_called_once()
        self.window.p3d.wait_for_paused.assert_called_with(3.)

    def test_pause_reassertion_requires_acknowledged_unpause_and_new_samples(self):
        def pause_during_frame(sequence, timeout):
            sample = self.fresh(sequence, timeout)
            if self.sequence == 10:
                self.running = False
            return sample
        self.window.p3d.wait_for_fresh_telemetry.side_effect = pause_during_frame
        self.prepare()
        report = self.window._session_context['manual_airflow_preparation']
        self.assertEqual(report['status'], 'READY')
        self.assertEqual(report['unpause_requests'], 2)
        self.assertGreaterEqual(self.sequence, 13)
        self.assertEqual([x['state'] for x in report['pause_transitions']],
                         ['PAUSED', 'RUNNING', 'PAUSED', 'RUNNING'])
        self.assertFalse(self.running)  # Final safety pause, not a recovery retry.
        self.window.p3d.pause_sim.assert_called_once()
        self.assertEqual(self.window.p3d.unpause_sim.call_count, 2)
        self.assertTrue(report['final_pause_confirmed'])
        messages = [call.args[1] for call in self.window.sig_update_stage.emit.call_args_list]
        self.assertTrue(any('P3D paused before airflow recovery' in text for text in messages))
        self.assertTrue(any('Unexpected P3D pause detected' in text for text in messages))

    def test_unknown_pause_state_never_unpauses_or_authorizes(self):
        for state in (None, RuntimeError('Pause event not received')):
            with self.subTest(state=state):
                self.window.p3d.simulation_running.side_effect = state if isinstance(state, Exception) else None
                self.window.p3d.simulation_running.return_value = state
                with self.assertRaisesRegex(RuntimeError, 'Unable to confirm P3D running state'):
                    self.prepare()
                self.assertEqual(self.window._session_context['manual_airflow_preparation']['running_state'], 'UNKNOWN')
                self.window.p3d.unpause_sim.assert_not_called()
                self.assertIsNone(self.window._manual_airflow_ready)
                self.assertFalse(self.window._prepared)
                self.assertFalse(self.running)

    def test_unpause_not_acknowledged_fails_without_repeated_commands(self):
        self.window.p3d.unpause_sim.side_effect = None
        with self.assertRaisesRegex(RuntimeError, 'Unexpected P3D pause: unpause did not confirm'):
            self.prepare()
        self.window.p3d.unpause_sim.assert_called_once()
        self.assertLess(self.clock, 4.)
        self.assertFalse(self.running)
        self.assertIsNone(self.window._manual_airflow_ready)
        self.window.p3d.wait_for_paused.assert_called_with(3.)

    def test_unpause_transport_exception_is_reported_and_safely_paused(self):
        self.window.p3d.unpause_sim.side_effect = RuntimeError('transport rejected')
        with self.assertRaisesRegex(RuntimeError, 'unpause request failed: transport rejected'):
            self.prepare()
        self.window.p3d.unpause_sim.assert_called_once()
        self.assertTrue(self.window._session_context['manual_airflow_preparation']['final_pause_confirmed'])
        self.assertFalse(self.window._prepared)

    def test_descent_after_successful_unpause_stops_without_another_unpause(self):
        def descending(sequence, timeout):
            sample = self.fresh(sequence, timeout)
            sample['pressure_alt'] = 1974.
            return sample
        self.window.p3d.wait_for_fresh_telemetry.side_effect = descending
        with self.assertRaisesRegex(RuntimeError, 'excessive MSL altitude deviation'):
            self.prepare()
        self.assertFalse(self.running)
        self.window.p3d.unpause_sim.assert_called_once()
        self.window.p3d.pause_sim.assert_called_once()
        self.window.p3d.queue_position_airborne.assert_not_called()

    def test_stop_during_pause_state_read_prevents_late_unpause(self):
        def stop_during_read():
            self.window.stop_worker_event.is_set.return_value = True
            return False
        self.window.p3d.simulation_running.side_effect = stop_during_read
        with self.assertRaisesRegex(RuntimeError, 'cancelled'): self.prepare()
        self.window.p3d.unpause_sim.assert_not_called()
        self.window.p3d.pause_sim.assert_called_once()

    def test_deadline_during_state_read_prevents_late_unpause(self):
        def late_read():
            self.clock = 60.
            return False
        self.window.p3d.simulation_running.side_effect = late_read
        with self.assertRaisesRegex(RuntimeError, 'timed out'): self.prepare()
        self.window.p3d.unpause_sim.assert_not_called()
        self.assertFalse(self.window._prepared)

    def test_frozen_airdata_with_advancing_packets_is_not_ready(self):
        self.sample.update(aircraft_wind_x_kt=.038, indicated_altitude=1262.834)
        with self.assertRaisesRegex(RuntimeError, 'airflow not recovered'): self.prepare()
        report = self.window._session_context['manual_airflow_preparation']
        self.assertGreater(report['receive_sequence'], 100)
        self.assertEqual(report['running_state'], 'RUNNING')  # Observed during recovery.
        self.assertEqual(report['physics_clock_progression'], 'UNVERIFIED')
        self.assertEqual(report['status'], 'FAILED')
        self.assertFalse(self.running)  # Safety pause happened after that observation.
        self.assertFalse(self.window._prepared)

    def test_existing_pause_interface_handles_pending_event_without_assuming_running(self):
        path = ROOT / 'interfaces/brunner_base/src/p3d_simconnect_v15.py'
        tree = ast.parse(path.read_text(encoding='utf-8'))
        method = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == 'simulation_running')
        scope = dict(time=SimpleNamespace(monotonic=lambda:self.clock, sleep=self.advance))
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), scope)
        bridge = Mock()
        client = SimpleNamespace(_bridge=bridge)
        bridge.GetSimulationRunningV1509.side_effect = [-1, -1, 1]
        self.assertTrue(scope['simulation_running'](client))
        bridge.RequestSimulationRunningV1509.assert_called_once()
        bridge.reset_mock()
        bridge.GetSimulationRunningV1509.side_effect = None
        bridge.GetSimulationRunningV1509.return_value = -1
        with self.assertRaisesRegex(RuntimeError, 'not confirmed'):
            scope['simulation_running'](client)
        bridge.GetSimulationRunningV1509.return_value = 0
        bridge.RequestSimulationRunningV1509.reset_mock()
        self.assertFalse(scope['simulation_running'](client))
        bridge.RequestSimulationRunningV1509.assert_not_called()

    def test_timeout_and_pause_failure_visible(self):
        initial = dict(self.sample)
        self.window.p3d.wait_for_fresh_telemetry.side_effect = lambda *args: dict(self.fresh(*args), aircraft_wind_x_kt=0.)
        with self.assertRaisesRegex(RuntimeError, 'timed out'): self.prepare()
        self.window.p3d.wait_for_paused.return_value = False
        with self.assertRaisesRegex(RuntimeError, 'Safety pause failed'):
            preparation.prepare(self.window, self.cfg, initial, clock=lambda: self.clock)

    def test_missing_nonfinite_and_altimeter_lag_not_accepted(self):
        self.sample['aircraft_wind_x_kt'] = float('nan')
        with self.assertRaisesRegex(RuntimeError, 'nonfinite'): self.prepare()
        del self.sample['aircraft_wind_x_kt']
        with self.assertRaisesRegex(RuntimeError, 'unavailable'): self.prepare()
        self.sample['aircraft_wind_x_kt'] = 8.
        self.sample['indicated_altitude'] = 2100.
        with self.assertRaisesRegex(RuntimeError, 'timed out'): self.prepare()

    def test_recording_rechecks_conditions_and_first_live_sample(self):
        self.prepare()
        with self.assertRaisesRegex(RuntimeError, 'IAS'):
            preparation.verify_recording_sample(self.window, dict(self.sample, airspeed_indicated=83.))
        self.assertIsNone(self.window._manual_airflow_ready)
        self.prepare()
        self.window.manual_initial_conditions = dict(self.cfg, throttle=10.)
        with self.assertRaisesRegex(RuntimeError, 'conditions changed'):
            preparation.verify_recording_sample(self.window, self.sample)

    def test_configured_conditions_applied_once_and_flag_off_unchanged(self):
        with patch('backend.airborne_preparation.prepare', return_value=self.sample) as prepare:
            apply_manual_conditions(self.window, self.cfg)
            prepare.assert_called_once()
            self.window.p3d.queue_position_airborne.assert_called_once()
            self.window.p3d.set_global_weather_metar.assert_called_once()
            self.window.p3d.set_initial_throttle.assert_called_once_with(5.)
            self.window._session_context['acceleration_manual_airflow_preparation'] = False
            prepare.reset_mock()
            apply_manual_conditions(self.window, self.cfg)
            prepare.assert_not_called()
            self.window.p3d.unpause_sim.assert_not_called()
            preparation.verify_recording_sample(self.window, {})

    def test_reapplying_conditions_invalidates_readiness_before_weather_or_position(self):
        self.prepare()
        def weather(*args):
            self.assertIsNone(self.window._manual_airflow_ready)
            return True
        self.window.p3d.set_global_weather_metar.side_effect = weather
        with patch('replay_support.reference_weather_metar', return_value='unchanged weather'), \
                patch('backend.airborne_preparation.prepare', return_value=self.sample):
            apply_manual_conditions(self.window, self.cfg)
        self.window.p3d.queue_position_airborne.assert_called_once()

    def test_preflight_missing_data_pauses_and_never_unpauses(self):
        del self.sample['ambient_wind_spd']
        with self.assertRaisesRegex(RuntimeError, 'unavailable ambient_wind_spd'): self.prepare()
        self.assertFalse(self.window._prepared)
        self.window.p3d.pause_sim.assert_called_once()
        self.window.p3d.unpause_sim.assert_not_called()

    def test_manual_entry_reports_ready_or_failure_without_recording_or_control_changes(self):
        import threading
        import qtg_common_engine as engine
        path = ROOT / 'interfaces/brunner_base/Super_Mushak_Logger_GUI.py'
        tree = ast.parse(path.read_text(encoding='utf-8'))
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'SuperMushakLoggerApp')
        method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'start_manual_test')
        method.decorator_list = []
        scope = dict(threading=threading, STAGE_INITIALIZING='Initializing aircraft',
                     STAGE_CONNECTED_READY='Ready', STAGE_CONNECTION_ERROR='Error')
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), scope)
        original_prepare = preparation.prepare
        self.window.brunner = Mock()
        self.window._update_stage_ui = Mock()
        for failure in (False, True):
            self.window.sig_update_stage.reset_mock()
            self.window.brunner.reset_mock()
            self.sample['airspeed_indicated'] = 83. if failure else 80.
            with patch.object(engine, 'apply_manual_conditions', side_effect=apply_manual_conditions), \
                    patch.object(preparation, 'prepare', side_effect=lambda *args: original_prepare(*args, clock=lambda:self.clock)), \
                    patch('replay_support.reference_weather_metar', return_value=None):
                scope['start_manual_test'](self.window)
                self.window.preparation_thread.join(timeout=2.)
            self.assertFalse(self.window.preparation_thread.is_alive())
            stage, message = self.window.sig_update_stage.emit.call_args.args
            if failure:
                self.assertEqual(stage, 'Error')
                self.assertIn('IAS 83.00', message)
                self.assertFalse(self.window._prepared)
            else:
                self.assertEqual(stage, 'Ready')
                self.assertIn('Ready for Manual Recording', message)
            self.window.p3d.wait_for_paused.assert_called_with(3.)
            self.window.brunner.prepare_manual_mode.assert_called_once()
            self.window.brunner.move_actuators.assert_not_called()
            self.assertFalse(hasattr(self.window, 'worker_thread'))
            self.assertFalse(hasattr(self.window, 'rec_time'))

    def test_operator_opt_in_is_default_off_and_strictly_scoped(self):
        key = 'QTG_ACCELERATION_MANUAL_AIRFLOW_PREPARATION'
        for value in ('', '0', 'true', '1'):
            with patch.dict(preparation.os.environ, {key:value}):
                self.assertEqual(preparation.activation_requested('1.f.(1)', 'manual'), value == '1')
                for reference, mode, method in [('1.f.(2)', 'manual', 'replay'),
                        ('1.f.(1)', 'auto', 'replay'), ('1.c.(1)', 'auto', 'pid'),
                        ('1.f.(1)', 'manual', 'pid')]:
                    self.assertFalse(preparation.activation_requested(reference, mode, method))

    def test_gui_forwards_option_on_both_launch_and_reinitialize(self):
        import qtg_common_engine as engine
        path = ROOT / 'frontend/main.py'
        tree = ast.parse(path.read_text(encoding='utf-8'))
        method = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == '_launch_common_test')
        scope = dict(RESULTS_ROOT=Path('mock-results'), COL=dict(blue='blue', amber='amber', red='red'))
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), scope)
        for reuse in (False, True):
            for flag in ('0', '1'):
                with self.subTest(reuse=reuse, flag=flag):
                    values = {'Alt[ft]':2000, 'IAS[kts]':80, 'Latitude[deg]':32, 'Longitude[deg]':74,
                              'Heading[deg]':35, 'Pitch[deg]':0, 'Bank[deg]':0, 'Flaps':0,
                              'Throttle[%]':5, 'Temperature[C]':15, 'Flt cond':'Airborne'}
                    app = SimpleNamespace(_common_duration_var=Mock(get=lambda:75),
                        _common_duration_vars={'1.f.(1)':Mock(get=lambda:75)},
                        _apply_test_settings=Mock(return_value=True),
                        _test_settings={'1.f.(1)':({k:Mock(get=lambda value=str(v):value) for k,v in values.items()}, None)},
                        _common_process=Mock(poll=lambda:None) if reuse else None,
                        _common_last_state={}, _common_context={}, _common_command=Mock(), after=Mock(),
                        _poll_common_test=Mock(), common_record_button=Mock(), common_status=Mock())
                    spec = SimpleNamespace(reference='1.f.(1)', title='Engine Acceleration', pdf_graphs=[])
                    with patch.dict(preparation.os.environ, {'QTG_ACCELERATION_MANUAL_AIRFLOW_PREPARATION':flag}), \
                            patch.object(engine, 'launch', return_value=(Mock(), {})) as launch:
                        scope['_launch_common_test'](app, 'manual', spec)
                    app.common_status.configure.assert_called()
                    if reuse:
                        app._common_command.assert_called_once()
                        actual = app._common_command.call_args.kwargs
                        launch.assert_not_called()
                    else:
                        launch.assert_called_once()
                        actual = launch.call_args.kwargs
                    self.assertEqual(actual['acceleration_manual_airflow_preparation'], flag == '1')

    def test_scope_excludes_automatic_and_deceleration(self):
        self.window.test_mode = 'auto'
        self.assertFalse(preparation.enabled(self.window))
        self.window.test_mode = 'manual'
        self.window._session_context['test_reference'] = '1.f.(2)'
        self.assertFalse(preparation.enabled(self.window))
        preparation.verify_recording_sample(self.window, {})

    def test_recording_boundary_rejects_before_timer_or_snapshot(self):
        path = ROOT / 'interfaces/brunner_base/Super_Mushak_Logger_GUI.py'
        tree = ast.parse(path.read_text(encoding='utf-8'))
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'SuperMushakLoggerApp')
        method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '_manual_recording_loop')
        clock = Mock()
        scope = dict(time=clock, STAGE_CONNECTION_ERROR='error')
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), scope)
        self.window.brunner = Mock()
        self.window.sig_update_stage = Mock()
        scope['_manual_recording_loop'](self.window)
        clock.perf_counter.assert_not_called()
        self.window.p3d.unpause_sim.assert_not_called()
        self.assertFalse(hasattr(self.window, 'manual_t0_snapshot'))
        self.assertFalse(self.window.is_recording)

    def test_first_unpaused_frame_cannot_capture_invalid_reference(self):
        self.prepare()
        path = ROOT / 'interfaces/brunner_base/Super_Mushak_Logger_GUI.py'
        tree = ast.parse(path.read_text(encoding='utf-8'))
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'SuperMushakLoggerApp')
        method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '_manual_recording_loop')
        scope = dict(time=Mock(perf_counter=Mock(return_value=0.)), STAGE_CONNECTION_ERROR='error')
        marker = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == '_observe_brake_recording_clock')
        exec(compile(ast.Module(body=[marker, method], type_ignores=[]), str(path), 'exec'), scope)
        self.window.brunner = Mock()
        self.window.brunner.read_axis.return_value = (0., 0., 0.)
        self.window.sig_update_stage = Mock()
        self.window.target_duration_sec = 10.
        self.window.rec_time = []
        self.window.p3d.wait_for_fresh_telemetry.side_effect = None
        self.window.p3d.wait_for_fresh_telemetry.return_value = dict(self.sample, pressure_alt=1974.)
        scope['_manual_recording_loop'](self.window)
        self.assertEqual(self.window.rec_time, [])
        self.assertFalse(hasattr(self.window, 'manual_t0_snapshot'))
        self.assertIsNone(self.window._manual_airflow_ready)
        self.window.brunner.release_controls.assert_called_once()
        self.assertFalse(self.window.is_recording)


if __name__ == '__main__':
    unittest.main()
