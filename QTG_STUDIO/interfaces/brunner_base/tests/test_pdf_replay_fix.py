import ast
import csv
import sys
import tempfile
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch, Mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import Super_Mushak_Logger_GUI as gui

class Signal:
    def __init__(self): self.calls = []
    def emit(self, *args): self.calls.append(args)

class FixTests(unittest.TestCase):
    def test_dense_plot_markers_are_spaced_visually(self):
        import numpy as np
        for path in (ROOT / 'src/plot_graphs.py',):
            namespace = {'__name__': 'plot_under_test'}
            exec(compile(path.read_text(encoding='utf-8'), str(path), 'exec'), namespace)
            fig, ax = namespace['plt'].subplots()
            t = np.linspace(0, 75, 3750)
            namespace['_plot_auto'](ax, t, t, t, t, 'Airspeed', tol=3)
            fig.canvas.draw()
            line = ax.lines[1]
            self.assertIsInstance(line.get_markevery(), float)
            self.assertLessEqual(line.get_markersize(), 3)
            self.assertIn('Tolerance (\u00b13 kt)', ax.get_legend_handles_labels()[1])
            namespace['plt'].close(fig)

    def test_first_csv_sample_preserves_actual_automatic_telemetry(self):
        with tempfile.TemporaryDirectory(dir=ROOT / 'Output') as folder:
            fake = SimpleNamespace(lbl_rec_status=Mock(),lbl_rec_progress=Mock(),rec_progress=Mock(),target_duration_sec=75,
                data_dir=Path(folder), test_mode='auto', rec_time=[0.02],
                p_pos_buf=[0.1], r_pos_buf=[0.2], y_pos_buf=[0.3],
                p_force_buf=[1], r_force_buf=[2], y_force_buf=[3],
                applied_force_buf=[(11,12,13)], virtual_force_buf=[(21,22,23)],
                p3d_tele_buf=[{'airspeed_indicated': 71, 'indicated_altitude': 1454, 'engine_throttle_percent': 0.0}],
                ref_manual_snapshot={'airspeed': 80, 'altitude': 3000},
                auto_t0_snapshot={}, active_manual_ref_path='manual.csv',
                _refresh_manual_reference_selector=lambda: None)
            with patch.object(gui, 'CURRENT_DIR', folder), patch.object(gui.plot_graphs, 'plot_aligned_comparison', return_value=None), patch.object(gui,'QProgressDialog'), patch.object(gui.QApplication,'processEvents'), patch.object(gui.QMessageBox, 'information'), patch.object(gui.QMessageBox, 'critical') as errors:
                gui.SuperMushakLoggerApp._save_csv_and_generate_reports(fake)
                errors.assert_not_called()
            with next(Path(folder).glob('Super_Mushak_auto*.csv')).open() as handle:
                row = next(csv.DictReader(handle))
            self.assertEqual(float(row['Indicated Airspeed (kts)']), 71)
            self.assertEqual(float(row['Indicated Altitude (ft)']), 1454)
            self.assertEqual(float(row['Throttle 1 Position (%)']),0.0)
            self.assertEqual(float(row['Joystick Pitch Force (N)']), 11)
            self.assertEqual(float(row['Pitch Virtual Force 0x51 (N)']), 21)

    def test_replay_uses_recorded_time_instead_of_50hz_row_index(self):
        clock = SimpleNamespace(value=0.0)
        hardware = SimpleNamespace(AXIS_PITCH=1, AXIS_ROLL=2, AXIS_RUDDER=4, commands=[])
        hardware.read_axis = lambda axis: (0, 0, 0)
        hardware.move_actuators = lambda p, r, y: hardware.commands.append(p)
        fake = SimpleNamespace(p3d=Mock(get_telemetry=Mock(return_value={"airspeed_indicated":80.,"engine_throttle_percent":80.,"engine_rpm":2670.}),wait_for_fresh_telemetry=Mock(return_value={"airspeed_indicated":80.})),ref_manual_snapshot={"airspeed":80.},ref_throt_pos=[80.,80.,80.], brunner=hardware,
            _refresh_paused_telemetry=lambda: {"airspeed_indicated":80.}, _settle_reference_controls=lambda: None, replay_command_buf=[], replay_axis_offsets=(0.,0.,0.), stop_worker_event=threading.Event(),
            target_duration_sec=0.13, ref_time=[0, 0.1, 0.12],
            ref_p_pos=[0.1, 0.5, 0.9], ref_r_pos=[0, 0, 0], ref_y_pos=[0, 0, 0],
            rec_time=[], p_pos_buf=[], p_force_buf=[], r_pos_buf=[], r_force_buf=[],
            y_pos_buf=[], y_force_buf=[], p3d_tele_buf=[], applied_force_buf=[], virtual_force_buf=[], sig_record_progress=Signal(),
            sig_recording_finished=Signal())
        def sleep(dt): clock.value += dt
        with patch.object(gui.time, 'perf_counter', side_effect=lambda: clock.value), patch.object(gui.time, 'sleep', side_effect=sleep), patch.object(gui, 'LEAD_MS', 0):
            gui.SuperMushakLoggerApp._auto_replay_loop(fake)
        self.assertAlmostEqual(hardware.commands[0],0.1)
        # A 20 ms command must interpolate the 100 ms sample, not jump to that row.
        self.assertGreater(fake.replay_command_buf[1][1],0.15)
        self.assertLess(fake.replay_command_buf[1][1],0.25)
        self.assertGreater(fake.replay_command_buf[4][1],0.35)
        self.assertLess(fake.replay_command_buf[4][1],0.48)
        self.assertEqual(len(fake.sig_recording_finished.calls), 1)

    def test_duplicate_reference_times_are_skipped_without_misaligning_controls(self):
        with tempfile.TemporaryDirectory(dir=ROOT / 'Output') as folder:
            path=Path(folder)/'manual.csv'
            path.write_text('Time (s),Pitch_Manual_Pos,Throttle 1 Position (%)\n0.0454,0.1,80\n0.0454,0.2,80\n0.0467,0.3,80\n')
            events=[]
            p3d=SimpleNamespace(is_connected=lambda: True, set_position_airborne=Mock(), set_exact_pose_state_no_airspeed=Mock(return_value=True), pause_sim=lambda: events.append('pause'),wait_for_paused=lambda timeout:True,get_global_weather_metar=lambda:'GLOB 000000Z 00000KT 9999 SKC 15/05 Q1013',set_global_weather_metar=lambda metar:True)
            brunner=SimpleNamespace(AXIS_PITCH=1,AXIS_ROLL=2,AXIS_RUDDER=4,read_axis=lambda a:(0.,0.,0.),configure_replay_mode=lambda: None, move_actuators=lambda *a: events.append('move'),release_controls=lambda:None)
            fake=SimpleNamespace(selected_ref_csv=str(path), _update_stage_ui=lambda *a: None,
                ref_p_pos=[], ref_r_pos=[], ref_y_pos=[], ref_throt_pos=[], ref_time=[],
                stop_worker_event=threading.Event(), _settle_reference_controls=lambda: None, _restore_reference_state=lambda: None, _prepare_reference_power=lambda: {},
                p3d=p3d, brunner=brunner, _wait_for_reference_rpm=lambda: None, _refresh_restored_airdata=lambda: None, _commit_initial_position=lambda: None, _wait_for_initial_altitude=lambda a: {}, _refresh_paused_telemetry=lambda: {"airspeed_indicated":80.,"engine_throttle_percent":80.,"engine_rpm":2670.}, sig_update_stage=Signal())
            class ImmediateThread:
                def __init__(self, target, **kw): self.target=target
                def start(self): self.target()
            with patch.object(gui.threading,'Thread',ImmediateThread), patch.object(gui.time,'sleep'):
                gui.SuperMushakLoggerApp.start_auto_test_only(fake)
            self.assertEqual(fake.ref_p_pos,[0.1,0.3])
            self.assertEqual(len(fake.ref_time),2)
            self.assertGreater(fake.ref_time[1],fake.ref_time[0])
            self.assertEqual(fake.sig_update_stage.calls[-1][0],gui.STAGE_CONNECTED_READY)
            p3d.set_position_airborne.assert_called_once()
            p3d.set_exact_pose_state_no_airspeed.assert_not_called()
            self.assertEqual(p3d.set_position_airborne.call_args.kwargs['speed_kts'],80.)
            self.assertLess(events.index('pause'),events.index('move'))

    def test_manual_pdf_uses_applied_force_when_virtual_force_differs(self):
        import pandas as pd
        import numpy as np
        df=pd.DataFrame({'Time (s)':[0.,1.],
            'Joystick Pitch Force (N)':[1.,2.], 'Joystick Roll Force (N)':[3.,4.],
            'Rudder Pedal Force (N)':[5.,6.], 'Pitch Virtual Force 0x51 (N)':[101.,102.],
            'Roll Virtual Force 0x51 (N)':[103.,104.], 'Yaw Virtual Force 0x51 (N)':[105.,106.]})
        with tempfile.TemporaryDirectory(dir=ROOT/'Output') as folder:
            with patch.object(gui.plot_graphs,'_read_csv',return_value=df), patch.object(gui.plot_graphs,'_plot_manual',wraps=gui.plot_graphs._plot_manual) as plot:
                gui.plot_graphs._manual_report('manual.csv',Path(folder)/'manual.pdf')
            calls=[c for c in plot.call_args_list if 'Applied Force' in c.args[3]]
            self.assertEqual(len(calls),3)
            for call,values in zip(calls,([1.,2.],[3.,4.],[5.,6.])):
                np.testing.assert_array_equal(call.args[2],values)

    def test_interpolation_handles_irregular_sampling_and_endpoints(self):
        from replay_support import interpolate_reference
        self.assertAlmostEqual(interpolate_reference([0,.1,.7],[0,1,4],.4),2.5)
        self.assertEqual(interpolate_reference([0,.1],[2,3],-1),2)
        self.assertEqual(interpolate_reference([0,.1],[2,3],5),3)

    def test_control_settling_checks_all_axes_and_requires_stability(self):
        hardware=Mock(AXIS_PITCH=1,AXIS_ROLL=2,AXIS_RUDDER=4)
        hardware.read_axis.side_effect=[(0,0,0)]*3+[(.3,0,0),(.2,0,0),(.1,0,0)]*3
        fake=SimpleNamespace(brunner=hardware, ref_p_pos=[.3], ref_r_pos=[.2], ref_y_pos=[.1], stop_worker_event=threading.Event())
        with patch.object(gui.time,'sleep'):
            gui.SuperMushakLoggerApp._settle_reference_controls(fake)
        self.assertEqual(hardware.move_actuators.call_count,4)
        hardware.release_controls.assert_not_called()

    def test_screenshot_residual_settles_without_relaxing_large_errors(self):
        hardware=Mock(AXIS_PITCH=1,AXIS_ROLL=2,AXIS_RUDDER=4)
        hardware.read_axis.side_effect=[(-.049765229,0,0),(-.005633831,0,0),(0.,0,0)]*3
        fake=SimpleNamespace(brunner=hardware,ref_p_pos=[-.046],ref_r_pos=[-.0028],ref_y_pos=[0.],stop_worker_event=threading.Event())
        with patch.object(gui.time,'sleep'):
            gui.SuperMushakLoggerApp._settle_reference_controls(fake)
        self.assertEqual(hardware.move_actuators.call_count,3)
        hardware.release_controls.assert_not_called()
        self.assertLess(max(abs(v) for v in fake.replay_alignment_errors),.005)

    def test_cancelled_preparation_releases_controls_without_moving(self):
        stop=threading.Event()
        stop.set()
        hardware=Mock(AXIS_PITCH=1,AXIS_ROLL=2,AXIS_RUDDER=4)
        fake=SimpleNamespace(brunner=hardware, ref_p_pos=[.3], ref_r_pos=[.2], ref_y_pos=[.1], stop_worker_event=stop)
        with self.assertRaisesRegex(RuntimeError,'cancelled'):
            gui.SuperMushakLoggerApp._settle_reference_controls(fake)
        hardware.move_actuators.assert_not_called()
        hardware.release_controls.assert_called_once()

    def test_initial_restore_maps_recorded_units_and_verifies_actual_state(self):
        from replay_support import build_dynamic_state
        state={k:0. for k in ['velocity_world_x','velocity_world_y','velocity_world_z',
            'velocity_body_x','velocity_body_y','velocity_body_z','vertical_speed','engine_rpm',
            'rotation_velocity_body_x','rotation_velocity_body_y','rotation_velocity_body_z',
            'elevator_trim','aileron_trim_pct','rudder_trim_pct']}
        state.update(velocity_world_x=105., velocity_world_z=93., velocity_world_y=-.26,
                     vertical_speed=489.,engine_rpm=2655., engine_mixture_lever_1=.63,
                     engine_propeller_lever_1=1.,fuel_tank_levels_all={'left_main':.95},
                     payload_station_weights_lb=[140,140,0],flaps_handle_index=0.,total_weight=847.8)
        driver=Mock()
        for method in ['is_connected','set_payload_station_weights','set_all_fuel_tank_levels',
                       'set_engine_controls','set_flaps_handle_index','set_initial_dynamic_state']:
            getattr(driver,method).return_value=True
        stale=dict(state,velocity_world_y=-1.5)
        refresh=Mock(side_effect=[stale,state])
        fake=SimpleNamespace(ref_manual_snapshot={'telemetry':state},p3d=driver,
                             _refresh_paused_telemetry=refresh,stop_worker_event=threading.Event())
        gui.SuperMushakLoggerApp._restore_reference_state(fake)
        driver.set_engine_controls.assert_not_called()
        driver.set_flaps_handle_index.assert_not_called()
        dynamic=driver.set_initial_dynamic_state.call_args.args[0]
        self.assertEqual(dynamic['velocity_world_y_fps'],-.26)
        self.assertNotIn('vertical_speed_fpm',dynamic)
        self.assertNotIn('elevator_trim_rad',dynamic)
        self.assertEqual(len(dynamic),9)
        self.assertEqual(refresh.call_count,2)
        with self.assertRaisesRegex(ValueError,'Missing or invalid'):
            build_dynamic_state({})
        driver.set_initial_dynamic_state.return_value=False
        with self.assertRaisesRegex(RuntimeError,'rejected initial-state restore'):
            gui.SuperMushakLoggerApp._restore_reference_state(fake)

    def test_control_settling_timeout_releases_hardware(self):
        hardware=Mock(AXIS_PITCH=1,AXIS_ROLL=2,AXIS_RUDDER=4)
        hardware.read_axis.return_value=(0,0,0)
        fake=SimpleNamespace(brunner=hardware, ref_p_pos=[.3], ref_r_pos=[.2], ref_y_pos=[.1], stop_worker_event=threading.Event())
        with patch.object(gui.time,'monotonic',side_effect=[0.,0.,6.]), patch.object(gui.time,'sleep'):
            with self.assertRaisesRegex(RuntimeError,'did not settle'):
                gui.SuperMushakLoggerApp._settle_reference_controls(fake)
        hardware.release_controls.assert_called_once()

    def test_settling_corrects_measured_bias_and_retains_the_offset(self):
        commands=[0.,0.,0.]
        hardware=Mock(AXIS_PITCH=1,AXIS_ROLL=2,AXIS_RUDDER=4)
        def move(*values): commands[:]=values
        hardware.move_actuators.side_effect=move
        def read(axis):
            index={1:0,2:1,4:2}[axis]
            return (commands[index]+(.028,.024,0.)[index],0.,0.)
        hardware.read_axis.side_effect=read
        fake=SimpleNamespace(brunner=hardware,ref_p_pos=[-.2784],ref_r_pos=[-.0423],ref_y_pos=[0.],stop_worker_event=threading.Event())
        with patch.object(gui.time,'sleep'):
            gui.SuperMushakLoggerApp._settle_reference_controls(fake)
        self.assertLess(fake.replay_axis_offsets[0],0.)
        self.assertTrue(all(abs(x)<=.05 for x in fake.replay_axis_offsets))
        self.assertLessEqual(abs(read(1)[0]+.2784),.01)
        self.assertLessEqual(abs(read(2)[0]+.0423),.01)
        hardware.release_controls.assert_not_called()

    def test_p3d_controller_methods_never_call_native_bridge(self):
        from p3d_simconnect_v15 import SimConnectClient
        bridge=Mock()
        fake=SimpleNamespace(_bridge=bridge)
        methods={'set_elevator':(.2,), 'set_aileron':(.2,), 'set_rudder':(.2,),
            'set_elevator_trim':(.1,), 'set_control_inputs':(.1,.2,.3,.4),
            'set_throttle':(80.,), 'set_engine_controls':(60.,100.),
            'set_flaps':(0.,),'set_flaps_handle_index':(0.,)}
        for name,args in methods.items():
            getattr(SimConnectClient,name)(fake,*args)
        self.assertEqual(bridge.mock_calls,[])

    def test_initialization_commit_requests_frames_then_repauses(self):
        driver=Mock()
        driver.get_telemetry_sequence.return_value=1
        driver.wait_for_fresh_telemetry.return_value={'airspeed_indicated':101.}
        fake=SimpleNamespace(p3d=driver,ref_manual_snapshot={},stop_worker_event=threading.Event(),_refresh_paused_telemetry=Mock())
        with patch.object(gui.time,'sleep'), patch.object(gui.time,'monotonic',side_effect=[0.,2.,2.3,2.6]):
            gui.SuperMushakLoggerApp._commit_initial_position(fake)
        self.assertGreaterEqual(driver.unpause_sim.call_count,1)
        self.assertEqual(driver.request_telemetry_once.call_count,3)
        driver.pause_sim.assert_called_once()
        fake._refresh_paused_telemetry.assert_called_once()

    def test_initialization_commit_timeout_always_repauses(self):
        driver=Mock()
        driver.wait_for_fresh_telemetry.return_value=None
        fake=SimpleNamespace(p3d=driver,ref_manual_snapshot={},stop_worker_event=threading.Event(),_refresh_paused_telemetry=Mock())
        with self.assertRaisesRegex(RuntimeError,'No live frame'):
            gui.SuperMushakLoggerApp._commit_initial_position(fake)
        driver.pause_sim.assert_called_once()
        fake._refresh_paused_telemetry.assert_not_called()

    def test_airflow_wait_does_not_accept_zero_wind(self):
        driver=Mock()
        driver.get_telemetry_sequence.return_value=1
        driver.wait_for_fresh_telemetry.side_effect=[
            {'aircraft_wind_x_kt':v,'aircraft_wind_y_kt':0.,'aircraft_wind_z_kt':0.}
            for v in [0.,4.,8.,8.,8.]]
        fake=SimpleNamespace(p3d=driver,ref_manual_snapshot={'telemetry':{
            'aircraft_wind_x_kt':8.,'aircraft_wind_y_kt':0.,'aircraft_wind_z_kt':0.}},
            stop_worker_event=threading.Event(),_refresh_paused_telemetry=Mock())
        with patch.object(gui.time,'sleep'), patch.object(gui.time,'monotonic',side_effect=[0.,2.,3.,4.,5.,6.]):
            gui.SuperMushakLoggerApp._commit_initial_position(fake)
        self.assertEqual(driver.request_telemetry_once.call_count,5)
        driver.pause_sim.assert_called_once()

    def test_initial_instruments_convert_fpm_to_fps_without_control_writes(self):
        from p3d_simconnect_v15 import SimConnectClient
        bridge=Mock()
        bridge.SetInitialInstrumentsV1506.return_value=1
        self.assertTrue(SimConnectClient.set_initial_instruments(SimpleNamespace(_bridge=bridge),240.,2900.))
        bridge.SetInitialInstrumentsV1506.assert_called_once_with(4.,2900.)
        self.assertEqual(len(bridge.mock_calls),1)
        bridge.reset_mock()
        self.assertFalse(SimConnectClient.set_initial_instruments(SimpleNamespace(_bridge=bridge),float('nan'),2900.))
        self.assertEqual(bridge.mock_calls,[])

    def test_motion_restore_uses_nine_kinematic_values(self):
        from p3d_simconnect_v15 import SimConnectClient
        bridge=Mock()
        bridge.SetFullKinematicStateV1505.return_value=1
        state={key:float(i) for i,key in enumerate(['velocity_world_x_fps','velocity_world_y_fps',
            'velocity_world_z_fps','velocity_body_x_fps','velocity_body_y_fps','velocity_body_z_fps','rotation_velocity_body_x_rad_s','rotation_velocity_body_y_rad_s',
            'rotation_velocity_body_z_rad_s'])}
        self.assertTrue(SimConnectClient.set_initial_dynamic_state(SimpleNamespace(_bridge=bridge),state))
        self.assertEqual(list(bridge.SetFullKinematicStateV1505.call_args.args[0]),[0.,1.,2.,3.,4.,5.,6.,7.,8.])
        self.assertEqual(len(bridge.mock_calls),1)

    def test_simulator_controller_channels_are_not_exposed(self):
        from p3d_simconnect_v15 import SimConnectClient
        fake=SimpleNamespace(_lock=threading.Lock(), _latest_data={
            'elevator_pos':.5,'elevator_trim':.1,'max_throttle':80.,'yoke_y_percent':50.,
            'engine_mixture_lever_1':.6,'airspeed_indicated':80.,'pitch_angle':3.})
        self.assertEqual(SimConnectClient.get_telemetry(fake),{'airspeed_indicated':80.,'pitch_angle':3.})

    def test_missing_fresh_telemetry_blocks_recording(self):
        p3d = SimpleNamespace(is_connected=lambda: True, get_telemetry_sequence=lambda: 10,
            request_telemetry_once=lambda: True, wait_for_fresh_telemetry=lambda *args: None)
        with self.assertRaisesRegex(RuntimeError, 'No fresh paused telemetry'):
            gui.SuperMushakLoggerApp._refresh_paused_telemetry(SimpleNamespace(p3d=p3d))

if __name__ == '__main__': unittest.main(verbosity=2)
