import csv
import json
from pathlib import Path
import sys
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock,patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/"brunner"/"src"))
import qtg_climb_pid as pid
import qtg_common_engine as engine

class ClimbPidTests(unittest.TestCase):
    def test_method_cannot_switch_while_existing_worker_is_alive(self):
        module=SimpleNamespace(SuperMushakLoggerApp=object)
        cls=engine.worker_class(module,{},lambda value:None)
        window=object.__new__(cls)
        window.is_recording=False;window.worker_thread=Mock(is_alive=lambda:True)
        window._control_method='replay';window._publish=Mock()
        window.handle_command({'action':'initialize','mode':'auto','control_method':'pid'})
        self.assertEqual(window._control_method,'replay')
        self.assertIn('wait',window._stage_details)

    def test_strict_hardware_does_not_replace_failed_feedback_with_zero(self):
        from brunner_interface import BrunnerInterface
        device=BrunnerInterface();device.strict_io=True
        with self.assertRaises(RuntimeError):device.read_axis(1)
        device.sock=Mock();device.sock.recv.side_effect=[BlockingIOError(),b'']
        with self.assertRaises(RuntimeError):device.read_axis(1)
        device.sock.recv.side_effect=None;device.sock.recv.return_value=b'\x00\x00\x01'
        with self.assertRaises(RuntimeError):device.write_setting(1,0x80,'f',0.)
        with self.assertRaises(RuntimeError):pid.watchdog(device)

    def test_pid_engagement_matches_original_controller_without_replay_overrides(self):
        import struct
        from brunner_interface import BrunnerInterface
        device=BrunnerInterface();device.strict_io=True
        class SharedSocket:
            def __init__(self):self.blocking=True;self.packets=[]
            def setblocking(self,value):self.blocking=value
            def settimeout(self,value):pass
            def sendto(self,packet,address):self.packets.append(packet)
            def recvfrom(self,size):
                if not self.blocking:raise BlockingIOError()
                return b'\x01\x00\x00',('127.0.0.1',15090)
        device.sock=SharedSocket()
        original_socket=device.sock
        with patch.object(pid.socket,'socket',side_effect=AssertionError('No second socket allowed')):
            device.configure_pid_mode(.1)
        expected=[struct.pack('<IIH',0xD8,0,1000),struct.pack('<Iii',0xD1,1,3),struct.pack('<Iii',0xD1,1,4),
            struct.pack('<IiiH',0xCE,1,0x50,2400),struct.pack('<IiiH',0xCE,1,0x60,80),
            struct.pack('<Iiif',0xCE,1,0x80,.1),struct.pack('<IiiB',0xCE,1,0x70,1)]
        self.assertEqual(device.sock.packets,expected)
        self.assertIs(device._pid_transport.sock,original_socket)
        self.assertIs(device._pid_transport.io_lock,device._io_lock)
        device.move_actuators(.2,None,None)
        self.assertEqual(device.sock.packets[-1],struct.pack('<Iiif',0xCE,1,0x80,.2))
        device.release_controls()
        self.assertIsNone(device._pid_transport)
        self.assertEqual(device.sock.packets[-4:],[struct.pack('<IiiB',0xCE,1,0x70,0),struct.pack('<Iii',0xD1,0,3),struct.pack('<Iii',0xD1,0,4),struct.pack('<IIH',0xD8,0,0)])

    def test_pid_pitch_read_prefers_active_node_two_on_shared_socket(self):
        import struct
        from brunner_interface import BrunnerInterface
        device=BrunnerInterface();device.strict_io=True;device.pid_position_nodes=True;device.sock=Mock()
        payload=struct.pack('<HfHf',1,0.,2,.25)
        position=struct.pack('<HB',len(payload)+1,0)+payload
        force=struct.pack('<HBIf',9,0,2,2.)
        # Position, applied force and virtual force all use the existing socket.
        device.sock.recv.side_effect=[BlockingIOError(),position,force,force]
        actual=device.read_axis(1)
        self.assertEqual(actual[0],.25)
        self.assertEqual(len(device.sock.send.call_args_list),3)

    def test_initial_flap_setter_requires_pause_and_keeps_runtime_setter_disabled(self):
        SimConnectClient=engine.load_base().SimConnectClient
        client=object.__new__(SimConnectClient)
        client._bridge=Mock()
        client._bridge.SetInitialFlapsV1517.return_value=1
        client.wait_for_paused=Mock(return_value=True)
        self.assertTrue(client.set_initial_flaps(0))
        client._bridge.SetInitialFlapsV1517.assert_called_once_with(0.)
        self.assertFalse(client.set_flaps_handle_index(0))
        client._bridge.SetInitialFlapsV1517.reset_mock()
        client.wait_for_paused.return_value=False
        with self.assertRaises(RuntimeError):client.set_initial_flaps(0)
        client._bridge.SetInitialFlapsV1517.assert_not_called()
        for value in [-1,.5,float('nan'),11]:
            with self.assertRaises(ValueError):client.set_initial_flaps(value)

    def test_preparation_leaves_pid_and_recording_under_operator_control(self):
        class Base:
            def _update_stage_ui(self,stage,details): self.current_stage=stage
        module=SimpleNamespace(SuperMushakLoggerApp=Base,STAGE_CONNECTED_READY='ready',STAGE_RECORDING='recording',STAGE_AUTO_BLOCKED='blocked',STAGE_CONNECTION_ERROR='error',STAGE_STOPPED_BY_USER='stopped')
        cls=engine.worker_class(module,{},lambda value:None)
        window=object.__new__(cls)
        window._control_method='pid';window._preparing=True;window._prepared=False
        window.stop_worker_event=threading.Event();window._publish=Mock();window.arm_recording=Mock();window.lbl_rec_status=Mock();window.lbl_rec_progress=Mock()
        window._update_stage_ui('ready')
        window.arm_recording.assert_not_called()
        self.assertTrue(window._prepared)
        self.assertEqual(window.current_stage,'ready')

    def test_absent_rudder_is_not_commanded_or_recorded_as_zero(self):
        import math
        from brunner_interface import AxisUnavailableError
        def read(axis):
            if axis==4:raise AxisUnavailableError('No rudder measurement')
            return (.1,2.,3.)
        device=SimpleNamespace(AXIS_PITCH=1,AXIS_ROLL=2,AXIS_RUDDER=4,read_axis=Mock(side_effect=read),move_actuators=Mock())
        axes=pid.discover_axes(device)
        self.assertEqual(axes,(1,2))
        window=SimpleNamespace(brunner=device,pid_axes=axes)
        pid.move_controls(window,.2)
        device.move_actuators.assert_called_once_with(.2,None,None)
        measured=pid.measured_axes(window)
        self.assertTrue(all(math.isnan(value) for value in measured[2]))
        device.read_axis.side_effect=AxisUnavailableError('No pitch measurement')
        with self.assertRaisesRegex(RuntimeError,'pitch-loader'):pid.discover_axes(device)

    def test_empty_success_reply_identifies_unavailable_axis(self):
        from brunner_interface import AxisUnavailableError,BrunnerInterface
        device=BrunnerInterface();device.strict_io=True;device.sock=Mock()
        device.sock.recv.side_effect=[BlockingIOError(),bytes.fromhex('010000')]
        with self.assertRaises(AxisUnavailableError):device.read_axis(4)

    def test_recording_requires_operator_request_after_pid_engagement(self):
        module=SimpleNamespace(SuperMushakLoggerApp=object)
        cls=engine.worker_class(module,{},lambda value:None)
        window=object.__new__(cls)
        window._control_method='pid';window.pid_controller_active=True;window.pid_paused=True
        window.pid_conditions={'target_ias':80};window.pid_live={'ias':80}
        window.pid_record_requested=False;window.is_recording=False;window._publish=Mock()
        window.handle_command({'action':'record'})
        self.assertFalse(window.pid_record_requested)
        window.pid_paused=False;window.pid_live={'ias':100}
        window.handle_command({'action':'record'})
        self.assertFalse(window.pid_record_requested)
        window.pid_live={'ias':80}
        window.handle_command({'action':'record'})
        self.assertTrue(window.pid_record_requested)
        self.assertFalse(window.is_recording)

    def test_running_state_reuses_subscription_and_observes_pause_changes(self):
        client=object.__new__(engine.load_base().SimConnectClient)
        client._bridge=Mock()
        client._bridge.GetSimulationRunningV1509.side_effect=[0,1,0]
        self.assertFalse(client.simulation_running())
        self.assertTrue(client.simulation_running())
        self.assertFalse(client.simulation_running())
        client._bridge.RequestSimulationRunningV1509.assert_not_called()

    def test_running_state_waits_for_asynchronous_response(self):
        driver=engine.load_base()
        client=object.__new__(driver.SimConnectClient);client._bridge=Mock()
        client._bridge.GetSimulationRunningV1509.side_effect=[-1,-1,1]
        client._bridge.RequestSimulationRunningV1509.return_value=1
        with patch.object(driver.time,'sleep'):
            self.assertTrue(client.simulation_running())
        client._bridge.RequestSimulationRunningV1509.assert_called_once()

    def test_unknown_running_state_never_assumes_simulator_is_running(self):
        driver=engine.load_base()
        client=object.__new__(driver.SimConnectClient);client._bridge=Mock()
        client._bridge.GetSimulationRunningV1509.return_value=-1
        client._bridge.RequestSimulationRunningV1509.return_value=1
        with patch.object(driver.time,'monotonic',side_effect=[0.,.1,.26]),patch.object(driver.time,'sleep'):
            with self.assertRaisesRegex(RuntimeError,'not confirmed'):client.simulation_running()

    def test_graph_selection_uses_virtual_force_and_supports_summary_only(self):
        import pymupdf
        import numpy as np
        renderer=engine.load_base().plot_graphs
        with tempfile.TemporaryDirectory() as root:
            source=Path(root)/'climb_manual.csv';output=Path(root)/'graphs.pdf'
            source.write_text('Time (s),Joystick Pitch Force (N),Pitch Virtual Force 0x51 (N)\n0,99,1.5\n1,99,2.5\n')
            captured=[];original=renderer.plt.Axes.plot
            def plot(ax,*args,**kwargs):
                if kwargs.get('label')=='Manual Test (Flight)':captured.append(np.asarray(args[1]).copy())
                return original(ax,*args,**kwargs)
            with patch.object(renderer.plt.Axes,'plot',plot):
                renderer.plot_aligned_comparison(source,output_pdf_path=output,test_reference='1.c.(1)',selected_graphs=['pitch_force'])
            self.assertEqual(captured[0].tolist(),[1.5,2.5])
            with pymupdf.open(output) as pdf:
                self.assertEqual(len(pdf),2)
                self.assertIn('Virtual Force',pdf[1].get_text())
                self.assertNotIn('Applied Force',pdf[1].get_text())
                self.assertNotIn('Combined Group 1',pdf[1].get_text())
            renderer.plot_aligned_comparison(source,output_pdf_path=output,test_reference='1.c.(1)',selected_graphs=[])
            with pymupdf.open(output) as pdf:self.assertEqual(len(pdf),1)

    def test_non_climb_reports_keep_original_renderer_even_with_graph_settings(self):
        renderer=engine.load_base().plot_graphs
        with tempfile.TemporaryDirectory() as root:
            manual=Path(root)/'non_climb_manual.csv';manual.write_text('Time (s)\n0\n')
            automatic=Path(root)/'non_climb_auto.csv';automatic.write_text('Time (s)\n0\n')
            with patch.object(renderer,'_manual_report') as original_manual,patch.object(renderer,'_auto_report') as original_auto,patch.object(renderer,'_pid_report') as climb:
                renderer.plot_aligned_comparison(manual,test_reference='2.a.(1)',selected_graphs=['pitch_force'])
                renderer.plot_aligned_comparison(automatic,manual_csv_path=manual,test_reference='2.a.(1)',selected_graphs=[])
                original_manual.assert_called_once()
                original_auto.assert_called_once()
                climb.assert_not_called()

    def test_climb_uses_increased_ias_gains_without_fixed_trim_bias(self):
        _,Controller,_=pid.controller_types()
        controller=Controller(**pid.CLIMB_PROFILE['ias'])
        cfg=pid.options()
        result=pid.initialize_airspeed_controller(controller,cfg,pid.CLIMB_PROFILE,-4.)
        self.assertFalse(result['calibrated_climb_trim'])
        self.assertEqual(controller.integral,-4.)
        self.assertEqual(controller.cmd_pitch,-4.)
        demand=controller.update(80.,90.,.04,-4.)
        self.assertLessEqual(abs(demand-(-4.)),1.25*.04+1e-9)
        self.assertEqual((controller.kp,controller.ki,controller.kd),(.32,.010,.45))
        result=pid.initialize_airspeed_controller(controller,dict(cfg,throttle=60),pid.CLIMB_PROFILE,-4.)
        self.assertFalse(result['calibrated_climb_trim'])
        self.assertEqual(controller.integral,-4.)

    def test_overspeed_startup_does_not_retain_nose_down_demand(self):
        _,Controller,_=pid.controller_types()
        controller=Controller(**pid.CLIMB_PROFILE['ias'])
        cfg=pid.options()
        state=pid.initialize_airspeed_controller(controller,cfg,pid.CLIMB_PROFILE,-3.,89.)
        self.assertTrue(state['overspeed_startup_correction'])
        self.assertEqual(controller.cmd_pitch,0.)
        self.assertEqual(controller.integral,0.)
        self.assertGreater(controller.update(80.,89.,.04,-3.),0.)
        state=pid.initialize_airspeed_controller(controller,cfg,pid.CLIMB_PROFILE,-3.,76.)
        self.assertFalse(state['overspeed_startup_correction'])
        self.assertEqual(controller.cmd_pitch,-3.)
        state=pid.initialize_airspeed_controller(controller,cfg,pid.CLIMB_PROFILE,5.,89.)
        self.assertFalse(state['overspeed_startup_correction'])
        self.assertEqual(controller.cmd_pitch,5.)

    def test_new_ias_gains_lower_pitch_demand_more_when_airspeed_is_below_target(self):
        _,Controller,_=pid.controller_types()
        original=Controller(kp=.24,ki=.008,kd=.32)
        changed=Controller(**pid.CLIMB_PROFILE['ias'])
        for controller in (original,changed):
            controller.reset(9.)
            controller.update(80.,80.,.04,9.)
            for _ in range(75):controller.update(80.,76.,.04,9.)
        self.assertLess(changed.cmd_pitch,original.cmd_pitch)
        self.assertEqual(pid.CLIMB_PROFILE['pitch'],dict(kp=.030,ki=.004,kd=.045,invert=False))
        self.assertEqual(pid.CLIMB_PROFILE['slew'],dict(max_rate=.28,max_accel=.90))

    def test_supplied_algorithms_and_limits(self):
        Pitch,Speed,slew=pid.controller_types()
        inner=Pitch();outer=Speed()
        outer.reset(4)
        self.assertGreater(outer.update(80,90,.04,4),4)
        for _ in range(100): result=inner.update(100,0,.04)
        self.assertLessEqual(result,.65)
        self.assertGreaterEqual(result,-.55)
        position,velocity=slew(0,0,1,.04,max_rate=.28,max_accel=.90)
        self.assertLessEqual(velocity,.90*.04)
        self.assertLessEqual(position,.28*.04)

    def test_pid_requires_neither_manual_csv_nor_other_test_support(self):
        spec=SimpleNamespace(reference='1.c.(1)',title='Normal climb')
        with tempfile.TemporaryDirectory() as folder,patch.object(engine.subprocess,'Popen'):
            _,context=engine.launch('auto',spec,folder,control_method='pid')
            self.assertIsNone(context['reference_csv'])
            self.assertEqual(context['control_method'],'pid')
            with self.assertRaises(ValueError):engine.launch('auto',SimpleNamespace(reference='1.f.(1)',title='Acceleration'),folder,control_method='pid')
            with self.assertRaises((ValueError,TypeError)):engine.launch('auto',spec,folder,control_method='replay')

    def test_initialization_dispatch_never_calls_replay(self):
        window=SimpleNamespace(p3d=Mock(is_connected=lambda:True),brunner=SimpleNamespace(sock=object()),_control_method='pid',_base_module=object(),_session_context={'test_reference':'1.c.(1)'},start_auto_test_only=Mock())
        with patch.object(pid,'prepare') as prepare:
            engine.initialize_mode(window,'auto')
            prepare.assert_called_once()
        window.start_auto_test_only.assert_not_called()

    def test_climb_initializes_bank_and_yaw_zero(self):
        actual=dict(measured_flaps_handle_index=0.,airspeed_indicated=100.,pressure_alt=3000.,bank_angle=0.,true_heading=0.,engine_throttle_percent=100.,plane_latitude=32.235,plane_longitude=74.135,ambient_temp=15.)
        sim=Mock();sim.wait_for_paused.return_value=True
        sim.set_flaps_handle_index.return_value=False
        sim.set_initial_flaps.return_value=True
        device=Mock();device.AXIS_PITCH=1;device.AXIS_ROLL=2;device.AXIS_RUDDER=4;device.read_axis.return_value=(0,0,0)
        window=SimpleNamespace(p3d=sim,brunner=device,stop_worker_event=threading.Event(),sig_update_stage=Mock(),_refresh_paused_telemetry=Mock(return_value=actual),_wait_for_initial_altitude=Mock(),_commit_initial_position=Mock())
        base=SimpleNamespace(STAGE_INITIALIZING='init',STAGE_CONNECTED_READY='ready',STAGE_AUTO_BLOCKED='blocked')
        window.target_duration_sec=10
        for name in ('rec_time','p_pos_buf','r_pos_buf','y_pos_buf','p_force_buf','r_force_buf','y_force_buf','p3d_tele_buf','applied_force_buf','virtual_force_buf','replay_command_buf'):setattr(window,name,[])
        class Immediate:
            def __init__(self,target,**kwargs):self.target=target
            def start(self):self.target()
        with patch.object(pid.threading,'Thread',Immediate),patch.object(pid,'loop') as controller,patch.object(pid,'watchdog'),patch.object(pid,'verify_forwarding'),patch('replay_support.reference_weather_metar',return_value=None):
            pid.prepare(window,base,{'test_reference':'1.c.(1)'})
        call=sim.queue_position_airborne.call_args.kwargs
        self.assertEqual(call['bank_deg'],0.)
        self.assertEqual(call['hdg_deg'],0.)
        controller.assert_called_once_with(window,base)
        self.assertTrue(window.pid_automatic_workflow)
        self.assertEqual(window.pid_conditions['target_ias'],80.)
        sim.unpause_sim.assert_not_called()
        sim.queue_position_airborne.assert_called_once_with(alt_ft=3000.,speed_kts=80.,pitch_deg=0.,bank_deg=0.,hdg_deg=0.,lat=32.235,lon=74.135,on_ground=False)
        device.prepare_manual_mode.assert_called_once()
        sim.set_exact_pose_state_no_airspeed.assert_not_called()
        window._commit_initial_position.assert_not_called()
        device.configure_replay_mode.assert_not_called()
        sim.set_initial_flaps.assert_called_once_with(0.)
        sim.set_flaps_handle_index.assert_not_called()
        sim.set_control_inputs.assert_not_called()
        sim.set_initial_dynamic_state.assert_not_called()

    def test_climb_automatic_uses_manual_initializer_before_pid_and_stops_on_failure(self):
        actual={'pressure_alt':4500}
        cfg=dict(alt_ft=4500,ias_kt=90,lat=31.5,lon=73.25,flaps=0,throttle=80,temperature_c=20,on_ground=False)
        class Immediate:
            def __init__(self,target,**kwargs):self.target=target
            def start(self):self.target()
        for fails in (False,True):
            with self.subTest(fails=fails):
                window=SimpleNamespace(brunner=Mock(),p3d=Mock(),target_duration_sec=10,stop_worker_event=threading.Event(),sig_update_stage=Mock())
                for name in ('rec_time','p_pos_buf','r_pos_buf','y_pos_buf','p_force_buf','r_force_buf','y_force_buf','p3d_tele_buf','applied_force_buf','virtual_force_buf','replay_command_buf'):setattr(window,name,[])
                context={'test_reference':'1.c.(1)','initial_conditions':cfg}
                base=SimpleNamespace(STAGE_INITIALIZING='init',STAGE_AUTO_BLOCKED='blocked')
                with patch.object(pid.threading,'Thread',Immediate),patch.object(engine,'apply_manual_conditions',return_value=actual) as manual,patch.object(pid,'loop') as run,patch.object(pid,'safe_release') as release,patch.object(pid,'watchdog') as timer:
                    if fails:manual.side_effect=RuntimeError('Manual initialization failed')
                    else:run.side_effect=lambda *args:manual.assert_called_once_with(window,cfg)
                    pid.prepare(window,base,context)
                manual.assert_called_once_with(window,cfg)
                window.brunner.prepare_manual_mode.assert_called_once()
                timer.assert_called_once_with(window.brunner,0)
                if fails:
                    run.assert_not_called();release.assert_called_once()
                    self.assertFalse(context['pid_complete'])
                else:
                    run.assert_called_once_with(window,base)
                    self.assertEqual(context['pid_initialization_method'],'Shared Manual initialization')
                    self.assertEqual(window.pid_conditions['alt_ft'],4500)

    def test_forwarding_waits_for_real_loader_startup_latency(self):
        clock=[0.];state={'command':0.,'actual':0.}
        def move(pitch,roll,yaw):state['command']=pitch
        def read(axis):
            if clock[0]>=1.:state['actual']=state['command']
            return state['actual'],0.,0.
        device=SimpleNamespace(AXIS_PITCH=1,_io_lock=threading.RLock(),read_axis=Mock(side_effect=read),move_actuators=Mock(side_effect=move))
        event=threading.Event()
        def wait(delay):clock[0]+=delay;return False
        event.wait=wait
        window=SimpleNamespace(brunner=device,p3d=Mock(),stop_worker_event=event)
        def feedback(*args):return {'surface_elevator_rad':state['actual']*.1,'bank_angle':0.,'airspeed_indicated':80.}
        with patch.object(pid,'watchdog'),patch.object(pid.time,'monotonic',side_effect=lambda:clock[0]),patch.object(pid,'fresh',side_effect=feedback):
            pid.verify_forwarding(window)
        self.assertGreaterEqual(clock[0],1.)
        self.assertTrue(window.pid_forwarding_check['verified'])
        self.assertEqual(window.pid_forwarding_check['targets_reached'],[True,True])
        self.assertTrue(all(abs(call.args[0])<=.025+1e-12 for call in device.move_actuators.call_args_list))

    def test_forwarding_does_not_accept_a_loader_that_never_moves(self):
        clock=[0.]
        device=SimpleNamespace(AXIS_PITCH=1,_io_lock=threading.RLock(),read_axis=Mock(return_value=(0.,0.,0.)),move_actuators=Mock())
        event=threading.Event()
        def wait(delay):clock[0]+=delay;return False
        event.wait=wait
        window=SimpleNamespace(brunner=device,p3d=Mock(),stop_worker_event=event)
        feedback={'surface_elevator_rad':0.,'bank_angle':0.,'airspeed_indicated':80.}
        with patch.object(pid,'watchdog'),patch.object(pid.time,'monotonic',side_effect=lambda:clock[0]),patch.object(pid,'fresh',return_value=feedback):
            with self.assertRaisesRegex(RuntimeError,'within 2.5 seconds'):pid.verify_forwarding(window)
        self.assertFalse(window.pid_forwarding_check['verified'])
        self.assertEqual(window.pid_forwarding_check['targets_reached'],[False,False])
        self.assertLess(clock[0],6.)

    def test_forwarding_check_requires_both_physical_and_surface_movement(self):
        device=SimpleNamespace(AXIS_PITCH=1,AXIS_ROLL=2,AXIS_RUDDER=4,_io_lock=threading.RLock(),read_axis=Mock(side_effect=[(0,0,0),(0,0,0),(-.025,0,0),(-.025,0,0),(.025,0,0)]),move_actuators=Mock())
        window=SimpleNamespace(brunner=device,p3d=Mock(),stop_worker_event=threading.Event())
        with patch.object(pid,'watchdog'),patch.object(pid.time,'sleep'),patch.object(pid,'fresh',return_value={'surface_elevator_rad':0.,'bank_angle':0.,'airspeed_indicated':80.}):
            with self.assertRaisesRegex(RuntimeError,'forwarding was not verified'):pid.verify_forwarding(window)
        self.assertFalse(window.pid_forwarding_check['verified'])
        self.assertEqual(len(window.pid_forwarding_check['samples']),2)
        self.assertEqual(window.pid_forwarding_check['surface_delta_rad'],0.)

    def test_pid_loop_records_all_channels_and_commands_no_simulator_controls(self):
        actual=dict(airspeed_indicated=80.,pitch_angle=-4.,bank_angle=0.,true_heading=0.)
        speeds=iter([100.,100.,90.,80.,80.,80.])
        sim=Mock();sim.is_connected.return_value=True;sim.request_telemetry_once.return_value=True;sim.wait_for_fresh_telemetry.side_effect=lambda *args:dict(actual,airspeed_indicated=next(speeds,80.));sim.simulation_running.return_value=True
        device=SimpleNamespace(AXIS_PITCH=1,AXIS_ROLL=2,AXIS_RUDDER=4,_io_lock=threading.RLock(),read_axis=Mock(return_value=(0.,2.,3.)),move_actuators=Mock(),release_controls=Mock(),configure_replay_mode=Mock(),configure_pid_mode=Mock(),write_setting=Mock())
        window=SimpleNamespace(p3d=sim,brunner=device,pid_conditions=pid.options(),pid_controller_active=True,pid_record_requested=False,is_recording=False,stop_worker_event=threading.Event(),target_duration_sec=.09,sig_recording_finished=Mock(),sig_record_progress=Mock(),sig_update_stage=Mock(),pid_diagnostics=[])
        for name in ['rec_time','p_pos_buf','r_pos_buf','y_pos_buf','p_force_buf','r_force_buf','y_force_buf','p3d_tele_buf','applied_force_buf','virtual_force_buf','replay_command_buf']:setattr(window,name,[])
        # Operator requests recording only after seeing stable IAS; the controller
        # keeps running throughout, with no internal unpause or auto trigger.
        calls=0
        def wait(delay):
            nonlocal calls
            calls+=1
            if calls>=3:window.pid_record_requested=True
            import time
            time.sleep(delay)
            return False
        window.stop_worker_event.wait=wait
        with patch.object(pid,'watchdog'):
            pid.loop(window,SimpleNamespace(STAGE_CONNECTION_ERROR='error',STAGE_RECORDING='Recording',STAGE_STOPPED_BY_USER='stopped'))
        sim.unpause_sim.assert_not_called()
        self.assertTrue(window.rec_time)
        self.assertEqual(window.rec_time[0],0.)
        self.assertTrue(all(row['airspeed_indicated']==80. for row in window.p3d_tele_buf))
        self.assertTrue(any(row['phase']=='holding' and row['ias']==100. for row in window.pid_diagnostics))
        self.assertEqual(len(window.applied_force_buf),len(window.rec_time))
        self.assertEqual(window.applied_force_buf[0],(2.,2.,2.))
        self.assertEqual(window.virtual_force_buf[0],(3.,3.,3.))
        self.assertTrue(all(call.args[1:]==(None,None) for call in device.move_actuators.call_args_list))
        sim.set_control_inputs.assert_not_called();sim.set_initial_dynamic_state.assert_not_called();sim.set_replay_throttle.assert_not_called()
        sim.pause_sim.assert_called();device.release_controls.assert_called()

    def test_climb_gui_launches_service_without_pid_window_or_manual_reference(self):
        import main
        spec=SimpleNamespace(reference='1.c.(1)',title='Normal climb',pdf_graphs=[])
        values={'Flt cond':'Climb','Alt[ft]':'3000','IAS[kts]':'80','Latitude[deg]':'32.235','Longitude[deg]':'74.135','Flaps':'0','Throttle[%]':'100','Temperature[C]':'15'}
        fields={key:Mock(get=lambda value=value:value) for key,value in values.items()}
        host=SimpleNamespace(_common_duration_var=Mock(get=lambda:'10'),_automatic_methods={spec.reference:Mock(get=lambda:'PID Automatic')},
            _test_settings={spec.reference:(fields,{})},_apply_test_settings=lambda spec:True,
            _pid_validation={spec.reference:Mock(get=lambda:'')},_common_process=None,
            common_record_button=Mock(),common_status=Mock(),after=Mock(),_poll_common_test=Mock(),_open_pid_monitor=Mock())
        with patch.object(engine,'launch',return_value=(Mock(),{})) as launch:
            main.App._launch_common_test(host,'auto',spec)
        launch.assert_called_once()
        self.assertEqual(launch.call_args.args[3],None)
        self.assertEqual(launch.call_args.args[4],10)
        self.assertEqual(launch.call_args.kwargs['control_method'],'pid')
        self.assertEqual(launch.call_args.kwargs['pid_conditions']['target_ias'],80.)
        host._open_pid_monitor.assert_not_called()
        # The separate Replay button bypasses the PID selection without changing it.
        host._common_reference_var=Mock(get=lambda:'manual.csv')
        host._common_process=None
        with patch.object(engine,'validate_reference',return_value='manual.csv'),patch.object(engine,'launch',return_value=(Mock(),{})) as replay:
            main.App._launch_common_test(host,'auto',spec,control_method='replay')
        replay.assert_called_once()
        self.assertEqual(replay.call_args.kwargs['control_method'],'replay')
        self.assertIsNone(replay.call_args.kwargs['pid_conditions'])
        self.assertEqual(replay.call_args.args[3],'manual.csv')
        self.assertEqual(host._automatic_methods[spec.reference].get(),'PID Automatic')
        host._open_pid_monitor.assert_not_called()

    def test_climb_airspeed_graph_shows_target_and_three_knot_tolerance(self):
        renderer=engine.load_base().plot_graphs
        fig,ax=renderer.plt.subplots()
        renderer._draw_climb_airspeed_target(ax,80.)
        self.assertEqual([list(line.get_ydata()) for line in ax.lines],[[77.,77.],[83.,83.],[80.,80.]])
        labels=ax.get_legend_handles_labels()[1]
        self.assertTrue(any('77-83' in label for label in labels))
        self.assertTrue(any('Target IAS (80 kt)' in label for label in labels))
        self.assertEqual(len(ax.patches),1)
        renderer.plt.close(fig)

    def test_climb_airspeed_axis_has_whole_number_ticks_without_rounding_data(self):
        renderer=engine.load_base().plot_graphs
        fig,ax=renderer.plt.subplots()
        values=[79.15,80.75]
        line,=ax.plot([0,75],values)
        renderer._format_climb_airspeed_axis(ax)
        fig.canvas.draw()
        self.assertTrue(all(float(tick).is_integer() for tick in ax.get_yticks()))
        self.assertTrue(all('.' not in tick.get_text() for tick in ax.get_yticklabels()))
        self.assertEqual(list(line.get_ydata()),values)
        renderer.plt.close(fig)

    def test_pid_and_manual_launch_default_to_seventy_five_seconds(self):
        with tempfile.TemporaryDirectory() as root:
            spec=SimpleNamespace(reference='1.c.(1)',title='Normal climb')
            with patch.object(engine.subprocess,'Popen'):
                _,automatic=engine.launch('auto',spec,Path(root)/'sessions',control_method='pid')
                _,manual=engine.launch('manual',spec,Path(root)/'sessions')
            self.assertEqual(automatic['duration_s'],75)
            self.assertEqual(manual['duration_s'],75)


    def test_startup_resume_is_bounded_and_never_overrides_stop_or_active_pid_pause(self):
        clock=[0.]
        window=SimpleNamespace(p3d=Mock(),stop_worker_event=threading.Event(),pid_phase='starting',_session_context={})
        window.p3d.simulation_running.return_value=True
        window.stop_worker_event.wait=lambda delay:clock.__setitem__(0,clock[0]+delay)
        with patch.object(pid.time,'monotonic',side_effect=lambda:clock[0]):
            pid.resume_initialization(window)
            pid.resume_initialization(window)
            with self.assertRaisesRegex(RuntimeError,'repeatedly paused'):pid.resume_initialization(window)
        self.assertEqual(window.p3d.unpause_sim.call_count,2)
        window.pid_phase='recording'
        with self.assertRaisesRegex(RuntimeError,'active PID'):pid.resume_initialization(window)
        window.stop_worker_event.set()
        with self.assertRaisesRegex(RuntimeError,'cancelled'):pid.resume_initialization(window)
        self.assertEqual(window.p3d.unpause_sim.call_count,2)

    def test_aircraft_settling_restarts_after_initial_pause(self):
        clock=[0.];running=[False]
        data=dict(airspeed_indicated=80.,pitch_angle=0.,bank_angle=0.,true_heading=0.,engine_throttle_percent=100.,engine_rpm=2700.,pressure_alt=3000.,ambient_temp=15.,measured_flaps_handle_index=0.)
        window=SimpleNamespace(p3d=Mock(),stop_worker_event=threading.Event(),pid_phase='starting',pid_conditions=pid.options(),_session_context={})
        window.p3d.simulation_running.side_effect=lambda:running[0]
        window.p3d.unpause_sim.side_effect=lambda:running.__setitem__(0,True)
        window.stop_worker_event.wait=lambda delay:clock.__setitem__(0,clock[0]+delay)
        with patch.object(pid.time,'monotonic',side_effect=lambda:clock[0]),patch.object(pid,'fresh',return_value=data):
            pid.wait_for_aircraft_settled(window)
        window.p3d.unpause_sim.assert_called_once()
        self.assertTrue(window._session_context['pid_aircraft_settling']['verified'])
        self.assertGreaterEqual(clock[0],2.25)

    def test_no_watchdog_is_armed_during_weather_or_aircraft_settling(self):
        clock=[0.]
        data=dict(airspeed_indicated=80.,pitch_angle=0.,bank_angle=0.,true_heading=0.,engine_throttle_percent=100.,engine_rpm=2700.,pressure_alt=3000.,ambient_temp=15.,measured_flaps_handle_index=0.)
        window=SimpleNamespace(p3d=Mock(),brunner=Mock(),stop_worker_event=threading.Event(),pid_phase='starting',pid_conditions=pid.options(),_session_context={})
        window.p3d.simulation_running.return_value=True
        window.stop_worker_event.wait=lambda delay:clock.__setitem__(0,clock[0]+delay)
        with patch.object(pid.time,'monotonic',side_effect=lambda:clock[0]),patch.object(pid,'fresh',return_value=data),patch.object(pid,'watchdog') as timer:
            pid.verify_live_temperature(window)
            pid.wait_for_aircraft_settled(window)
        self.assertGreaterEqual(clock[0],2.)
        timer.assert_not_called()
        window.brunner.configure_pid_mode.assert_not_called()

    def test_live_temperature_waits_for_weather_and_never_accepts_persistent_mismatch(self):
        data=dict(airspeed_indicated=80.,pitch_angle=0.,bank_angle=0.,true_heading=0.)
        window=SimpleNamespace(p3d=Mock(),brunner=Mock(),stop_worker_event=threading.Event(),pid_conditions=pid.options(),_session_context={})
        window.p3d.simulation_running.return_value=True
        with patch.object(pid,'watchdog'),patch.object(pid,'fresh',side_effect=[dict(data,ambient_temp=20.85),dict(data,ambient_temp=15.)]):
            pid.verify_live_temperature(window)
        self.assertTrue(window._session_context['pid_temperature_check']['verified'])
        with patch.object(pid,'watchdog'),patch.object(pid,'fresh',return_value=dict(data,ambient_temp=20.85)),patch.object(pid.time,'monotonic',side_effect=[0.,16.]):
            with self.assertRaisesRegex(RuntimeError,'requested 15 C, actual 20.85 C'):pid.verify_live_temperature(window)
        self.assertFalse(window._session_context['pid_temperature_check']['verified'])
        window.stop_worker_event.set()
        with patch.object(pid,'fresh') as read:
            with self.assertRaisesRegex(RuntimeError,'cancelled'):pid.verify_live_temperature(window)
            read.assert_not_called()

    def test_live_pid_response_is_required_even_if_airspeed_is_stable(self):
        verifier=pid.PIDResponseVerifier()
        self.assertFalse(verifier.update(0.,0.,0.)['verified'])
        self.assertFalse(verifier.update(.1,0.,0.)['verified'])
        self.assertFalse(verifier.update(.1,.1,0.)['verified'])
        self.assertTrue(verifier.update(.12,.12,.012)['verified'])
        with self.assertRaisesRegex(RuntimeError,'Invalid Pitch'):verifier.update(.1,.1,float('nan'))

    def test_stabilization_requires_continuous_five_seconds_without_acquisition_timeout(self):
        gate=pid.StabilizationGate()
        self.assertFalse(gate.update(0,80,0))
        self.assertFalse(gate.update(4.9,80,0))
        self.assertFalse(gate.update(4.95,81.1,0))
        self.assertFalse(gate.update(5,79,0))
        self.assertFalse(gate.update(9.99,81,0))
        self.assertTrue(gate.update(10,80,0))
        self.assertTrue(gate.update(45.01,80,0))
        late=pid.StabilizationGate()
        self.assertFalse(late.update(0,90,0))
        self.assertFalse(late.update(120,80,0))
        self.assertTrue(late.update(125,80,0))

    def run_automatic_fixture(self,scenario='complete'):
        clock=[0.]
        data=dict(airspeed_indicated=80.,pitch_angle=-4.,bank_angle=0.,true_heading=0.,ambient_temp=15.,measured_flaps_handle_index=0.,pressure_alt=3000.,vertical_speed=500.,engine_throttle_percent=100.,engine_rpm=2700.)
        sim=Mock();sim.is_connected.return_value=True;sim.request_telemetry_once.return_value=True;sim.simulation_running.return_value=True
        engaged=[None]
        def telemetry(*args):
            elapsed=0. if engaged[0] is None else clock[0]-engaged[0]
            position=device.move_actuators.call_args.args[0] if device.move_actuators.call_args else 0.
            return dict(data,airspeed_indicated=90. if engaged[0] is not None and (elapsed<50. if scenario=='timeout' else elapsed<1.) else 80.,surface_elevator_rad=0. if scenario=='forwarding' else position*.1)
        sim.wait_for_fresh_telemetry.side_effect=telemetry
        device=SimpleNamespace(AXIS_PITCH=1,AXIS_ROLL=2,AXIS_RUDDER=4,_io_lock=threading.RLock(),read_axis=Mock(return_value=(0.,2.,3.)),move_actuators=Mock(),release_controls=Mock(),configure_replay_mode=Mock(),configure_pid_mode=Mock(),write_setting=Mock())
        device.read_axis.side_effect=lambda axis:((device.move_actuators.call_args.args[0] if device.move_actuators.call_args else 0.),2.,3.)
        window=SimpleNamespace(p3d=sim,brunner=device,pid_conditions=pid.options(),pid_controller_active=True,pid_automatic_workflow=True,is_recording=False,stop_worker_event=threading.Event(),target_duration_sec=10.,sig_recording_finished=Mock(),sig_record_progress=Mock(),sig_update_stage=Mock(),pid_diagnostics=[],_session_context={})
        for name in ('rec_time','p_pos_buf','r_pos_buf','y_pos_buf','p_force_buf','r_force_buf','y_force_buf','p3d_tele_buf','applied_force_buf','virtual_force_buf','replay_command_buf'):setattr(window,name,[])
        def wait(delay):
            clock[0]+=max(delay,.001)
            if scenario=='stop' and clock[0]>10:window.stop_worker_event.set()
            if scenario=='disconnect' and clock[0]>10:sim.is_connected.return_value=False
            return window.stop_worker_event.is_set()
        window.stop_worker_event.wait=wait
        def engage(position):
            sim.unpause_sim.assert_called_once()
            self.assertTrue(window._session_context['pid_temperature_check']['verified'])
            self.assertTrue(window._session_context['pid_aircraft_settling']['verified'])
            engaged[0]=clock[0]
        device.configure_pid_mode.side_effect=engage
        base=SimpleNamespace(STAGE_INITIALIZING='Initializing',STAGE_CONNECTION_ERROR='Failed',STAGE_RECORDING='Recording',STAGE_STOPPED_BY_USER='Stopped')
        with patch.object(pid.time,'monotonic',side_effect=lambda:clock[0]),patch.object(pid,'watchdog'),patch.object(pid,'verify_forwarding') as forwarding:
            pid.loop(window,base)
            forwarding.assert_not_called()
        return window

    def test_automatic_sequence_stabilizes_records_ten_seconds_and_releases(self):
        window=self.run_automatic_fixture()
        self.assertIsNone(window.pid_fault)
        self.assertEqual(window.rec_time[0],0.)
        self.assertGreaterEqual(window.rec_time[-1],9.9)
        self.assertLess(window.rec_time[-1],10.)
        recording=[row for row in window.pid_diagnostics if row['phase']=='recording']
        self.assertGreaterEqual(recording[0]['time_s'],5.)
        self.assertEqual(len(recording),len(window.rec_time))
        self.assertTrue(window.pid_stabilization['achieved'])
        self.assertTrue(window.pid_forwarding_check['verified'])
        window.brunner.configure_pid_mode.assert_called_once()
        window.brunner.configure_replay_mode.assert_not_called()
        self.assertTrue(all(call.args[1:]==(None,None) for call in window.brunner.move_actuators.call_args_list))
        window.p3d.unpause_sim.assert_called_once()
        window.p3d.set_control_inputs.assert_not_called()
        window.p3d.queue_position_airborne.assert_not_called()
        window.p3d.set_initial_dynamic_state.assert_not_called()
        window.brunner.release_controls.assert_called_once()
        window.sig_recording_finished.emit.assert_called_once()
        self.assertFalse(window.pid_controller_active)
        self.assertTrue(window._session_context['pid_complete'])
        self.assertTrue(window._session_context['pid_engaged_after_live_initialization'])

    def test_pid_can_acquire_airspeed_after_more_than_forty_five_seconds(self):
        window=self.run_automatic_fixture('timeout')
        self.assertIsNone(window.pid_fault)
        self.assertTrue(window.pid_forwarding_check['verified'])
        self.assertTrue(window.pid_stabilization['achieved'])
        self.assertGreaterEqual(window.pid_stabilization['elapsed_s'],55.)
        self.assertIsNone(window.pid_stabilization['timeout_s'])
        self.assertEqual(window.rec_time[0],0.)
        self.assertTrue(window._session_context['pid_complete'])

    def test_automatic_failures_and_stop_always_release_and_mark_incomplete(self):
        for scenario in ('stop','disconnect','forwarding'):
            with self.subTest(scenario=scenario):
                window=self.run_automatic_fixture(scenario)
                self.assertTrue(window.pid_fault)
                self.assertFalse(window._session_context['pid_complete'])
                self.assertFalse(window.pid_controller_active)
                window.brunner.release_controls.assert_called_once()
                if scenario in ('timeout','forwarding'):
                    self.assertFalse(window.rec_time)
                    window.sig_recording_finished.emit.assert_not_called()
                if scenario=='forwarding':
                    window.p3d.unpause_sim.assert_called_once()
                    window.p3d.pause_sim.assert_called_once()

    def test_feedback_loss_always_releases_even_if_pause_fails(self):
        sim=Mock();sim.pause_sim.side_effect=RuntimeError('Disconnected')
        device=SimpleNamespace(strict_io=True,release_controls=Mock())
        window=SimpleNamespace(p3d=sim,brunner=device)
        with patch.object(pid,'watchdog'):errors=pid.safe_release(window)
        self.assertTrue(errors);device.release_controls.assert_called_once();self.assertFalse(device.strict_io)

    def test_reports_do_not_use_manual_as_validation_or_pass_missing_data(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'climb_auto.csv';validation=Path(folder)/'validation.csv'
            text='Time (s),Indicated Airspeed (kts),Rate of Climb/Descent (fpm)\n0,80,500\n1,80,500\n'
            path.write_text(text);validation.write_text(text)
            output=pid.report(path)
            import pymupdf
            with pymupdf.open(output) as document:
                self.assertEqual(len(document),6)
                self.assertTrue(all(page.rect.height>page.rect.width for page in document))
                self.assertIn('PID Automatic',document[0].get_text())
                self.assertIn('Combined Group 1',document[1].get_text())
                self.assertIn('Combined Group 4',document[4].get_text())
                self.assertIn('Measurement not available',document[5].get_text())
            result=path.with_name(path.stem+'_evaluation.json')
            self.assertEqual(json.loads(result.read_text())['result'],'NOT EVALUATED')
            pid.report(path,str(validation))
            self.assertEqual(json.loads(result.read_text())['result'],'PASS')
            pid.report(path,str(validation),'feedback lost')
            self.assertIn('INTERRUPTED',json.loads(result.read_text())['result'])

if __name__=='__main__':unittest.main()
