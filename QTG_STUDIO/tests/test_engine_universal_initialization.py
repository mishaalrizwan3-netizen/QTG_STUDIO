"""Engine manager integration with mocked feedback/dispatch; never opens hardware."""
import csv
import hashlib
from dataclasses import replace
import json
import os
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import Mock, patch

import qtg_common_engine as engine
from backend import engine_initialization_adapter as integration
from backend.initialization_requirements import REQUIREMENTS
from backend.universal_initialization import Feedback, Measurement, InitializationManager, State
from backend.telemetry_verification import TelemetryUnavailable
import test_initialization_compatibility as compatibility
from _initialization_plan_fixtures import approve_document


class EngineUniversalTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        folder = patch('backend.deceleration_config._folder', return_value=self.root)
        folder.start(); self.addCleanup(folder.stop)
        self.now = 100.
        self.worker = compatibility.InitializationCompatibilityTests().worker()
        self.worker.sig_update_stage = Mock()
        self.worker._base_module = type('Base', (), {'STAGE_AUTO_BLOCKED':'Blocked'})
        self.worker.stop_test = Mock(side_effect=self.worker.stop_worker_event.set)
        self.environment = patch.dict(os.environ, {value:'0' for value in integration.ENV.values()})
        self.environment.start()
        self.addCleanup(self.environment.stop)
        os.environ[integration.INITIALIZATION_ONLY]='0'
        os.environ['QTG_DECELERATION_BRAKE_CAPTURE']='0'
        self.configure('1.f.(2)', 'manual')

    def configure(self, reference, mode):
        ground = reference == '1.f.(2)'
        cfg = dict(alt_ft=755. if ground else 2000., ias_kt=0. if ground else 80.,
                   throttle=100. if ground else 5., flaps=0., on_ground=ground,
                   lat=32., lon=74., pitch_deg=0., bank_deg=0., heading_deg=0., temperature_c=15.)
        snapshot = dict(altitude_msl=cfg['alt_ft'], airspeed=cfg['ias_kt'], throttle=cfg['throttle'],
                        lat=32.,lon=74.,heading=0.,pitch_ang=0.,bank_ang=0.,
                        rpm=2000., pitch_pos=0., roll_pos=0., rudder_pos=0.,
                        telemetry=dict(sim_on_ground=float(ground), measured_flaps_handle_index=0.,
                            ambient_temp=15., ambient_wind_spd=0., ambient_wind_dir=0., baro_pressure=1013.,
                            prop_rpms=[2000.], engine_rpm=2000.))
        selected = self.root / 'selected_manual.csv'
        row = {'Time (s)':0, 'Pitch_Manual_Pos':0, 'Roll_Manual_Pos':0, 'Rudder_Manual_Pos':0,
               'Throttle 1 Position (%)':cfg['throttle'], 'Indicated Airspeed (kts)':cfg['ias_kt'],
               'Altitude MSL (ft)':cfg['alt_ft']}
        with selected.open('w', newline='') as stream:
            writer=csv.DictWriter(stream,fieldnames=list(row));writer.writeheader();writer.writerow(row)
            writer.writerow(dict(row, **{'Time (s)':1}))
        selected.with_name(selected.stem+'_init_conditions.json').write_text(json.dumps({'manual':snapshot}))
        context = dict(test_reference=reference, mode=mode, control_method='replay',
                       initial_conditions=cfg, reference_csv=str(selected) if mode=='auto' else None)
        (self.root/'initialization.json').write_text(json.dumps(dict(cfg, test_reference='1.f.(2)', flight_condition='Ground' if ground else 'Airborne')))
        keys = {'on_ground','alt_ft','ias_kt','throttle','flaps','temperature_c','wind_speed','wind_direction','barometer'}
        keys |= {'lat','lon','heading_deg','pitch_deg','bank_deg'}
        keys |= {'engine_rpm','prop_rpm'}
        values = dict(cfg, wind_speed=0.,wind_direction=0.,barometer=1013.,engine_rpm=2000.,prop_rpm=2000.)
        specs = {k:dict(value=values[k], value_source='Explicit synthetic test data; not a live target', tolerance=(REQUIREMENTS[reference].requirements[k].tolerance
                    if REQUIREMENTS[reference].requirements[k].tolerance is not None else 0.),
                    tolerance_source='Explicit synthetic offline fixture; not live review') for k in keys}
        if mode=='manual':specs['engine_rpm']['tolerance']=7.  # Synthetic Manual band, not Replay's 60/400.
        if mode=='manual':
            # Targets exist only in the same configuration used by the worker.
            configured=dict(cfg, **{k:v for k,v in values.items() if k not in cfg})
            if reference=='1.f.(2)':
                (self.root/'initialization.json').write_text(json.dumps(dict(configured,test_reference='1.f.(2)',flight_condition='Ground')))
            else:
                context['initial_conditions'] = configured
            for spec in specs.values():
                spec.pop('value');spec.pop('value_source')
                spec['target_source']='resolved_initial_conditions'
        self.document = dict(test_reference=reference, mode='manual' if mode=='manual' else 'recorded_replay',
            reference_csv=str(selected), checks=specs,
            exclusions={k:'Synthetic test exclusion, not authorization for live use' for k in REQUIREMENTS[reference].requirements if k not in keys},
            review_source='Offline fixture only',timeout_s=10.,max_age_s=1.,stable_samples=3)
        self.plan_path = self.root/'plan.json'
        if mode=='manual':
            self.document['configuration_identity']=integration.configuration_snapshot(context)[1]
        if mode=='auto':
            self.document['reference_identity']=dict(csv_sha256=hashlib.sha256(selected.read_bytes()).hexdigest(),
                sidecar_sha256=hashlib.sha256(selected.with_name(selected.stem+'_init_conditions.json').read_bytes()).hexdigest())
        self.write_document()
        self.worker._session_context=context
        self.worker.ref_manual_snapshot=snapshot
        self.worker._control_method='replay'
        self.worker.stop_worker_event.clear()
        os.environ[integration.ENV[reference]]='1'
        os.environ[integration.ENV[reference]+'_PLAN']=str(self.plan_path)
        self.worker._initialization_generation=getattr(self.worker,'_initialization_generation',0)
        def begin():
            self.worker._initialization_generation+=1
            self.worker._prepared=self.worker._preparing=False
            integration.begin_attempt(self.worker)
        self.worker._begin_initialization=Mock(side_effect=begin)

    def write_document(self):
        self.plan_path.write_text(json.dumps(approve_document(self.document)))

    def start(self):
        manager_type = InitializationManager
        with patch.object(integration,'InitializationManager',side_effect=lambda a:manager_type(a,clock=lambda:self.now)), \
                patch.object(engine,'_initialize_mode') as dispatch:
            engine.initialize_mode(self.worker,self.worker._session_context['mode'],self.worker._session_context.get('reference_csv'))
        dispatch.assert_called_once_with(self.worker,self.worker._session_context['mode'],self.worker._session_context.get('reference_csv'))
        adapter=self.worker._engine_initialization_adapter
        adapter.clock=lambda:self.now
        adapter.thread=Mock(is_alive=Mock(return_value=False))  # Supply asynchronous sampler results explicitly.
        return adapter

    def frame(self, sequence, **changes):
        plan=self.worker._universal_plan
        self.worker._engine_initialization_adapter.native_timing = dict(sequence=sequence, running=False,
            generation=1,request_id=sequence,provenance='Synthetic correlated native fixture')
        values={k:Measurement(c.value,c.unit,c.telemetry) for k,c in plan.checks.items()}
        for k,value in changes.items():values[k]=replace(values[k],value=value)
        return Feedback(True,sequence,self.now,values)

    def ready(self):
        adapter=self.start()
        for seq in (1,2,3):
            self.now+=.1
            adapter._frame=self.frame(seq);adapter.physical={}
            if seq==1:self.worker._initialization_stage(self.worker._initialization_generation,'Ready')
            else:self.worker._poll_universal_initialization()
        self.assertTrue(self.worker._prepared)
        return adapter

    def test_both_engines_and_modes_dispatch_once_and_authorize_after_three_frames(self):
        for reference in ('1.f.(2)','1.f.(1)'):
            for mode in ('manual','auto'):
                with self.subTest(reference=reference,mode=mode):
                    self.configure(reference,mode)
                    self.worker._universal_recording_authorized=False
                    adapter=self.ready()
                    record=self.worker._session_context['initialization_attempt']
                    self.assertEqual(record['attempt_id'],self.worker._universal_attempt_id)
                    self.assertEqual(record['selected_reference'],self.worker._session_context['reference_csv'])
                    self.now+=.1;adapter._frame=self.frame(4)
                    self.worker.handle_command({'action':'record'})
                    self.assertTrue(adapter.cancelled.is_set())
                    self.assertEqual(record['status'],'RECORDING')
                    self.assertEqual(self.worker._universal_manager.state,State.RECORDING)
        self.assertEqual(self.worker.arm_recording.call_count,4)

    def test_no_initialization_feedback_or_commands_after_recording_handoff(self):
        adapter=self.ready();self.now+=.1;adapter._frame=self.frame(4)
        self.worker.handle_command({'action':'record'})
        adapter.feedback=Mock(side_effect=AssertionError('initialization must be stopped'))
        adapter.dispatch=Mock(side_effect=AssertionError('no repeated preparation'))
        self.worker.latest_telemetry={'airspeed_indicated':150.,'pressure_alt':1200.,'pitch_angle':20.}
        self.worker._poll_universal_initialization()
        self.worker.handle_command({'action':'record'})
        adapter.feedback.assert_not_called();adapter.dispatch.assert_not_called()
        self.worker.arm_recording.assert_called_once()
        integration.mark(self.worker,'COMPLETE')
        self.assertEqual(self.worker._session_context['initialization_attempt']['status'],'COMPLETE')
        self.assertEqual(self.worker._universal_manager.state,State.COMPLETE)

    def test_manager_itself_never_polls_initialization_feedback_while_recording(self):
        adapter=self.ready();self.now+=.1;adapter._frame=self.frame(4)
        self.worker.handle_command({'action':'record'})
        adapter.feedback=Mock(side_effect=AssertionError('no target monitoring during manoeuvre'))
        manager=self.worker._universal_manager
        self.assertFalse(manager.poll(manager.attempt_id))
        self.assertFalse(manager.authorize_recording(manager.attempt_id))
        adapter.feedback.assert_not_called()
        self.assertFalse(manager.recording_started(manager.attempt_id))

    def test_completed_attempt_cannot_be_revived_or_changed_by_late_failure(self):
        adapter=self.ready();self.now+=.1;adapter._frame=self.frame(4)
        self.worker.handle_command({'action':'record'})
        manager=self.worker._universal_manager
        self.assertTrue(manager.complete(manager.attempt_id))
        self.assertFalse(manager.fail(manager.attempt_id,'old callback'))
        self.assertFalse(manager.prepared(manager.attempt_id))
        self.assertEqual(manager.state,State.COMPLETE)

    def test_pending_read_cannot_authorize_and_timeout_cleans_through_legacy_owner(self):
        self.start();self.worker._initialization_stage(self.worker._initialization_generation,'Ready')
        self.worker.handle_command({'action':'record'})
        self.worker.arm_recording.assert_not_called()
        self.now+=11.;self.worker._poll_universal_initialization()
        self.assertEqual(self.worker._universal_manager.state,State.FAILED)
        self.worker.stop_test.assert_called_once()

    def test_late_ready_after_stop_is_ignored(self):
        self.start();generation=self.worker._initialization_generation
        self.worker.handle_command({'action':'stop'})
        self.worker._initialization_stage(generation,'Ready')
        self.assertEqual(self.worker._universal_manager.state,State.CANCELLED)
        self.worker.handle_command({'action':'record'})
        self.worker.arm_recording.assert_not_called()

    def test_weather_mismatch_fails_without_fallback(self):
        adapter=self.start();adapter._frame=self.frame(1,temperature_c=39.7);adapter.physical={}
        self.worker._initialization_stage(self.worker._initialization_generation,'Ready')
        self.assertEqual(self.worker._universal_manager.state,State.FAILED)
        self.assertIn('temperature_c',self.worker._stage_details)
        record=self.worker._session_context['initialization_attempt']
        self.assertEqual(record['verified_measurements'],{})
        self.assertEqual(record['observed_measurements']['temperature_c']['value'],39.7)
        self.worker.arm_recording.assert_not_called()

    def test_native_timing_and_missing_ground_measurement_are_explicit_gaps(self):
        adapter=self.start()
        adapter.reader=Mock(issues=['P3D timestamp is Python cache-copy time, not native acquisition time'])
        adapter.reader.native_timing={'running':False}
        adapter.reader.feedback.return_value=self.frame(1)
        with self.assertRaisesRegex(TelemetryUnavailable,'cache-copy time'):adapter._sample()
        adapter.reader.issues=[]
        frame=self.frame(2);adapter.reader.feedback.return_value=replace(frame,values={})
        with self.assertRaisesRegex(TelemetryUnavailable,'on_ground: authoritative telemetry unavailable'):adapter._sample()
        self.worker.brunner.read_axis.assert_not_called()

    def test_real_read_path_uses_existing_client_and_separate_physical_positions(self):
        adapter=self.start()
        adapter.reader=Mock(issues=[]);adapter.reader.feedback.return_value=self.frame(1)
        adapter.reader.native_timing={'running':False}
        device=self.worker.brunner;device._io_lock=threading.RLock();device.strict_io=False
        device.read_axis.return_value=(.01,0.,0.)
        frame,physical=adapter._sample()
        self.assertEqual(set(physical),{'pitch','roll','rudder'})
        self.assertEqual(frame.sequence,1)
        self.assertFalse(device.strict_io)
        device.move_actuators.assert_not_called();device.release_controls.assert_not_called()
        adapter.cleanup(adapter.attempt_id)
        self.worker.p3d.close.assert_not_called()

    def test_physical_read_failure_is_visible_and_restores_io_policy(self):
        adapter=self.start();adapter.reader=Mock(issues=[])
        adapter.reader.native_timing={'running':False}
        adapter.reader.feedback.return_value=self.frame(1)
        device=self.worker.brunner;device._io_lock=threading.RLock();device.strict_io=False
        device.read_axis.side_effect=RuntimeError('No physical position reply')
        with self.assertRaisesRegex(RuntimeError,'No physical position reply'):adapter._sample()
        self.assertFalse(device.strict_io)
        device.move_actuators.assert_not_called()

    def test_automatic_physical_misalignment_is_not_corrected_by_adapter(self):
        self.configure('1.f.(1)','auto');adapter=self.start()
        adapter.reader=Mock(issues=[]);adapter.reader.feedback.return_value=self.frame(1)
        adapter.reader.native_timing={'running':False}
        device=self.worker.brunner;device._io_lock=threading.RLock();device.strict_io=False
        device.read_axis.return_value=(.1,0.,0.)
        with self.assertRaisesRegex(RuntimeError,'differs from selected Manual'):adapter._sample()
        device.move_actuators.assert_not_called();device.configure_replay_mode.assert_not_called()

    def test_existing_automatic_failure_cleanup_is_not_repeated(self):
        self.configure('1.f.(2)','auto');adapter=self.start()
        adapter.note_legacy_cleanup()
        self.worker._initialization_stage(self.worker._initialization_generation,'Failed','existing preparation failure')
        self.assertEqual(self.worker._universal_manager.state,State.FAILED)
        self.worker.stop_test.assert_not_called()
        self.worker.brunner.release_controls.assert_not_called()

    def test_brunnner_controller_ownership_conflict_prevents_dispatch(self):
        self.worker.pid_controller_active=True
        with patch.object(engine,'_initialize_mode') as dispatch:
            engine.initialize_mode(self.worker,'manual')
        dispatch.assert_not_called()
        self.assertEqual(self.worker._universal_manager.state,State.FAILED)
        self.assertIn('ownership conflict',self.worker._stage_details)
        self.worker.stop_test.assert_not_called()
        self.worker.brunner.release_controls.assert_not_called()

    def test_stale_sampler_feedback_fails(self):
        adapter=self.start();adapter._frame=replace(self.frame(1),measured_at=self.now-2);adapter.physical={}
        self.worker._initialization_stage(self.worker._initialization_generation,'Ready')
        self.assertEqual(self.worker._universal_manager.state,State.FAILED)

    def test_ready_expires_if_sampler_stops_producing_new_frames(self):
        self.ready();self.now+=1.1
        self.worker._poll_universal_initialization()
        self.assertEqual(self.worker._universal_manager.state,State.FAILED)
        self.assertFalse(self.worker._prepared)

    def test_snapshot_expiring_during_handoff_blocks_recording(self):
        adapter=self.ready();self.now+=.1;adapter._frame=self.frame(4)
        adapter.thread.join.side_effect=lambda **kwargs:setattr(self,'now',self.now+1.1)
        self.worker.handle_command({'action':'record'})
        self.worker.arm_recording.assert_not_called()
        self.assertEqual(self.worker._universal_manager.state,State.FAILED)
        self.assertIn('expired during recording handoff',self.worker._stage_details)

    def test_attempt_metadata_clears_manual_ready_and_does_not_reuse_attempt_id(self):
        self.worker._session_context['manual_airflow_preparation']={'status':'READY'}
        self.worker._session_context['weather_diagnostics']=[{'status':'MATCHED'}]
        integration.begin_attempt(self.worker)
        first=self.worker._session_context['initialization_attempt']['attempt_id']
        self.assertNotIn('manual_airflow_preparation',self.worker._session_context)
        self.worker._session_context.update(mode='auto',reference_csv='different.csv')
        integration.begin_attempt(self.worker)
        record=self.worker._session_context['initialization_attempt']
        self.assertNotEqual(first,record['attempt_id']);self.assertEqual(record['selected_reference'],'different.csv')
        self.assertEqual(record['verified_measurements'],{})

    def test_actual_worker_begin_hook_isolates_metadata_and_queued_generation(self):
        self.worker._session_context['manual_airflow_preparation']={'status':'READY'}
        old=self.worker._initialization_generation
        type(self.worker)._begin_initialization(self.worker)
        self.assertNotIn('manual_airflow_preparation',self.worker._session_context)
        self.assertEqual(self.worker._initialization_generation,old+1)
        self.worker.sig_update_stage.connect.assert_called_once()

    def test_sampler_is_pending_then_stops_before_handoff(self):
        adapter=self.start();adapter.thread=None
        entered=threading.Event();release=threading.Event()
        def sample():
            entered.set();release.wait(timeout=2.)
            return self.frame(1),{}
        adapter._sample=Mock(side_effect=sample)
        try:
            self.assertIsNone(adapter.feedback(adapter.attempt_id))
            self.assertTrue(entered.wait(timeout=1.))
        finally:
            release.set();adapter.handoff()
        self.assertFalse(adapter.thread.is_alive())
        with self.assertRaisesRegex(RuntimeError,'cancelled'):
            adapter.feedback(adapter.attempt_id)

    def test_duplicate_preparation_and_reader_that_cannot_stop_are_blocked(self):
        adapter=self.start()
        with self.assertRaisesRegex(RuntimeError,'already dispatched'):
            adapter.prepare(adapter.attempt_id,self.worker._universal_plan)
        self.now+=.1
        adapter._frame=self.frame(1);adapter.physical={}
        self.worker._initialization_stage(self.worker._initialization_generation,'Ready')
        adapter.thread.is_alive.return_value=True
        with self.assertRaisesRegex(RuntimeError,'has not stopped'):adapter.handoff()

    def test_flags_off_and_other_28_tests_use_legacy_dispatch(self):
        for value in integration.ENV.values():os.environ[value]='0'
        for reference in REQUIREMENTS:
            self.worker._session_context=dict(test_reference=reference,mode='manual')
            with patch.object(engine,'_initialize_mode') as legacy:
                engine.initialize_mode(self.worker,'manual')
                legacy.assert_called_once()
        for value in integration.ENV.values():os.environ[value]='1'
        for reference in set(REQUIREMENTS)-set(integration.ENV):
            for mode in ('manual','auto'):
                self.worker._session_context=dict(test_reference=reference,mode=mode,control_method='replay')
                self.assertFalse(engine.universal_initialization_enabled(self.worker))
        self.worker._session_context=dict(test_reference='1.c.(1)',mode='auto',control_method='pid')
        self.assertFalse(engine.universal_initialization_enabled(self.worker))

    def test_missing_plan_and_wrong_reference_block_before_preparation(self):
        os.environ.pop(integration.ENV['1.f.(2)']+'_PLAN',None)
        with patch.object(engine,'_initialize_mode') as legacy:
            engine.initialize_mode(self.worker,'manual')
            legacy.assert_not_called()
            self.assertEqual(self.worker._session_context['initialization_attempt']['status'],'BLOCKED')
        self.configure('1.f.(1)','auto')
        self.document['reference_csv']=str(self.root/'wrong.csv')
        self.write_document()
        with self.assertRaisesRegex(ValueError,'reference mismatch'):
            integration.load_plan(self.worker._session_context,self.plan_path)

    def test_initialization_only_ready_never_arms_recording_or_pid(self):
        os.environ[integration.INITIALIZATION_ONLY]='1'
        adapter=self.ready()
        for action in ('record','pid_engage','pid_tune'):
            self.worker.handle_command({'action':action})
        self.worker.arm_recording.assert_not_called()
        self.assertFalse(adapter.cancelled.is_set())
        record=self.worker._session_context['initialization_attempt']
        self.assertTrue(record['initialization_only'])
        self.assertEqual(record['selected_system'],'Universal Initialization Manager')
        self.assertTrue(record['preparation_dispatch_started'])
        self.assertTrue(record['preparation_dispatch_returned'])
        self.assertEqual(record['planned_checks']['flaps']['channel'],'measured_flaps_handle_index')
        self.assertEqual(record['planned_checks']['alt_ft']['tolerance'],25.)
        self.assertEqual(record['status'],'READY')
        os.environ[integration.INITIALIZATION_ONLY]='0'
        self.worker.handle_command({'action':'record'})
        self.worker.arm_recording.assert_not_called()  # Latched until worker exit.
        self.worker.handle_command({'action':'stop'})
        self.assertEqual(self.worker._universal_manager.state,State.CANCELLED)

    def test_trial_status_disables_record_action_and_flag_off_preserves_status(self):
        for enabled in (False,True):
            os.environ[integration.INITIALIZATION_ONLY]='1' if enabled else '0'
            status={'can_record':True}
            with patch.object(engine,'snapshot',return_value=status):
                type(self.worker)._publish(self.worker)
            self.assertEqual(status['can_record'],not enabled)
            self.assertEqual(status.get('initialization_only',False),enabled)

    def test_trial_rejects_other_modes_flags_and_legacy_fallback_before_dispatch(self):
        for reference,mode,acceleration,brakes,deceleration in (
                ('1.f.(1)','manual','1','0','1'),
                ('1.f.(2)','auto','0','0','1'),
                ('1.f.(2)','manual','1','0','1'),
                ('1.f.(2)','manual','0','1','1'),
                ('1.f.(2)','manual','0','0','0')):
            with self.subTest(reference=reference,mode=mode,acceleration=acceleration,brakes=brakes,deceleration=deceleration):
                self.configure(reference,mode)
                os.environ[integration.INITIALIZATION_ONLY]='1'
                os.environ[integration.ENV['1.f.(1)']]=acceleration
                os.environ[integration.ENV['1.f.(2)']]=deceleration
                os.environ['QTG_DECELERATION_BRAKE_CAPTURE']=brakes
                with patch.object(engine,'_initialize_mode') as dispatch:
                    with self.assertRaisesRegex(RuntimeError,'Initialization-only trial'):
                        engine.initialize_mode(self.worker,mode)
                dispatch.assert_not_called()
                self.assertFalse(self.worker._prepared)

    def test_final_running_or_unknown_state_fails_before_physical_reads(self):
        adapter=self.start();adapter.reader=Mock(issues=[])
        adapter.reader.feedback.return_value=self.frame(1)
        for state in (True,None):
            adapter.reader.native_timing={'running':state}
            with self.assertRaisesRegex(TelemetryUnavailable,'Final P3D pause not verified'):
                adapter._sample()
        self.worker.brunner.read_axis.assert_not_called()

    def test_pause_metadata_must_belong_to_delivered_frame(self):
        adapter=self.start();adapter._frame=self.frame(1);adapter.physical={}
        adapter.native_timing={'sequence':2,'running':False}
        self.worker._initialization_stage(self.worker._initialization_generation,'Ready')
        self.assertEqual(self.worker._universal_manager.state,State.FAILED)
        self.worker.arm_recording.assert_not_called()

    def test_unpause_after_ready_invalidates_readiness(self):
        adapter=self.ready();self.now+=.1;adapter._frame=self.frame(4)
        adapter.native_timing['running']=True
        self.worker._poll_universal_initialization()
        self.assertFalse(self.worker._prepared)
        self.assertEqual(self.worker._universal_manager.state,State.FAILED)
        self.worker.arm_recording.assert_not_called()

    def test_reader_failure_during_handoff_blocks_recording(self):
        adapter=self.ready();self.now+=.1;adapter._frame=self.frame(4)
        adapter.thread.join.side_effect=lambda **kwargs:setattr(adapter,'_error','connection lost')
        self.worker.handle_command({'action':'record'})
        self.worker.arm_recording.assert_not_called()
        self.assertEqual(self.worker._universal_manager.state,State.FAILED)

    def test_missing_required_checks_and_mismatched_sidecar_are_not_invented(self):
        self.document['checks'].pop('wind_speed')
        self.write_document()
        with self.assertRaisesRegex(ValueError,'Missing required'):integration.load_plan(self.worker._session_context,self.plan_path)
        self.configure('1.f.(2)','auto')
        sidecar=self.root/'selected_manual_init_conditions.json'
        data=json.loads(sidecar.read_text());data['manual']['airspeed']=3.
        sidecar.write_text(json.dumps(data))
        self.document['reference_identity']['sidecar_sha256']=hashlib.sha256(sidecar.read_bytes()).hexdigest()
        self.write_document()
        with self.assertRaisesRegex(ValueError,'recording-start mismatch'):integration.load_plan(self.worker._session_context,self.plan_path)


if __name__=='__main__':unittest.main()
