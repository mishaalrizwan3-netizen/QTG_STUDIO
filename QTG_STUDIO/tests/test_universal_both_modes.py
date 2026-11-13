"""Synthetic reviewed plans and mocked workers only. Never opens simulator I/O."""
import csv
import hashlib
import json
import os
from pathlib import Path
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import qtg_common_engine as engine
from backend import engine_initialization_adapter as integration
from backend.initialization_configuration import (AUTOMATIC_PLANS, MANUAL_PLANS,
    ENGINE_MANDATORY, automatic_plans)
from backend.initialization_plan_safety import PlanBlocked
from backend.initialization_reference import capture_reference
from backend.initialization_requirements import REQUIREMENTS
from backend.universal_initialization import InitializationManager, State
import test_engine_universal_initialization as fixtures


class BothModesTests(unittest.TestCase):
    def setUp(self):
        self.env = patch.dict(os.environ, {AUTOMATIC_PLANS:'{}', MANUAL_PLANS:'{}'})
        self.env.start(); self.addCleanup(self.env.stop)
        self.f = fixtures.EngineUniversalTests(); self.f.setUp()
        self.addCleanup(self.f.doCleanups)
        self.w = self.f.worker

    def automatic(self, method='replay', reference='1.c.(1)'):
        self.f.configure('1.f.(1)', 'auto')
        for flag in integration.ENV.values(): os.environ[flag] = '0'
        context = self.w._session_context
        context.update(test_reference=reference, control_method=method,
                       configuration_revision='reviewed-reference-revision')
        self.w._control_method = method
        self.w._base_module.STAGE_CONNECTED_READY = 'Ready'
        mode = 'pid' if method == 'pid' else 'recorded_replay'
        self.f.document.update(test_reference=reference, mode=mode)
        if reference not in integration.ENV:
            self.f.document['automatic_policy'] = dict(status='CONFIGURED',
                preparation='existing_climb_pid' if method == 'pid' else 'existing_recorded_replay',
                stability_scope='paused_state', source='Synthetic offline policy only',
                mandatory_checks=sorted(ENGINE_MANDATORY))
        # Criteria carry no second copy of the recorded targets.
        for spec in self.f.document['checks'].values():
            spec.pop('value', None); spec.pop('value_source', None)
            spec['target_source'] = 'manual_recording_t0'
        self.f.write_document()
        os.environ[AUTOMATIC_PLANS] = json.dumps({reference:{mode:str(self.f.plan_path)}})

    def start(self):
        with patch.object(integration, 'InitializationManager', side_effect=lambda a, **kw:
                InitializationManager(a, clock=lambda:self.f.now, **kw)), \
                patch.object(engine, '_initialize_mode') as dispatch:
            engine.initialize_mode(self.w, self.w._session_context['mode'])
        dispatch.assert_called_once()
        adapter = self.w._engine_initialization_adapter
        adapter.clock = lambda:self.f.now
        adapter.thread = Mock(is_alive=Mock(return_value=False))
        return adapter

    def ready(self):
        adapter = self.start()
        for sequence in (1, 2, 3):
            self.f.now += .1; adapter._frame=self.f.frame(sequence); adapter.physical={}
            if sequence == 1:
                self.w._initialization_stage(self.w._initialization_generation, 'Ready')
            else: self.w._poll_universal_initialization()
        self.assertEqual(self.w._universal_manager.state, State.READY)
        return adapter

    def change_reference(self, **values):
        path = Path(self.w._session_context['reference_csv'])
        sidecar = path.with_name(path.stem+'_init_conditions.json')
        snapshot = json.loads(sidecar.read_text())
        snapshot['manual'].update(values)
        sidecar.write_text(json.dumps(snapshot))
        with path.open(newline='') as stream:
            reader=csv.DictReader(stream); names=reader.fieldnames; rows=list(reader)
        columns={'airspeed':'Indicated Airspeed (kts)', 'altitude_msl':'Altitude MSL (ft)',
                 'throttle':'Throttle 1 Position (%)'}
        for key, value in values.items():
            if key in columns: rows[0][columns[key]]=value
        with path.open('w',newline='') as stream:
            writer=csv.DictWriter(stream,fieldnames=names);writer.writeheader();writer.writerows(rows)
        self.f.document['reference_identity']=dict(csv_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
            sidecar_sha256=hashlib.sha256(sidecar.read_bytes()).hexdigest())
        self.f.write_document()

    def test_manual_and_replay_and_pid_use_one_manager_class(self):
        self.f.ready()
        self.assertIsInstance(self.w._universal_manager, InitializationManager)
        for method in ('replay', 'pid'):
            self.automatic(method)
            self.ready()
            self.assertIsInstance(self.w._universal_manager, InitializationManager)
            self.assertEqual(REQUIREMENTS['1.c.(1)'].status, 'UNCONFIGURED')

    def test_different_recordings_change_automatic_targets_not_gui(self):
        self.automatic()
        self.w._session_context['initial_conditions']['ias_kt']=300.
        first=integration.load_plan(self.w._session_context,self.f.plan_path)
        self.change_reference(airspeed=77.25)
        second=integration.load_plan(self.w._session_context,self.f.plan_path)
        self.assertEqual(first.checks['ias_kt'].value,80.)
        self.assertEqual(second.checks['ias_kt'].value,77.25)
        self.assertNotEqual(first.binding,second.binding)

    def test_missing_pid_reference_blocks_without_dispatch_or_gui_fallback(self):
        self.automatic('pid');self.w._session_context['reference_csv']=None
        with patch.object(engine,'_initialize_mode') as dispatch:
            engine.initialize_mode(self.w,'auto')
        dispatch.assert_not_called()
        self.assertIn('Select a Manual reference', self.w._stage_details)
        self.w.arm_recording.assert_not_called()

    def test_stale_sidecar_and_changed_reference_revoke_readiness(self):
        self.automatic();adapter=self.ready()
        self.change_reference(airspeed=76.)
        with self.assertRaisesRegex(PlanBlocked,'changed'):
            adapter.check_integrity()
        self.w.handle_command({'action':'record','configuration_revision':'reviewed-reference-revision'})
        self.w.arm_recording.assert_not_called()
        self.assertEqual(self.w._universal_manager.state,State.BLOCKED)

    def test_reference_selection_gui_invalidation_cancels_automatic(self):
        self.automatic();self.ready()
        self.w.handle_command({'action':'invalidate_initialization',
            'configuration_revision':'reviewed-reference-revision'})
        self.assertEqual(self.w._universal_manager.state,State.CANCELLED)
        self.w.arm_recording.assert_not_called()

    def test_reference_snapshot_is_immutable_and_zeros_are_preserved(self):
        self.f.configure('1.f.(2)','auto')
        captured=capture_reference(self.w._session_context['reference_csv'],ENGINE_MANDATORY)
        self.assertEqual(captured.conditions()['ias_kt'],0.)
        with self.assertRaises(TypeError):captured.values['ias_kt']=9.
        copy=captured.snapshot();copy['rpm']=42.
        self.assertEqual(captured.snapshot()['rpm'],2000.)

    def test_csv_sidecar_mismatch_is_not_resolved_using_a_later_event(self):
        self.automatic();self.change_reference(rpm=999.)
        with self.assertRaisesRegex(PlanBlocked,'inconsistent sidecar'):
            integration.load_plan(self.w._session_context,self.f.plan_path)

    def test_missing_genuine_flaps_never_uses_legacy_ground_alias(self):
        self.automatic()
        path=Path(self.w._session_context['reference_csv']).with_name('selected_manual_init_conditions.json')
        data=json.loads(path.read_text());data['manual']['telemetry'].pop('measured_flaps_handle_index')
        data['manual']['telemetry']['flaps_handle_index']=0
        path.write_text(json.dumps(data))
        with self.assertRaisesRegex(PlanBlocked,'flaps: missing'):
            capture_reference(self.w._session_context['reference_csv'],ENGINE_MANDATORY)

    def test_first_invalid_telemetry_row_is_not_silently_retimed(self):
        self.automatic()
        path=Path(self.w._session_context['reference_csv'])
        path.write_text(path.read_text().replace(',80.0,',',nan,',1))
        with self.assertRaisesRegex(PlanBlocked,'leading invalid samples'):
            capture_reference(path,ENGINE_MANDATORY)

    def test_replay_engine_rpm_tolerance_is_retained(self):
        self.automatic(reference='1.f.(2)')
        # This fixture is airborne; the target comes from its recording, never a Manual default.
        self.f.document['checks']['engine_rpm']['tolerance']=60.;self.f.write_document()
        plan=integration.load_plan(self.w._session_context,self.f.plan_path)
        self.assertEqual(plan.checks['engine_rpm'].tolerance,60.)
        self.f.document['checks']['engine_rpm']['tolerance']=61.;self.f.write_document()
        with self.assertRaisesRegex(PlanBlocked,'existing test-specific tolerance'):
            integration.load_plan(self.w._session_context,self.f.plan_path)

    def test_reviewed_manual_can_exclude_both_paused_rpm_targets(self):
        for key in ('engine_rpm','prop_rpm'):
            self.f.document['checks'].pop(key)
            self.f.document['exclusions'][key]='Synthetic reviewed paused-phase nonapplicability'
        self.f.write_document()
        self.f.ready()
        self.assertNotIn('engine_rpm',self.w._universal_plan.checks)
        self.assertNotIn('prop_rpm',self.w._universal_plan.checks)

    def test_auto_opt_in_removal_cannot_fall_back_to_legacy(self):
        self.automatic();self.ready();os.environ[AUTOMATIC_PLANS]='{}'
        self.assertTrue(engine.universal_initialization_enabled(self.w))
        self.w.handle_command({'action':'record','configuration_revision':'reviewed-reference-revision'})
        self.w.arm_recording.assert_not_called()
        self.assertEqual(self.w._universal_manager.state,State.BLOCKED)

    def test_pid_handoff_requires_additional_fresh_frame_then_internal_authorization(self):
        self.automatic('pid');adapter=self.ready();manager=self.w._universal_manager
        self.assertFalse(adapter.execution_allowed.is_set())
        with self.assertRaisesRegex(RuntimeError,'authorization'):
            integration.authorize_pid_recording(self.w)
        self.f.now+=.1;adapter._frame=self.f.frame(4)
        self.w._poll_universal_initialization()
        self.assertEqual(manager.state,State.EXECUTING)
        self.assertTrue(adapter.execution_allowed.is_set())
        self.assertTrue(adapter.cancelled.is_set())  # No target polling in PID flight.
        self.w.arm_recording.assert_not_called()
        integration.authorize_pid_recording(self.w)
        self.assertEqual(manager.state,State.RECORDING)

    def test_cancel_or_reference_edit_during_pid_stabilization_blocks_recording(self):
        self.automatic('pid');adapter=self.ready()
        self.f.now+=.1;adapter._frame=self.f.frame(4);self.w._poll_universal_initialization()
        self.w.handle_command({'action':'invalidate_initialization',
            'configuration_revision':'reviewed-reference-revision'})
        with self.assertRaises(RuntimeError):integration.authorize_pid_recording(self.w)
        self.assertEqual(self.w._universal_manager.state,State.CANCELLED)

    def test_changed_reference_file_after_pid_handoff_blocks_internal_recording(self):
        self.automatic('pid');adapter=self.ready()
        self.f.now+=.1;adapter._frame=self.f.frame(4);self.w._poll_universal_initialization()
        self.change_reference(airspeed=76.)
        with self.assertRaises(PlanBlocked):integration.authorize_pid_recording(self.w)

    def test_pid_pause_verification_does_not_impose_replay_physical_alignment(self):
        self.automatic('pid');adapter=self.start()
        adapter.reader=Mock(issues=[]);adapter.reader.feedback.return_value=self.f.frame(1)
        adapter.reader.native_timing={'running':False}
        self.w.brunner._io_lock=threading.RLock();self.w.brunner.read_axis.return_value=(.2,0.,0.)
        _,physical=adapter._sample()
        self.assertEqual(physical['pitch']['value'],.2)
        self.w.brunner.move_actuators.assert_not_called()

    def test_pid_reference_application_uses_existing_interfaces_and_awaits_manager(self):
        self.automatic('pid');adapter=self.start()
        self.w._restore_reference_weather=Mock();self.w._restore_reference_state=Mock()
        with patch('backend.initializer.apply_manual_conditions',return_value={'measured':True}) as apply:
            adapter.prepare_conditions()
        self.assertEqual(apply.call_args.args[1]['ias_kt'],80.)
        self.w._restore_reference_weather.assert_called_once()
        self.w._restore_reference_state.assert_called_once()
        self.assertFalse(adapter.execution_allowed.is_set())
        self.w.brunner.move_actuators.assert_not_called()

    def test_pid_wait_can_be_cancelled_and_cannot_release_itself(self):
        self.automatic('pid');adapter=self.start()
        self.w.stop_worker_event.set()
        with self.assertRaisesRegex(RuntimeError,'cancelled'):adapter.wait_for_execution()
        self.assertFalse(adapter.execution_allowed.is_set())

    def test_bad_automatic_mapping_and_unsupported_pid_test_are_blocked(self):
        for mapping in ('bad json', '{"1.f.(2)":{"pid":"some-plan"}}'):
            os.environ[AUTOMATIC_PLANS]=mapping
            with self.assertRaises(PlanBlocked):automatic_plans()

    def test_generic_automatic_policies_require_explicit_review(self):
        self.automatic();self.f.document.pop('automatic_policy');self.f.write_document()
        with self.assertRaisesRegex(PlanBlocked,'reviewed Automatic policy'):
            integration.load_plan(self.w._session_context,self.f.plan_path)

    def test_all_automatic_routes_are_off_without_explicit_selection(self):
        for flag in integration.ENV.values():os.environ[flag]='0'
        for ref in REQUIREMENTS:
            for method in ('replay','pid'):
                self.assertFalse(integration.requested(dict(test_reference=ref,mode='auto',control_method=method)))

    def test_initialization_only_lock_cannot_release_pid(self):
        self.automatic('pid');adapter=self.ready()
        os.environ[integration.INITIALIZATION_ONLY]='1'
        self.f.now+=.1;adapter._frame=self.f.frame(4);self.w._poll_universal_initialization()
        self.assertFalse(adapter.execution_allowed.is_set())
        self.w.arm_recording.assert_not_called()

    def test_enabled_gui_pid_route_reuses_reference_selector(self):
        import main
        self.automatic('pid')
        spec=SimpleNamespace(reference='1.c.(1)',title='Normal climb',pdf_graphs=[])
        values={'Flt cond':'Climb','Alt[ft]':'9000','IAS[kts]':'95','Latitude[deg]':'32',
                'Longitude[deg]':'74','Heading[deg]':'0','Pitch[deg]':'0','Bank[deg]':'0',
                'Flaps':'0','Throttle[%]':'100','Temperature[C]':'15'}
        fields={key:Mock(get=lambda value=value:value) for key,value in values.items()}
        host=SimpleNamespace(_common_duration_var=Mock(get=lambda:'10'),
            _automatic_methods={spec.reference:Mock(get=lambda:'PID Automatic')},
            _test_settings={spec.reference:(fields,{})},_apply_test_settings=lambda spec:True,
            _pid_validation={spec.reference:Mock(get=lambda:'')},_common_process=None,
            _common_reference_var=Mock(get=lambda:self.w._session_context['reference_csv']),
            _manual_condition_fingerprint=Mock(return_value='snapshot-fingerprint'),
            common_record_button=Mock(),common_status=Mock(),after=Mock(),_poll_common_test=Mock())
        with patch.object(engine,'launch',return_value=(Mock(),{})) as launch:
            main.App._launch_common_test(host,'auto',spec)
        self.assertEqual(Path(launch.call_args.args[3]).resolve(),
                         Path(self.w._session_context['reference_csv']).resolve())
        self.assertEqual(launch.call_args.kwargs['control_method'],'pid')
        self.assertIn('configuration_revision',launch.call_args.kwargs)
        host._common_reference_var=Mock(get=lambda:'')
        host._common_process=None
        with patch.object(engine,'launch') as launch:
            main.App._launch_common_test(host,'auto',spec)
        launch.assert_not_called()

    def test_pid_preparation_waits_before_entering_unchanged_controller(self):
        import qtg_climb_pid as pid
        self.automatic('pid');adapter=self.start()
        context=self.w._session_context
        context['initial_conditions']=adapter.reviewed_inputs.reference_snapshot.conditions()
        self.w.target_duration_sec=10
        for name in ('rec_time','p_pos_buf','r_pos_buf','y_pos_buf','p_force_buf','r_force_buf',
                     'y_force_buf','p3d_tele_buf','applied_force_buf','virtual_force_buf','replay_command_buf'):
            setattr(self.w,name,[])
        self.w._base_module.STAGE_INITIALIZING='Initializing'
        adapter.prepare_conditions=Mock(return_value={'pressure_alt':2000.})
        adapter.wait_for_execution=Mock(side_effect=lambda:self.assertFalse(self.w.pid_controller_active))
        class Immediate:
            def __init__(self,target,**kwargs):self.target=target
            def start(self):self.target()
        with patch.object(pid.threading,'Thread',Immediate),patch.object(pid,'watchdog'), \
                patch.object(pid,'loop') as controller,patch.object(engine,'apply_manual_conditions') as gui_apply:
            pid.prepare(self.w,self.w._base_module,context)
        gui_apply.assert_not_called()
        adapter.prepare_conditions.assert_called_once()
        adapter.wait_for_execution.assert_called_once()
        controller.assert_called_once()
        self.assertEqual(self.w.pid_conditions['target_ias'],80.)
        self.w.brunner.configure_pid_mode.assert_not_called()

    def test_actual_pid_loop_cannot_record_without_manager_authorization(self):
        import test_climb_pid
        for permit in (False,True):
            guard=Mock() if permit else Mock(side_effect=RuntimeError('Universal authorization denied'))
            with patch.object(engine,'universal_initialization_enabled',return_value=True), \
                    patch.object(integration,'authorize_pid_recording',guard):
                window=test_climb_pid.ClimbPidTests().run_automatic_fixture()
            guard.assert_called_once()
            if permit:
                self.assertTrue(window.rec_time)
                self.assertGreaterEqual(window.pid_stabilization['elapsed_s'],5.)
            else:
                self.assertFalse(window.rec_time)
                self.assertIn('Universal authorization denied',window.pid_fault)
            window.brunner.release_controls.assert_called_once()

    def test_inherited_record_button_cannot_bypass_manager(self):
        base=SimpleNamespace(SuperMushakLoggerApp=type('Base',(),{'arm_recording':Mock()}))
        cls=engine.worker_class(base,{},lambda value:None)
        window=object.__new__(cls)
        window._session_context={'universal_initialization_enabled':True}
        window.stop_worker_event=threading.Event();window._publish=Mock()
        window._universal_attempt_id='current'
        manager=SimpleNamespace(state=State.READY,attempt_id='current',_plan=SimpleNamespace(mode='manual'))
        window._universal_manager=manager;window._universal_recording_authorized=False
        window.arm_recording()
        base.SuperMushakLoggerApp.arm_recording.assert_not_called()
        window._universal_recording_authorized=True
        manager.state=State.RECORDING
        window.arm_recording()
        base.SuperMushakLoggerApp.arm_recording.assert_called_once()
        base.SuperMushakLoggerApp.arm_recording.reset_mock()
        manager._plan.mode='pid'
        window.arm_recording()
        base.SuperMushakLoggerApp.arm_recording.assert_not_called()
        window._session_context={}
        window.arm_recording()
        base.SuperMushakLoggerApp.arm_recording.assert_called_once()


if __name__ == '__main__':
    unittest.main()
