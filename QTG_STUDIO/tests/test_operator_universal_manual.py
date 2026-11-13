"""Operator-gated Manual initialization; no simulator or hardware connections."""
import os
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from backend import engine_initialization_adapter as integration
from frontend.main import App
from frontend.universal_manual_view import display_state
import test_manual_calm_wind as calm


class AdapterInstallTests(unittest.TestCase):
    def test_installer_uses_bound_calm_conditions(self):
        fixture = calm.CalmWindTests()
        fixture.setUp()
        self.addCleanup(fixture.doCleanups)
        adapter_window = fixture.f.worker
        integration.install(adapter_window, 'manual', None, Mock())
        adapter = adapter_window._engine_initialization_adapter
        self.assertEqual(adapter.reviewed_inputs.conditions['wind_speed'], 0.)
        self.assertNotIn('wind_direction', adapter.policy.mandatory)
        self.assertIn('wind_speed', adapter.policy.mandatory)


class PreflightTests(unittest.TestCase):
    def setUp(self):
        self.window = SimpleNamespace(
            brunner=SimpleNamespace(_pid_transport=None), p3d=Mock(), pid_controller_active=False,
            _external_control_active=False, is_recording=False, _prepared=False,
            _universal_recording_authorized=False, stop_worker_event=threading.Event(),
            _session_context={'operator_initiated_universal_manual':True,
                'initialization_attempt':{}})
        self.dispatch = Mock()
        self.adapter = integration.EngineAdapter(self.window, self.dispatch, 'manual', None)
        self.adapter.reader = Mock(native_timing={'running':False})
        self.adapter._physical_readback = Mock(return_value={'pitch':1,'roll':2,'rudder':3})
        self.adapter._diagnose = Mock()
        self.plan = SimpleNamespace(max_age_s=1., review_source='offline test', exclusions={}, checks={},
                                    approval=SimpleNamespace(identity='offline'))

    def test_paused_connected_preflight_then_dispatch(self):
        self.adapter.prepare('attempt', self.plan)
        self.adapter.reader.feedback.assert_called_once_with('attempt',_authoritative=True)
        self.adapter._physical_readback.assert_called_once()
        self.dispatch.assert_called_once()
        self.assertTrue(self.window._session_context['initialization_attempt']['operator_preflight']['paused'])

    def test_running_or_unknown_blocks_before_dispatch(self):
        for running in (True,None):
            with self.subTest(running=running):
                self.setUp()
                self.adapter.reader.native_timing={'running':running}
                with self.assertRaisesRegex(RuntimeError,'confirmed paused'):
                    self.adapter.prepare('attempt',self.plan)
                self.dispatch.assert_not_called()

    def test_stale_or_disconnected_telemetry_blocks(self):
        self.adapter.reader.feedback.side_effect=RuntimeError('stale or disconnected')
        with self.assertRaisesRegex(RuntimeError,'stale or disconnected'):
            self.adapter.prepare('attempt',self.plan)
        self.dispatch.assert_not_called()

    def test_ownership_or_recording_blocks(self):
        for attr in ('pid_controller_active','_external_control_active','is_recording','_prepared','_universal_recording_authorized'):
            with self.subTest(attr=attr):
                self.setUp()
                setattr(self.window,attr,True)
                with self.assertRaises(RuntimeError): self.adapter.prepare('attempt',self.plan)
                self.dispatch.assert_not_called()

    def test_axis_read_failure_or_cancellation_blocks(self):
        self.adapter._physical_readback.side_effect=RuntimeError('axis unavailable')
        with self.assertRaisesRegex(RuntimeError,'axis unavailable'):self.adapter.prepare('attempt',self.plan)
        self.dispatch.assert_not_called()
        self.setUp()
        self.window.stop_worker_event.set()
        with self.assertRaisesRegex(RuntimeError,'cancelled'):self.adapter.prepare('attempt',self.plan)
        self.dispatch.assert_not_called()


class GuiTests(unittest.TestCase):
    def test_manual_selection_is_idle_and_does_not_launch(self):
        gui=SimpleNamespace(_operator_universal_enabled=Mock(return_value=True),
            _universal_initialize_requested=False, common_record_button=Mock(),
            _universal_initialize_button=Mock(),_universal_summary=Mock(),common_status=Mock())
        with patch.dict(os.environ,{'QTG_UNIVERSAL_INITIALIZATION_ONLY':'1'}):
            App._launch_common_test(gui,'manual',SimpleNamespace(reference='1.f.(2)'))
        self.assertTrue(gui._universal_manual_selected)
        gui._universal_initialize_button.configure.assert_called_once_with(state='normal')
        gui.common_record_button.configure.assert_called_once_with(state='disabled')

    def test_button_dispatches_only_after_manual_selection(self):
        gui=SimpleNamespace(_universal_manual_selected=False,_launch_common_test=Mock(),
            _universal_initialize_button=Mock(),_universal_summary=Mock())
        spec=SimpleNamespace(reference='1.f.(2)')
        App._initialize_universal_manual(gui,spec)
        gui._launch_common_test.assert_not_called()
        gui._universal_manual_selected=True
        App._initialize_universal_manual(gui,spec)
        gui._launch_common_test.assert_called_once_with('manual',spec,force_initialize=True)
        self.assertFalse(gui._universal_initialize_requested)

    def test_init_only_blocks_automatic_and_record_commands(self):
        gui=SimpleNamespace(common_status=Mock())
        with patch.dict(os.environ,{'QTG_UNIVERSAL_INITIALIZATION_ONLY':'1'}):
            App._launch_common_test(gui,'auto',SimpleNamespace(reference='1.f.(2)'))
            App._common_command(gui,'record')
            App._common_command(gui,'pid_engage')
        self.assertEqual(gui.common_status.configure.call_count,3)

    def test_display_status_lock_errors_and_only_required_rows(self):
        state={'initialization_only':True,'can_record':False,'initialization_attempt':{
            'status':'BLOCKED','failure_reason':'pause unknown',
            'reviewed_criteria':{'ias_kt':{}},'readiness_diagnostics':[
                {'parameter':'ias_kt','actual_p3d':None,'status':'UNAVAILABLE'},
                {'parameter':'engine_rpm','actual_p3d':0,'status':'UNAVAILABLE'}]}}
        phase,lock,error,rows=display_state(state)
        self.assertEqual((phase,lock,error),('FAILED','LOCKED','pause unknown'))
        self.assertEqual([r['parameter'] for r in rows],['ias_kt'])
        self.assertEqual(display_state({})[:2],('IDLE','UNVERIFIED'))
        self.assertEqual(display_state({'initialization_attempt':{'status':'READY'}})[0],'READY')
