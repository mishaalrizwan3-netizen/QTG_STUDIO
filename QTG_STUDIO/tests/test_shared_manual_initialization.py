"""Shared Manual integration. All targets/approvals below are offline fixtures."""
from dataclasses import replace
import json
import os
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import qtg_common_engine as engine
from backend import engine_initialization_adapter as integration
from backend.initialization_configuration import (MANUAL_PLANS, ENGINE_MANDATORY,
    ResolvedConfiguration, manual_plans)
from backend.initialization_plan_safety import PlanBlocked
from backend.initialization_requirements import REQUIREMENTS
from backend.universal_initialization import InitializationManager, State
import test_engine_universal_initialization as fixtures


class SharedManualTests(unittest.TestCase):
    def setUp(self):
        self.f = fixtures.EngineUniversalTests()
        self.f.setUp()
        self.addCleanup(self.f.doCleanups)
        self.w = self.f.worker
        self.env = patch.dict(os.environ, {MANUAL_PLANS: '{}'})
        self.env.start()
        self.addCleanup(self.env.stop)
        self.configure('1.c.(1)')

    def configure(self, reference, keys=None):
        self.f.configure('1.f.(2)', 'manual')
        for flag in integration.ENV.values():
            os.environ[flag] = '0'
        context = self.w._session_context
        context['test_reference'] = reference
        context['configuration_revision'] = 'current-gui-revision'
        conditions = dict(context['initial_conditions'])
        if reference != '1.f.(2)':
            conditions.update(on_ground=reference.startswith(('2.a', '4.')),
                              alt_ft=3000., ias_kt=80., throttle=100.)
            if reference == '1.f.(1)':
                conditions.update(alt_ft=2000., throttle=5.)
            conditions.update(wind_speed=0., wind_direction=0., barometer=1013.,
                              engine_rpm=2000., prop_rpm=2000.)
            context['initial_conditions'] = conditions
            context['initial_condition_overrides'] = None
        keys = set(keys or (ENGINE_MANDATORY if reference.startswith('1.f') else
                           {'on_ground', 'alt_ft', 'ias_kt', 'flaps', 'throttle'}))
        self.f.document = dict(test_reference=reference, mode='manual',
            checks={key: dict(target_source='resolved_initial_conditions',
                tolerance=REQUIREMENTS[reference].requirements[key].tolerance or 0.,
                tolerance_source='Synthetic test criterion; no live approval') for key in keys},
            exclusions={key: 'Synthetic reviewed nonapplicability; no live approval'
                        for key in REQUIREMENTS[reference].requirements if key not in keys},
            configuration_identity=integration.configuration_snapshot(context)[1],
            review_source='Synthetic offline integration only', timeout_s=10., max_age_s=1., stable_samples=3,
            manual_policy=dict(status='CONFIGURED', preparation='existing_manual',
                mandatory_checks=sorted(keys), physical_controls='read_only' if reference.startswith('1.f') else 'not_required',
                stability=dict(kind='paused_state', source='Synthetic paused-state scope; not flight stability')))
        if 'engine_rpm' in keys:
            self.f.document['checks']['engine_rpm']['tolerance'] = 7.
        self.f.write_document()
        os.environ[MANUAL_PLANS] = json.dumps({reference: str(self.f.plan_path)})
        self.w.stop_worker_event.clear()

    def start(self):
        with patch.object(integration, 'InitializationManager',
                side_effect=lambda adapter, **kwargs: InitializationManager(adapter, clock=lambda: self.f.now, **kwargs)), \
                patch.object(engine, '_initialize_mode') as dispatch:
            engine.initialize_mode(self.w, 'manual')
        dispatch.assert_called_once_with(self.w, 'manual', None)
        adapter = self.w._engine_initialization_adapter
        adapter.clock = lambda: self.f.now
        adapter.thread = Mock(is_alive=Mock(return_value=False))
        return adapter

    def ready(self):
        adapter = self.start()
        for sequence in (1, 2, 3):
            self.f.now += .1
            adapter._frame = self.f.frame(sequence)
            adapter.physical = {}
            if sequence == 1:
                self.w._initialization_stage(self.w._initialization_generation, 'Ready')
            else:
                self.w._poll_universal_initialization()
        self.assertEqual(self.w._universal_manager.state, State.READY)
        return adapter

    def assert_blocked(self, text):
        with patch.object(engine, '_initialize_mode') as dispatch:
            engine.initialize_mode(self.w, 'manual')
        dispatch.assert_not_called()
        self.assertEqual(self.w._session_context['initialization_attempt']['status'], 'BLOCKED')
        self.assertIn(text, self.w._session_context['initialization_attempt']['failure_reason'])
        self.w.handle_command({'action': 'record'})
        self.w.arm_recording.assert_not_called()

    def test_one_manager_for_deceleration_acceleration_and_climb(self):
        for reference in ('1.f.(2)', '1.f.(1)', '1.c.(1)'):
            with self.subTest(reference=reference):
                self.configure(reference)
                adapter = self.ready()
                self.assertIsInstance(self.w._universal_manager, InitializationManager)
                self.assertIsInstance(adapter, integration.ManualPreparationAdapter)
                self.assertEqual(self.w._universal_plan.reference, reference)
                self.w.arm_recording.assert_not_called()
                self.w._universal_manager.cancel(self.w._universal_attempt_id)

    def test_global_unconfigured_policy_is_not_promoted(self):
        self.ready()
        self.assertEqual(REQUIREMENTS['1.c.(1)'].status, 'UNCONFIGURED')

    def test_every_selectable_reference_can_use_the_same_reviewed_manual_contract(self):
        for reference in REQUIREMENTS:
            if reference == '1.c.(2)':
                continue
            with self.subTest(reference=reference):
                keys = ENGINE_MANDATORY if reference.startswith('1.f') else {'heading_deg'}
                self.configure(reference, keys)
                self.ready()
                self.w.arm_recording.assert_not_called()
        self.assertEqual(sum(p.status == 'CONFIGURED' for p in REQUIREMENTS.values()), 2)

    def test_static_and_visual_policies_do_not_inherit_engine_requirements(self):
        for reference, keys in [('2.a.(1)', {'on_ground'}), ('4.b.(1)', {'heading_deg'})]:
            self.configure(reference, keys)
            adapter = self.ready()
            self.assertEqual(set(self.w._universal_plan.checks), keys)
            self.assertEqual(adapter._physical_readback(), {})
            self.w.brunner.read_axis.assert_not_called()
            self.assertNotIn('engine_rpm', self.w._universal_plan.checks)

    def test_zero_targets_and_snapshot_are_immutable(self):
        self.configure('4.c.(1)', {'heading_deg'})
        captured = ResolvedConfiguration.capture(self.w._session_context)
        self.assertEqual(captured.conditions['heading_deg'], 0.)
        with self.assertRaises(TypeError):
            captured.conditions['heading_deg'] = 12.
        with self.assertRaises(TypeError):
            captured.provenance['heading_deg']['unit'] = 'wrong'
        adapter = self.ready()
        self.assertEqual(self.w._universal_plan.checks['heading_deg'].value, 0.)
        self.assertEqual(adapter.bound_conditions()['heading_deg'], 0.)
        self.assertEqual(self.w._session_context['initialization_attempt']['configuration_provenance']['heading_deg']['unit'], 'deg true')

    def test_changed_target_requires_new_approval_then_changes_manager_target(self):
        self.w._session_context['initial_conditions']['ias_kt'] = 72.
        self.assert_blocked('configuration identity mismatch')
        self.f.document['configuration_identity'] = integration.configuration_snapshot(self.w._session_context)[1]
        self.f.write_document()
        self.ready()
        self.assertEqual(self.w._universal_plan.checks['ias_kt'].value, 72.)

    def test_commands_receive_bound_snapshot_for_nonengine_test(self):
        adapter = self.start()
        self.w._base_module = SimpleNamespace()
        self.w.start_manual_test = Mock()
        with patch.object(engine, 'configure_test_extension'):
            engine._initialize_mode(self.w, 'manual')
        self.w.start_manual_test.assert_called_once()
        self.assertEqual(self.w.manual_initial_conditions, adapter.bound_conditions())
        self.w._session_context['initial_conditions']['alt_ft'] = 4000.
        with self.assertRaises(PlanBlocked):
            adapter.bound_conditions()

    def test_duplicates_unapproved_limits_and_missing_mandatory_block(self):
        self.f.document['checks']['ias_kt']['value'] = 80.
        self.f.write_document()
        self.assert_blocked('duplicate plan target values')
        self.configure('1.c.(1)')
        self.f.document['checks']['ias_kt']['tolerance'] = None
        self.f.write_document()
        self.assert_blocked('no approved tolerance')
        self.configure('1.c.(1)')
        self.f.document['checks'].pop('ias_kt')
        self.f.document['exclusions']['ias_kt'] = 'Synthetic exclusion'
        self.f.write_document()
        self.assert_blocked('Missing required Manual')

    def test_unapproved_policy_and_unknown_adapter_block(self):
        self.f.document['manual_policy']['status'] = 'UNCONFIGURED'
        self.f.write_document()
        self.assert_blocked('reviewed CONFIGURED')
        self.configure('1.c.(1)')
        self.f.document['manual_policy']['preparation'] = 'new_unreviewed_controller'
        self.f.write_document()
        self.assert_blocked('existing_manual')

    def test_pause_unknown_stale_and_missing_feedback_fail_without_recording(self):
        for kind in ('stale', 'missing', 'pause'):
            self.configure('1.c.(1)')
            adapter = self.start()
            frame = self.f.frame(1)
            if kind == 'stale':
                frame = replace(frame, measured_at=self.f.now-2.)
            if kind == 'missing':
                frame = replace(frame, values={})
            if kind == 'pause':
                adapter.native_timing['running'] = None
            adapter._frame, adapter.physical = frame, {}
            self.w._initialization_stage(self.w._initialization_generation, 'Ready')
            self.assertEqual(self.w._universal_manager.state, State.FAILED)
            self.w.handle_command({'action':'record'})
            self.w.arm_recording.assert_not_called()

    def test_cancellation_worker_failure_and_new_attempt(self):
        self.start()
        first = self.w._universal_attempt_id
        generation = self.w._initialization_generation
        self.w.handle_command({'action':'stop'})
        self.w._initialization_stage(generation, 'Ready')
        self.assertEqual(self.w._universal_manager.state, State.CANCELLED)
        self.start()
        self.assertNotEqual(first, self.w._universal_attempt_id)
        self.w._initialization_stage(self.w._initialization_generation, 'Failed', 'Synthetic worker failure')
        self.assertEqual(self.w._universal_manager.state, State.FAILED)

    def test_gui_invalidation_is_revision_correlated_and_blocks_recording(self):
        self.ready()
        self.w.handle_command({'action':'invalidate_initialization', 'configuration_revision':'old'})
        self.assertEqual(self.w._universal_manager.state, State.READY)
        self.w.handle_command({'action':'invalidate_initialization', 'configuration_revision':'current-gui-revision'})
        self.assertEqual(self.w._universal_manager.state, State.CANCELLED)
        self.w.handle_command({'action':'record'})
        self.w.arm_recording.assert_not_called()

    def test_recording_handoff_stops_initialization_and_gui_edit_does_not_stop_manoeuvre(self):
        adapter = self.ready()
        self.f.now += .1
        adapter._frame = self.f.frame(4)
        self.w.handle_command({'action':'record', 'configuration_revision':'current-gui-revision'})
        self.w.arm_recording.assert_called_once()
        self.assertEqual(self.w._universal_manager.state, State.RECORDING)
        self.w.stop_test.reset_mock()
        self.w.handle_command({'action':'invalidate_initialization', 'configuration_revision':'current-gui-revision'})
        self.w.stop_test.assert_not_called()
        self.assertTrue(adapter.cancelled.is_set())

    def test_generic_activation_does_not_select_pid_or_replay_and_is_off_by_default(self):
        for reference in ('1.c.(1)', '1.f.(1)', '1.f.(2)'):
            for method in ('pid', 'replay'):
                self.assertFalse(integration.requested(dict(test_reference=reference,mode='auto',control_method=method)))
        os.environ[MANUAL_PLANS] = '{}'
        self.assertFalse(integration.requested(self.w._session_context))
        with patch.object(engine, '_initialize_mode') as legacy:
            engine.initialize_mode(self.w, 'manual')
        legacy.assert_called_once()

    def test_plan_selection_changes_revoke_ready(self):
        self.ready()
        os.environ[MANUAL_PLANS] = '{}'
        self.w.handle_command({'action':'record', 'configuration_revision':'current-gui-revision'})
        self.w.arm_recording.assert_not_called()
        self.assertEqual(self.w._universal_manager.state, State.BLOCKED)

    def test_record_requires_current_gui_revision(self):
        self.ready()
        self.w.handle_command({'action':'record', 'configuration_revision':'old'})
        self.w.arm_recording.assert_not_called()
        self.assertEqual(self.w._universal_manager.state, State.READY)

    def test_invalid_mapping_is_explicit_block_not_legacy_fallback(self):
        os.environ[MANUAL_PLANS] = 'not json'
        self.assert_blocked('invalid explicit Manual selection')


class ManualStabilityTests(SharedManualTests):
    # Discovery loads only this class's additional tests through load_tests below.
    def stability_adapter(self):
        self.f.document['manual_policy']['stability'] = dict(kind='running_window',
            source='Synthetic offline window only', limits={'ias_kt': .2}, duration_s=.2, min_samples=3, max_gap_s=.5)
        self.f.write_document()
        # Create via the real installer, but prevent its background collection thread.
        integration.begin_attempt(self.w)
        with patch.object(integration.ManualPreparationAdapter, 'prepare'):
            integration.install(self.w, 'manual', None, Mock())
        adapter = self.w._engine_initialization_adapter
        adapter.attempt_id = 'synthetic'
        adapter.plan = self.w._universal_plan
        adapter.clock = lambda: self.f.now
        self.w._session_context['initialization_attempt']['attempt_id'] = 'synthetic'
        adapter.reader.feedback = Mock()
        return adapter

    def sample(self, adapter, sequence, running, *, advancing=True, **changes):
        self.f.now += .11
        adapter.reader.native_timing = dict(running=running, simulator_time_advanced=advancing)
        adapter.reader.feedback.return_value = self.f.frame(sequence, **changes)
        return adapter._sample()

    def test_paused_frames_never_establish_running_stability(self):
        adapter = self.stability_adapter()
        for sequence in (1,2,3):
            self.assertEqual(self.sample(adapter, sequence, False), (None, {}))
        adapter.preparation_finished()
        with self.assertRaisesRegex(integration.TelemetryUnavailable, 'paused snapshots are not stabilized flight'):
            self.sample(adapter, 4, False)

    def test_running_window_then_final_pause_and_held_paused_state(self):
        adapter = self.stability_adapter()
        for sequence in (1,2,3):
            self.sample(adapter, sequence, True)
        adapter.preparation_finished()
        frame, physical = self.sample(adapter, 4, False)
        self.assertEqual(frame.sequence, 4)
        self.assertEqual(physical, {})
        self.f.now += 2.
        self.assertEqual(self.sample(adapter, 5, False)[0].sequence, 5)
        self.assertEqual(self.w._session_context['initialization_attempt']['stability_evidence']['kind'], 'running_window')

    def test_frozen_physics_changing_packets_and_out_of_band_reset_window(self):
        for kind in ('frozen', 'deviation'):
            adapter = self.stability_adapter()
            for sequence in (1,2,3):
                self.sample(adapter, sequence, True, advancing=kind!='frozen',
                            **({'ias_kt': 100.} if kind=='deviation' else {}))
            adapter.preparation_finished()
            with self.assertRaises(integration.TelemetryUnavailable):
                self.sample(adapter, 4, False)

    def test_cancelled_window_does_not_read_or_issue_controls(self):
        adapter = self.stability_adapter()
        adapter.cancelled.set()
        with self.assertRaisesRegex(RuntimeError, 'cancelled'):
            self.sample(adapter, 1, True)
        adapter.reader.feedback.assert_not_called()
        self.w.brunner.move_actuators.assert_not_called()

    def test_request_crossing_preparation_boundary_is_not_delivered_as_verification(self):
        adapter = self.stability_adapter()
        for sequence in (1,2,3):
            self.sample(adapter, sequence, True)
        old = self.f.frame(4)
        self.f.now += .1
        adapter.preparation_finished()
        adapter.reader.native_timing = dict(running=False)
        adapter.reader.feedback.return_value = old
        self.assertEqual(adapter._sample(), (None, {}))


class GuiEditGuardTests(unittest.TestCase):
    def setUp(self):
        import tkinter as tk
        from frontend.main import App
        methods = {name: getattr(App, name) for name in ('_manual_condition_fingerprint',
                    '_manual_conditions_changed', '_common_command')}
        self.gui = type('Gui', (SimpleNamespace,), methods)()
        self.tcl = tk.Tcl()
        self.value = tk.StringVar(master=self.tcl, value='0')
        self.gui._test_settings = {'4.b.(1)': ({'Heading[deg]': self.value}, [])}
        self.gui._common_reference_vars = {}
        self.gui._common_display_spec = SimpleNamespace(reference='4.b.(1)')
        self.gui.common_record_button = Mock()
        self.gui.common_status = Mock()
        self.gui._common_process = Mock(poll=Mock(return_value=None))
        self.gui._common_context = dict(test_reference='4.b.(1)', configuration_revision='current')
        self.gui._manual_gui_bindings = {'4.b.(1)': dict(
            fingerprint=self.gui._manual_condition_fingerprint('4.b.(1)'), revision='current',
            context=dict(self.gui._common_context), invalid=False)}
        self.value.trace_add('write', lambda *_: self.gui._manual_conditions_changed('4.b.(1)'))

    def test_unsent_gui_edit_immediately_cancels_and_blocks_record(self):
        with patch.object(engine, 'send_command') as send:
            self.value.set('12')
            self.gui._common_command('record')
        send.assert_called_once_with(self.gui._common_context, 'invalidate_initialization', configuration_revision='current')
        self.gui.common_record_button.configure.assert_called_with(state='disabled')
        self.assertTrue(self.gui._manual_gui_bindings['4.b.(1)']['invalid'])

    def test_record_guard_detects_edit_even_without_trace_delivery(self):
        self.gui._manual_gui_bindings['4.b.(1)']['fingerprint'] = 'old value'
        with patch.object(engine, 'send_command') as send:
            self.gui._common_command('record')
        self.assertEqual(send.call_args.args[1], 'invalidate_initialization')

    def test_changing_a_field_back_does_not_revive_cancelled_attempt(self):
        with patch.object(engine, 'send_command') as send:
            self.value.set('12')
            self.value.set('0')
            self.gui._common_command('record')
        self.assertTrue(self.gui._manual_gui_bindings['4.b.(1)']['invalid'])
        send.assert_called_once()

    def test_disabled_legacy_edit_does_not_cancel_and_record_is_unchanged(self):
        self.gui._manual_gui_bindings = {}
        with patch.object(engine, 'send_command') as send:
            self.value.set('12')
            self.gui._common_command('record')
        send.assert_called_once_with(self.gui._common_context, 'record')

    def test_real_manual_button_dispatch_supplies_conditions_without_reference(self):
        from frontend.main import App
        class Value:
            def __init__(self, value): self.value = value
            def get(self): return self.value
        methods = {name: getattr(App, name) for name in ('_launch_common_test',
                    '_manual_condition_fingerprint', '_manual_conditions_changed',
                    '_operator_universal_enabled', '_initialize_universal_manual')}
        for reference in ('1.c.(1)', '1.f.(1)', '1.f.(2)'):
            values = {'Alt[ft]':'2000', 'IAS[kts]':'80', 'Latitude[deg]':'32', 'Longitude[deg]':'74',
                'Heading[deg]':'0', 'Pitch[deg]':'0', 'Bank[deg]':'0', 'Flaps':'0', 'Throttle[%]':'5',
                'Temperature[C]':'15', 'Flt cond':'Airborne'}
            gui = type('Gui', (SimpleNamespace,), methods)()
            gui._test_settings = {reference: ({key: Value(value) for key, value in values.items()}, [])}
            gui._test_defaults = {reference: dict(values)}
            gui._common_reference_vars = {}
            gui._common_duration_var = Value('21')
            gui._common_duration_vars = {reference: Value('21')}
            gui._apply_test_settings = Mock(return_value=True)
            gui._common_process = None
            gui.after = Mock()
            gui._poll_common_test = Mock()
            gui.common_status = Mock()
            gui.common_record_button = Mock()
            gui._universal_initialize_button = Mock()
            gui._universal_summary = Mock()
            context = dict(test_reference=reference, configuration_revision=None)
            with patch.dict(os.environ, {MANUAL_PLANS: json.dumps({reference: 'reviewed.json'})}), \
                    patch.object(engine, 'launch', return_value=(Mock(), context)) as launch, \
                    patch.object(engine, 'validate_reference') as validate:
                gui._launch_common_test('manual', SimpleNamespace(reference=reference, title='Offline', pdf_graphs=[]))
                if reference == '1.f.(2)':
                    launch.assert_not_called()
                    gui._initialize_universal_manual(SimpleNamespace(reference=reference, title='Offline', pdf_graphs=[]))
            launch.assert_called_once()
            self.assertIsNone(launch.call_args.args[3])
            validate.assert_not_called()
            self.assertIn('configuration_revision', launch.call_args.kwargs)
            self.assertIn(reference, gui._manual_gui_bindings)
            gui._automatic_methods = {reference: Value('PID Automatic')}
            gui._pid_validation = {reference: Value('')}
            gui._common_reference_var = Value('manual.csv')
            with patch.dict(os.environ, {MANUAL_PLANS: json.dumps({reference: 'reviewed.json'})}), \
                    patch.object(engine, 'launch', return_value=(Mock(), context)) as launch, \
                    patch.object(engine, 'validate_reference', return_value='manual.csv'):
                gui._launch_common_test('auto', SimpleNamespace(reference=reference, title='Offline', pdf_graphs=[]))
            launch.assert_called_once()
            self.assertEqual(launch.call_args.kwargs['control_method'], 'pid' if reference == '1.c.(1)' else 'replay')
            self.assertNotIn('configuration_revision', launch.call_args.kwargs)
            self.assertNotIn(reference, gui._manual_gui_bindings)


def load_tests(loader, tests, pattern):
    suite = unittest.TestSuite(loader.loadTestsFromTestCase(SharedManualTests))
    suite.addTests(loader.loadTestsFromTestCase(GuiEditGuardTests))
    for name in ManualStabilityTests.__dict__:
        if name.startswith('test_'):
            suite.addTest(ManualStabilityTests(name))
    return suite
