"""Plan/configuration regression tests. Synthetic approval, no hardware or DLLs."""
from dataclasses import replace
import json
import unittest
from unittest.mock import Mock, patch

import qtg_common_engine as engine
from backend import engine_initialization_adapter as integration
from backend.initialization_plan_safety import PlanBlocked
from backend.universal_initialization import InitializationManager, State
import test_engine_universal_initialization as fixtures


class PlanSafetyTests(unittest.TestCase):
    def setUp(self):
        self.f = fixtures.EngineUniversalTests()
        self.f.setUp()
        self.addCleanup(self.f.doCleanups)
        self.w = self.f.worker

    def assert_blocked(self, reason):
        with patch.object(engine, '_initialize_mode') as dispatch:
            engine.initialize_mode(self.w, self.w._session_context['mode'])
        dispatch.assert_not_called()
        record = self.w._session_context['initialization_attempt']
        self.assertEqual(record['status'], 'BLOCKED')
        self.assertIn(reason, record['failure_reason'])
        self.assertFalse(self.w._prepared)
        self.w._initialization_stage(self.w._initialization_generation,'Ready')
        self.assertEqual(record['status'],'BLOCKED')
        self.w.handle_command({'action':'record'})
        self.w.arm_recording.assert_not_called()
        self.w.brunner.move_actuators.assert_not_called()
        self.w.stop_test.assert_not_called()  # Rejection acquired no hardware.

    def test_manual_band_is_explicit_and_not_replay_60(self):
        plan = integration.load_plan(self.w._session_context, self.f.plan_path)
        self.assertEqual(plan.checks['engine_rpm'].tolerance, 7.)
        self.assertEqual(plan.checks['engine_rpm'].value, 2000.)  # Synthetic, not a live target.
        self.assertEqual(plan.mode, 'manual')

    def test_replay_preserves_60_and_rejects_61(self):
        self.f.configure('1.f.(2)', 'auto')
        self.assertEqual(integration.load_plan(self.w._session_context, self.f.plan_path).checks['engine_rpm'].tolerance,60.)
        self.f.document['checks']['engine_rpm']['tolerance']=61.
        self.f.write_document()
        self.assert_blocked('existing test-specific tolerance')

    def test_draft_status_blocks_even_with_complete_values(self):
        self.f.document['approval_status']='DRAFT'
        self.f.plan_path.write_text(json.dumps(self.f.document))
        self.assert_blocked('Draft or unapproved')

    def test_missing_approval_blocks(self):
        self.f.document.pop('approval')
        self.f.plan_path.write_text(json.dumps(self.f.document))
        self.assert_blocked('Draft or unapproved')

    def test_stage5i_candidate_remains_inactive(self):
        from pathlib import Path
        candidate=Path(__file__).resolve().parents[1]/'docs/deceleration_manual_plan_candidate.json'
        with self.assertRaisesRegex(PlanBlocked,'Draft or unapproved'):
            integration.load_plan(self.w._session_context,candidate)

    def test_changed_target_cannot_reuse_approval(self):
        self.f.document['checks']['wind_speed']['value']=9.
        self.f.plan_path.write_text(json.dumps(self.f.document))
        self.assert_blocked('identity mismatch')

    def test_unapproved_exclusion_blocks(self):
        self.f.document['approval']['approved_exclusions'].pop()
        self.f.plan_path.write_text(json.dumps(self.f.document))
        self.assert_blocked('every exclusions entry')

    def test_missing_and_nonfinite_manual_rpm_target_blocks(self):
        for value in (None, float('nan')):
            with self.subTest(value=value):
                p=self.f.root/'initialization.json'
                data=json.loads(p.read_text());data['engine_rpm']=value;p.write_text(json.dumps(data))
                if value is None:
                    self.f.document['configuration_identity']=integration.configuration_snapshot(self.w._session_context)[1]
                    self.f.write_document()
                    self.assert_blocked('engine_rpm: no configured paused-initialization target')
                else:
                    with self.assertRaisesRegex(PlanBlocked,'nonfinite'):
                        integration.configuration_snapshot(self.w._session_context)

    def test_reviewed_manual_rpm_exclusion_is_phase_specific(self):
        self.f.document['checks'].pop('engine_rpm')
        self.f.document['exclusions']['engine_rpm']='Synthetic approved paused-phase nonapplicability; no live approval'
        self.f.write_document()
        plan = integration.load_plan(self.w._session_context, self.f.plan_path)
        self.assertNotIn('engine_rpm', plan.checks)
        self.assertIn('engine_rpm', plan.exclusions)
        # A changed exclusion without a fresh review still blocks.
        self.f.document['exclusions']['engine_rpm'] = 'Changed after synthetic review'
        self.f.plan_path.write_text(json.dumps(self.f.document))
        self.assert_blocked('identity mismatch')

    def test_missing_tolerance_blocks_even_when_document_is_approved(self):
        self.f.document['checks']['engine_rpm']['tolerance']=None
        self.f.write_document()
        self.assert_blocked('engine_rpm: no approved tolerance/source')

    def test_unsupported_configuration_cannot_be_promoted_by_plan_approval(self):
        self.f.document['exclusions'].pop('weight')
        self.f.document['checks']['weight']=dict(value=834.,value_source='synthetic fixture only',
            tolerance=25.,tolerance_source='synthetic fixture only')
        self.f.write_document()
        self.assert_blocked('authoritative V1 verification unavailable')

    def test_approval_requires_each_check_and_reviewer(self):
        self.f.document['approval']['approved_checks'].pop()
        self.f.plan_path.write_text(json.dumps(self.f.document))
        self.assert_blocked('every checks entry')
        self.f.write_document()
        self.f.document['approval']['reviewer']=''
        self.f.plan_path.write_text(json.dumps(self.f.document))
        self.assert_blocked('reviewer')

    def test_changed_config_file_blocks_even_if_values_unchanged(self):
        p=self.f.root/'initialization.json'
        p.write_text(p.read_text()+'\n')
        self.assert_blocked('configuration identity mismatch')

    def test_final_reload_precedes_plan_and_actual_dispatch_uses_bound_values(self):
        p=self.f.root/'initialization.json'
        cfg=json.loads(p.read_text());cfg['alt_ft']=780.;p.write_text(json.dumps(cfg))
        self.f.document['configuration_identity']=integration.configuration_snapshot(self.w._session_context)[1]
        self.f.write_document()
        self.assertEqual(self.w._session_context['initial_conditions']['alt_ft'],755.)
        adapter=self.f.start()
        self.assertEqual(self.w._universal_plan.checks['alt_ft'].value,780.)
        self.assertEqual(adapter.bound_conditions()['alt_ft'],780.)
        commands=[]
        def start_manual_test(window):commands.append(dict(window.manual_initial_conditions))
        base=type('Base',(),{})()
        base.__file__=__file__
        base.SuperMushakLoggerApp=type('App',(),{'start_manual_test':staticmethod(start_manual_test)})
        self.w._base_module=base
        with patch.object(engine,'configure_test_extension'), patch.object(engine,'engine_test_policy',return_value=None):
            engine._initialize_mode(self.w,'manual')
        self.assertEqual(commands[0],adapter.bound_conditions())

    def test_changed_overrides_block(self):
        self.w._session_context['initial_condition_overrides']={'alt_ft':780.}
        self.assert_blocked('configuration identity mismatch')

    def test_changed_reference_requires_review_and_replay_targets_are_not_overridden(self):
        self.f.configure('1.f.(2)','auto')
        self.f.document['checks']['engine_rpm']['value']=1000.
        self.f.write_document()
        self.assert_blocked('reviewed target differs from selected reference')
        self.f.configure('1.f.(2)','auto')
        p=self.f.root/'selected_manual_init_conditions.json';p.write_text(p.read_text()+'\n')
        self.assert_blocked('reference contents identity mismatch')

    def test_configuration_change_during_handoff_prevents_recording(self):
        adapter=self.f.ready();self.f.now+=.1;adapter._frame=self.f.frame(4)
        p=self.f.root/'initialization.json'
        adapter.thread.join.side_effect=lambda **kwargs:p.write_text(p.read_text()+'\n')
        self.w.handle_command({'action':'record'})
        self.w.arm_recording.assert_not_called()
        self.assertEqual(self.w._universal_manager.state,State.FAILED)

    def test_modification_after_dispatch_revokes_attempt_before_ready(self):
        self.f.start()
        p=self.f.root/'initialization.json';p.write_text(p.read_text()+'\n')
        self.w._initialization_stage(self.w._initialization_generation,'Ready')
        self.assertEqual(self.w._universal_manager.state,State.BLOCKED)
        self.assertFalse(self.w._prepared)
        self.w.handle_command({'action':'record'})
        self.w.arm_recording.assert_not_called()
        self.w.stop_test.assert_called_once()

    def test_plan_modified_after_ready_blocks_recording(self):
        self.f.ready()
        self.f.plan_path.write_text(self.f.plan_path.read_text()+'\n')
        self.w.handle_command({'action':'record'})
        self.assertEqual(self.w._universal_manager.state,State.BLOCKED)
        self.w.arm_recording.assert_not_called()

    def test_core_rejects_missing_approval_and_changed_compiled_plan(self):
        plan=integration.load_plan(self.w._session_context,self.f.plan_path)
        for invalid in (replace(plan,approval=None),replace(plan,max_age_s=99.)):
            adapter=Mock();manager=InitializationManager(adapter)
            attempt=manager.start(invalid)
            self.assertEqual(manager.state,State.BLOCKED)
            self.assertFalse(manager.prepared(attempt))
            self.assertFalse(manager.authorize_recording(attempt))
            self.assertFalse(manager.cancel(attempt))
            adapter.prepare.assert_not_called()


if __name__=='__main__':unittest.main()
