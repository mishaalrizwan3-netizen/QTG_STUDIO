"""Resolved GUI conditions are the target source. Mocked workers; no GUI/hardware."""
from dataclasses import replace
import json
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import qtg_common_engine as engine
from backend import deceleration_config as configuration
from backend import engine_initialization_adapter as integration
from backend.initialization_plan_safety import PlanBlocked
from backend.universal_initialization import State
from backend.initialization_readiness_diagnostic import comparison_rows, text_table
import test_engine_universal_initialization as fixtures


class GuiTargetTests(unittest.TestCase):
    def setUp(self):
        self.f=fixtures.EngineUniversalTests();self.f.setUp()
        self.addCleanup(self.f.doCleanups)
        self.w=self.f.worker

    def revise_file(self, **values):
        p=self.f.root/'initialization.json'
        document=json.loads(p.read_text());document.update(values)
        p.write_text(json.dumps(document))

    def review_current_settings(self):
        # Synthetic review only; production never creates an approval automatically.
        self.f.document['configuration_identity']=integration.configuration_snapshot(self.w._session_context)[1]
        self.f.write_document()

    def assert_blocked(self, phrase):
        with patch.object(engine,'_initialize_mode') as dispatcher:
            engine.initialize_mode(self.w,'manual')
        dispatcher.assert_not_called()
        record=self.w._session_context['initialization_attempt']
        self.assertEqual(record['status'],'BLOCKED')
        self.assertIn(phrase,record['failure_reason'])
        self.w.handle_command({'action':'record'})
        self.w.arm_recording.assert_not_called()
        return record

    def test_gui_overrides_launch_and_manager_share_one_target_after_new_review(self):
        defaults=configuration.display_defaults()
        overrides=configuration.display_overrides(dict(defaults, **{'Alt[ft]':'780','Temperature[C]':'14'}),defaults)
        self.assertEqual(overrides,{'alt_ft':'780','temperature_c':'14'})
        with patch.object(engine,'brunner_root'),patch.object(engine.subprocess,'Popen'), \
                patch.object(engine,'engine_test_policy',return_value=None):
            _,context=engine.launch('manual',SimpleNamespace(reference='1.f.(2)',title='Engine deceleration'),
                self.f.root/'sessions',initial_conditions=overrides)
        self.w._session_context=context
        self.assert_blocked('configuration identity mismatch')
        self.review_current_settings()
        adapter=self.f.start()
        for key in ('alt_ft','temperature_c'):
            self.assertNotIn('value',self.f.document['checks'][key])
            self.assertEqual(self.w._universal_plan.checks[key].value,adapter.bound_conditions()[key])
        self.assertEqual(self.w._universal_plan.checks['alt_ft'].value,780.)
        self.assertEqual(self.w._universal_plan.checks['temperature_c'].value,14.)

    def test_literal_target_in_criteria_file_is_rejected_even_if_equal(self):
        self.f.document['checks']['alt_ft']['value']=755.
        self.f.write_document()
        self.assert_blocked('duplicate plan target values are forbidden')

    def test_plan_cannot_supply_an_unconfigured_rpm_or_weather_target(self):
        p=self.f.root/'initialization.json';data=json.loads(p.read_text())
        for key in ('engine_rpm','prop_rpm','wind_speed'):data.pop(key)
        p.write_text(json.dumps(data));self.review_current_settings()
        record=self.assert_blocked('engine_rpm: no configured paused-initialization target')
        self.assertIn('prop_rpm: no configured',record['failure_reason'])
        self.assertIn('wind_speed: no configured',record['failure_reason'])
        rows={r['parameter']:r for r in record['readiness_diagnostics']}
        self.assertIsNone(rows['engine_rpm']['configured_target'])
        self.assertEqual(rows['engine_rpm']['tolerance'],7.)
        self.assertTrue(all(r['status']=='UNAVAILABLE' for r in rows.values()))

    def test_explicit_paused_rpm_targets_come_from_configuration_only(self):
        self.revise_file(engine_rpm=1234.,prop_rpm=1250.)  # Offline values, not live targets.
        self.review_current_settings();adapter=self.f.start()
        self.assertEqual(self.w._universal_plan.checks['engine_rpm'].value,1234.)
        self.assertEqual(self.w._universal_plan.checks['prop_rpm'].value,1250.)
        self.assertEqual(adapter.bound_conditions()['engine_rpm'],1234.)
        self.w.p3d.set_initial_engine_rpm.assert_not_called()

    def test_gui_change_after_ready_invalidates_attempt_and_diagnostic(self):
        self.f.ready()
        self.w._session_context['initial_condition_overrides']={'alt_ft':780.}
        self.w.handle_command({'action':'record'})
        self.assertEqual(self.w._universal_manager.state,State.BLOCKED)
        self.w.arm_recording.assert_not_called()
        rows=self.w._session_context['initialization_attempt']['readiness_diagnostics']
        self.assertTrue(all(r['status']=='UNAVAILABLE' for r in rows))

    def test_weather_override_cannot_be_ignored_before_or_after_ready(self):
        self.w._session_context['weather_overrides']={'wind_speed':8.}
        self.assert_blocked('weather_overrides has no existing Manual command interface')
        self.w._session_context.pop('weather_overrides')
        self.f.ready()
        self.w._session_context['weather_overrides']={'temperature_c':25.}
        self.w.handle_command({'action':'record'})
        self.assertEqual(self.w._universal_manager.state,State.BLOCKED)
        self.w.arm_recording.assert_not_called()

    def test_manual_reference_selection_change_invalidates_ready(self):
        self.f.ready();self.w._session_context['reference_csv']='different_reference.csv'
        self.w.handle_command({'action':'record'})
        self.assertEqual(self.w._universal_manager.state,State.BLOCKED)
        self.w.arm_recording.assert_not_called()

    def test_missing_tolerance_reports_parameter_without_default(self):
        self.f.document['checks']['alt_ft']['tolerance']=None
        self.f.write_document()
        record=self.assert_blocked('alt_ft: no approved tolerance/source')
        row=next(r for r in record['readiness_diagnostics'] if r['parameter']=='alt_ft')
        self.assertIsNone(row['tolerance'])  # Existing 25 is not silently reapproved.

    def test_diagnostic_has_pass_fail_unavailable_units_and_freshness(self):
        self.f.ready();self.f.now+=.1
        frame=self.f.frame(4,temperature_c=39.)
        frame=replace(frame,values={k:v for k,v in frame.values.items() if k!='prop_rpm'})
        timing=self.w._engine_initialization_adapter.native_timing
        rows={r['parameter']:r for r in comparison_rows(self.w._universal_plan,frame,timing,self.f.now)}
        self.assertEqual(rows['alt_ft']['configured_target'],755.)
        self.assertEqual(rows['alt_ft']['actual_p3d'],755.)
        self.assertEqual(rows['alt_ft']['status'],'PASS')
        self.assertEqual(rows['temperature_c']['status'],'FAIL')
        self.assertEqual(rows['prop_rpm']['status'],'UNAVAILABLE')
        self.assertIsNone(rows['prop_rpm']['actual_p3d'])
        self.assertEqual(rows['alt_ft']['unit'],'ft MSL')
        self.assertEqual(rows['alt_ft']['freshness'],'FRESH_NATIVE_RECEIPT')
        self.assertIn('Not available',text_table(list(rows.values())))

    def test_repeated_pending_frame_ages_out_in_diagnostic(self):
        self.f.ready();self.f.now+=1.1
        self.w._poll_universal_initialization()
        self.assertEqual(self.w._universal_manager.state,State.FAILED)
        rows=self.w._session_context['initialization_attempt']['readiness_diagnostics']
        self.assertTrue(all(r['freshness']=='STALE_NATIVE_RECEIPT' and r['status']=='UNAVAILABLE' for r in rows))

    def test_diagnostic_does_not_promote_unverified_pause_or_native_identity(self):
        self.f.ready();frame=self.f.frame(4)
        for timing in ({'sequence':4,'running':False},
                       {'sequence':4,'running':None,'generation':1,'request_id':4}):
            rows=comparison_rows(self.w._universal_plan,frame,timing,self.f.now)
            self.assertTrue(all(r['status']=='UNAVAILABLE' for r in rows))


if __name__=='__main__':unittest.main()
