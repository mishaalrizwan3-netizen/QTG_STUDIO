"""Offline conditional Manual policy tests; synthetic approval, no device I/O."""
import json
import unittest

from backend.initialization_configuration import ENGINE_MANDATORY, reviewed_manual_policy
from backend.initialization_plan_safety import PlanBlocked
from backend import engine_initialization_adapter as integration
import test_engine_universal_initialization as fixtures


class CalmWindTests(unittest.TestCase):
    def setUp(self):
        self.f = fixtures.EngineUniversalTests()
        self.f.setUp()
        self.addCleanup(self.f.doCleanups)
        self.context = self.f.worker._session_context
        self.document = self.f.document
        self.document['manual_policy'] = dict(status='CONFIGURED', preparation='existing_manual',
            mandatory_checks=sorted(ENGINE_MANDATORY), physical_controls='read_only',
            stability=dict(kind='paused_state', source='Synthetic paused policy'),
            wind_direction_applicability=dict(kind='not_applicable_when_configured_calm',
                                              source='Synthetic offline decision only'))
        self.document['checks'].pop('wind_direction')
        self.document['exclusions']['wind_direction'] = 'Not applicable only for configured calm wind'
        path = self.f.root / 'initialization.json'
        cfg = json.loads(path.read_text())
        cfg.pop('wind_direction')
        path.write_text(json.dumps(cfg))
        self.refresh()

    def refresh(self):
        self.document['configuration_identity'] = integration.configuration_snapshot(self.context)[1]
        self.f.write_document()  # Synthetic fixture approval only.

    def load(self):
        return integration.load_plan(self.context, self.f.plan_path)

    def test_calm_without_direction_loads_and_speed_remains_checked(self):
        plan = self.load()
        self.assertNotIn('wind_direction', plan.checks)
        self.assertIn('wind_direction', plan.exclusions)
        self.assertEqual(plan.checks['wind_speed'].value, 0.)

    def test_nonzero_speed_cannot_use_calm_exclusion(self):
        self.context['initial_condition_overrides'] = {'wind_speed': 1.}
        self.refresh()
        with self.assertRaisesRegex(PlanBlocked, 'wind_direction'):
            self.load()

    def test_changed_configuration_cannot_reuse_approval(self):
        self.context['initial_condition_overrides'] = {'wind_speed': 1.}
        with self.assertRaisesRegex(PlanBlocked, 'identity mismatch'):
            self.load()

    def test_no_conditional_rule_keeps_original_mandatory_direction(self):
        self.document['manual_policy'].pop('wind_direction_applicability')
        self.refresh()
        with self.assertRaisesRegex(PlanBlocked, 'wind_direction'):
            self.load()

    def test_missing_unapproved_exclusion_blocks(self):
        self.document['approval']['approved_exclusions'].remove('wind_direction')
        self.f.plan_path.write_text(json.dumps(self.document))
        with self.assertRaises(PlanBlocked):
            self.load()

    def test_draft_rule_does_not_approve_plan(self):
        self.document['approval_status'] = 'INACTIVE_NOT_APPROVED'
        self.f.plan_path.write_text(json.dumps(self.document))
        with self.assertRaisesRegex(PlanBlocked, 'Draft or unapproved'):
            self.load()

    def test_missing_invalid_or_nonzero_speed_never_removes_direction(self):
        for speed in (None, True, float('nan'), float('inf'), -1.):
            with self.subTest(speed=speed), self.assertRaises(PlanBlocked):
                reviewed_manual_policy(self.document, '1.f.(2)', {'wind_speed': speed})
        for speed in (0.000001, 1.):
            policy = reviewed_manual_policy(self.document, '1.f.(2)', {'wind_speed': speed})
            self.assertIn('wind_direction', policy.mandatory)

    def test_calm_rule_cannot_drop_wind_speed_check(self):
        self.document['checks'].pop('wind_speed')
        with self.assertRaisesRegex(PlanBlocked, 'wind-speed check'):
            reviewed_manual_policy(self.document, '1.f.(2)', {'wind_speed': 0.})

    def test_calm_rule_requires_explicit_exclusion(self):
        self.document['exclusions'].pop('wind_direction')
        with self.assertRaisesRegex(PlanBlocked, 'direction exclusion'):
            reviewed_manual_policy(self.document, '1.f.(2)', {'wind_speed': 0.})

    def test_rule_change_invalidates_document_approval(self):
        self.document['manual_policy']['wind_direction_applicability']['source'] = 'Changed'
        self.f.plan_path.write_text(json.dumps(self.document))
        with self.assertRaisesRegex(PlanBlocked, 'identity mismatch'):
            self.load()
