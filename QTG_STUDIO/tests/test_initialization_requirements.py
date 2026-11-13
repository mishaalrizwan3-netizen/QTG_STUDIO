from dataclasses import replace
import unittest

from backend.catalog import TESTS
from backend.initialization_requirements import FIELDS, REQUIREMENTS, build_registry, validate_policy


class InitializationRequirementsTests(unittest.TestCase):
    def test_all_thirty_catalog_references_and_names(self):
        self.assertEqual(len(TESTS), 30)
        self.assertEqual(set(REQUIREMENTS), {test.reference for test in TESTS})
        for test in TESTS:
            policy = REQUIREMENTS[test.reference]
            self.assertEqual(policy.name, test.title)
            self.assertEqual(set(policy.requirements), set(FIELDS))
            self.assertEqual(validate_policy(policy), [], test.reference)

    def test_existing_engine_baselines_and_zero_values(self):
        decel = REQUIREMENTS['1.f.(2)']
        accel = REQUIREMENTS['1.f.(1)']
        self.assertEqual(decel.status, 'CONFIGURED')
        self.assertIs(decel.requirements['on_ground'].value, True)
        for key in ('ias_kt', 'flaps', 'pitch_deg', 'bank_deg'):
            self.assertEqual(decel.requirements[key].value, 0)
            self.assertIsNone(decel.requirements[key].missing_reason)
        self.assertEqual(decel.requirements['alt_ft'].value, 755.)
        self.assertEqual(decel.requirements['throttle'].value, 100.)
        self.assertIs(accel.requirements['on_ground'].value, False)
        self.assertEqual(accel.requirements['lat'].value, 32.224)
        self.assertEqual(accel.requirements['ias_kt'].value, 80.)
        self.assertTrue(any('conflict' in note for note in accel.notes))

    def test_rpm_tolerances_are_evidence_not_targets(self):
        for ref, tolerance in (('1.f.(1)', 400.), ('1.f.(2)', 60.)):
            item = REQUIREMENTS[ref].requirements['engine_rpm']
            self.assertIsNone(item.value)
            self.assertEqual(item.tolerance, tolerance)
            self.assertIn('startup_rpm_tolerance', item.tolerance_source)
            self.assertIn('snapshot', item.missing_reason)

    def test_unknown_tests_have_no_invented_defaults_or_modes(self):
        for ref, policy in REQUIREMENTS.items():
            if ref in ('1.c.(1)', '1.f.(1)', '1.f.(2)'):
                continue
            self.assertEqual(policy.status, 'UNCONFIGURED')
            self.assertEqual(policy.modes, ())
            self.assertEqual(len(policy.missing_requirements), len(FIELDS))
            self.assertTrue(all(item.value is None and item.tolerance is None
                                and item.missing_reason for item in policy.requirements.values()))

    def test_climb_is_partial_and_pid_defaults_are_scoped(self):
        policy = REQUIREMENTS['1.c.(1)']
        self.assertEqual(policy.status, 'UNCONFIGURED')
        self.assertEqual(policy.requirements['alt_ft'].value, 3000.)
        self.assertEqual(policy.requirements['throttle'].scope, 'PID default only')
        self.assertIn('heading_deg', policy.missing_requirements)
        self.assertIn('cg', policy.missing_requirements)
        self.assertIn('weight', policy.missing_requirements)

    def test_invalid_schema_sources_and_tolerances_are_reported(self):
        policy = REQUIREMENTS['1.f.(2)']
        for key, broken in (
                ('alt_ft', replace(policy.requirements['alt_ft'], source=None)),
                ('weight', replace(policy.requirements['weight'], missing_reason=None)),
                ('ias_kt', replace(policy.requirements['ias_kt'], unit='')),
                ('throttle', replace(policy.requirements['throttle'], tolerance_source=None)),
                ('throttle', replace(policy.requirements['throttle'], tolerance=-1)),
                ('alt_ft', replace(policy.requirements['alt_ft'], value=float('nan')))):
            with self.subTest(key=key, broken=broken):
                fields = dict(policy.requirements, **{key: broken})
                self.assertTrue(validate_policy(replace(policy, requirements=fields)))
        self.assertTrue(validate_policy(replace(policy, requirements={})))
        self.assertTrue(validate_policy(replace(policy, status='UNKNOWN')))
        self.assertTrue(validate_policy(replace(policy, modes=('unknown',))))

    def test_registry_builds_independent_snapshots_and_reports_gaps(self):
        fresh = build_registry()
        self.assertIsNot(fresh['1.f.(2)'].requirements, REQUIREMENTS['1.f.(2)'].requirements)
        self.assertEqual(sum(p.status == 'CONFIGURED' for p in fresh.values()), 2)
        self.assertEqual(sum(p.status == 'UNCONFIGURED' for p in fresh.values()), 28)
        self.assertIn('cg', fresh['1.f.(2)'].missing_requirements)


if __name__ == '__main__':
    unittest.main()
