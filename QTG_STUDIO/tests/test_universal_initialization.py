from dataclasses import replace
import unittest
from unittest.mock import Mock

from backend.initialization_requirements import REQUIREMENTS
from backend.universal_initialization import Feedback, InitializationManager, Measurement, Plan, State
from _initialization_plan_fixtures import approve_plan


class UniversalInitializationTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.
        self.adapter = Mock()
        self.manager = InitializationManager(self.adapter, clock=lambda: self.now)

    def plan(self, reference='1.f.(2)', mode='manual'):
        policy = REQUIREMENTS[reference]
        # Synthetic offline review: excludes other fields explicitly, not live approval.
        check = replace(policy.requirements['ias_kt'], value=0., tolerance=0.,
                        tolerance_source='synthetic offline exact-match fixture',
                        source='synthetic fixture', missing_reason=None, scope=mode)
        return approve_plan(Plan(reference, mode, {'ias_kt': check},
                    {key: 'Synthetic offline fixture only; not approved for live execution'
                     for key in policy.requirements if key != 'ias_kt'},
                    'Offline test review only', 10., 1., 2))

    def frame(self, sequence=1, value=0., connected=True):
        check = self.plan().checks['ias_kt']
        return Feedback(connected, sequence, self.now, {
            'ias_kt': Measurement(value, check.unit, check.telemetry)})

    def begin(self):
        attempt = self.manager.start(self.plan())
        self.assertTrue(self.manager.prepared(attempt))
        return attempt

    def ready(self):
        attempt = self.begin()
        self.adapter.feedback.return_value = self.frame()
        self.assertFalse(self.manager.poll(attempt))
        self.now += .1
        self.adapter.feedback.return_value = self.frame(2)
        self.assertTrue(self.manager.poll(attempt))
        return attempt

    def test_success_requires_verification_and_fresh_recording_authorization(self):
        attempt = self.ready()
        self.now += .1
        self.adapter.feedback.return_value = self.frame(3)
        self.assertTrue(self.manager.authorize_recording(attempt))
        self.assertEqual([state for _, state in self.manager.history],
                         [State.VALIDATING, State.PREPARING, State.VERIFYING, State.READY])

    def test_recording_blocked_before_verification(self):
        attempt = self.manager.start(self.plan())
        self.assertFalse(self.manager.authorize_recording(attempt))
        self.manager.prepared(attempt)
        self.assertFalse(self.manager.authorize_recording(attempt))

    def test_missing_or_unreviewed_requirements_and_invalid_mode(self):
        for plan in (replace(self.plan(), checks={}), replace(self.plan(), review_source=''),
                     replace(self.plan(), exclusions={}), replace(self.plan(), reference='unknown'),
                     replace(self.plan(), mode='pid'), replace(self.plan(), reference='1.c.(1)')):
            with self.subTest(plan=plan):
                self.setUp()
                self.manager.start(plan)
                self.assertEqual(self.manager.state, State.BLOCKED)
                self.adapter.prepare.assert_not_called()
                self.adapter.cleanup.assert_called_once()

    def test_baseline_is_not_a_fully_resolved_plan(self):
        for reference in ('1.f.(1)', '1.f.(2)'):
            self.setUp()
            plan = replace(self.plan(reference), checks=REQUIREMENTS[reference].requirements, exclusions={})
            self.manager.start(plan)
            self.assertEqual(self.manager.state, State.BLOCKED)

    def test_missing_nonfinite_stale_and_disconnected_feedback(self):
        for kind in ('missing', 'nan', 'old', 'future', 'disconnected', 'units', 'mismatch'):
            with self.subTest(kind=kind):
                self.setUp()
                attempt = self.begin()
                frame = self.frame()
                if kind == 'missing': frame = replace(frame, values={})
                if kind == 'nan': frame = self.frame(value=float('nan'))
                if kind == 'old': frame = replace(frame, measured_at=self.now-2)
                if kind == 'future': frame = replace(frame, measured_at=self.now+1)
                if kind == 'disconnected': frame = self.frame(connected=False)
                if kind == 'units': frame = replace(frame, values={'ias_kt': Measurement(0., 'm/s', 'airspeed_indicated')})
                if kind == 'mismatch': frame = self.frame(value=5.)
                self.adapter.feedback.return_value = frame
                self.assertFalse(self.manager.poll(attempt))
                self.assertEqual(self.manager.state, State.FAILED)
                self.adapter.cleanup.assert_called_once_with(attempt)

    def test_duplicate_feedback_cannot_establish_readiness(self):
        attempt = self.begin()
        self.adapter.feedback.return_value = self.frame()
        self.manager.poll(attempt)
        self.manager.poll(attempt)
        self.assertEqual(self.manager.state, State.FAILED)

    def test_failed_preparation_and_exception(self):
        attempt = self.manager.start(self.plan())
        self.manager.prepared(attempt, success=False, reason='mock preparation failed')
        self.assertEqual(self.manager.state, State.FAILED)
        self.assertFalse(self.manager.prepared(attempt))
        self.adapter.prepare.side_effect = RuntimeError('mock adapter failed')
        self.manager.start(self.plan())
        self.assertEqual(self.manager.reason, 'mock adapter failed')

    def test_cancellation_and_late_callback(self):
        attempt = self.manager.start(self.plan())
        self.assertTrue(self.manager.cancel(attempt))
        self.assertFalse(self.manager.prepared(attempt))
        self.assertFalse(self.manager.authorize_recording(attempt))
        self.assertEqual(self.manager.state, State.CANCELLED)
        self.adapter.cleanup.assert_called_once_with(attempt)

    def test_timeouts_before_preparation_completion_and_during_verification(self):
        for prepared in (False, True):
            self.setUp()
            attempt = self.manager.start(self.plan())
            if prepared: self.manager.prepared(attempt)
            self.now += 10.
            self.manager.poll(attempt)
            self.assertEqual(self.manager.state, State.FAILED)
            self.assertFalse(self.manager.prepared(attempt))

    def test_repeated_attempts_are_unique_and_old_messages_cannot_change_state(self):
        first = self.ready()
        second = self.manager.start(self.plan())
        self.assertNotEqual(first, second)
        self.assertFalse(self.manager.prepared(first))
        self.assertFalse(self.manager.cancel(first))
        self.assertFalse(self.manager.poll(first))
        self.assertFalse(self.manager.authorize_recording(first))
        self.assertEqual(self.manager.state, State.PREPARING)
        self.adapter.cleanup.assert_called_once_with(first)

    def test_cleanup_failure_visible_and_blocks_reuse(self):
        attempt = self.manager.start(self.plan())
        self.adapter.cleanup.side_effect = RuntimeError('release failed')
        self.manager.cancel(attempt)
        self.assertIn('release failed', self.manager.cleanup_errors[0])
        self.assertFalse(self.manager.authorize_recording(attempt))
        self.adapter.prepare.reset_mock()
        self.manager.start(self.plan())
        self.assertEqual(self.manager.state, State.FAILED)
        self.adapter.prepare.assert_not_called()

    def test_independent_modes_and_instances_preserve_units_zero_and_tolerances(self):
        for reference, mode in (('1.f.(1)', 'manual'), ('1.f.(2)', 'recorded_replay')):
            manager = InitializationManager(Mock(), clock=lambda: self.now)
            manager.start(self.plan(reference, mode))
            self.assertEqual(manager.state, State.PREPARING)
            plan = manager.adapter.prepare.call_args.args[1]
            self.assertEqual(plan.checks['ias_kt'].value, 0.)
            self.assertEqual(plan.mode, mode)
        self.assertIsNone(self.manager.state)
        plan = self.plan()
        rpm = replace(REQUIREMENTS['1.f.(2)'].requirements['engine_rpm'], value=2700.,
                      source='synthetic', missing_reason=None, scope='recorded_replay', tolerance=61.)
        self.manager.start(approve_plan(replace(plan, mode='recorded_replay', checks={'engine_rpm': rpm},
            exclusions={k: 'fixture' for k in REQUIREMENTS['1.f.(2)'].requirements if k != 'engine_rpm'})))
        self.assertEqual(self.manager.state, State.BLOCKED)

    def test_readiness_revoked_if_feedback_expires(self):
        attempt = self.ready()
        self.now += 2.
        self.assertFalse(self.manager.authorize_recording(attempt))
        self.assertEqual(self.manager.state, State.FAILED)

    def test_feedback_from_before_preparation_is_not_verification(self):
        attempt = self.begin()
        self.adapter.feedback.return_value = replace(self.frame(), measured_at=self.now-.1)
        self.assertFalse(self.manager.poll(attempt))
        self.assertEqual(self.manager.state, State.FAILED)

    def test_feedback_call_cannot_overrun_verification_deadline(self):
        attempt = self.begin()
        def delayed_feedback(attempt_id):
            self.now += 11.
            return self.frame()
        self.adapter.feedback.side_effect = delayed_feedback
        self.assertFalse(self.manager.poll(attempt))
        self.assertIn('timeout', self.manager.reason)


if __name__ == '__main__':
    unittest.main()
