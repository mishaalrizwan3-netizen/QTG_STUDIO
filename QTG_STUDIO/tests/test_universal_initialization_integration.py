"""Mocked runtime integration only; no simulator, Qt loop or hardware I/O."""
import unittest
from unittest.mock import Mock, patch

import qtg_common_engine as engine
from backend.universal_initialization import State
import test_initialization_compatibility as compatibility
import test_universal_initialization as core_tests


class UniversalIntegrationTests(unittest.TestCase):
    def setUp(self):
        fixture = compatibility.InitializationCompatibilityTests()
        self.worker = fixture.worker()
        core = core_tests.UniversalInitializationTests()
        core.setUp()
        self.core = core
        self.worker._session_context = {'test_reference': '1.f.(2)',
                                        'universal_initialization_enabled': True}
        self.worker._universal_manager = core.manager
        self.worker._universal_plan = core.plan()
        core.adapter.cleanup_owner = 'legacy_runtime'
        self.worker._initialization_generation = 0
        def begin():
            self.worker._initialization_generation += 1
            self.worker._prepared = self.worker._preparing = False
        self.worker._begin_initialization = Mock(side_effect=begin)
        self.worker.stop_test = Mock(side_effect=self.worker.stop_worker_event.set)

    def start(self):
        with patch.object(engine, '_initialize_mode') as legacy:
            engine.initialize_mode(self.worker, 'manual')
            legacy.assert_not_called()
        return self.worker._universal_attempt_id

    def ready(self):
        attempt = self.start()
        self.core.adapter.feedback.return_value = self.core.frame()
        self.worker._initialization_stage(self.worker._initialization_generation, 'Ready')
        self.assertFalse(self.worker._prepared)
        self.core.now += .1
        self.core.adapter.feedback.return_value = self.core.frame(2)
        self.worker._poll_universal_initialization()
        self.assertTrue(self.worker._prepared)
        return attempt

    def test_flag_defaults_off_and_legacy_dispatch_is_unchanged_for_all_modes(self):
        for flag in (None, False):
            self.worker._session_context = {} if flag is None else {'universal_initialization_enabled': flag}
            for mode, method in (('manual', 'replay'), ('auto', 'replay'), ('auto', 'pid')):
                self.worker._control_method = method
                with patch.object(engine, '_initialize_mode') as legacy:
                    engine.initialize_mode(self.worker, mode, 'reference.csv')
                    legacy.assert_called_once_with(self.worker, mode, 'reference.csv')
        self.core.adapter.prepare.assert_not_called()
        self.core.adapter.cleanup.assert_not_called()
        self.worker.stop_test.assert_not_called()

    def test_enabled_requires_manager_plan_and_cleanup_ownership(self):
        self.worker._universal_plan = None
        with self.assertRaisesRegex(RuntimeError, 'reviewed plan'):
            self.start()
        self.assertFalse(self.worker._prepared)
        self.worker._universal_plan = self.core.plan()
        self.core.adapter.cleanup_owner = 'adapter'
        with self.assertRaisesRegex(RuntimeError, 'hardware ownership'):
            self.start()
        self.core.adapter.prepare.assert_not_called()

    def test_ready_and_recording_require_fresh_manager_authorization(self):
        attempt = self.ready()
        self.core.now += .1
        self.core.adapter.feedback.return_value = self.core.frame(3)
        self.worker.handle_command({'action': 'record'})
        self.worker.arm_recording.assert_called_once()
        self.assertEqual(self.core.manager.attempt_id, attempt)
        self.worker.stop_test.assert_not_called()

    def test_legacy_ready_cannot_bypass_enabled_manager(self):
        self.start()
        self.worker._update_stage_ui('Ready')
        self.worker.handle_command({'action': 'record'})
        self.worker.arm_recording.assert_not_called()
        self.assertFalse(self.worker._prepared)

    def test_invalid_plan_fails_without_fallback_and_cleans_once(self):
        from dataclasses import replace
        self.worker._universal_plan = replace(self.core.plan(), review_source='')
        self.start()
        self.assertEqual(self.core.manager.state, State.BLOCKED)
        self.worker._initialization_stage(self.worker._initialization_generation, 'Ready')
        self.worker.handle_command({'action': 'record'})
        self.worker.arm_recording.assert_not_called()
        self.core.adapter.cleanup.assert_called_once()
        self.worker.stop_test.assert_called_once()

    def test_repeated_initialization_ignores_prior_callbacks(self):
        first = self.ready()
        generation = self.worker._initialization_generation
        second = self.start()
        self.assertNotEqual(first, second)
        self.worker._initialization_stage(generation, 'Ready')
        self.worker._initialization_stage(generation, 'Failed', 'old failure')
        self.assertEqual(self.core.manager.state, State.PREPARING)
        self.assertFalse(self.worker._prepared)

    def test_cancellation_keeps_single_legacy_cleanup_owner(self):
        attempt = self.start()
        generation = self.worker._initialization_generation
        self.worker.handle_command({'action': 'stop'})
        self.worker._initialization_stage(generation, 'Ready')
        self.worker._poll_universal_initialization()
        self.assertEqual(self.core.manager.state, State.CANCELLED)
        self.core.adapter.cleanup.assert_called_once_with(attempt)
        self.worker.stop_test.assert_called_once()
        self.worker.arm_recording.assert_not_called()

    def test_timeout_and_preparation_failure_never_fall_back(self):
        for failure in ('timeout', 'preparation'):
            self.setUp()
            self.start()
            if failure == 'timeout':
                self.core.now += 11.
                self.worker._poll_universal_initialization()
            else:
                self.worker._initialization_stage(self.worker._initialization_generation, 'Failed', 'mock failure')
            self.assertEqual(self.core.manager.state, State.FAILED)
            self.assertFalse(self.worker._prepared)
            self.worker.stop_test.assert_called_once()

    def test_disconnection_revokes_recording_authorization(self):
        self.ready()
        self.core.now += .1
        self.core.adapter.feedback.return_value = self.core.frame(3, connected=False)
        self.worker.handle_command({'action': 'record'})
        self.worker.arm_recording.assert_not_called()
        self.assertEqual(self.core.manager.state, State.FAILED)
        self.worker.stop_test.assert_called_once()

    def test_cleanup_error_is_visible_in_runtime_details(self):
        self.start()
        self.core.adapter.cleanup.side_effect = RuntimeError('mock producer did not stop')
        self.core.now += 11.
        self.worker._poll_universal_initialization()
        self.assertIn('mock producer did not stop', self.worker._stage_details)
        self.assertFalse(self.worker._prepared)

    def test_runtime_failure_invalidates_manager_and_cannot_become_ready(self):
        self.ready()
        self.worker._update_stage_ui('Failed', 'runtime failure')
        self.assertEqual(self.core.manager.state, State.FAILED)
        self.worker._initialization_stage(self.worker._initialization_generation, 'Ready')
        self.assertFalse(self.worker._prepared)
        self.worker.handle_command({'action': 'record'})
        self.worker.arm_recording.assert_not_called()

    def test_t0_verification_stops_after_recording_is_authorized(self):
        self.ready()
        self.core.now += .1
        self.core.adapter.feedback.return_value = self.core.frame(3)
        self.worker.handle_command({'action': 'record'})
        self.core.adapter.feedback.reset_mock()
        self.worker._poll_universal_initialization()
        self.core.adapter.feedback.assert_not_called()
        self.worker.handle_command({'action': 'record'})
        self.worker.arm_recording.assert_called_once()


if __name__ == '__main__':
    unittest.main()
