"""Exercise the production queue ordering with existing mocked runtime fixtures."""
from pathlib import Path
import unittest
from unittest.mock import patch

import qtg_common_engine as engine
from backend.universal_initialization import State
import test_universal_both_modes as fixtures


class CommandQueueTests(unittest.TestCase):
    def setUp(self):
        self.f = fixtures.BothModesTests(); self.f.setUp()
        self.addCleanup(self.f.doCleanups)
        self.w = self.f.w

    def queued_commands(self, mode='auto'):
        if mode == 'auto':
            self.f.automatic()
            adapter = self.f.ready()
            revision = 'reviewed-reference-revision'
        else:
            adapter = self.f.f.ready()
            revision = self.w._session_context.get('configuration_revision')
        self.f.f.now += .1
        adapter._frame = self.f.f.frame(4)
        return [(Path('001_record.json'), dict(action='record', configuration_revision=revision)),
                (Path('002_edit.json'), dict(action='invalidate_initialization', configuration_revision=revision))]

    def test_automatic_reference_change_precedes_earlier_queued_record(self):
        commands = self.queued_commands()
        ordered = engine.prioritize_initialization_commands(self.w, commands)
        self.assertEqual(ordered[0][1]['action'], 'invalidate_initialization')
        for _, command in ordered: self.w.handle_command(command)
        self.w.arm_recording.assert_not_called()
        self.assertEqual(self.w._universal_manager.state, State.CANCELLED)
        self.assertTrue(self.w._session_context['gui_configuration_invalidated'])

    def test_manual_retains_invalidation_priority(self):
        commands = self.queued_commands('manual')
        for _, command in engine.prioritize_initialization_commands(self.w, commands):
            self.w.handle_command(command)
        self.w.arm_recording.assert_not_called()
        self.assertEqual(self.w._universal_manager.state, State.CANCELLED)

    def test_stale_record_revision_cannot_authorize_automatic(self):
        self.queued_commands()
        self.w.handle_command(dict(action='record', configuration_revision='old-revision'))
        self.w.arm_recording.assert_not_called()
        self.assertEqual(self.w._universal_manager.state, State.READY)

    def test_reference_edit_after_ready_blocks_subsequent_record(self):
        commands = self.queued_commands()
        self.w.handle_command(commands[1][1])
        self.w.handle_command(commands[0][1])
        self.w.arm_recording.assert_not_called()

    def test_legacy_fifo_and_input_list_remain_unchanged(self):
        commands = [(None, dict(action='record')), (None, dict(action='invalidate_initialization'))]
        with patch.object(engine, 'universal_initialization_enabled', return_value=False):
            self.assertEqual(engine.prioritize_initialization_commands(self.w, commands), commands)
        self.assertEqual(commands[0][1]['action'], 'record')

    def test_superseded_invalidation_does_not_cancel_current_attempt(self):
        commands = self.queued_commands()
        commands[1][1]['configuration_revision'] = 'previous-session'
        for _, command in engine.prioritize_initialization_commands(self.w, commands):
            self.w.handle_command(command)
        self.w.arm_recording.assert_called_once()
        self.assertEqual(self.w._universal_manager.state, State.RECORDING)


if __name__ == '__main__': unittest.main()
