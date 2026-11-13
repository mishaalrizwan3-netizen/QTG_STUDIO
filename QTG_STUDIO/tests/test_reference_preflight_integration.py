"""Offline presentation/selection tests; no Tk window or hardware is created."""
import ast
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from frontend import reference_preflight as view


class ReferencePreflightIntegrationTests(unittest.TestCase):
    def report(self, stable=True):
        return dict(csv='selected.csv', stable=stable, event_time_s=7.7, window_s=2.,
            provenance_warning='Legacy exporter defaults may be unreliable.',
            recording_start={'engine_rpm': dict(value=983., unit='rpm', source='CSV row 2', time_s=.12)},
            event_start={'engine_rpm': dict(value=2705., source='CSV row 380', time_s=7.62)},
            pre_event={'engine_rpm': dict(value=2701., stable=stable, range=25., range_limit=30.)},
            sidecar_start={'engine_rpm': dict(value=983.44, source='sidecar rpm')},
            discrepancies={'engine_rpm': dict(pre_event_minus_start=1718.)}, issues=[])

    def test_both_engine_selections_delegate_to_existing_validator(self):
        for ref, mode in view.MODES.items():
            with self.subTest(ref=ref), patch.object(view, 'analyze_reference', return_value=self.report()) as analyze:
                text = view.diagnostic_text(ref, 'selected.csv')
                analyze.assert_called_once_with('selected.csv', mode=mode)
                self.assertIn('Preflight: STABLE', text)
                self.assertIn('readiness are unchanged', text)

    def test_report_distinguishes_measured_event_sample_and_window_median(self):
        text = view.format_report(self.report())
        for value in ('recording start=983', 'event-start sample=2705',
                      'pre-event median=2701', 'median minus recording start=1718',
                      'CSV row 380', 'sidecar rpm', '[rpm]', 'BEFORE onset'):
            self.assertIn(value, text)

    def test_unstable_reason_is_displayed(self):
        report = self.report(False)
        report['issues'] = ['Unstable pre-event measurement: engine_rpm',
                            'Unstable pre-event measurement: ground_speed']
        text = view.format_report(report)
        self.assertIn('Preflight: UNSTABLE', text)
        self.assertIn('ground_speed', text)
        self.assertIn('Window: unstable', text)

    def test_missing_and_sparse_data_are_insufficient(self):
        for kind in ('missing', 'sparse', 'no_event'):
            report = self.report(False)
            if kind == 'missing':
                report['pre_event']['engine_rpm'].update(value=None, stable=None)
            elif kind == 'sparse':
                report['issues'] = ['Incomplete or sparse pre-event window']
            else:
                report.update(pre_event={}, event_start={}, stable=None, event_time_s=None)
            self.assertIn('INSUFFICIENT DATA', view.format_report(report))

    def test_unrelated_tests_and_empty_selection_do_not_analyze(self):
        with patch.object(view, 'analyze_reference') as analyze:
            self.assertIn('only', view.diagnostic_text('1.c.(1)', 'selected.csv'))
            self.assertIn('INSUFFICIENT DATA', view.diagnostic_text('1.f.(2)', ''))
            analyze.assert_not_called()

    def test_unreadable_reference_is_informational(self):
        with patch.object(view, 'analyze_reference', side_effect=OSError('file unavailable')):
            self.assertIn('INSUFFICIENT DATA', view.diagnostic_text('1.f.(2)', 'missing.csv'))

    def test_optional_gui_action_reads_current_selection_without_runtime_commands(self):
        # Compile just the callback to avoid importing/starting the application.
        path = Path(__file__).resolve().parents[1] / 'frontend/main.py'
        tree = ast.parse(path.read_text(encoding='utf-8'))
        app = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == 'App')
        callback = next(node for node in app.body if isinstance(node, ast.FunctionDef)
                        and node.name == '_inspect_common_reference')
        tk, ttk = Mock(), Mock()
        scope = {'tk': tk, 'ttk': ttk}
        exec(compile(ast.Module(body=[callback], type_ignores=[]), str(path), 'exec'), scope)
        host = SimpleNamespace(_common_display_spec=SimpleNamespace(reference='1.f.(2)'),
                               _common_reference_var=Mock(), _common_command=Mock())
        host._common_reference_var.get.return_value = 'current.csv'
        with patch.object(view, 'diagnostic_text', return_value='diagnostic') as diagnose:
            scope['_inspect_common_reference'](host)
        diagnose.assert_called_once_with('1.f.(2)', 'current.csv')
        tk.Text.return_value.insert.assert_called_once_with('1.0', 'diagnostic')
        tk.Text.return_value.configure.assert_called_once_with(state='disabled')
        host._common_command.assert_not_called()


if __name__ == '__main__':
    unittest.main()
