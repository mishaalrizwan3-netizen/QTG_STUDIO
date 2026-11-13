import csv
import json
from pathlib import Path
import tempfile
import unittest

from backend.manual_reference_preflight import CHANNELS, analyze_reference


class ManualReferencePreflightTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / 'manual.csv'

    def write(self, change=None):
        rows = []
        for i in range(101):
            row = {column: 0. for column, *_ in CHANNELS.values()}
            row.update({'Time (s)': i*.1, 'Engine 1 RPM': 2700.,
                        'Propeller 1 RPM': 2700., 'Throttle 1 Position (%)': 100. if i < 60 else 5.,
                        'Engine Manifold Pressure (psi)': 13.8, 'Gross Wt (kg)': 850.})
            if change:
                change(i, row)
            rows.append(row)
        with self.path.open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        self.path.with_name('manual_init_conditions.json').write_text(json.dumps({
            'manual': {'rpm': 2700., 'telemetry': {'engine_rpm': 2700., 'prop_rpms': [2700.]}}}))

    def test_stable_window_and_event_provenance(self):
        self.write()
        result = analyze_reference(self.path)
        self.assertTrue(result['stable'])
        self.assertAlmostEqual(result['event_time_s'], 5.902)
        self.assertEqual(result['pre_event']['map']['unit'], 'psi')
        self.assertEqual(result['pre_event']['engine_rpm']['value'], 2700.)
        self.assertIn('CSV row 2', result['recording_start']['engine_rpm']['source'])
        json.dumps(result, allow_nan=False)

    def test_transient_start_is_distinct_from_stable_pre_event(self):
        self.write(lambda i, r: r.update({'Engine 1 RPM': 983. if i == 0 else 2700.}))
        result = analyze_reference(self.path)
        self.assertTrue(result['stable'])
        self.assertEqual(result['recording_start']['engine_rpm']['value'], 983.)
        self.assertIn('Transient recording-start condition: engine_rpm', result['issues'])
        self.assertIn('CSV/sidecar disagreement: engine_rpm', result['issues'])

    def test_unstable_window(self):
        self.write(lambda i, r: r.update({'Engine 1 RPM': 2200.+i*10.}))
        result = analyze_reference(self.path)
        self.assertFalse(result['stable'])
        self.assertFalse(result['pre_event']['engine_rpm']['stable'])

    def test_missing_values_are_not_replaced_by_sidecar_or_estimates(self):
        self.write(lambda i, r: r.update({'Engine 1 RPM': '', 'Engine Manifold Pressure (psi)': 'nan'}))
        result = analyze_reference(self.path)
        self.assertIsNone(result['recording_start']['engine_rpm']['value'])
        self.assertIsNone(result['pre_event']['map']['value'])
        self.assertEqual(result['sidecar_start']['engine_rpm']['value'], 2700.)
        self.assertFalse(result['stable'])

    def test_invalid_timestamps_are_not_silently_dropped(self):
        self.write(lambda i, r: r.update({'Time (s)': 0}) if i == 50 else None)
        result = analyze_reference(self.path)
        self.assertIsNone(result['event_time_s'])
        self.assertTrue(any('timestamps' in issue for issue in result['issues']))

    def test_missing_sidecar_is_diagnostic_only(self):
        self.write()
        result = analyze_reference(self.path, sidecar_path=self.path.with_name('absent.json'))
        self.assertTrue(result['stable'])
        self.assertTrue(any('sidecar' in issue for issue in result['issues']))

    def test_configurable_window_and_ranges(self):
        self.write(lambda i, r: r.update({'Engine 1 RPM': 2200.+i*10.}))
        result = analyze_reference(self.path, window_s=1., range_limits={'engine_rpm': 200.})
        self.assertTrue(result['pre_event']['engine_rpm']['stable'])
        with self.assertRaises(ValueError):
            analyze_reference(self.path, window_s=0)

    def test_sparse_window_cannot_be_stable(self):
        self.write(lambda i, r: r.update({'Time (s)': i*.5}))
        result = analyze_reference(self.path)
        self.assertFalse(result['stable'])
        self.assertIn('Incomplete or sparse pre-event window', result['issues'])

    def test_november_six_deceleration_regression(self):
        path = Path('D:/QTG_Test_Results/1_f_2_Engine_deceleration/1_f_2_Engine_deceleration_manual_2026-11-06_15-45-07.csv')
        if not path.is_file():
            self.skipTest('External November 6 reference unavailable')
        result = analyze_reference(path)
        self.assertEqual(result['recording_start']['engine_rpm']['value'], 983.)
        self.assertAlmostEqual(result['snapshot_rpm_target']['value'], 983.4428053, places=5)
        self.assertAlmostEqual(result['event_time_s'], 7.704337494, places=7)
        self.assertGreater(result['pre_event']['engine_rpm']['value'], 2650.)
        self.assertIn('Transient recording-start condition: engine_rpm', result['issues'])
        self.assertFalse(result['pre_event']['ground_speed']['stable'])


if __name__ == '__main__':
    unittest.main()
