import csv
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

from backend.telemetry_diagnostic import DiagnosticMonitor, SessionSummary


class DiagnosticSummaryTests(unittest.TestCase):
    def row(self, index, value):
        return dict(observed_utc=f't{index:02}',channel='engine_rpm',value=value,unit='rpm',
                    source='eng_rpms[0]',status='UNVERIFIED',sequence=index,
                    cache_copy_monotonic=index,native_acquired_at=None,freshness='UNVERIFIED')

    def test_63_snapshots_54_available_nine_missing_retains_session_evidence(self):
        summary=SessionSummary()
        for index in range(63):
            summary.observe([self.row(index,983. if index<54 else None)],
                'CACHE_AVAILABLE' if index<54 else 'STALE_OR_REPEATED_CACHE',
                {'worker_stage':'Recording' if index<54 else 'Generating reports'})
        report=summary.document()
        self.assertEqual(report['snapshot_count'],63)
        self.assertEqual(report['p3d_available_snapshots'],54)
        self.assertEqual(report['p3d_missing_snapshots'],9)
        engine=report['channels']['engine_rpm']
        self.assertEqual(engine['first_valid_reading']['observed_utc'],'t00')
        self.assertEqual(engine['last_valid_reading']['observed_utc'],'t53')
        self.assertEqual(engine['verified_sample_count'],0)
        self.assertEqual(engine['missing_intervals'][0]['samples'],9)
        self.assertEqual(len(report['connection_and_availability_transitions']),2)

    def test_missing_interval_closes_on_recovery(self):
        summary=SessionSummary()
        for index,value in enumerate((None,1.,None,2.)):
            summary.observe([self.row(index,value)],'UNKNOWN')
        self.assertEqual([gap['open'] for gap in summary.channels['engine_rpm']['missing_intervals']],[False,False])

    def test_access_denied_retry_private_temp_and_worker_file_untouched(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            worker=root/'worker_status.json'
            worker.write_text('ordinary worker status')
            monitor=DiagnosticMonitor(Mock(),root,root/'worker_status.telemetry.json')
            original=Path.replace
            calls=[]
            def replace(path,target):
                calls.append(path)
                if len(calls)<3: raise PermissionError(5,'Access is denied')
                return original(path,target)
            with patch.object(Path,'replace',replace): monitor._publish({'active':True})
            self.assertEqual(monitor.publication_retries,2)
            self.assertEqual(worker.read_text(),'ordinary worker status')
            self.assertTrue(all(path.name.startswith('worker_status.telemetry.json.') for path in calls))
            self.assertFalse(list(root.glob('*.tmp')))

    def test_permanent_status_failure_does_not_lose_csv_or_summary(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            monitor=DiagnosticMonitor(Mock(),root,root/'status.json')
            def sample():
                monitor.stop(reason='worker_shutdown')
                return [self.row(1,983.)],[]
            monitor.sample=sample
            with patch.object(monitor,'_publish',side_effect=PermissionError(5,'Access is denied')):
                monitor.start(); monitor.thread.join(2)
            report=json.loads(Path(monitor.outputs['summary']).read_text())
            self.assertEqual(report['end_reason'],'worker_shutdown')
            self.assertEqual(report['session']['p3d_available_snapshots'],1)
            self.assertGreater(report['status_publication_failures'],0)
            self.assertTrue(Path(monitor.outputs['csv']).is_file())

    def test_supplied_recording_regression(self):
        path=Path('D:/QTG_Test_Results/sessions/telemetry_diagnostics/20261110T075149Z_a138ce34.csv')
        if not path.exists(): self.skipTest('Supplied live evidence unavailable')
        groups={}
        with path.open(newline='',encoding='utf-8-sig') as stream:
            for row in csv.DictReader(stream):
                groups.setdefault(row['observed_utc'],[]).append(row)
        summary=SessionSummary()
        for rows in groups.values(): summary.observe(rows,'RECORDED_SOURCE_STATE_UNKNOWN')
        report=summary.document()
        self.assertEqual(report['snapshot_count'],63)
        self.assertEqual(report['p3d_available_snapshots'],54)
        self.assertEqual(report['p3d_missing_snapshots'],9)
        self.assertEqual(report['channels']['engine_rpm']['missing_intervals'][-1]['samples'],9)
        self.assertEqual(report['channels']['brunner.pitch']['verified_sample_count'],0)


if __name__=='__main__': unittest.main()
