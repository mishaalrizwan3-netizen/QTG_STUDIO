"""Pure offline tests: nothing connects to P3D or BRUNNER hardware."""
import csv
import json
import math
import tempfile
import unittest
from pathlib import Path
from SMK_Engine_TiTt_V2_1_3_Addon import measure, evaluate_engine_run, compare


def synthetic(path, mode='acceleration', delay=.10, tau=1.2, interval=.04, with_companion=True,
              flat=False):
    if with_companion:
        high = path.with_name(path.stem+'_engine_channels.csv')
    else:
        high = path
    # Deliberately contrasting low-rate data; confirms precedence of high-rate channel export.
    if with_companion:
        with path.open('w', newline='') as f:
            w=csv.writer(f);w.writerow(['time_s','throttle_pct','manifold_inhg'])
            for i in range(21):w.writerow([i, 10 if mode=='acceleration' else 98, 30])
    n=int(20/interval)+1
    with high.open('w',newline='') as f:
        w=csv.writer(f);w.writerow(['time_s','throttle_pct','manifold_inhg','engine_rpm'])
        for i in range(n):
            t=i*interval
            st=0.0 if t<5+delay else 1-math.exp(-(t-5-delay)/tau)
            if flat: st=0
            throttle=(10 if t<5 else 100) if mode=='acceleration' else (98 if t<5 else 0)
            mapval=12+16*st if mode=='acceleration' else 29-17*st
            rpm=800+1500*st if mode=='acceleration' else 2600-1700*st
            w.writerow([round(t,5), throttle, round(mapval,6), round(rpm,3)])
    return high


class TiTtIntegrationTests(unittest.TestCase):
    def test_acceleration_uses_high_rate_data_and_detects_times(self):
        with tempfile.TemporaryDirectory() as td:
            p=Path(td)/'manual.csv';high=synthetic(p)
            m, _=measure(p,'Manual','acceleration')
            self.assertEqual(m.status,'MEASURED',m.warnings)
            self.assertEqual(m.columns['source_file'],str(high))
            self.assertTrue(0.1 < m.Ti_s < .5, m.Ti_s)
            self.assertTrue(2.5 < m.Tt_s < 3.5, m.Tt_s)
    def test_deceleration(self):
        with tempfile.TemporaryDirectory() as td:
            p=Path(td)/'dec.csv';synthetic(p,mode='deceleration')
            m, _=measure(p,'Manual','deceleration')
            self.assertEqual(m.status,'MEASURED',m.warnings)
            self.assertTrue(2.5<m.Tt_s<3.5)
    def test_manual_only_is_not_automatic(self):
        with tempfile.TemporaryDirectory() as td:
            p=Path(td)/'manual.csv';synthetic(p)
            d=evaluate_engine_run(p, mode='acceleration', out_dir=Path(td))
            self.assertIn('Manual',d['measurements'])
            self.assertNotIn('Automatic',d['measurements'])
            self.assertEqual(d['qualification_status'],'NOT_ASSESSED')
            self.assertTrue((Path(td)/'manual_TiTt_results.json').exists())
            self.assertTrue((Path(td)/'manual_TiTt_overlay.png').exists())
    def test_automatic_repeatability_compare(self):
        with tempfile.TemporaryDirectory() as td:
            man=Path(td)/'manual.csv';auto=Path(td)/'auto.csv'
            synthetic(man);synthetic(auto,delay=.13)
            report=evaluate_engine_run(auto, manual_path=man, mode='acceleration', out_dir=Path(td))
            self.assertIn(report['status'],['WITHIN_TOLERANCE','OUTSIDE_TOLERANCE'])
            self.assertEqual(report['qualification_status'],'NOT_ASSESSED')
            self.assertEqual(report['comparisons'][0]['assessment_type'],'manual_vs_automatic_engineering')
    def test_1_hz_only_does_not_pass(self):
        with tempfile.TemporaryDirectory() as td:
            p=Path(td)/'slow.csv';synthetic(p,interval=1.,with_companion=False)
            m,_=measure(p,'Manual','acceleration')
            self.assertEqual(m.status,'INVALID')
            self.assertTrue(any('TEMPORAL RESOLUTION' in s for s in m.warnings))
    def test_flat_power_does_not_pass(self):
        with tempfile.TemporaryDirectory() as td:
            p=Path(td)/'flat.csv';synthetic(p,flat=True)
            m,_=measure(p,'Manual','acceleration')
            self.assertEqual(m.status,'INVALID')
    def test_legacy_plot_uses_correct_companion(self):
        with tempfile.TemporaryDirectory() as td:
            p=Path(td)/'manual.csv';high=synthetic(p)
            legacy=p.with_name('manual_legacy_plot.csv')
            legacy.write_text('time_s,throttle_pct,manifold_inhg\n0,10,10\n1,10,10\n')
            m,_=measure(legacy,'Manual','acceleration')
            self.assertEqual(m.status,'MEASURED')
            self.assertEqual(m.columns['source_file'],str(high))
    def test_invalid_measurement_never_passes_comparison(self):
        with tempfile.TemporaryDirectory() as td:
            low=Path(td)/'low.csv';ok=Path(td)/'ok.csv'
            synthetic(low,interval=1.,with_companion=False);synthetic(ok)
            bad,_=measure(low,'Auto','acceleration');good,_=measure(ok,'Manual','acceleration')
            self.assertEqual(compare(bad,good,'engineering')['status'],'INVALID')

if __name__=='__main__':unittest.main()
