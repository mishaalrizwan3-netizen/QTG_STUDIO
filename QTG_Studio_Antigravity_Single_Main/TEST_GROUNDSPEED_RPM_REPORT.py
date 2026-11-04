"""Offline regressions for deceleration ground speed and RPM data transparency."""
import ast
import tempfile
import unittest
from pathlib import Path
import numpy as np
import pandas as pd
from engine_rules_v2 import frame_values
from plot_graphs import (_load_engine_dataframe, _rpm_signal_diagnostics,
                         _groups_for_test, _render_report, _series)

class DecelerationReportTests(unittest.TestCase):
    def test_ground_speed_is_independent_from_ias(self):
        f = frame_values({'p3d':{'ground_speed':0.0,'airspeed_indicated':82,
                'eng_rpms':[2700], 'prop_rpms':[2660]},'observation_elapsed_s':0})
        self.assertEqual(f['ground_speed_kt'],0.0)
        self.assertEqual(f['ias_kt'],82)
        src=(Path(__file__).parent/'Super_Mushak_Logger_V15_FULL_CAUSAL_STATE.py').read_text(encoding='utf-8')
        self.assertNotIn('gs_val = ias if',src)
        self.assertIn('"Ground Speed (kt)": round(gs_val, 2) if gs_val is not None else ""', src)

    def test_decel_vs_accel_groups(self):
        dec=_groups_for_test('deceleration')
        acc=_groups_for_test('acceleration')
        self.assertEqual([x[1] for x in dec[0][1]],["ground_speed_kt"])
        self.assertEqual([x[1] for x in acc[0][1]],['ias_kt','altitude_msl_ft','height_agl_ft'])
        self.assertEqual(dec[1:],acc[1:])

    def test_rpm_diagnostic_does_not_overwrite_zero_readings(self):
        hi=pd.DataFrame({'time_s':[0,.04,.08,.12], 'engine_rpm':[0,0,0,0],'prop_rpm':[0,0,0,0]})
        low=pd.DataFrame({'Engine 1 RPM':[1900,1850,1700,1500],
                          'Propeller 1 RPM':[1900,1850,1700,1500]})
        check=_rpm_signal_diagnostics(hi,low,'engine_rpm')
        self.assertEqual(check['state'],'zero_high_rate_legacy_disagrees')
        self.assertEqual(_series(hi,'engine_rpm').tolist(),[0.,0.,0.,0.])
        self.assertTrue(check['legacy_positive'])

    def test_zero_rpm_report_and_ground_speed_report(self):
        import fitz
        with tempfile.TemporaryDirectory() as td:
            root=Path(td)
            stamp='2026-10-23_11-30-00'
            base='Super_Mushak_manual_takeoff_data_'+stamp
            t=np.arange(0,21.1,1)
            throttle=np.where(t<5,100.,0.)
            rpm=np.where(t<5,2650.,950.)
            legacy=pd.DataFrame({
                'Time (s)':t,'Ground Speed (kt)':np.where(t<5,0.0,np.minimum(6.,(t-5)*.5)),
                'Indicated Airspeed (kts)':np.full(len(t),80.),
                'Throttle 1 Position (%)':throttle,
                'Engine 1 RPM':rpm,
                'Propeller 1 RPM':rpm,
                'Engine Manifold Pressure 1 (inHg)':np.where(t<5,27.,5.),
            })
            f=root/(base+'_legacy_plot.csv');legacy.to_csv(f,index=False)
            hi_t=np.arange(0,21.04,.04)
            ch=pd.DataFrame({'time_s':hi_t,'ground_speed_kt':np.maximum(0,(hi_t-5)*.5),
                             'throttle_pct':np.where(hi_t<5,100.,0.),
                             'engine_rpm':np.zeros(len(hi_t)), # artificial failure reproduction
                             'prop_rpm':np.zeros(len(hi_t)),
                             'manifold_inhg':np.where(hi_t<5,27.,5.),
                             'fuel_flow_pph':np.where(hi_t<5,90.,2.)})
            ch.to_csv(root/(base+'_engine_channels.csv'),index=False)
            df=_load_engine_dataframe(f)
            self.assertEqual(len(df),len(hi_t))
            self.assertAlmostEqual(float(df.ground_speed_kt.iloc[0]),0.)
            dest=root/'ground_dec_report.pdf'
            _render_report(f,ref_path=None,out_path=dest,titt_results=None,
                           metadata={'kind':'deceleration','event_time_s':5,'status':'complete'})
            pdf=fitz.open(dest); texts=[p.get_text() for p in pdf]
            self.assertEqual(len(pdf),4)
            self.assertIn('Ground Speed',texts[1]);self.assertIn('Ground Speed (kt)',texts[0])
            for term in ('Airspeed','Altitude MSL','Height AGL'):
                self.assertNotIn(term,texts[1]);self.assertNotIn(term,texts[0])
            self.assertIn('HIGH-RATE RPM = 0',texts[2])
            self.assertIn('legacy 1 Hz',texts[2])
            self.assertIn('Engine RPM',texts[2]);self.assertIn('Propeller RPM',texts[2])
            print('DECEL_TEST_REPORT=',dest,'pages=',len(pdf))

    def test_legacy_ground_speed_zero_is_not_ias(self):
        from engine_rules_v2 import frame_values
        row=frame_values({'p3d': {'ground_speed':0.0,'airspeed_indicated':60},'observation_elapsed_s':0.})
        self.assertEqual(row['ground_speed_kt'],0.0)
        self.assertNotEqual(row['ground_speed_kt'],row['ias_kt'])

if __name__=='__main__': unittest.main(verbosity=2)
