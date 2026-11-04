import tempfile
import unittest
from pathlib import Path
import numpy as np
import pandas as pd
from plot_graphs import _load_engine_dataframe, _resolve_event_time


class TimebaseRepairTests(unittest.TestCase):
    def test_legacy_one_hz_never_truncates_high_rate_engine(self):
        with tempfile.TemporaryDirectory() as tmp:
            name = 'Super_Mushak_auto_takeoff_data_test'
            parent = Path(tmp)
            t1 = np.arange(22, dtype=float)
            pd.DataFrame({'Time (s)':t1, 'Joystick Pitch Position (mm)': t1 * 2,
                          'Joystick Pitch Force (N)': t1 / 10}).to_csv(
                              parent/(name+'_legacy_plot.csv'), index=False)
            tfast = np.arange(.15,21,.04)
            pd.DataFrame({'time_s':tfast, 'throttle_pct':np.where(tfast<5.4,5,100),
                          'engine_rpm':np.where(tfast<5.4,1620,2300)}).to_csv(
                              parent/(name+'_engine_channels.csv'),index=False)
            path = parent/(name+'_legacy_plot.csv')
            result = _load_engine_dataframe(path)
            self.assertEqual(len(result), len(tfast))
            self.assertAlmostEqual(float(result.time_s.iloc[-1]), float(tfast[-1]))
            self.assertAlmostEqual(float(result['Joystick Pitch Position (mm)'].iloc[100]),
                                   float(2*tfast[100]),delta=.001)
            event = _resolve_event_time(path, result)
            self.assertGreater(float(result.time_s.max()-event), 15)
            self.assertLess(float(result.time_s.min()-event), -5)

if __name__ == '__main__':
    unittest.main()
