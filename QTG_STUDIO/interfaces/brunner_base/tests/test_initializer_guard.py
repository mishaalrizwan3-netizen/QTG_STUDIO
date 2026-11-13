import unittest,sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/"src"))
sys.path.insert(0,str(ROOT/'src'))
from replay_support import validate_initial_airspeed
import Super_Mushak_Logger_GUI as gui

class InitializationTests(unittest.TestCase):
    def test_previous_good_airspeed_is_accepted(self):
        self.assertEqual(validate_initial_airspeed({'airspeed_indicated':100.84},100.98),100.84)
    def test_reported_regression_is_rejected(self):
        with self.assertRaisesRegex(ValueError,'59.97'):
            validate_initial_airspeed({'airspeed_indicated':59.97},100.98)
    def test_missing_or_invalid_speed_is_rejected(self):
        for data in ({},{'airspeed_indicated':float('nan')}):
            with self.assertRaises(ValueError): validate_initial_airspeed(data,100.98)
    def test_replay_never_unpauses_with_regressed_airspeed(self):
        driver=Mock()
        hardware=Mock()
        fake=SimpleNamespace(p3d=driver,brunner=hardware,is_recording=True,
            ref_p_pos=[-.02],ref_r_pos=[0.],ref_y_pos=[0.],replay_axis_offsets=(0.,0.,0.),
            ref_manual_snapshot={'airspeed':100.98},_settle_reference_controls=lambda:None,
            _refresh_paused_telemetry=lambda:{'airspeed_indicated':59.97},
            sig_update_stage=Mock())
        gui.SuperMushakLoggerApp._auto_replay_loop(fake)
        driver.unpause_sim.assert_not_called()
        hardware.release_controls.assert_called_once()
        self.assertFalse(fake.is_recording)
        self.assertEqual(fake.sig_update_stage.emit.call_args.args[0],gui.STAGE_CONNECTION_ERROR)

if __name__=='__main__': unittest.main(verbosity=2)
