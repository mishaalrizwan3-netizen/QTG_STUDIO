import sys,unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/"src"));sys.path.insert(0,str(ROOT/'src'))
from replay_support import ThrottleReplay
from p3d_simconnect_v15 import SimConnectClient
class ThrottleReplayTests(unittest.TestCase):
    def test_manual_change_to_full_power_is_replayed_at_its_time(self):
        replay=ThrottleReplay([0.,20.,30.,75.],[81.4,81.4,100.,100.])
        self.assertEqual(replay.command(0.),81.4)
        self.assertEqual(replay.command(20.),81.4)
        self.assertAlmostEqual(replay.command(25.),90.7)
        self.assertEqual(replay.command(30.),100.)
        self.assertEqual(replay.command(75.),100.)
    def test_zero_throttle_and_unchanged_command_refresh(self):
        replay=ThrottleReplay([0.,1.],[0.,0.])
        self.assertEqual(replay.command(0.),0.)
        self.assertIsNone(replay.command(.01))
        self.assertEqual(replay.command(.21),0.)
    def test_missing_invalid_or_backward_data_rejected(self):
        for times,values in [([],[]),([0.,1.],[81.4]),([0.,0.],[0.,100.]),([0.,1.],[0.,101.]),([0.,1.],[0.,float('nan')])]:
            with self.assertRaises(ValueError):ThrottleReplay(times,values)
        replay=ThrottleReplay([0.,1.],[0.,100.]);replay.command(.5)
        with self.assertRaises(ValueError):replay.command(.4)
    def test_driver_calls_only_authorized_throttle_export(self):
        bridge=Mock();bridge.SetReplayThrottleV1514.return_value=1
        fake=SimpleNamespace(_bridge=bridge)
        self.assertTrue(SimConnectClient.set_replay_throttle(fake,100.))
        bridge.SetReplayThrottleV1514.assert_called_once_with(100.)
        self.assertEqual(len(bridge.mock_calls),1)
        bridge.reset_mock()
        for value in [-1.,101.,float('nan')]:self.assertFalse(SimConnectClient.set_replay_throttle(fake,value))
        self.assertEqual(bridge.mock_calls,[])
if __name__=='__main__':unittest.main()
