"""Offline checks: no Air Manager/vJoy ownership or BRUNNER Yaw commands."""
import ast
import unittest
from pathlib import Path
from engine_airmanager_passive_yaw import EngineAirManagerPassiveYaw

ROOT = Path(__file__).resolve().parent
APP = (ROOT/'SMK_Engine_From_Climb.py').read_text('utf-8')

class FakeClimb:
    def _summarize_reference_replay(self):
        self.reference_has_native_cls = bool(self.reference_cls_axes)
        return self.reference_has_native_cls
    def _set_active_cls_axes(self, names):
        self.active_cls_axes = list(names)
        self.active_cls_axis_mask = tuple(names)

class App(EngineAirManagerPassiveYaw, FakeClimb):
    def __init__(self, mode='auto', refs=None):
        self.test_mode = mode
        self.reference_cls_axes = refs or []
        self.active_cls_axes = []

class PassiveYawTests(unittest.TestCase):
    def test_auto_excludes_yaw_reference_before_hardware(self):
        a = App(refs=['Yaw','Pitch','Roll'])
        self.assertTrue(a._summarize_reference_replay())
        self.assertEqual(a.reference_cls_axes, ['Pitch','Roll'])
        self.assertTrue(a.reference_has_native_cls)

    def test_only_yaw_cannot_arm_engine_automatic(self):
        a = App(refs=['Yaw'])
        self.assertFalse(a._summarize_reference_replay())
        self.assertEqual(a.reference_cls_axes, [])

    def test_auto_active_replay_never_includes_yaw(self):
        a = App()
        a._set_active_cls_axes(['Pitch','Roll','Yaw'])
        self.assertEqual(a.active_cls_axes, ['Pitch','Roll'])
        a._set_active_cls_axes(['Yaw'])
        self.assertEqual(a.active_cls_axes, [])

    def test_manual_does_not_override_brunner_axis_selection(self):
        a = App('manual', ['Pitch','Roll','Yaw'])
        self.assertTrue(a._summarize_reference_replay())
        self.assertEqual(a.reference_cls_axes, ['Pitch','Roll','Yaw'])
        a._set_active_cls_axes(['Pitch','Roll','Yaw'])
        self.assertEqual(a.active_cls_axes, ['Pitch','Roll','Yaw'])

    def test_engine_inherits_climb_but_no_vjoy_relay_or_yaw_bridge(self):
        tree=ast.parse(APP)
        cls=next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name=='EngineApp')
        self.assertEqual([ast.unparse(base) for base in cls.bases],
                         ['EngineAirManagerPassiveYaw','ClimbV15125BrunnerApp'])
        self.assertNotIn('qtg_vjoy_yaw', APP)
        self.assertNotIn('QTGYawSender', APP)
        self.assertNotIn('qtg_airmanager_relay', APP)
        self.assertNotIn('send_controls(rudder', APP)
        self.assertIn('yaw_qtg_replay_enabled=False', APP)
        self.assertIn('yaw_qtg_calibration_enabled=False', APP)

    def test_no_new_feeder_or_modified_airmanager_required(self):
        # This build excludes the previous experimental communication files.
        for filename in ('qtg_vjoy_simpc.py', 'qtg_vjoy_yaw.py',
                         'AirManager_Rudder_QTG.lua','qtg_yaw_config.json'):
            self.assertFalse((ROOT/filename).exists(), filename)

if __name__ == '__main__':
    unittest.main()
