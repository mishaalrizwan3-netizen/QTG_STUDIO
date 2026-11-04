"""Offline regression: engine BRUNNER is controlled by the inherited climb code.

These checks do not contact CLS2Sim, BRUNNER motors, or Prepar3D.
"""
import ast
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent
engine_src = (ROOT / 'SMK_Engine_From_Climb.py').read_text(encoding='utf-8')
climb_src = (ROOT / 'Super_Mushak_Logger_V15_FULL_CAUSAL_STATE.py').read_text(encoding='utf-8')
engine_tree = ast.parse(engine_src)
assert 'class EngineApp(EngineAirManagerPassiveYaw, ClimbV15125BrunnerApp)' in engine_src
climb_tree = ast.parse(climb_src)
engine_cls = next(n for n in engine_tree.body if isinstance(n, ast.ClassDef) and n.name == 'EngineApp')
engine_methods = {n.name: n for n in engine_cls.body if isinstance(n, ast.FunctionDef)}
climb_cls = next(n for n in climb_tree.body if isinstance(n, ast.ClassDef) and n.name == 'SuperMushakLoggerApp')
climb_methods = {n.name: n for n in climb_cls.body if isinstance(n, ast.FunctionDef)}

class ClimbParity(unittest.TestCase):
    def test_no_engine_specific_ap_alignment_tuning(self):
        for name in ('AUTO_BUMPLESS_START_TIMEOUT_SEC',
                     'AUTO_BUMPLESS_START_POSITION_TOL',
                     'V14_65_T0_POSITION_TOL',
                     'V14_64_T0_POSITION_TOL'):
            self.assertNotIn('climb.' + name + ' =', engine_src)
        self.assertIn('AUTO_BUMPLESS_START_POSITION_TOL = 0.08', climb_src)
        self.assertIn('V14_65_T0_POSITION_TOL = 0.030', climb_src)

    def test_engine_inherits_complete_climb_brunnner_logic(self):
        self.assertNotIn('_v1465_cls_only_t0_force_lock', engine_methods)
        for method in ('_open_and_arm_hardware', '_final_bumpless_cls_settle',
                       '_run_tracking_endpoint_precheck', '_selected_ap_config',
                       '_set_cls_position_forwarding_to_p3d',
                       '_v1458_runtime_bridge_health',
                       '_v1458_abort_runtime_bridge', '_v1465_cls_only_t0_force_lock',
                       '_on_telemetry_tick'):
            self.assertIn(method, climb_methods)
        self.assertFalse(any(isinstance(n, ast.ClassDef) and n.name == 'EngineTransport' for n in engine_tree.body))
        self.assertNotIn('PositionAlignment =', engine_src)

    def test_engine_never_requires_flight_control_equality(self):
        self.assertIn('self.engine_p3d_diagnostic_check.setChecked(False)', engine_src)
        self.assertIn('self.engine_p3d_diagnostic_run = False', engine_src)
        method = ast.get_source_segment(engine_src,engine_methods['_v1458_control_surfaces_match'])
        self.assertIn('flight-control/surface/trim equality checks OMITTED', method)
        self.assertIn('Throttle=',method)
        self.assertIn('RPM=',method)

    def test_no_second_brunnner_veto_before_recording(self):
        method = ast.get_source_segment(engine_src, engine_methods['arm_recording'])
        self.assertNotIn('Manual t=0=', method)
        self.assertNotIn('BRUNNER t=0 alignment failed', method)
        self.assertIn('super().arm_recording()', method)
        self.assertIn("self.engine_kind == 'deceleration'", method)

    def test_no_extra_engine_automatic_tracking_stop(self):
        m = ast.get_source_segment(engine_src, engine_methods['_on_telemetry_tick'])
        self.assertIn('return super()._on_telemetry_tick()', m)
        self.assertNotIn('abs(float(a) - float(b)) > 0.02', m)
        self.assertIn("if self.engine_kind=='acceleration'",m) # still stops engine on touchdown, never purely tracking
        self.assertIn('SID_AP_POSITION, target', climb_src)
        self.assertIn('CLS_TRACK_PRECHECK_TOL = 0.008', climb_src)
        self.assertIn('self.cls_replay_ap_off_counts[name] >= 20', climb_src)

    def test_engine_tests_remain(self):
        self.assertIn('class EngineClient', engine_src)
        self.assertIn("self.engine_kind='acceleration'", engine_src)
        self.assertIn("self.engine_kind=='deceleration'", engine_src)
        self.assertIn('_v1468_manual_throttle_at_elapsed', engine_src)
        self.assertIn('EngineTiming', engine_src)
        self.assertIn('stable_rpm_sample', engine_src)
        self.assertIn('TiTt', (ROOT/'SMK_Engine_TiTt_V2_1_3_Addon.py').name)

if __name__=='__main__':
    unittest.main(verbosity=2)
