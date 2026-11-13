"""Engine-only offline tests (NO P3D/BRUNNER motors required)."""
import ast
import math
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent
SRC = (ROOT / 'SMK_Engine_From_Climb.py').read_text('utf8')
LOGGER = (ROOT / 'Super_Mushak_Logger_V15_FULL_CAUSAL_STATE.py').read_text('utf8')
TREE = ast.parse(SRC)
CLS = next(n for n in TREE.body if isinstance(n, ast.ClassDef) and n.name == 'EngineApp')
METHODS = {n.name: n for n in CLS.body if isinstance(n, ast.FunctionDef)}


def isolated_method(name, env=None):
    n = METHODS[name]
    standalone = ast.FunctionDef(name=n.name, args=n.args, body=n.body,
                                 decorator_list=[], returns=None, type_comment=None)
    module = ast.fix_missing_locations(ast.Module(body=[standalone], type_ignores=[]))
    class Settings:
        V14_58_THROTTLE_TOL_PCT = 1.0
        V14_58_ENGINE_RPM_TOL = 80.0
        V14_68_WORLD_XZ_TOL_FPS = 1.0
        V14_68_WORLD_Y_TOL_FPS = 1.0
        V14_68_BODY_XZ_TOL_FPS = 1.0
        V14_68_BODY_Y_TOL_FPS = 1.0
        V14_30_ROTATION_TOL_RAD_S = .05
    data = {'climb': Settings}
    data.update(env or {})
    exec(compile(module, '<engine-method>', 'exec'), data)
    return data[name]


class App:
    def __init__(self):
        self.reference_initial_dynamic_state = {'throttle_pct':5.0, 'engine_rpm':1736.6}
    @staticmethod
    def _v1430_finite(v):
        try:
            f = float(v)
            return f if math.isfinite(f) else None
        except (TypeError, ValueError):
            return None


class EngineControlOmission(unittest.TestCase):
    def test_live_controls_do_not_block_but_throttle_and_rpm_do(self):
        method = isolated_method('_v1458_control_surfaces_match')
        app = App()
        data = dict(throttles=[5], eng_rpms=[1757], elevator_pos=-.05493,
                    aileron_pos=.9, rudder_pos=-.9, elevator_deflection=99.0,
                    rudder_deflection=99.0, elevator_trim=99.0,
                    aileron_trim_pct=99.0, rudder_trim_pct=-99.0)
        ok, detail = method(app, data, 'v15 t0')
        self.assertTrue(ok, detail)
        self.assertIn('OMITTED', detail)
        self.assertNotIn('ElevInput', detail)
        self.assertFalse(method(app, dict(data, throttles=[70]), 't0')[0])
        self.assertFalse(method(app, dict(data, eng_rpms=[1300]), 't0')[0])
        self.assertIn('RPM=MISSING', method(app, dict(data, eng_rpms=[]), 't0')[1])

    def test_trim_mismatch_ignored_but_velocity_still_gated(self):
        method = isolated_method('_v1430_dynamic_matches')
        app = App()
        state, data = {}, {}
        for live, ref in (('velocity_world_x','velocity_world_x_fps'),
                          ('velocity_world_y','velocity_world_y_fps'),
                          ('velocity_world_z','velocity_world_z_fps'),
                          ('velocity_body_x','velocity_body_x_fps'),
                          ('velocity_body_y','velocity_body_y_fps'),
                          ('velocity_body_z','velocity_body_z_fps'),
                          ('rotation_velocity_body_x','rotation_velocity_body_x_rad_s'),
                          ('rotation_velocity_body_y','rotation_velocity_body_y_rad_s'),
                          ('rotation_velocity_body_z','rotation_velocity_body_z_rad_s')):
            state[ref] = 10.0
            data[live] = 10.0
        data.update(elevator_trim=1000.,aileron_trim_pct=1000.,rudder_trim_pct=1000.)
        self.assertTrue(method(app,data,state)[0])
        data['velocity_world_x'] = 100.0
        self.assertFalse(method(app,data,state)[0])

    def test_climb_replay_is_intact_and_hardware_checks_remain(self):
        self.assertIn('class EngineApp(EngineAirManagerPassiveYaw, ClimbV15125BrunnerApp)', SRC)
        for name in ('_final_bumpless_cls_settle',
                     '_verify_controls_at_initial_targets',
                     '_v1465_cls_only_t0_force_lock',
                     '_v1458_runtime_bridge_health'):
            self.assertIn('def ' + name, LOGGER)
            self.assertNotIn(name, METHODS)
        self.assertIn('_v1481_plot_failed_start', METHODS)

    def test_engine_pdf_removes_all_control_groups_and_table(self):
        import pandas as pd
        import numpy as np
        import fitz
        from plot_graphs import _render_report, _groups_for_test
        self.assertEqual(len(_groups_for_test('acceleration')),3)
        self.assertEqual(len(_groups_for_test('deceleration')),3)
        for mode in ('acceleration','deceleration'):
            with self.subTest(mode=mode), tempfile.TemporaryDirectory() as d:
                root = Path(d)
                data = pd.DataFrame({
                    'time_s': np.arange(0, 11, dtype=float),
                    'ground_speed_kt': np.linspace(0,9,11),
                    'ias_kt': np.linspace(80,82,11),
                    'altitude_msl_ft': np.linspace(2000,2010,11),
                    'height_agl_ft': np.linspace(1700,1710,11),
                    'throttle_pct': np.r_[np.repeat(5.,5),np.repeat(100.,6)],
                    'engine_rpm': np.linspace(1700,2600,11),
                    'prop_rpm': np.linspace(1700,2600,11),
                    'manifold_inhg': np.linspace(15,29,11),
                    'fuel_flow_pph': np.linspace(20,65,11),
                    'Joystick Pitch Position (mm)': np.linspace(-10,10,11),
                    'Joystick Pitch Force (N)': np.linspace(3,20,11),
                    'Joystick Roll Position (mm)': np.linspace(-10,10,11),
                    'Joystick Roll Force (N)': np.linspace(3,20,11),
                    'Rudder Pedal Position (mm)': np.linspace(-10,10,11),
                    'Rudder Pedal Force (N)': np.linspace(3,20,11),
                })
                manual = root/'Super_Mushak_manual_takeoff_data_2026-10-26_09-01-00_legacy_plot.csv'
                auto = root/'Super_Mushak_auto_takeoff_data_2026-10-26_09-02-00_legacy_plot.csv'
                data.to_csv(manual, index=False)
                data.to_csv(auto, index=False)
                out = root/'engine_report.pdf'
                _render_report(auto,manual,out,metadata={'kind':mode,'event_time_s':5})
                with fitz.open(out) as pdf:
                    txt = '\n'.join(p.get_text() for p in pdf)
                    self.assertEqual(len(pdf),4)
                    for bad in ('Joystick Pitch','Joystick Roll','Rudder Pedal',
                                'Control Parameters', 'Pitch actual', 'Roll actual',
                                'Physical CLS position', 'Group 4', 'Group 5'):
                        self.assertNotIn(bad,txt)
                    self.assertIn('Engine RPM',txt)
                    self.assertIn('Manifold pressure',txt)
                    self.assertIn('Fuel flow',txt)

    def test_failed_start_pdf_has_no_cls_graph(self):
        import fitz
        from engine_failed_start_report import plot_engine_failed_start
        class Fake:
            pass
        with tempfile.TemporaryDirectory() as d:
            app=Fake();app.data_dir=Path(d)
            app.p3d_state_diag_rows=[{
                'diag_elapsed_s':float(t),
                'p3d_sideslip':.1,'p3d_aoa':2.5,'p3d_ias_kts':80,
                'p3d_roll_rate_deg_s':0,'p3d_yaw_rate_deg_s':0,
                'p3d_engine_rpm_1':1700+t,'p3d_actual_wind_speed_kt':0,
                'cls_pitch_actual_norm':.6,'cls_roll_actual_norm':.9,
            } for t in range(4)]
            out=plot_engine_failed_start(app,'TEST','control mismatch')
            self.assertTrue(out and out.exists())
            with fitz.open(out) as pdf:
                content='\n'.join(p.get_text() for p in pdf)
                self.assertEqual(len(pdf),2)
                self.assertNotIn('Physical CLS position',content)
                self.assertNotIn('Pitch actual',content)
                self.assertIn('Engine RPM',content)

if __name__ == '__main__':
    unittest.main(verbosity=2)
