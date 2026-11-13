"""Offline BRUNNER replay tests using deterministic virtual time; no hardware accessed."""
import ast
import math
import time
import unittest
from typing import Dict, Any
from pathlib import Path


class FakeClock:
    """Virtual clock: unittest timing must not depend on Windows scheduling."""
    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        self.now += 0.001
        return self.now

    def sleep(self, seconds):
        self.now += max(0.0, float(seconds))


ROOT = Path(__file__).resolve().parent
SRC = (ROOT / 'Super_Mushak_Logger_V15_FULL_CAUSAL_STATE.py').read_text(encoding='utf-8')
PLOT = (ROOT / 'plot_graphs.py').read_text(encoding='utf-8')


def extract_method(name):
    tree = ast.parse(SRC)
    for cls in (node for node in ast.walk(tree) if isinstance(node, ast.ClassDef)):
        for method in cls.body:
            if isinstance(method, ast.FunctionDef) and method.name == name:
                standalone = ast.FunctionDef(name=method.name, args=method.args, body=method.body,
                                             decorator_list=[], returns=method.returns, type_comment=None)
                standalone = ast.fix_missing_locations(standalone)
                code = compile(ast.Module(body=[standalone], type_ignores=[]), '<isolated-method>', 'exec')
                d = {'CLS_TRACK_PRECHECK_LIMIT': .95, 'CLS_TRACK_PRECHECK_TOL': .008, 'CLS_TRACK_PRECHECK_HOLD_SEC': 6.0,
                     'packet_setting_float': lambda *a: (a[0], a[1], a[2]),
                     'SID_AP_POSITION': 0x80, 'time': FakeClock(), 'math': math, 'Dict': Dict, 'Any': Any}
                exec(code, d)
                return d[name]
    raise AssertionError(name)


class FakeCLS:
    def __init__(self, ack):
        self.ack = ack
        self.commands = []
    def send_with_ack(self, packet, label):
        self.commands.append((packet, label))
        return self.ack


class FakeApp:
    def __init__(self, ack, actual):
        self.cls = FakeCLS(ack)
        self.actual = actual
    def _axis_actual_position(self, name):
        return self.actual


class Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.method = staticmethod(extract_method('_command_and_wait_tracking_target'))

    def test_precheck_requires_ack(self):
        app = FakeApp(False, .1)
        result = self.method(app, 1, 'Pitch', .1, hold_sec=.25)
        self.assertFalse(result['reached'])
        self.assertIn('not acknowledged', result['reason'])

    def test_ack_plus_physical_match(self):
        app = FakeApp(True, .1)
        result = self.method(app, 1, 'Pitch', .1, hold_sec=.25)
        self.assertTrue(result['reached'])
        self.assertEqual(app.cls.commands[0][0][1], 0x80)

    def test_ack_with_small_position_error(self):
        app = FakeApp(True, .107)
        result = self.method(app, 1, 'Pitch', .1, hold_sec=.25)
        self.assertTrue(result['reached'])
        self.assertLess(result['error'], .008)

    def test_ack_not_equal_to_tracking(self):
        app = FakeApp(True, .115)
        result = self.method(app, 1, 'Pitch', .1, hold_sec=.25)
        self.assertFalse(result['reached'])

    def test_climb_ap_commands_not_p3d(self):
        target_loop = SRC.split('targets = {}          # FUTURE AP position commands')[1].split('# PITCH POSITION DIAGNOSTIC ONLY.')[0]
        self.assertIn('SID_AP_POSITION, target', target_loop)
        self.assertNotIn('p3d_direct_bridge', target_loop)
        self.assertNotIn('send_controls(', target_loop)

    def test_stale_oversized_tolerance_removed(self):
        self.assertIn('CLS_TRACK_PRECHECK_TOL = 0.008', SRC)
        self.assertNotIn('CLS_TRACK_PRECHECK_TOL = 0.12', SRC)

    def test_insufficient_reference_movement_identified(self):
        self.assertIn('CLS_MIN_MEANINGFUL_MOTION_NORM = 0.008', SRC)
        self.assertIn('STATIC POSITION ONLY, DYNAMIC TRACKING UNPROVEN', SRC)
        self.assertLess(20/65000, .008)

    def test_failed_udp_sends_checked(self):
        self.assertIn('if not command_ok:', SRC)
        self.assertIn('self.cls_replay_send_failures[name]', SRC)

    def test_ap_off_detection(self):
        self.assertIn('if self.cls_replay_ap_off_counts[name] >= 20:', SRC)

    def test_engine_report_has_no_control_groups(self):
        self.assertNotIn('Group 4 - Joystick Position & Force', PLOT)
        self.assertNotIn('Group 5 - Rudder Pedal Movement & Force', PLOT)
        self.assertNotIn('*** Control Parameters ***', PLOT)

    def test_engine_timing_unchanged(self):
        self.assertIn('throttle_ref = self._v1468_manual_throttle_at_elapsed', SRC)
        self.assertIn('AUTO_POSITION_LOOKAHEAD_SEC = {', SRC)

if __name__ == '__main__':
    unittest.main(verbosity=2)
