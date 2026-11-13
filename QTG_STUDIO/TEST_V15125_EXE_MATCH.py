"""Offline structural and simulated-feedback parity tests; never energize axes."""
import ast
import math
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parent
SRC = (ROOT/'climb_brunner_v15125.py').read_text('utf-8')
ENG = (ROOT/'SMK_Engine_From_Climb.py').read_text('utf-8')

class Clock:
    def __init__(self): self.now = 0
    def monotonic(self): self.now += .001; return self.now
    def sleep(self,s): self.now += s

class Transport:
    def __init__(self): self.commands=[]
    def send(self,p): self.commands.append(p);return True

class FakeApp:
    def __init__(self,actual):self.cls=Transport();self.actual=actual
    def _axis_actual_position(self,name): return self.actual

class Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        tree=ast.parse(SRC)
        cls_node=next(x for x in tree.body if isinstance(x,ast.ClassDef) and x.name=='ClimbV15125BrunnerApp')
        cls.methods={x.name:x for x in cls_node.body if isinstance(x,ast.FunctionDef)}

    def test_exe_verified_constants_and_inheritance(self):
        self.assertIn("'CLS_TRACK_PRECHECK_TOL': 0.12",SRC)
        self.assertIn("'CLS_FOLLOW_WARN_ERROR': 0.20",SRC)
        self.assertIn("'CLS_FOLLOW_ABORT_ERROR': 0.90",SRC)
        self.assertIn('class EngineApp(EngineAirManagerPassiveYaw, ClimbV15125BrunnerApp)',ENG)
        self.assertIn('qualification_eligible=False',ENG)

    def test_no_legacy_to_newer_static_precheck_shortcut(self):
        src=ast.get_source_segment(SRC,self.methods['_run_tracking_endpoint_precheck'])
        self.assertFalse(any(isinstance(n,ast.Name) and n.id=='CLS_MIN_MEANINGFUL_MOTION_NORM' for n in ast.walk(self.methods['_run_tracking_endpoint_precheck'])))
        self.assertIn("'negative'",src)
        self.assertIn("'positive'",src)
        self.assertIn("'return_after_negative'",src)
        self.assertIn("'return_after_positive'",src)
        self.assertIn('self.cls.send(',src)

    def test_error_that_failed_engine_is_allowed_under_actual_climb_exe(self):
        m=self.methods['_command_and_wait_tracking_target']
        isolated=ast.fix_missing_locations(ast.Module(body=[ast.FunctionDef(
            name='test_func',args=m.args,body=m.body,decorator_list=[],returns=None,
            type_comment=None)],type_ignores=[]))
        class Settings:
            CLS_TRACK_PRECHECK_LIMIT=.95
            CLS_TRACK_PRECHECK_HOLD_SEC=6.0
            CLS_TRACK_PRECHECK_TOL=.12
        clock=Clock()
        d={'climb':Settings,'math':math,'time':clock,
           'packet_setting_float':lambda axis,sid,val:(axis,sid,val),
           'SID_AP_POSITION':0x80}
        exec(compile(isolated,'<no-hardware>','exec'),d)
        fake=FakeApp(.0084507465)
        r=d['test_func'](fake,2,'Roll',.0342723131,.5)
        self.assertTrue(r['reached'])
        self.assertAlmostEqual(r['error'],.0258215666,places=5)
        self.assertEqual(fake.cls.commands[0][1],0x80)
        Settings.CLS_TRACK_PRECHECK_TOL=.008
        fake2=FakeApp(.0084507465)
        r2=d['test_func'](fake2,2,'Roll',.0342723131,.5)
        self.assertFalse(r2['reached'])

    def test_protocol_transport_is_unchanged(self):
        src=ast.parse((ROOT/'climb_brunner_transport.py').read_text('utf-8'))
        self.assertIn('Cls2SimUdpTransport',[n.name for n in src.body if isinstance(n,ast.ClassDef)])

if __name__=='__main__':unittest.main(verbosity=2)
