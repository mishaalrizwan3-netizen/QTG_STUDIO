"""Static + protocol-level tests: never starts P3D, AP motors or GUI."""
import ast
import struct
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent
ENGINE = (ROOT / 'SMK_Engine_From_Climb.py').read_text(encoding='utf-8')
CLIMB = (ROOT / 'Super_Mushak_Logger_V15_FULL_CAUSAL_STATE.py').read_text(encoding='utf-8')
TRANSPORT = (ROOT / 'climb_brunner_transport.py').read_text(encoding='utf-8')

class NativeClimbTransportTests(unittest.TestCase):
    def test_exact_climb_class_not_monkeypatched(self):
        tree = ast.parse(ENGINE)
        self.assertFalse(any(isinstance(n, ast.ClassDef) and n.name == 'EngineTransport' for n in tree.body))
        self.assertNotIn('climb.Cls2SimUdpTransport =', ENGINE)
        self.assertNotIn('self.cls.engine_alignment', ENGINE)
        self.assertNotIn('PositionAlignment', ENGINE.split('class EngineClient')[1])
    def test_no_interception_or_dynamic_offset(self):
        self.assertNotIn('engine_wire_targets', ENGINE)
        self.assertNotIn('position_alignment import', ENGINE)
        self.assertNotIn('struct.unpack', ENGINE)
    def test_native_protocol_exact_normalized_target(self):
        # Isolate exact source packet builder, without importing PySide6.
        tree=ast.parse(TRANSPORT)
        fn=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='packet_setting_float')
        decl=ast.fix_missing_locations(ast.FunctionDef(name=fn.name,args=fn.args,body=fn.body,decorator_list=[],returns=None,type_comment=None))
        module=ast.Module(body=[decl],type_ignores=[])
        names={'CMD_SETTINGS':0xCC,'struct':struct}
        exec(compile(module, '<protocol>', 'exec'),names)
        for axis in (1,2,4):
            for target in (-.048357, .02723, -.016901, .029577, .0):
                pkt=names['packet_setting_float'](axis,0x80,target)
                cmd,ax,setting,wire=struct.unpack('<Iiif',pkt)
                self.assertEqual((cmd,ax,setting),(0xCC,axis,0x80))
                self.assertAlmostEqual(wire,target,places=6)
    def test_replay_commands_are_climb_packets(self):
        self.assertIn('packet_setting_float(axis, SID_AP_POSITION, target)', CLIMB)
        self.assertIn('AUTO_POSITION_LOOKAHEAD_SEC', CLIMB)
        self.assertIn('CLS_TRACK_PRECHECK_TOL = 0.008',CLIMB)
    def test_p3d_direct_axis_guard_retained(self):
        self.assertIn('Q(TG app must never directly drive P3D',CLIMB.replace('QTG app must never directly drive P3D','Q(TG app must never directly drive P3D'))
        self.assertIn('BRUNNER/CLS2Sim is the only permitted pitch/roll/rudder command path',CLIMB)
        self.assertIn('_set_cls_position_forwarding_to_p3d(True)',CLIMB)
    def test_both_engine_modes_unchanged(self):
        self.assertIn("self.engine_kind='acceleration'",ENGINE)
        self.assertIn("self.engine_kind=='deceleration'",ENGINE)
        self.assertIn('from throttle_timeline import load_manual_throttle',ENGINE)
        self.assertIn('engine_critical_parameter',ENGINE)

if __name__=='__main__': unittest.main(verbosity=2)
