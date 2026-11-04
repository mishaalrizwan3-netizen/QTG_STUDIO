"""Offline checks that reusable BRUNNER split has not changed climb mechanics.

Tests only inspect Python syntax/source. They never contact CLS2Sim, P3D,
or motors. Reference digests were computed from the USER-SUPPLIED engine ZIP's
V15-derived Super_Mushak_Logger_V15_FULL_CAUSAL_STATE.py, NOT extracted
from the compiled Super_Mushak_V15_12_5.exe.
"""
from pathlib import Path
import ast
import hashlib
import unittest

ROOT = Path(__file__).resolve().parent
TRANSPORT = ROOT / 'climb_brunner_transport.py'
LOGGER = ROOT / 'Super_Mushak_Logger_V15_FULL_CAUSAL_STATE.py'
ADAPTER = ROOT / 'climb_brunner.py'
ENGINE = ROOT / 'SMK_Engine_From_Climb.py'

# Digest of parsed functions/classes (ignores comments, indentation and
# line numbers; captures operational statements and defaults).
EXPECTED_PROTOCOL_AST_PREFIX = {
    'packet_override': '6f5753abc46fe91e',
    'packet_setting_u8': 'd08899cb96cf3f22',
    'packet_setting_u16': '2c611f35042c3a35',
    'packet_setting_i32': 'a37d2c0a28c52e08',
    'packet_setting_float': '70daa37b6a0c2566',
    'packet_read': 'fd240857c717a283',
    'packet_watchdog': 'f9997027bf22d96f',
    'packet_state': '4f0076c2d2753f90',
    'node_id_mask': '94931efb5595506f',
    'packet_remote_config_u8': '4fc2accb694010cb',
    'Cls2SimUdpTransport': '5cf0b39b0cbb790a',
}
EXPECTED_CLIMB_LOGGER_CLASS_AST_SHA256 = (
    '47108c107ff591fd6dde89bb22921dbd383c8b1e326fff75022f42180b4e03d9'
)


def _code_digest(node):
    return hashlib.sha256(ast.dump(node, include_attributes=False).encode('utf-8')).hexdigest()


class ReusableClimbBrunnerTests(unittest.TestCase):
    def test_transport_is_exact_original_extracted_implementation(self):
        tree = ast.parse(TRANSPORT.read_text(encoding='utf-8'))
        nodes = {node.name: node for node in tree.body
                 if isinstance(node, (ast.FunctionDef, ast.ClassDef))}
        for name, digest_prefix in EXPECTED_PROTOCOL_AST_PREFIX.items():
            with self.subTest(name=name):
                self.assertIn(name, nodes)
                self.assertTrue(_code_digest(nodes[name]).startswith(digest_prefix),
                                f'{name}: source changed since original climb-derived logger')

    def test_climb_logger_replay_state_machine_is_unchanged(self):
        tree = ast.parse(LOGGER.read_text(encoding='utf-8'))
        logger = next(node for node in tree.body if isinstance(node, ast.ClassDef)
                      and node.name == 'SuperMushakLoggerApp')
        self.assertEqual(_code_digest(logger), EXPECTED_CLIMB_LOGGER_CLASS_AST_SHA256)
        imports = [node for node in tree.body if isinstance(node, ast.ImportFrom)
                   and node.module == 'climb_brunner_transport']
        self.assertEqual(len(imports), 1)
        exposed = {name.name for name in imports[0].names}
        self.assertIn('Cls2SimUdpTransport', exposed)
        self.assertIn('packet_setting_float', exposed)

    def test_controller_base_and_engine_use_the_same_climb_path(self):
        controller = ast.parse(ADAPTER.read_text(encoding='utf-8'))
        base = next(node for node in controller.body if isinstance(node, ast.ClassDef)
                    and node.name == 'ClimbBrunnerApp')
        self.assertEqual(len(base.bases), 1)
        self.assertIsInstance(base.bases[0], ast.Name)
        self.assertEqual(base.bases[0].id, 'SuperMushakLoggerApp')
        self.assertFalse(any(isinstance(n, ast.FunctionDef) for n in base.body),
                         'base must not redefine climb startup/replay methods')
        engine = ast.parse(ENGINE.read_text(encoding='utf-8'))
        app = next(node for node in engine.body if isinstance(node, ast.ClassDef)
                   and node.name == 'EngineApp')
        self.assertEqual(len(app.bases), 2)
        self.assertTrue(all(isinstance(base, ast.Name) for base in app.bases))
        self.assertEqual([base.id for base in app.bases],
                         ['EngineAirManagerPassiveYaw', 'ClimbV15125BrunnerApp'])


if __name__ == '__main__':
    unittest.main(verbosity=2)
