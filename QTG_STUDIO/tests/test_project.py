import ast
import pathlib
import unittest
from unittest.mock import patch

ROOT = pathlib.Path(__file__).resolve().parents[1]

class ProjectCheck(unittest.TestCase):
    def test_gui_and_p3d_wrapper_are_separate(self):
        self.assertIn("from frontend import main as _frontend", (ROOT / "main.py").read_text(encoding="utf-8"))
        text = (ROOT / "frontend" / "main.py").read_text(encoding="utf-8")
        for needle in ("class App(tk.Tk)", "from backend.catalog import TestSpec, TESTS", "from interfaces.p3d import P3DClient"):
            self.assertIn(needle, text)
        self.assertIn("TESTS = [", (ROOT / "backend" / "catalog.py").read_text(encoding="utf-8"))
        self.assertNotIn("class P3DClient", text)
        ast.parse(text)
        bridge = (ROOT / "interfaces" / "p3d.py").read_text(encoding="utf-8")
        for needle in ("class P3DClient", "class QTG_P3D_State", "SimConnect"):
            self.assertIn(needle, bridge)
        ast.parse(bridge)

    def test_p3d_cpp_is_read_only(self):
        text = (ROOT / "bridge" / "qtg_p3d_bridge.cpp").read_text(encoding="utf-8")
        # Excluding the safety comment line that mentions this API.
        noncomments = "\n".join(line for line in text.splitlines() if not line.strip().startswith("//"))
        self.assertNotIn("SimConnect_SetDataOnSimObject", noncomments)
        self.assertIn("SimConnect_Open", text)

    def test_dashboard_bridge_constructors_accept_gui_paths(self):
        from interfaces.p3d import BridgeWriter, P3DClient

        dll = ROOT / "bridge" / "bin" / "qtg_p3d_bridge.dll"
        self.assertEqual(P3DClient(dll).dll_path, dll)
        with patch.object(BridgeWriter, "_init"):
            writer = BridgeWriter(ROOT, ROOT, ROOT)
        self.assertEqual((writer.base, writer.meipass, writer.local_dir), (ROOT, ROOT, ROOT))

if __name__ == "__main__":
    unittest.main()
