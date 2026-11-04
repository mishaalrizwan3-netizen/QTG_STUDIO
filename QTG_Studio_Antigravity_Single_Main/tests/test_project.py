import ast
import pathlib
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]

class ProjectCheck(unittest.TestCase):
    def test_single_main_contains_gui_and_p3d_wrapper(self):
        text = (ROOT / "main.py").read_text(encoding="utf-8")
        for needle in ("class App(tk.Tk)", "class P3DClient", "class QTG_P3D_State", "TESTS = [", "SimConnect"):
            self.assertIn(needle, text)
        ast.parse(text)

    def test_p3d_cpp_is_read_only(self):
        text = (ROOT / "bridge" / "qtg_p3d_bridge.cpp").read_text(encoding="utf-8")
        # Excluding the safety comment line that mentions this API.
        noncomments = "\n".join(line for line in text.splitlines() if not line.strip().startswith("//"))
        self.assertNotIn("SimConnect_SetDataOnSimObject", noncomments)
        self.assertIn("SimConnect_Open", text)

if __name__ == "__main__":
    unittest.main()
