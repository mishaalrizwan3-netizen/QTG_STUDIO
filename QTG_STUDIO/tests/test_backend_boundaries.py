import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import main
import qtg_common_engine as engine
from backend import artifacts, catalog, control, initializer, recorder, registry, safety, verifier
from interfaces import p3d


class BackendBoundariesTests(unittest.TestCase):
    def test_gui_uses_shared_catalog_and_interface(self):
        self.assertIs(main.TESTS, catalog.TESTS)
        self.assertEqual(len(catalog.TESTS), 30)
        self.assertIs(main.P3DClient, p3d.P3DClient)
        self.assertEqual(registry.get_test("1.f.(1)").title, "Engine acceleration")
        self.assertEqual(registry.get_definition("1.c.(1)")["reference"], "1.c.(1)")
        self.assertEqual(registry.get_definition("1.f.(2)")["reference"], "1.f.(2)")

    def test_worker_uses_shared_pure_services(self):
        self.assertIs(engine.validate_reference, verifier.validate_reference)
        self.assertIs(engine.validate_conditions, verifier.validate_conditions)
        self.assertIs(engine.test_directory, artifacts.test_directory)
        self.assertIs(engine.named_result, artifacts.named_result)
        self.assertIs(engine.validate_duration, safety.validate_duration)
        self.assertIs(engine.snapshot, recorder.snapshot)
        self.assertIs(engine.apply_manual_conditions, initializer.apply_manual_conditions)
        self.assertIs(engine.configure_test_extension, initializer.configure_test_extension)

    def test_existing_controls_and_artifact_names(self):
        self.assertEqual(control.validate_method("1.c.(1)", "pid"), "pid")
        with self.assertRaises(ValueError):
            control.validate_method("1.f.(1)", "pid")
        with self.assertRaises(ValueError):
            safety.validate_duration(0)
        self.assertEqual(
            artifacts.named_result("Super_Mushak_manual_takeoff_data_run.csv", "1.f.(1)", "Engine acceleration"),
            "1_f_1_Engine_acceleration_manual_run.csv",
        )


if __name__ == "__main__":
    unittest.main()
