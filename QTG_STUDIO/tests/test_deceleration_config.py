import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from backend import deceleration_config as config
import qtg_common_engine as engine


class DecelerationConfigTests(unittest.TestCase):
    def test_initialization_file_is_reloaded_and_gui_changes_override_it(self):
        source = config.suite_root() / "engine_deceleration"
        with tempfile.TemporaryDirectory() as directory:
            suite = Path(directory)
            target = suite / "engine_deceleration"
            target.mkdir()
            for name in ("initialization.json", "input.json", "output.json"):
                (target / name).write_bytes((source / name).read_bytes())
            with patch.object(config, "suite_root", return_value=suite):
                first = config.load_initialization()
                defaults = config.display_defaults()
                self.assertEqual(defaults["Latitude[deg]"], "32.23366667")
                self.assertEqual(first["alt_ft"], 755.0)

                document = json.loads((target / "initialization.json").read_text())
                document["alt_ft"] = 780
                document["heading_deg"] = 49.5
                (target / "initialization.json").write_text(json.dumps(document))
                overrides = config.display_overrides({**defaults, "Throttle[%]": "90"}, defaults)
                resolved = config.resolve_initial_conditions(overrides)
                self.assertEqual(resolved["alt_ft"], 780.0)
                self.assertEqual(resolved["heading_deg"], 49.5)
                self.assertEqual(resolved["throttle"], 90.0)

    def test_launch_writes_resolved_input_and_output_manifest_lists_real_files(self):
        spec = SimpleNamespace(reference="1.f.(2)", title="Engine deceleration")
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            with patch.object(engine, "brunner_root"), patch.object(engine.subprocess, "Popen"):
                _, context = engine.launch("manual", spec, root / "sessions", initial_conditions={"throttle": 92})
            saved = json.loads(Path(context["input_file"]).read_text())
            self.assertEqual(saved["initial_conditions"]["throttle"], 92.0)
            self.assertEqual(saved["initial_conditions"]["lat"], 32.23366667)
            csv_file = root / "measured.csv"
            csv_file.write_text("Time (s),Engine RPM\n0,2700\n")
            manifest = config.write_output_manifest(context, [str(csv_file)])
            contents = json.loads(manifest.read_text())
            self.assertEqual(contents["artifacts"], [str(csv_file)])
            self.assertEqual(contents["initial_conditions"]["throttle"], 92.0)
            self.assertNotIn("samples", contents)


if __name__ == "__main__":
    unittest.main()
