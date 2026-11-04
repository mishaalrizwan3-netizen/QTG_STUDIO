import importlib.util
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

spec=importlib.util.spec_from_file_location('engine_adapter',Path(__file__).with_name('engine_initialization.py'))
adapter=importlib.util.module_from_spec(spec);spec.loader.exec_module(adapter)

class EngineAdapterTests(unittest.TestCase):
    def make_csv(self,folder,changing_baro=False,stable=False):
        path=Path(folder)/'manual.csv'
        path.write_text('Time (s),Altitude MSL (ft),Indicated Altitude (ft),Baro Press (hPa)\n'
            +'0,2000,1985,1013\n'+''.join(f'{t},2000,{1985 if stable else 2001},{1014 if changing_baro else 1013}\n' for t in (.5,.7,1.,1.5,2.)))
        return path
    def test_stale_initial_instrument_uses_settled_same_baro_samples(self):
        with tempfile.TemporaryDirectory() as folder:
            result=adapter.settled_altimeter(self.make_csv(folder))
            self.assertEqual(result['initial_offset_ft'],-15)
            self.assertEqual(result['settled_offset_ft'],1)
    def test_real_offset_and_changed_barometer_are_preserved(self):
        with tempfile.TemporaryDirectory() as folder:
            self.assertIsNone(adapter.settled_altimeter(self.make_csv(folder,stable=True)))
            self.assertIsNone(adapter.settled_altimeter(self.make_csv(folder,changing_baro=True)))
    def test_refresh_restores_original_snapshot_even_on_failure(self):
        with tempfile.TemporaryDirectory() as folder:
            original={'telemetry':{'pressure_alt':2000.,'indicated_altitude':1985.}}
            host=SimpleNamespace(selected_ref_csv=self.make_csv(folder),ref_manual_snapshot=original,_session_context={})
            host._commit_initial_position=Mock()
            def refresh():
                self.assertEqual(host.ref_manual_snapshot['telemetry']['indicated_altitude'],2001.)
                raise RuntimeError('cancelled')
            host._refresh_restored_airdata=refresh
            adapter.install(host)
            host._commit_initial_position()
            self.assertIs(host.ref_manual_snapshot,original)
            with self.assertRaisesRegex(RuntimeError,'cancelled'):host._refresh_restored_airdata()
            self.assertIs(host.ref_manual_snapshot,original)
            self.assertEqual(original['telemetry']['indicated_altitude'],1985.)
if __name__=='__main__':unittest.main()
