import unittest,sys,threading
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/"src"));sys.path.insert(0,str(ROOT/'src'))
from replay_support import reference_altimeter_offset
import Super_Mushak_Logger_GUI as gui
class AltitudeIntegrityTests(unittest.TestCase):
    def test_teleport_lagged_indication_does_not_become_target_altitude(self):
        self.assertEqual(reference_altimeter_offset({'telemetry':{'indicated_altitude':3562.55,'pressure_alt':3000.,'baro_pressure':1013.1}}),0.)
    def test_legitimate_small_offset_is_retained(self):
        self.assertEqual(reference_altimeter_offset({'telemetry':{'indicated_altitude':2998.,'pressure_alt':3000.,'baro_pressure':1013.1}}),-2.)
    def test_native_instrument_restore_never_writes_indicated_altitude(self):
        source=(ROOT/'native'/'src'/'p3d_bridge_v15_full_causal_state.cpp').read_text()
        definitions=[line for line in source.splitlines() if 'TrackedAddToDataDefinition' in line and 'DEFINITION_INITIAL_INSTRUMENTS_V1506,' in line]
        self.assertEqual(len(definitions),1)
        self.assertIn('VERTICAL SPEED',definitions[0])
        self.assertNotIn('INDICATED ALTITUDE',definitions[0])
    def test_missing_pause_ack_prevents_all_initial_state_writes(self):
        sim=Mock();sim.is_connected.return_value=True;sim.wait_for_paused.return_value=False
        fake=SimpleNamespace(p3d=sim,ref_manual_snapshot={'telemetry':{'pressure_alt':3000.}},stop_worker_event=threading.Event())
        with self.assertRaisesRegex(RuntimeError,'confirm pause'):
            gui.SuperMushakLoggerApp._restore_reference_state(fake)
        sim.set_initial_dynamic_state.assert_not_called()
        sim.set_initial_instruments.assert_not_called()
if __name__=='__main__':unittest.main()
