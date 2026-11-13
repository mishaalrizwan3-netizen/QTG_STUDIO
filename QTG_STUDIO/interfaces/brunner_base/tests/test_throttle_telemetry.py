import sys,threading,unittest,math
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/"src"))
import p3d_simconnect_v15 as driver
import Super_Mushak_Logger_GUI as gui

class ThrottleTests(unittest.TestCase):
    def test_raw_engine_throttle_is_exposed_but_flight_controllers_are_excluded(self):
        for throttle in (0.,42.5,100.):
            raw=driver.TelemetryDataStruct()
            raw.title=b'Test Aircraft'
            raw.numberOfEngines=1
            raw.throttle1=throttle
            raw.elevatorPos=.8
            fake=SimpleNamespace(_lock=threading.Lock(),bridge_version=1503,
                get_telemetry_sequence=lambda:9,get_extended_causal_state=lambda:{},get_engine_support_state=lambda:{})
            driver.SimConnectClient._update_latest_data(fake,raw)
            data=driver.SimConnectClient.get_telemetry(fake)
            self.assertEqual(data['engine_throttle_percent'],throttle)
            self.assertFalse(set(data)&driver.P3D_CONTROLLER_KEYS)

    def test_display_uses_measured_throttle(self):
        fake=SimpleNamespace(p3d=SimpleNamespace(is_connected=lambda:True,
            get_telemetry=lambda:{'engine_throttle_percent':42.5,'engine_rpm':2655}),
            is_recording=True)
        for key in ['lbl_throttle_val','throttle_bar','lbl_engine_rpm','lbl_ias_alt','lbl_pitch_bank']:
            setattr(fake,key,Mock())
        gui.SuperMushakLoggerApp._on_ui_timer_tick(fake)
        fake.lbl_throttle_val.setText.assert_called_with('Throttle: 42.5 %')
        fake.throttle_bar.setValue.assert_called_with(42)

    def test_missing_throttle_is_reported_unavailable(self):
        fake=SimpleNamespace(p3d=SimpleNamespace(is_connected=lambda:True,
            get_telemetry=lambda:{}),is_recording=True)
        for key in ['lbl_throttle_val','throttle_bar','lbl_engine_rpm','lbl_ias_alt','lbl_pitch_bank']:
            setattr(fake,key,Mock())
        gui.SuperMushakLoggerApp._on_ui_timer_tick(fake)
        fake.lbl_throttle_val.setText.assert_called_with('Throttle: unavailable')

if __name__=='__main__': unittest.main(verbosity=2)
