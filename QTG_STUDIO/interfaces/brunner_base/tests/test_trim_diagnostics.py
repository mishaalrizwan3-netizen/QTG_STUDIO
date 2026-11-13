import sys,unittest,tempfile,csv,json,threading
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock,patch
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/"src"))
import Super_Mushak_Logger_GUI as gui
from p3d_simconnect_v15 import SimConnectClient,TelemetryDataStruct
class TrimTests(unittest.TestCase):
    def test_units_sign_and_zero_survive_driver_filter(self):
        raw=TelemetryDataStruct();raw.elevatorTrim=.01;raw.elevatorTrimPct=.25;raw.aileronTrimPct=-.5;raw.rudderTrimPct=0.
        fake=SimpleNamespace(_lock=threading.RLock(),bridge_version=1513,get_telemetry_sequence=lambda:1,get_extended_causal_state=lambda:{},get_engine_support_state=lambda:{})
        SimConnectClient._update_latest_data(fake,raw)
        measured=SimConnectClient.get_telemetry(fake)
        self.assertEqual(measured['trim_elevator_angle_rad'],.01)
        self.assertEqual(measured['trim_elevator_percent'],25.)
        self.assertEqual(measured['trim_aileron_percent'],-50.)
        self.assertEqual(measured['trim_rudder_percent'],0.)
        self.assertNotIn('elevator_pos',measured)
    def test_both_modes_save_actual_trim_and_surfaces_to_csv_and_snapshot(self):
        values={'trim_elevator_angle_rad':.0123,'trim_elevator_percent':12.3,'trim_aileron_percent':-4.5,'trim_rudder_percent':0.,'surface_elevator_rad':-.031}
        for mode in ['manual','auto']:
            with self.subTest(mode=mode),tempfile.TemporaryDirectory(dir=ROOT/'Output') as folder:
                snapshot={'telemetry':dict(values)}
                fake=SimpleNamespace(lbl_rec_status=Mock(),lbl_rec_progress=Mock(),rec_progress=Mock(),target_duration_sec=75,data_dir=Path(folder),test_mode=mode,rec_time=[.02],p_pos_buf=[.1],r_pos_buf=[.2],y_pos_buf=[0.],p_force_buf=[1.],r_force_buf=[2.],y_force_buf=[0.],p3d_tele_buf=[dict(values)],manual_t0_snapshot=snapshot,auto_t0_snapshot=snapshot,active_manual_ref_path=None,_refresh_manual_reference_selector=lambda:None)
                with patch.object(gui,'CURRENT_DIR',folder),patch.object(gui.plot_graphs,'plot_aligned_comparison',return_value=None),patch.object(gui,'QProgressDialog'),patch.object(gui.QApplication,'processEvents'),patch.object(gui.QMessageBox,'information'),patch.object(gui.QMessageBox,'critical') as errors:
                    gui.SuperMushakLoggerApp._save_csv_and_generate_reports(fake)
                    errors.assert_not_called()
                with next(Path(folder).glob('Super_Mushak_*.csv')).open() as stream:
                    row=next(csv.DictReader(stream))
                self.assertEqual(float(row['Elevator Trim Angle (rad)']),.0123)
                self.assertEqual(float(row['Elevator Trim (%)']),12.3)
                self.assertEqual(float(row['Aileron Trim (%)']),-4.5)
                self.assertEqual(float(row['Rudder Trim (%)']),0.)
                self.assertEqual(float(row['Elevator Surface (rad)']),-.031)
                saved=json.loads(next(Path(folder).glob('*init_conditions.json')).read_text())
                self.assertEqual(saved[mode]['telemetry'],values)
    def test_trim_setters_remain_disabled(self):
        bridge=Mock();fake=SimpleNamespace(_bridge=bridge)
        SimConnectClient.set_elevator_trim(fake,.1)
        SimConnectClient.set_control_inputs(fake,.1,.2,.3,.4)
        self.assertEqual(bridge.mock_calls,[])
if __name__=='__main__':unittest.main()
