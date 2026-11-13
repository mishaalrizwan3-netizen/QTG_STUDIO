import unittest,sys,tempfile,json,threading
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock,patch
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/"src"))
sys.path.insert(0,str(ROOT/'src'))
from replay_support import validate_reference_power
import Super_Mushak_Logger_GUI as gui

REFERENCE={'throttle':81.40625,'telemetry':{'engine_oil_temperature_c_1':99.97857,'engine_cylinder_head_temperature_c_1':208.3847}}
COLD={'engine_throttle_percent':90.,'engine_oil_temperature_c_1':27.77,'engine_cylinder_head_temperature_c_1':29.09}

class ProcessTests(unittest.TestCase):
    def test_matching_warm_conditions_pass_without_changing_values(self):
        data={'engine_throttle_percent':81.4,'engine_oil_temperature_c_1':100.,'engine_cylinder_head_temperature_c_1':210.}
        before=dict(data)
        validate_reference_power(data,REFERENCE)
        self.assertEqual(data,before)
    def test_actual_cold_high_throttle_run_reports_all_mismatches(self):
        with self.assertRaises(ValueError) as error:
            validate_reference_power(COLD,REFERENCE)
        for text in ['Throttle','Oil temperature','Cylinder-head','90.0','81.4']:
            self.assertIn(text,str(error.exception))
    def test_unknown_legacy_conditions_are_not_invented(self):
        validate_reference_power({}, {})
    def test_unavailable_measured_throttle_blocks_when_reference_is_known(self):
        with self.assertRaisesRegex(ValueError,'unavailable'):
            validate_reference_power({}, {'throttle':81.4})
    def test_setup_failure_keeps_simulator_paused_and_does_not_move_hardware(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'Output') as folder:
            path=Path(folder)/'manual.csv'
            path.write_text('Time (s),Pitch_Manual_Pos,Throttle 1 Position (%)\n0,0.1,81.4\n0.02,0.2,81.4\n')
            path.with_name('manual_init_conditions.json').write_text(json.dumps({'manual':REFERENCE}))
            driver=Mock()
            driver.is_connected.return_value=True
            hardware=Mock()
            fake=SimpleNamespace(selected_ref_csv=str(path),_update_stage_ui=Mock(),
                ref_p_pos=[],ref_r_pos=[],ref_y_pos=[],ref_throt_pos=[],ref_time=[],
                stop_worker_event=threading.Event(),p3d=driver,brunner=hardware,
                _refresh_paused_telemetry=lambda:dict(COLD),_prepare_reference_power=Mock(side_effect=RuntimeError("Power setup rejected")),sig_update_stage=Mock())
            class ImmediateThread:
                def __init__(self,target,**kw): self.target=target
                def start(self): self.target()
            with patch.object(gui.threading,'Thread',ImmediateThread):
                gui.SuperMushakLoggerApp.start_auto_test_only(fake)
            driver.pause_sim.assert_called()
            driver.set_position_airborne.assert_not_called()
            hardware.move_actuators.assert_not_called()
            hardware.configure_replay_mode.assert_not_called()
            self.assertEqual(fake.sig_update_stage.emit.call_args.args[0],gui.STAGE_AUTO_BLOCKED)

if __name__=='__main__': unittest.main(verbosity=2)
