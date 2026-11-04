import unittest,sys,threading,ctypes
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/"src"))
import Super_Mushak_Logger_GUI as gui
import p3d_simconnect_v15 as driver

class PowerPreparationTests(unittest.TestCase):
    def test_sets_reference_throttle_and_restores_thermal_state_without_motor_commands(self):
        state={'engine_oil_temperature_c_1':100.,'engine_cylinder_head_temperature_c_1':208.}
        sim=Mock()
        sim.set_initial_throttle.return_value=True
        sim.restore_engine_thermal_state.return_value=dict(state)
        fake=SimpleNamespace(p3d=sim,ref_manual_snapshot={'throttle':81.4,'telemetry':state},
            _refresh_paused_telemetry=lambda:{'engine_throttle_percent':81.4},brunner=Mock())
        result=gui.SuperMushakLoggerApp._prepare_reference_power(fake)
        sim.set_initial_throttle.assert_called_once_with(81.4)
        sim.restore_engine_thermal_state.assert_called_once_with(state)
        self.assertEqual(result['engine_oil_temperature_c_1'],100.)
        self.assertEqual(fake.brunner.mock_calls,[])
        sim.set_throttle.assert_not_called()
        sim.set_control_inputs.assert_not_called()

    def test_csv_starting_throttle_overrides_stale_metadata_including_zero(self):
        for target in (0.,25.,100.):
            with self.subTest(target=target):
                sim=Mock();sim.restore_engine_thermal_state.return_value={}
                fake=SimpleNamespace(p3d=sim,ref_throt_pos=[target,90.],
                    ref_manual_snapshot={'throttle':80.,'telemetry':{}},
                    _refresh_paused_telemetry=lambda:{'engine_throttle_percent':target})
                gui.SuperMushakLoggerApp._prepare_reference_power(fake)
                sim.set_initial_throttle.assert_called_once_with(target)
                self.assertEqual(fake.ref_manual_snapshot['throttle'],target)

    def test_partial_support_packet_retains_valid_temperatures(self):
        bridge=Mock()
        bridge.GetEngineSupportSequenceV1496.return_value=4
        def fill(ptr):
            raw=ctypes.cast(ptr,ctypes.POINTER(driver.EngineSupportStateStructV1496)).contents
            raw.values[:]=[100.,208.,float('nan'),1.,float('nan'),1.,1.,float('nan')]
        bridge.GetEngineSupportStateV1496.side_effect=fill
        data=driver.SimConnectClient.get_engine_support_state(SimpleNamespace(_bridge=bridge))
        self.assertEqual(data['engine_oil_temperature_c_1'],100.)
        self.assertEqual(data['engine_cylinder_head_temperature_c_1'],208.)
        self.assertNotIn('general_eng_starter_1',data)

    def test_thermal_restore_writes_temperatures_only_and_verifies_readback(self):
        target={'engine_oil_temperature_c_1':100.,'engine_cylinder_head_temperature_c_1':208.}
        live={'engine_oil_temperature_c_1':28.,'engine_cylinder_head_temperature_c_1':29.}
        bridge=Mock()
        def write(index,value):
            live[driver.ENGINE_SUPPORT_KEYS_V1496[index]]=value
            return True
        bridge.SetEngineSupportValueV1496.side_effect=write
        fake=SimpleNamespace(_bridge=bridge,_lock=threading.Lock(),_latest_data={},
            get_engine_support_state=lambda **kw:dict(live))
        data=driver.SimConnectClient.restore_engine_thermal_state(fake,target)
        self.assertEqual(data,target)
        self.assertEqual([c.args[0] for c in bridge.SetEngineSupportValueV1496.call_args_list],[0,1])
        self.assertEqual(len(bridge.mock_calls),2)

    def test_initial_throttle_path_is_separate_from_disabled_replay_setter(self):
        bridge=Mock()
        bridge.SetInitialThrottleV1504.return_value=1
        fake=SimpleNamespace(_bridge=bridge)
        self.assertTrue(driver.SimConnectClient.set_initial_throttle(fake,81.4))
        bridge.SetInitialThrottleV1504.assert_called_once_with(81.4)
        driver.SimConnectClient.set_throttle(fake,90.)
        self.assertEqual(len(bridge.mock_calls),1)
        with self.assertRaises(ValueError): driver.SimConnectClient.set_initial_throttle(fake,101.)

if __name__=='__main__': unittest.main(verbosity=2)
