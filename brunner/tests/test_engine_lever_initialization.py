import sys,threading,unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock,patch
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/"src"))
import Super_Mushak_Logger_GUI as gui
from p3d_simconnect_v15 import SimConnectClient

class EngineLeverInitializationTests(unittest.TestCase):
    def test_restart_mixture_difference_is_restored_and_measured(self):
        live={"engine_mixture_percent":73.1,"engine_propeller_percent":100.}
        state={"engine_mixture_percent":56.6,"engine_propeller_percent":100.}
        sim=Mock();sim.is_connected.return_value=True
        sim.restore_engine_thermal_state.return_value={}
        def restore(mixture,propeller):
            live.update(engine_mixture_percent=mixture,engine_propeller_percent=propeller)
            return True
        sim.set_initial_engine_levers.side_effect=restore
        fake=SimpleNamespace(p3d=sim,ref_manual_snapshot={"telemetry":state},_refresh_paused_telemetry=lambda:dict(live))
        result=gui.SuperMushakLoggerApp._prepare_reference_power(fake)
        sim.set_initial_engine_levers.assert_called_once_with(56.6,100.)
        self.assertEqual(result["engine_mixture_percent"],56.6)
        sim.set_control_inputs.assert_not_called()

    def test_driver_requires_confirmed_pause_and_keeps_missing_lever(self):
        bridge=Mock();bridge.SetInitialEngineLeversV1515.return_value=1
        fake=SimpleNamespace(_bridge=bridge,wait_for_paused=Mock(return_value=True))
        self.assertTrue(SimConnectClient.set_initial_engine_levers(fake,56.6,None))
        bridge.SetInitialEngineLeversV1515.assert_called_once_with(56.6,-1.)
        bridge.reset_mock();fake.wait_for_paused.return_value=False
        with self.assertRaisesRegex(RuntimeError,"confirm pause"):
            SimConnectClient.set_initial_engine_levers(fake,56.6,100.)
        bridge.SetInitialEngineLeversV1515.assert_not_called()

    def test_invalid_values_never_write(self):
        fake=SimpleNamespace(_bridge=Mock(),wait_for_paused=Mock())
        for value in [-1.,101.,float("nan"),float("inf")]:
            with self.assertRaises(ValueError):SimConnectClient.set_initial_engine_levers(fake,value,100.)
        fake.wait_for_paused.assert_not_called()
        self.assertEqual(fake._bridge.mock_calls,[])

    def test_legacy_missing_levers_are_not_guessed(self):
        fake=SimpleNamespace(_bridge=Mock(),wait_for_paused=Mock())
        self.assertTrue(SimConnectClient.set_initial_engine_levers(fake,None,None))
        self.assertEqual(fake._bridge.mock_calls,[])
        fake.wait_for_paused.assert_not_called()

    def test_rejected_setup_does_not_pass_readiness(self):
        sim=Mock();sim.set_initial_engine_levers.return_value=False
        fake=SimpleNamespace(p3d=sim,ref_manual_snapshot={"telemetry":{"engine_mixture_percent":56.6}},_refresh_paused_telemetry=Mock())
        with self.assertRaisesRegex(RuntimeError,"rejected recorded mixture"):
            gui.SuperMushakLoggerApp._prepare_reference_power(fake)
        fake._refresh_paused_telemetry.assert_not_called()

    def test_successful_write_with_wrong_readback_still_blocks(self):
        sim=Mock();sim.restore_engine_thermal_state.return_value={}
        fake=SimpleNamespace(p3d=sim,ref_manual_snapshot={"telemetry":{"engine_mixture_percent":56.6}},_refresh_paused_telemetry=lambda:{"engine_mixture_percent":73.1})
        with patch.object(gui.time,"monotonic",side_effect=[0.,4.]),self.assertRaisesRegex(ValueError,"Mixture"):
            gui.SuperMushakLoggerApp._prepare_reference_power(fake)
