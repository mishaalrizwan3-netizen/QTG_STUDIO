import sys,threading,unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock,patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import Super_Mushak_Logger_GUI as gui

class RpmSettlingTests(unittest.TestCase):
    def test_airdata_refresh_follows_last_engine_warmup(self):
        import inspect
        source=inspect.getsource(gui.SuperMushakLoggerApp.start_auto_test_only)
        rpm=source.index('self._wait_for_reference_rpm()')
        air=source.index('self._refresh_restored_airdata()')
        final=source.index('validate_initial_airspeed', air)
        self.assertLess(rpm, air)
        self.assertLess(air, final)
        self.assertIn('self._restore_reference_state()', source[rpm:air])

    def run_settling(self, frames, cancel=False):
        sim=Mock()
        sim.is_connected.return_value=True
        sim.wait_for_paused.return_value=True
        sim.wait_for_fresh_telemetry.side_effect=lambda *a: next(frames)
        stop=threading.Event()
        if cancel: stop.set()
        fake=SimpleNamespace(p3d=sim,ref_manual_snapshot={"rpm":2700.,"telemetry":{"prop_rpms":[2700.]}},stop_worker_event=stop)
        clock=iter(i*.25 for i in range(300))
        return sim,fake,patch.object(gui.time,"monotonic",side_effect=lambda:next(clock))

    def test_startup_oscillation_must_decay_before_ready(self):
        values=[2720,2680,2715,2685]+[2700]*30
        frames=iter({"engine_rpm":v,"prop_rpms":[v]} for v in values)
        sim,fake,clock=self.run_settling(frames)
        with clock,patch.object(gui.time,"sleep"):
            gui.SuperMushakLoggerApp._wait_for_reference_rpm(fake)
        self.assertGreaterEqual(sim.wait_for_fresh_telemetry.call_count,13)
        sim.unpause_sim.assert_called_once()
        sim.pause_sim.assert_called_once()
        sim.set_initial_throttle.assert_not_called()
        sim.set_control_inputs.assert_not_called()

    def test_permanent_offset_times_out_paused(self):
        frames=iter([{"engine_rpm":2750.,"prop_rpms":[2750.]}]*100)
        sim,fake,clock=self.run_settling(frames)
        with clock,patch.object(gui.time,"sleep"),self.assertRaisesRegex(RuntimeError,"did not settle"):
            gui.SuperMushakLoggerApp._wait_for_reference_rpm(fake)
        sim.pause_sim.assert_called_once()

    def test_missing_propeller_data_cannot_pass(self):
        frames=iter([{"engine_rpm":2700.}]*100)
        sim,fake,clock=self.run_settling(frames)
        with clock,patch.object(gui.time,"sleep"),self.assertRaisesRegex(RuntimeError,"did not settle"):
            gui.SuperMushakLoggerApp._wait_for_reference_rpm(fake)
        sim.pause_sim.assert_called_once()

    def test_cancellation_leaves_simulator_paused(self):
        sim,fake,clock=self.run_settling(iter([]),cancel=True)
        with clock,self.assertRaisesRegex(RuntimeError,"cancelled"):
            gui.SuperMushakLoggerApp._wait_for_reference_rpm(fake)
        sim.pause_sim.assert_called_once()

    def test_missing_fresh_packet_blocks_and_pauses(self):
        sim,fake,clock=self.run_settling(iter([None]))
        with clock,self.assertRaisesRegex(RuntimeError,"No fresh"):
            gui.SuperMushakLoggerApp._wait_for_reference_rpm(fake)
        sim.pause_sim.assert_called_once()
