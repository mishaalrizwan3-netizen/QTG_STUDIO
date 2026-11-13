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

    def test_engine_drift_is_restored_before_rpm_can_pass(self):
        good=dict(engine_rpm=2700.,prop_rpms=[2700.],airspeed_indicated=80.,pressure_alt=2000.)
        drift=dict(good,airspeed_indicated=101.,pressure_alt=1354.)
        sim,fake,clock=self.run_settling(iter([drift]+[good]*40))
        fake._session_context={'test_reference':'1.c.(1)'}
        fake.ref_manual_snapshot.update(airspeed=80.,altitude_msl=2000.)
        fake._restore_reference_state=Mock()
        with clock,patch.object(gui.time,'sleep'):
            gui.SuperMushakLoggerApp._wait_for_reference_rpm(fake)
        fake._restore_reference_state.assert_called_once()
        self.assertEqual(fake._session_context['engine_rpm_initialization']['flight_state_corrections'],1)
        sim.set_initial_throttle.assert_not_called()
        sim.set_control_inputs.assert_not_called()

    def test_climb_drift_is_restored_before_rpm_can_settle(self):
        good=dict(engine_rpm=2618.2,prop_rpms=[2618.2],airspeed_indicated=80.,pressure_alt=3000.)
        drift=dict(good,engine_rpm=2699.9,prop_rpms=[2699.9],airspeed_indicated=90.)
        sim,fake,clock=self.run_settling(iter([drift]+[good]*40))
        fake._session_context={'test_reference':'1.c.(1)'}
        fake.ref_manual_snapshot.update(rpm=2618.2,airspeed=80.,altitude_msl=3000.,telemetry={'prop_rpms':[2618.2]})
        fake._restore_reference_state=Mock()
        with clock,patch.object(gui.time,'sleep'):
            gui.SuperMushakLoggerApp._wait_for_reference_rpm(fake)
        fake._restore_reference_state.assert_called_once()
        self.assertEqual(fake._session_context['engine_rpm_initialization']['flight_state_corrections'],1)
        sim.set_initial_throttle.assert_not_called()
        sim.set_control_inputs.assert_not_called()

    def test_engine_warmup_allows_small_ias_transient_without_reinitializing(self):
        frame=dict(engine_rpm=2700.,prop_rpms=[2700.],airspeed_indicated=82.6,pressure_alt=2000.)
        sim,fake,clock=self.run_settling(iter([frame]*40))
        fake._session_context={'test_reference':'1.c.(1)'}
        fake.ref_manual_snapshot.update(airspeed=80.,altitude_msl=2000.)
        fake._restore_reference_state=Mock()
        with clock,patch.object(gui.time,'sleep'):
            gui.SuperMushakLoggerApp._wait_for_reference_rpm(fake)
        fake._restore_reference_state.assert_not_called()
        self.assertEqual(fake._session_context['engine_rpm_initialization']['actual_ias_kt'],82.6)
        # Warm-up allowance must not weaken the final replay IAS requirement.
        with self.assertRaises(ValueError):
            gui.validate_initial_airspeed(frame,80.)

    def test_engine_persistent_drift_cannot_pass_matching_rpm(self):
        drift=dict(engine_rpm=2700.,prop_rpms=[2700.],airspeed_indicated=101.,pressure_alt=1354.)
        sim,fake,clock=self.run_settling(iter([drift]*100))
        fake._session_context={'test_reference':'1.c.(1)'}
        fake.ref_manual_snapshot.update(airspeed=80.,altitude_msl=2000.)
        fake._restore_reference_state=Mock()
        with clock,patch.object(gui.time,'sleep'),self.assertRaisesRegex(RuntimeError,'drifted'):
            gui.SuperMushakLoggerApp._wait_for_reference_rpm(fake)
        self.assertTrue(fake._restore_reference_state.called)
        self.assertTrue(sim.pause_sim.called)

    def test_reported_climb_offset_is_ready_without_twenty_second_wait(self):
        frames=iter([dict(engine_rpm=2697.2,prop_rpms=[2697.2],airspeed_indicated=80.,pressure_alt=3000.)]*100)
        sim,fake,clock=self.run_settling(frames)
        fake._session_context={'test_reference':'1.c.(1)'}
        fake.ref_manual_snapshot.update(rpm=2618.2,airspeed=80.,altitude_msl=3000.,telemetry={'prop_rpms':[2618.2]})
        with clock,patch.object(gui.time,'sleep'):
            gui.SuperMushakLoggerApp._wait_for_reference_rpm(fake)
        self.assertLess(sim.wait_for_fresh_telemetry.call_count,20)
        self.assertEqual(fake._session_context['rpm_startup_readiness']['actual_engine_rpm'],2697.2)
        self.assertEqual(gui.startup_rpm_tolerance('1.c.(1)'),100.)
        sim.set_initial_throttle.assert_not_called()

    def test_frequent_flight_restores_do_not_starve_rpm_readiness(self):
        good=dict(engine_rpm=2697.2,prop_rpms=[2697.2],airspeed_indicated=79.6,pressure_alt=3000.)
        drift=dict(good,airspeed_indicated=84.7)
        # Actual failure: airspeed corrections occur sooner than the 2s window.
        # Every burst has only 0.25s of accepted live readings.
        frames=iter(([good,good,drift]*30))
        sim,fake,clock=self.run_settling(frames)
        fake._session_context={'test_reference':'1.c.(1)'}
        fake.ref_manual_snapshot.update(rpm=2618.2,airspeed=79.36,altitude_msl=3000.,telemetry={'prop_rpms':[2618.2]})
        fake._restore_reference_state=Mock()
        with clock,patch.object(gui.time,'sleep'):
            gui.SuperMushakLoggerApp._wait_for_reference_rpm(fake)
        self.assertGreater(fake._restore_reference_state.call_count,1)
        readiness=fake._session_context['rpm_startup_readiness']
        self.assertGreaterEqual(readiness['stable_seconds'],2.)
        self.assertEqual(readiness['actual_engine_rpm'],2697.2)
        self.assertLess(sim.wait_for_fresh_telemetry.call_count,80)

    def test_paused_restore_time_does_not_count_as_live_stability(self):
        good=dict(engine_rpm=2697.2,prop_rpms=[2697.2],airspeed_indicated=79.6,pressure_alt=3000.)
        drift=dict(good,airspeed_indicated=84.7)
        sim,fake,clock=self.run_settling(iter([good,drift]*50))
        fake._session_context={'test_reference':'1.c.(1)'}
        fake.ref_manual_snapshot.update(rpm=2618.2,airspeed=79.36,altitude_msl=3000.,telemetry={'prop_rpms':[2618.2]})
        fake._restore_reference_state=Mock()
        with clock,patch.object(gui.time,'sleep'),self.assertRaisesRegex(RuntimeError,'did not settle|drifted'):
            gui.SuperMushakLoggerApp._wait_for_reference_rpm(fake)
        self.assertNotIn('rpm_startup_readiness',fake._session_context)

    def test_climb_excessive_offset_still_blocks(self):
        frames=iter([dict(engine_rpm=2801.,prop_rpms=[2801.])]*100)
        sim,fake,clock=self.run_settling(frames)
        fake._session_context={'test_reference':'1.c.(1)'}
        with clock,patch.object(gui.time,'sleep'),self.assertRaisesRegex(RuntimeError,'did not settle'):
            gui.SuperMushakLoggerApp._wait_for_reference_rpm(fake)
        sim.pause_sim.assert_called_once()

    def test_engine_acceleration_restores_transient_without_live_settling(self):
        sim,fake,clock=self.run_settling(iter([]))
        fake._session_context={'test_reference':'1.f.(1)'}
        fake.ref_manual_snapshot={'rpm':2648.5,'throttle':5.,'telemetry':{'prop_rpms':[2648.5]}}
        fake._refresh_paused_telemetry=Mock(return_value={'engine_rpm':2648.5,'prop_rpms':[2648.5]})
        fake._restore_reference_engine_rpm=lambda:gui.SuperMushakLoggerApp._restore_reference_engine_rpm(fake)
        with clock:
            gui.SuperMushakLoggerApp._wait_for_reference_rpm(fake)
        sim.set_initial_engine_rpm.assert_called_once_with(2648.5)
        sim.unpause_sim.assert_not_called()
        self.assertEqual(fake._session_context['rpm_startup_readiness']['actual_engine_rpm'],2648.5)

    def test_engine_transient_wrong_readback_blocks(self):
        sim,fake,clock=self.run_settling(iter([]))
        fake._session_context={'test_reference':'1.f.(1)'}
        fake.ref_manual_snapshot={'rpm':2648.5,'telemetry':{'prop_rpms':[2648.5]}}
        fake._refresh_paused_telemetry=lambda:{'engine_rpm':1476.,'prop_rpms':[1476.]}
        with clock,self.assertRaisesRegex(RuntimeError,'failed readback'):
            gui.SuperMushakLoggerApp._restore_reference_engine_rpm(fake)
        sim.unpause_sim.assert_not_called()

    def test_engine_tests_keep_strict_startup_tolerance(self):
        for reference in ('1.f.(1)','1.f.(2)',None):
            self.assertEqual(gui.startup_rpm_tolerance(reference),10.)

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
