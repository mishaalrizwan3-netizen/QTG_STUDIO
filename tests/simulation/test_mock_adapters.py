"""Smoke tests for the offline hardware adapters; no real simulator required."""
import unittest

from simulator_mock import MockP3D, MockBrunner
from brunner.src.replay_support import validate_initial_airspeed, ThrottleReplay


class MockAdapterTests(unittest.TestCase):
    def test_zero_ias_ground_deceleration_validation_accepts_stationary_state(self):
        stationary = {
            "airspeed_indicated": 0.0,
            "sim_on_ground": 1.0,
            "ground_speed": 0.0,
        }
        self.assertEqual(validate_initial_airspeed(
            stationary, 0.0, on_ground=True), 0.0)

        moving = dict(stationary, ground_speed=1.2)
        with self.assertRaisesRegex(ValueError, "moving on the ground"):
            validate_initial_airspeed(moving, 0.0, on_ground=True)

    def test_engine_deceleration_manual_initialization_keeps_aircraft_on_ground(self):
        import sys
        import threading
        from pathlib import Path

        root = Path(__file__).resolve().parents[2]
        sys.path.insert(0, str(root / "QTG_Studio_Antigravity_Single_Main"))
        sys.path.insert(0, str(root / "brunner" / "src"))
        import qtg_common_engine

        sim = MockP3D()
        sim.connect()

        class FakeWindow:
            def __init__(self):
                self.p3d = sim
                self.stop_worker_event = threading.Event()

            def _wait_for_initial_altitude(self, altitude):
                state = sim.get_telemetry()
                if abs(state["pressure_alt"] - altitude) > 25:
                    raise AssertionError("Mock aircraft did not reach requested ground-test altitude")

            def _refresh_paused_telemetry(self):
                sequence = sim.get_telemetry_sequence()
                self.p3d.request_telemetry_once()
                state = self.p3d.wait_for_fresh_telemetry(sequence, 2.0)
                if state is None:
                    raise AssertionError("Mock P3D did not return fresh paused telemetry")
                if "engine_oil_temperature_c_1" not in state:
                    state.update(self.p3d.get_engine_support_state(fresh=True))
                return state

        window = FakeWindow()
        actual = qtg_common_engine.apply_manual_conditions(window, {
            "alt_ft": 755, "ias_kt": 0, "throttle": 100, "flaps": 0,
            "lat": 32.235, "lon": 74.135, "temperature_c": 15,
            "on_ground": True,
        })
        self.assertEqual(actual["sim_on_ground"], 1.0)
        self.assertEqual(actual["airspeed_indicated"], 0.0)
        self.assertEqual(actual["ground_speed"], 0.0)
        self.assertEqual(actual["engine_throttle_percent"], 100.0)

    def test_p3d_connect_pause_and_ground_initial_state(self):
        sim = MockP3D()
        self.assertTrue(sim.connect_and_run())
        self.assertTrue(sim.is_connected())
        sim.pause_sim()
        self.assertTrue(sim.wait_for_paused())
        self.assertTrue(sim.queue_position_airborne(
            alt_ft=755, speed_kts=0, pitch_deg=0, bank_deg=0,
            hdg_deg=0, lat=32.235, lon=74.135, on_ground=True))
        state = sim.get_telemetry()
        self.assertEqual(state["airspeed_indicated"], 0.0)
        self.assertEqual(state["ground_speed"], 0.0)
        self.assertEqual(state["sim_on_ground"], 1.0)
        self.assertAlmostEqual(state["indicated_altitude"], 755.0)

    def test_position_airborne_matches_real_client_signature(self):
        sim = MockP3D()
        self.assertTrue(sim.set_position_airborne(
            2000, 80, 1.0, 2.0, 48.4, 32.235, 74.135, 35.0))
        self.assertTrue(sim.wait_for_paused())
        state = sim.get_telemetry()
        self.assertAlmostEqual(state["indicated_altitude"], 2000.0)
        self.assertAlmostEqual(state["airspeed_indicated"], 80.0)
        self.assertEqual(state["sim_on_ground"], 0.0)

    def test_connect_and_run_accepts_real_startup_options(self):
        connected = []
        sim = MockP3D()
        self.assertTrue(sim.connect_and_run(
            on_connected=lambda: connected.append(True),
            place_airborne=True, alt_ft=2500, speed_kts=90,
            initial_pitch=2, initial_bank=1, initial_hdg=45,
            initial_lat=32.2, initial_lon=74.1, initial_throttle=40))
        self.assertEqual(connected, [True])
        state = sim.get_telemetry()
        self.assertAlmostEqual(state["indicated_altitude"], 2500)
        self.assertAlmostEqual(state["airspeed_indicated"], 90)

    def test_engine_support_telemetry_has_required_temperature_fields(self):
        sim = MockP3D()
        support = sim.get_engine_support_state(fresh=True)
        self.assertIn("engine_oil_temperature_c_1", support)
        self.assertIn("engine_cylinder_head_temperature_c_1", support)
        self.assertIn("engine_carburetor_temperature_c_1", support)

    def test_paused_telemetry_request_does_not_advance_engine(self):
        sim = MockP3D()
        sim.connect()
        sim.pause_sim()
        before = sim.get_telemetry()["engine_rpm"]
        sim.request_telemetry_once()
        after = sim.get_telemetry()["engine_rpm"]
        self.assertEqual(before, after)

    def test_mock_p3d_rejects_invalid_initial_controls_like_real_client(self):
        sim = MockP3D()
        with self.assertRaises(ValueError):
            sim.set_initial_throttle(101)
        with self.assertRaises(ValueError):
            sim.set_initial_flaps(1.5)
        self.assertFalse(sim.set_replay_throttle(101))
        self.assertFalse(sim.set_throttle(50))
        self.assertFalse(sim.set_flaps_handle_index(1))

    def test_engine_rpm_mock_targets_match_engine_test_power_points(self):
        sim = MockP3D()
        sim.set_initial_throttle(5)
        for _ in range(60):
            sim._last_tick -= 0.25
            sim._advance(force=True)
        self.assertAlmostEqual(sim.get_telemetry()["engine_rpm"], 705.0, delta=2.0)

        sim.set_initial_throttle(100)
        for _ in range(60):
            sim._last_tick -= 0.25
            sim._advance(force=True)
        self.assertAlmostEqual(sim.get_telemetry()["engine_rpm"], 2700.0, delta=2.0)

    def test_ground_creep_fault_is_explicit(self):
        sim = MockP3D(scenario={"ground_creep": True})
        sim.connect()
        sim.queue_position_airborne(
            alt_ft=755, speed_kts=0, pitch_deg=0, bank_deg=0,
            hdg_deg=0, lat=32.235, lon=74.135, on_ground=True)
        sim.set_initial_throttle(100)
        sim.unpause_sim()
        sim._last_tick -= 0.2
        state = sim.get_telemetry()
        self.assertGreater(state["ground_speed"], 0.0)

    def test_brunner_axis_commands_and_release(self):
        brunner = MockBrunner()
        self.assertTrue(brunner.connect())
        brunner.configure_replay_mode()
        self.assertEqual(brunner.overrides, brunner.ALL_AXES)
        self.assertTrue(brunner._autopilot_enabled)
        self.assertTrue(brunner._virtual_force_enabled)
        self.assertEqual(brunner._settings[(brunner.AXIS_PITCH, 0x50)], 1800)
        self.assertEqual(brunner._settings[(brunner.AXIS_PITCH, 0x60)], 300)
        brunner.move_actuators(0.25, -0.1, 0.05)
        self.assertAlmostEqual(brunner.read_axis(brunner.AXIS_PITCH)[0], 0.25)
        self.assertAlmostEqual(brunner.read_axis(brunner.AXIS_ROLL)[0], -0.1)
        brunner.prepare_manual_mode()
        self.assertEqual(brunner.overrides, 0)
        self.assertFalse(brunner._autopilot_enabled)
        self.assertFalse(brunner._virtual_force_enabled)
        brunner.release_controls()
        self.assertAlmostEqual(brunner.read_axis(brunner.AXIS_PITCH)[0], 0.0)

    def test_brunner_replay_mode_respects_axis_mask(self):
        brunner = MockBrunner()
        brunner.connect()
        axes = brunner.AXIS_PITCH | brunner.AXIS_ROLL
        brunner.configure_replay_mode(axes)
        self.assertEqual(brunner.overrides, axes)
        self.assertIn((brunner.AXIS_PITCH, 0x50), brunner._settings)
        self.assertIn((brunner.AXIS_ROLL, 0x50), brunner._settings)
        self.assertNotIn((brunner.AXIS_RUDDER, 0x50), brunner._settings)


    def test_replay_throttle_interpolates_and_holds_end_value(self):
        replay = ThrottleReplay([0.0, 1.0, 2.0], [5.0, 50.0, 100.0])
        self.assertEqual(replay.command(0.0), 5.0)
        self.assertAlmostEqual(replay.command(0.5), 27.5)
        self.assertAlmostEqual(replay.command(1.5), 75.0)
        self.assertAlmostEqual(replay.command(2.5), 100.0)

    def test_replay_throttle_rejects_non_monotonic_clock(self):
        replay = ThrottleReplay([0.0, 1.0], [5.0, 100.0])
        replay.command(0.5)
        with self.assertRaisesRegex(ValueError, "monotonic"):
            replay.command(0.4)


if __name__ == "__main__":
    unittest.main()
