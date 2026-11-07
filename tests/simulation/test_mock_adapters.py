"""Smoke tests for the offline hardware adapters; no real simulator required."""
import unittest

from simulator_mock import MockP3D, MockBrunner


class MockAdapterTests(unittest.TestCase):
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
        state = sim.get_telemetry()
        self.assertAlmostEqual(state["indicated_altitude"], 2000.0)
        self.assertAlmostEqual(state["airspeed_indicated"], 80.0)
        self.assertEqual(state["sim_on_ground"], 0.0)

    def test_paused_telemetry_request_does_not_advance_engine(self):
        sim = MockP3D()
        sim.connect()
        sim.pause_sim()
        before = sim.get_telemetry()["engine_rpm"]
        sim.request_telemetry_once()
        after = sim.get_telemetry()["engine_rpm"]
        self.assertEqual(before, after)

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


if __name__ == "__main__":
    unittest.main()
