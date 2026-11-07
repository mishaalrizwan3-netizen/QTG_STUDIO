"""Deterministic, offline Prepar3D/SimConnect stand-in.

Implements the small, explicit subset of the current SimConnect client API
needed by QTG initialization and recording. Dynamics are intentionally simple;
they validate software flow, not aircraft performance.
"""
from __future__ import annotations

import math
import threading
import time


class MockP3D:
    def __init__(self, app_name="QTG Mock P3D", *, scenario=None):
        self.app_name = app_name
        self.scenario = dict(scenario or {})
        self._lock = threading.RLock()
        self._connected = False
        self._paused = True
        self._last_tick = time.monotonic()
        self._sequence = 0
        self._state = {
            "title": "SIMULATION ONLY - Super Mushshak",
            "number_of_engines": 1,
            "engine_throttle_percent": 5.0,
            "airspeed_indicated": 0.0,
            "ground_speed": 0.0,
            "vertical_speed": 0.0,
            "indicated_altitude": 755.0,
            "pressure_alt": 755.0,
            "magnetic_heading": 0.0,
            "true_heading": 0.0,
            "pitch_angle": 0.0,
            "bank_angle": 0.0,
            "engine_rpm": 700.0,
            "eng_rpms": [700.0, 0.0, 0.0, 0.0],
            "prop_rpms": [700.0, 0.0, 0.0, 0.0],
            "throttle1": 5.0, "throttle2": 0.0, "throttle3": 0.0, "throttle4": 0.0,
            "throttles": [5.0, 0.0, 0.0, 0.0],
            "flaps": 0.0, "measured_flaps_handle_index": 0.0,
            "plane_latitude": 32.235, "plane_longitude": 74.135,
            "on_ground": 1.0, "sim_on_ground": 1.0,
            "parking_brake_position": 1.0,
            "ambient_temp": 15.0, "ambient_pressure_mb": 1013.25,
            "fuel_flow_pph": 0.0, "egt_c": 0.0, "engine_manifold_pressure_psi_1": 0.0,
            "bridge_version": "MOCK", "bridge_telemetry_sequence": 0,
        }
        self._rpm_target = 700.0
        self._throttle_target = 5.0
        self._ground_speed_override = None

    def connect_and_run(self):
        with self._lock:
            self._connected = True
            self._last_tick = time.monotonic()
        return True

    def connect(self):
        return self.connect_and_run()

    def stop(self):
        with self._lock:
            self._connected = False

    def is_connected(self):
        with self._lock:
            return self._connected

    def pause_sim(self):
        with self._lock:
            self._advance()
            self._paused = True

    def unpause_sim(self):
        with self._lock:
            self._last_tick = time.monotonic()
            self._paused = False

    def wait_for_paused(self, timeout_sec=3.0):
        return self._paused

    def simulation_running(self):
        return not self._paused

    def get_telemetry_sequence(self):
        return self._sequence

    def request_telemetry_once(self):
        with self._lock:
            self._advance(force=True)
        return True

    def wait_for_fresh_telemetry(self, previous_sequence, timeout_sec=1.0):
        self.request_telemetry_once()
        return self.get_telemetry() if self._sequence > previous_sequence else None

    def get_telemetry(self):
        with self._lock:
            self._advance()
            result = dict(self._state)
            result["eng_rpms"] = list(self._state["eng_rpms"])
            result["prop_rpms"] = list(self._state["prop_rpms"])
            result["throttles"] = list(self._state["throttles"])
            return result

    def queue_position_airborne(self, alt_ft, speed_kts, pitch_deg, bank_deg,
                                hdg_deg, lat, lon, throttle=80.0, on_ground=False):
        with self._lock:
            self._state.update({
                "pressure_alt": float(alt_ft), "indicated_altitude": float(alt_ft),
                "airspeed_indicated": 0.0 if on_ground else float(speed_kts),
                "ground_speed": 0.0 if on_ground else float(speed_kts),
                "pitch_angle": float(pitch_deg), "bank_angle": float(bank_deg),
                "true_heading": float(hdg_deg) % 360.0,
                "magnetic_heading": float(hdg_deg) % 360.0,
                "plane_latitude": float(lat), "plane_longitude": float(lon),
                "on_ground": 1.0 if on_ground else 0.0,
                "sim_on_ground": 1.0 if on_ground else 0.0,
            })
            self._ground_speed_override = 0.0 if on_ground else None
            self._sequence += 1
        return True

    def set_position_airborne(self, **kwargs):
        return self.queue_position_airborne(**kwargs)

    def set_initial_throttle(self, percent):
        with self._lock:
            value = max(0.0, min(100.0, float(percent)))
            self._throttle_target = value
            self._state["engine_throttle_percent"] = value
            self._state["throttle1"] = value
            self._state["throttle"] = value
            self._state["throttles"][0] = value
            self._sequence += 1
        return True

    def set_throttle(self, percent):
        return self.set_initial_throttle(percent)

    def set_replay_throttle(self, percent):
        return self.set_initial_throttle(percent)

    def set_initial_flaps(self, index):
        with self._lock:
            self._state["flaps"] = float(index)
            self._state["measured_flaps_handle_index"] = float(index)
            self._sequence += 1
        return True

    def set_flaps_handle_index(self, index):
        return self.set_initial_flaps(index)

    def set_global_weather_metar(self, metar):
        return True

    def get_global_weather_metar(self):
        return ""

    def set_initial_engine_levers(self, *args, **kwargs):
        return True

    def set_engine_controls(self, *args, **kwargs):
        return True

    def get_engine_support_state(self):
        return {}

    def restore_engine_thermal_state(self, *args, **kwargs):
        return True

    def set_initial_fuel_pressure(self, *args, **kwargs):
        return True

    def _advance(self, force=False):
        now = time.monotonic()
        dt = max(0.0, min(0.25, now - self._last_tick))
        self._last_tick = now
        if self._paused and not force:
            return
        throttle = self._throttle_target
        target_rpm = 650.0 + 20.5 * throttle
        tau = 1.8
        self._rpm_target += (target_rpm - self._rpm_target) * (1.0 - math.exp(-dt / tau))
        rpm = self._rpm_target
        self._state["engine_rpm"] = rpm
        self._state["eng_rpms"][0] = rpm
        self._state["prop_rpms"][0] = rpm
        self._state["engine_throttle_percent"] = throttle
        self._state["throttle1"] = throttle
        self._state["throttles"][0] = throttle
        self._state["fuel_flow_pph"] = max(0.0, throttle * 0.42)
        self._state["egt_c"] = 250.0 + throttle * 3.0
        # Deliberate fault scenario: full power can cause ground creep despite brake.
        creep = bool(self.scenario.get("ground_creep", False))
        if self._state["on_ground"] and creep and throttle >= self.scenario.get("creep_throttle", 95.0):
            speed = float(self.scenario.get("creep_speed_kts", 1.2))
            self._state["ground_speed"] = speed
            self._state["airspeed_indicated"] = speed
            self._state["plane_latitude"] += speed * 0.514444 * dt / 111320.0
        elif self._state["on_ground"]:
            self._state["ground_speed"] = 0.0
            self._state["airspeed_indicated"] = 0.0
        self._sequence += 1
        self._state["bridge_telemetry_sequence"] = self._sequence
