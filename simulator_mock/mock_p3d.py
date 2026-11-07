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
            "yaw_angle": 0.0,
            "pitch_angle": 0.0,
            "bank_angle": 0.0,
            "vertical_speed": 0.0,
            "indicated_altitude": 755.0,
            "engine_rpm": 700.0,
            "engine_throttle_percent": 5.0,
            "throttle": 5.0,
            "max_throttle": 5.0,
            "mach_number": 0.0,
            "radar_alt": 0.0,
            "ground_alt": 755.0,
            "aoa": 0.0,
            "sideslip": 0.0,
            "track_angle": 0.0,
            "accelX": 0.0, "accelY": 0.0, "accelZ": 0.0,
            "rotorRPM": 0.0, "fuelFlow": 0.0, "torque": 0.0, "egt": 0.0,
            "eng_rpms": [700.0, 0.0, 0.0, 0.0],
            "prop_rpms": [700.0, 0.0, 0.0, 0.0],
            "throttle1": 5.0, "throttle2": 0.0, "throttle3": 0.0, "throttle4": 0.0,
            "throttles": [5.0, 0.0, 0.0, 0.0],
            "flaps": 0.0, "measured_flaps_handle_index": 0.0,
            "plane_latitude": 32.235, "plane_longitude": 74.135,
            "on_ground": 1.0, "sim_on_ground": 1.0,
            "velocity_world_x": 0.0, "velocity_world_y": 0.0, "velocity_world_z": 0.0,
            "velocity_body_x": 0.0, "velocity_body_y": 0.0, "velocity_body_z": 0.0,
            "rotation_velocity_body_x": 0.0, "rotation_velocity_body_y": 0.0, "rotation_velocity_body_z": 0.0,
            "trim_elevator_angle_rad": 0.0, "trim_elevator_percent": 0.0,
            "trim_aileron_percent": 0.0, "trim_rudder_percent": 0.0,
            "surface_elevator_rad": 0.0, "surface_aileron_left_rad": 0.0,
            "surface_aileron_right_rad": 0.0, "surface_rudder_rad": 0.0,
            "engine_mixture_percent": 100.0, "engine_propeller_percent": 100.0,
            "propeller_blade_angle_rad": 0.0, "engine_torque_ft_lb": 0.0,
            "engine_fuel_air_ratio": 0.0, "engine_combustion_1": 1.0,
            "engine_fuel_valve_1": 1.0, "engine_fuel_pressure_psi_1": 0.0,
            "total_weight": 851.4, "total_fuel_quantity": 112.2,
            "baro_pressure": 1013.25, "ambient_wind_dir": 0.0, "ambient_wind_spd": 0.0,
            "roll_rate": 0.0, "pitch_rate": 0.0, "yaw_rate": 0.0,
            "extended_causal_sequence": 1, "extended_causal_layout": "MOCK",
            "parking_brake_position": 1.0,
            "ambient_temp": 15.0, "ambient_pressure_mb": 1013.25,
            "fuel_flow_pph": 0.0, "egt_c": 0.0, "engine_manifold_pressure_psi_1": 0.0,
            "bridge_version": "MOCK", "bridge_telemetry_sequence": 0, "last_updated": time.time(),
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
        # Refresh telemetry sequence without advancing physics while paused.
        with self._lock:
            if self._paused:
                self._sequence += 1
                self._state["bridge_telemetry_sequence"] = self._sequence
            else:
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

    def set_position_airborne(
        self,
        alt_ft=3000.0,
        speed_kts=80.0,
        pitch_deg=0.0,
        bank_deg=0.0,
        hdg_deg=48.4,
        lat=32.23383333,
        lon=74.12366667,
        throttle=80.0,
    ):
        """Match SimConnectClient.set_position_airborne positional/default API."""
        return self.queue_position_airborne(
            alt_ft=alt_ft,
            speed_kts=speed_kts,
            pitch_deg=pitch_deg,
            bank_deg=bank_deg,
            hdg_deg=hdg_deg,
            lat=lat,
            lon=lon,
            throttle=throttle,
            on_ground=False,
        )

    def set_exact_pose_state_no_airspeed(self, state):
        try:
            return self.queue_position_airborne(
                alt_ft=state["altitude_ft"], speed_kts=0.0,
                pitch_deg=state["pitch_deg"], bank_deg=state["bank_deg"],
                hdg_deg=state["true_heading_deg"], lat=state["latitude_deg"],
                lon=state["longitude_deg"], on_ground=bool(self._state["on_ground"]))
        except (KeyError, TypeError, ValueError):
            return False

    def set_initial_instruments(self, vertical_speed_fpm, indicated_altitude_ft):
        with self._lock:
            self._state["vertical_speed"] = float(vertical_speed_fpm)
            self._state["indicated_altitude"] = float(indicated_altitude_ft)
            self._sequence += 1
        return True

    def set_initial_dynamic_state(self, state):
        keys = ("velocity_world_x_fps", "velocity_world_y_fps", "velocity_world_z_fps",
                "velocity_body_x_fps", "velocity_body_y_fps", "velocity_body_z_fps",
                "rotation_velocity_body_x_rad_s", "rotation_velocity_body_y_rad_s",
                "rotation_velocity_body_z_rad_s")
        try:
            values = {key: float(state[key]) for key in keys}
            if not all(math.isfinite(value) for value in values.values()):
                return False
            with self._lock:
                self._state.update(values)
                self._sequence += 1
            return True
        except (KeyError, TypeError, ValueError):
            return False

    def set_initial_trim(self, elevator_trim_rad, aileron_trim_pct, rudder_trim_pct):
        with self._lock:
            self._state.update({"trim_elevator_angle_rad": float(elevator_trim_rad),
                                "trim_aileron_percent": float(aileron_trim_pct),
                                "trim_rudder_percent": float(rudder_trim_pct)})
            self._sequence += 1
        return True

    def set_payload_station_weights(self, *args, **kwargs):
        return True

    def set_all_fuel_tank_levels(self, *args, **kwargs):
        return True

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
        self._weather_metar = str(metar or "")
        return True

    def get_global_weather_metar(self):
        return getattr(self, "_weather_metar", "")

    def set_initial_engine_levers(self, *args, **kwargs):
        return True

    def set_engine_controls(self, *args, **kwargs):
        return True

    def get_engine_support_state(self, fresh=False, timeout=0.6):
        return {"egt_c": self._state["egt_c"]}

    def restore_engine_thermal_state(self, state, timeout=3.0):
        with self._lock:
            for key, value in dict(state or {}).items():
                if isinstance(value, (int, float)) and math.isfinite(float(value)):
                    self._state[key] = float(value)
            self._sequence += 1
        return self.get_engine_support_state()

    def set_initial_fuel_pressure(self, *args, **kwargs):
        return True

    def set_daytime(self):
        return True

    def set_fuel_tank_levels(self, *args, **kwargs):
        return True

    def set_linear_body_acceleration(self, *args, **kwargs):
        return True

    def set_rotation_body_acceleration(self, *args, **kwargs):
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
        self._state["throttle"] = throttle
        self._state["last_updated"] = now
        self._state["fuel_flow_pph"] = max(0.0, throttle * 0.42)
        self._state["egt_c"] = 250.0 + throttle * 3.0
        # Deliberate fault scenario: full power can cause ground creep despite brake.
        creep = bool(self.scenario.get("ground_creep", False))
        if self._state["on_ground"] and creep and throttle >= self.scenario.get("creep_throttle", 95.0):
            speed = float(self.scenario.get("creep_speed_kts", 1.2))
            self._state["ground_speed"] = speed
            self._state["airspeed_indicated"] = speed
            self._state["plane_latitude"] += speed * 0.514444 * dt / 111320.0
            self._state["velocity_world_x"] = speed * 0.514444
            self._state["velocity_body_x"] = speed * 0.514444
        elif self._state["on_ground"]:
            self._state["ground_speed"] = 0.0
            self._state["airspeed_indicated"] = 0.0
            self._state["velocity_world_x"] = 0.0
            self._state["velocity_world_y"] = 0.0
            self._state["velocity_world_z"] = 0.0
            self._state["velocity_body_x"] = 0.0
            self._state["velocity_body_y"] = 0.0
            self._state["velocity_body_z"] = 0.0
        self._sequence += 1
        self._state["bridge_telemetry_sequence"] = self._sequence
