"""Offline Brunner/CLS2Sim stand-in with protocol-compatible control settings."""
from __future__ import annotations

import math
import threading


class MockBrunner:
    AXIS_PITCH = 0x01
    AXIS_ROLL = 0x02
    AXIS_RUDDER = 0x04
    ALL_AXES = AXIS_PITCH | AXIS_ROLL | AXIS_RUDDER

    def __init__(self, host='127.0.0.1', port=15090, timeout=0.05, *, scenario=None):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.scenario = dict(scenario or {})
        self._lock = threading.RLock()
        self._connected = False
        self.strict_io = True
        self._axes = {self.AXIS_PITCH: 0.0, self.AXIS_ROLL: 0.0, self.AXIS_RUDDER: 0.0}
        self._targets = dict(self._axes)
        self._forces = dict(self._axes)
        self._virtual_forces = dict(self._axes)
        self._override_mask = 0
        self._position_forwarding = True
        self._autopilot_enabled = False
        self._virtual_force_enabled = False
        self._settings = {}

    @property
    def sock(self):
        # Existing QTG connection checks use socket presence.
        return self if self._connected else None

    @property
    def overrides(self):
        return self._override_mask

    def connect(self):
        if self.scenario.get("disconnect"):
            return False
        self._connected = True
        return True

    def close(self):
        self.release_controls()
        self._connected = False

    def _require_connection(self):
        if not self._connected:
            raise RuntimeError("Mock Brunner is disconnected")

    def read_axis(self, axis):
        with self._lock:
            self._require_connection()
            if axis not in self._axes:
                raise ValueError("Unknown Brunner axis")
            return self._axes[axis], self._forces[axis], self._virtual_forces[axis]

    def move_actuators(self, p_cmd, r_cmd=None, y_cmd=None):
        with self._lock:
            self._require_connection()
            values = {self.AXIS_PITCH: p_cmd, self.AXIS_ROLL: r_cmd, self.AXIS_RUDDER: y_cmd}
            for axis, value in values.items():
                if value is not None:
                    value = float(value)
                    if not math.isfinite(value) or not -1.05 <= value <= 1.05:
                        raise ValueError("Mock Brunner command is outside normalized axis range")
                    self._targets[axis] = max(-1.0, min(1.0, value))
                    self._axes[axis] = self._targets[axis]

    def write_setting(self, axis, setting_id, fmt, val):
        """Model the setting IDs actually sent by BrunnerInterface."""
        with self._lock:
            self._require_connection()
            if axis <= 0 or axis & ~self.ALL_AXES:
                raise ValueError("Unknown Brunner axis or axis mask")
            self._settings[(axis, setting_id)] = val
            if setting_id == 0x80:  # Position command
                if axis not in self._axes:
                    raise ValueError("Position command requires one axis")
                self.move_actuators(
                    val if axis == self.AXIS_PITCH else None,
                    val if axis == self.AXIS_ROLL else None,
                    val if axis == self.AXIS_RUDDER else None)
            elif setting_id == 0x70:  # Autopilot ON/OFF
                self._autopilot_enabled = bool(val)
            elif setting_id == 0xA0:  # Virtual force enable
                self._virtual_force_enabled = bool(val)
            elif setting_id == 0x20:  # Force scale
                self._settings[("force_scale", axis)] = int(val)
            elif setting_id in (0x50, 0x60):  # AP force and speed tuning
                self._settings[(axis, setting_id)] = int(val)
            return True

    def _write_override(self, axis_mask, override_id):
        if not self._connected:
            return False
        # The mock records ownership state, not UDP packet exchange.
        if override_id in (0x01, 0x03, 0x04):
            self._override_mask = axis_mask
        elif override_id == 0x06:
            self._position_forwarding = True
        return True

    def set_overrides(self, axis_mask, enable):
        if axis_mask & ~self.ALL_AXES or axis_mask < 0:
            raise ValueError("Invalid Brunner axis mask")
        mask = axis_mask if enable else 0
        results = [self._write_override(mask, override_id) for override_id in (0x01, 0x03, 0x04)]
        return all(results)

    def set_position_forwarding(self):
        return self._write_override(0, 0x06)

    def configure_replay_mode(self, axis_mask=None):
        self._require_connection()
        axes = self.ALL_AXES if axis_mask is None else int(axis_mask)
        if not axes or axes & ~self.ALL_AXES:
            raise ValueError("Invalid automatic axis mask")
        if not self.set_overrides(axes, True) or not self.set_position_forwarding():
            raise RuntimeError("Mock Brunner could not acquire automatic control")
        self.write_setting(axes, 0x20, "H", 100)
        self.write_setting(axes, 0xA0, "B", 1)
        if axes & self.AXIS_PITCH:
            self.write_setting(self.AXIS_PITCH, 0x50, "H", 1800)
            self.write_setting(self.AXIS_PITCH, 0x60, "H", 300)
        if axes & self.AXIS_ROLL:
            self.write_setting(self.AXIS_ROLL, 0x50, "H", 1500)
            self.write_setting(self.AXIS_ROLL, 0x60, "H", 200)
        if axes & self.AXIS_RUDDER:
            self.write_setting(self.AXIS_RUDDER, 0x50, "H", 1500)
            self.write_setting(self.AXIS_RUDDER, 0x60, "H", 200)
        self.write_setting(axes, 0x70, "B", 1)
        return True

    def prepare_manual_mode(self):
        self._require_connection()
        self.write_setting(self.ALL_AXES, 0x70, "B", 0)
        self.write_setting(self.ALL_AXES, 0xA0, "B", 0)
        released = self.set_overrides(self.ALL_AXES, False)
        forwarded = self.set_position_forwarding()
        if not released or not forwarded:
            raise RuntimeError("Mock Brunner did not release Manual control ownership")
        return True

    def release_controls(self):
        with self._lock:
            if not self._connected:
                return True
            for axis in (self.AXIS_PITCH, self.AXIS_ROLL, self.AXIS_RUDDER):
                self.write_setting(axis, 0x80, "f", 0.0)
            self.write_setting(self.ALL_AXES, 0x70, "B", 0)
            self.write_setting(self.ALL_AXES, 0xA0, "B", 0)
            self.set_overrides(self.ALL_AXES, False)
            self.set_position_forwarding()
            self._targets = {axis: 0.0 for axis in self._axes}
            self._axes = dict(self._targets)
            return True
