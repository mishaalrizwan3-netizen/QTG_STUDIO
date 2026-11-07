"""Offline Brunner/CLS2Sim stand-in with normalized control axes."""
from __future__ import annotations

import math
import threading


class MockBrunner:
    AXIS_PITCH = 0x01
    AXIS_ROLL = 0x02
    AXIS_RUDDER = 0x04
    ALL_AXES = AXIS_PITCH | AXIS_ROLL | AXIS_RUDDER

    def __init__(self, *, scenario=None):
        self.scenario = dict(scenario or {})
        self._lock = threading.RLock()
        self._connected = False
        self.strict_io = True
        self._axes = {self.AXIS_PITCH: 0.0, self.AXIS_ROLL: 0.0, self.AXIS_RUDDER: 0.0}
        self._targets = dict(self._axes)
        self._forces = dict(self._axes)
        self._virtual_forces = dict(self._axes)
        self._overrides = False
        self._released = True

    @property
    def sock(self):
        # Existing QTG connection checks use socket presence.
        return self if self._connected else None

    def connect(self):
        if self.scenario.get("disconnect"):
            return False
        self._connected = True
        return True

    def close(self):
        self.release_controls()
        self._connected = False

    def read_axis(self, axis):
        with self._lock:
            if not self._connected:
                raise RuntimeError("Mock Brunner is disconnected")
            if axis not in self._axes:
                raise ValueError("Unknown Brunner axis")
            value = self._axes[axis]
            return value, self._forces[axis], self._virtual_forces[axis]

    def move_actuators(self, p_cmd, r_cmd=None, y_cmd=None):
        with self._lock:
            if not self._connected:
                raise RuntimeError("Mock Brunner is disconnected")
            values = {self.AXIS_PITCH: p_cmd, self.AXIS_ROLL: r_cmd, self.AXIS_RUDDER: y_cmd}
            for axis, value in values.items():
                if value is not None:
                    value = float(value)
                    if not math.isfinite(value) or not -1.05 <= value <= 1.05:
                        raise ValueError("Mock Brunner command is outside normalized axis range")
                    self._targets[axis] = max(-1.0, min(1.0, value))
                    self._axes[axis] = self._targets[axis]

    def write_setting(self, axis, setting_id, fmt, val):
        if not self._connected:
            raise RuntimeError("Mock Brunner is disconnected")
        if setting_id == 0x80:
            self.move_actuators(
                val if axis == self.AXIS_PITCH else None,
                val if axis == self.AXIS_ROLL else None,
                val if axis == self.AXIS_RUDDER else None)
        elif setting_id == 0x70:
            self._overrides = bool(val)
        elif setting_id == 0xA0:
            self._virtual_forces = {key: float(val) for key in self._virtual_forces}
        return True

    def configure_replay_mode(self, axis_mask=None):
        if not self._connected:
            raise RuntimeError("Mock Brunner is disconnected")
        self._overrides = True
        self._released = False
        return True

    def prepare_manual_mode(self):
        self._overrides = False
        self._released = True
        return True

    def release_controls(self):
        with self._lock:
            self._targets = {axis: 0.0 for axis in self._axes}
            self._axes = dict(self._targets)
            self._overrides = False
            self._released = True
        return True

    def set_overrides(self, axis_mask, enable):
        self._overrides = bool(enable)
        return True

    def set_position_forwarding(self):
        self._overrides = False
        return True
