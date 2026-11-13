"""
brunner_interface.py - Brunner CLS2Sim Hardware Interface
=========================================================
Purpose:
  1. Manages UDP communication with Brunner CLS2Sim (127.0.0.1:15090).
  2. Reads Pilot Physical Inputs: Position (0x11), Applied Force (0x21), and Virtual Force (0x51).
  3. Commands Motor Actuators during Automatic Replay (Autopilot Override bitmask 0x07).
  4. Keeps all Brunner/Hardware control code completely isolated from simulator/P3D code.
"""

import socket
import math
import struct
import threading
from functools import wraps
from typing import Tuple, Optional


def _serialized(method):
    @wraps(method)
    def wrapped(self, *args, **kwargs):
        with self._io_lock:
            return method(self, *args, **kwargs)
    return wrapped


class AxisUnavailableError(RuntimeError):
    """A successful read returned no loader-node measurements for this axis."""


class BrunnerInterface:
    # Axis bitmask identifiers
    AXIS_PITCH  = 0x01  # Elevator / Pitch
    AXIS_ROLL   = 0x02  # Aileron / Roll
    AXIS_RUDDER = 0x04  # Rudder / Yaw
    ALL_AXES    = AXIS_PITCH | AXIS_ROLL | AXIS_RUDDER  # 0x07 (All 3 axes)

    def __init__(self, host: str = "127.0.0.1", port: int = 15090, timeout: float = 0.05):
        self._io_lock = threading.RLock()
        self.host = host
        self.port = port
        self.timeout = timeout
        self.sock: Optional[socket.socket] = None
        self._connected = False

    @_serialized
    def connect(self) -> bool:
        """Opens UDP socket to CLS2Sim."""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.settimeout(self.timeout)
            self.sock.connect((self.host, self.port))
            self._connected = True
            return True
        except Exception as e:
            print(f"[Brunner] Connection error: {e}")
            self._connected = False
            return False

    @_serialized
    def read_axis(self, axis: int) -> Tuple[float, float, float]:
        """
        Reads:
          1. Position (-1.0 to +1.0) via 0x11
          2. Applied Pilot Force (Load Cell) via 0x21
          3. Aerodynamic Virtual Force (Nm) via 0x51
        """
        if not self.sock:
            if getattr(self,"strict_io",False): raise RuntimeError("CLS2sim disconnected")
            return 0.0, 0.0, 0.0

        pos, force, v_force = 0.0, 0.0, 0.0
        try:
            # Drain any lingering UDP responses before reading
            self.sock.setblocking(False)
            while True:
                try:
                    self.sock.recv(1024)
                except BlockingIOError:
                    break
            self.sock.settimeout(self.timeout)

            if getattr(self,'_pid_transport',None) is not None and axis==self.AXIS_PITCH:
                pos=self._pid_transport.read_position()
            else:
                # 1. Position
                self.sock.send(struct.pack("<III", 0xD0, axis, 0x11))
                r = self.sock.recv(1024)
                if len(r) >= 9 and r[2] == 0:
                    if axis==self.AXIS_PITCH and getattr(self,'pid_position_nodes',False):
                        if struct.unpack_from('<H',r)[0]!=len(r)-2 or (len(r)-3)%6:
                            raise RuntimeError('Malformed PID position response')
                        nodes=[struct.unpack_from('<Hf',r,offset) for offset in range(3,len(r),6)]
                        candidates=[(node,value) for node,value in nodes if math.isfinite(value) and abs(value)<=1.05]
                        chosen=next((value for node,value in candidates if node==2),None)
                        if chosen is None:chosen=next((value for node,value in candidates if node==1),None)
                        if chosen is None:chosen=next((value for node,value in candidates),None)
                        if chosen is None:raise RuntimeError('PID Pitch position missing across all nodes')
                        pos=chosen
                    else:
                        pos = struct.unpack_from("<f", r, 5)[0]
                elif getattr(self,"strict_io",False):
                    if len(r)==3 and r[2]==0:
                        raise AxisUnavailableError(f"CLS2sim axis {axis} has no position measurement")
                    raise RuntimeError(f"Invalid position response for axis {axis}: {r.hex()}")

            # 2. Applied Hand/Foot Force (0x22 Engineering Units in Newtons, fallback to 0x21)
            self.sock.send(struct.pack("<III", 0xD0, axis, 0x22))
            r = self.sock.recv(1024)
            if len(r) >= 11 and r[2] == 0:
                force = struct.unpack_from("<f", r, 7)[0]
            elif len(r) >= 9 and r[2] == 0:
                force = struct.unpack_from("<f", r, 5)[0]
            else:
                self.sock.send(struct.pack("<III", 0xD0, axis, 0x21))
                r = self.sock.recv(1024)
                if len(r) >= 9 and r[2] == 0:
                    force = struct.unpack_from("<f", r, 5)[0]
                elif getattr(self,"strict_io",False): raise RuntimeError("Invalid force response")

            # 3. Virtual Force
            self.sock.send(struct.pack("<III", 0xD0, axis, 0x51))
            r = self.sock.recv(1024)
            if len(r) >= 11 and r[2] == 0:
                v_force = struct.unpack_from("<f", r, 7)[0]
            elif getattr(self,"strict_io",False): raise RuntimeError("Invalid virtual force response")
        except Exception:
            if getattr(self,"strict_io",False): raise

        if getattr(self,"strict_io",False):
            if not all(math.isfinite(v) for v in (pos,force,v_force)) or abs(pos)>1.05: raise RuntimeError("Invalid physical feedback")
        return pos, force, v_force

    @_serialized
    def write_setting(self, axis: int, setting_id: int, fmt: str, val):
        """Sends setting command (0xCE) to CLS2Sim."""
        if not self.sock:
            if getattr(self,"strict_io",False): raise RuntimeError("CLS2sim disconnected")
            return
        if getattr(self,'_pid_transport',None) is not None and axis==self.AXIS_PITCH and setting_id==0x80:
            self._pid_transport.position(float(val))
            return
        try:
            self.sock.send(struct.pack(f"<III{fmt}", 0xCE, axis, setting_id, val))
            response=self.sock.recv(1024)
            if getattr(self,"strict_io",False) and (len(response)<3 or response[2]!=0): raise RuntimeError("Actuator setting rejected")
        except Exception:
            if getattr(self,"strict_io",False): raise

    def _write_override(self, axis_mask: int, override_id: int) -> bool:
        if not self.sock:
            return False
        try:
            self.sock.send(struct.pack("<III",0xD1,axis_mask,override_id))
            response=self.sock.recv(1024)
            if len(response)<3 or response[2]!=0:
                raise RuntimeError(f"negative or incomplete ACK: {response.hex()}")
            return True
        except Exception as exc:
            print(f"[Brunner] Override 0x{override_id:02X}, mask=0x{axis_mask:02X} failed: {exc}")
            return False

    @_serialized
    def set_overrides(self, axis_mask: int, enable: bool) -> bool:
        """D1 ownership uses the axis mask; override IDs never change on release."""
        mask=axis_mask if enable else 0
        results=[self._write_override(mask,override_id) for override_id in (0x01,0x03,0x04)]
        return all(results)

    @_serialized
    def set_position_forwarding(self) -> bool:
        """Release Position-to-Sim override so CLS2Sim forwards physical controls."""
        return self._write_override(0,0x06)

    @_serialized
    def prepare_manual_mode(self):
        """Return ownership to CLS2Sim without commanding a new stick position."""
        self.write_setting(self.ALL_AXES,0x70,"B",0)
        self.write_setting(self.ALL_AXES,0xA0,"B",0)
        released=self.set_overrides(self.ALL_AXES,False)
        forwarded=self.set_position_forwarding()
        if not released or not forwarded:
            raise RuntimeError("Brunner did not acknowledge Manual control ownership.")

    @_serialized
    def configure_pid_mode(self, start_position):
        """V4.1 Pitch engagement sequence, on the existing serialized socket.

        Original engage arms the watchdog first. No replay force override, force scale,
        virtual-force enable, or other-axis motor commands are added here.
        """
        if not math.isfinite(float(start_position)) or not -.55<=float(start_position)<=.65:
            raise RuntimeError('Pitch loader is outside PID starting range.')
        from qtg_climb_pid import original_pitch_transport
        self._pid_transport=original_pitch_transport(self)
        self._pid_transport.engage(float(start_position))

    @_serialized
    def configure_replay_mode(self, axis_mask=None):
        """Prepares motors for automatic test replay with validated servo tuning."""
        axes=self.ALL_AXES if axis_mask is None else int(axis_mask)
        if not axes or axes & ~self.ALL_AXES: raise ValueError("Invalid automatic axis mask")
        if not self.set_overrides(axes, True) or not self.set_position_forwarding():
            raise RuntimeError("Brunner did not acknowledge Automatic ownership/physical-axis forwarding.")
        self.write_setting(axes, 0x20, "H", 100)   # Force Scale 100%
        self.write_setting(axes, 0xA0, "B", 1)     # Virtual Force Enable
        # Validated AP tracking force and speed parameters (prevents oscillation/thrashing)
        if axes & self.AXIS_PITCH: self.write_setting(self.AXIS_PITCH,  0x50, "H", 1800)  # Pitch AP force
        if axes & self.AXIS_PITCH: self.write_setting(self.AXIS_PITCH,  0x60, "H", 300)   # Pitch AP speed
        if axes & self.AXIS_ROLL: self.write_setting(self.AXIS_ROLL,   0x50, "H", 1500)  # Roll AP force
        if axes & self.AXIS_ROLL: self.write_setting(self.AXIS_ROLL,   0x60, "H", 200)   # Roll AP speed (validated smooth tracking)
        if axes & self.AXIS_RUDDER: self.write_setting(self.AXIS_RUDDER, 0x50, "H", 1500)  # Rudder AP force
        if axes & self.AXIS_RUDDER: self.write_setting(self.AXIS_RUDDER, 0x60, "H", 200)   # Rudder AP speed
        self.write_setting(axes, 0x70, "B", 1)     # Autopilot ON


    @_serialized
    def move_actuators(self, p_cmd: float, r_cmd: Optional[float], y_cmd: Optional[float]):
        """Commands physical actuators to target positions."""
        self.write_setting(self.AXIS_PITCH,  0x80, "f", p_cmd)
        if r_cmd is not None: self.write_setting(self.AXIS_ROLL,   0x80, "f", r_cmd)
        if y_cmd is not None: self.write_setting(self.AXIS_RUDDER, 0x80, "f", y_cmd)

    @_serialized
    def release_controls(self):
        """Safely returns motors to center and releases all autopilot overrides."""
        if not self.sock:
            return
        if getattr(self,'_pid_transport',None) is not None:
            try:self._pid_transport.release()
            finally:
                self._pid_transport=None
                self.pid_position_nodes=False
                self.sock.settimeout(self.timeout)
            return
        strict=getattr(self,"strict_io",False)
        self.strict_io=False
        try:
            self.write_setting(self.AXIS_PITCH,  0x80, "f", 0.0)
            self.write_setting(self.AXIS_ROLL,   0x80, "f", 0.0)
            self.write_setting(self.AXIS_RUDDER, 0x80, "f", 0.0)
            self.write_setting(self.ALL_AXES, 0x70, "B", 0)
            self.write_setting(self.ALL_AXES, 0xA0, "B", 0)
            self.set_overrides(self.ALL_AXES, False)
            self.set_position_forwarding()
        except Exception:
            pass
        finally:
            self.strict_io=strict

    @_serialized
    def close(self):
        """Closes UDP socket cleanly."""
        self.release_controls()
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None
        self._connected = False
        print("[Brunner] Overrides released and socket closed.")

