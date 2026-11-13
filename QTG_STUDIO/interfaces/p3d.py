"""Simulator telemetry and command bridge for the visible QTG dashboard."""

import ctypes
import os
import platform
from pathlib import Path


class QTG_P3D_State(ctypes.Structure):
    _pack_ = 8
    _fields_ = [
        ("size", ctypes.c_uint32),
        ("abi_version", ctypes.c_uint32),
        ("connected", ctypes.c_int32),
        ("telemetry_valid", ctypes.c_int32),
        ("paused", ctypes.c_int32),
        ("sim_running", ctypes.c_int32),
        ("telemetry_age_ms", ctypes.c_uint64),
        ("last_error", ctypes.c_int32),
        ("reserved", ctypes.c_int32),
        ("pitch_deg", ctypes.c_double),
        ("bank_deg", ctypes.c_double),
        ("ias_kt", ctypes.c_double),
        ("altitude_ft", ctypes.c_double),
        ("throttle_percent", ctypes.c_double),
        ("engine_rpm", ctypes.c_double),
        ("prop_rpm", ctypes.c_double),
        ("weight_kg", ctypes.c_double),
        ("temperature_c", ctypes.c_double),
        ("elevator_position", ctypes.c_double),
        ("aileron_position", ctypes.c_double),
        ("rudder_position", ctypes.c_double),
    ]


class P3DClient:
    def __init__(self, dll_path):
        self.dll_path = Path(dll_path)
        self.dll = None
        self.error = ""
        self.running = False

    def start(self) -> bool:
        if platform.system() != "Windows":
            self.error = "Sim Flight Model bridge requires Windows x64."
            return False
        if not self.dll_path.is_file():
            self.error = f"Bridge not built: {self.dll_path}\nRun BUILD_P3D_BRIDGE.bat first."
            return False
        try:
            self.dll = ctypes.CDLL(str(self.dll_path))
            self.dll.QTG_BridgeVersion.restype = ctypes.c_int
            self.dll.QTG_BridgeStart.restype = ctypes.c_int
            self.dll.QTG_BridgeStop.restype = None
            self.dll.QTG_GetState.argtypes = [ctypes.POINTER(QTG_P3D_State)]
            self.dll.QTG_GetState.restype = ctypes.c_int
            if self.dll.QTG_BridgeVersion() != 1:
                raise RuntimeError("Bridge ABI version must be 1")
            self.running = self.dll.QTG_BridgeStart() == 1
            if not self.running:
                raise RuntimeError("Bridge thread could not start (SimConnect)")
            self.error = ""
            return True
        except (OSError, ValueError, RuntimeError) as exc:
            self.error = str(exc)
            self.dll = None
            self.running = False
            return False

    def snapshot(self) -> QTG_P3D_State | None:
        if not self.running or self.dll is None:
            return None
        state = QTG_P3D_State()
        state.size = ctypes.sizeof(QTG_P3D_State)
        if not self.dll.QTG_GetState(ctypes.byref(state)):
            self.error = "Unable to read bridge snapshot"
            return None
        return state

    def stop(self) -> None:
        if self.running and self.dll is not None:
            self.dll.QTG_BridgeStop()
        self.running = False
        self.dll = None


class InitPositionStruct(ctypes.Structure):
    _pack_ = 8
    _fields_ = [
        ("Latitude",  ctypes.c_double),
        ("Longitude", ctypes.c_double),
        ("Altitude",  ctypes.c_double),
        ("Pitch",     ctypes.c_double),
        ("Bank",      ctypes.c_double),
        ("Heading",   ctypes.c_double),
        ("OnGround",  ctypes.c_uint32),
        ("Airspeed",  ctypes.c_uint32),
    ]


class BridgeWriter:
    """In-process controller for setting aircraft position and throttle commands via p3d_bridge.dll."""
    def __init__(self, base, meipass, local_dir):
        self.base = Path(base)
        self.meipass = Path(meipass)
        self.local_dir = Path(local_dir)
        self.dll = None
        self._init()

    def _init(self):
        cand = [
            self.base / "p3d_bridge.dll",
            self.meipass / "p3d_bridge.dll",
            self.local_dir / "p3d_bridge.dll",
        ]
        for p in cand:
            if p.is_file():
                try:
                    if hasattr(os, "add_dll_directory"):
                        try:
                            os.add_dll_directory(str(p.parent))
                        except Exception:
                            pass
                    self.dll = ctypes.CDLL(str(p))
                    if hasattr(self.dll, "SetThrottle"):
                        self.dll.SetThrottle.argtypes = [ctypes.c_double]
                        self.dll.SetThrottle.restype = None
                    if hasattr(self.dll, "PauseSim"):
                        self.dll.PauseSim.restype = None
                    if hasattr(self.dll, "UnpauseSim"):
                        self.dll.UnpauseSim.restype = None
                    if hasattr(self.dll, "SetUserPosition"):
                        self.dll.SetUserPosition.argtypes = [ctypes.POINTER(InitPositionStruct)]
                        self.dll.SetUserPosition.restype = None
                    break
                except Exception as e:
                    print(f"[BridgeWriter Init Note]: {e}", flush=True)

    def set_throttle(self, percent: float) -> bool:
        if self.dll and hasattr(self.dll, "SetThrottle"):
            try:
                self.dll.SetThrottle(ctypes.c_double(max(0.0, min(100.0, float(percent)))))
                return True
            except Exception as e:
                print(f"[BridgeWriter SetThrottle Error]: {e}", flush=True)
        return False

    def pause_sim(self) -> bool:
        if self.dll and hasattr(self.dll, "PauseSim"):
            try:
                self.dll.PauseSim()
                return True
            except Exception:
                pass
        return False

    def unpause_sim(self) -> bool:
        if self.dll and hasattr(self.dll, "UnpauseSim"):
            try:
                self.dll.UnpauseSim()
                return True
            except Exception:
                pass
        return False

    def set_position(self, alt_ft: float, ias_kt: float, on_ground: bool = False, heading_deg: float = 74.0, lat: float = 32.235, lon: float = 74.135) -> bool:
        if self.dll and hasattr(self.dll, "SetUserPosition"):
            try:
                pos = InitPositionStruct(
                    Latitude=float(lat),
                    Longitude=float(lon),
                    Altitude=float(alt_ft),
                    Pitch=0.0,
                    Bank=0.0,
                    Heading=float(heading_deg),
                    OnGround=1 if on_ground else 0,
                    Airspeed=0 if on_ground else int(round(float(ias_kt))),
                )
                self.dll.SetUserPosition(ctypes.byref(pos))
                return True
            except Exception as e:
                print(f"[BridgeWriter SetUserPosition Error]: {e}", flush=True)
        return False


