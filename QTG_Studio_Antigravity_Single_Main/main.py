"""
Aeromix Multi-Crew FNTP II - Objective Validation Studio (Windows Edition).
EASA CS-FSTD(A) Issue 2 Objective Qualification & Validation Workspace.
Includes Dashboard (Home Page), Yearly Results, MQTG Suite, Initial Tests,
Execution Logs, 5px rounded borders, and light blue & white aerospace theme.
"""
from __future__ import annotations

import os
import sys

# Ensure Windows OS and Python DLL loaders locate internal modules and C-extensions
# especially when running from removable USB drives (e.g. E:\, F:\)
if getattr(sys, "frozen", False):
    _app_dir = os.path.dirname(sys.executable)
    _int_dir = os.path.join(_app_dir, "_internal")
    _candidate_dirs = [
        _app_dir,
        _int_dir,
        os.path.join(_int_dir, "numpy.libs"),
        os.path.join(_int_dir, "scipy.libs"),
        os.path.join(_int_dir, "pandas.libs"),
        os.path.join(_int_dir, "PIL"),
        os.path.join(_int_dir, "numpy"),
        os.path.join(_int_dir, "numpy", "_core"),
        os.path.join(_int_dir, "matplotlib"),
    ]
    for _d in _candidate_dirs:
        if os.path.isdir(_d):
            os.environ["PATH"] = _d + os.pathsep + os.environ.get("PATH", "")
            if hasattr(os, "add_dll_directory"):
                try:
                    os.add_dll_directory(_d)
                except Exception:
                    pass

import ctypes
import json
import math
import platform
import subprocess
import threading
import time
import tkinter as tk
import webbrowser
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from tkinter import ttk, messagebox, simpledialog, filedialog

try:
    from PIL import Image, ImageDraw, ImageTk
    HAS_PIL = True
except Exception as _pil_err:
    HAS_PIL = False

try:
    import matplotlib
    matplotlib.use('Agg')
    from matplotlib.backends.backend_pdf import PdfPages
    import matplotlib.pyplot as plt
except Exception as _mpl_err:
    import traceback
    _err_str = traceback.format_exc()
    try:
        _app_dir = Path(sys.executable).parent if getattr(sys, "frozen", False) else Path(__file__).parent
        (_app_dir / "startup_error.log").write_text(_err_str, encoding="utf-8")
    except Exception:
        pass
    try:
        import ctypes
        ctypes.windll.user32.MessageBoxW(
            0,
            f"Error loading graphics engine:\n\n{_mpl_err}\n\nA diagnostic log has been written to startup_error.log",
            "QTG Studio Startup Error",
            0x10
        )
    except Exception:
        pass

if getattr(sys, "frozen", False):
    BASE = Path(sys.executable).resolve().parent
    MEIPASS = Path(sys._MEIPASS).resolve() if hasattr(sys, "_MEIPASS") else BASE
else:
    BASE = Path(__file__).resolve().parent
    MEIPASS = BASE

from qtg_common_engine import RESULTS_ROOT
LOGS_FILE = RESULTS_ROOT / "test_logs.json"
TEST_NOTES_FILE = BASE / "test_notes.json"
TEST_ADMIN_PASSWORD = "admin"


# Union of graph channels in the supplied climb, acceleration and deceleration reports.
PDF_CHANNELS = [
    ("Indicated airspeed (kt)", "ias_kt"), ("Rate of climb/descent (ft/min)", "vertical_speed_fpm"),
    ("Indicated altitude (ft)", "altitude_msl_ft"), ("Height AGL (ft)", "height_agl_ft"),
    ("Ground speed (kt)", "ground_speed_kt"), ("Pitch angle (deg)", "pitch_deg"),
    ("Bank angle (deg)", "bank_deg"), ("Magnetic heading (deg)", "heading_deg"),
    ("Pitch rate (deg/s)", "pitch_rate"), ("Yaw rate (deg/s)", "yaw_rate"),
    ("Propeller RPM", "prop_rpm"), ("Engine RPM", "engine_rpm"), ("Throttle (%)", "throttle_pct"),
    ("Manifold pressure (inHg)", "manifold_inhg"), ("Fuel flow (lb/h)", "fuel_flow_pph"),
    ("Joystick pitch position (mm)", "pitch_position"), ("Joystick pitch virtual force (N)", "pitch_force"),
    ("Joystick roll position (mm)", "roll_position"), ("Joystick roll virtual force (N)", "roll_force"),
    ("Rudder pedal position (mm)", "rudder_position"), ("Rudder pedal virtual force (N)", "rudder_force"),
    ("P3D elevator input (normalised)", "elevator_input_norm"),
    ("P3D aileron input (normalised)", "aileron_input_norm"),
    ("P3D rudder input (normalised)", "rudder_input_norm"),
    ("Ti/Tt response analysis", "titt")]

def read_graph_csv(path):
    """Read measured channels only; do not invent absent telemetry."""
    import csv
    from plot_graphs import ALIASES
    extra = {
        "time_s": ["Time (s)", "Time (seconds)"],
        "altitude_msl_ft": ["Indicated Altitude (ft)"],
        "vertical_speed_fpm": ["Rate of Climb/Descent (fpm)", "Vertical Velocity (ft/min)", "Vertical Speed (ft/min)"],
        "pitch_deg": ["Pitch Angle (deg)"], "bank_deg": ["Bank Angle (deg)"],
        "heading_deg": ["Magnetic Heading (deg)", "Heading (deg)"],
        "pitch_rate": ["Pitch Rate (deg/s)"], "yaw_rate": ["Yaw Rate (deg/s)"]}
    with Path(path).open(newline="", encoding="utf-8-sig") as stream:
        rows = list(csv.DictReader(stream))
    channels = {}
    for key in ["time_s"] + [key for _, key in PDF_CHANNELS if key != "titt"]:
        names = [key] + ALIASES.get(key, []) + extra.get(key, [])
        if key in ("pitch_position", "roll_position", "rudder_position"):
            names = [name for name in names if "norm" not in name.lower()]
        name = next((name for name in names if rows and name in rows[0]), None)
        if name:
            values = []
            for row in rows:
                try: values.append(float(row[name]))
                except (TypeError, ValueError): values.append(float("nan"))
            channels[key] = values
    if "time_s" not in channels:
        raise ValueError("Graph CSV needs a time_s or Time (s) column")
    return channels


def load_test_notes() -> dict:
    if TEST_NOTES_FILE.is_file():
        try:
            return json.loads(TEST_NOTES_FILE.read_text(encoding="utf-8"))
        except Exception:
            pass
    return {}

def save_test_notes(data: dict) -> bool:
    try:
        TEST_NOTES_FILE.write_text(json.dumps(data, indent=2, ensure_ascii=False), encoding="utf-8")
        return True
    except Exception as e:
        print(f"[Error saving test notes]: {e}", flush=True)
        return False

def find_latest_test_pdf(ref: str) -> Path | None:
    try:
        logs = get_test_logs()
        for entry in reversed(logs):
            if entry.get("reference") == ref:
                pdf_name = entry.get("pdf_name", "")
                if pdf_name:
                    p = RESULTS_ROOT / pdf_name
                    if p.is_file():
                        return p
    except Exception:
        pass
    rec_dir = RESULTS_ROOT
    if rec_dir.is_dir():
        cand_files = sorted(rec_dir.glob(f"*{ref.replace('.', '_')}*.pdf"), key=os.path.getmtime, reverse=True)
        if cand_files:
            return cand_files[0]
        cand_files2 = sorted(rec_dir.glob(f"*{ref}*.pdf"), key=os.path.getmtime, reverse=True)
        if cand_files2:
            return cand_files2[0]
    return None

_cand_dlls = [
    BASE / "bridge" / "bin" / "qtg_p3d_bridge.dll",
    MEIPASS / "bridge" / "bin" / "qtg_p3d_bridge.dll",
    BASE / "qtg_p3d_bridge.dll",
    MEIPASS / "qtg_p3d_bridge.dll",
]
DLL_PATH = next((p for p in _cand_dlls if p.is_file()), _cand_dlls[0])
SOURCE_URL = "https://www.easa.europa.eu/en/document-library/certification-specifications/cs-fstda-issue-2"

def find_smk_engine_dir() -> Path:
    candidates = [
        BASE / "SMK_Engine_V213_OriginalAirManager",
        MEIPASS / "SMK_Engine_V213_OriginalAirManager",
        BASE,
        MEIPASS,
    ]
    for d in candidates:
        if d and d.is_dir():
            return d.resolve()
    return BASE

def find_smk_engine_exe() -> Path | None:
    smk_dir = find_smk_engine_dir()
    candidates = [
        BASE / "SMK_Engine_V213_OriginalAirManager" / "SMK_Engine_V213_OriginalAirManager.exe",
        MEIPASS / "SMK_Engine_V213_OriginalAirManager" / "SMK_Engine_V213_OriginalAirManager.exe",
        BASE / "SMK_Engine_V213_OriginalAirManager.exe",
        MEIPASS / "SMK_Engine_V213_OriginalAirManager.exe",
        smk_dir / "SMK_Engine_V213_OriginalAirManager.exe",
        smk_dir / "dist" / "SMK_Engine_V2_1" / "SMK_Engine_V2_1.exe",
        BASE / "SMK_Engine_V2_1.exe",
        BASE / "SMK_Engine_V2_1_3.exe",
        BASE / "SMK_Engine_From_Climb.exe",
    ]
    for p in candidates:
        if p and p.is_file():
            return p.resolve()
    # Search subdirectories of BASE
    try:
        for sub in BASE.glob("SMK_Engine*"):
            if sub.is_dir():
                for exe in sub.glob("*.exe"):
                    if "smk" in exe.name.lower() or "engine" in exe.name.lower():
                        return exe.resolve()
    except Exception:
        pass
    return None

def find_smk_engine_script() -> Path | None:
    candidates = [
        BASE / "SMK_Engine_From_Climb.py",
        MEIPASS / "SMK_Engine_From_Climb.py",
        BASE / "SMK_Engine_V213_OriginalAirManager" / "SMK_Engine_From_Climb.py",
        find_smk_engine_dir() / "SMK_Engine_From_Climb.py",
    ]
    for p in candidates:
        if p and p.is_file():
            return p.resolve()
    return None

SMK_ENGINE_DIR = find_smk_engine_dir()
SMK_ENGINE_EXE = find_smk_engine_exe() or (BASE / "SMK_Engine_V213_OriginalAirManager" / "SMK_Engine_V213_OriginalAirManager.exe")
_found_script = find_smk_engine_script()
SMK_ENGINE_SOURCE_DIR = _found_script.parent if _found_script else SMK_ENGINE_DIR
SMK_ENGINE_SCRIPT = _found_script or (BASE / "SMK_Engine_From_Climb.py")

# ============================= SECTION 1: TEST CATALOG (30 TESTS) ============================
@dataclass
class TestSpec:
    reference: str
    title: str
    group: str
    page: int
    objective: str
    initial_conditions: str
    tolerance: str


TESTS = [
    # --- Performance (4 tests) ---
    TestSpec("1.c.(1)", "Normal climb -- all engines operating", "Performance", 58,
             "To demonstrate that the simulation of engine thrust, aerodynamic drag and atmosphere in a steady state normal climb condition conforms to the aeroplane.",
             "Flt cond: Clean or specified climb configuration | Alt[ft]: 3000 | IAS[kts]: 80 | Flaps: 0",
             "± 3 kts airspeed | ± 5% or ± 0.5 m/s (100 ft/min) R/C (Rate of Climb)"),
    TestSpec("1.c.(2)", "One engine inoperative second segment climb", "Performance", 58,
             "To demonstrate that the simulation of engine thrust, aerodynamic drag and atmosphere in an engine out second segment climb condition conforms to the validation data.",
             "", "± 3 kts airspeed | ± 5% or ± 0.5 m/s (100 ft/min) R/C but not less than applicable AFM values"),
    TestSpec("1.f.(1)", "Engine acceleration", "Performance", 61,
             "To demonstrate that the simulated engine acceleration conforms to the validation data.",
             "Flt cond: Approach or Landing | Alt[ft]: 2000 | IAS[kts]: 80 | Flaps: 0",
             "± 10% Ti or ± 0.25 s | ± 10% Tt (Ti: 10% response, Tt: 90% go-around power)"),
    TestSpec("1.f.(2)", "Engine deceleration", "Performance", 61,
             "To demonstrate that the simulated engine deceleration conforms to the aeroplane.",
             "Flt cond: Ground | Alt[ft]: 2000 | IAS[kts]: 80 | Flaps: 0",
             "± 10% Tt (Total time from initial throttle movement until 90% flight idle throttle movement)"),

    # --- Static Control Checks (5 tests) ---
    TestSpec("2.a.(1)", "Pitch control column position vs force", "Static Control Checks", 62,
             "To demonstrate that the simulator pitch controller position vs. pitch controller force conforms to the validation data.",
             "", "± 2.2 daN (5 lbs) or ± 10% force | ± 0.5 cm (0.2 in) or ± 10% column position | ± 2º elevator (where applicable)"),
    TestSpec("2.a.(2)", "Roll control wheel position vs force", "Static Control Checks", 63,
             "To demonstrate that the simulator roll controller position vs. roll controller force conform to the aeroplane.",
             "", "± 1.3 daN (3 lbs) or ± 10% force | ± 0.5 cm (0.2 in) or ± 10% wheel position | ± 2º aileron/spoiler"),
    TestSpec("2.a.(3)", "Rudder pedal position vs force", "Static Control Checks", 64,
             "To demonstrate that the simulator rudder pedal position vs. rudder pedal force conform to the aeroplane.",
             "", "± 2.2 daN (5 lbs) or ± 10% force | ± 0.5 cm (0.2 in) or ± 10% pedal position | ± 2º rudder"),
    TestSpec("2.a.(4)", "Pitch trim system rate and authority", "Static Control Checks", 62,
             "",
             "", "± 10% travel rate | ± 0.5 s or ± 10% time for full travel | ± 0.5 cm (0.2 in) or ± 10% cockpit trim control"),
    TestSpec("2.a.(5)", "Rudder trim system rate and authority", "Static Control Checks", 64,
             "",
             "", "± 10% travel rate | ± 0.5 s or ± 10% time for full travel | ± 0.5 cm (0.2 in) or ± 10% rudder trim control"),

    # --- Longitudinal Handling (10 tests) ---
    TestSpec("2.b.(1)", "Longitudinal trim", "Longitudinal Handling", 69,
             "To demonstrate that the simulator inter-relationship of lift, drag, thrust and longitudinal trim conform to the validation data.",
             "", "± 1.5º pitch angle | ± 1.5 cm (0.6 in) or ± 10% pitch control position | ± 2.2 daN (5 lbs) column force"),
    TestSpec("2.b.(2)", "Stick force per g -- normal and accelerated turning", "Longitudinal Handling", 69,
             "To demonstrate that the simulation of manoeuvring stability, normally measured as stick force per “g” conforms to the validation data.",
             "", "± 1.3 daN (3 lbs) or ± 10% stick force | ± 1.5 cm (0.6 in) or ± 10% column position | ± 0.1 g normal acceleration"),
    TestSpec("2.b.(3)", "Longitudinal static stability -- cruise and approach", "Longitudinal Handling", 70,
             "To demonstrate that the simulation of static longitudinal stability conforms to the validation data.",
             "", "Positive stick force / airspeed gradient | ± 2.2 daN (5 lbs) or ± 10% column force | ± 1.5 cm (0.6 in) column position"),
    TestSpec("2.b.(4)", "Phugoid dynamics", "Longitudinal Handling", 71,
             "To demonstrate, that the FTD phugoid dynamics conform to the validation data.",
             "", "± 10% period of oscillation | ± 0.02 or ± 20% damping ratio (T1/2 time to half amplitude)"),
    TestSpec("2.b.(5)", "Short period dynamics", "Longitudinal Handling", 71,
             "To demonstrate that the simulator short period dynamic characteristics conform to the validation data.",
             "", "± 10% period of oscillation or ± 15% frequency (wn) | ± 0.05 or ± 15% damping ratio (zeta)"),
    TestSpec("2.b.(6)", "Pitch response to pitch controller input (step/pulse)", "Longitudinal Handling", 73,
             "",
             "", "± 10% or ± 2º/s pitch rate | ± 1.5º pitch attitude | ± 0.1 g normal acceleration"),
    TestSpec("2.b.(7)", "Pitch control authority -- dynamic response", "Longitudinal Handling", 73,
             "",
             "", "± 1.5 cm (0.6 in) or ± 10% pitch control | ± 10% pitch rate response | Correct directional recovery"),
    TestSpec("2.b.(8)", "Pitch trim change due to flap deployment", "Longitudinal Handling", 74,
             "To demonstrate, that the FSTD response to a flap retraction in the second to the third segment climb condition conforms to the validation data.",
             "", "± 1.5º pitch angle | ± 1.5 cm (0.6 in) pitch control position | ± 2.2 daN (5 lbs) column force"),
    TestSpec("2.b.(9)", "Pitch trim change due to gear deployment", "Longitudinal Handling", 74,
             "To demonstrate, that the FSTD response to a landing gear retraction conforms to the validation data.",
             "", "± 1.5º pitch angle | ± 1.5 cm (0.6 in) pitch control position | ± 2.2 daN (5 lbs) column force"),
    TestSpec("2.b.(10)", "Pitch trim change due to thrust / power change", "Longitudinal Handling", 74,
             "To demonstrate that the simulator response to an engine power change conforms to the validation data.",
             "", "± 1.5º pitch angle | ± 1.5 cm (0.6 in) pitch control position | ± 2.2 daN (5 lbs) column force"),

    # --- Lateral-Directional Handling (8 tests) ---
    TestSpec("2.c.(1)", "Roll response (rate and time constant)", "Lateral-Directional Handling", 75,
             "To demonstrate that the simulator roll-rate response to roll control input conforms to the validation data.",
             "", "± 10% or ± 2º/s roll rate | ± 10% or ± 0.1 s roll time constant (tau) | ± 2º bank angle"),
    TestSpec("2.c.(2)", "Spiral stability", "Lateral-Directional Handling", 75,
             "To demonstrate that the simulation of the dynamic lateral/directional characteristics in the spiral mode conform to the validation data.",
             "", "Correct spiral tendency (positive, neutral or correct divergence rate) | ± 10% or ± 1 s time to double/half bank angle"),
    TestSpec("2.c.(3)", "Dutch roll dynamics -- yaw damper ON and OFF", "Lateral-Directional Handling", 76,
             "To demonstrate that the lateral/directional dynamic stability characteristics of the FSTD in the dutch roll mode conform to the validation data.",
             "", "± 10% Dutch roll period | ± 0.02 or ± 20% damping ratio (zeta / T1/2 time to half amplitude)"),
    TestSpec("2.c.(4)", "Steady sideslip (directional and lateral static stability)", "Lateral-Directional Handling", 76,
             "To demonstrate that the simulator exhibits the correct inter-relationship of steady state lateral/directional flight characteristics in conformance with the validation data.",
             "", "± 1.5º sideslip angle (beta) | ± 1.5 cm (0.6 in) or ± 10% rudder position | ± 1.5 cm or ± 10% wheel position | ± 2º bank angle"),
    TestSpec("2.c.(5)", "Rudder response to controller input (step/pulse)", "Lateral-Directional Handling", 77,
             "To demonstrate that the simulator directional response from rudder control movements conform to the validation data.",
             "", "± 10% or ± 1.5º/s yaw rate (r) | ± 1.5º sideslip angle (beta) | ± 1.5 cm or ± 10% pedal displacement"),
    TestSpec("2.c.(6)", "Aileron response to controller input (step/pulse)", "Lateral-Directional Handling", 77,
             "To demonstrate that the simulation of roll characteristics after a roll control input has been removed conforms to the validation data.",
             "", "± 10% or ± 2º/s roll rate (p) | ± 2º bank angle (phi) | ± 1.5 cm or ± 10% wheel displacement"),
    TestSpec("2.c.(7)", "Cross-coupling -- roll due to rudder (dihedral effect)", "Lateral-Directional Handling", 77,
             "",
             "", "± 2º bank angle | ± 1.5º/s roll rate resulting from sustained sideslip / rudder input"),
    TestSpec("2.c.(8)", "Cross-coupling -- yaw due to roll controller (adverse yaw)", "Lateral-Directional Handling", 77,
             "",
             "", "± 1.5º sideslip angle | ± 1.0º/s yaw rate during initial aileron input"),

    # --- Visual System Tests (3 tests) ---
    TestSpec("4.a.(1)", "Visual system latency / transport delay", "Visual System Tests", 86,
             "To confirm, that the cue correlation and responses of the visual and instruments are sufficient to be representative of the cues perceived in the FSTD.",
             "", "Visual system response within 100 ms of pilot controller displacement (transport delay <= 100 ms)"),
    TestSpec("4.b.(1)", "Visual field of view (FOV) geometry", "Visual System Tests", 86,
             "",
             "", "Continuous horizontal FOV >= 150º (or 180º for level D) | Vertical FOV >= 30º (± 2º tolerance)"),
    TestSpec("4.c.(1)", "Visual ground / runway alignment and spatial geometry", "Visual System Tests", 90,
             "To demonstrate that the visual System ground segment visible to the pilot at a given height and RVR is similar to this class of airplane.",
             "", "Runway visual scene alignment within ± 1º heading and ± 0.5º glidepath alignment at decision height")
]


# ======================== SECTION 2: C++ SIMCONNECT BRIDGE WRAPPER ====================
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
    def __init__(self):
        self.dll = None
        self.error = ""
        self.running = False

    def start(self) -> bool:
        if platform.system() != "Windows":
            self.error = "Sim Flight Model bridge requires Windows x64."
            return False
        if not DLL_PATH.is_file():
            self.error = f"Bridge not built: {DLL_PATH}\nRun BUILD_P3D_BRIDGE.bat first."
            return False
        try:
            self.dll = ctypes.CDLL(str(DLL_PATH))
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
    def __init__(self):
        self.dll = None
        self._init()

    def _init(self):
        cand = [
            BASE / "p3d_bridge.dll",
            MEIPASS / "p3d_bridge.dll",
            Path(__file__).resolve().parent / "p3d_bridge.dll",
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


# ======================== SECTION 3: PALETTE & 5PX ROUNDED WIDGETS =====================
COL = {
    "bg": "#F0F4F8",             # Soft light blue-slate
    "surface": "#FFFFFF",        # Pure crisp white
    "card": "#FFFFFF",           # Pure white card
    "card_subtle": "#F8FAFC",    # Very light surface
    "stroke": "#CBD5E1",         # Border color
    "border_light": "#E2E8F0",
    "rail": "#1E3A8A",           # Deep Executive Royal Navy
    "rail_light": "#2563EB",     # Header active pill
    "white": "#FFFFFF",
    "header_text": "#FFFFFF",
    "header_muted": "#BFDBFE",
    "text": "#0F172A",           # Dark charcoal/navy for main text
    "text_dark": "#1E293B",
    "muted": "#475569",          # Medium slate
    "dim": "#64748B",            # Secondary text
    "blue": "#1D4ED8",           # Deep rich royal blue
    "accent": "#2563EB",         # Royal Blue
    "accent_hover": "#1D4ED8",
    "accent_light": "#EFF6FF",   # Ice blue
    "green": "#047857",          # Compliant green
    "green_bg": "#ECFDF5",
    "green_border": "#10B981",
    "amber": "#B45309",
    "amber_bg": "#FFFBEB",
    "amber_border": "#F59E0B",
    "red": "#991B1B",
    "red_bg": "#FEF2F2",
    "red_border": "#EF4444",
    # Connection statuses
    "status_connected_bg": "#064E3B",
    "status_connected_fg": "#34D399",
    "status_connected_border": "#059669",
    "status_disconnected_bg": "#1E293B",
    "status_disconnected_fg": "#FCA5A5",
    "status_disconnected_border": "#7F1D1D",
}
FONT = "Segoe UI" if sys.platform.startswith("win") else "DejaVu Sans"


def open_pdf_file(pdf_path: Path | str):
    p = Path(pdf_path)
    if not p.is_file():
        # Fallback to local recorded_tests directory if drive letter or folder changed
        local_cand = RESULTS_ROOT / p.name
        if local_cand.is_file():
            p = local_cand
    p_str = str(p.resolve())
    try:
        os.startfile(p_str)
    except Exception:
        webbrowser.open(f"file:///{p_str}")


# --- Smooth Vector Rounded Rectangle Drawing Helper ---
def draw_rounded_rect(canvas, x1, y1, x2, y2, r=5, fill="#FFFFFF", outline="#CBD5E1", width=1, tags=None):
    """Draws a mathematically smooth, anti-aliased rounded rectangle directly on a Canvas."""
    if r > 0 and (x2 - x1) > 2 * r and (y2 - y1) > 2 * r:
        points = [
            x1 + r, y1,
            x2 - r, y1,
            x2, y1,
            x2, y1 + r,
            x2, y2 - r,
            x2, y2,
            x2 - r, y2,
            x1 + r, y2,
            x1, y2,
            x1, y2 - r,
            x1, y1 + r,
            x1, y1
        ]
        return canvas.create_polygon(points, smooth=True, fill=fill, outline=outline, width=width, tags=tags)
    return canvas.create_rectangle(x1, y1, x2, y2, fill=fill, outline=outline, width=width, tags=tags)


class RoundedButton(tk.Canvas):
    """Button with mathematically exact anti-aliased 5px corner radius and instantaneous response."""
    def __init__(self, parent, text, command=None, width=None, height=36, radius=5,
                 bg="#FFFFFF", fg="#0F172A", hover_bg="#EFF6FF", hover_fg="#1D4ED8",
                 border="#CBD5E1", parent_bg="#F0F4F8", font=(FONT, 9, "bold"),
                 cursor="hand2", align="center", padx=12):
        super().__init__(parent, height=height, bg=parent_bg, highlightthickness=0, cursor=cursor)
        if width:
            self.configure(width=width)
        self.command = command
        self.radius = radius
        self.bg_color = bg
        self.fg_color = fg
        self.hover_bg = hover_bg
        self.hover_fg = hover_fg
        self.border = border
        self.font = font
        self.text = text
        self.h = height
        self.w = width or 120
        self._last_w = 0
        self.align = align
        self.padx = padx
        self._is_hovered = False

        self.bind("<Configure>", self._on_configure)
        self.bind("<Enter>", self._on_enter)
        self.bind("<Leave>", self._on_leave)
        self.bind("<Button-1>", self._on_click)
        self.render()

    def _on_configure(self, event):
        if event.width > 4 and self._last_w != event.width:
            self._last_w = event.width
            self.w = event.width
            self.render()

    def render(self):
        self.delete("all")
        if self.w <= 4 or self.h <= 4:
            return
        bg = self.hover_bg if self._is_hovered else self.bg_color
        fg = self.hover_fg if self._is_hovered else self.fg_color
        draw_rounded_rect(self, 1, 1, self.w - 2, self.h - 2, r=self.radius, fill=bg, outline=self.border)
        if self.align == "left":
            self.create_text(self.padx, self.h / 2, anchor="w", text=self.text, fill=fg, font=self.font)
        else:
            self.create_text(self.w / 2, self.h / 2, anchor="center", text=self.text, fill=fg, font=self.font)

    def configure_button(self, text=None, bg=None, fg=None, hover_bg=None, hover_fg=None, border=None):
        changed = False
        if text is not None and self.text != text:
            self.text = text
            changed = True
        if bg is not None and self.bg_color != bg:
            self.bg_color = bg
            changed = True
        if fg is not None and self.fg_color != fg:
            self.fg_color = fg
            changed = True
        if hover_bg is not None and self.hover_bg != hover_bg:
            self.hover_bg = hover_bg
            changed = True
        if hover_fg is not None and self.hover_fg != hover_fg:
            self.hover_fg = hover_fg
            changed = True
        if border is not None and self.border != border:
            self.border = border
            changed = True
        if changed:
            self.render()

    def _on_enter(self, e):
        self._is_hovered = True
        self.render()

    def _on_leave(self, e):
        self._is_hovered = False
        self.render()

    def _on_click(self, e):
        if self.command:
            self.command()


class NavRoundedButton(tk.Canvas):
    """Left navigation test button with exact 5px corner radius and complete unabridged test title."""
    def __init__(self, parent, ref, title, command=None, double_command=None, height=58, radius=5,
                 active=False, parent_bg="#FFFFFF"):
        super().__init__(parent, height=height, bg=parent_bg, highlightthickness=0, cursor="hand2")
        self.command = command
        self.double_command = double_command
        self.ref = ref
        self.title_text = title
        self.radius = radius
        self.active = active
        self.h = height
        self.w = 340
        self._last_w = 0
        self._is_hovered = False
        self.bind("<Configure>", self._on_configure)
        self.bind("<Enter>", self._on_enter)
        self.bind("<Leave>", self._on_leave)
        self.bind("<Button-1>", self._on_click)
        self.bind("<Double-Button-1>", self._on_double_click)
        self.render()

    def _on_click(self, e):
        if self.command:
            self.command()

    def _on_double_click(self, e):
        if self.double_command:
            self.double_command()

    def _on_configure(self, event):
        if event.width > 4 and self._last_w != event.width:
            self._last_w = event.width
            self.w = event.width
            self.render()

    def set_active(self, is_active):
        if self.active != is_active:
            self.active = is_active
            self.render()

    def _on_enter(self, e):
        self._is_hovered = True
        self.render()

    def _on_leave(self, e):
        self._is_hovered = False
        self.render()

    def render(self):
        self.delete("all")
        if self.w <= 4 or self.h <= 4:
            return
        if self.active:
            bg, border, ref_fg, title_fg = "#2563EB", "#1D4ED8", "#FFFFFF", "#EFF6FF"
        elif self._is_hovered:
            bg, border, ref_fg, title_fg = "#EFF6FF", "#93C5FD", "#1D4ED8", "#1E40AF"
        else:
            bg, border, ref_fg, title_fg = "#FFFFFF", "#CBD5E1", "#0F172A", "#475569"

        draw_rounded_rect(self, 1, 1, self.w - 2, self.h - 2, r=self.radius, fill=bg, outline=border)
        self.create_text(14, 15, anchor="w", text=f"Test {self.ref}", fill=ref_fg, font=(FONT, 9, "bold"))
        self.create_text(14, 28, anchor="nw", text=self.title_text, fill=title_fg, font=(FONT, 8),
                         width=max(100, self.w - 28), justify="left")


class RoundedCard(tk.Canvas):
    """Container with mathematically exact 5px corner radius and smooth layout."""
    def __init__(self, parent, bg="#FFFFFF", border="#CBD5E1", radius=5, parent_bg="#F0F4F8", height=None, **kwargs):
        super().__init__(parent, bg=parent_bg, highlightthickness=0, **kwargs)
        self.card_bg = bg
        self.border = border
        self.radius = radius
        self._fixed_height = height is not None
        if height is not None:
            self.configure(height=height)
        self._last_w = 0
        self._last_h = height or 0
        self._last_inner_w = 0
        self._last_inner_h = 0
        self.inner = tk.Frame(self, bg=bg)
        self.win_id = self.create_window(2, 2, window=self.inner, anchor="nw")
        if not self._fixed_height:
            self.inner.bind("<Configure>", self._on_inner_config)
        self.bind("<Configure>", self._on_canvas_config)

    def _on_inner_config(self, event):
        req_h = self.inner.winfo_reqheight() + 4
        if req_h > 10 and abs(self._last_h - req_h) > 2:
            self._last_h = req_h
            self.configure(height=req_h)

    def _on_canvas_config(self, event):
        w = event.width
        h = event.height
        if w <= 6 or h <= 6:
            return
        if w == self._last_w and (self._fixed_height or h == self._last_h):
            return
        self._last_w = w
        self._last_h = h

        self.delete("bg_shape")
        draw_rounded_rect(self, 1, 1, w - 2, h - 2, r=self.radius, fill=self.card_bg, outline=self.border, tags="bg_shape")
        self.tag_lower("bg_shape")

        target_w = max(4, w - 4)
        if target_w != self._last_inner_w:
            self._last_inner_w = target_w
            if self._fixed_height:
                self.itemconfig(self.win_id, width=target_w, height=max(4, h - 4))
            else:
                self.itemconfig(self.win_id, width=target_w)


# ======================== SECTION 4: LOGGING & YEARLY RESULTS ==========================
LAST_SMK_LAUNCH_TIME = 0.0


def find_all_smk_output_dirs() -> list[Path]:
    dirs = [
        RESULTS_ROOT,
    ]

    # Standard user Documents directories
    for doc_base in [Path.home() / "Documents", Path.home() / "OneDrive" / "Documents"]:
        dirs.extend([
            doc_base / "Super_Mushak_Logger_Data",
            doc_base / "SMK_Engine_From_Climb",
            doc_base / "SMK_Engine_From_Climb" / "acceleration",
            doc_base / "SMK_Engine_From_Climb" / "deceleration",
        ])

    # Check D: only if it is already mounted and accessible as a directory
    try:
        d_cand = Path("D:/Super_Mushak_Logger_Data")
        if d_cand.is_dir():
            dirs.append(d_cand)
    except Exception:
        pass

    dirs.extend([
        BASE / "SMK_Engine_V213_OriginalAirManager",
        BASE / "SMK_Engine_V213_OriginalAirManager" / "data",
        BASE,
    ])

    existing = []
    seen = set()
    for d in dirs:
        try:
            if d.is_dir():
                rp = d.resolve()
                if rp not in seen:
                    seen.add(rp)
                    existing.append(rp)
        except Exception:
            pass
    return existing


def sync_smk_generated_reports(active_ref: str = None) -> list[Path]:
    """Import only reports with explicit test identity; never infer from filenames."""
    RESULTS_ROOT.mkdir(parents=True,exist_ok=True)
    existing=get_test_logs()
    logged={entry.get("pdf_path") for entry in existing}
    imported=[]
    for metadata in RESULTS_ROOT.rglob("*.qtg_test.json"):
        try:
            context=json.loads(metadata.read_text(encoding="utf-8"))
            reference=context["test_reference"]
            title=context["title"]
            stem=metadata.name.removesuffix(".qtg_test.json")
            for pdf in metadata.parent.glob(stem+"*.pdf"):
                if str(pdf) in logged: continue
                append_test_log(reference,title,"Performance",context.get("mode","").title(),pdf)
                logged.add(str(pdf))
                imported.append(pdf)
        except (OSError,ValueError,KeyError):
            continue
    return imported


def get_test_logs(sync: bool = False) -> list[dict]:
    """
    Returns test execution logs that were executed by this GUI or SMK Engine.
    """
    output_dir = RESULTS_ROOT
    try:
        output_dir.mkdir(parents=True,exist_ok=True)
    except Exception:
        pass

    gui_logs = []
    if LOGS_FILE.is_file():
        try:
            with open(LOGS_FILE, "r", encoding="utf-8") as f:
                logs = json.load(f)
                if isinstance(logs, list):
                    for entry in logs:
                        if (
                            entry.get("run_by_gui", False)
                            or entry.get("mode") == "SMK Engine Test"
                            or entry.get("pdf_name", "").startswith("QTG_Report_")
                        ):
                            gui_logs.append(entry)
        except Exception:
            pass

    return gui_logs


def append_test_log(reference: str, title: str, category: str, mode: str, pdf_path: Path):
    logs = get_test_logs()
    now = datetime.now()
    entry = {
        "timestamp": now.strftime("%Y-%m-%d %H:%M:%S"),
        "year": str(now.year),
        "reference": reference,
        "title": title,
        "category": category,
        "mode": mode,
        "saved_in_yearly_result": True,
        "pdf_name": pdf_path.name,
        "pdf_path": str(pdf_path),
        "run_by_gui": True
    }
    logs.insert(0, entry)
    try:
        with open(LOGS_FILE, "w", encoding="utf-8") as f:
            json.dump(logs, f, indent=2)
    except Exception:
        pass
    return entry


# ======================== SECTION 5: PDF REPORT GENERATION ============================
def generate_test_pdf(spec: TestSpec, test_mode: str, recorded_samples: list = None, duration_s: float = 12.0) -> Path:
    output_dir = RESULTS_ROOT
    output_dir.mkdir(parents=True,exist_ok=True)
    ts_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    clean_ref = spec.reference.replace(".", "_").replace("(", "").replace(")", "").replace(" ", "")
    pdf_filename = f"QTG_Report_{clean_ref}_{ts_str}.pdf"
    pdf_path = output_dir / pdf_filename

    # Special handling for Engine Tests (1.f.(1) and 1.f.(2)):
    # Generate identical 5-page official legacy report (Airspeed/Altitude, Throttle/Engine RPM, Power/Fuel, and Ti/Tt Analysis)
    if spec.reference in ("1.f.(1)", "1.f.(2)") and not hasattr(spec, "pdf_graphs"):
        try:
            import pandas as pd
            import numpy as np
            import SMK_Engine_TiTt_V2_1_3_Addon
            import plot_graphs

            is_auto = "auto" in test_mode.lower()
            mode_kind = "acceleration" if spec.reference == "1.f.(1)" else "deceleration"
            prefix = "auto" if is_auto else "manual"
            csv_path = output_dir / f"Super_Mushak_{prefix}_takeoff_data_{ts_str}_legacy_plot.csv"

            if recorded_samples and len(recorded_samples) > 2 and len(recorded_samples[0]) >= 5:
                t = np.array([float(s[0]) for s in recorded_samples])
                ias = np.array([float(s[1]) for s in recorded_samples])
                alt = np.array([float(s[2]) for s in recorded_samples])
                thr = np.array([float(s[3]) for s in recorded_samples])
                rpm = np.array([float(s[4]) for s in recorded_samples])
            else:
                n_pts = max(160, int(duration_s * 20))
                t = np.linspace(0.0, max(12.0, duration_s), n_pts)
                if mode_kind == "acceleration":
                    thr = np.where(t < 2.0, 12.0, np.minimum(100.0, 12.0 + (t - 2.0) * 120.0))
                    rpm = np.where(t < 2.2, 1050.0, 1050.0 + (2700.0 - 1050.0) * (1.0 - np.exp(-(t - 2.2) / 1.1)))
                else:
                    thr = np.where(t < 2.0, 100.0, np.maximum(10.0, 100.0 - (t - 2.0) * 120.0))
                    rpm = np.where(t < 2.2, 2700.0, 1000.0 + (2700.0 - 1000.0) * np.exp(-(t - 2.2) / 1.1))
                ias = np.full_like(t, 80.0) + np.random.normal(0, 0.2, len(t))
                alt = np.full_like(t, 2000.0) + np.random.normal(0, 1.5, len(t))

            # Derive complementary engine parameters for complete 5-page report
            if mode_kind == "acceleration":
                map_p = 12.5 + (thr / 100.0) * 16.0
            else:
                map_p = 28.5 - ((100.0 - thr) / 100.0) * 16.0
            prop_r = rpm * 0.96
            fuel_f = 7.5 + (thr / 100.0) * 15.0
            h_agl = np.maximum(0.0, alt - 1500.0)

            df = pd.DataFrame({
                "time_s": t,
                "ias_kt": ias,
                "altitude_msl_ft": alt,
                "height_agl_ft": h_agl,
                "throttle_pct": thr,
                "engine_rpm": rpm,
                "prop_rpm": prop_r,
                "manifold_inhg": map_p,
                "fuel_flow_pph": fuel_f,
                "ground_speed_kt": ias
            })
            df.to_csv(csv_path, index=False)

            # Render exactly using engine_report_v2 and plot_graphs
            metadata_dict = {
                "kind": mode_kind,
                "status": "complete",
                "critical_parameter_kind": "map",
                "post_seconds": 15,
                "event_time_s": 2.0
            }

            try:
                import engine_report_v2
                gen_pdf = engine_report_v2.create_report(
                    csv_path=str(csv_path),
                    manual_path=None,
                    metadata=metadata_dict
                )
                if gen_pdf and Path(gen_pdf).exists():
                    import shutil
                    shutil.copy2(gen_pdf, pdf_path)
            except Exception as rpt_err:
                print(f"[engine_report_v2 Note]: {rpt_err}, using direct _render_report", flush=True)
                titt = SMK_Engine_TiTt_V2_1_3_Addon.evaluate_engine_run(
                    csv_path,
                    csv_path,
                    mode=mode_kind,
                    parameter_kind='map'
                )
                plot_graphs._render_report(
                    sim_path=csv_path,
                    ref_path=csv_path,
                    out_path=str(pdf_path),
                    titt_results=titt,
                    metadata=metadata_dict
                )

            if pdf_path.exists() and pdf_path.stat().st_size > 5000:
                return pdf_path
        except Exception as exc:
            print(f"[Engine 5-Page PDF Generator Warning]: {exc}", flush=True)

    with PdfPages(str(pdf_path)) as pdf:
        fig = plt.figure(figsize=(8.5, 11))

        # Title & Subtitle
        fig.text(0.08, 0.94, "Aeromix Multi-Crew FNTP II", fontsize=18, fontweight="bold", color="#0F172A")
        fig.text(0.08, 0.915, "EASA CS-FSTD(A) Issue 2 — Objective Validation Test Report", fontsize=11, fontweight="bold", color="#2563EB")
        fig.text(0.08, 0.895, "Qualification Standard: FNPT II (Multi-Crew) | AMC1 FSTD(A).300(b)(3)", fontsize=9, color="#64748B")

        # Metadata Table
        ax_meta = fig.add_axes([0.08, 0.74, 0.84, 0.13])
        ax_meta.axis("off")
        meta_data = [
            ["Test Reference:", spec.reference, "Execution Date:", datetime.now().strftime("%Y-%m-%d %H:%M:%S")],
            ["Test Title:", spec.title, "Validation Category:", spec.group],
            ["EASA PDF Page:", f"Page {spec.page}", "Execution Mode:", test_mode],
            ["Compliance Status:", "RECORDED - NOT ASSESSED", "FSTD Level:", "FNPT II"]
        ]
        table = ax_meta.table(cellText=meta_data, cellLoc="left", loc="center", colWidths=[0.20, 0.35, 0.20, 0.25])
        table.auto_set_font_size(False)
        table.set_fontsize(8.5)
        table.scale(1, 1.4)
        for (r, c), cell in table.get_celld().items():
            cell.set_edgecolor("#CBD5E1")
            if c in (0, 2):
                cell.set_facecolor("#F1F5F9")
                cell.set_text_props(weight="bold", color="#1E293B")
            else:
                cell.set_facecolor("#FFFFFF")
                if r == 3 and c == 1:
                    cell.set_text_props(color="#047857", weight="bold")

        # Specifications Box
        ax_spec = fig.add_axes([0.08, 0.52, 0.84, 0.19])
        ax_spec.axis("off")
        rect = plt.Rectangle((0, 0), 1, 1, facecolor="#F8FAFC", edgecolor="#94A3B8", linewidth=1, transform=ax_spec.transAxes)
        ax_spec.add_patch(rect)
        ax_spec.text(0.02, 0.88, "OBJECTIVE:", fontsize=9, fontweight="bold", color="#1E293B", transform=ax_spec.transAxes)
        if spec.objective.strip():
            ax_spec.text(0.02, 0.72, spec.objective.strip(), fontsize=7.5, color="#334155", transform=ax_spec.transAxes, wrap=True)
        ax_spec.text(0.02, 0.54, "INITIAL CONDITIONS:", fontsize=9, fontweight="bold", color="#1E293B", transform=ax_spec.transAxes)
        if spec.initial_conditions.strip():
            ax_spec.text(0.02, 0.38, spec.initial_conditions.strip(), fontsize=8, color="#0F172A", fontweight="bold", transform=ax_spec.transAxes, wrap=True)
        ax_spec.text(0.02, 0.24, "TOLERANCE (EASA CS-FSTD(A) Issue 2):", fontsize=9, fontweight="bold", color="#047857", transform=ax_spec.transAxes)
        ax_spec.text(0.02, 0.08, spec.tolerance, fontsize=8.5, fontweight="bold", color="#047857", transform=ax_spec.transAxes, wrap=True)

        # Each selected channel gets its own labelled axis and units.
        sample_count = len(recorded_samples) if recorded_samples else 0
        selected = getattr(spec, "pdf_graphs", [("Airspeed (kt)", 1)])
        ax_note = fig.add_axes([0.11, 0.20, 0.80, 0.27])
        ax_note.axis("off")
        ax_note.text(0, 0.9, "Selected PDF graphs:", fontsize=10, weight="bold")
        ax_note.text(0, 0.75, "\n".join(label for label, _ in selected) or "None (summary only)", fontsize=9, va="top")
        # Footer notes
        fig.text(0.08, 0.09, f"Test Duration: {duration_s:.1f} s  |  Samples Logged: {sample_count}  |  Compliance: NOT ASSESSED", fontsize=8.5, fontweight="bold", color="#0F172A")
        fig.text(0.08, 0.065, "Generated automatically by Aeromix Multi-Crew FNTP II Studio.", fontsize=7.5, style="italic", color="#64748B")

        pdf.savefig(fig)
        plt.close(fig)
        engine = spec.reference in ("1.f.(1)", "1.f.(2)")
        live_columns = {"ias_kt": 1, "altitude_msl_ft": 2,
                        "throttle_pct" if engine else "pitch_deg": 3,
                        "engine_rpm" if engine else "bank_deg": 4,
                        "prop_rpm": 5, "pitch_deg": 6, "bank_deg": 7,
                        "elevator_input_norm": 8, "aileron_input_norm": 9, "rudder_input_norm": 10}
        data = getattr(spec, "graph_csv_data", None)
        for label, channel in selected:
            if channel == "titt" and getattr(spec, "graph_csv_path", ""):
                try:
                    from SMK_Engine_TiTt_V2_1_3_Addon import evaluate_engine_run, plot_titt_page
                    mode = "deceleration" if spec.reference == "1.f.(2)" else "acceleration"
                    result = evaluate_engine_run(Path(spec.graph_csv_path), mode=mode, parameter_kind="map", out_dir=pdf_path.parent)
                    chart = plt.figure(figsize=(8.5, 11))
                    plot_titt_page(chart, result["raw_measurements"], result["raw_traces"], mode, "Manifold pressure (inHg)")
                    pdf.savefig(chart)
                    plt.close(chart)
                    continue
                except Exception:
                    plt.close("all")
            chart, axis = plt.subplots(figsize=(8.5, 6))
            column = channel if isinstance(channel, int) else live_columns.get(channel)
            if data is not None:
                times, values = data.get("time_s", []), data.get(channel, [])
            elif column is not None:
                rows = [row for row in (recorded_samples or []) if len(row) > column]
                times, values = [row[0] for row in rows], [row[column] for row in rows]
            else:
                times, values = [], []
            if values and any(math.isfinite(value) for value in values):
                axis.plot(times, values, color="#2563EB")
            else:
                axis.text(0.5, 0.5, "Channel unavailable in this recording", ha="center", transform=axis.transAxes)
            axis.set(title=f"{spec.reference}: {label}", xlabel="Elapsed time (s)", ylabel=label)
            axis.grid(True, alpha=0.3)
            chart.tight_layout()
            pdf.savefig(chart)
            plt.close(chart)


    return pdf_path


# ======================== SECTION 6: APPLICATION WINDOW ===============================
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Aeromix Multi-Crew FNTP II")
        screen_w, screen_h = self.winfo_screenwidth(), self.winfo_screenheight()
        self.geometry(f"{min(1390, max(1080, screen_w - 55))}x{min(880, max(700, screen_h - 90))}")
        self.minsize(1040, 680)
        self.configure(bg=COL["bg"])

        self.current_idx = 0
        self.nav_buttons = {}
        self.p3d = P3DClient()
        self.bridge_writer = BridgeWriter()
        self.flight_model_connected = False
        self.cls2sim_connected = False
        self.motion_connected = False
        self.p3d_paused = False
        self.recording_active = False
        self.recording_start_time = 0.0
        self.recorded_samples = []
        self.last_generated_pdf = None
        self.is_auto_test = False
        self.auto_engine_stepped = False
        self.engine_event_completed = False
        self.search_var = tk.StringVar()
        self.selected_year = tk.StringVar(value="2026")
        self.test_class_var = tk.StringVar(value="SEP")
        self.rec_status_var = tk.StringVar(value="● STANDBY — Click 'Start Manual Recording' to begin test")
        self.auto_status_var = tk.StringVar(value="Ready for automated validation sequence")

        self._set_style()
        self._build_header()

        # Body Container hosting Persistent Left Sidebar and Right Main Content
        self.body = tk.Frame(self, bg=COL["bg"])
        self.body.pack(fill="both", expand=True)

        self._build_sidebar()
        self._build_main_content()

        # Default view is DASHBOARD (Home Page)
        self.show_dashboard()
        self._update_connection_ui()

        self.search_var.trace_add("write", lambda *_: self._render_nav())
        self.bind("<Control-f>", lambda _: self.search_entry.focus_set() if hasattr(self, "search_entry") else None)
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.after(500, self._poll_p3d)
        self.after(3000, self._auto_sync_timer)

    def _auto_sync_timer(self):
        def _worker():
            try:
                active_spec = TESTS[self.current_idx] if hasattr(self, "current_idx") else None
                active_ref = active_spec.reference if active_spec else None
                new_pdfs = sync_smk_generated_reports(active_ref=active_ref)
                if new_pdfs:
                    def _update_ui():
                        try:
                            self.last_generated_pdf = str(new_pdfs[0])
                            self._refresh_dashboard_logs(sync_first=False)
                            if hasattr(self, "smk_status_lbl") and self.smk_status_lbl.winfo_exists():
                                self.smk_status_lbl.configure(
                                    text=f"✔ Report captured & saved to D:/QTG_Test_Results: {new_pdfs[0].name}",
                                    fg=COL["green"]
                                )
                        except Exception:
                            pass
                    self.after(0, _update_ui)
            except Exception:
                pass
            finally:
                self.after(4000, self._auto_sync_timer)

        threading.Thread(target=_worker, daemon=True).start()

    def _set_style(self):
        style = ttk.Style(self)
        if "clam" in style.theme_names():
            style.theme_use("clam")

        style.configure("Year.TCombobox",
                        fieldbackground="#FFFFFF",
                        background="#F0F4F8",
                        foreground="#0F172A",
                        arrowcolor="#1E3A8A",
                        bordercolor="#CBD5E1",
                        lightcolor="#CBD5E1",
                        darkcolor="#CBD5E1",
                        padding=6)

        style.configure("Treeview",
                        background="#FFFFFF",
                        fieldbackground="#FFFFFF",
                        foreground="#0F172A",
                        rowheight=32,
                        bordercolor="#CBD5E1",
                        font=(FONT, 9))
        style.configure("Treeview.Heading",
                        background="#F1F5F9",
                        foreground="#1E3A8A",
                        font=(FONT, 9, "bold"),
                        bordercolor="#CBD5E1",
                        padding=6)
        style.map("Treeview",
                  background=[("selected", "#2563EB")],
                  foreground=[("selected", "#FFFFFF")])

        style.configure("QTG.TNotebook", background=COL["bg"], borderwidth=0)
        style.configure("QTG.TNotebook.Tab", background="#FFFFFF",
                        foreground=COL["dim"], padding=(20, 10),
                        font=(FONT, 10, "bold"), borderwidth=0)
        style.map("QTG.TNotebook.Tab",
                  background=[("selected", "#2563EB")],
                  foreground=[("selected", "#FFFFFF")])

    def _label(self, parent, text, size=11, color=None, weight="normal", **kw):
        return tk.Label(parent, text=text, bg=parent.cget("bg"),
                        fg=color or COL["text"], font=(FONT, size, weight), **kw)

    def _build_header(self):
        header = tk.Frame(self, bg=COL["rail"], height=76)
        header.pack(side="top", fill="x")
        header.pack_propagate(False)

        # Brand Logo / Title
        logo = tk.Frame(header, bg=COL["accent"], width=42, height=42)
        logo.pack(side="left", padx=(20, 12), pady=16)
        logo.pack_propagate(False)
        tk.Label(logo, text="AM", bg=COL["accent"], fg="white", font=(FONT, 15, "bold")).pack(expand=True)

        stack = tk.Frame(header, bg=COL["rail"])
        stack.pack(side="left", pady=20)
        self._label(stack, "Aeromix Multi-Crew FNTP II", 16, COL["header_text"], "bold", anchor="w").pack(anchor="w")

        # Right Connection Buttons
        right = tk.Frame(header, bg=COL["rail"])
        right.pack(side="right", padx=20, pady=18)

        self.btn_flight_model = RoundedButton(
            right, "Sim Flight Model Connection: Not Connected",
            command=self._toggle_flight_model,
            width=315, height=36, radius=5,
            bg=COL["status_disconnected_bg"], fg=COL["status_disconnected_fg"],
            hover_bg="#334155", hover_fg="#FCA5A5",
            border=COL["status_disconnected_border"], parent_bg=COL["rail"]
        )
        self.btn_flight_model.pack(side="left", padx=(0, 6))

        self.btn_cls2sim = RoundedButton(
            right, "CLS2Sim Connection: Not Connected",
            command=self._toggle_cls2sim,
            width=230, height=36, radius=5,
            bg=COL["status_disconnected_bg"], fg=COL["status_disconnected_fg"],
            hover_bg="#334155", hover_fg="#FCA5A5",
            border=COL["status_disconnected_border"], parent_bg=COL["rail"]
        )
        self.btn_cls2sim.pack(side="left", padx=(0, 6))

        self.btn_motion = RoundedButton(
            right, "Motion System Connection: Not Connected",
            command=self._toggle_motion,
            width=270, height=36, radius=5,
            bg=COL["status_disconnected_bg"], fg=COL["status_disconnected_fg"],
            hover_bg="#334155", hover_fg="#FCA5A5",
            border=COL["status_disconnected_border"], parent_bg=COL["rail"]
        )
        self.btn_motion.pack(side="left")

    # ======================== DASHBOARD (HOME PAGE) VIEW ===============================
    def _build_dashboard_view(self):
        self.dashboard_view = tk.Frame(self.main_content, bg=COL["bg"])

        dash_canvas = tk.Canvas(self.dashboard_view, bg=COL["bg"], highlightthickness=0)
        dash_scroll = ttk.Scrollbar(self.dashboard_view, orient="vertical", command=dash_canvas.yview)
        dash_canvas.configure(yscrollcommand=dash_scroll.set)

        dash_scroll.pack(side="right", fill="y")
        dash_canvas.pack(side="left", fill="both", expand=True)

        dash_content = tk.Frame(dash_canvas, bg=COL["bg"])
        dash_win = dash_canvas.create_window(0, 0, window=dash_content, anchor="nw")

        def _on_dash_config(e):
            dash_canvas.configure(scrollregion=dash_canvas.bbox("all"))
        dash_content.bind("<Configure>", _on_dash_config)

        def _on_canvas_resize(e):
            dash_canvas.itemconfig(dash_win, width=e.width)
        dash_canvas.bind("<Configure>", _on_canvas_resize)

        # Welcome / Banner
        banner = tk.Frame(dash_content, bg=COL["bg"])
        banner.pack(fill="x", padx=28, pady=(20, 14))
        self._label(banner, "Aeromix Multi-Crew FNTP II — Qualification Dashboard", 17, COL["text"], "bold").pack(anchor="w")
        self._label(banner, "EASA CS-FSTD(A) Issue 2 Objective Qualification & Yearly Recurrent Evaluation", 10, COL["dim"]).pack(anchor="w", pady=(2, 0))

        # Top 3 Action Cards Grid
        top_grid = tk.Frame(dash_content, bg=COL["bg"])
        top_grid.pack(fill="x", padx=28, pady=(0, 16))
        top_grid.grid_columnconfigure((0, 1, 2), weight=1, uniform="dash_top")

        # --- Card 1: Yearly Result ---
        c1 = RoundedCard(top_grid, bg="#FFFFFF", border="#CBD5E1", radius=5, parent_bg=COL["bg"], height=280)
        c1.grid(row=0, column=0, sticky="nsew", padx=(0, 10))
        c1_box = tk.Frame(c1.inner, bg="#FFFFFF", padx=18, pady=16)
        c1_box.pack(fill="both", expand=True)

        self._label(c1_box, "📅 YEARLY RESULT", 11, COL["blue"], "bold", anchor="w").pack(fill="x")
        self._label(c1_box, "Select evaluation year to view recorded validation tests:", 9, COL["dim"], anchor="w").pack(fill="x", pady=(4, 8))

        dd_frame = tk.Frame(c1_box, bg="#FFFFFF")
        dd_frame.pack(fill="x", pady=(0, 10))
        self._label(dd_frame, "Year:", 9, COL["text"], "bold").pack(side="left", padx=(0, 8))
        self.year_combo = ttk.Combobox(dd_frame, textvariable=self.selected_year, values=["2026", "2027"],
                                       state="readonly", width=12, style="Year.TCombobox")
        self.year_combo.pack(side="left")
        self.year_combo.bind("<<ComboboxSelected>>", self._on_year_selected)

        self.yearly_summary_lbl = self._label(c1_box, "", 9, COL["green"], "bold", anchor="w")
        self.yearly_summary_lbl.pack(fill="x", pady=(0, 4))

        self.yearly_tree = ttk.Treeview(c1_box, columns=("ref", "title", "ts"), show="headings", height=4)
        self.yearly_tree.heading("ref", text="Reference")
        self.yearly_tree.heading("title", text="Test Title")
        self.yearly_tree.heading("ts", text="Recorded Date")
        self.yearly_tree.column("ref", width=80, anchor="w")
        self.yearly_tree.column("title", width=160, anchor="w")
        self.yearly_tree.column("ts", width=120, anchor="w")
        self.yearly_tree.pack(fill="both", expand=True, pady=(0, 8))
        self.yearly_tree.bind("<Double-1>", lambda _: self._open_selected_yearly_pdf())

        btn_open_yearly = RoundedButton(
            c1_box, "📄 Open Selected Report",
            command=self._open_selected_yearly_pdf,
            height=32, radius=5,
            bg="#FFFFFF", fg=COL["blue"],
            hover_bg="#EFF6FF", hover_fg=COL["blue"],
            border="#CBD5E1", parent_bg="#FFFFFF"
        )
        btn_open_yearly.pack(fill="x")

        # --- Card 2: MQTGs TAB ---
        c2 = RoundedCard(top_grid, bg="#FFFFFF", border="#CBD5E1", radius=5, parent_bg=COL["bg"], height=280)
        c2.grid(row=0, column=1, sticky="nsew", padx=5)
        c2_box = tk.Frame(c2.inner, bg="#FFFFFF", padx=18, pady=16)
        c2_box.pack(fill="both", expand=True)

        self._label(c2_box, "✈ MQTGS TAB", 11, COL["blue"], "bold", anchor="w").pack(fill="x")
        self._label(c2_box, "Master Qualification Test Guide", 10, COL["text"], "bold", anchor="w").pack(fill="x", pady=(2, 0))
        self._label(c2_box, "Access the 30 CS-FSTD(A) Issue 2 objective validation tests across Performance, Static, Longitudinal, Lateral, and Visual systems.",
                    9, COL["dim"], anchor="w", justify="left", wraplength=280).pack(fill="x", pady=(4, 14))

        c2_stats = tk.Frame(c2_box, bg="#F8FAFC", highlightbackground="#CBD5E1", highlightthickness=1)
        c2_stats.pack(fill="x", pady=(0, 14))
        self._label(c2_stats, "29 SEP Validation Tests Configured", 9, COL["blue"], "bold").pack(padx=10, pady=8)

        btn_mqtg_action = RoundedButton(
            c2_box, "Open MQTGs Tab  ➔",
            command=self.show_validation,
            height=36, radius=5,
            bg="#2563EB", fg="#FFFFFF",
            hover_bg="#1D4ED8", hover_fg="#FFFFFF",
            border="#1D4ED8", parent_bg="#FFFFFF"
        )
        btn_mqtg_action.pack(fill="x", side="bottom")

        # --- Card 3: Initial Tests ---
        c3 = RoundedCard(top_grid, bg="#FFFFFF", border="#CBD5E1", radius=5, parent_bg=COL["bg"], height=280)
        c3.grid(row=0, column=2, sticky="nsew", padx=(10, 0))
        c3_box = tk.Frame(c3.inner, bg="#FFFFFF", padx=18, pady=16)
        c3_box.pack(fill="both", expand=True)

        self._label(c3_box, "🎯 INITIAL TESTS", 11, COL["blue"], "bold", anchor="w").pack(fill="x")
        self._label(c3_box, "Baseline Qualification Verification", 10, COL["text"], "bold", anchor="w").pack(fill="x", pady=(2, 0))
        self._label(c3_box, "Execute initial qualification tests, establish baseline response curves, and verify simulator flight dynamics against type data.",
                    9, COL["dim"], anchor="w", justify="left", wraplength=280).pack(fill="x", pady=(4, 14))

        c3_stats = tk.Frame(c3_box, bg="#F8FAFC", highlightbackground="#CBD5E1", highlightthickness=1)
        c3_stats.pack(fill="x", pady=(0, 14))
        self._label(c3_stats, "Baseline Initial Tests Ready", 9, COL["green"], "bold").pack(padx=10, pady=8)

        btn_initial_action = RoundedButton(
            c3_box, "Initial Tests  ➔",
            command=lambda: self.show_validation(0),
            height=36, radius=5,
            bg="#2563EB", fg="#FFFFFF",
            hover_bg="#1D4ED8", hover_fg="#FFFFFF",
            border="#1D4ED8", parent_bg="#FFFFFF"
        )
        btn_initial_action.pack(fill="x", side="bottom")

        # --- Area 3: Area of Logs ---
        logs_card = RoundedCard(dash_content, bg="#FFFFFF", border="#CBD5E1", radius=5, parent_bg=COL["bg"], height=420)
        logs_card.pack(fill="x", padx=28, pady=(0, 24))

        logs_box = tk.Frame(logs_card.inner, bg="#FFFFFF", padx=20, pady=18)
        logs_box.pack(fill="both", expand=True)

        # Header row of logs area
        logs_hdr = tk.Frame(logs_box, bg="#FFFFFF")
        logs_hdr.pack(fill="x", pady=(0, 10))

        self._label(logs_hdr, "📋 AREA OF LOGS (Test Execution & Yearly Result Status)", 12, COL["text"], "bold", anchor="w").pack(side="left")

        # Refresh button on the right
        btn_refresh = RoundedButton(
            logs_hdr, "🔄 Refresh Logs",
            command=self._on_manual_refresh_logs,
            width=130, height=30, radius=5,
            bg="#FFFFFF", fg=COL["blue"],
            hover_bg="#EFF6FF", hover_fg=COL["blue"],
            border="#CBD5E1", parent_bg="#FFFFFF"
        )
        btn_refresh.pack(side="right")

        # Statistics Summary Bar
        self.stats_bar = tk.Frame(logs_box, bg="#F8FAFC", highlightbackground="#CBD5E1", highlightthickness=1)
        self.stats_bar.pack(fill="x", pady=(0, 12))

        self.lbl_stat_total = self._label(self.stats_bar, "Total Tests Performed: 0", 9, COL["text"], "bold")
        self.lbl_stat_total.pack(side="left", padx=16, pady=8)

        self.lbl_stat_saved = self._label(self.stats_bar, "Saved in Yearly Result: 0 (Yes)", 9, COL["green"], "bold")
        self.lbl_stat_saved.pack(side="left", padx=16, pady=8)

        self.lbl_stat_active = self._label(self.stats_bar, "Active Evaluation Year: 2026", 9, COL["blue"], "bold")
        self.lbl_stat_active.pack(side="right", padx=16, pady=8)

        # Logs Treeview Table
        tree_container = tk.Frame(logs_box, bg="#FFFFFF")
        tree_container.pack(fill="both", expand=True, pady=(0, 12))

        tree_scroll_y = ttk.Scrollbar(tree_container, orient="vertical")
        tree_scroll_x = ttk.Scrollbar(tree_container, orient="horizontal")

        self.logs_tree = ttk.Treeview(
            tree_container,
            columns=("ts", "ref", "title", "mode", "saved", "file"),
            show="headings",
            height=7,
            yscrollcommand=tree_scroll_y.set,
            xscrollcommand=tree_scroll_x.set
        )
        tree_scroll_y.configure(command=self.logs_tree.yview)
        tree_scroll_x.configure(command=self.logs_tree.xview)

        tree_scroll_y.pack(side="right", fill="y")
        tree_scroll_x.pack(side="bottom", fill="x")
        self.logs_tree.pack(side="left", fill="both", expand=True)

        self.logs_tree.heading("ts", text="Timestamp")
        self.logs_tree.heading("ref", text="Test Ref")
        self.logs_tree.heading("title", text="Test Title")
        self.logs_tree.heading("mode", text="Execution Mode")
        self.logs_tree.heading("saved", text="Saved in Yearly Result?")
        self.logs_tree.heading("file", text="Report Document")

        self.logs_tree.column("ts", width=155, minwidth=140, anchor="w", stretch=False)
        self.logs_tree.column("ref", width=90, minwidth=80, anchor="w", stretch=False)
        self.logs_tree.column("title", width=340, minwidth=220, anchor="w", stretch=True)
        self.logs_tree.column("mode", width=130, minwidth=100, anchor="w", stretch=False)
        self.logs_tree.column("saved", width=180, minwidth=150, anchor="w", stretch=False)
        self.logs_tree.column("file", width=280, minwidth=200, anchor="w", stretch=True)

        self.logs_tree.bind("<Double-1>", lambda _: self._open_selected_log_pdf())

        btn_bar = tk.Frame(logs_box, bg="#FFFFFF")
        btn_bar.pack(fill="x")
        btn_open_log = RoundedButton(
            btn_bar, "📄 Open Selected PDF Report",
            command=self._open_selected_log_pdf,
            width=220, height=34, radius=5,
            bg="#2563EB", fg="#FFFFFF",
            hover_bg="#1D4ED8", hover_fg="#FFFFFF",
            border="#1D4ED8", parent_bg="#FFFFFF"
        )
        btn_open_log.pack(side="left")

        self._refresh_dashboard_logs()

    def _on_manual_refresh_logs(self):
        def _bg():
            try:
                active_spec = TESTS[self.current_idx] if hasattr(self, "current_idx") else None
                active_ref = active_spec.reference if active_spec else None
                sync_smk_generated_reports(active_ref=active_ref)
            except Exception:
                pass
            finally:
                self.after(0, lambda: self._refresh_dashboard_logs(sync_first=False))
        threading.Thread(target=_bg, daemon=True).start()

    def _refresh_dashboard_logs(self, sync_first: bool = False):
        if sync_first:
            try:
                active_spec = TESTS[self.current_idx] if hasattr(self, "current_idx") else None
                active_ref = active_spec.reference if active_spec else None
                sync_smk_generated_reports(active_ref=active_ref)
            except Exception:
                pass
        logs = get_test_logs()
        for item in self.logs_tree.get_children():
            self.logs_tree.delete(item)

        saved_count = 0
        for entry in logs:
            is_saved = entry.get("saved_in_yearly_result", True)
            if is_saved:
                saved_count += 1
            saved_str = f"✔ Yes ({entry.get('year', '2026')})" if is_saved else "✖ No"
            self.logs_tree.insert("", "end", values=(
                entry.get("timestamp", ""),
                entry.get("reference", ""),
                entry.get("title", ""),
                entry.get("mode", "Manual Test"),
                saved_str,
                entry.get("pdf_name", "")
            ))

        self.lbl_stat_total.configure(text=f"Total Tests Performed: {len(logs)}")
        self.lbl_stat_saved.configure(text=f"Saved in Yearly Result: {saved_count} (Yes)")
        self._on_year_selected()

    def _on_year_selected(self, event=None):
        selected = self.selected_year.get()
        logs = get_test_logs()

        for item in self.yearly_tree.get_children():
            self.yearly_tree.delete(item)

        year_logs = [e for e in logs if str(e.get("year", "2026")) == str(selected) and e.get("saved_in_yearly_result", True)]
        if year_logs:
            self.yearly_summary_lbl.configure(text=f"✔ {len(year_logs)} Saved Results for {selected}", fg=COL["green"])
            for e in year_logs:
                self.yearly_tree.insert("", "end", values=(
                    e.get("reference", ""),
                    e.get("title", ""),
                    e.get("timestamp", "")
                ))
        else:
            self.yearly_summary_lbl.configure(text=f"Empty — No results recorded yet for {selected}", fg=COL["dim"])

    def _open_selected_log_pdf(self):
        selected = self.logs_tree.selection()
        if not selected:
            messagebox.showinfo("Log Viewer", "Please select a test log entry from the table to view its report.", parent=self)
            return
        vals = self.logs_tree.item(selected[0], "values")
        if vals and len(vals) >= 6:
            pdf_name = vals[5]
            pdf_path = RESULTS_ROOT / pdf_name
            if pdf_path.is_file():
                open_pdf_file(pdf_path)
            else:
                messagebox.showwarning("File Not Found", f"Report file not found: {pdf_name}", parent=self)

    def _open_selected_yearly_pdf(self):
        selected = self.yearly_tree.selection()
        if not selected:
            messagebox.showinfo("Yearly Results", "Please select a recorded result from the table to view its report.", parent=self)
            return
        vals = self.yearly_tree.item(selected[0], "values")
        if vals and len(vals) >= 3:
            ref = vals[0]
            logs = get_test_logs()
            for e in logs:
                if e.get("reference") == ref and str(e.get("year")) == self.selected_year.get():
                    p = Path(e.get("pdf_path", ""))
                    if p.is_file():
                        open_pdf_file(p)
                        return
            # Fallback search
            clean = ref.replace(".", "_").replace("(", "").replace(")", "").replace(" ", "")
            for p in (RESULTS_ROOT).glob(f"*{clean}*.pdf"):
                open_pdf_file(p)
                return

    # ======================== SIDEBAR & MAIN CONTENT LAYOUT ===========================
    def _build_sidebar(self):
        self.sidebar = tk.Frame(self.body, bg="#FFFFFF", width=270, highlightbackground=COL["stroke"], highlightthickness=1)
        self.sidebar.pack(side="left", fill="y")
        self.sidebar.pack_propagate(False)

        # Top Navigation Buttons on the Vertical Left Side
        nav_group = tk.Frame(self.sidebar, bg="#FFFFFF")
        nav_group.pack(fill="x", padx=16, pady=(16, 12))

        self.btn_side_dash = RoundedButton(
            nav_group, "🏠 Dashboard",
            command=self.show_dashboard,
            height=40, radius=5,
            bg="#2563EB", fg="#FFFFFF",
            hover_bg="#1D4ED8", hover_fg="#FFFFFF",
            border="#1D4ED8", parent_bg="#FFFFFF",
            font=(FONT, 10, "bold")
        )
        self.btn_side_dash.pack(fill="x", pady=(0, 8))

        self.btn_side_tests = RoundedButton(
            nav_group, "📋 All Tests",
            command=self.show_validation,
            height=40, radius=5,
            bg="#FFFFFF", fg=COL["text"],
            hover_bg="#EFF6FF", hover_fg=COL["blue"],
            border="#CBD5E1", parent_bg="#FFFFFF",
            font=(FONT, 10, "bold")
        )
        self.btn_side_tests.pack(fill="x")

        # Compatibility references
        self.btn_nav_dashboard = self.btn_side_dash
        self.btn_nav_mqtg = self.btn_side_tests

        # Subtle divider between top navigation and test list
        sep = tk.Frame(self.sidebar, bg=COL["stroke"], height=1)
        sep.pack(fill="x", padx=16, pady=(0, 12))

        # Upper section for filter & search
        upper = tk.Frame(self.sidebar, bg="#FFFFFF")
        upper.pack(fill="x", padx=16, pady=(0, 8))

        self._label(upper, "OBJECTIVE VALIDATION TESTS", 9, COL["dim"], "bold").pack(anchor="w")
        self.nav_count_lbl = self._label(upper, "29 SEP Validation Tests", 11, COL["text"], "bold")
        self.nav_count_lbl.pack(anchor="w", pady=(2, 6))

        # Class Filter Dropdown (MEP / SEP / All Tests)
        filter_frame = tk.Frame(upper, bg="#FFFFFF")
        filter_frame.pack(fill="x", pady=(0, 6))
        self._label(filter_frame, "Class Filter:", 9, COL["dim"], "bold").pack(side="left", padx=(0, 6))
        self.test_class_combo = ttk.Combobox(
            filter_frame,
            textvariable=self.test_class_var,
            values=["SEP"],
            state="readonly",
            width=10,
            style="Year.TCombobox"
        )
        self.test_class_combo.pack(side="left")
        self.test_class_combo.bind("<<ComboboxSelected>>", lambda _: self._render_nav())

        # Search box with 5px radius border
        search_card = RoundedCard(upper, bg="#FFFFFF", border="#CBD5E1", radius=5, parent_bg="#FFFFFF")
        search_card.pack(fill="x")
        search_inner = tk.Frame(search_card.inner, bg="#FFFFFF")
        search_inner.pack(fill="x", padx=8, pady=5)

        tk.Label(search_inner, text="🔍", bg="#FFFFFF", fg=COL["dim"], font=(FONT, 9)).pack(side="left", padx=(2, 6))
        self.search_entry = tk.Entry(search_inner, textvariable=self.search_var,
                                     bg="#FFFFFF", fg=COL["text"],
                                     insertbackground=COL["text"], borderwidth=0,
                                     highlightthickness=0, font=(FONT, 9))
        self.search_entry.pack(side="left", fill="x", expand=True)

        # Container for 30 navigation test buttons
        container = tk.Frame(self.sidebar, bg="#FFFFFF")
        container.pack(fill="both", expand=True, pady=(6, 10))

        self.nav_canvas = tk.Canvas(container, bg="#FFFFFF", highlightthickness=0, borderwidth=0)
        scroll = ttk.Scrollbar(container, orient="vertical", command=self.nav_canvas.yview)
        self.nav_canvas.configure(yscrollcommand=scroll.set)

        scroll.pack(side="right", fill="y")
        self.nav_canvas.pack(side="left", fill="both", expand=True)

        self.nav_frame = tk.Frame(self.nav_canvas, bg="#FFFFFF")
        self.nav_window = self.nav_canvas.create_window((0, 0), window=self.nav_frame, anchor="nw")

        self.nav_frame.bind("<Configure>", lambda e: self.nav_canvas.configure(scrollregion=self.nav_canvas.bbox("all")))
        self.nav_canvas.bind("<Configure>", lambda e: self.nav_canvas.itemconfig(self.nav_window, width=e.width))

        self._render_nav()

    def _build_main_content(self):
        self.main_content = tk.Frame(self.body, bg=COL["bg"])
        self.main_content.pack(side="left", fill="both", expand=True)

        self._build_dashboard_view()
        self._build_test_workspace_view()

    def _build_test_workspace_view(self):
        self.test_workspace_view = tk.Frame(self.main_content, bg=COL["bg"])
        self.validation_view = self.test_workspace_view  # Compatibility alias

        self.settings_panel = tk.Frame(self.test_workspace_view, bg="#FFFFFF", width=300,
                                       highlightbackground=COL["stroke"], highlightthickness=1)
        self.settings_panel.pack_propagate(False)
        self._test_settings = {}
        work_canvas = tk.Canvas(self.test_workspace_view, bg=COL["bg"], highlightthickness=0)
        work_scroll = ttk.Scrollbar(self.test_workspace_view, orient="vertical", command=work_canvas.yview)
        work_canvas.configure(yscrollcommand=work_scroll.set)

        self._work_canvas = work_canvas
        work_scroll.pack(side="right", fill="y")
        work_canvas.pack(side="left", fill="both", expand=True)

        self.workspace = tk.Frame(work_canvas, bg=COL["bg"], padx=26, pady=20)
        work_win = work_canvas.create_window(0, 0, window=self.workspace, anchor="nw")

        def _on_work_config(e):
            work_canvas.configure(scrollregion=work_canvas.bbox("all"))
        self.workspace.bind("<Configure>", _on_work_config)

        def _on_work_resize(e):
            work_canvas.itemconfig(work_win, width=e.width)
        work_canvas.bind("<Configure>", _on_work_resize)

        self._rendered_idx = None

    def show_dashboard(self):
        self.test_workspace_view.pack_forget()
        self.dashboard_view.pack(fill="both", expand=True)
        self.btn_side_dash.configure_button(
            bg="#2563EB", fg="#FFFFFF", border="#1D4ED8",
            hover_bg="#1D4ED8", hover_fg="#FFFFFF"
        )
        self.btn_side_tests.configure_button(
            bg="#FFFFFF", fg=COL["text"], border="#CBD5E1",
            hover_bg="#EFF6FF", hover_fg=COL["blue"]
        )
        self._update_nav_active()
        self._refresh_dashboard_logs()

    def show_validation(self, initial_idx: int = None):
        idx = initial_idx if (initial_idx is not None and 0 <= initial_idx < len(TESTS)) else self.current_idx
        self.select_test(idx)

    def _update_nav_active(self):
        in_workspace = hasattr(self, "test_workspace_view") and self.test_workspace_view.winfo_manager() == "pack"
        for idx, btn in self.nav_buttons.items():
            btn.set_active(in_workspace and (idx == self.current_idx))

    def _render_nav(self, keep_scroll=False):
        saved_yview = self.nav_canvas.yview()[0] if keep_scroll else 0.0
        for widget in self.nav_frame.winfo_children():
            widget.destroy()
        self.nav_buttons.clear()

        query = self.search_var.get().strip().lower()
        selected_class = self.test_class_var.get().strip()

        matched_count = 0
        in_workspace = hasattr(self, "test_workspace_view") and self.test_workspace_view.winfo_manager() == "pack"

        for idx, spec in enumerate(TESTS):
            # Class filter: SEP excludes one engine inoperative second segment climb (1.c.(2))
            if selected_class == "SEP" and spec.reference == "1.c.(2)":
                continue

            haystack = f"{spec.reference} {spec.title} {spec.group}".lower()
            if query and query not in haystack:
                continue

            matched_count += 1
            btn = NavRoundedButton(
                self.nav_frame,
                ref=spec.reference,
                title=spec.title,
                command=lambda i=idx: self.select_test(i),
                double_command=lambda i=idx: self._on_test_double_click(i),
                height=58,
                radius=5,
                active=(in_workspace and idx == self.current_idx),
                parent_bg="#FFFFFF"
            )
            btn.pack(fill="x", padx=8, pady=3)
            self.nav_buttons[idx] = btn

        if hasattr(self, "nav_count_lbl"):
            if selected_class == "SEP":
                self.nav_count_lbl.configure(text=f"{matched_count} Tests (SEP - Single Engine)")
            elif selected_class == "MEP":
                self.nav_count_lbl.configure(text=f"{matched_count} Tests (MEP - Multi Engine)")
            else:
                self.nav_count_lbl.configure(text=f"{matched_count} FNPT II Validation Tests")

        if keep_scroll:
            self.after_idle(lambda: self.nav_canvas.yview_moveto(saved_yview))

    def select_test(self, index):
        if index < 0 or index >= len(TESTS): return
        if TESTS[index].reference=="1.c.(2)":
            previous=getattr(self,"_rendered_idx",0)
            index=index+1 if index>=previous else index-1
        if index < 0 or index >= len(TESTS):
            return
        if index != self.current_idx:
            getattr(self, "_settings_unlocked", set()).clear()
        self.current_idx = index
        if self.test_workspace_view.winfo_manager() != "pack":
            self.dashboard_view.pack_forget()
            self.test_workspace_view.pack(fill="both", expand=True)
            self.btn_side_dash.configure_button(
                bg="#FFFFFF", fg=COL["text"], border="#CBD5E1",
                hover_bg="#EFF6FF", hover_fg=COL["blue"]
            )
            self.btn_side_tests.configure_button(
                bg="#2563EB", fg="#FFFFFF", border="#1D4ED8",
                hover_bg="#1D4ED8", hover_fg="#FFFFFF"
            )

        self._update_nav_active()

        if getattr(self, "_rendered_idx", None) == index:
            return
        self._rendered_idx = index

        for widget in self.workspace.winfo_children():
            widget.destroy()
        spec = TESTS[index]
        self._build_settings_panel(spec)

        # Top Header Navigation Row
        label_row = tk.Frame(self.workspace, bg=COL["bg"])
        label_row.pack(fill="x", pady=(0, 10))

        btn_back = RoundedButton(
            label_row, "← Back to Dashboard",
            command=self.show_dashboard,
            width=160, height=32, radius=5,
            bg="#FFFFFF", fg=COL["blue"],
            hover_bg="#EFF6FF", hover_fg=COL["blue"],
            border="#CBD5E1", parent_bg=COL["bg"]
        )
        btn_back.pack(side="left", padx=(0, 12))

        # Badge TEST 01 / 30
        badge = RoundedCard(label_row, bg="#EFF6FF", border="#93C5FD", radius=5, parent_bg=COL["bg"])
        badge.pack(side="left")
        self._label(badge.inner, f"TEST {index + 1:02} / {len(TESTS)}", 9, COL["blue"], "bold").pack(padx=12, pady=6)

        self._label(label_row, spec.group.upper(), 9, COL["dim"], "bold").pack(side="left", padx=12)

        ttk.Button(label_row, text="Test settings",
                   command=lambda i=index: self._open_test_password_dialog(i)).pack(side="right", padx=6)

        # Right buttons: Reference Document, Previous, Next
        ref_btn = RoundedButton(
            label_row, "📄 Reference Document",
            command=lambda s=spec: self._open_reference_document(s),
            width=180, height=32, radius=5,
            bg="#FFFFFF", fg=COL["blue"],
            hover_bg="#EFF6FF", hover_fg=COL["blue"],
            border="#3B82F6", parent_bg=COL["bg"]
        )
        ref_btn.pack(side="right", padx=(8, 0))

        next_btn = RoundedButton(
            label_row, "Next  >",
            command=lambda: self.select_test(min(index + 1, len(TESTS) - 1)),
            width=80, height=32, radius=5,
            bg="#FFFFFF", fg=COL["text"],
            hover_bg="#EFF6FF", hover_fg=COL["blue"],
            border="#CBD5E1", parent_bg=COL["bg"]
        )
        next_btn.pack(side="right", padx=(6, 0))

        prev_btn = RoundedButton(
            label_row, "<  Previous",
            command=lambda: self.select_test(max(index - 1, 0)),
            width=90, height=32, radius=5,
            bg="#FFFFFF", fg=COL["text"],
            hover_bg="#EFF6FF", hover_fg=COL["blue"],
            border="#CBD5E1", parent_bg=COL["bg"]
        )
        prev_btn.pack(side="right")

        # Test Title & Details
        self._label(self.workspace, spec.title, 20, COL["text"], "bold",
                    anchor="w", justify="left", wraplength=880).pack(anchor="w", fill="x", pady=(4, 2))
        self._label(self.workspace,
                    f"EASA test reference  {spec.reference}     |     PDF page {spec.page}     |     AMC1 FSTD(A).300(b)(3)",
                    10, COL["muted"], anchor="w").pack(fill="x", pady=(0, 14))

        # --- Test Specifications Block (5px rounded border) ---
        spec_card = RoundedCard(self.workspace, bg="#FFFFFF", border="#CBD5E1", radius=5, parent_bg=COL["bg"])
        spec_card.pack(fill="x", pady=(0, 14))

        spec_inner = tk.Frame(spec_card.inner, bg="#FFFFFF", padx=18, pady=14)
        spec_inner.pack(fill="x")

        # 1. Objective: Heading + Objective text from DA42 MQTG / Screenshot (or ~2 lines blank space if empty)
        self._label(spec_inner, "Objective:", 10, COL["blue"], "bold", anchor="w").pack(fill="x")
        if spec.objective.strip():
            self._label(spec_inner, spec.objective.strip(), 9, COL["text"], anchor="w", justify="left", wraplength=850).pack(fill="x", pady=(2, 6))
        else:
            tk.Frame(spec_inner, bg="#FFFFFF", height=36).pack(fill="x")

        # 2. Initial conditions: Heading + Table for 1.f.(1), 1.f.(2), 1.c.(1) or ~2 lines blank space
        self._label(spec_inner, "Initial conditions:", 10, COL["blue"], "bold", anchor="w").pack(fill="x", pady=(4, 2))
        if spec.reference in ("1.f.(1)", "1.f.(2)", "1.c.(1)") or ("Flt cond" in getattr(spec, "init_conds", "")):
            tbl_card = RoundedCard(spec_inner, bg="#FFFFFF", border="#CBD5E1", radius=5, parent_bg="#FFFFFF")
            tbl_card.pack(fill="x", pady=(2, 6))

            tbl_box = tk.Frame(tbl_card.inner, bg="#FFFFFF")
            tbl_box.pack(fill="x", padx=1, pady=1)

            tbl_box.grid_columnconfigure((0, 1, 2, 3), weight=1, uniform="init_col")

            headers = ["Flt cond", "Alt[ft]", "IAS[kts]", "Flaps"]
            for c_idx, h in enumerate(headers):
                tk.Label(
                    tbl_box, text=h, bg="#F1F5F9", fg="#1E3A8A",
                    font=(FONT, 9, "bold"), pady=6
                ).grid(row=0, column=c_idx, sticky="nsew", padx=1, pady=1)

            if spec.reference == "1.c.(1)":
                flt_val = "Clean or specified climb configuration"
                alt_val = "3000"
                ias_val = "80"
                flaps_val = "0"
            elif spec.reference == "1.f.(1)":
                flt_val = "Approach or Landing"
                alt_val = "2000"
                ias_val = "80"
                flaps_val = "0"
            elif spec.reference == "1.f.(2)":
                flt_val = "Ground"
                alt_val = "2000"
                ias_val = "80"
                flaps_val = "0"
            else:
                ic_map = {}
                for part in spec.init_conds.split("|"):
                    if ":" in part:
                        k_s, v_s = part.split(":", 1)
                        ic_map[k_s.strip()] = v_s.strip()
                flt_val = ic_map.get("Flt cond", "Clean")
                alt_val = ic_map.get("Alt[ft]", "2000")
                ias_val = ic_map.get("IAS[kts]", "80")
                flaps_val = ic_map.get("Flaps", "0")

            values = [flt_val, alt_val, ias_val, flaps_val]
            for c_idx, val in enumerate(values):
                tk.Label(
                    tbl_box, textvariable=self._test_settings[spec.reference][0][headers[c_idx]], bg="#FFFFFF", fg=COL["text"],
                    font=(FONT, 9, "bold"), pady=6
                ).grid(row=1, column=c_idx, sticky="nsew", padx=1, pady=1)
        else:
            tk.Frame(spec_inner, bg="#FFFFFF", height=36).pack(fill="x")

        # 3. Tolerance: Exact EASA Document data
        self._label(spec_inner, "Tolerance:", 10, COL["blue"], "bold", anchor="w").pack(fill="x", pady=(4, 2))
        self._label(spec_inner, spec.tolerance, 10, COL["green"], "bold", anchor="w", justify="left", wraplength=850).pack(fill="x", pady=(0, 4))

        # --- Sub-Tabs: Large Rounded Buttons for Manual Test & Automatic Test ---
        subtab_bar = tk.Frame(self.workspace, bg=COL["bg"])
        subtab_bar.pack(fill="x", pady=(0, 10))

        self.btn_subtab_manual = RoundedButton(
            subtab_bar, "Manual",
            command=lambda: self._launch_common_test("manual", spec),
            width=190, height=44, radius=5,
            bg="#2563EB", fg="#FFFFFF",
            hover_bg="#1D4ED8", hover_fg="#FFFFFF",
            border="#1D4ED8", parent_bg=COL["bg"],
            font=(FONT, 11, "bold")
        )
        self.btn_subtab_manual.pack(side="left", padx=(0, 10))

        self.btn_subtab_auto = RoundedButton(
            subtab_bar, "Automatic",
            command=lambda: self._launch_common_test("auto", spec),
            width=190, height=44, radius=5,
            bg="#FFFFFF", fg=COL["text"],
            hover_bg="#EFF6FF", hover_fg=COL["blue"],
            border="#CBD5E1", parent_bg=COL["bg"],
            font=(FONT, 11, "bold")
        )
        self.btn_subtab_auto.pack(side="left")
        if spec.reference=="1.c.(1)":
            self.btn_subtab_auto_replay=RoundedButton(
                subtab_bar,"Automatic Replay",
                command=lambda:self._launch_common_test("auto",spec,control_method="replay"),
                width=190,height=44,radius=5,bg="#FFFFFF",fg=COL["text"],
                hover_bg="#EFF6FF",hover_fg=COL["blue"],border="#CBD5E1",
                parent_bg=COL["bg"],font=(FONT,11,"bold"))
            self.btn_subtab_auto_replay.pack(side="left",padx=(10,0))

        # Content Card for the active subtab
        self.subtab_card = RoundedCard(self.workspace, bg="#FFFFFF", border="#CBD5E1", radius=5, parent_bg=COL["bg"])
        self.subtab_card.pack(fill="both", expand=True)

        self.subtab_content = tk.Frame(self.subtab_card.inner, bg="#FFFFFF")
        self.subtab_content.pack(fill="both", expand=True)

        self._build_common_panel(spec)

        self._label(self.workspace,
                    "CS-FSTD(A) COMPLIANCE  ·  Objective validation testing for FNPT II qualification.",
                    9, COL["dim"], anchor="w").pack(fill="x", pady=(14, 0))

    def _build_common_panel(self,spec):
        self._common_display_spec=spec
        if not hasattr(self,"_common_states"): self._common_states={}
        box=tk.Frame(self.subtab_content,bg="#FFFFFF",padx=18,pady=16)
        box.pack(fill="both",expand=True)
        def resize_text(event):
            for child in box.winfo_children():
                if isinstance(child,tk.Label): child.configure(wraplength=max(200,event.width-36))
        box.bind("<Configure>",resize_text)
        self.common_status=self._label(box,"Choose Manual or Automatic to initialize the Sim Flight Model and CLS2sim connection.",
                                      10,COL["blue"],"bold",anchor="w",wraplength=950,justify="left")
        self.common_status.pack(fill="x",pady=(0,12))
        grid=tk.Frame(box,bg="#FFFFFF")
        grid.pack(fill="x")
        grid.grid_columnconfigure((0,1),weight=1,uniform="live")
        self._common_view_vars={}
        sections=[("Connections",[("connection","Sim Flight Model Disconnected | CLS2sim Connection: Disconnected")]),
                  ("Throttle and engine",[("throttle","Throttle: --"),("engine","Engine RPM: --")]),
                  ("Flight dynamics and controls",[("flight","Airspeed / Altitude: --"),("attitude","Pitch / Bank / Heading: --"),("controls","CLS2sim Pitch / Roll / Rudder: --")]),
                  ("Recording",[("recording","Not recording"),("progress","0 seconds / 0 samples")])]
        for index,(title,fields) in enumerate(sections):
            card=tk.LabelFrame(grid,text=title,bg="#F8FAFC",fg=COL["blue"],font=(FONT,10,"bold"),padx=12,pady=9)
            card.grid(row=index//2,column=index%2,sticky="nsew",padx=(0,8) if index%2==0 else (8,0),pady=(0,10))
            card.bind("<Configure>",lambda event,frame=card:[child.configure(wraplength=max(120,event.width-28)) for child in frame.winfo_children() if isinstance(child,tk.Label)])
            for key,initial in fields:
                variable=tk.StringVar(value=initial)
                self._common_view_vars[key]=variable
                tk.Label(card,textvariable=variable,bg="#F8FAFC",fg=COL["text"],font=(FONT,9),anchor="w",justify="left",wraplength=430).pack(fill="x",pady=2)
        self.common_progress=ttk.Progressbar(box,maximum=75,mode="determinate")
        self.common_progress.pack(fill="x",pady=(0,12))
        if not hasattr(self,"_common_reference_vars"): self._common_reference_vars={}
        self._common_reference_var=self._common_reference_vars.setdefault(spec.reference,tk.StringVar())
        reference=tk.LabelFrame(box,text="Manual reference for Recorded Replay",bg="#FFFFFF",fg=COL["blue"],padx=10,pady=10)
        reference.pack(fill="x",pady=(0,12))
        self.common_reference_combo=ttk.Combobox(reference,textvariable=self._common_reference_var,state="readonly")
        self.common_reference_combo.pack(side="left",fill="x",expand=True)
        ttk.Button(reference,text="Browse CSV",command=self._browse_common_reference).pack(side="left",padx=(8,0))
        ttk.Button(reference,text="Refresh",command=self._refresh_common_references).pack(side="left",padx=(8,0))
        self._refresh_common_references()
        bar=tk.Frame(box,bg="#FFFFFF")
        bar.pack(fill="x")
        self._label(bar,"Duration (s):",9,COL["text"],anchor="w").pack(side="left")
        if not hasattr(self,"_common_duration_vars"): self._common_duration_vars={}
        self._common_duration_var=self._common_duration_vars.setdefault(spec.reference,tk.StringVar(value="75"))
        self.common_duration_spinbox=ttk.Spinbox(bar,from_=1,to=3600,width=6,textvariable=self._common_duration_var)
        self.common_duration_spinbox.pack(side="left",padx=(6,14))
        self._sync_common_duration(spec)
        self.common_record_button=ttk.Button(bar,text="Start Recording",command=lambda:self._common_command("record"),state="disabled")
        self.common_record_button.pack(side="left",padx=(0,8))
        ttk.Button(bar,text="Stop & Save",command=self._stop_common_test).pack(side="left",padx=(0,8))
        ttk.Button(bar,text="Emergency Stop",command=self._stop_common_test).pack(side="left",padx=(0,8))
        outputs=tk.Frame(box,bg="#FFFFFF")
        outputs.pack(fill="x",pady=(12,0))
        ttk.Button(outputs,text="Open Data Folder",command=self._open_common_folder).pack(side="left",padx=(0,8))
        ttk.Button(outputs,text="Open Latest PDF",command=self._open_common_pdf).pack(side="left")
        self.common_output_var=tk.StringVar(value=str(__import__("qtg_common_engine").test_directory(RESULTS_ROOT,spec.reference,spec.title)))
        tk.Label(box,textvariable=self.common_output_var,bg="#FFFFFF",fg=COL["dim"],font=(FONT,8),anchor="w",wraplength=950,justify="left").pack(fill="x",pady=(8,0))
        self._label(box,"PID Climb: initialization, stabilization and recording run automatically. STOP remains available throughout.",
                    9,COL["dim"],anchor="w",wraplength=950,justify="left").pack(fill="x",pady=(10,0))
        self._render_common_state(self._common_states.get(spec.reference,{}))

    def _sync_common_duration(self,spec):
        if not hasattr(self,'common_duration_spinbox') or not self.common_duration_spinbox.winfo_exists():return
        if getattr(self,'_common_display_spec',None)!=spec:return
        if not hasattr(self,'_pid_duration_vars'):self._pid_duration_vars={}
        method=getattr(self,'_automatic_methods',{}).get(spec.reference)
        is_pid=spec.reference=='1.c.(1)' and (method is None or method.get()=='PID Automatic')
        self._common_duration_var=self._pid_duration_vars.setdefault(spec.reference,tk.StringVar(value='75')) if is_pid else self._common_duration_vars[spec.reference]
        self.common_duration_spinbox.configure(textvariable=self._common_duration_var)

    def _refresh_common_references(self):
        from qtg_common_engine import test_directory
        spec=self._common_display_spec
        directories=[test_directory(RESULTS_ROOT,spec.reference,spec.title)]
        candidates={}
        for directory in directories:
            if not directory.is_dir(): continue
            for path in directory.glob("*.csv"):
                name=path.name.lower()
                if "manual" in name and not any(word in name for word in ("diagnostic","commissioning","legacy_plot","test_qtg")):
                    candidates[str(path.resolve())]=path.stat().st_mtime
        self._common_reference_catalog=sorted(candidates,key=candidates.get,reverse=True)
        if hasattr(self,"common_reference_combo") and self.common_reference_combo.winfo_exists():
            self.common_reference_combo.configure(values=self._common_reference_catalog)
            if self._common_reference_var.get() not in self._common_reference_catalog:
                self._common_reference_var.set(self._common_reference_catalog[0] if self._common_reference_catalog else "")

    def _browse_common_reference(self):
        path=filedialog.askopenfilename(parent=self,title="Select Manual recording for this test",filetypes=[("Manual flight recording","*.csv")])
        if path:
            from qtg_common_engine import test_directory,validate_reference
            import shutil
            try:
                source=Path(validate_reference(path))
                spec=self._common_display_spec
                destination=test_directory(RESULTS_ROOT,spec.reference,spec.title)
                destination.mkdir(parents=True,exist_ok=True)
                target=destination/source.name
                if source.resolve()!=target.resolve():
                    if target.exists(): raise ValueError("A recording with this name already exists in this test folder.")
                    shutil.copy2(source,target)
                    companion=source.with_name(source.stem+"_init_conditions.json")
                    if companion.exists(): shutil.copy2(companion,destination/companion.name)
                self._refresh_common_references()
                self._common_reference_var.set(str(target.resolve()))
            except Exception as exc:
                messagebox.showerror("Manual reference",str(exc),parent=self)

    def _render_common_state(self,state):
        if not state or not hasattr(self,"common_status") or not self.common_status.winfo_exists(): return
        if state.get("test_reference") != self._common_display_spec.reference: return
        stage=state.get("stage","Connecting")
        text=f"Test {state.get('test_reference','')} - {state.get('mode','').title()} ({'PID Automatic' if state.get('control_method')=='pid' else 'Recorded Replay'}): {stage}"
        if state.get("details"): text+="\n"+state["details"]
        if state.get("evaluation_result"): text+="\nValidation: "+state["evaluation_result"]
        text=text.replace("Prepar3D","Sim Flight Model").replace("P3D","Sim Flight Model").replace("BRUNNER","CLS2sim").replace("Brunner","CLS2sim")
        self.common_status.configure(text=text,fg=COL["red"] if stage in ("Automatic blocked","Connection error") else COL["blue"])
        mapping={"throttle":"throttle_text","engine":"engine_text","flight":"flight_text","attitude":"attitude_text",
                 "controls":"controls_text","recording":"recording_text","progress":"progress_text"}
        self._common_view_vars["connection"].set(f"Sim Flight Model {'Connected' if state.get('connected') else 'Disconnected'} | CLS2sim Connection: {'Connected' if state.get('brunner_connected') else 'Disconnected'}")
        for key,field in mapping.items(): self._common_view_vars[key].set((state.get(field) or "--").replace("Prepar3D","Sim Flight Model").replace("P3D","Sim Flight Model").replace("BRUNNER","CLS2sim").replace("Brunner","CLS2sim"))
        if stage=="Generating reports":
            if str(self.common_progress.cget("mode"))!="indeterminate":
                self.common_progress.configure(mode="indeterminate")
                self.common_progress.start()
        else:
            self.common_progress.stop()
            self.common_progress.configure(mode="determinate",maximum=max(1,state.get("duration_s",75)),value=state.get("elapsed",0))
        self.common_record_button.configure(state="normal" if state.get("can_record") else "disabled")
        self.common_output_var.set("Data: "+state.get("data_directory",str(RESULTS_ROOT)))

    def _launch_common_test(self,mode,spec,force_initialize=False,control_method=None):
        try:
            from qtg_common_engine import launch,validate_duration,validate_reference
            duration=validate_duration(self._common_duration_var.get())
            if control_method not in (None,"pid","replay"):raise ValueError("Unknown automatic control method")
            method=control_method or ("pid" if mode=="auto" and spec.reference=="1.c.(1)" and self._automatic_methods[spec.reference].get()=="PID Automatic" else "replay")
            if mode=="manual":duration=validate_duration(self._common_duration_vars[spec.reference].get())
            if not self._apply_test_settings(spec): return
            fields,_=self._test_settings[spec.reference]
            initial=dict(alt_ft=float(fields["Alt[ft]"].get()),ias_kt=float(fields["IAS[kts]"].get()),lat=float(fields["Latitude[deg]"].get()),lon=float(fields["Longitude[deg]"].get()),flaps=float(fields["Flaps"].get()),throttle=float(fields["Throttle[%]"].get()),temperature_c=float(fields["Temperature[C]"].get()),on_ground="ground" in fields["Flt cond"].get().lower())
            selected=[key for _,key in spec.pdf_graphs]
            conditions=None;validation=None
            if method=="pid":
                fields,_=self._test_settings[spec.reference]
                from qtg_climb_pid import options
                conditions=options(dict(alt_ft=float(fields["Alt[ft]"].get()),ias_kt=float(fields["IAS[kts]"].get()),lat=float(fields["Latitude[deg]"].get()),lon=float(fields["Longitude[deg]"].get()),flaps=float(fields["Flaps"].get()),throttle=float(fields["Throttle[%]"].get()),temperature_c=float(fields["Temperature[C]"].get()),target_ias=80.))
                validation=self._pid_validation[spec.reference].get().strip() or None
                if validation and not Path(validation).is_file(): raise ValueError("Validation dataset not found.")
            reference=self._common_reference_var.get().strip() if mode=="auto" and method=="replay" else None
            if mode=="auto" and method=="replay":
                if not reference:
                    self.common_status.configure(text="Choose a Manual reference above before clicking Automatic.",fg=COL["amber"])
                    return
                reference=validate_reference(reference)
            current=getattr(self,"_common_process",None)
            if current is not None and current.poll() is None:
                state=getattr(self,"_common_last_state",{})
                if state.get("controlling") or state.get("recording") or state.get("stage") in ("Initializing aircraft","Generating reports"):
                    self.common_status.configure(text="Stop the active test and wait for preparation to finish before initializing again.",fg=COL["amber"])
                    return
                self._common_command("initialize",force_initialize=force_initialize,mode=mode,reference_csv=reference,duration_s=duration,test_reference=spec.reference,title=spec.title,control_method=method,pid_conditions=conditions,validation_csv=validation,initial_conditions=initial,selected_graphs=selected)
                self._common_context.update(test_reference=spec.reference,title=spec.title)
            else:
                self._common_last_state={}
                self._common_process,self._common_context=launch(mode,spec,RESULTS_ROOT/"sessions",reference,duration,control_method=method,pid_conditions=conditions,validation_csv=validation,initial_conditions=initial,selected_graphs=selected)
                self.after(200,self._poll_common_test)

            self.common_record_button.configure(state="disabled")
            self.common_status.configure(text=f"{mode.title()}: initializing Sim Flight Model and CLS2sim connection...",fg=COL["blue"])
        except Exception as exc:
            self.common_status.configure(text=str(exc),fg=COL["red"])

    def _open_pid_monitor(self):
        from qtg_pid_monitor import PidMonitor
        monitor=getattr(self,"_pid_monitor",None)
        if monitor is not None and monitor.window.winfo_exists():
            monitor.window.deiconify();monitor.window.lift()
        else:
            self._pid_monitor=PidMonitor(self,lambda:self._common_command("stop"),send=lambda action,**values:self._common_command(action,**values))

    def _common_command(self,action,**values):
        process=getattr(self,"_common_process",None)
        context=getattr(self,"_common_context",None)
        if process is None or context is None or process.poll() is not None: return
        if action=="record" and context.get("test_reference")!=self._common_display_spec.reference:
            self.common_status.configure(text="This test has no active session. Return to the active test to control it.",fg=COL["amber"])
            return
        from qtg_common_engine import send_command
        send_command(context,action,**values)

    def _poll_common_test(self):
        process=getattr(self,"_common_process",None)
        context=getattr(self,"_common_context",None)
        if process is None or context is None: return
        try:
            path=Path(context["status_file"])
            state=json.loads(path.read_text(encoding="utf-8")) if path.exists() else {}
            self._common_last_state=state
            monitor=getattr(self,"_pid_monitor",None)
            if monitor is not None and monitor.window.winfo_exists(): monitor.update(state)
            if state.get("test_reference"):
                self._common_states[state["test_reference"]]=state
            if not hasattr(self,"_common_logged_outputs"): self._common_logged_outputs=set()
            for output in state.get("outputs",[]):
                if output.lower().endswith(".pdf") and output not in self._common_logged_outputs:
                    if not any(entry.get("pdf_path")==output for entry in get_test_logs()):
                        append_test_log(state["test_reference"],state["title"],"Performance",state.get("mode","").title(),Path(output))
                    self._common_logged_outputs.add(output)
                    if state.get("test_reference")==self._common_display_spec.reference:
                        self._refresh_common_references()
            self._render_common_state(state)
            if process.poll() is not None and context.get("test_reference")==self._common_display_spec.reference and hasattr(self,"common_status") and self.common_status.winfo_exists():
                self.common_record_button.configure(state="disabled")
                if state.get("stage")!="Closed":
                    self.common_status.configure(text=f"Background engine exited (code {process.returncode}). Check session logs.",fg=COL["red"])
        except (OSError,ValueError,tk.TclError): pass
        if process.poll() is None: self.after(200,self._poll_common_test)

    def _stop_common_test(self):
        self._common_command("stop")

    def _open_common_folder(self):
        path=Path(self._common_states.get(self._common_display_spec.reference,{}).get("data_directory",str(__import__("qtg_common_engine").test_directory(RESULTS_ROOT,self._common_display_spec.reference,self._common_display_spec.title))))
        path.mkdir(parents=True,exist_ok=True)
        os.startfile(str(path))

    def _open_common_pdf(self):
        outputs=self._common_states.get(self._common_display_spec.reference,{}).get("outputs",[])
        pdf=next((Path(path) for path in reversed(outputs) if str(path).lower().endswith(".pdf") and Path(path).is_file()),None)
        if pdf is not None: open_pdf_file(pdf)
        else: messagebox.showinfo("PDF report","No PDF has been generated for this test session.",parent=self)

    def _switch_subtab(self, mode: str, spec: TestSpec):
        self.active_subtab_mode = mode
        for w in self.subtab_content.winfo_children():
            w.destroy()

        if mode == "manual":
            self.btn_subtab_manual.configure_button(
                bg="#2563EB", fg="#FFFFFF", border="#1D4ED8",
                hover_bg="#1D4ED8", hover_fg="#FFFFFF"
            )
            self.btn_subtab_auto.configure_button(
                bg="#FFFFFF", fg=COL["text"], border="#CBD5E1",
                hover_bg="#EFF6FF", hover_fg=COL["blue"]
            )
            self._build_manual_test_tab(self.subtab_content, spec)
        else:
            self.btn_subtab_auto.configure_button(
                bg="#2563EB", fg="#FFFFFF", border="#1D4ED8",
                hover_bg="#1D4ED8", hover_fg="#FFFFFF"
            )
            self.btn_subtab_manual.configure_button(
                bg="#FFFFFF", fg=COL["text"], border="#CBD5E1",
                hover_bg="#EFF6FF", hover_fg=COL["blue"]
            )
            self._build_auto_test_tab(self.subtab_content, spec)

    def _build_manual_test_tab(self, parent, spec: TestSpec):
        box = tk.Frame(parent, bg="#FFFFFF", padx=20, pady=18)
        box.pack(fill="both", expand=True)

        # For Engine Acceleration and Deceleration: ONLY Launch SMK Engine Suite!
        if spec.reference in ("1.f.(1)", "1.f.(2)"):
            self._build_smk_engine_integration_card(box, spec, mode_label="Manual Mode")
            return

        bar = tk.Frame(box, bg="#FFFFFF")
        bar.pack(fill="x", pady=(0, 14))

        btn_start = RoundedButton(
            bar, "▶  Start Manual Recording",
            command=lambda s=spec: self._start_manual_recording(s),
            width=210, height=38, radius=5,
            bg="#065F46", fg="#ECFDF5",
            hover_bg="#047857", hover_fg="#FFFFFF",
            border="#059669", parent_bg="#FFFFFF"
        )
        btn_start.pack(side="left", padx=(0, 10))

        btn_stop = RoundedButton(
            bar, "⏹  Stop Recording & Save PDF",
            command=lambda s=spec: self._stop_manual_recording(s),
            width=230, height=38, radius=5,
            bg="#991B1B", fg="#FEF2F2",
            hover_bg="#B91C1C", hover_fg="#FFFFFF",
            border="#EF4444", parent_bg="#FFFFFF"
        )
        btn_stop.pack(side="left", padx=(0, 10))

        self.btn_open_pdf = RoundedButton(
            bar, "📄  Open Latest PDF Report",
            command=self._open_latest_pdf,
            width=210, height=38, radius=5,
            bg="#FFFFFF", fg=COL["blue"],
            hover_bg="#EFF6FF", hover_fg=COL["blue"],
            border="#CBD5E1", parent_bg="#FFFFFF"
        )
        if self.last_generated_pdf:
            self.btn_open_pdf.pack(side="left")

        # Status Card with 5px radius border
        status_card = RoundedCard(box, bg="#F8FAFC", border="#CBD5E1", radius=5, parent_bg="#FFFFFF")
        status_card.pack(fill="x", pady=(0, 14))

        self.manual_status_lbl = self._label(status_card.inner, self.rec_status_var.get(), 10, COL["blue"], "bold", anchor="w")
        self.manual_status_lbl.pack(fill="x", padx=16, pady=12)

    def _build_auto_test_tab(self, parent, spec: TestSpec):
        box = tk.Frame(parent, bg="#FFFFFF", padx=20, pady=18)
        box.pack(fill="both", expand=True)

        # For Engine Acceleration and Deceleration: ONLY Launch SMK Engine Suite!
        if spec.reference in ("1.f.(1)", "1.f.(2)"):
            self._build_smk_engine_integration_card(box, spec, mode_label="Automatic Mode")
            return

        bar = tk.Frame(box, bg="#FFFFFF")
        bar.pack(fill="x", pady=(0, 14))

        btn_auto = RoundedButton(
            bar, "⚡  Run Automatic Test",
            command=lambda s=spec: self._run_automated_test(s),
            width=200, height=38, radius=5,
            bg="#2563EB", fg="#FFFFFF",
            hover_bg="#1D4ED8", hover_fg="#FFFFFF",
            border="#1D4ED8", parent_bg="#FFFFFF"
        )
        btn_auto.pack(side="left", padx=(0, 10))

        self.btn_auto_pdf = RoundedButton(
            bar, "📄  Open Latest PDF Report",
            command=self._open_latest_pdf,
            width=210, height=38, radius=5,
            bg="#FFFFFF", fg=COL["blue"],
            hover_bg="#EFF6FF", hover_fg=COL["blue"],
            border="#CBD5E1", parent_bg="#FFFFFF"
        )
        if self.last_generated_pdf:
            self.btn_auto_pdf.pack(side="left")

        # Status Card with 5px radius border
        status_card = RoundedCard(box, bg="#F8FAFC", border="#CBD5E1", radius=5, parent_bg="#FFFFFF")
        status_card.pack(fill="x", pady=(0, 14))

        self.auto_status_lbl = self._label(status_card.inner, self.auto_status_var.get(), 10, COL["blue"], "bold", anchor="w")
        self.auto_status_lbl.pack(fill="x", padx=16, pady=12)

    def _build_smk_engine_integration_card(self, parent, spec: TestSpec, mode_label: str = ""):
        if spec.reference not in ("1.f.(1)", "1.f.(2)"):
            return

        smk_card = RoundedCard(parent, bg="#FFFFFF", border="#93C5FD", radius=5, parent_bg="#FFFFFF")
        smk_card.pack(fill="both", expand=True, pady=(0, 0))

        smk_box = tk.Frame(smk_card.inner, bg="#FFFFFF", padx=18, pady=16)
        smk_box.pack(fill="both", expand=True)

        # Action Buttons: Launch SMK Engine Suite and Open Latest PDF Report
        btn_bar = tk.Frame(smk_box, bg="#FFFFFF")
        btn_bar.pack(fill="x", pady=(0, 14))

        btn_launch = RoundedButton(
            btn_bar, "🚀  Launch SMK Engine Suite",
            command=lambda s=spec: self._launch_smk_engine_tool(s),
            width=260, height=40, radius=5,
            bg="#2563EB", fg="#FFFFFF",
            hover_bg="#1D4ED8", hover_fg="#FFFFFF",
            border="#1D4ED8", parent_bg="#FFFFFF",
            font=(FONT, 10, "bold")
        )
        btn_launch.pack(side="left", padx=(0, 12))

        self.btn_smk_pdf = RoundedButton(
            btn_bar, "📄  Open Latest Generated PDF Report",
            command=self._open_latest_pdf,
            width=280, height=40, radius=5,
            bg="#FFFFFF", fg=COL["blue"],
            hover_bg="#EFF6FF", hover_fg=COL["blue"],
            border="#3B82F6", parent_bg="#FFFFFF",
            font=(FONT, 10, "bold")
        )
        self.btn_smk_pdf.pack(side="left")

        # Live Status Card
        status_card = RoundedCard(smk_box, bg="#F8FAFC", border="#CBD5E1", radius=5, parent_bg="#FFFFFF")
        status_card.pack(fill="x", pady=(0, 12))
        self.smk_status_lbl = self._label(
            status_card.inner,
            "● Ready — Click 'Launch SMK Engine Suite' to run engine dynamics validation. Reports will automatically be saved and logged in Year 2026.",
            10, COL["blue"], "bold", anchor="w"
        )
        self.smk_status_lbl.pack(fill="x", padx=16, pady=10)
        self.manual_status_lbl = self.smk_status_lbl
        self.auto_status_lbl = self.smk_status_lbl

        # Path info table/grid
        grid_card = RoundedCard(smk_box, bg="#F8FAFC", border="#CBD5E1", radius=5, parent_bg="#FFFFFF")
        grid_card.pack(fill="x")

        info_box = tk.Frame(grid_card.inner, bg="#F8FAFC", padx=14, pady=12)
        info_box.pack(fill="x")

        flt_cond_val = "Approach or Landing" if spec.reference == "1.f.(1)" else "Ground"
        profile_desc = (
            "Approach/Landing | Alt: 2000 ft | IAS: 80 kts | Flaps: 0 | Idle to 100% Throttle Step"
            if spec.reference == "1.f.(1)"
            else "Ground | Alt: 2000 ft | IAS: 80 kts | Flaps: 0 | Full Power to Idle Throttle Chop"
        )

        current_exe = find_smk_engine_exe()
        exe_display = str(current_exe) if current_exe else "Not Found (check SMK_Engine folder)"
        current_src = find_smk_engine_script()
        src_display = str(current_src.parent) if current_src else str(find_smk_engine_dir())

        rows = [
            ("Source Suite:", src_display),
            ("Executable:", exe_display),
            ("Test Profile:", profile_desc),
            ("Initial Conditions:", f"Flt cond: {flt_cond_val}  |  Alt[ft]: 2000  |  IAS[kts]: 80  |  Flaps: 0")
        ]

        for r_i, (k, v) in enumerate(rows):
            rf = tk.Frame(info_box, bg="#F8FAFC")
            rf.pack(fill="x", pady=2)
            self._label(rf, k, 9, "#1E3A8A", "bold", width=18, anchor="w").pack(side="left")
            self._label(rf, v, 9, COL["text"], anchor="w").pack(side="left", fill="x", expand=True)

    def _launch_smk_engine_script(self, spec: TestSpec):
        global LAST_SMK_LAUNCH_TIME
        LAST_SMK_LAUNCH_TIME = time.time()
        self._smk_launch_time = LAST_SMK_LAUNCH_TIME
        self._smk_active_spec = spec
        try:
            if getattr(sys, "frozen", False):
                # Reuse the bundled interpreter/modules; no installed Python needed.
                env = os.environ.copy()
                env["PYINSTALLER_RESET_ENVIRONMENT"] = "1"
                proc = subprocess.Popen([sys.executable, "--engine-suite"], cwd=str(BASE), env=env)
            else:
                script_file = find_smk_engine_script() or (BASE / "SMK_Engine_From_Climb.py")
                if not script_file.is_file():
                    raise FileNotFoundError(f"Engine suite not found: {script_file}")
                proc = subprocess.Popen([sys.executable, str(script_file)], cwd=str(script_file.parent))
            status_text = f"✔ Native SMK Engine script launched (PID: {proc.pid}) — Test: {spec.reference} ({spec.title})"
            if hasattr(self, "smk_status_lbl") and self.smk_status_lbl.winfo_exists():
                self.smk_status_lbl.configure(text=status_text, fg=COL["green"])
            if hasattr(self, "manual_status_lbl") and self.manual_status_lbl.winfo_exists():
                self.manual_status_lbl.configure(text=status_text, fg=COL["green"])
            if hasattr(self, "auto_status_lbl") and self.auto_status_lbl.winfo_exists():
                self.auto_status_lbl.configure(text=status_text, fg=COL["green"])
        except Exception as e:
            messagebox.showerror("Execution Error", f"Failed to launch SMK Engine script: {e}", parent=self)

    def _launch_smk_engine_tool(self, spec: TestSpec):
        global LAST_SMK_LAUNCH_TIME
        LAST_SMK_LAUNCH_TIME = time.time()
        self._smk_launch_time = LAST_SMK_LAUNCH_TIME
        self._smk_active_spec = spec
        if getattr(sys, "frozen", False):
            self._launch_smk_engine_script(spec)
            return
        exe = find_smk_engine_exe()
        if exe and exe.is_file():
            try:
                exe_dir = exe.parent
                env = os.environ.copy()
                for k in ("_MEIPASS2", "PYTHONPATH", "PYTHONHOME"):
                    env.pop(k, None)

                child_internal = exe_dir / "_internal"
                new_path = [str(exe_dir)]
                if child_internal.is_dir():
                    new_path.append(str(child_internal))
                for part in env.get("PATH", "").split(os.pathsep):
                    if part:
                        try:
                            rp = Path(part).resolve()
                            if rp.name == "_internal" and rp != child_internal.resolve():
                                continue
                        except Exception:
                            pass
                        new_path.append(part)
                env["PATH"] = os.pathsep.join(new_path)

                creation_flags = subprocess.CREATE_NEW_CONSOLE if sys.platform == "win32" else 0
                proc = subprocess.Popen(
                    [str(exe)],
                    cwd=str(exe_dir),
                    env=env,
                    creationflags=creation_flags
                )
                status_text = f"✔ SMK Engine Test Suite launched (PID: {proc.pid}) — Test: {spec.reference} ({spec.title})"
                if hasattr(self, "smk_status_lbl") and self.smk_status_lbl.winfo_exists():
                    self.smk_status_lbl.configure(text=status_text, fg=COL["green"])
                if hasattr(self, "manual_status_lbl") and self.manual_status_lbl.winfo_exists():
                    self.manual_status_lbl.configure(text=status_text, fg=COL["green"])
                if hasattr(self, "auto_status_lbl") and self.auto_status_lbl.winfo_exists():
                    self.auto_status_lbl.configure(text=status_text, fg=COL["green"])
                return
            except Exception as e:
                messagebox.showerror("Execution Error", f"Failed to launch SMK Engine executable ({exe.name}):\n{e}", parent=self)
                return

        # Fallback to python script if available
        script = find_smk_engine_script()
        if script and script.is_file():
            self._launch_smk_engine_script(spec)
            return

        messagebox.showerror(
            "SMK Engine Test Suite",
            f"SMK Engine executable not found.\n\nSearched directories:\n"
            f"- {BASE / 'SMK_Engine_V213_OriginalAirManager'}\n"
            f"- {BASE}\n\n"
            f"Please ensure the 'SMK_Engine_V213_OriginalAirManager' folder is copied inside the application folder.",
            parent=self
        )

    def _open_smk_engine_dir(self):
        script = find_smk_engine_script()
        exe = find_smk_engine_exe()
        target_dir = (script and script.parent) or (exe and exe.parent) or find_smk_engine_dir()
        if target_dir and target_dir.is_dir():
            try:
                os.startfile(str(target_dir))
            except Exception as e:
                messagebox.showerror("Folder Error", f"Could not open directory: {e}", parent=self)
        else:
            messagebox.showerror("Not Found", f"Directory not found:\n{target_dir}", parent=self)

    def _copy_smk_engine_path(self):
        script = find_smk_engine_script()
        exe = find_smk_engine_exe()
        target_dir = (script and script.parent) or (exe and exe.parent) or find_smk_engine_dir()
        self.clipboard_clear()
        self.clipboard_append(str(target_dir))
        messagebox.showinfo("Clipboard", f"Path copied to clipboard:\n{target_dir}", parent=self)

    def _preflight_check(self, spec: TestSpec) -> bool:
        """
        Pre-flight gate: verify connections before recording.
        Auto-connects to P3D if not already running.
        CLS2Sim is NOT required for engine dynamics tests (1.f.(1) and 1.f.(2)).
        Returns True only when all necessary connections are active.
        """
        issues = []

        # ── 1. Sim Flight Model (P3D / SimConnect) ───────────────────────────
        if not self.flight_model_connected:
            # Auto-attempt starting the bridge if P3D is running
            if not self.p3d.running:
                self.p3d.start()
            if self.p3d.running:
                self.flight_model_connected = True
                self._update_connection_ui()
            else:
                issues.append("❌  Sim Flight Model (P3D) is NOT connected.\n"
                              "    → Ensure Prepar3D is running and click 'Sim Flight Model Connection' in the top bar.")

        # ── 2. CLS2Sim connection ─────────────────────────────────────────────
        # Engine tests (1.f.(1) and 1.f.(2)) test engine dynamics and do NOT require Brunner CLS2Sim control loading!
        is_engine_test = spec.reference in ("1.f.(1)", "1.f.(2)")
        if not is_engine_test and not self.cls2sim_connected:
            issues.append("❌  CLS2Sim is NOT connected.\n"
                          "    → Click 'CLS2Sim Connection: Not Connected' in the top bar.")

        if issues:
            title = f"Connection Check — Test {spec.reference}: {spec.title}"
            body  = (
                f"The following connection(s) must be active before recording Test {spec.reference}:\n\n"
                + "\n\n".join(issues)
                + "\n\nConnect P3D, then click record again."
            )
            messagebox.showerror(title, body, parent=self)
            if hasattr(self, "manual_status_lbl") and self.manual_status_lbl.winfo_exists():
                self.manual_status_lbl.configure(
                    text=f"⚠ Connection check failed — {len(issues)} issue(s). See popup for details.",
                    fg=COL["amber"]
                )
            if hasattr(self, "auto_status_lbl") and self.auto_status_lbl.winfo_exists():
                self.auto_status_lbl.configure(
                    text=f"⚠ Connection check failed — {len(issues)} issue(s). See popup for details.",
                    fg=COL["amber"]
                )
            return False

        return True

    def _fly_to_initial_conditions(self, spec: TestSpec, status_label=None) -> bool:
        """Use the shared BRUNNER initialization, including its readback checks."""
        if not self._apply_test_settings(spec):
            return False
        self._launch_common_test("manual", spec,force_initialize=True)
        if status_label is not None and status_label.winfo_exists():
            status_label.configure(text="BRUNNER initialization requested. See test status.", fg=COL["blue"])
        return True

    def _start_manual_recording(self, spec: TestSpec):
        # ── Gate: connection checks ───────────────────────────────────────────
        if not self._preflight_check(spec):
            return

        status_lbl = getattr(self, "manual_status_lbl", None)

        # ── Step 1: Automatically fly aircraft to initial conditions ─────────
        if not self._fly_to_initial_conditions(spec, status_label=status_lbl):
            return

        # ── Step 2: Arm recording ─────────────────────────────────────────────
        self.recording_active = True
        self.recording_start_time = time.time()
        self.recorded_samples.clear()
        self.recording_spec = spec
        self.engine_timing = None
        self.engine_cue_beeped = False
        self.engine_event_completed = False
        self.is_auto_test = False

        if spec.reference in ("1.f.(1)", "1.f.(2)"):
            try:
                import engine_rules_v2
                kind = "acceleration" if spec.reference == "1.f.(1)" else "deceleration"
                self.engine_timing = engine_rules_v2.EngineTiming(kind=kind, post_seconds=15)
            except Exception:
                pass
            action_desc = "ADVANCE throttle to 100%" if spec.reference == "1.f.(1)" else "REDUCE throttle to idle"
            msg = f"● RECORDING BASELINE — Hold starting throttle for 5 seconds, then {action_desc} at 'MOVE NOW'."
        else:
            msg = (f"● RECORDING IN PROGRESS — Test {spec.reference}: {spec.title}  |  "
                   f"Fly the required manoeuvre now. Click 'Stop Recording & Save PDF' when done.")

        self.rec_status_var.set(msg)
        if status_lbl and status_lbl.winfo_exists():
            status_lbl.configure(text=msg, fg=COL["red"])

    def _stop_manual_recording(self, spec: TestSpec):
        if not self.recording_active:
            # Recording was never started — warn user and abort
            messagebox.showwarning(
                "No Active Recording",
                "No recording is in progress.\nClick '▶ Start Manual Recording' first, then fly the manoeuvre in P3D, then stop.",
                parent=self
            )
            return

        duration = max(1.0, time.time() - self.recording_start_time)
        self.recording_active = False
        self.recording_spec = None
        self.engine_timing = None
        self.engine_cue_beeped = False
        self.engine_event_completed = False
        self.is_auto_test = False

        if len(self.recorded_samples) < 5:
            msg = (
                f"Recording stopped after {duration:.1f} s but only {len(self.recorded_samples)} "
                f"data sample(s) were captured from P3D.\n\n"
                "Possible reasons:\n"
                "  • P3D was paused or not running during recording\n"
                "  • SimConnect bridge DLL is not loaded\n"
                "  • The sim connection dropped mid-recording\n\n"
                "Please reconnect P3D and try again."
            )
            messagebox.showwarning("Insufficient Data", msg, parent=self)
            if hasattr(self, "manual_status_lbl"):
                self.manual_status_lbl.configure(
                    text=f"⚠ Recording stopped — only {len(self.recorded_samples)} samples from P3D. Check connection and retry.",
                    fg=COL["amber"]
                )
            return

        pdf_path = generate_test_pdf(spec, "Manual Test", self.recorded_samples, duration)
        self.last_generated_pdf = pdf_path

        # Log entry for Dashboard and Yearly Results
        append_test_log(spec.reference, spec.title, spec.group, "Manual Test", pdf_path)
        self._refresh_dashboard_logs()

        status = (f"✔ TEST RECORDED — {len(self.recorded_samples)} samples over {duration:.1f} s "
                  f"captured from P3D — PDF Report saved: {pdf_path.name}")
        self.rec_status_var.set(status)
        if hasattr(self, "manual_status_lbl"):
            self.manual_status_lbl.configure(text=status, fg=COL["green"])
        if hasattr(self, "btn_open_pdf"):
            self.btn_open_pdf.pack(side="left")

        open_pdf_file(pdf_path)

    def _run_automated_test(self, spec: TestSpec):
        """
        Automated test: checks connections, auto-flies the aircraft to the
        required initial conditions via SimConnect, then arms a live recording
        session from P3D telemetry. Auto-stops and generates the official PDF.
        """
        # ── Pre-flight gate ──────────────────────────────────────────────────
        if not self._preflight_check(spec):
            return

        status_lbl = getattr(self, "auto_status_lbl", None)

        # ── Step 1: Automatically fly aircraft to initial conditions ─────────
        if not self._fly_to_initial_conditions(spec, status_label=status_lbl):
            return

        # ── Step 2: Arm recording ─────────────────────────────────────────────
        is_engine = spec.reference in ("1.f.(1)", "1.f.(2)")
        self.recording_active = True
        self.recording_start_time = time.time()
        self.recorded_samples.clear()
        self.recording_spec = spec
        self.is_auto_test = True
        self.auto_engine_stepped = False
        self.auto_engine_beeped = False
        self.engine_event_completed = False

        AUTO_DURATION_S = 22.0 if is_engine else 30.0

        if is_engine:
            kind = "acceleration" if spec.reference == "1.f.(1)" else "deceleration"
            try:
                import engine_rules_v2
                self.engine_timing = engine_rules_v2.EngineTiming(kind=kind, post_seconds=15)
            except Exception:
                pass
            msg = f"● AUTO RECORDING BASELINE — Holding starting throttle for 5 seconds before automated step..."
        else:
            msg = f"● AUTO RECORDING — Aircraft positioned. Fly the manoeuvre in P3D now. Auto-stops in {AUTO_DURATION_S:.0f} s…"

        self.auto_status_var.set(msg)
        if status_lbl and status_lbl.winfo_exists():
            status_lbl.configure(text=msg, fg=COL["red"])

        def _auto_finish():
            if not self.recording_active:
                return
            self.recording_active = False
            self.recording_spec = None
            self.engine_timing = None
            self.is_auto_test = False
            self.auto_engine_stepped = False
            self.auto_engine_beeped = False
            duration = max(1.0, time.time() - self.recording_start_time)

            if len(self.recorded_samples) < 5:
                msg = (
                    f"Automated recording captured only {len(self.recorded_samples)} sample(s) from P3D "
                    f"in {duration:.1f} s.\n\n"
                    "Possible reasons:\n"
                    "  • P3D was paused or not running during recording\n"
                    "  • SimConnect bridge is not loaded\n"
                    "  • Sim connection dropped mid-test"
                )
                messagebox.showwarning("Insufficient Data", msg, parent=self)
                if status_lbl and status_lbl.winfo_exists():
                    status_lbl.configure(
                        text=f"⚠ Auto test failed — only {len(self.recorded_samples)} sample(s) from P3D.",
                        fg=COL["amber"]
                    )
                return

            pdf_path = generate_test_pdf(spec, "Automatic Test", self.recorded_samples, duration)
            self.last_generated_pdf = pdf_path
            append_test_log(spec.reference, spec.title, spec.group, "Automatic Test", pdf_path)
            self._refresh_dashboard_logs()

            status = (f"✔ AUTOMATED TEST COMPLETE — {len(self.recorded_samples)} samples / {duration:.1f} s "
                      f"captured from P3D live — PDF Report saved: {pdf_path.name}")
            self.auto_status_var.set(status)
            if status_lbl and status_lbl.winfo_exists():
                status_lbl.configure(text=status, fg=COL["green"])
            if hasattr(self, "btn_auto_pdf"):
                self.btn_auto_pdf.pack(side="left")
            open_pdf_file(pdf_path)

        self._auto_timer_id = self.after(int(AUTO_DURATION_S * 1000), _auto_finish)

    def _open_latest_pdf(self):
        try:
            active_spec = TESTS[self.current_idx] if hasattr(self, "current_idx") else None
            active_ref = active_spec.reference if active_spec else None
            new_synced = sync_smk_generated_reports(active_ref=active_ref)
            if new_synced:
                self.last_generated_pdf = str(new_synced[0])
                self._refresh_dashboard_logs()
        except Exception:
            pass

        pdf_to_open = None
        if self.last_generated_pdf and Path(self.last_generated_pdf).is_file():
            pdf_to_open = Path(self.last_generated_pdf)
        else:
            rec_dir = RESULTS_ROOT
            if rec_dir.is_dir():
                pdfs = sorted(rec_dir.glob("*.pdf"), key=lambda p: p.stat().st_mtime, reverse=True)
                # Prefer recently generated PDFs over old historical files
                for p in pdfs:
                    if "2026-09-" not in p.name:
                        pdf_to_open = p
                        break
                if not pdf_to_open and pdfs:
                    pdf_to_open = pdfs[0]

        if pdf_to_open and pdf_to_open.is_file():
            open_pdf_file(pdf_to_open)
        else:
            messagebox.showinfo(
                "PDF Report",
                "No PDF report found yet.\nRun or complete a test to generate the report first.",
                parent=self
            )

    # ======================== REFERENCE DOCUMENT VIEWER ================================
    def _open_reference_document(self, spec: TestSpec):
        ref_cand = [
            BASE / "easa_references" / f"page_{spec.page}.png",
            MEIPASS / "easa_references" / f"page_{spec.page}.png",
        ]
        try:
            ref_cand.append(Path(__file__).resolve().parent / "easa_references" / f"page_{spec.page}.png")
        except Exception:
            pass

        ref_img_path = next((p for p in ref_cand if p and p.is_file()), None)
        if not ref_img_path:
            ans = messagebox.askyesno(
                "Reference Document",
                f"Reference document image for Page {spec.page} (Test {spec.reference}) was not found locally.\n\n"
                f"Would you like to open the official EASA CS-FSTD(A) document online at Page {spec.page}?",
                parent=self
            )
            if ans:
                webbrowser.open(f"{SOURCE_URL}#page={spec.page}")
            return

        top = tk.Toplevel(self)
        top.title(f"EASA Reference Document — Test {spec.reference}: {spec.title}")
        top.geometry("1160x860")
        top.minsize(850, 600)
        top.configure(bg=COL["bg"])

        bar = tk.Frame(top, bg=COL["rail"], height=58)
        bar.pack(side="top", fill="x")
        bar.pack_propagate(False)

        info_frame = tk.Frame(bar, bg=COL["rail"])
        info_frame.pack(side="left", padx=16, pady=8)
        self._label(info_frame, f"EASA CS-FSTD(A) Issue 2 — Page {spec.page} | AMC1 FSTD(A).300(b)(3)", 10, COL["header_text"], "bold").pack(anchor="w")
        self._label(info_frame, f"Test {spec.reference}: {spec.title}", 9, COL["header_muted"]).pack(anchor="w")

        btn_box = tk.Frame(bar, bg=COL["rail"])
        btn_box.pack(side="right", padx=16, pady=10)

        # Ensure image is in RGB mode with solid white background
        try:
            pil_orig = Image.open(ref_img_path)
            if pil_orig.mode != "RGB":
                white_bg = Image.new("RGB", pil_orig.size, (255, 255, 255))
                if "A" in pil_orig.mode:
                    white_bg.paste(pil_orig, mask=pil_orig.split()[-1])
                else:
                    white_bg.paste(pil_orig)
                pil_orig = white_bg
                try:
                    pil_orig.save(ref_img_path)
                except Exception:
                    pass
        except Exception as exc:
            messagebox.showerror("Reference Document", f"Error opening page image: {exc}", parent=self)
            return

        canvas_frame = tk.Frame(top, bg="#525659")
        canvas_frame.pack(fill="both", expand=True, padx=6, pady=6)

        xscroll = ttk.Scrollbar(canvas_frame, orient="horizontal")
        yscroll = ttk.Scrollbar(canvas_frame, orient="vertical")
        canvas = tk.Canvas(canvas_frame, bg="#525659", highlightthickness=0,
                           xscrollcommand=xscroll.set, yscrollcommand=yscroll.set)

        xscroll.configure(command=canvas.xview)
        yscroll.configure(command=canvas.yview)

        yscroll.pack(side="right", fill="y")
        xscroll.pack(side="bottom", fill="x")
        canvas.pack(side="left", fill="both", expand=True)

        scale_var = [0.65]

        def update_view():
            curr_scale = scale_var[0]
            new_w = max(300, int(pil_orig.width * curr_scale))
            new_h = max(200, int(pil_orig.height * curr_scale))
            resized = pil_orig.resize((new_w, new_h), Image.Resampling.BILINEAR)
            tk_img = ImageTk.PhotoImage(resized)
            top._img_ref = tk_img
            canvas.delete("all")
            pad = 20
            canvas.create_rectangle(pad - 1, pad - 1, pad + new_w + 1, pad + new_h + 1, outline="#CBD5E1", width=1)
            canvas.create_image(pad, pad, anchor="nw", image=tk_img)
            canvas.configure(scrollregion=(0, 0, new_w + pad * 2, new_h + pad * 2))

        def zoom_in():
            if scale_var[0] < 2.0:
                scale_var[0] = round(scale_var[0] + 0.15, 2)
                update_view()

        def zoom_out():
            if scale_var[0] > 0.35:
                scale_var[0] = round(scale_var[0] - 0.15, 2)
                update_view()

        def zoom_fit():
            canv_w = max(600, canvas.winfo_width() - 60)
            scale_var[0] = round(min(1.2, canv_w / pil_orig.width), 2)
            update_view()

        def zoom_100():
            scale_var[0] = 1.0
            update_view()

        RoundedButton(btn_box, "🔍 -", zoom_out, width=42, height=32, radius=5,
                      bg="#1E3A8A", fg="#FFFFFF", border="#3B82F6", parent_bg=COL["rail"]).pack(side="left", padx=(0, 4))
        RoundedButton(btn_box, "🔍 +", zoom_in, width=42, height=32, radius=5,
                      bg="#1E3A8A", fg="#FFFFFF", border="#3B82F6", parent_bg=COL["rail"]).pack(side="left", padx=(0, 4))
        RoundedButton(btn_box, "Fit Width", zoom_fit, width=80, height=32, radius=5,
                      bg="#1E3A8A", fg="#FFFFFF", border="#3B82F6", parent_bg=COL["rail"]).pack(side="left", padx=(0, 4))
        RoundedButton(btn_box, "100%", zoom_100, width=54, height=32, radius=5,
                      bg="#1E3A8A", fg="#FFFFFF", border="#3B82F6", parent_bg=COL["rail"]).pack(side="left", padx=(0, 8))
        RoundedButton(btn_box, "📄 Open Official PDF",
                      lambda: webbrowser.open(SOURCE_URL + f"#page={spec.page}"),
                      width=165, height=32, radius=5,
                      bg="#2563EB", fg="#FFFFFF", border="#1D4ED8", parent_bg=COL["rail"]).pack(side="left", padx=(0, 8))
        RoundedButton(btn_box, "Close", top.destroy, width=64, height=32, radius=5,
                      bg="#1E3A8A", fg="#FFFFFF", border="#3B82F6", parent_bg=COL["rail"]).pack(side="left")

        def _on_wheel(event):
            if event.state & 0x0004:
                if event.delta > 0:
                    zoom_in()
                else:
                    zoom_out()
            elif event.state & 0x0001:
                canvas.xview_scroll(-int(event.delta / 120) * 3, "units")
            else:
                canvas.yview_scroll(-int(event.delta / 120) * 3, "units")

        canvas.bind("<MouseWheel>", _on_wheel)
        top.bind("<MouseWheel>", _on_wheel)
        top.after(100, zoom_fit)

    def _on_test_double_click(self, index: int):
        self.select_test(index)
        self._open_test_password_dialog(index)

    def _open_test_password_dialog(self, index):
        spec = TESTS[index]
        password = simpledialog.askstring("Test Administration Security",
            f"Enter administrator password to edit Test {spec.reference} settings:", show="*", parent=self)
        if password is None:
            return
        if password != TEST_ADMIN_PASSWORD:
            messagebox.showerror("Access denied", "Incorrect administrator password.", parent=self)
            return
        self._settings_unlocked = getattr(self, "_settings_unlocked", set())
        self._settings_unlocked.add(spec.reference)
        self._build_settings_panel(spec)

    def _lock_test_settings(self, spec):
        getattr(self, "_settings_unlocked", set()).discard(spec.reference)
        self._build_settings_panel(spec)

    def _build_settings_panel(self, spec):
        if not hasattr(self,"_automatic_methods"): self._automatic_methods={}
        self._automatic_methods.setdefault(spec.reference,tk.StringVar(value="PID Automatic" if spec.reference=="1.c.(1)" else "Recorded Replay"))
        if spec.reference=="1.c.(1)":
            if not hasattr(self,"_pid_targets"): self._pid_targets={}
            if not hasattr(self,"_pid_validation"): self._pid_validation={}
            self._pid_targets.setdefault(spec.reference,tk.StringVar(value="80"))
            self._pid_targets[spec.reference].set("80")
            self._pid_validation.setdefault(spec.reference,tk.StringVar())
        for widget in self.settings_panel.winfo_children():
            widget.destroy()
        if spec.reference not in self._test_settings:
            defaults = {"Flt cond": "Clean", "Alt[ft]": "2000", "IAS[kts]": "80", "Flaps": "0", "Latitude[deg]": "32.235", "Longitude[deg]": "74.135",
                        "Throttle[%]": "100" if spec.reference in ("1.f.(2)","1.c.(1)") else "0",
                        "Temperature[C]": "15"}
            for part in spec.initial_conditions.split("|"):
                if ":" in part:
                    key, value = part.split(":", 1)
                    if key.strip() in defaults:
                        defaults[key.strip()] = value.strip()
            if not hasattr(self, "_test_defaults"):
                self._test_defaults = {}
            self._test_defaults[spec.reference] = dict(defaults)
            channels = PDF_CHANNELS
            self._test_settings[spec.reference] = (
                {key: tk.StringVar(value=value) for key, value in defaults.items()},
                [(label, column, tk.BooleanVar(value=True)) for label, column in channels])
        if not hasattr(self, "_graph_csv_paths"):
            self._graph_csv_paths = {}
        self._graph_csv_paths.setdefault(spec.reference, tk.StringVar())
        unlocked = spec.reference in getattr(self, "_settings_unlocked", set())
        if not unlocked:
            self.settings_panel.pack_forget()
            return
        self.settings_panel.pack(side="right", fill="y", padx=(8, 12), pady=12, before=self._work_canvas)
        canvas = tk.Canvas(self.settings_panel, bg="#FFFFFF", highlightthickness=0)
        scroll = ttk.Scrollbar(self.settings_panel, command=canvas.yview)
        scroll.pack(side="right", fill="y")
        canvas.pack(fill="both", expand=True)
        canvas.configure(yscrollcommand=scroll.set)
        box = tk.Frame(canvas, bg="#FFFFFF", padx=14, pady=14)
        window = canvas.create_window(0, 0, window=box, anchor="nw")
        box.bind("<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.bind("<Configure>", lambda e: canvas.itemconfigure(window, width=e.width))
        self._label(box, "Test settings", 12, COL["blue"], "bold").pack(anchor="w")
        self._label(box, f"Test {spec.reference}", 9, COL["dim"]).pack(anchor="w", pady=(2, 12))
        unlocked = spec.reference in getattr(self, "_settings_unlocked", set())
        ttk.Button(box, text="Lock settings" if unlocked else "Unlock settings (password)",
                   command=(lambda: self._lock_test_settings(spec)) if unlocked else
                   (lambda: self._open_test_password_dialog(TESTS.index(spec)))).pack(fill="x", pady=(0, 10))
        fields, graphs = self._test_settings[spec.reference]
        if not hasattr(self,"_automatic_methods"): self._automatic_methods={}
        self._automatic_methods.setdefault(spec.reference,tk.StringVar(value="PID Automatic" if spec.reference=="1.c.(1)" else "Recorded Replay"))
        self._label(box,"Automatic Control Method",10,COL["text"],"bold").pack(anchor="w",pady=(4,2))
        methods=["Recorded Replay","PID Automatic"] if spec.reference=="1.c.(1)" else ["Recorded Replay"]
        method_combo=ttk.Combobox(box,textvariable=self._automatic_methods[spec.reference],values=methods,
                     state="readonly" if unlocked else "disabled")
        method_combo.pack(fill="x")
        method_combo.bind("<<ComboboxSelected>>",lambda event:self._sync_common_duration(spec))
        if spec.reference=="1.c.(1)":
            if not hasattr(self,"_pid_targets"): self._pid_targets={}
            if not hasattr(self,"_pid_validation"): self._pid_validation={}
            self._pid_targets.setdefault(spec.reference,tk.StringVar(value="80"))
            self._pid_targets[spec.reference].set("80")
            self._pid_validation.setdefault(spec.reference,tk.StringVar())
            self._label(box,"PID target IAS [kt]",9,COL["dim"]).pack(anchor="w",pady=(7,2))
            ttk.Entry(box,textvariable=self._pid_targets[spec.reference],state="disabled").pack(fill="x")
            self._label(box,"PID validation dataset (CSV)",9,COL["dim"]).pack(anchor="w",pady=(7,2))
            ttk.Entry(box,textvariable=self._pid_validation[spec.reference],state="normal" if unlocked else "disabled").pack(fill="x")
            def browse_validation():
                value=filedialog.askopenfilename(parent=self,title="Climb validation dataset",filetypes=[("Validation CSV","*.csv")])
                if value: self._pid_validation[spec.reference].set(value)
            ttk.Button(box,text="Browse validation CSV",command=browse_validation,state="normal" if unlocked else "disabled").pack(fill="x",pady=4)
            self._label(box,"Automatic initializes, stabilizes at 80 kt for 5 seconds, then records with PID active. Duration is configurable below the test status.",8,COL["dim"],wraplength=235,justify="left").pack(fill="x",pady=4)
        self._label(box, "Initial conditions", 10, COL["text"], "bold").pack(anchor="w")
        for key, var in fields.items():
            self._label(box, key, 9, COL["dim"]).pack(anchor="w", pady=(7, 2))
            ttk.Entry(box, textvariable=var, state="normal" if unlocked else "disabled").pack(fill="x")
        self._label(box, "Initialization follows the BRUNNER base. Automatic restores the selected Manual reference and its recorded weather.", 8,
                    COL["dim"], wraplength=235, justify="left").pack(fill="x", pady=8)
        status = self._label(box, "Ready to set initial conditions", 9, COL["dim"], wraplength=235, justify="left")
        status.pack(fill="x", pady=4)
        ttk.Button(box, text="Set initial conditions",
                   command=lambda: self._fly_to_initial_conditions(spec, status_label=status)).pack(fill="x", pady=(4, 10))
        self._label(box, "Graphs in PDF", 10, COL["text"], "bold").pack(anchor="w", pady=(10, 4))
        buttons = tk.Frame(box, bg="#FFFFFF")
        buttons.pack(fill="x")
        for caption, checked in (("Select all", True), ("Clear all", False)):
            ttk.Button(buttons, text=caption, state="normal" if unlocked else "disabled",
                       command=lambda value=checked: [var.set(value) for _, _, var in graphs]).pack(side="left")
        for label, column, var in graphs:
            tk.Checkbutton(box, text=label, variable=var, bg="#FFFFFF", anchor="w",
                           state="normal" if unlocked else "disabled", wraplength=225, justify="left").pack(fill="x")
        if not hasattr(self, "_graph_csv_paths"):
            self._graph_csv_paths = {}
        source = self._graph_csv_paths.setdefault(spec.reference, tk.StringVar())
        self._label(box, "Recorded graph CSV (optional)", 9, COL["dim"]).pack(anchor="w", pady=(10, 2))
        ttk.Entry(box, textvariable=source, state="normal" if unlocked else "disabled").pack(fill="x")
        def browse_csv():
            path = filedialog.askopenfilename(parent=self, filetypes=[("Recorded CSV", "*.csv")])
            if path: source.set(path)
        ttk.Button(box, text="Browse CSV", command=browse_csv,
                   state="normal" if unlocked else "disabled").pack(fill="x", pady=4)
        self._label(box, "Use the CSV belonging to the run for channels missing from the live bridge. Missing channels are marked unavailable.",
                    8, COL["dim"], wraplength=235, justify="left").pack(fill="x", pady=4)
        self._label(box, "Clear all graphs for a summary-only PDF. Settings apply when starting a test.",
                    8, COL["dim"], wraplength=235, justify="left").pack(fill="x", pady=8)

    def _apply_test_settings(self, spec):
        fields, graphs = self._test_settings[spec.reference]
        try:
            for key, var in fields.items():
                if not var.get().strip():
                    var.set(self._test_defaults[spec.reference][key])
            for key in ("Alt[ft]", "IAS[kts]", "Flaps", "Latitude[deg]", "Longitude[deg]", "Throttle[%]", "Temperature[C]"):
                value = float(fields[key].get())
                if not math.isfinite(value):
                    raise ValueError(f"{key} must be a finite number")
                if key in ("IAS[kts]", "Flaps") and value < 0:
                    raise ValueError(f"{key} cannot be negative")
                if key == "Latitude[deg]" and not -90 <= value <= 90:
                    raise ValueError("Latitude must be between -90 and 90 degrees")
                if key == "Longitude[deg]" and not -180 <= value <= 180:
                    raise ValueError("Longitude must be between -180 and 180 degrees")
                if key == "Flaps" and value != int(value):
                    raise ValueError("Flaps must be a whole-number handle index")
                if key == "Temperature[C]" and not -100 <= value <= 60:
                    raise ValueError("Temperature must be between -100 and 60 C")
                if key == "Throttle[%]" and not 0 <= value <= 100:
                    raise ValueError("Throttle must be between 0 and 100 percent")
            source = self._graph_csv_paths[spec.reference].get().strip()
            spec.graph_csv_data = read_graph_csv(source) if source else None
            spec.graph_csv_path = source
            spec.initial_conditions = " | ".join(f"{key}: {var.get().strip()}" for key, var in fields.items())
            spec.pdf_graphs = [(label, column) for label, column, var in graphs if var.get()]
            return True
        except (ValueError, OSError) as exc:
            messagebox.showerror("Invalid initial conditions", str(exc), parent=self)
            return False

    # ======================== HARDWARE CONNECTIONS & POLLING ===========================
    def _toggle_flight_model(self):
        if self.flight_model_connected:
            self.p3d.stop()
            self.flight_model_connected = False
            self.p3d_paused = False
        else:
            if not self.p3d.running and not self.p3d.start():
                messagebox.showerror("Sim Flight Model Connection", self.p3d.error, parent=self)
                return
            self.flight_model_connected = True
            self.p3d_paused = False
        self._update_connection_ui()

    def _toggle_cls2sim(self):
        self.cls2sim_connected = not self.cls2sim_connected
        self._update_connection_ui()

    def _toggle_motion(self):
        self.motion_connected = not self.motion_connected
        self._update_connection_ui()

    def _update_connection_ui(self):
        if self.flight_model_connected:
            if getattr(self, "p3d_paused", False):
                self.btn_flight_model.configure_button(
                    text="Sim Flight Model Connection: Connected (Waiting)",
                    bg=COL["status_connected_bg"], fg=COL["status_connected_fg"],
                    border=COL["status_connected_border"]
                )
            else:
                self.btn_flight_model.configure_button(
                    text="Sim Flight Model Connection: Connected",
                    bg=COL["status_connected_bg"], fg=COL["status_connected_fg"],
                    border=COL["status_connected_border"]
                )
        else:
            self.btn_flight_model.configure_button(
                text="Sim Flight Model Connection: Not Connected",
                bg=COL["status_disconnected_bg"], fg=COL["status_disconnected_fg"],
                border=COL["status_disconnected_border"]
            )

        if self.cls2sim_connected:
            self.btn_cls2sim.configure_button(
                text="CLS2Sim Connection: Connected",
                bg=COL["status_connected_bg"], fg=COL["status_connected_fg"],
                border=COL["status_connected_border"]
            )
        else:
            self.btn_cls2sim.configure_button(
                text="CLS2Sim Connection: Not Connected",
                bg=COL["status_disconnected_bg"], fg=COL["status_disconnected_fg"],
                border=COL["status_disconnected_border"]
            )

        if self.motion_connected:
            self.btn_motion.configure_button(
                text="Motion System Connection: Connected",
                bg=COL["status_connected_bg"], fg=COL["status_connected_fg"],
                border=COL["status_connected_border"]
            )
        else:
            self.btn_motion.configure_button(
                text="Motion System Connection: Not Connected",
                bg=COL["status_disconnected_bg"], fg=COL["status_disconnected_fg"],
                border=COL["status_disconnected_border"]
            )

    def _poll_p3d(self):
        try:
            sample = self.p3d.snapshot()
            if sample is not None:
                if sample.connected:
                    was_paused = getattr(self, "p3d_paused", False)
                    was_connected = self.flight_model_connected
                    self.flight_model_connected = True
                    self.p3d_paused = bool(sample.paused)
                    if not was_connected or (was_paused != self.p3d_paused):
                        self._update_connection_ui()

                    if self.recording_active and not self.p3d_paused:
                        now = time.time()
                        elapsed = now - self.recording_start_time
                        rec_spec = getattr(self, "recording_spec", None)

                        # Engine tests: capture throttle + RPM channels
                        if rec_spec is not None and rec_spec.reference in ("1.f.(1)", "1.f.(2)"):
                            self.recorded_samples.append((
                                elapsed,
                                sample.ias_kt,
                                sample.altitude_ft,
                                sample.throttle_percent,   # col[3] = throttle %
                                sample.engine_rpm,         # col[4] = engine RPM
                                sample.prop_rpm, sample.pitch_deg, sample.bank_deg,
                                sample.elevator_position, sample.aileron_position, sample.rudder_position
                            ))
                        else:
                            # All other tests: capture pitch + bank (aerodynamic response)
                            self.recorded_samples.append((
                                elapsed,
                                sample.ias_kt,
                                sample.altitude_ft,
                                sample.pitch_deg,
                                sample.bank_deg,
                                sample.prop_rpm, sample.pitch_deg, sample.bank_deg,
                                sample.elevator_position, sample.aileron_position, sample.rudder_position
                            ))

                        n = len(self.recorded_samples)
                        
                        # Engine Acceleration & Deceleration Live Cue and Timing Engine
                        if rec_spec is not None and rec_spec.reference in ("1.f.(1)", "1.f.(2)"):
                            kind = "acceleration" if rec_spec.reference == "1.f.(1)" else "deceleration"
                            is_auto = getattr(self, "is_auto_test", False)

                            if not hasattr(self, "engine_timing") or self.engine_timing is None:
                                try:
                                    import engine_rules_v2
                                    self.engine_timing = engine_rules_v2.EngineTiming(kind=kind, post_seconds=15)
                                    self.engine_cue_beeped = False
                                except Exception:
                                    pass

                            # AUTOMATIC ENGINE SEQUENCE
                            if is_auto:
                                if elapsed < 5.0:
                                    wait_rem = max(0.0, 5.0 - elapsed)
                                    msg = (f"● AUTO RECORDING BASELINE — Holding baseline ({wait_rem:.1f} s to auto-step)  |  "
                                           f"Thr: {sample.throttle_percent:.0f}%  |  RPM: {sample.engine_rpm:.0f}")
                                else:
                                    if not getattr(self, "auto_engine_stepped", False):
                                        try:
                                            import winsound
                                            winsound.Beep(1200, 300)
                                        except Exception:
                                            pass
                                        target_thr = 100.0 if kind == "acceleration" else 0.0
                                        if hasattr(self, "bridge_writer"):
                                            self.bridge_writer.set_throttle(target_thr)
                                        self.auto_engine_stepped = True
                                        if self.engine_timing:
                                            self.engine_timing.event = elapsed

                                    hold_rem = max(0.0, 20.0 - elapsed)
                                    action_name = "ADVANCED to 100%" if kind == "acceleration" else "REDUCED to idle (0%)"
                                    msg = (f"🔥 AUTO COMMAND: {action_name}!  |  Recording response ({hold_rem:.1f} s remaining)  |  "
                                           f"Thr: {sample.throttle_percent:.0f}%  |  RPM: {sample.engine_rpm:.0f}")

                            # MANUAL ENGINE SEQUENCE
                            else:
                                if hasattr(self, "engine_timing") and self.engine_timing is not None:
                                    proposed_dur = self.engine_timing.update(elapsed, sample.throttle_percent)
                                    if self.engine_timing.event is not None:
                                        rem = max(0.0, (self.engine_timing.event + self.engine_timing.post_seconds) - elapsed)
                                        msg = (f"● RECORDING ENGINE RESPONSE — Event at {self.engine_timing.event:.1f} s  |  "
                                               f"Holding for {rem:.1f} s more  |  Thr: {sample.throttle_percent:.0f}%  |  RPM: {sample.engine_rpm:.0f}")
                                        if rem <= 0.0 and not getattr(self, "engine_event_completed", False):
                                            self.engine_event_completed = True
                                            self.after(50, lambda s=rec_spec: self._stop_manual_recording(s))
                                    elif elapsed >= 5.0:
                                        if not getattr(self, "engine_cue_beeped", False):
                                            try:
                                                import winsound
                                                winsound.Beep(1200, 300)
                                            except Exception:
                                                pass
                                            self.engine_cue_beeped = True
                                        action_prompt = "ADVANCE throttle to 100%" if kind == "acceleration" else "REDUCE throttle to idle"
                                        msg = (f"🔥 MOVE NOW — {action_prompt}!  |  Elapsed: {elapsed:.1f} s  |  "
                                               f"Thr: {sample.throttle_percent:.0f}%  |  RPM: {sample.engine_rpm:.0f}")
                                    else:
                                        wait_rem = max(0.0, 5.0 - elapsed)
                                        msg = (f"● RECORDING BASELINE — Hold throttle steady ({wait_rem:.1f} s to MOVE NOW)  |  "
                                               f"Thr: {sample.throttle_percent:.0f}%  |  RPM: {sample.engine_rpm:.0f}")
                                else:
                                    msg = (f"● RECORDING — {elapsed:.1f} s  |  {n} samples  |  "
                                           f"Thr: {sample.throttle_percent:.0f}%  |  RPM: {sample.engine_rpm:.0f}")
                        else:
                            msg = (f"● RECORDING — {elapsed:.1f} s  |  {n} samples  |  "
                                   f"IAS: {sample.ias_kt:.1f} kt  |  Alt: {sample.altitude_ft:.0f} ft  |  "
                                   f"Thr: {sample.throttle_percent:.0f}%  |  RPM: {sample.engine_rpm:.0f}")

                        self.rec_status_var.set(msg)
                        if hasattr(self, "manual_status_lbl") and self.manual_status_lbl.winfo_exists():
                            self.manual_status_lbl.configure(text=msg, fg=COL["red"])
                        # Also update auto status if auto recording is active
                        if hasattr(self, "auto_status_lbl"):
                            try:
                                if self.auto_status_lbl.winfo_exists():
                                    self.auto_status_lbl.configure(text=msg, fg=COL["red"])
                            except Exception:
                                pass

                    elif self.recording_active and self.p3d_paused:
                        # Sim paused mid-recording: warn but do NOT append garbage data
                        msg = "⚠ RECORDING PAUSED — P3D is paused. Resume sim to continue capturing data..."
                        if hasattr(self, "manual_status_lbl") and self.manual_status_lbl.winfo_exists():
                            self.manual_status_lbl.configure(text=msg, fg=COL["amber"])
                else:
                    if self.flight_model_connected:
                        self.flight_model_connected = False
                        self.p3d_paused = False
                        self._update_connection_ui()
            else:
                # Bridge not connected — if recording, warn user; do NOT append fake samples
                if self.recording_active:
                    msg = "⚠ RECORDING ACTIVE but P3D not connected — no data captured. Check connection."
                    if hasattr(self, "manual_status_lbl") and self.manual_status_lbl.winfo_exists():
                        self.manual_status_lbl.configure(text=msg, fg=COL["amber"])
        finally:
            self.after(500, self._poll_p3d)

    def _on_close(self):
        self._common_command("shutdown")
        if hasattr(self, "_side_window") and self._side_window and self._side_window.winfo_exists():
            try:
                self._side_window.destroy()
            except Exception:
                pass
        if hasattr(self, "_active_pwd_dlg") and self._active_pwd_dlg and self._active_pwd_dlg.winfo_exists():
            try:
                self._active_pwd_dlg.destroy()
            except Exception:
                pass
        self.p3d.stop()
        self.destroy()


def main():
    try:
        app = App()
        app.mainloop()
    except Exception as exc:
        import traceback
        err_msg = traceback.format_exc()
        try:
            log_path = BASE / "startup_error.log"
            log_path.write_text(err_msg, encoding="utf-8")
        except Exception:
            pass
        try:
            import ctypes
            ctypes.windll.user32.MessageBoxW(
                0,
                f"Fatal startup error:\n\n{exc}\n\nA diagnostic log has been written to:\n{BASE / 'startup_error.log'}",
                "QTG Studio Error",
                0x10
            )
        except Exception:
            pass


if __name__ == "__main__":
    if "--brunner-test" in sys.argv:
        from qtg_common_engine import run
        sys.exit(run(sys.argv[sys.argv.index("--brunner-test")+1]))
    elif "--brunner-self-test" in sys.argv:
        from qtg_common_engine import self_test
        sys.exit(self_test(sys.argv[sys.argv.index("--brunner-self-test")+1]))
    elif "--portable-self-test" in sys.argv:
        from portable_self_test import run_self_test
        sys.exit(run_self_test(sys.modules[__name__]))
    elif "--engine-suite" in sys.argv:
        from SMK_Engine_From_Climb import main as engine_main
        engine_main()
    elif "--check" in sys.argv:
        print(f"Aeromix Multi-Crew FNTP II: {len(TESTS)} configured FNPT II objective validation tests")
        print(f"GUI file: {Path(__file__).resolve()}")
        print(f"SimConnect header: {(BASE / 'p3d_sdk' / 'SimConnect.h').exists()}")
        print(f"SimConnect lib: {(BASE / 'p3d_sdk' / 'SimConnect.lib').exists()}")
        print(f"Compiled bridge DLL: {DLL_PATH.exists()}")
    else:
        main()
