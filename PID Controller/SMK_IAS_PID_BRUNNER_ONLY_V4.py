"""
SMK Prepar3D Fast Pitch Angle & Airspeed Controller (V4 - Brunner Only)
Integrates SimConnect 64-bit Native C++ Bridge with Brunner CLS2Sim UDP Protocol.

Features:
1. Guidance Modes:
   - Direct Pitch Angle Hold (-20° .. +20°)
   - Airspeed Hold via Pitch / Brunner (Pitch-for-Speed / FLC Mode, 40 .. 220 knots)
2. Controls Brunner Active Control Loader ONLY (no direct elevator SimConnect commands).
3. Hardcoded wings-level bank angle (0.0°) enforced at 25 Hz.
4. Target Flight Conditions:
   - Aircraft Gross Weight: 1050 kg (displayed & monitored)
   - Ambient Temperature: 20.0°C (displayed & monitored)
5. Independent thread timing:
   - SimConnect Bridge: 50 Hz
   - PID / Guidance Worker: 25 Hz
   - GUI / Matplotlib Graphs: 10 Hz
"""

import collections
import csv
import ctypes
import datetime
import math
import os
import socket
import struct
import subprocess
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

def get_base_dir():
    """Returns directory of running script or bundled executable."""
    if getattr(sys, 'frozen', False):
        return os.path.dirname(sys.executable)
    return os.path.dirname(os.path.abspath(__file__))


# -----------------------------------------------------------------------------
# Brunner CLS2Sim UDP Protocol Constants & Helpers
# -----------------------------------------------------------------------------
HOST, PORT = '127.0.0.1', 15090
AX_PITCH = 1
CMD_READ, CMD_OVERRIDE, CMD_SETTINGS, CMD_REMOTE_CONFIG = 0xD0, 0xD1, 0xCE, 0xD8
POS_NORM = 0x11
OVR_AP_ENABLE, OVR_AP_POS, OVR_POSITION_TO_SIM = 0x03, 0x04, 0x06
SID_AP_FORCE, SID_AP_SPEED, SID_AP_ENABLE, SID_AP_POSITION = 0x50, 0x60, 0x70, 0x80
AP_FORCE, AP_SPEED = 2400, 80
WATCHDOG_MS = 1000

def pack_read(var):
    return struct.pack('<III', CMD_READ, AX_PITCH, var)

def pack_override(axes, var):
    return struct.pack('<Iii', CMD_OVERRIDE, int(axes), int(var))

def pack_u16(var, value):
    return struct.pack('<IiiH', CMD_SETTINGS, AX_PITCH, var, int(value))

def pack_u8(var, value):
    return struct.pack('<IiiB', CMD_SETTINGS, AX_PITCH, var, int(value))

def pack_float(var, value):
    return struct.pack('<Iiif', CMD_SETTINGS, AX_PITCH, var, float(value))

def pack_watchdog(ms):
    return struct.pack('<IIH', CMD_REMOTE_CONFIG, 0, int(ms))

def reply_status(blob):
    if len(blob) < 3 or struct.unpack_from('<H', blob)[0] != len(blob) - 2:
        raise RuntimeError('Malformed CLS2Sim response')
    return blob[2]

def reply_normalized(blob):
    if reply_status(blob) != 0:
        raise RuntimeError('CLS2Sim read returned error')
    # 1. Try Node 2 first (standard 2-axis yoke/stick pitch)
    for offset in range(3, len(blob) - 5, 6):
        node, value = struct.unpack_from('<Hf', blob, offset)
        if node == 2 and math.isfinite(value) and abs(value) <= 1.05:
            return float(value)
    # 2. Fallback to Node 1 (single-axis column or dedicated pitch loader)
    for offset in range(3, len(blob) - 5, 6):
        node, value = struct.unpack_from('<Hf', blob, offset)
        if node == 1 and math.isfinite(value) and abs(value) <= 1.05:
            return float(value)
    # 3. Universal fallback: accept any valid active loader node in payload
    for offset in range(3, len(blob) - 5, 6):
        node, value = struct.unpack_from('<Hf', blob, offset)
        if math.isfinite(value) and abs(value) <= 1.05:
            return float(value)
    raise RuntimeError('CLS2Sim pitch position missing across all nodes')

def slew_position(current, velocity, desired, dt, max_rate=0.45, max_accel=1.80):
    """Smooth acceleration & velocity limiter to prevent sudden physical motor jerks."""
    if dt <= 0:
        return current, velocity
    req_v = max(-max_rate, min(max_rate, (desired - current) / dt))
    next_v = max(velocity - max_accel * dt, min(velocity + max_accel * dt, req_v))
    next_p = current + next_v * dt
    if (desired - current) * (desired - next_p) <= 0:
        return desired, 0.0
    return next_p, next_v


class CLS2Sim:
    """Robust Brunner CLS2Sim UDP Driver based on kit hardware protocol."""
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sock.bind(('127.0.0.1', 0))
        except Exception:
            pass
        self.sock.settimeout(0.16)
        self.latest_position = 0.0
        self.latest_time = 0.0
        self.owns_overrides = False
        self.watchdog_enabled = False
        self.last_watchdog = 0.0
        self.connected = False

    def drain(self):
        """Flushes any lingering packets in the UDP socket buffer."""
        try:
            self.sock.setblocking(False)
            while True:
                try:
                    self.sock.recvfrom(4096)
                except (BlockingIOError, socket.error):
                    break
        except Exception:
            pass
        finally:
            self.sock.setblocking(True)
            self.sock.settimeout(0.16)

    def exchange(self, packet: bytes) -> bytes:
        self.drain()
        try:
            self.sock.sendto(packet, (HOST, PORT))
            reply, addr = self.sock.recvfrom(4096)
        except (socket.timeout, OSError) as e:
            self.connected = False
            raise RuntimeError('CLS2Sim UDP timeout/unavailable') from e
        if addr[0] != HOST:
            raise RuntimeError('Unexpected CLS2Sim response IP')
        self.connected = True
        return reply

    def write(self, packet: bytes):
        reply = self.exchange(packet)
        status = reply_status(reply)
        if status != 0:
            raise RuntimeError(f'CLS2Sim write rejected with status {status}')

    def read_position(self) -> float:
        reply = self.exchange(pack_read(POS_NORM))
        val = reply_normalized(reply)
        self.latest_position = val
        self.latest_time = time.monotonic()
        return val

    def poll(self):
        """Non-throwing poll for background loop."""
        try:
            pos = self.read_position()
            return True, pos
        except Exception:
            return False, self.latest_position

    def watchdog(self):
        """Periodically refresh watchdog (every 400ms) to maintain CLS2Sim motor engagement."""
        now = time.monotonic()
        if self.watchdog_enabled and (now - self.last_watchdog > 0.40):
            try:
                self.write(pack_watchdog(WATCHDOG_MS))
                self.last_watchdog = now
            except Exception:
                pass

    def engage(self, start_pos: float):
        """Arm Brunner motor overrides and configure autopilot force & speed."""
        self.drain()
        self.write(pack_watchdog(WATCHDOG_MS))
        self.watchdog_enabled = True
        self.last_watchdog = time.monotonic()

        self.write(pack_override(AX_PITCH, OVR_AP_ENABLE))
        self.write(pack_override(AX_PITCH, OVR_AP_POS))
        # IMPORTANT: Do NOT override OVR_POSITION_TO_SIM!
        # This allows CLS2Sim to transmit the physical stick position directly to Prepar3D!
        self.owns_overrides = True

        self.write(pack_u16(SID_AP_FORCE, AP_FORCE))
        self.write(pack_u16(SID_AP_SPEED, AP_SPEED))
        self.write(pack_float(SID_AP_POSITION, start_pos))
        self.write(pack_u8(SID_AP_ENABLE, 1))

    def position(self, target_pos: float):
        """Command new target position to Brunner autopilot motor."""
        clamped = max(-0.95, min(0.95, float(target_pos)))
        self.write(pack_float(SID_AP_POSITION, clamped))

    def release(self):
        """Gracefully release motor overrides and return manual stick control."""
        if self.owns_overrides:
            for pkt in (
                pack_u8(SID_AP_ENABLE, 0),
                pack_override(0, OVR_AP_ENABLE),
                pack_override(0, OVR_AP_POS),
                pack_watchdog(0)
            ):
                try:
                    self.write(pkt)
                except Exception:
                    pass
        self.watchdog_enabled = False
        self.owns_overrides = False

    def close(self):
        self.release()
        try:
            self.sock.close()
        except Exception:
            pass


# -----------------------------------------------------------------------------
# SimConnect C++ Bridge Interface
# -----------------------------------------------------------------------------
class PitchBridgeState(ctypes.Structure):
    _fields_ = [
        ('pitchDeg', ctypes.c_double),
        ('elevatorPosition', ctypes.c_double),
        ('yokeYPosition', ctypes.c_double),
        ('iasKt', ctypes.c_double),
        ('altitudeFt', ctypes.c_double),
        ('throttlePercent', ctypes.c_double),
        ('sentElevator', ctypes.c_double),
        ('sentYokeY', ctypes.c_double),
        ('sentAxis', ctypes.c_double),
        ('telemetryAgeMs', ctypes.c_uint64),
        ('connected', ctypes.c_int),
        ('telemetryValid', ctypes.c_int),
        ('writingEnabled', ctypes.c_int),
        ('controlMode', ctypes.c_int),
        ('lastWriteOk', ctypes.c_int),
        ('lastError', ctypes.c_int),
        ('totalWeightKg', ctypes.c_double),
        ('ambientTempC', ctypes.c_double),
        ('isPaused', ctypes.c_int),
    ]


class SimConnectBridge:
    def __init__(self):
        search_dirs = []
        if getattr(sys, 'frozen', False):
            search_dirs.append(os.path.dirname(sys.executable))
            if hasattr(sys, '_MEIPASS'):
                search_dirs.append(sys._MEIPASS)
        search_dirs.append(os.path.dirname(os.path.abspath(__file__)))
        search_dirs.append(os.getcwd())

        dll_path = None
        base_dir = search_dirs[0]
        for d in search_dirs:
            for name in ('smk_simbridge.dll', 'smk_simbridge_v4.dll'):
                candidate = os.path.join(d, name)
                if os.path.isfile(candidate):
                    dll_path = candidate
                    base_dir = d
                    break
            if dll_path:
                break

        if not dll_path:
            raise FileNotFoundError("Native SimConnect bridge DLL 'smk_simbridge.dll' not found in search paths")

        if sys.platform == 'win32':
            try:
                os.add_dll_directory(base_dir)
            except Exception:
                pass

        self.dll = ctypes.CDLL(dll_path)
        self.dll.SC_Start.restype = ctypes.c_int
        self.dll.SC_GetState.argtypes = [ctypes.c_void_p]
        self.dll.SC_GetState.restype = ctypes.c_int
        self.dll.SC_SetControlMode.argtypes = [ctypes.c_int]
        self.dll.SC_Command.argtypes = [ctypes.c_double]
        self.dll.SC_Enable.argtypes = [ctypes.c_int]
        self.dll.SC_Stop.restype = None

        if hasattr(self.dll, 'SC_IsPaused'):
            self.dll.SC_IsPaused.restype = ctypes.c_int

        if self.dll.SC_Start() != 1:
            raise RuntimeError("Failed to start SimConnect background worker")

    def read(self) -> PitchBridgeState:
        state = PitchBridgeState()
        self.dll.SC_GetState(ctypes.byref(state))
        return state

    def is_paused(self) -> bool:
        if hasattr(self.dll, 'SC_IsPaused'):
            try:
                return bool(self.dll.SC_IsPaused())
            except Exception:
                pass
        return False

    def set_mode(self, mode: int):
        self.dll.SC_SetControlMode(int(mode))

    def command(self, value: float):
        self.dll.SC_Command(float(value))

    def enable(self, enabled: bool):
        self.dll.SC_Enable(1 if enabled else 0)

    def close(self):
        try:
            self.enable(False)
            self.dll.SC_Stop()
        except Exception:
            pass


# -----------------------------------------------------------------------------
# Inner-Loop: Pitch Attitude PID Controller
# -----------------------------------------------------------------------------
class PitchPID:
    def __init__(self, kp=0.035, ki=0.015, kd=0.020, invert=False):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.invert = bool(invert)
        self.integral = 0.0
        self.last_pitch = None
        self.pitch_rate = 0.0
        self.output = 0.0

    def reset(self, initial_pos=0.0):
        self.integral = float(initial_pos)
        self.last_pitch = None
        self.pitch_rate = 0.0
        self.output = float(initial_pos)

    def update(self, target_deg: float, actual_deg: float, dt: float) -> float:
        if dt <= 0.0001:
            return self.output

        # Error: target_deg - actual_deg (negative when nose is too high, requiring forward push)
        error = target_deg - actual_deg

        if self.last_pitch is None:
            self.last_pitch = actual_deg
        raw_rate = (actual_deg - self.last_pitch) / dt
        self.last_pitch = actual_deg
        self.pitch_rate += (dt / (0.12 + dt)) * (raw_rate - self.pitch_rate)

        p_term = self.kp * error

        # Full trim authority [-0.55 .. +0.65] to dynamically hold any speed
        self.integral += self.ki * error * dt
        self.integral = max(-0.55, min(0.65, self.integral))
        i_term = self.integral

        # Aerodynamic pitch rate damping opposes rapid pitch departures
        d_term = -self.kd * self.pitch_rate

        effort = p_term + i_term + d_term

        if self.invert:
            effort = -effort

        self.output = max(-0.55, min(0.65, effort))
        return self.output

# Outer-Loop: Airspeed Controller (Pitch-for-Speed / FLC Mode)
# -----------------------------------------------------------------------------
class AirspeedController:
    """
    Outer-Loop Airspeed Controller (Pitch-for-Speed / FLC Mode).
    Computes commanded pitch angle to actively hold indicated airspeed (IAS):
    - When Airspeed < Target (too slow): pitches DOWN (lower pitch) -> accelerates.
    - When Airspeed > Target (too fast): pitches UP (higher pitch) -> decelerates.
    """
    def __init__(self, kp=0.40, ki=0.030, kd=0.15, min_pitch=-15.0, max_pitch=15.0):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.min_pitch = float(min_pitch)
        self.max_pitch = float(max_pitch)
        self.integral = 4.0
        self.last_ias = None
        self.ias_rate = 0.0
        self.cmd_pitch = 4.0

    def reset(self, initial_pitch=4.0):
        # Initialize integral to current pitch so transitions are bumpless,
        # but allow the integral and proportional terms to actively seek target speed!
        self.integral = max(self.min_pitch, min(self.max_pitch, float(initial_pitch)))
        self.last_ias = None
        self.ias_rate = 0.0
        self.cmd_pitch = float(initial_pitch)

    def update(self, target_ias: float, actual_ias: float, dt: float, current_pitch: float) -> float:
        if dt <= 0.0001:
            return self.cmd_pitch

        # Speed error: positive means too fast, negative means too slow
        spd_error = actual_ias - target_ias

        # Derivative term with 0.25s low-pass filter
        if self.last_ias is None:
            self.last_ias = actual_ias
        raw_rate = (actual_ias - self.last_ias) / dt
        self.last_ias = actual_ias
        self.ias_rate += (dt / (0.25 + dt)) * (raw_rate - self.ias_rate)

        # Proportional term (deg nose-up per kt too fast, nose-down per kt too slow)
        p_term = self.kp * spd_error

        # Active integral adaptation: continuously drives speed error to ZERO
        if not ((self.cmd_pitch >= self.max_pitch and spd_error > 0) or
                (self.cmd_pitch <= self.min_pitch and spd_error < 0)):
            self.integral += self.ki * spd_error * dt
            self.integral = max(self.min_pitch, min(self.max_pitch, self.integral))

        i_term = self.integral
        d_term = self.kd * self.ias_rate

        # Demanded pitch angle: dynamically drives toward TARGET IAS
        pitch_demand = p_term + i_term + d_term

        # Clamp strictly within flight safety envelope (-15.0 deg to +15.0 deg)
        desired_target = max(self.min_pitch, min(self.max_pitch, pitch_demand))

        # Responsive pitch slew rate limit (2.20 deg/sec)
        max_step = 2.20 * dt
        delta = desired_target - self.cmd_pitch
        if abs(delta) > max_step:
            self.cmd_pitch += max_step if delta > 0 else -max_step
        else:
            self.cmd_pitch = desired_target

        return self.cmd_pitch


# -----------------------------------------------------------------------------
# GUI Application with Decoupled Control & Render Loops
# -----------------------------------------------------------------------------
class PitchPIDApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Prepar3D Pitch & Airspeed Flight Controller (Brunner Only V4)")
        self.root.geometry("1260x890")
        self.root.minsize(1100, 810)

        # Dark theme styling
        self.bg_color = "#1e222b"
        self.panel_bg = "#272c38"
        self.text_color = "#e6edf3"
        self.accent_color = "#388bfd"
        self.root.configure(bg=self.bg_color)

        # Subsystems & Controllers
        self.bridge = SimConnectBridge()
        self.cls = CLS2Sim()
        self.pid = PitchPID(kp=0.035, ki=0.015, kd=0.020, invert=False)
        self.ias_ctrl = AirspeedController(kp=0.40, ki=0.030, kd=0.15, min_pitch=-15.0, max_pitch=15.0)

        # Thread synchronization
        self.lock = threading.Lock()
        self.running = True
        self.after_id = None

        self.pid_enabled = False
        self.guidance_mode = "IAS"  # Default: Airspeed Hold
        self.target_pitch = 0.0
        self.target_ias = 80.0
        self.cmd_pitch = 0.0
        # Default Mode 3: BRUNNER CLS2Sim Only (direct SimConnect elevator writing removed)
        self.control_mode = 3

        # Brunner motor state
        self.brunner_command = 0.0
        self.brunner_velocity = 0.0
        self.brunner_initial = 0.0

        # CSV Logging for every test run (saved in dedicated 'runs' directory)
        self.runs_dir = os.path.join(get_base_dir(), "runs")
        os.makedirs(self.runs_dir, exist_ok=True)
        self.csv_file = None
        self.csv_writer = None
        self.csv_filepath = None
        self.csv_row_count = 0
        self.csv_run_start_time = 0.0
        self.csv_last_flush_time = 0.0
        self.csv_status_msg = "Ready"

        # Pause tracking
        self.snap_paused = False
        self.was_paused = False

        # Snapshots shared from control thread to GUI thread
        self.snap_connected = False
        self.snap_telemetry_age = 9999
        self.snap_brunner_connected = False
        self.snap_brunner_pos = 0.0
        self.snap_pitch_actual = 0.0
        self.snap_pitch_target = 0.0
        self.snap_pitch_error = 0.0
        self.snap_ias_actual = 0.0
        self.snap_ias_target = 80.0
        self.snap_ias_error = 0.0
        self.snap_cmd_pitch = 0.0
        self.snap_guidance_mode = "IAS"
        self.snap_throttle = 0.0
        self.snap_elev_pos = 0.0
        self.snap_yoke_pos = 0.0
        self.snap_axis_sent = 0.0
        self.snap_altitude = 0.0
        self.snap_weight = 1050.0
        self.snap_temp = 20.0
        self.measured_hz = 25.0
        self.snap_measured_hz = 25.0

        # Rolling telemetry buffer for live graphs (30 seconds at 10 Hz = 300 pts)
        self.history_len = 300
        self.time_history = collections.deque(maxlen=self.history_len)
        self.pitch_actual_hist = collections.deque(maxlen=self.history_len)
        self.pitch_target_hist = collections.deque(maxlen=self.history_len)
        self.pitch_error_hist = collections.deque(maxlen=self.history_len)
        self.ias_actual_hist = collections.deque(maxlen=self.history_len)
        self.ias_target_hist = collections.deque(maxlen=self.history_len)
        self.ias_error_hist = collections.deque(maxlen=self.history_len)
        self.elev_pos_hist = collections.deque(maxlen=self.history_len)
        self.yoke_y_hist = collections.deque(maxlen=self.history_len)
        self.brunner_pos_hist = collections.deque(maxlen=self.history_len)
        self.brunner_cmd_hist = collections.deque(maxlen=self.history_len)
        self.axis_sent_hist = collections.deque(maxlen=self.history_len)

        self.start_time = time.monotonic()

        self._build_ui()
        self.on_guidance_change()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        # 1. Start dedicated high-precision PID control worker (25 Hz = 40 ms)
        self.worker_thread = threading.Thread(target=self._pid_worker_loop, name="PIDControlWorker", daemon=True)
        self.worker_thread.start()

        # 2. Start smooth GUI chart rendering loop (10 Hz = 100 ms)
        self.after_id = self.root.after(100, self.gui_tick)

    def _build_ui(self):
        style = ttk.Style()
        style.theme_use('default')
        style.configure('.', background=self.panel_bg, foreground=self.text_color)
        style.configure('TLabel', background=self.panel_bg, foreground=self.text_color, font=('Segoe UI', 10))
        style.configure('Header.TLabel', font=('Segoe UI', 12, 'bold'), foreground='#58a6ff')
        style.configure('TCheckbutton', background=self.panel_bg, foreground=self.text_color, font=('Segoe UI', 10, 'bold'))
        style.configure('TRadiobutton', background=self.panel_bg, foreground=self.text_color, font=('Segoe UI', 10))

        # ----------------- Top Header Bar -----------------
        top_bar = tk.Frame(self.root, bg=self.panel_bg, height=56, padx=16, pady=8)
        top_bar.pack(side=tk.TOP, fill=tk.X, padx=10, pady=(10, 6))

        title_lbl = tk.Label(top_bar, text="P3D FLIGHT CONTROLLER (BRUNNER ONLY)",
                             font=('Segoe UI', 12, 'bold'), bg=self.panel_bg, fg="#58a6ff")
        title_lbl.pack(side=tk.LEFT)

        # Status pills
        self.conn_badge = tk.Label(top_bar, text="SimConnect: Disconnected",
                                   font=('Segoe UI', 9, 'bold'), bg="#484f58", fg="#f0f6fc",
                                   padx=10, pady=3)
        self.conn_badge.pack(side=tk.LEFT, padx=8)

        self.brunner_badge = tk.Label(top_bar, text="Brunner: Checking...",
                                      font=('Segoe UI', 9, 'bold'), bg="#484f58", fg="#f0f6fc",
                                      padx=10, pady=3)
        self.brunner_badge.pack(side=tk.LEFT, padx=4)

        self.mode_badge = tk.Label(top_bar, text="Mode: AIRSPEED HOLD (80 kt)",
                                   font=('Segoe UI', 9, 'bold'), bg="#1f6feb", fg="#ffffff",
                                   padx=10, pady=3)
        self.mode_badge.pack(side=tk.LEFT, padx=4)

        self.loop_badge = tk.Label(top_bar, text="PID Rate: 25.0 Hz",
                                   font=('Segoe UI', 9, 'bold'), bg="#1f6feb", fg="#ffffff",
                                   padx=10, pady=3)
        self.loop_badge.pack(side=tk.LEFT, padx=4)

        self.csv_badge = tk.Label(top_bar, text="CSV: Ready",
                                  font=('Segoe UI', 9, 'bold'), bg="#30363d", fg="#8b949e",
                                  padx=10, pady=3)
        self.csv_badge.pack(side=tk.LEFT, padx=4)

        # Engage / Disengage PID Button
        self.engage_btn = tk.Button(top_bar, text="START AIRSPEED HOLD",
                                    font=('Segoe UI', 11, 'bold'), bg="#238636", fg="#ffffff",
                                    activebackground="#2ea043", activeforeground="#ffffff",
                                    relief=tk.FLAT, padx=18, pady=4, cursor="hand2",
                                    command=self.toggle_pid)
        self.engage_btn.pack(side=tk.RIGHT)

        self.open_runs_btn = tk.Button(top_bar, text="📁 Runs Folder",
                                       font=('Segoe UI', 9, 'bold'), bg="#21262d", fg="#58a6ff",
                                       activebackground="#30363d", activeforeground="#58a6ff",
                                       relief=tk.FLAT, padx=10, pady=4, cursor="hand2",
                                       command=self.open_runs_folder)
        self.open_runs_btn.pack(side=tk.RIGHT, padx=6)

        # ----------------- Middle Controls Panel -----------------
        ctrl_frame = tk.Frame(self.root, bg=self.panel_bg, padx=14, pady=8)
        ctrl_frame.pack(side=tk.TOP, fill=tk.X, padx=10, pady=4)

        # Row 1: Guidance Strategy & Target Inputs & Output Channels
        row1 = tk.Frame(ctrl_frame, bg=self.panel_bg)
        row1.pack(fill=tk.X, pady=(0, 6))

        # --- Guidance Strategy Box ---
        strategy_box = tk.LabelFrame(row1, text="  Guidance Mode  ",
                                     font=('Segoe UI', 10, 'bold'), bg=self.panel_bg, fg="#79c0ff",
                                     padx=10, pady=6)
        strategy_box.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))

        self.strategy_var = tk.StringVar(value="IAS")
        lbl_mode_fixed = tk.Label(strategy_box, text="AIRSPEED HOLD\n(Pitch-for-Speed)",
                                  font=('Segoe UI', 10, 'bold'), fg="#38bdf8", bg=self.panel_bg, justify=tk.LEFT)
        lbl_mode_fixed.pack(anchor=tk.W, padx=4, pady=4)

        # --- Target Airspeed Section ---
        self.ias_box = tk.LabelFrame(row1, text="  Target Airspeed (knots) [ACTIVE]  ",
                                     font=('Segoe UI', 10, 'bold'), bg=self.panel_bg, fg="#38bdf8",
                                     padx=8, pady=4)
        self.ias_box.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 8))

        self.target_ias_var = tk.DoubleVar(value=80.0)
        self.target_ias_slider = tk.Scale(self.ias_box, from_=40.0, to=200.0, resolution=1.0,
                                          orient=tk.HORIZONTAL, variable=self.target_ias_var,
                                          bg=self.panel_bg, fg=self.text_color, troughcolor="#161b22",
                                          highlightthickness=0, length=120, command=self.on_target_ias_change)
        self.target_ias_slider.pack(side=tk.LEFT, padx=3)

        self.target_ias_entry = tk.Entry(self.ias_box, width=5, font=('Segoe UI', 10, 'bold'),
                                         bg="#161b22", fg="#38bdf8", insertbackground="#ffffff", justify=tk.CENTER)
        self.target_ias_entry.insert(0, "80")
        self.target_ias_entry.pack(side=tk.LEFT, padx=4)
        self.target_ias_entry.bind("<Return>", self.on_target_ias_entry)
        self.target_ias_entry.bind("<FocusOut>", self.on_target_ias_entry)

        self.ias_quick_btns = []
        for val in [65, 80, 90, 100]:
            btn = tk.Button(self.ias_box, text=f"{val}", font=('Segoe UI', 8, 'bold'),
                            bg="#30363d", fg=self.text_color, relief=tk.FLAT, padx=4, pady=1,
                            command=lambda v=val: self.set_target_ias(v))
            btn.pack(side=tk.LEFT, padx=1)
            self.ias_quick_btns.append(btn)

        # --- Output Control Channel Selector (BRUNNER CLS2Sim Only) ---
        mode_box = tk.LabelFrame(row1, text="  Pitch Output Channel  ",
                                 font=('Segoe UI', 10, 'bold'), bg=self.panel_bg, fg="#79c0ff",
                                 padx=8, pady=4)
        mode_box.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 8))

        self.mode_var = tk.IntVar(value=3)
        rb4 = ttk.Radiobutton(mode_box, text="BRUNNER CLS2Sim (Motor Drive)", value=3, variable=self.mode_var,
                              command=self.on_mode_change)
        rb4.pack(anchor=tk.W, padx=6, pady=4)

        lbl_hint = tk.Label(mode_box, text="Direct Sim elevator writing disabled", font=('Segoe UI', 8, 'italic'),
                            fg="#8b949e", bg=self.panel_bg)
        lbl_hint.pack(anchor=tk.W, padx=4, pady=(0, 2))

        # --- Sign Inversion Toggle ---
        sign_box = tk.LabelFrame(row1, text="  Polarity  ",
                                 font=('Segoe UI', 10, 'bold'), bg=self.panel_bg, fg="#79c0ff",
                                 padx=8, pady=4)
        sign_box.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.invert_var = tk.BooleanVar(value=False)
        inv_chk = ttk.Checkbutton(sign_box, text="Invert Sign (-/+)",
                                  variable=self.invert_var, command=self.on_invert_change)
        inv_chk.pack(anchor=tk.W, padx=4, pady=6)

        # Row 2: Realtime PID Sliders & Live Telemetry Readout
        row2 = tk.Frame(ctrl_frame, bg=self.panel_bg)
        row2.pack(fill=tk.X, pady=(2, 0))

        # --- Realtime PID Gains ---
        pid_box = tk.LabelFrame(row2, text="  Realtime Control Gains  ",
                                font=('Segoe UI', 10, 'bold'), bg=self.panel_bg, fg="#79c0ff",
                                padx=8, pady=4)
        pid_box.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))

        # Airspeed (IAS) Outer-Loop Gains
        tk.Label(pid_box, text="IAS Kp:", font=('Segoe UI', 9, 'bold'), bg=self.panel_bg, fg="#38bdf8").grid(row=0, column=0, padx=4, sticky=tk.W)
        self.ias_kp_var = tk.DoubleVar(value=0.40)
        self.ias_kp_scale = tk.Scale(pid_box, from_=0.000, to=1.500, resolution=0.01, orient=tk.HORIZONTAL,
                                     variable=self.ias_kp_var, bg=self.panel_bg, fg=self.text_color, troughcolor="#161b22",
                                     highlightthickness=0, length=90, command=self.on_ias_pid_change)
        self.ias_kp_scale.grid(row=0, column=1, padx=4)
        self.ias_kp_entry = tk.Entry(pid_box, width=7, font=('Segoe UI', 9, 'bold'),
                                     bg="#161b22", fg="#38bdf8", insertbackground="#ffffff", justify=tk.CENTER)
        self.ias_kp_entry.insert(0, "0.35")
        self.ias_kp_entry.grid(row=0, column=2, padx=4)
        self.ias_kp_entry.bind("<Return>", self.on_ias_kp_entry)
        self.ias_kp_entry.bind("<FocusOut>", self.on_ias_kp_entry)

        tk.Label(pid_box, text="Ki:", font=('Segoe UI', 9, 'bold'), bg=self.panel_bg, fg="#38bdf8").grid(row=0, column=3, padx=4, sticky=tk.W)
        self.ias_ki_var = tk.DoubleVar(value=0.0250)
        self.ias_ki_scale = tk.Scale(pid_box, from_=0.000, to=0.080, resolution=0.001, orient=tk.HORIZONTAL,
                                     variable=self.ias_ki_var, bg=self.panel_bg, fg=self.text_color, troughcolor="#161b22",
                                     highlightthickness=0, length=90, command=self.on_ias_pid_change)
        self.ias_ki_scale.grid(row=0, column=4, padx=4)
        self.ias_ki_entry = tk.Entry(pid_box, width=7, font=('Segoe UI', 9, 'bold'),
                                     bg="#161b22", fg="#38bdf8", insertbackground="#ffffff", justify=tk.CENTER)
        self.ias_ki_entry.insert(0, "0.0250")
        self.ias_ki_entry.grid(row=0, column=5, padx=4)
        self.ias_ki_entry.bind("<Return>", self.on_ias_ki_entry)
        self.ias_ki_entry.bind("<FocusOut>", self.on_ias_ki_entry)

        tk.Label(pid_box, text="Kd:", font=('Segoe UI', 9, 'bold'), bg=self.panel_bg, fg="#38bdf8").grid(row=0, column=6, padx=4, sticky=tk.W)
        self.ias_kd_var = tk.DoubleVar(value=0.15)
        self.ias_kd_scale = tk.Scale(pid_box, from_=0.000, to=0.800, resolution=0.005, orient=tk.HORIZONTAL,
                                     variable=self.ias_kd_var, bg=self.panel_bg, fg=self.text_color, troughcolor="#161b22",
                                     highlightthickness=0, length=90, command=self.on_ias_pid_change)
        self.ias_kd_scale.grid(row=0, column=7, padx=4)
        self.ias_kd_entry = tk.Entry(pid_box, width=7, font=('Segoe UI', 9, 'bold'),
                                     bg="#161b22", fg="#38bdf8", insertbackground="#ffffff", justify=tk.CENTER)
        self.ias_kd_entry.insert(0, "0.12")
        self.ias_kd_entry.grid(row=0, column=8, padx=4)
        self.ias_kd_entry.bind("<Return>", self.on_ias_kd_entry)
        self.ias_kd_entry.bind("<FocusOut>", self.on_ias_kd_entry)

        # --- Digital Readouts ---
        data_box = tk.LabelFrame(row2, text="  Live Flight Values  ",
                                 font=('Segoe UI', 10, 'bold'), bg=self.panel_bg, fg="#79c0ff",
                                 padx=8, pady=4)
        data_box.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Row 0: Pitch Actual, Pitch Error, Throttle
        self.lbl_pitch_act = tk.Label(data_box, text="Pitch: 0.00°", font=('Segoe UI', 9, 'bold'), fg="#39d353", bg=self.panel_bg)
        self.lbl_pitch_act.grid(row=0, column=0, padx=6, sticky=tk.W)

        self.lbl_pitch_err = tk.Label(data_box, text="Pitch Err: 0.00°", font=('Segoe UI', 9, 'bold'), fg="#f85149", bg=self.panel_bg)
        self.lbl_pitch_err.grid(row=0, column=1, padx=6, sticky=tk.W)

        self.lbl_throttle = tk.Label(data_box, text="Throttle: 0.0%", font=('Segoe UI', 9, 'bold'), fg="#e3b341", bg=self.panel_bg)
        self.lbl_throttle.grid(row=0, column=2, padx=6, sticky=tk.W)

        # Row 1: Actual IAS, IAS Error, Commanded Pitch
        self.lbl_ias_act = tk.Label(data_box, text="IAS: 0.0 kt", font=('Segoe UI', 9, 'bold'), fg="#38bdf8", bg=self.panel_bg)
        self.lbl_ias_act.grid(row=1, column=0, padx=6, sticky=tk.W)

        self.lbl_ias_err = tk.Label(data_box, text="IAS Err: 0.0 kt", font=('Segoe UI', 9, 'bold'), fg="#ffa657", bg=self.panel_bg)
        self.lbl_ias_err.grid(row=1, column=1, padx=6, sticky=tk.W)

        self.lbl_cmd_pitch = tk.Label(data_box, text="Cmd Pitch: 0.00°", font=('Segoe UI', 9, 'bold'), fg="#d2a8ff", bg=self.panel_bg)
        self.lbl_cmd_pitch.grid(row=1, column=2, padx=6, sticky=tk.W)

        # Row 2: Brunner stick position
        self.lbl_brunner = tk.Label(data_box, text="Brunner: 0.000", font=('Segoe UI', 9, 'bold'), fg="#58a6ff", bg=self.panel_bg)
        self.lbl_brunner.grid(row=2, column=0, columnspan=3, padx=6, pady=(2, 0), sticky=tk.W)

        # ----------------- Bottom Live Graphs -----------------
        graph_frame = tk.Frame(self.root, bg="#161b22")
        graph_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=10, pady=4)

        plt.style.use('dark_background')
        self.fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(3, 1, sharex=True, figsize=(10, 6.2), dpi=100)
        self.fig.patch.set_facecolor('#161b22')

        # --- Graph 1: Pitch Attitude Tracking (deg) ---
        self.ax1.set_facecolor('#0d1117')
        self.ax1.set_ylabel("Pitch (°)", fontsize=9, color="#39d353")
        self.ax1.tick_params(colors="#39d353")
        self.ax1.grid(True, linestyle="--", alpha=0.3, color="#30363d")
        self.ax1.set_ylim(-15.0, 15.0)

        self.line_actual, = self.ax1.plot([], [], label="Pitch Actual", color="#39d353", linewidth=2.0)
        self.line_target, = self.ax1.plot([], [], label="Pitch Target / Cmd", color="#e3b341", linestyle="--", linewidth=1.8)
        self.line_error, = self.ax1.plot([], [], label="Pitch Error", color="#f85149", linewidth=1.4)
        self.ax1.legend(loc="upper left", facecolor="#161b22", edgecolor="#30363d", fontsize=8)
        self.ax1.set_title("Pitch Attitude Tracking & Error (deg)", fontsize=10, fontweight="bold", color="#58a6ff", pad=3)

        # --- Graph 2: Dedicated Airspeed Tracking (knots) ---
        self.ax2.set_facecolor('#0d1117')
        self.ax2.set_ylabel("IAS (kt)", fontsize=9, color="#38bdf8")
        self.ax2.tick_params(colors="#38bdf8")
        self.ax2.grid(True, linestyle="--", alpha=0.3, color="#30363d")

        self.line_ias_act, = self.ax2.plot([], [], label="Actual Airspeed (IAS)", color="#38bdf8", linewidth=2.0)
        self.line_ias_tgt, = self.ax2.plot([], [], label="Target Airspeed", color="#c084fc", linestyle="--", linewidth=1.8)
        self.line_ias_err, = self.ax2.plot([], [], label="Airspeed Error", color="#ffa657", linewidth=1.4)
        self.ax2.legend(loc="upper left", facecolor="#161b22", edgecolor="#30363d", fontsize=8)
        self.ax2.set_title("Airspeed Tracking & Error (knots)", fontsize=10, fontweight="bold", color="#38bdf8", pad=3)

        # --- Graph 3: Brunner Stick Control Channel ---
        self.ax3.set_facecolor('#0d1117')
        self.ax3.set_xlabel("Time (seconds)", fontsize=9, color="#c9d1d9")
        self.ax3.set_ylabel("Brunner [-1 .. 1]", fontsize=9, color="#ffa657")
        self.ax3.grid(True, linestyle="--", alpha=0.3, color="#30363d")

        self.line_brunner, = self.ax3.plot([], [], label="Brunner Stick (Measured)", color="#ffa657", linewidth=2.0)
        self.line_brunner_cmd, = self.ax3.plot([], [], label="Brunner Stick (Commanded)", color="#38bdf8", linestyle="--", linewidth=1.6)
        self.ax3.legend(loc="upper left", facecolor="#161b22", edgecolor="#30363d", fontsize=8)
        self.ax3.set_title("Brunner Stick Control Channel", fontsize=10, fontweight="bold", color="#ffa657", pad=3)

        self.fig.tight_layout()
        self.canvas = FigureCanvasTkAgg(self.fig, master=graph_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    # ----------------- Dedicated 25 Hz Control Thread -----------------
    def _pid_worker_loop(self):
        last_t = time.monotonic()
        while self.running:
            now = time.monotonic()
            dt = now - last_t
            last_t = now
            dt = min(0.1, max(0.01, dt))

            inst_hz = 1.0 / dt if dt > 0.0001 else 25.0
            self.measured_hz = 0.90 * self.measured_hz + 0.10 * inst_hz

            # 1. Read SimConnect telemetry
            state = self.bridge.read()
            connected = (state.connected == 1 and state.telemetryValid == 1 and state.telemetryAgeMs < 1500)
            is_paused = (getattr(state, 'isPaused', 0) == 1) or self.bridge.is_paused()
            
            # Aviation standard: Positive (+) = Nose UP (Climb), Negative (-) = Nose DOWN (Dive)
            pitch_actual = -state.pitchDeg if math.isfinite(state.pitchDeg) else 0.0
            ias_actual = state.iasKt if math.isfinite(state.iasKt) else 0.0
            elev_pos = state.elevatorPosition if math.isfinite(state.elevatorPosition) else 0.0
            yoke_pos = state.yokeYPosition if math.isfinite(state.yokeYPosition) else 0.0
            axis_sent = state.sentAxis if math.isfinite(state.sentAxis) else 0.0
            throttle_val = state.throttlePercent if math.isfinite(state.throttlePercent) else 0.0
            alt_val = state.altitudeFt if math.isfinite(state.altitudeFt) else 0.0
            weight_val = state.totalWeightKg if math.isfinite(state.totalWeightKg) else 1050.0
            temp_val = state.ambientTempC if math.isfinite(state.ambientTempC) else 20.0

            # 2. Poll Brunner hardware & service hardware watchdog every cycle
            brunner_conn, brunner_pos = self.cls.poll()
            self.cls.watchdog()

            with self.lock:
                enabled = self.pid_enabled
                guidance = self.guidance_mode
                mode = self.control_mode
                target_p = self.target_pitch
                target_ias = self.target_ias

            # If Prepar3D is paused, freeze controller execution
            if is_paused:
                self.was_paused = True
                last_t = now  # Reset time step to prevent dt surge when resuming
                if connected and brunner_conn and enabled and mode == 3:
                    self.cls.watchdog()
                    self.cls.position(self.brunner_command)
                with self.lock:
                    self.snap_paused = True
                    self.snap_connected = connected
                    self.snap_telemetry_age = state.telemetryAgeMs
                    self.snap_measured_hz = 0.0
                    self.snap_brunner_connected = brunner_conn
                    self.snap_brunner_pos = brunner_pos
                    self.snap_pitch_actual = pitch_actual
                    self.snap_ias_actual = ias_actual
                time.sleep(0.040)
                continue

            # Bumpless resumption when unpausing
            if self.was_paused:
                self.was_paused = False
                last_t = now
                if guidance == "IAS":
                    self.ias_ctrl.reset(initial_pitch=pitch_actual)
                self.pid.reset(initial_pos=self.brunner_command)

            # 3. Outer Guidance Strategy: Pitch Hold vs Airspeed Hold (Pitch-for-Speed)
            if guidance == "IAS":
                if ias_actual >= 10.0 and enabled:
                    active_target_pitch = self.ias_ctrl.update(target_ias, ias_actual, dt, pitch_actual)
                else:
                    active_target_pitch = 0.0
                    if not enabled:
                        self.ias_ctrl.reset(initial_pitch=pitch_actual)
            else:
                active_target_pitch = target_p

            pitch_error = pitch_actual - active_target_pitch
            ias_error = ias_actual - target_ias

            # 4. Inner Attitude Control Execution (BRUNNER MOTOR ONLY)
            if enabled:
                if mode == 3:
                    # Mode 3: BRUNNER CLS2Sim Motor Control ONLY
                    if connected and brunner_conn:
                        # Auto-rearm Brunner motor overrides & watchdog if connection dropped
                        if not self.cls.owns_overrides or not self.cls.watchdog_enabled:
                            self.cls.engage(self.brunner_command)

                        # Direct full-authority Brunner stick positioning [-0.55 .. +0.65]
                        desired_pos = self.pid.update(active_target_pitch, pitch_actual, dt)

                        # Smooth, responsive slew limiter prevents motor jerks while ensuring prompt response
                        next_p, next_v = slew_position(self.brunner_command, self.brunner_velocity, desired_pos, dt, max_rate=0.28, max_accel=0.90)
                        self.brunner_command = next_p
                        self.brunner_velocity = next_v

                        # Send position to Brunner motor ONLY - no elevator bridge command!
                        self.cls.position(self.brunner_command)
                        output_cmd = self.brunner_command
                    else:
                        # Temporary UDP/telemetry jitter: maintain command without disengaging motor!
                        output_cmd = self.brunner_command
                else:
                    output_cmd = 0.0
            else:
                if self.cls.watchdog_enabled or self.cls.owns_overrides:
                    self.cls.release()
                output_cmd = 0.0

            # 5. Log telemetry sample to CSV if actively recording this test run
            with self.lock:
                csv_w = self.csv_writer
                csv_f = self.csv_file
                t0 = self.csv_run_start_time

            if enabled and csv_w is not None and not is_paused:
                now_mono = time.monotonic()
                elapsed = now_mono - t0
                iso_time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                row = [
                    iso_time,
                    f"{elapsed:.3f}",
                    guidance,
                    f"{target_ias:.1f}",
                    f"{ias_actual:.2f}",
                    f"{ias_error:.2f}",
                    f"{active_target_pitch:.2f}",
                    f"{pitch_actual:.2f}",
                    f"{pitch_error:.2f}",
                    f"{output_cmd:.4f}",
                    f"{brunner_pos:.4f}",
                    f"{elev_pos:.4f}",
                    f"{yoke_pos:.4f}",
                    f"{axis_sent:.4f}",
                    f"{throttle_val:.2f}",
                    f"{alt_val:.1f}",
                    f"{weight_val:.1f}",
                    f"{temp_val:.1f}",
                    f"{self.ias_ctrl.kp:.4f}",
                    f"{self.ias_ctrl.ki:.5f}",
                    f"{self.ias_ctrl.kd:.4f}",
                    f"{self.pid.kp:.4f}",
                    f"{self.pid.ki:.5f}",
                    f"{self.pid.kd:.4f}",
                    1 if connected else 0,
                    1 if brunner_conn else 0,
                    1 if is_paused else 0,
                ]
                try:
                    csv_w.writerow(row)
                    self.csv_row_count += 1
                    if now_mono - self.csv_last_flush_time > 1.0:
                        csv_f.flush()
                        self.csv_last_flush_time = now_mono
                except Exception:
                    pass

            # Store snapshot for UI thread
            with self.lock:
                self.snap_paused = False
                self.snap_connected = connected
                self.snap_telemetry_age = state.telemetryAgeMs
                self.snap_measured_hz = self.measured_hz
                self.snap_brunner_connected = brunner_conn
                self.snap_brunner_pos = brunner_pos
                self.snap_pitch_actual = pitch_actual
                self.snap_pitch_target = active_target_pitch
                self.snap_pitch_error = pitch_error
                self.snap_ias_actual = ias_actual
                self.snap_ias_target = target_ias
                self.snap_ias_error = ias_error
                self.snap_cmd_pitch = active_target_pitch
                self.snap_guidance_mode = guidance
                self.snap_throttle = throttle_val
                self.snap_elev_pos = elev_pos
                self.snap_yoke_pos = yoke_pos
                self.snap_axis_sent = axis_sent
                self.snap_altitude = alt_val
                self.snap_weight = weight_val
                self.snap_temp = temp_val

            time.sleep(0.040)

    # ----------------- Smooth 10 Hz GUI Tick Loop (100 ms) -----------------
    def gui_tick(self):
        if not self.running:
            return

        with self.lock:
            conn = self.snap_connected
            age = self.snap_telemetry_age
            hz = self.snap_measured_hz
            is_paused = self.snap_paused
            b_conn = self.snap_brunner_connected
            b_pos = self.snap_brunner_pos
            pitch_act = self.snap_pitch_actual
            pitch_tgt = self.snap_pitch_target
            pitch_err = self.snap_pitch_error
            ias_act = self.snap_ias_actual
            ias_tgt = self.snap_ias_target
            ias_err = self.snap_ias_error
            cmd_p = self.snap_cmd_pitch
            guidance = self.snap_guidance_mode
            throttle = self.snap_throttle
            elev = self.snap_elev_pos
            yoke = self.snap_yoke_pos
            axis = self.snap_axis_sent
            weight = self.snap_weight
            temp = self.snap_temp

        # 1. Update Connection Badges
        if is_paused:
            self.conn_badge.configure(text="P3D: PAUSED", bg="#d29922", fg="#000000")
            self.loop_badge.configure(text="PID: PAUSED", bg="#d29922", fg="#000000")
        elif conn:
            self.conn_badge.configure(text=f"SimConnect: Connected ({age}ms)", bg="#238636", fg="#ffffff")
            self.loop_badge.configure(text=f"PID Rate: {hz:.1f} Hz", bg="#1f6feb", fg="#ffffff")
        else:
            self.conn_badge.configure(text="SimConnect: Disconnected / Waiting", bg="#484f58", fg="#f0f6fc")
            self.loop_badge.configure(text=f"PID Rate: {hz:.1f} Hz", bg="#1f6feb", fg="#ffffff")

        if b_conn:
            self.brunner_badge.configure(text=f"Brunner: Connected ({b_pos:+.2f})", bg="#238636", fg="#ffffff")
        else:
            self.brunner_badge.configure(text="Brunner: Offline (Port 15090)", bg="#484f58", fg="#f0f6fc")

        if guidance == "IAS":
            self.mode_badge.configure(text=f"Mode: AIRSPEED HOLD ({ias_tgt:.0f} kt)", bg="#1f6feb", fg="#ffffff")
        else:
            self.mode_badge.configure(text=f"Mode: PITCH HOLD ({pitch_tgt:+.1f}°)", bg="#8957e5", fg="#ffffff")

        # 2. Update Digital Labels & CSV Status
        pause_suffix = " [PAUSED]" if is_paused else ""
        self.lbl_pitch_act.configure(text=f"Pitch: {pitch_act:+.2f}°{pause_suffix}")
        self.lbl_pitch_err.configure(text=f"Pitch Err: {pitch_err:+.2f}°")
        self.lbl_throttle.configure(text=f"Throttle: {throttle:.1f}%")

        self.lbl_ias_act.configure(text=f"IAS: {ias_act:.1f} kt{pause_suffix}")
        self.lbl_ias_err.configure(text=f"IAS Err: {ias_err:+.1f} kt")
        self.lbl_cmd_pitch.configure(text=f"Cmd Pitch: {cmd_p:+.2f}°")

        self.lbl_brunner.configure(text=f"Stick (Meas/Cmd): {b_pos:+.2f} / {self.brunner_command:+.2f}")

        # Update CSV status badge
        with self.lock:
            csv_active = (self.csv_file is not None and self.pid_enabled)
            csv_rows = self.csv_row_count
            csv_t0 = self.csv_run_start_time
            csv_msg = self.csv_status_msg

        if csv_active:
            csv_dur = max(0.0, time.monotonic() - csv_t0)
            self.csv_badge.configure(text=f"● REC ({csv_rows} pts / {csv_dur:.0f}s)", bg="#b62324", fg="#ffffff")
        elif csv_rows > 0:
            self.csv_badge.configure(text=f"CSV: {csv_msg}", bg="#238636", fg="#ffffff")
        else:
            self.csv_badge.configure(text="CSV: Ready (runs/)", bg="#30363d", fg="#8b949e")

        # When paused, freeze graph history and skip appending duplicate points
        if is_paused:
            if self.running:
                try:
                    if self.root.winfo_exists():
                        self.after_id = self.root.after(100, self.gui_tick)
                except Exception:
                    pass
            return

        # 3. Append to Graph History (only while running)
        elapsed = time.monotonic() - self.start_time
        self.time_history.append(elapsed)
        self.pitch_actual_hist.append(pitch_act)
        self.pitch_target_hist.append(pitch_tgt)
        self.pitch_error_hist.append(pitch_err)
        self.ias_actual_hist.append(ias_act)
        self.ias_target_hist.append(ias_tgt)
        self.ias_error_hist.append(ias_err)
        self.brunner_pos_hist.append(b_pos)
        self.brunner_cmd_hist.append(self.brunner_command)

        # 4. Redraw Matplotlib Plot
        if len(self.time_history) > 1:
            t_list = list(self.time_history)
            self.line_actual.set_data(t_list, list(self.pitch_actual_hist))
            self.line_target.set_data(t_list, list(self.pitch_target_hist))
            self.line_error.set_data(t_list, list(self.pitch_error_hist))

            self.line_ias_act.set_data(t_list, list(self.ias_actual_hist))
            self.line_ias_tgt.set_data(t_list, list(self.ias_target_hist))
            self.line_ias_err.set_data(t_list, list(self.ias_error_hist))

            self.line_brunner.set_data(t_list, list(self.brunner_pos_hist))
            self.line_brunner_cmd.set_data(t_list, list(self.brunner_cmd_hist))

            t_max = t_list[-1]
            t_min = max(0.0, t_max - 30.0)
            self.ax1.set_xlim(t_min, max(30.0, t_max + 0.5))

            # Constant pitch scale [-15.0 .. +15.0 deg]
            self.ax1.set_ylim(-15.0, 15.0)

            spd_min = min(min(self.ias_actual_hist), min(self.ias_target_hist), min(self.ias_error_hist))
            spd_max = max(max(self.ias_actual_hist), max(self.ias_target_hist), max(self.ias_error_hist))
            spd_margin = max(8.0, (spd_max - spd_min) * 0.15)
            self.ax2.set_ylim(max(-20.0, spd_min - spd_margin), spd_max + spd_margin)

            self.ax3.set_ylim(-1.15, 1.15)

            if self.running and self.root.winfo_exists():
                try:
                    self.canvas.draw_idle()
                except Exception:
                    pass

        if self.running:
            try:
                if self.root.winfo_exists():
                    self.after_id = self.root.after(100, self.gui_tick)
            except Exception:
                pass

    # ----------------- Control Event Handlers -----------------
    def toggle_pid(self):
        with self.lock:
            self.pid_enabled = not self.pid_enabled
            enabled = self.pid_enabled
            mode = self.control_mode
            guidance = self.guidance_mode

        if enabled:
            self.pid.reset()
            with self.lock:
                curr_pitch = self.snap_pitch_actual
            self.ias_ctrl.reset(initial_pitch=curr_pitch)

            if mode == 3:
                curr_pos = self.cls.latest_position
                self.brunner_initial = curr_pos
                self.brunner_command = curr_pos
                self.brunner_velocity = 0.0
                self.cls.engage(curr_pos)
                self.pid.reset(initial_pos=curr_pos)
                self.bridge.set_mode(3)
                # Enable SimBridge 50 Hz wings-level (0.0 bank) hold (elevator writes remain 100% off)
                self.bridge.enable(True)
            else:
                self.bridge.set_mode(mode)
                self.bridge.enable(True)

            self._start_csv_logging()
            btn_txt = "STOP AIRSPEED HOLD" if guidance == "IAS" else "STOP PITCH PID"
            self.engage_btn.configure(text=btn_txt, bg="#da3633", activebackground="#f85149")
        else:
            self.bridge.enable(False)
            self.cls.release()
            self._stop_csv_logging()
            btn_txt = "START AIRSPEED HOLD" if guidance == "IAS" else "START PITCH PID"
            self.engage_btn.configure(text=btn_txt, bg="#238636", activebackground="#2ea043")

    def _start_csv_logging(self):
        """Starts high-rate 25 Hz CSV logging for this flight test run."""
        try:
            os.makedirs(self.runs_dir, exist_ok=True)
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            with self.lock:
                mode_str = self.guidance_mode
                tgt_val = f"{self.target_ias:.0f}kt" if mode_str == "IAS" else f"{self.target_pitch:+.1f}deg"
            filename = f"SMK_Run_{timestamp}_{mode_str}_{tgt_val}.csv"
            filepath = os.path.join(self.runs_dir, filename)

            f = open(filepath, "w", newline="", encoding="utf-8")
            writer = csv.writer(f)
            header = [
                "timestamp_iso",
                "elapsed_sec",
                "guidance_mode",
                "target_ias_kt",
                "actual_ias_kt",
                "ias_error_kt",
                "target_pitch_deg",
                "actual_pitch_deg",
                "pitch_error_deg",
                "brunner_cmd",
                "brunner_pos",
                "elevator_pos",
                "yoke_y_pos",
                "sent_axis",
                "throttle_pct",
                "altitude_ft",
                "total_weight_kg",
                "ambient_temp_c",
                "ias_kp",
                "ias_ki",
                "ias_kd",
                "pitch_kp",
                "pitch_ki",
                "pitch_kd",
                "simconnect_connected",
                "brunner_connected",
                "sim_paused",
            ]
            writer.writerow(header)
            f.flush()

            with self.lock:
                self.csv_file = f
                self.csv_writer = writer
                self.csv_filepath = filepath
                self.csv_row_count = 0
                self.csv_run_start_time = time.monotonic()
                self.csv_last_flush_time = time.monotonic()
                self.csv_status_msg = f"REC: {filename}"
            print(f"[CSV Log] Started recording run data to: {filepath}")
        except Exception as e:
            print(f"[CSV Log Error] Failed to start CSV recording: {e}")

    def _stop_csv_logging(self):
        """Cleanly flushes and closes active CSV file for the completed run."""
        with self.lock:
            f = self.csv_file
            count = self.csv_row_count
            filepath = self.csv_filepath
            self.csv_file = None
            self.csv_writer = None

        if f is not None:
            try:
                f.flush()
                f.close()
                fn = os.path.basename(filepath) if filepath else "run.csv"
                with self.lock:
                    self.csv_status_msg = f"Saved: {fn} ({count} pts)"
                print(f"[CSV Log] Run finished. Saved {count} rows to: {filepath}")
            except Exception as e:
                print(f"[CSV Log Error] Failed to close CSV file: {e}")

    def open_runs_folder(self):
        """Opens the runs folder in Windows File Explorer."""
        try:
            os.makedirs(self.runs_dir, exist_ok=True)
            if sys.platform == 'win32':
                os.startfile(self.runs_dir)
            else:
                subprocess.Popen(['xdg-open', self.runs_dir])
        except Exception as e:
            print(f"[Open Runs Error] Could not open runs folder: {e}")

    def on_guidance_change(self):
        pass

    def on_target_change(self, val):
        pass

    def on_target_entry(self, event=None):
        pass

    def set_target_pitch(self, val: float):
        pass

    def on_target_ias_change(self, val):
        try:
            with self.lock:
                self.target_ias = float(val)
            self.target_ias_entry.delete(0, tk.END)
            self.target_ias_entry.insert(0, f"{float(val):.0f}")
        except ValueError:
            pass

    def on_target_ias_entry(self, event=None):
        try:
            val = float(self.target_ias_entry.get().strip())
            with self.lock:
                self.target_ias = val
            if self.target_ias_slider['from'] <= val <= self.target_ias_slider['to']:
                self.target_ias_var.set(val)
        except ValueError:
            pass

    def set_target_ias(self, val: float):
        with self.lock:
            self.target_ias = float(val)
        self.target_ias_var.set(val)
        self.target_ias_entry.delete(0, tk.END)
        self.target_ias_entry.insert(0, f"{val:.0f}")

    def on_mode_change(self):
        with self.lock:
            self.control_mode = 3
        self.mode_var.set(3)

    def on_invert_change(self):
        inv = bool(self.invert_var.get())
        with self.lock:
            self.pid.invert = inv
            self.pid.integral = 0.0

    def on_pid_change(self, event=None):
        kp = float(self.kp_var.get())
        ki = float(self.ki_var.get())
        kd = float(self.kd_var.get())
        with self.lock:
            self.pid.kp = kp
            self.pid.ki = ki
            self.pid.kd = kd
        self.kp_entry.delete(0, tk.END)
        self.kp_entry.insert(0, f"{kp:.4f}".rstrip('0').rstrip('.'))
        self.ki_entry.delete(0, tk.END)
        self.ki_entry.insert(0, f"{ki:.5f}".rstrip('0').rstrip('.'))
        self.kd_entry.delete(0, tk.END)
        self.kd_entry.insert(0, f"{kd:.4f}".rstrip('0').rstrip('.'))

    def on_kp_entry(self, event=None):
        try:
            val = float(self.kp_entry.get().strip())
            with self.lock:
                self.pid.kp = val
            if self.kp_scale['from'] <= val <= self.kp_scale['to']:
                self.kp_var.set(val)
        except ValueError:
            pass

    def on_ki_entry(self, event=None):
        try:
            val = float(self.ki_entry.get().strip())
            with self.lock:
                self.pid.ki = val
            if self.ki_scale['from'] <= val <= self.ki_scale['to']:
                self.ki_var.set(val)
        except ValueError:
            pass

    def on_kd_entry(self, event=None):
        try:
            val = float(self.kd_entry.get().strip())
            with self.lock:
                self.pid.kd = val
            if self.kd_scale['from'] <= val <= self.kd_scale['to']:
                self.kd_var.set(val)
        except ValueError:
            pass

    def on_ias_pid_change(self, event=None):
        kp = float(self.ias_kp_var.get())
        ki = float(self.ias_ki_var.get())
        kd = float(self.ias_kd_var.get())
        with self.lock:
            self.ias_ctrl.kp = kp
            self.ias_ctrl.ki = ki
            self.ias_ctrl.kd = kd
        self.ias_kp_entry.delete(0, tk.END)
        self.ias_kp_entry.insert(0, f"{kp:.4f}".rstrip('0').rstrip('.'))
        self.ias_ki_entry.delete(0, tk.END)
        self.ias_ki_entry.insert(0, f"{ki:.5f}".rstrip('0').rstrip('.'))
        self.ias_kd_entry.delete(0, tk.END)
        self.ias_kd_entry.insert(0, f"{kd:.4f}".rstrip('0').rstrip('.'))

    def on_ias_kp_entry(self, event=None):
        try:
            val = float(self.ias_kp_entry.get().strip())
            with self.lock:
                self.ias_ctrl.kp = val
            if self.ias_kp_scale['from'] <= val <= self.ias_kp_scale['to']:
                self.ias_kp_var.set(val)
        except ValueError:
            pass

    def on_ias_ki_entry(self, event=None):
        try:
            val = float(self.ias_ki_entry.get().strip())
            with self.lock:
                self.ias_ctrl.ki = val
            if self.ias_ki_scale['from'] <= val <= self.ias_ki_scale['to']:
                self.ias_ki_var.set(val)
        except ValueError:
            pass

    def on_ias_kd_entry(self, event=None):
        try:
            val = float(self.ias_kd_entry.get().strip())
            with self.lock:
                self.ias_ctrl.kd = val
            if self.ias_kd_scale['from'] <= val <= self.ias_kd_scale['to']:
                self.ias_kd_var.set(val)
        except ValueError:
            pass

    def on_close(self):
        self.running = False
        self._stop_csv_logging()
        if self.after_id:
            try:
                self.root.after_cancel(self.after_id)
            except Exception:
                pass
            self.after_id = None
        self.cls.close()
        self.bridge.close()
        try:
            plt.close('all')
        except Exception:
            pass
        try:
            if self.root.winfo_exists():
                self.root.destroy()
        except Exception:
            pass


def main():
    root = tk.Tk()
    app = PitchPIDApp(root)
    try:
        root.mainloop()
    except (KeyboardInterrupt, SystemExit):
        app.on_close()


if __name__ == '__main__':
    main()
