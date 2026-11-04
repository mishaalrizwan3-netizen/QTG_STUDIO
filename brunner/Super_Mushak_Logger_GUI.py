#!/usr/bin/env python3
"""
Super_Mushak_Logger_GUI.py - PySide6 Flight Test Recording & Replay GUI
======================================================================
Architecture:
  - 100% Brunner Exclusive Controls: Flight controls (Pitch, Roll, Rudder) are
    read from and commanded to Brunner CLS2Sim hardware over UDP port 15090
    using src/brunner_interface.py. Zero P3D control writes.
  - P3D SimConnect Integration: Read-only live telemetry (Airspeed, Alt, Pitch,
    Bank, Heading, RPM, Throttle) and deterministic paused initial-state repositioning.
  - Instant Recording: Unpauses P3D and records verified live flight data at t=0.
  - Deterministic Automatic Replay: Teleports P3D while paused to exact Manual t=0,
    pre-positions physical Brunner stick/pedals, settles forces, and releases in lockstep.
  - Sim Pause Enforcement: Prepar3D is immediately paused when recording concludes (75s or STOP).
  - Official 6-Page QTG PDF Report: Generated via plot_graphs.py (plot_aligned_comparison).
"""

import os
import sys
import time
import csv
import json
import threading
import math
from pathlib import Path
from typing import Optional, Dict, Any, List, Callable, Tuple

from PySide6.QtCore import Qt, QTimer, Slot, Signal
from PySide6.QtGui import QFont, QKeySequence, QShortcut, QCursor
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QLabel, QPushButton, QProgressBar, QFrame,
    QGroupBox, QComboBox, QMessageBox, QFileDialog, QProgressDialog
)

# Ensure local directories are on import path
RESOURCE_DIR = str(Path(getattr(sys, "_MEIPASS", Path(__file__).resolve().parent)))
CURRENT_DIR = str(Path(sys.executable).resolve().parent if getattr(sys,"frozen",False) else Path(__file__).resolve().parent)
SRC_DIR = os.path.join(RESOURCE_DIR, "src")
if CURRENT_DIR not in sys.path:
    sys.path.insert(0, CURRENT_DIR)
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

# Exclusive Hardware and Simulator Drivers
from brunner_interface import BrunnerInterface
from p3d_simconnect_v15 import SimConnectClient
import plot_graphs
from replay_support import reference_weather_metar, ThrottleReplay, reference_altimeter_offset, estimate_replay_lead, replay_feedforward, interpolate_reference, build_dynamic_state, BrunnerPositionTracker, validate_initial_airspeed, validate_reference_power

# =====================================================================
# CONSTANTS & STAGES
# =====================================================================
CLS2SIM_HOST = "127.0.0.1"
CLS2SIM_PORT = 15090

SAMPLE_RATE_HZ  = 50
INTERVAL_SEC    = 1.0 / SAMPLE_RATE_HZ
LEAD_MS         = 80
LEAD_SAMPLES    = round((LEAD_MS / 1000.0) * SAMPLE_RATE_HZ)
GAIN            = 1.00
TEST_DURATION_S = 75

STAGE_NOT_CONNECTED    = "Not connected"
STAGE_CONNECTING       = "Connecting to Prepar3D"
STAGE_CONNECTED_READY  = "Connected & Ready to Record"
STAGE_ARMED            = "Armed - waiting for full throttle"
STAGE_INITIALIZING     = "Initializing aircraft"
STAGE_RECORDING        = "Recording"
STAGE_COMPLETED        = "Completed"
STAGE_STOPPED_BY_USER  = "Stopped by user"
STAGE_CONNECTION_ERROR = "Connection error"
STAGE_AUTO_BLOCKED     = "Automatic blocked"


def make_font(family: str = "Segoe UI", size: int = 10, bold: bool = False) -> QFont:
    f = QFont(family, size)
    f.setBold(bold)
    return f


class SuperMushakLoggerApp(QMainWindow):
    """Integrated PySide6 GUI Application for Super Mushshak QTG Flight Testing."""

    # Thread-safe Qt Signals for GUI updates from worker threads
    sig_update_stage = Signal(str, str)
    sig_record_progress = Signal(int)
    sig_recording_finished = Signal()

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Super Mushshak Automatic Testing Logger - PURE BRUNNER")
        self.resize(1080, 840)

        # Wire up thread-safe signals
        self.sig_update_stage.connect(self._update_stage_ui)
        self.sig_record_progress.connect(self._update_record_progress)
        self.sig_recording_finished.connect(self._on_recording_finished)

        # Output Directory
        if os.path.exists("D:\\"):
            self.data_dir = Path("D:/Super_Mushak_Logger_Data")
        else:
            self.data_dir = Path.home() / "Documents" / "Super_Mushak_Logger_Data"
        self.data_dir.mkdir(parents=True, exist_ok=True)

        # State Variables
        self.test_mode: str = ""  # "manual" or "auto"
        self.is_recording: bool = False
        self.record_seconds: int = 0
        self.target_duration_sec: int = TEST_DURATION_S

        # Hardware & Sim Objects (Only BrunnerInterface for hardware controls)
        self.brunner = BrunnerInterface(host=CLS2SIM_HOST, port=CLS2SIM_PORT)
        self.p3d: Optional[SimConnectClient] = None
        self.p3d_connected: bool = False

        # Live Telemetry Snapshot
        self.latest_telemetry: Dict[str, Any] = {}
        self.manual_t0_snapshot: Dict[str, Any] = {}
        self.auto_t0_snapshot: Dict[str, Any] = {}

        # High-Rate Recording Buffers
        self.rec_time: List[float] = []
        self.p_pos_buf: List[float] = []; self.p_force_buf: List[float] = []
        self.r_pos_buf: List[float] = []; self.r_force_buf: List[float] = []
        self.y_pos_buf: List[float] = []; self.y_force_buf: List[float] = []
        self.p3d_tele_buf: List[Dict[str, Any]] = []
        self.applied_force_buf = []
        self.virtual_force_buf = []
        self.replay_command_buf = []
        self.replay_axis_offsets: Tuple[float, float, float] = (0.0, 0.0, 0.0)

        # Reference data for Auto Replay
        self.selected_ref_csv: Optional[str] = None
        self.active_manual_ref_path: Optional[str] = None
        self.ref_time: List[float] = []
        self.ref_p_pos: List[float] = []
        self.ref_r_pos: List[float] = []
        self.ref_y_pos: List[float] = []
        self.ref_throt_pos: List[float] = []
        self.ref_manual_snapshot: Dict[str, Any] = {}

        # Background 50 Hz Recording / Replay Thread
        self.worker_thread: Optional[threading.Thread] = None
        self.stop_worker_event = threading.Event()

        # UI Refresh Timer (10 Hz for live meters & status display)
        self.ui_timer = QTimer(self)
        self.ui_timer.timeout.connect(self._on_ui_timer_tick)
        self.ui_timer.start(100)

        # Build UI layout & theme
        self._init_ui()
        self._apply_dark_theme()
        self._update_stage_ui(STAGE_NOT_CONNECTED)
        self._start_hardware_connections()

    def _play_beep(self, kind: str = "start"):
        """Plays non-blocking audible beep for recording start and stop."""
        def _beep_worker():
            try:
                import winsound
                if kind == "start":
                    # Distinct high chime on start (1200 Hz, 220 ms)
                    winsound.Beep(1200, 220)
                elif kind == "stop":
                    # Distinct descending two-tone chime on stop (900 Hz, 150 ms -> 600 Hz, 250 ms)
                    winsound.Beep(900, 150)
                    time.sleep(0.04)
                    winsound.Beep(600, 250)
            except Exception:
                try:
                    QApplication.beep()
                except Exception:
                    pass
        threading.Thread(target=_beep_worker, daemon=True).start()

    # =====================================================================
    # HARDWARE & P3D INITIALIZATION
    # =====================================================================
    def _start_hardware_connections(self):
        """Asynchronously connect to Brunner UDP and Prepar3D SimConnect."""
        def _connect_task():
            # 1. Connect Brunner
            self.brunner.connect()
            # 2. Connect P3D
            try:
                self.p3d = SimConnectClient(app_name="Super Mushshak GUI")
                threading.Thread(target=self.p3d.connect_and_run, daemon=True).start()
                t_end = time.time() + 3.0
                while time.time() < t_end:
                    if self.p3d.is_connected():
                        self.p3d_connected = True
                        break
                    time.sleep(0.15)
            except Exception as e:
                print(f"[P3D] Init note: {e}")

        threading.Thread(target=_connect_task, daemon=True).start()

    # =====================================================================
    # GUI LAYOUT
    # =====================================================================
    def _create_card(self, title: str, icon: str):
        card = QFrame()
        card.setObjectName("Card")
        outer_layout = QVBoxLayout(card)
        outer_layout.setContentsMargins(16, 14, 16, 14)
        outer_layout.setSpacing(8)

        lbl_head = QLabel(f"{icon}  {title}")
        lbl_head.setFont(make_font("Segoe UI", 10, bold=True))
        lbl_head.setStyleSheet("color: #b4befe;")
        outer_layout.addWidget(lbl_head)

        sep = QFrame()
        sep.setFrameShape(QFrame.Shape.HLine)
        sep.setStyleSheet("color: #313244;")
        outer_layout.addWidget(sep)

        content_layout = QVBoxLayout()
        content_layout.setSpacing(6)
        outer_layout.addLayout(content_layout)
        return card, content_layout

    def _init_ui(self):
        main_widget = QWidget(self)
        self.setCentralWidget(main_widget)
        main_layout = QVBoxLayout(main_widget)
        main_layout.setSpacing(14)
        main_layout.setContentsMargins(20, 20, 20, 20)

        # 1. Header Title Card
        header_card = QFrame()
        header_card.setObjectName("HeaderCard")
        header_layout = QHBoxLayout(header_card)
        header_layout.setContentsMargins(20, 14, 20, 14)

        title_vbox = QVBoxLayout()
        app_title = QLabel("Super Mushshak Automatic Testing Logger - PURE BRUNNER")
        app_title.setFont(make_font("Segoe UI", 16, bold=True))
        app_title.setStyleSheet("color: #f5e0dc;")

        app_subtitle = QLabel("CLS2Sim Physical Closed-Loop (UDP 15090) | Prepar3D Telemetry + Throttle Replay | Flight Axes: Brunner Only")
        app_subtitle.setFont(make_font("Segoe UI", 10))
        app_subtitle.setStyleSheet("color: #a6adc8;")
        title_vbox.addWidget(app_title)
        title_vbox.addWidget(app_subtitle)

        header_layout.addLayout(title_vbox)
        header_layout.addStretch()

        self.stage_badge = QLabel("Not connected")
        self.stage_badge.setFont(make_font("Segoe UI", 11, bold=True))
        self.stage_badge.setContentsMargins(16, 8, 16, 8)
        self.stage_badge.setAlignment(Qt.AlignmentFlag.AlignCenter)
        header_layout.addWidget(self.stage_badge)
        main_layout.addWidget(header_card)

        # 2. Status & Telemetry Grid
        grid_layout = QGridLayout()
        grid_layout.setSpacing(14)

        # Card 1: Connection Status
        card_conn, card_conn_layout = self._create_card("Connection Status", "📡")
        self.lbl_conn_status = QLabel("Checking connections...")
        self.lbl_conn_status.setFont(make_font("Segoe UI", 11, bold=True))
        self.lbl_conn_status.setStyleSheet("color: #89b4fa;")

        self.lbl_aircraft = QLabel("Aircraft: Prepar3D v5")
        self.lbl_aircraft.setFont(make_font("Segoe UI", 9))
        self.lbl_aircraft.setStyleSheet("color: #bac2de;")

        self.lbl_cls_status = QLabel(f"CLS2Sim: UDP {CLS2SIM_HOST}:{CLS2SIM_PORT}")
        self.lbl_cls_status.setFont(make_font("Segoe UI", 9))
        self.lbl_cls_status.setStyleSheet("color: #bac2de;")

        card_conn_layout.addWidget(self.lbl_conn_status)
        card_conn_layout.addWidget(self.lbl_aircraft)
        card_conn_layout.addWidget(self.lbl_cls_status)
        grid_layout.addWidget(card_conn, 0, 0)

        # Card 2: Throttle & Engine Power
        card_eng, card_eng_layout = self._create_card("Throttle & Engine Power", "⚡")
        self.lbl_throttle_val = QLabel("0.0 %")
        self.lbl_throttle_val.setFont(make_font("Segoe UI", 18, bold=True))
        self.lbl_throttle_val.setStyleSheet("color: #89dceb;")

        self.throttle_bar = QProgressBar()
        self.throttle_bar.setRange(0, 100)
        self.throttle_bar.setValue(0)
        self.throttle_bar.setTextVisible(False)
        self.throttle_bar.setFixedHeight(8)

        self.lbl_engine_rpm = QLabel("Engine RPM: 0 | Status: IDLE/OFF")
        self.lbl_engine_rpm.setFont(make_font("Segoe UI", 9))
        self.lbl_engine_rpm.setStyleSheet("color: #bac2de;")

        card_eng_layout.addWidget(self.lbl_throttle_val)
        card_eng_layout.addWidget(self.throttle_bar)
        card_eng_layout.addWidget(self.lbl_engine_rpm)
        grid_layout.addWidget(card_eng, 0, 1)

        # Card 3: Aircraft Attitude & Flight Dynamics
        card_dyn, card_dyn_layout = self._create_card("Aircraft Attitude & Flight Dynamics", "📐")
        self.lbl_ias_alt = QLabel("Airspeed (IAS): 0.0 kt | Altitude: 0 ft")
        self.lbl_ias_alt.setFont(make_font("Segoe UI", 11, bold=True))
        self.lbl_ias_alt.setStyleSheet("color: #a6e3a1;")

        self.lbl_pitch_bank = QLabel("Pitch: +0.0° | Bank: -0.0° | Heading: 0.0°")
        self.lbl_pitch_bank.setFont(make_font("Segoe UI", 9))
        self.lbl_pitch_bank.setStyleSheet("color: #bac2de;")

        self.lbl_brunner_sticks = QLabel("Brunner Controls: Pitch: +0.00 | Roll: +0.00 | Rudder: +0.00")
        self.lbl_brunner_sticks.setFont(make_font("Segoe UI", 9))
        self.lbl_brunner_sticks.setStyleSheet("color: #f9e2af;")

        card_dyn_layout.addWidget(self.lbl_ias_alt)
        card_dyn_layout.addWidget(self.lbl_pitch_bank)
        card_dyn_layout.addWidget(self.lbl_brunner_sticks)
        grid_layout.addWidget(card_dyn, 1, 0)

        # Card 4: Recording Telemetry
        card_rec, card_rec_layout = self._create_card("Recording Telemetry", "⏱")
        self.lbl_rec_status = QLabel("Idle - Select a Test Mode below to start")
        self.lbl_rec_status.setFont(make_font("Segoe UI", 11, bold=True))
        self.lbl_rec_status.setStyleSheet("color: #cdd6f4;")

        self.lbl_rec_progress = QLabel("0 / 75 seconds (0 rows)")
        self.lbl_rec_progress.setFont(make_font("Segoe UI", 9))
        self.lbl_rec_progress.setStyleSheet("color: #bac2de;")

        self.rec_progress = QProgressBar()
        self.rec_progress.setRange(0, TEST_DURATION_S)
        self.rec_progress.setValue(0)
        self.rec_progress.setFixedHeight(10)

        card_rec_layout.addWidget(self.lbl_rec_status)
        card_rec_layout.addWidget(self.lbl_rec_progress)
        card_rec_layout.addWidget(self.rec_progress)
        grid_layout.addWidget(card_rec, 1, 1)

        main_layout.addLayout(grid_layout)

        # 3. CSV Data Output Location
        box_data = QGroupBox("Data Output Location")
        box_data_layout = QHBoxLayout(box_data)
        self.lbl_data_dir = QLabel(f"📁 {self.data_dir}")
        self.lbl_data_dir.setFont(make_font("Segoe UI", 9))
        self.lbl_data_dir.setStyleSheet("color: #f9e2af;")

        btn_open_folder = QPushButton("Open Folder")
        btn_open_folder.setCursor(QCursor(Qt.CursorShape.PointingHandCursor))
        btn_open_folder.clicked.connect(self._open_data_folder)

        box_data_layout.addWidget(self.lbl_data_dir)
        box_data_layout.addStretch()
        box_data_layout.addWidget(btn_open_folder)
        main_layout.addWidget(box_data)

        # 4. Manual Reference for Automatic Replay
        box_ref = QGroupBox("Manual Reference for Automatic Replay")
        box_ref_layout = QVBoxLayout(box_ref)

        ref_sel_hbox = QHBoxLayout()
        self.cmb_manual_reference = QComboBox()
        self.cmb_manual_reference.setFont(make_font("Segoe UI", 9, bold=True))
        self.cmb_manual_reference.currentIndexChanged.connect(self._on_manual_reference_selection_changed)

        btn_refresh_refs = QPushButton("↻ Refresh Manuals")
        btn_refresh_refs.setCursor(QCursor(Qt.CursorShape.PointingHandCursor))
        btn_refresh_refs.clicked.connect(self._refresh_manual_reference_selector)

        ref_sel_hbox.addWidget(self.cmb_manual_reference, stretch=4)
        ref_sel_hbox.addWidget(btn_refresh_refs, stretch=1)
        box_ref_layout.addLayout(ref_sel_hbox)

        self.lbl_manual_ref_info = QLabel("Select a previously recorded Manual flight test above.")
        self.lbl_manual_ref_info.setFont(make_font("Segoe UI", 9))
        self.lbl_manual_ref_info.setStyleSheet("color: #a6adc8;")
        box_ref_layout.addWidget(self.lbl_manual_ref_info)
        main_layout.addWidget(box_ref)

        # 5. Flight Test Execution Controls
        box_ctrl = QGroupBox("Flight Test Execution Controls")
        box_ctrl_layout = QHBoxLayout(box_ctrl)
        box_ctrl_layout.setSpacing(12)

        self.btn_manual_test = QPushButton("🎮 Manual Testing")
        self.btn_manual_test.setFixedHeight(48)
        self.btn_manual_test.setCursor(QCursor(Qt.CursorShape.PointingHandCursor))
        self.btn_manual_test.clicked.connect(self.start_manual_test)

        self.btn_auto_test = QPushButton("🤖 Automatic Testing")
        self.btn_auto_test.setFixedHeight(48)
        self.btn_auto_test.setCursor(QCursor(Qt.CursorShape.PointingHandCursor))
        self.btn_auto_test.clicked.connect(self.start_auto_test_only)

        self.btn_record = QPushButton("🔴 Start Recording (Press SPACEBAR)")
        self.btn_record.setFixedHeight(48)
        self.btn_record.setCursor(QCursor(Qt.CursorShape.PointingHandCursor))
        self.btn_record.clicked.connect(self.arm_recording)
        self.btn_record.setVisible(False)

        self.btn_stop = QPushButton("🛑 STOP")
        self.btn_stop.setFixedHeight(48)
        self.btn_stop.setFixedWidth(130)
        self.btn_stop.setCursor(QCursor(Qt.CursorShape.PointingHandCursor))
        self.btn_stop.clicked.connect(self.stop_test)
        self.btn_stop.setEnabled(False)

        box_ctrl_layout.addWidget(self.btn_manual_test, stretch=2)
        box_ctrl_layout.addWidget(self.btn_auto_test, stretch=2)
        box_ctrl_layout.addWidget(self.btn_record, stretch=4)
        box_ctrl_layout.addWidget(self.btn_stop, stretch=1)
        main_layout.addWidget(box_ctrl)

        # Spacebar Shortcut for Instant Start Recording
        self.shortcut_space = QShortcut(QKeySequence(Qt.Key.Key_Space), self)
        self.shortcut_space.activated.connect(self._on_spacebar_pressed)

        # Populate manual reference catalog
        self._refresh_manual_reference_selector()

    def _apply_dark_theme(self):
        self.setStyleSheet("""
            QMainWindow { background-color: #1e1e2e; }
            QWidget { color: #cdd6f4; font-family: "Segoe UI", sans-serif; font-size: 13px; }
            QGroupBox { border: 1px solid #313244; border-radius: 8px; margin-top: 10px; padding-top: 14px; font-weight: bold; color: #cdd6f4; }
            QGroupBox::title { subcontrol-origin: margin; left: 14px; padding: 0 4px; }
            QFrame#HeaderCard { background-color: #252538; border: 1px solid #313244; border-radius: 10px; }
            QFrame#Card { background-color: #252538; border: 1px solid #313244; border-radius: 10px; }
            QComboBox { background-color: #313244; color: #cdd6f4; border: 1px solid #45475a; border-radius: 6px; padding: 6px 12px; font-weight: bold; }
            QComboBox::drop-down { border: none; }
            QComboBox QAbstractItemView { background-color: #252538; color: #cdd6f4; selection-background-color: #45475a; }
            QPushButton { background-color: #313244; color: #cdd6f4; border: 1px solid #45475a; border-radius: 6px; padding: 6px 12px; font-weight: bold; }
            QPushButton:hover { background-color: #45475a; }
            QProgressBar { background-color: #313244; border-radius: 4px; text-align: center; }
            QProgressBar::chunk { background-color: #89dceb; border-radius: 4px; }
        """)
        self.btn_manual_test.setStyleSheet("QPushButton { background-color: #89b4fa; color: #11111b; font-weight: bold; border-radius: 6px; font-size: 13px; } QPushButton:hover { background-color: #b4befe; } QPushButton:disabled { background-color: #45475a; color: #7f849c; }")
        self.btn_auto_test.setStyleSheet("QPushButton { background-color: #cba6f7; color: #11111b; font-weight: bold; border-radius: 6px; font-size: 13px; } QPushButton:hover { background-color: #f5c2e7; } QPushButton:disabled { background-color: #45475a; color: #7f849c; }")
        self.btn_record.setStyleSheet("QPushButton { background-color: #a6e3a1; color: #11111b; font-weight: bold; border-radius: 6px; font-size: 13px; } QPushButton:hover { background-color: #94e2d5; }")
        self.btn_stop.setStyleSheet("QPushButton { background-color: #f38ba8; color: #11111b; font-weight: bold; border-radius: 6px; font-size: 13px; } QPushButton:hover { background-color: #eba0ac; } QPushButton:disabled { background-color: #45475a; color: #7f849c; }")

    # =====================================================================
    # UI STAGE BADGE & STATUS
    # =====================================================================
    @Slot(str, str)
    def _update_stage_ui(self, stage: str, details: str = ""):
        self.current_stage = stage
        self.stage_badge.setText(stage)
        badge_styles = {
            STAGE_NOT_CONNECTED: ("background-color: #313244; color: #a6adc8; border-radius: 14px;", "Ready to Connect", "#89b4fa"),
            STAGE_CONNECTING: ("background-color: #89b4fa; color: #11111b; border-radius: 14px;", "Connecting...", "#89b4fa"),
            STAGE_CONNECTED_READY: ("background-color: #a6e3a1; color: #11111b; border-radius: 14px;", "Ready to Record", "#a6e3a1"),
            STAGE_INITIALIZING: ("background-color: #fab387; color: #11111b; border-radius: 14px;", "Initializing...", "#fab387"),
            STAGE_RECORDING: ("background-color: #f38ba8; color: #11111b; border-radius: 14px;", "Recording LIVE", "#f38ba8"),
            STAGE_COMPLETED: ("background-color: #cba6f7; color: #11111b; border-radius: 14px;", "Completed", "#a6e3a1"),
            STAGE_STOPPED_BY_USER: ("background-color: #fab387; color: #11111b; border-radius: 14px;", "Stopped", "#fab387"),
            STAGE_AUTO_BLOCKED: ("background-color: #fab387; color: #11111b; border-radius: 14px;", "Connected - preparation failed", "#fab387"),
            STAGE_CONNECTION_ERROR: ("background-color: #f38ba8; color: #11111b; border-radius: 14px;", "Error", "#f38ba8"),
        }
        b_style, conn_text, conn_color = badge_styles.get(stage, ("background-color: #313244; color: #cdd6f4;", "Connected", "#a6e3a1"))
        self.stage_badge.setStyleSheet(b_style)
        self.lbl_conn_status.setText(conn_text)
        self.lbl_conn_status.setStyleSheet(f"color: {conn_color};")

        # Button Visibilities and dynamic labeling
        if stage in [STAGE_NOT_CONNECTED, STAGE_COMPLETED, STAGE_STOPPED_BY_USER, STAGE_CONNECTION_ERROR, STAGE_AUTO_BLOCKED]:
            self.btn_manual_test.setText("🎮 Manual Testing")
            self.btn_manual_test.setVisible(True)
            self.btn_manual_test.setEnabled(True)
            self.btn_auto_test.setText("🤖 Automatic Testing")
            self.btn_auto_test.setVisible(True)
            self.btn_auto_test.setEnabled(True)
            self.btn_record.setVisible(False)
            self.btn_stop.setEnabled(False)
            if stage in (STAGE_CONNECTION_ERROR, STAGE_AUTO_BLOCKED) and details:
                self.lbl_rec_status.setWordWrap(True)
                self.lbl_rec_status.setText(details)
                self.lbl_rec_status.setStyleSheet("color: #f38ba8;")
        elif stage == STAGE_INITIALIZING:
            self.rec_progress.setValue(0)
            self.lbl_rec_progress.setText("Preparing - no new samples recorded")
            if self.test_mode == "manual":
                self.btn_manual_test.setText("Teleporting P3D...")
            else:
                self.btn_auto_test.setText("Preparing & Aligning...")
            self.btn_manual_test.setEnabled(False)
            self.btn_auto_test.setEnabled(False)
            self.btn_record.setVisible(False)
            self.btn_stop.setEnabled(True)
            self.lbl_rec_status.setText(details or "Preparing initial conditions...")
            self.lbl_rec_status.setStyleSheet("color: #fab387;")
            QApplication.processEvents()
        elif stage == STAGE_CONNECTED_READY:
            self.btn_manual_test.setVisible(False)
            self.btn_auto_test.setVisible(False)
            mode_label = "MANUAL" if self.test_mode == "manual" else "AUTOMATIC"
            self.btn_record.setText(f"START {mode_label} RECORDING (Press SPACEBAR)")
            self.btn_record.setVisible(True)
            self.btn_record.setEnabled(True)
            self.btn_record.setFocus()
            self.btn_stop.setEnabled(True)
            self.lbl_rec_status.setText((details + "\n" if details else "") + f"Aircraft Initialized & Ready! Press START {mode_label} RECORDING or SPACEBAR")
            self.lbl_rec_status.setStyleSheet("color: #a6e3a1;")
            QApplication.processEvents()
        elif stage == STAGE_RECORDING:
            self.btn_manual_test.setVisible(False)
            self.btn_auto_test.setVisible(False)
            self.btn_record.setVisible(False)
            self.btn_stop.setEnabled(True)
            self.lbl_rec_status.setText(f"RECORDING TELEMETRY ({self.test_mode.upper()} MODE) @ 50 Hz")
            self.lbl_rec_status.setStyleSheet("color: #f38ba8;")

    # =====================================================================
    # 10 Hz UI TIMER TICK (LIVE METERS)
    # =====================================================================
    def _on_ui_timer_tick(self):
        # 1. Read live P3D Telemetry
        if self.p3d and self.p3d.is_connected():
            if not self.is_recording:
                # Query fresh telemetry while paused so meters reflect aircraft state immediately
                self.p3d.request_telemetry_once()

            tele = self.p3d.get_telemetry()
            self.latest_telemetry = tele
            ias = tele.get("airspeed_indicated", 0.0)
            # pressure_alt is the bridge's PLANE ALTITUDE (MSL), not pressure altitude.
            alt = tele.get("pressure_alt", tele.get("indicated_altitude", 0.0))
            pitch = tele.get("pitch_angle", 0.0)
            bank = tele.get("bank_angle", 0.0)
            hdg = tele.get("true_heading", 0.0)
            # Read-only engine instrumentation; flight controllers stay exclusively on Brunner.
            rpm = tele.get("engine_rpm", 0.0) or (tele.get("eng_rpms", [0])[0])

            throttle = float(tele.get("engine_throttle_percent", float("nan")))
            if math.isfinite(throttle):
                self.lbl_throttle_val.setText(f"Throttle: {throttle:.1f} %")
                self.throttle_bar.setValue(round(throttle))
            else:
                self.lbl_throttle_val.setText("Throttle: unavailable")
                self.throttle_bar.setValue(0)
            self.lbl_engine_rpm.setText(f"Engine RPM: {rpm:>4.0f} | Status: {'RUNNING' if rpm > 300 else 'IDLE/OFF'}")
            self.lbl_ias_alt.setText(f"Airspeed (IAS): {ias:>5.1f} kt | Altitude (MSL): {alt:>6.0f} ft")
            self.lbl_pitch_bank.setText(f"Pitch: {pitch:>+5.1f}° | Bank: {bank:>+5.1f}° | Heading: {hdg:>5.1f}°")

        # 2. Read live Brunner Controls exclusively via UDP 15090
        # When recording, the worker thread polls hardware to avoid socket collisions
        if not self.is_recording and not getattr(self,"_external_control_active",False) and self.current_stage != STAGE_INITIALIZING and self.brunner and self.brunner.sock:
            try:
                pp, _, _ = self.brunner.read_axis(self.brunner.AXIS_PITCH)
                rp, _, _ = self.brunner.read_axis(self.brunner.AXIS_ROLL)
                yp, _, _ = self.brunner.read_axis(self.brunner.AXIS_RUDDER)
                self.lbl_brunner_sticks.setText(f"Brunner Controls: Pitch: {pp:+.2f} | Roll: {rp:+.2f} | Rudder: {yp:+.2f}")
            except Exception:
                pass

    # =====================================================================
    # MANUAL TESTING FLOW
    # =====================================================================
    @Slot()
    def start_manual_test(self):
        """Operator clicks Manual Testing: teleports aircraft airborne and prepares recording."""
        self.test_mode = "manual"
        self.stop_worker_event.clear()
        self._update_stage_ui(STAGE_INITIALIZING, "Applying selected Manual initial conditions..." if getattr(self,"manual_initial_conditions",None) is not None else "Placing aircraft airborne at 3000ft @ 80kt (Rahwali)...")

        def _prep_manual():
            try:
                # 1. Connect Brunner
                if not self.brunner.sock:
                    self.brunner.connect()

                self.brunner.prepare_manual_mode()

                configured=getattr(self,"manual_initial_conditions",None)
                if configured is not None:
                    from qtg_common_engine import apply_manual_conditions
                    actual=apply_manual_conditions(self,configured)
                    self.sig_update_stage.emit(STAGE_CONNECTED_READY,
                        f"Manual settings applied: altitude (MSL) {actual['pressure_alt']:.0f} ft "
                        f"(requested {configured['alt_ft']:.0f} ft), initial airspeed {configured['ias_kt']:.0f} kt. "
                        "Simulator remains paused; indicated instruments may refresh after unpausing.")
                    return

                # 2. Teleport aircraft in Prepar3D to airborne initial conditions
                # Wait up to 3s for P3D connection if needed
                t_wait = time.time() + 3.0
                while time.time() < t_wait:
                    if self.p3d and self.p3d.is_connected():
                        break
                    time.sleep(0.15)

                if self.p3d and self.p3d.is_connected():
                    print("[Manual Prep] Teleporting P3D: Lat=32.23367, Lon=74.12350, Alt=3000ft, Speed=80kt, Hdg=48.4°...", flush=True)
                    self.p3d.set_position_airborne(
                        alt_ft=3000.0,
                        speed_kts=80.0,
                        pitch_deg=0.0,
                        bank_deg=0.0,
                        hdg_deg=48.4,
                        lat=32.23367,
                        lon=74.12350,
                    )
                    time.sleep(0.3)
                    self._wait_for_initial_altitude(3000.0)
                    initial=self._refresh_paused_telemetry()
                    print("[Manual Prep] Waiting for local airflow and altimeter to recover before arming.",flush=True)
                    self._commit_initial_position(reference_snapshot={"telemetry":initial})
                    actual=self._refresh_paused_telemetry()
                    print(f"[Manual Prep] Actual ready state: MSL={actual.get('pressure_alt')} ft, "
                          f"IAS={actual.get('airspeed_indicated')} kt, RPM={actual.get('engine_rpm')}.",flush=True)


                # Ready for recording (signal dispatched to main UI thread)
                print("[Manual Prep] Initial conditions set and aircraft ready for recording.", flush=True)
                self.sig_update_stage.emit(STAGE_CONNECTED_READY, "")
            except Exception as e:
                print(f"[Manual Prep Error] {e}", flush=True)
                self.sig_update_stage.emit(STAGE_CONNECTION_ERROR, str(e))

        self.preparation_thread=threading.Thread(target=_prep_manual, daemon=True)
        self.preparation_thread.start()

    # =====================================================================
    # AUTOMATIC TESTING FLOW
    # =====================================================================
    @Slot()
    def start_auto_test_only(self):
        """Operator clicks Automatic Testing: loads selected reference and prepares P3D/Brunner."""
        ref_path = self.selected_ref_csv
        if not ref_path or not os.path.exists(ref_path):
            QMessageBox.warning(self, "No Manual Reference", "Please select a recorded Manual CSV reference from the dropdown first.")
            return

        self.active_manual_ref_path = ref_path
        self.test_mode = "auto"
        self.replay_axis_offsets: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.stop_worker_event.clear()
        self._update_stage_ui(STAGE_INITIALIZING, "Loading reference and teleporting Prepar3D...")

        def _prep_auto():
            preparation_started = False
            try:
                self.ref_p_pos.clear(); self.ref_r_pos.clear(); self.ref_y_pos.clear()
                self.ref_throt_pos.clear()
                self.ref_time.clear()
                first_row = {}
                with open(ref_path, "r", encoding="utf-8") as f:
                    reader = csv.DictReader(f)
                    for idx, row in enumerate(reader):
                        if idx == 0:
                            first_row = dict(row)
                        
                        stamp = float(row.get("Time (s)", idx * INTERVAL_SEC))
                        if not math.isfinite(stamp) or (self.ref_time and stamp < self.ref_time[-1]):
                            raise ValueError("Manual reference times must be finite and must not run backwards.")
                        if self.ref_time and stamp == self.ref_time[-1]:
                            print(f"[Auto Prep] Skipping duplicate timestamp at CSV row {idx + 2}: {stamp}s")
                            continue
                        self.ref_time.append(stamp)

                        # Pitch: Prioritize exact normalized float column (-1.0 to +1.0)
                        p_val = (row.get("Pitch_Manual_Pos") or 
                                 row.get("Pitch_Man_Pos") or 
                                 row.get("Pitch_Pos"))
                        if p_val is not None and str(p_val).strip() != "":
                            p_norm = float(p_val)
                        else:
                            p_mm = float(row.get("Joystick Pitch Position (mm)", 0.0) or 0.0)
                            p_norm = p_mm / 100.0
                        self.ref_p_pos.append(max(-1.0, min(1.0, p_norm)))

                        # Roll: Prioritize exact normalized float column (-1.0 to +1.0)
                        r_val = (row.get("Roll_Manual_Pos") or 
                                 row.get("Roll_Man_Pos") or 
                                 row.get("Roll_Pos"))
                        if r_val is not None and str(r_val).strip() != "":
                            r_norm = float(r_val)
                        else:
                            r_mm = float(row.get("Joystick Roll Position (mm)", 0.0) or 0.0)
                            r_norm = r_mm / 100.0
                        self.ref_r_pos.append(max(-1.0, min(1.0, r_norm)))

                        # Rudder: Prioritize exact normalized float column (-1.0 to +1.0)
                        y_val = (row.get("Rudder_Manual_Pos") or 
                                 row.get("Rudder_Man_Pos") or 
                                 row.get("Rudder_Pos"))
                        if y_val is not None and str(y_val).strip() != "":
                            y_norm = float(y_val)
                        else:
                            y_mm = float(row.get("Rudder Pedal Position (mm)", 0.0) or 0.0)
                            y_norm = y_mm / 100.0
                        self.ref_y_pos.append(max(-1.0, min(1.0, y_norm)))

                        # Throttle replay
                        th_val = next((row.get(key) for key in ("Throttle 1 Position (%)","Throttle_Manual","Throttle")
                                       if row.get(key) is not None and str(row.get(key)).strip()),None)
                        if th_val is None or not math.isfinite(float(th_val)) or not 0 <= float(th_val) <= 100:
                            raise ValueError("Manual reference has a missing/invalid throttle sample; replay was not started.")
                        self.ref_throt_pos.append(float(th_val))

                if not self.ref_time:
                    raise ValueError("Manual reference contains no samples.")
                origin = self.ref_time[0]
                self.ref_time = [t - origin for t in self.ref_time]
                SuperMushakLoggerApp._load_replay_timing(self, ref_path)

                # Load initial conditions JSON if available
                json_ref = os.path.splitext(ref_path)[0] + "_init_conditions.json"
                if not os.path.exists(json_ref):
                    json_ref = os.path.splitext(ref_path)[0] + "_initial_state.json"
                if os.path.exists(json_ref):
                    try:
                        with open(json_ref, "r", encoding="utf-8") as jf:
                            data = json.load(jf)
                            self.ref_manual_snapshot = data.get("manual", data)
                    except Exception:
                        self.ref_manual_snapshot = {}
                else:
                    self.ref_manual_snapshot = {}

                if not self.ref_manual_snapshot and first_row:
                    self.ref_manual_snapshot = {
                        "altitude": float(first_row.get("Indicated Altitude (ft)") or first_row.get("Pressure Alt (ft)") or 3000.0),
                        "airspeed": float(first_row.get("Indicated Airspeed (kts)") or 80.0),
                        "pitch_ang": float(first_row.get("Pitch Angle (deg)") or 0.0),
                        "bank_ang": float(first_row.get("Bank Angle (deg)") or 0.0),
                        "heading": float(first_row.get("Magnetic Heading (deg)") or 48.4),
                        "throttle": float(first_row.get("Throttle 1 Position (%)") or 80.0),
                        "rpm": float(first_row.get("Engine 1 RPM") or 2670.0),
                        "gross_wt": float(first_row.get("Gross Wt (kg)") or 851.4),
                        "fuel_qty": float(first_row.get("Total Fuel Quantity (kg)") or 112.2),
                        "baro_press": float(first_row.get("Baro Press (hPa)") or 1013.2),
                        "oat": float(first_row.get("OAT (deg C)") or 11.4),
                        "wind_dir": float(first_row.get("Wind Dir (mag,deg)") or 0.0),
                        "wind_spd": float(first_row.get("Wind Spd (kt)") or 0.0),
                    }

                # -------------------------------------------------------------
                # 5-STEP AUTOMATIC INITIALIZATION: BRUNNER FIRST, P3D SECOND
                # -------------------------------------------------------------
                
                # STEP 1: Load the selected Manual reference
                print("[Auto Prep Step 1/5] Loading selected Manual reference...", flush=True)
                p0 = self.ref_p_pos[0]
                r0 = self.ref_r_pos[0]
                y0 = self.ref_y_pos[0]

                m_alt = float(self.ref_manual_snapshot.get("altitude_msl") or
                              first_row.get("Altitude MSL (ft)") or
                              self.ref_manual_snapshot.get("altitude") or 
                              self.ref_manual_snapshot.get("altitude_ft") or 
                              first_row.get("Indicated Altitude (ft)") or 
                              first_row.get("Altitude_Manual") or 3000.0)

                m_spd = float(self.ref_manual_snapshot.get("airspeed") or 
                              self.ref_manual_snapshot.get("ias_kts") or 
                              first_row.get("Indicated Airspeed (kts)") or 
                              first_row.get("Airspeed_Manual") or 80.0)

                m_pitch = float(self.ref_manual_snapshot.get("pitch_ang") or 
                                self.ref_manual_snapshot.get("pitch_deg") or 
                                first_row.get("Pitch Angle (deg)") or 
                                first_row.get("Pitch_Angle_Manual") or 0.0)

                m_bank = float(self.ref_manual_snapshot.get("bank_ang") or 
                               self.ref_manual_snapshot.get("bank_deg") or 
                               first_row.get("Bank Angle (deg)") or 
                               first_row.get("Bank_Angle_Manual") or 0.0)

                m_hdg = float(self.ref_manual_snapshot.get("heading") or 
                              self.ref_manual_snapshot.get("true_heading_deg") or 
                              first_row.get("Magnetic Heading (deg)") or 
                              first_row.get("Heading_Manual") or 48.4)

                m_throt = float(self.ref_throt_pos[0])
                self.ref_manual_snapshot['throttle']=m_throt

                m_lat = float(self.ref_manual_snapshot.get("lat") or 
                              self.ref_manual_snapshot.get("latitude_deg") or 32.23367)

                m_lon = float(self.ref_manual_snapshot.get("lon") or 
                              self.ref_manual_snapshot.get("longitude_deg") or 74.12350)

                print(f"[Auto Prep Step 1/5] Reference verified: {len(self.ref_time)} samples. Initial stick: P={p0:+.3f}, R={r0:+.3f}, Y={y0:+.3f}. Flight: Alt={m_alt:.0f}ft, IAS={m_spd:.1f}kt, Hdg={m_hdg:.1f}°.", flush=True)

                # Restore and verify the recorded power conditions while paused,
                # before moving the rig. Flight-control axes remain Brunner-only.
                if not self.p3d or not self.p3d.is_connected():
                    raise RuntimeError("Prepar3D is not connected.")
                preparation_started = True
                self.p3d.pause_sim()
                SuperMushakLoggerApp._restore_reference_weather(self)
                self._prepare_reference_power()
                print("[Auto Prep] Recorded initial power restored and verified; P3D paused BEFORE Brunner movement.", flush=True)

                # STEP 2: Prepare BRUNNER
                print("[Auto Prep Step 2/5] Preparing BRUNNER hardware and autopilot configuration...", flush=True)
                curr_p, _, _ = self.brunner.read_axis(self.brunner.AXIS_PITCH)
                curr_r, _, _ = self.brunner.read_axis(self.brunner.AXIS_ROLL)
                curr_y, _, _ = self.brunner.read_axis(self.brunner.AXIS_RUDDER)
                self.brunner.configure_replay_mode()

                # STEP 3: Move and verify the physical controls (ramp over ~2 seconds)
                print(f"[Auto Prep Step 3/5] Ramping BRUNNER to Manual starting position (P={p0:+.3f}, R={r0:+.3f}, Y={y0:+.3f}) over ~2 seconds...", flush=True)
                start_p = curr_p if math.isfinite(curr_p) else 0.0
                start_r = curr_r if math.isfinite(curr_r) else 0.0
                start_y = curr_y if math.isfinite(curr_y) else 0.0

                RAMP_STEPS = 40
                RAMP_DT = 0.05
                for step in range(1, RAMP_STEPS + 1):
                    if self.stop_worker_event.is_set():
                        self.brunner.release_controls()
                        raise RuntimeError("Automatic preparation cancelled by user.")
                    alpha = step / float(RAMP_STEPS)
                    s_alpha = 0.5 * (1.0 - math.cos(math.pi * alpha))
                    cmd_p = start_p + s_alpha * (p0 - start_p)
                    cmd_r = start_r + s_alpha * (r0 - start_r)
                    cmd_y = start_y + s_alpha * (y0 - start_y)
                    self.brunner.move_actuators(cmd_p, cmd_r, cmd_y)
                    time.sleep(RAMP_DT)

                self._settle_reference_controls()
                fb_p, fb_pf, _ = self.brunner.read_axis(self.brunner.AXIS_PITCH)
                fb_r, fb_rf, _ = self.brunner.read_axis(self.brunner.AXIS_ROLL)
                fb_y, fb_yf, _ = self.brunner.read_axis(self.brunner.AXIS_RUDDER)
                print(f"[Auto Prep Step 3/5] BRUNNER controls settled at P={fb_p:+.3f}, R={fb_r:+.3f}, Y={fb_y:+.3f}. Forces: P={fb_pf:+.2f}N, R={fb_rf:+.2f}N, Y={fb_yf:+.2f}N.", flush=True)

                # STEP 4: Initialize P3D
                print(f"[Auto Prep Step 4/5] Initializing Prepar3D simulation (Paused, Alt={m_alt:.0f}ft, IAS={m_spd:.1f}kt, Hdg={m_hdg:.1f}°)...", flush=True)
                t_wait = time.time() + 3.0
                while time.time() < t_wait:
                    if self.p3d and self.p3d.is_connected():
                        break
                    time.sleep(0.15)

                if not self.p3d or not self.p3d.is_connected():
                    raise RuntimeError("Prepar3D is not connected.")

                self.p3d.pause_sim()
                time.sleep(0.1)
                # Establish airspeed/body motion using the earlier, better-performing
                # airborne initialization before restoring recorded world motion.
                self.p3d.set_position_airborne(
                    alt_ft=m_alt, speed_kts=m_spd, pitch_deg=m_pitch,
                    bank_deg=-m_bank, hdg_deg=m_hdg, lat=m_lat, lon=m_lon
                )
                time.sleep(0.3)
                self.p3d.pause_sim()
                self._wait_for_initial_altitude(m_alt)
                self._commit_initial_position()
                self._restore_reference_state()
                self._settle_reference_controls()
                self._prepare_reference_power()
                self._wait_for_reference_rpm()
                # Refresh IAS after the LAST live engine warm-up.
                self._restore_reference_state()
                self._refresh_restored_airdata()
                self._restore_reference_state()
                print("[Auto Prep Step 4/5] Prepar3D initial state restored and confirmed while paused.", flush=True)

                # STEP 5: Final verification
                print("[Auto Prep Step 5/5] Final verification: rechecking BRUNNER and Prepar3D readiness...", flush=True)
                self._settle_reference_controls()
                chk_p, _, _ = self.brunner.read_axis(self.brunner.AXIS_PITCH)
                chk_r, _, _ = self.brunner.read_axis(self.brunner.AXIS_ROLL)
                chk_y, _, _ = self.brunner.read_axis(self.brunner.AXIS_RUDDER)
                cmd_p = max(-1.0, min(1.0, p0 + self.replay_axis_offsets[0]))
                cmd_r = max(-1.0, min(1.0, r0 + self.replay_axis_offsets[1]))
                cmd_y = max(-1.0, min(1.0, y0 + self.replay_axis_offsets[2]))
                self.brunner.move_actuators(cmd_p, cmd_r, cmd_y)
                tele = self._refresh_paused_telemetry()
                validate_initial_airspeed(tele, m_spd)
                validate_reference_power(tele, self.ref_manual_snapshot)
                target_rpm = self.ref_manual_snapshot.get("rpm")
                if target_rpm is not None:
                    rpm = float(tele.get("engine_rpm", float("nan")))
                    if not math.isfinite(rpm) or abs(rpm-float(target_rpm)) > 10.0:
                        raise RuntimeError("Engine RPM changed after settling; Automatic recording remains blocked.")
                print(f"[Auto Prep Step 5/5] Ready for Replay! P3D paused: MSL={tele.get('pressure_alt', 0):.0f}ft, indicated={tele.get('indicated_altitude', 0):.0f}ft, IAS={tele.get('airspeed_indicated', 0):.1f}kt. Stick: P={chk_p:+.3f}, R={chk_r:+.3f}, Y={chk_y:+.3f}.", flush=True)

                # Ready for recording (signal dispatched to main UI thread)
                self.sig_update_stage.emit(STAGE_CONNECTED_READY, "")
            except Exception as e:
                if preparation_started:
                    if self.p3d:
                        self.p3d.pause_sim()
                    self.brunner.release_controls()
                print(f"[Auto Prep Error] {e}", flush=True)
                stage = STAGE_AUTO_BLOCKED if self.p3d and self.p3d.is_connected() else STAGE_CONNECTION_ERROR
                self.sig_update_stage.emit(stage, str(e))

        self.preparation_thread=threading.Thread(target=_prep_auto, daemon=True)
        self.preparation_thread.start()

    # =====================================================================
    # START RECORDING (INSTANT, NO COUNTDOWN)
    # =====================================================================
    @Slot()
    def arm_recording(self):
        """Triggered by Start Recording button or Spacebar. Starts recording instantly."""
        if self.is_recording or self.test_mode not in ["manual", "auto"]:
            return

        self._play_beep("start")
        self.is_recording = True
        self.record_seconds = 0
        self.rec_time.clear()
        self.p_pos_buf.clear(); self.p_force_buf.clear()
        self.r_pos_buf.clear(); self.r_force_buf.clear()
        self.y_pos_buf.clear(); self.y_force_buf.clear()
        self.p3d_tele_buf.clear()
        self.applied_force_buf.clear()
        self.virtual_force_buf.clear()
        self.replay_command_buf.clear()
        self.stop_worker_event.clear()

        self._update_stage_ui(STAGE_RECORDING)

        # Launch 50 Hz execution loop in background thread
        if self.test_mode == "manual":
            self.worker_thread = threading.Thread(target=self._manual_recording_loop, daemon=True)
        else:
            self.worker_thread = threading.Thread(target=self._auto_replay_loop, daemon=True)
        self.worker_thread.start()

    def _on_spacebar_pressed(self):
        if self.btn_record.isVisible() and self.btn_record.isEnabled() and not self.is_recording:
            self.arm_recording()

    # =====================================================================
    # 50 Hz WORKER LOOPS
    # =====================================================================
    def _settle_reference_controls(self):
        """Measure and correct a small steady servo offset while P3D stays paused."""
        target = (self.ref_p_pos[0], self.ref_r_pos[0], self.ref_y_pos[0])
        axes = (self.brunner.AXIS_PITCH, self.brunner.AXIS_ROLL, self.brunner.AXIS_RUDDER)
        offsets = list(getattr(self, "replay_axis_offsets", (0.0, 0.0, 0.0)))
        deadline = time.monotonic() + 5.0
        stable = 0
        alignment_window = []
        measured = (float("nan"),) * 3
        while time.monotonic() < deadline:
            if self.stop_worker_event.is_set():
                self.brunner.release_controls()
                raise RuntimeError("Automatic preparation cancelled.")
            commands = tuple(max(-1.0, min(1.0, t+o)) for t,o in zip(target, offsets))
            self.brunner.move_actuators(*commands)
            measured = tuple(self.brunner.read_axis(axis)[0] for axis in axes)
            errors = tuple(t-m for t,m in zip(target, measured))
            if not all(math.isfinite(e) for e in errors):
                break
            # Require a stable window rather than rejecting a 0.38 mm residual
            # against a single 0.3 mm reading. Large errors still reset the window.
            if all(abs(e) <= 0.005 for e in errors):
                alignment_window.append(measured)
                alignment_window = alignment_window[-3:]
            else:
                alignment_window.clear()
            stable = len(alignment_window)
            quiet = stable == 3 and all(max(v[i] for v in alignment_window)-
                min(v[i] for v in alignment_window) <= 0.003 for i in range(3))
            if quiet:
                self.replay_alignment_errors = errors
                # Rudder offset is strictly 0.0 to prevent t=0 jump
                self.replay_axis_offsets = (float(offsets[0]), float(offsets[1]), 0.0)
                print(f"[Auto Prep] Measured axis errors: {errors}; command offsets: {self.replay_axis_offsets}", flush=True)
                return
            # Correct only a small residual bias. Keep the existing motor force/speed settings.
            # Wait between updates so the motor can respond before another correction.
            for axis, error in enumerate(errors):
                if axis == 2:
                    # Rudder must strictly lock to exact manual reference position (0.0 offset)
                    offsets[axis] = 0.0
                elif abs(error) > 0.003:
                    offsets[axis] = max(-0.05, min(0.05, offsets[axis] + 0.25*error))
            time.sleep(0.10)
        self.brunner.release_controls()
        raise RuntimeError(f"Brunner starting position did not settle. Target={target}, measured={measured}. "
                           "Replay requires three stable measurements within 0.5 mm.")

    def _load_replay_timing(self, reference_path):
        self.replay_axis_leads = (LEAD_MS/1000.0,)*3
        self.replay_timing_source = None
        calibration_path = Path(CURRENT_DIR)/"config"/"brunner_timing_calibration.json"
        try:
            calibration_read_path = calibration_path if calibration_path.exists() else Path(RESOURCE_DIR)/"config"/"brunner_timing_calibration.json"
            calibration = json.loads(calibration_read_path.read_text(encoding="utf-8"))
            leads = tuple(float(v) for v in calibration["leads_s"])
            if len(leads) == 3 and all(math.isfinite(v) and 0 <= v <= 0.5 for v in leads):
                self.replay_axis_leads = leads
                self.replay_timing_source = calibration.get("source")
        except (OSError,ValueError,KeyError,TypeError):
            pass
        directory = Path(reference_path).parent
        for path in sorted(directory.glob("*auto*_init_conditions.json"), key=lambda p:p.stat().st_mtime, reverse=True):
            try:
                meta = json.loads(path.read_text(encoding="utf-8"))
                if os.path.normcase(os.path.abspath(meta.get("manual_ref_csv", ""))) != os.path.normcase(os.path.abspath(reference_path)):
                    continue
                csv_path = path.with_name(path.name.replace("_init_conditions.json", ".csv"))
                with csv_path.open(newline="",encoding="utf-8-sig") as stream:
                    rows = list(csv.DictReader(stream))
                times = [float(row["Time (s)"]) for row in rows]
                if len(times) < 100 or times[-1] < self.ref_time[-1]*0.9:
                    continue
                previous = meta.get("replay_axis_leads_s", (LEAD_MS/1000.0,)*3)
                values = (self.ref_p_pos,self.ref_r_pos,self.ref_y_pos)
                columns = ("Joystick Pitch Position (mm)","Joystick Roll Position (mm)","Rudder Pedal Position (mm)")
                self.replay_axis_leads = tuple(estimate_replay_lead(self.ref_time,refs,times,
                    [float(row[column])/100.0 for row in rows],previous[axis])
                    for axis,(refs,column) in enumerate(zip(values,columns)))
                self.replay_timing_source = str(csv_path)
                calibration_path.parent.mkdir(parents=True,exist_ok=True)
                calibration_path.write_text(json.dumps({"leads_s":self.replay_axis_leads,
                    "source":str(csv_path)},indent=2),encoding="utf-8")
                print(f"[Auto Prep] Measured Brunner command leads (pitch/roll/yaw): {self.replay_axis_leads} s; source={csv_path.name}",flush=True)
                break
            except (OSError,ValueError,KeyError,TypeError,IndexError):
                continue

    def _prepare_reference_power(self):
        """Automatic engine initial conditions; no controller writes during replay."""
        sim = self.p3d
        if sim is None or not sim.is_connected():
            raise RuntimeError("Prepar3D is not connected during automatic preparation.")
        state = self.ref_manual_snapshot.get("telemetry", {})
        # Use the same measured t=0 throttle as the replay loop. Sidecar metadata
        # may be missing or stale, and zero is a valid starting throttle.
        recorded=getattr(self,'ref_throt_pos',None)
        target = float(recorded[0]) if recorded is not None and len(recorded) else self.ref_manual_snapshot.get("throttle", state.get("engine_throttle_percent"))
        if recorded is not None and len(recorded):
            if not math.isfinite(target) or not 0<=target<=100:
                raise ValueError('Invalid initial replay throttle.')
            self.ref_manual_snapshot['throttle']=target
        if target is not None and math.isfinite(float(target)):
            if not sim.set_initial_throttle(float(target)):
                raise RuntimeError("Prepar3D rejected initial throttle setup.")
        mixture = state.get("engine_mixture_percent")
        propeller = state.get("engine_propeller_percent")
        if mixture is not None or propeller is not None:
            if not sim.set_initial_engine_levers(mixture, propeller):
                raise RuntimeError("Prepar3D rejected recorded mixture/propeller initialization.")
            print(f"[Auto Prep] Restoring recorded engine levers: mixture={mixture}%, propeller={propeller}% (paused initialization only).", flush=True)
        thermal = sim.restore_engine_thermal_state(state)
        deadline = time.monotonic() + 3.0
        while True:
            fresh = self._refresh_paused_telemetry()
            fresh.update(thermal)
            try:
                validate_reference_power(fresh, self.ref_manual_snapshot)
                print(f"[Auto Prep] Measured engine levers: mixture={fresh.get('engine_mixture_percent')}%, "
                      f"propeller={fresh.get('engine_propeller_percent')}%. "
                      f"Reference mixture={state.get('engine_mixture_percent')}, "
                      f"propeller={state.get('engine_propeller_percent')}.",flush=True)
                print(f"[Auto Prep] Initial throttle verified: {fresh.get('engine_throttle_percent')}%; "
                      f"oil={fresh.get('engine_oil_temperature_c_1')} C, "
                      f"CHT={fresh.get('engine_cylinder_head_temperature_c_1')} C.", flush=True)
                return fresh
            except ValueError:
                if time.monotonic() >= deadline:
                    raise

    def _wait_for_reference_rpm(self):
        """Require two seconds of stable, measured engine/prop RPM on live frames."""
        sim = self.p3d
        if sim is None or not sim.is_connected():
            raise RuntimeError("Prepar3D disconnected before engine settling.")
        reference = self.ref_manual_snapshot
        state = reference.get("telemetry", {})
        target = reference.get("rpm", state.get("engine_rpm"))
        if target is None or not math.isfinite(float(target)) or float(target) <= 300:
            raise RuntimeError("Manual starting RPM is missing or invalid; record a running-engine Manual reference.")
        target = float(target)
        prop_values = state.get("prop_rpms", [])
        prop_target = float(prop_values[0]) if prop_values and prop_values[0] is not None else target
        if not math.isfinite(prop_target) or prop_target <= 300:
            raise RuntimeError("Manual starting propeller RPM is invalid.")
        samples = []
        engine_test = getattr(self, '_session_context', {}).get('test_reference') in ('1.f.(1)', '1.f.(2)')
        reference_ias = float(reference.get('airspeed', float('nan')))
        reference_alt = float(reference.get('altitude_msl', state.get('pressure_alt', float('nan'))))
        if engine_test and not all(math.isfinite(v) for v in (reference_ias, reference_alt)):
            raise RuntimeError('Engine replay requires recorded initial IAS and MSL altitude.')
        corrections = 0
        started = time.monotonic()
        print(f"[Auto Prep] Settling measured RPM near Manual engine={target:.0f}, prop={prop_target:.0f} before recording.", flush=True)
        try:
            sim.unpause_sim()
            while True:
                if self.stop_worker_event.is_set():
                    raise RuntimeError("Engine settling cancelled.")
                if not sim.is_connected():
                    raise RuntimeError("Prepar3D disconnected during engine settling.")
                sequence = sim.get_telemetry_sequence()
                if not sim.request_telemetry_once():
                    raise RuntimeError("Prepar3D rejected engine-settling telemetry.")
                fresh = sim.wait_for_fresh_telemetry(sequence, 1.5)
                if fresh is None:
                    raise RuntimeError("No fresh live RPM telemetry; replay remains blocked.")
                now = time.monotonic()
                if engine_test:
                    actual_ias = float(fresh.get('airspeed_indicated', float('nan')))
                    actual_alt = float(fresh.get('pressure_alt', float('nan')))
                    if not all(math.isfinite(v) for v in (actual_ias, actual_alt)):
                        raise RuntimeError('Missing flight-state feedback during engine RPM initialization.')
                    if abs(actual_ias-reference_ias) > 2.0 or abs(actual_alt-reference_alt) > 25.0:
                        samples.clear()  # Never accept RPM measured outside the reference flight state.
                        if now-started >= 20.0:
                            raise RuntimeError(f'Engine initialization drifted from Manual flight conditions: IAS={actual_ias:.1f} kt (target {reference_ias:.1f}), MSL={actual_alt:.0f} ft (target {reference_alt:.0f}). Replay remains blocked.')
                        print(f'[Auto Prep] Restoring initialization flight state before RPM check: IAS={actual_ias:.1f}/{reference_ias:.1f} kt, MSL={actual_alt:.0f}/{reference_alt:.0f} ft.', flush=True)
                        sim.pause_sim()
                        if not sim.wait_for_paused(3.0):
                            raise RuntimeError('Simulator did not pause for engine initialization correction.')
                        if self.stop_worker_event.is_set():
                            raise RuntimeError('Engine settling cancelled.')
                        self._restore_reference_state()
                        if self.stop_worker_event.is_set():
                            raise RuntimeError('Engine settling cancelled.')
                        corrections += 1
                        sim.unpause_sim()
                        continue
                    self._session_context['engine_rpm_initialization'] = dict(
                        reference_ias_kt=reference_ias, actual_ias_kt=actual_ias,
                        reference_altitude_msl_ft=reference_alt, actual_altitude_msl_ft=actual_alt,
                        flight_state_corrections=corrections, elapsed_s=now-started)
                engine = float(fresh.get("engine_rpm", float("nan")))
                props = fresh.get("prop_rpms", [])
                prop = float(props[0]) if props else float("nan")
                if (math.isfinite(engine) and math.isfinite(prop) and
                        abs(engine-target) <= 10.0 and abs(prop-prop_target) <= 10.0):
                    samples.append((now, engine, prop))
                    # Keep a trailing window with one sample spanning its full duration.
                    while len(samples) > 1 and samples[1][0] <= now-2.0:
                        samples.pop(0)
                    if (len(samples) >= 5 and now-samples[0][0] >= 2.0 and
                            all(max(row[i] for row in samples)-min(row[i] for row in samples) <= 10.0 for i in (1, 2))):
                        print(f"[Auto Prep] RPM settled: engine={engine:.1f}, prop={prop:.1f}; restoring paused flight state.", flush=True)
                        return
                else:
                    samples.clear()
                if now-started >= 20.0:
                    raise RuntimeError(f"Engine did not settle near Manual RPM within 20s: engine={engine:.1f} (target {target:.1f}), prop={prop:.1f} (target {prop_target:.1f}). Replay remains blocked.")
                time.sleep(.1)
        finally:
            sim.pause_sim()
            if not sim.wait_for_paused(3.0):
                raise RuntimeError("Prepar3D did not confirm pause after engine settling.")

    def _restore_reference_weather(self):
        """Apply the selected Manual air mass before waiting for local recovery."""
        snapshot=self.ref_manual_snapshot
        state=snapshot.get("telemetry",{})
        if not any(state.get(key) is not None for key in ("ambient_wind_spd","ambient_temp")) and not any(snapshot.get(key) is not None for key in ("wind_spd","oat")):
            return  # Legacy references with no recorded weather cannot supply a target.
        sim=self.p3d
        if sim is None or not sim.is_connected():raise RuntimeError("Prepar3D disconnected before weather restoration.")
        if self.stop_worker_event.is_set():raise RuntimeError("Automatic preparation cancelled.")
        if not sim.wait_for_paused(3.0):raise RuntimeError("Prepar3D did not confirm pause before weather restoration.")
        metar=reference_weather_metar(snapshot,sim.get_global_weather_metar())
        if metar is not None and not sim.set_global_weather_metar(metar):
            raise RuntimeError("Prepar3D rejected the recorded Manual weather. Replay remains blocked.")
        print(f"[Auto Prep] Restored Manual weather before airflow recovery: {metar}",flush=True)

    def _commit_initial_position(self, reference_snapshot=None):
        """Finish teleport/air-data recovery before either Manual or Automatic recording."""
        sim = self.p3d
        if sim is None or not sim.is_connected():
            raise RuntimeError("Prepar3D is not connected during automatic preparation.")
        if self.stop_worker_event.is_set():
            raise RuntimeError("Automatic preparation cancelled.")
        selected_snapshot = self.ref_manual_snapshot if reference_snapshot is None else reference_snapshot
        reference = selected_snapshot.get("telemetry", {})
        wind_keys = ("aircraft_wind_x_kt", "aircraft_wind_y_kt", "aircraft_wind_z_kt")
        expected_wind = None
        reference_wind_transient = False
        if all(reference.get(k) is not None for k in wind_keys):
            expected_wind = math.sqrt(sum(float(reference[k])**2 for k in wind_keys))
            ambient=float(reference.get("ambient_wind_spd",expected_wind))
            if expected_wind < ambient*0.8:
                reference_wind_transient=True
                expected_wind=ambient
        expected_offset = reference_altimeter_offset(selected_snapshot)
        started = time.monotonic()
        stable = 0
        last_status = -5.0
        try:
            sim.unpause_sim()
            while True:
                if self.stop_worker_event.is_set():
                    raise RuntimeError("Automatic preparation cancelled.")
                sequence = sim.get_telemetry_sequence()
                if not sim.request_telemetry_once():
                    raise RuntimeError("Prepar3D rejected initialization telemetry request.")
                fresh = sim.wait_for_fresh_telemetry(sequence, 1.5)
                if fresh is None:
                    raise RuntimeError("No live frame confirming aircraft initialization.")
                elapsed = time.monotonic() - started
                actual_wind = math.sqrt(sum(float(fresh.get(k, float("nan")))**2 for k in wind_keys))
                wind_target=expected_wind
                wind_ready = wind_target is None or (math.isfinite(actual_wind) and
                    abs(actual_wind-wind_target) <= max(0.5,wind_target*0.1))
                offset = float(fresh.get("indicated_altitude", float("nan"))) - float(fresh.get("pressure_alt", float("nan")))
                altitude_ready = expected_offset is None or (math.isfinite(offset) and abs(offset) <= 10.0) or (math.isfinite(offset) and abs(offset - expected_offset) <= 10.0)
                stable = stable + 1 if elapsed >= 2.0 and wind_ready and altitude_ready else 0
                if elapsed - last_status >= 5.0:
                    print(f"[Auto Prep] Waiting for local airflow/altimeter: {elapsed:.1f}s, "
                          f"wind={actual_wind:.2f} kt (Manual={expected_wind}), "
                          f"indicated-minus-MSL={offset:.1f} ft (Manual={expected_offset}).", flush=True)
                    last_status = elapsed
                    # Scenery loading can reassert pause after our first unpause.
                    sim.unpause_sim()
                if stable >= 3:
                    break
                if elapsed >= 60.0:
                    raise RuntimeError(f"Prepar3D initial conditions did not match within 60 seconds: "
                                       f"wind {actual_wind:.2f} kt (target {wind_target}); "
                                       f"altimeter offset {offset:.1f} ft (target {expected_offset}). "
                                       "Replay has not started; simulator remains paused.")
                time.sleep(0.25)
        finally:
            sim.pause_sim()
        self._refresh_paused_telemetry()
        print("[Auto Prep] Local airflow and altimeter recovered; paused before final Manual motion restore.", flush=True)

    def _refresh_restored_airdata(self):
        """Let real air data catch up with restored motion, without instrument-altitude writes."""
        sim=self.p3d
        if sim is None or not sim.is_connected():
            raise RuntimeError("Prepar3D disconnected while refreshing restored air data.")
        expected=reference_altimeter_offset(self.ref_manual_snapshot)
        try:
            sim.unpause_sim()
            started=time.monotonic()
            while True:
                if self.stop_worker_event.is_set():
                    raise RuntimeError("Automatic preparation cancelled.")
                sequence=sim.get_telemetry_sequence()
                if not sim.request_telemetry_once():
                    raise RuntimeError("Prepar3D rejected air-data refresh.")
                fresh=sim.wait_for_fresh_telemetry(sequence,1.5)
                if fresh is None:
                    raise RuntimeError("No fresh restored air data.")
                try:
                    validate_initial_airspeed(fresh,self.ref_manual_snapshot["airspeed"])
                    offset=float(fresh["indicated_altitude"])-float(fresh["pressure_alt"])
                    if time.monotonic()-started>=0.2 and (expected is None or abs(offset)<=10.0 or abs(offset-expected)<=10.0):
                        return
                except (ValueError,KeyError):
                    pass
                if time.monotonic()-started>=5.0:
                    raise RuntimeError("Restored air data did not converge; replay remains paused.")
                time.sleep(.05)
        finally:
            sim.pause_sim()

    def _restore_reference_state(self):
        """Restore supported initial conditions while paused, before replay begins."""
        state = self.ref_manual_snapshot.get("telemetry", {})
        if not state:
            print("[Auto Prep] Legacy reference has no full initial telemetry; velocity/fuel restore unavailable.")
            return
        if not self.p3d or not self.p3d.is_connected():
            raise RuntimeError("Prepar3D disconnected during initial-state restore.")
        self.p3d.pause_sim()
        if not self.p3d.wait_for_paused(3.0):
            raise RuntimeError("Prepar3D did not confirm pause before initial-state restoration.")
        operations: List[tuple[str, Callable[..., bool], tuple[Any, ...]]] = []
        snapshot = self.ref_manual_snapshot
        if all(snapshot.get(key) is not None for key in ("lat", "lon", "altitude_msl", "pitch_ang", "bank_ang", "heading")):
            pose = {"latitude_deg": snapshot["lat"], "longitude_deg": snapshot["lon"],
                    "altitude_ft": snapshot["altitude_msl"], "pitch_deg": snapshot["pitch_ang"],
                    "bank_deg": -float(snapshot["bank_ang"]), "true_heading_deg": snapshot["heading"]}
            operations.append(("final geometric pose", self.p3d.set_exact_pose_state_no_airspeed, (pose,)))
        if state.get("payload_station_weights_lb") is not None:
            operations.append(("payload", self.p3d.set_payload_station_weights,
                               (state["payload_station_weights_lb"],)))
        if state.get("fuel_tank_levels_all") is not None:
            operations.append(("fuel", self.p3d.set_all_fuel_tank_levels,
                               (state["fuel_tank_levels_all"],)))
        # Synchronize exact manual reference trim values into Prepar3D at t=0
        trim_el = float(state.get("trim_elevator_angle_rad", state.get("elevator_trim", 0.0)))
        trim_ai = float(state.get("trim_aileron_percent", 0.0)) / 100.0 if "trim_aileron_percent" in state else float(state.get("aileron_trim_pct", 0.0))
        trim_ru = float(state.get("trim_rudder_percent", 0.0)) / 100.0 if "trim_rudder_percent" in state else float(state.get("rudder_trim_pct", 0.0))
        if hasattr(self.p3d, "set_initial_trim"):
            operations.append(("initial aerodynamic trim (elevator/aileron/rudder)", self.p3d.set_initial_trim, (trim_el, trim_ai, trim_ru)))
        operations.append(("world/body/angular velocity (no controller writes)",
                           self.p3d.set_initial_dynamic_state, (build_dynamic_state(state),)))
        if state.get("vertical_speed") is not None and state.get("indicated_altitude") is not None:
            operations.append(("initial VSI memory (physical altitude preserved)", self.p3d.set_initial_instruments,
                               (state["vertical_speed"], state["indicated_altitude"])))
        for label, setter, args in operations:
            if self.stop_worker_event.is_set():
                raise RuntimeError("Automatic preparation cancelled.")
            if not setter(*args):
                raise RuntimeError("Prepar3D rejected initial-state restore: " + label)
        fresh = self._refresh_paused_telemetry()
        # A native ACK only means that a write was queued. Verify the measured result.
        tolerances = {"velocity_world_x": 0.5, "velocity_world_y": 0.5, "velocity_world_z": 0.5,
                      "velocity_body_x": 0.5, "velocity_body_y": 0.5, "velocity_body_z": 0.5,
                      "rotation_velocity_body_x": 0.01, "rotation_velocity_body_y": 0.01,
                      "rotation_velocity_body_z": 0.01, "total_weight": 0.5}
        if state.get("vertical_speed") is not None and state.get("indicated_altitude") is not None:
            tolerances.update(vertical_speed=25.0)
        if state.get("pressure_alt") is not None:
            tolerances.update(pressure_alt=5.0)
        deadline = time.monotonic() + 3.0
        while True:
            mismatches = [k for k,tol in tolerances.items()
                          if k in state and (not math.isfinite(float(fresh.get(k, float("nan"))))
                                             or abs(float(fresh[k])-float(state[k])) > tol)]
            if not mismatches:
                break
            if time.monotonic() >= deadline:
                raise RuntimeError("Initial-state verification failed: " + ", ".join(mismatches))
            fresh = self._refresh_paused_telemetry()
        print("[Auto Prep] Verified world/body velocity, rotation and weight against Manual.", flush=True)
        if state.get("velocity_world_y") is not None:
            print(f"[Auto Prep] Physical climb at start: Manual={float(state['velocity_world_y'])*60:.1f} fpm, "
                  f"Automatic={float(fresh['velocity_world_y'])*60:.1f} fpm. "
                  "VSI indication is recorded separately.", flush=True)

    def _wait_for_initial_altitude(self, altitude_ft):
        # The driver's pressure_alt channel is PLANE ALTITUDE (MSL), matching INITPOSITION.
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            tele = self._refresh_paused_telemetry()
            if abs(float(tele["pressure_alt"]) - float(altitude_ft)) <= 25.0:
                return tele
            time.sleep(0.05)
        raise RuntimeError("Prepar3D has not reached the requested initial altitude. "
                           "Wait for aircraft/scenery initialization and retry.")

    def _refresh_paused_telemetry(self):
        """Refresh the Python cache before releasing the paused simulator."""
        if not self.p3d or not self.p3d.is_connected():
            raise RuntimeError("Prepar3D is not connected.")
        seq = self.p3d.get_telemetry_sequence()
        if not self.p3d.request_telemetry_once():
            raise RuntimeError("Prepar3D rejected the telemetry request.")
        tele = self.p3d.wait_for_fresh_telemetry(seq, 2.0)
        if tele is None:
            raise RuntimeError("No fresh paused telemetry; recording was not started.")
        if ("engine_oil_temperature_c_1" not in tele or
                "engine_cylinder_head_temperature_c_1" not in tele):
            support = self.p3d.get_engine_support_state(fresh=True, timeout=1.0)
            tele.update(support)
        return tele

    def _manual_recording_loop(self):
        """High-precision 50 Hz manual recording loop."""
        try:
            self._refresh_paused_telemetry()
        except Exception as exc:
            self.is_recording = False
            self.brunner.release_controls()
            self.sig_update_stage.emit(STAGE_CONNECTION_ERROR, str(exc))
            return
        # Capture the sequence on every path before checking for a fresh packet.
        initial_sequence = self.p3d.get_telemetry_sequence() if self.p3d else -1
        # Unpause P3D in exact lockstep at t=0
        if self.p3d:
            self.p3d.unpause_sim()
            print("[Manual Recording] Prepar3D unpaused in lockstep at t=0.", flush=True)

        t0 = time.perf_counter()
        next_tick = 0.0
        sec_counter = 0

        while not self.stop_worker_event.is_set():
            now = time.perf_counter() - t0
            if now >= self.target_duration_sec:
                break

            # 1. Read Brunner Controls exclusively via UDP 15090
            pp, pf, pvf = self.brunner.read_axis(self.brunner.AXIS_PITCH)
            rp, rf, rvf = self.brunner.read_axis(self.brunner.AXIS_ROLL)
            yp, yf, yvf = self.brunner.read_axis(self.brunner.AXIS_RUDDER)

            # 2. Read P3D Telemetry (Read-only)
            tele = self.p3d.get_telemetry() if self.p3d else {}

            if not self.rec_time and self.p3d:
                fresh = self.p3d.wait_for_fresh_telemetry(initial_sequence, 2.0)
                if fresh is None:
                    self.p3d.pause_sim()
                    self.brunner.release_controls()
                    self.is_recording = False
                    self.sig_update_stage.emit(STAGE_CONNECTION_ERROR, "No fresh telemetry after unpause.")
                    return
                tele = fresh
                now = time.perf_counter() - t0
            now = time.perf_counter() - t0
            self.rec_time.append(now)
            self.p_pos_buf.append(pp); self.p_force_buf.append(pf)
            self.r_pos_buf.append(rp); self.r_force_buf.append(rf)
            self.y_pos_buf.append(yp); self.y_force_buf.append(yf)
            self.p3d_tele_buf.append(tele)
            self.applied_force_buf.append((pf, rf, yf))
            self.virtual_force_buf.append((pvf, rvf, yvf))

            # Snapshot exact t=0 on first sample
            if len(self.rec_time) == 1:
                self.manual_t0_snapshot = {
                    "pitch_pos": pp, "roll_pos": rp, "rudder_pos": yp,
                    "airspeed": tele.get("airspeed_indicated", 80.0),
                    "altitude": tele.get("indicated_altitude", 3000.0),
                    "altitude_msl": tele.get("pressure_alt", tele.get("indicated_altitude", 3000.0)),
                    "pitch_ang": tele.get("pitch_angle", 0.0),
                    "bank_ang": tele.get("bank_angle", 0.0),
                    "heading": tele.get("true_heading", 48.4),
                    "rpm": tele.get("engine_rpm", 2700.0) or (tele.get("eng_rpms", [2700.0])[0]),
                    "throttle": float(tele.get("engine_throttle_percent", float("nan"))),
                    "lat": tele.get("plane_latitude", 32.23367),
                    "lon": tele.get("plane_longitude", 74.12350),
                    "telemetry": dict(tele),
                }

            # Update UI progress every second via Signal
            current_sec = int(now)
            if current_sec > sec_counter:
                sec_counter = current_sec
                self.sig_record_progress.emit(sec_counter)

            next_tick += INTERVAL_SEC
            if next_tick <= now:
                next_tick = now + INTERVAL_SEC
            dt = next_tick - (time.perf_counter() - t0)
            if dt > 0:
                time.sleep(dt)

        # Immediately pause Prepar3D simulation lockstep with finish
        if self.p3d:
            try:
                self.p3d.pause_sim()
                print("[FINISH] Manual recording completed -> Prepar3D simulation PAUSED immediately.", flush=True)
            except Exception:
                pass

        self.sig_recording_finished.emit()

    def _auto_replay_loop(self):
        """High-precision replay: Brunner flight axes and authorized P3D throttle."""
        # Recalibrate against final airflow immediately before releasing the simulator.
        try:
            self._settle_reference_controls()
        except Exception as exc:
            self.is_recording = False
            self.brunner.release_controls()
            self.sig_update_stage.emit(STAGE_AUTO_BLOCKED,str(exc))
            return
        # Hold pre-aligned manual t=0 Brunner control positions
        p_cmd = max(-1.0, min(1.0, self.ref_p_pos[0] + self.replay_axis_offsets[0]))
        r_cmd = max(-1.0, min(1.0, self.ref_r_pos[0] + self.replay_axis_offsets[1]))
        y_cmd = float(self.ref_y_pos[0])  # Locked directly to exact manual reference target
        self.brunner.move_actuators(p_cmd, r_cmd, y_cmd)

        try:
            ready = self._refresh_paused_telemetry()
            if self.p3d:
                validate_initial_airspeed(ready, self.ref_manual_snapshot["airspeed"])
                validate_reference_power(ready, self.ref_manual_snapshot)
        except Exception as exc:
            self.is_recording = False
            self.brunner.release_controls()
            self.sig_update_stage.emit(STAGE_CONNECTION_ERROR, str(exc))
            return
        try:
            throttle_replay=ThrottleReplay(self.ref_time,self.ref_throt_pos)
            throttle_cmd=throttle_replay.command(0.0)
            throttle_target=float(self.ref_throt_pos[0]) if throttle_cmd is None else float(throttle_cmd)
            if self.p3d is None or not self.p3d.set_replay_throttle(throttle_target):
                raise RuntimeError("Prepar3D rejected the starting replay throttle.")
        except (ValueError,RuntimeError) as exc:
            self.is_recording=False
            self.brunner.release_controls()
            self.sig_update_stage.emit(STAGE_AUTO_BLOCKED,str(exc))
            return
        tracker = BrunnerPositionTracker(self.replay_axis_offsets)
        feedback = tuple(self.brunner.read_axis(axis)[0] for axis in
                         (self.brunner.AXIS_PITCH, self.brunner.AXIS_ROLL, self.brunner.AXIS_RUDDER))
        # Capture the sequence on every path before checking for a fresh packet.
        initial_sequence = self.p3d.get_telemetry_sequence() if self.p3d else -1
        # Unpause P3D in exact lockstep at t=0
        if self.p3d:
            self.p3d.unpause_sim()
            print("[Auto Replay] Prepar3D unpaused in lockstep at t=0.", flush=True)

        t0 = time.perf_counter()
        next_tick = 0.0
        sec_counter = 0

        while not self.stop_worker_event.is_set():
            now = time.perf_counter() - t0
            if now > self.ref_time[-1] or now >= self.target_duration_sec:
                break

            # Interpolate by recorded time; sparse or jittered sampling must not create steps.
            command_time = now
            references = tuple(interpolate_reference(self.ref_time, values, now) for values in
                               (self.ref_p_pos, self.ref_r_pos, self.ref_y_pos))
            feedforward = tuple(value*GAIN for value in replay_feedforward(self.ref_time,
                (self.ref_p_pos,self.ref_r_pos,self.ref_y_pos),now,
                getattr(self,"replay_axis_leads",(LEAD_MS/1000.0,)*3)))
            try:
                throttle_command=throttle_replay.command(now)
                if throttle_command is not None:
                    throttle_target=throttle_command
                    if self.p3d is None or not self.p3d.set_replay_throttle(throttle_command):
                        raise RuntimeError("Prepar3D rejected throttle replay; test paused.")
                p_cmd, r_cmd, y_cmd = tracker.command(now, references, feedforward, feedback)
            except (ValueError,RuntimeError) as exc:
                if self.p3d:
                    self.p3d.pause_sim()
                self.brunner.release_controls()
                self.is_recording = False
                self.sig_update_stage.emit(STAGE_CONNECTION_ERROR, str(exc))
                return
            self.brunner.move_actuators(p_cmd, r_cmd, y_cmd)

            # 2. Read back Brunner positions & virtual forces
            pp, pf, pvf = self.brunner.read_axis(self.brunner.AXIS_PITCH)
            rp, rf, rvf = self.brunner.read_axis(self.brunner.AXIS_ROLL)
            yp, yf, yvf = self.brunner.read_axis(self.brunner.AXIS_RUDDER)
            feedback = (pp, rp, yp)

            # 3. Read measured P3D telemetry after the throttle-only command.
            tele = self.p3d.get_telemetry() if self.p3d else {}

            if not self.rec_time and self.p3d:
                fresh = self.p3d.wait_for_fresh_telemetry(initial_sequence, 2.0)
                if fresh is None:
                    self.p3d.pause_sim()
                    self.brunner.release_controls()
                    self.is_recording = False
                    self.sig_update_stage.emit(STAGE_CONNECTION_ERROR, "No fresh telemetry after unpause.")
                    return
                try:
                    validate_initial_airspeed(fresh, self.ref_manual_snapshot["airspeed"])
                except ValueError as exc:
                    self.p3d.pause_sim()
                    self.brunner.release_controls()
                    self.is_recording = False
                    self.sig_update_stage.emit(STAGE_CONNECTION_ERROR, str(exc))
                    return
                tele = fresh
                now = time.perf_counter() - t0
            now = time.perf_counter() - t0
            self.replay_command_buf.append((command_time, p_cmd, r_cmd, y_cmd, throttle_target))
            self.rec_time.append(now)
            self.p_pos_buf.append(pp); self.p_force_buf.append(pvf)
            self.r_pos_buf.append(rp); self.r_force_buf.append(rvf)
            self.y_pos_buf.append(yp); self.y_force_buf.append(yvf)
            self.p3d_tele_buf.append(tele)
            self.applied_force_buf.append((pf, rf, yf))
            self.virtual_force_buf.append((pvf, rvf, yvf))

            # Snapshot exact t=0 on first sample
            if len(self.rec_time) == 1:
                self.auto_t0_snapshot = {
                    "pitch_pos": pp, "roll_pos": rp, "rudder_pos": yp,
                    "airspeed": tele.get("airspeed_indicated", 80.0),
                    "altitude": tele.get("indicated_altitude", 3000.0),
                    "altitude_msl": tele.get("pressure_alt", tele.get("indicated_altitude", 3000.0)),
                    "pitch_ang": tele.get("pitch_angle", 0.0),
                    "bank_ang": tele.get("bank_angle", 0.0),
                    "heading": tele.get("true_heading", 48.4),
                    "rpm": tele.get("engine_rpm", 2700.0) or (tele.get("eng_rpms", [2700.0])[0]),
                    "throttle": float(tele.get("engine_throttle_percent", float("nan"))),
                    "lat": tele.get("plane_latitude", 32.23367),
                    "lon": tele.get("plane_longitude", 74.12350),
                    "telemetry": dict(tele),
                }

            current_sec = int(now)
            if current_sec > sec_counter:
                sec_counter = current_sec
                self.sig_record_progress.emit(sec_counter)

            next_tick += INTERVAL_SEC
            if next_tick <= now:
                next_tick = now + INTERVAL_SEC
            dt = next_tick - (time.perf_counter() - t0)
            if dt > 0:
                time.sleep(dt)

        # Immediately pause Prepar3D simulation lockstep with finish
        if self.p3d:
            try:
                self.p3d.pause_sim()
                print("[FINISH] Auto replay completed -> Prepar3D simulation PAUSED immediately.", flush=True)
            except Exception:
                pass

        self.sig_recording_finished.emit()

    @Slot(int)
    def _update_record_progress(self, sec: int):
        if not self.is_recording:
            return
        self.record_seconds = sec
        rows = len(self.rec_time)
        self.lbl_rec_progress.setText(f"{sec} / {self.target_duration_sec} seconds ({rows} rows)")
        self.rec_progress.setValue(min(self.target_duration_sec, sec))

    # =====================================================================
    # TEST STOP & COMPLETION (ENFORCES P3D PAUSE & BRUNNER RELEASE)
    # =====================================================================
    @Slot()
    def stop_test(self):
        """Emergency or manual STOP: pauses P3D and releases Brunner actuators."""
        self._play_beep("stop")
        self.stop_worker_event.set()
        self.is_recording = False

        # 1. Immediately pause Prepar3D simulation
        if self.p3d:
            try:
                self.p3d.pause_sim()
                print("[STOP] Prepar3D simulation PAUSED immediately.")
            except Exception as e:
                print(f"[STOP] P3D pause error: {e}")

        # 2. Release Brunner controls safely
        self.brunner.release_controls()
        self._update_stage_ui(STAGE_STOPPED_BY_USER)
        print("[STOP] Test stopped by user. Hardware released.")

    @Slot()
    def _on_recording_finished(self):
        """Called when 75-second flight test recording completes."""
        self._play_beep("stop")
        self.is_recording = False

        # 1. Immediately pause Prepar3D simulation lockstep with finish
        if self.p3d:
            try:
                self.p3d.pause_sim()
                print("[FINISH] 75-second test completed -> Prepar3D simulation PAUSED.")
            except Exception as e:
                print(f"[FINISH] P3D pause error: {e}")

        # 2. Release Brunner controls
        self.brunner.release_controls()

        # 3. Update stage and save reports
        self._update_stage_ui(STAGE_COMPLETED)
        elapsed = self.rec_time[-1] if self.rec_time else 0.0
        self.lbl_rec_status.setText("COMPLETED - simulation paused")
        self.lbl_rec_status.setStyleSheet("color: #a6e3a1;")
        self.lbl_rec_progress.setText(f"{elapsed:.1f} seconds ({len(self.rec_time)} recorded rows)")
        self.rec_progress.setValue(self.target_duration_sec)
        self._save_csv_and_generate_reports()

    # =====================================================================
    # SAVE CSV & GENERATE OFFICIAL 6-PAGE QTG PDF REPORT
    # =====================================================================
    def _save_csv_and_generate_reports(self):
        ts = time.strftime("%Y-%m-%d_%H-%M-%S")
        prefix = "manual" if self.test_mode == "manual" else "auto"
        csv_filename = f"Super_Mushak_{prefix}_takeoff_data_{ts}.csv"
        csv_path = str(self.data_dir / csv_filename)

        print(f"\n[SAVE] Saving CSV to: {csv_path}")
        try:
            # Standard QTG Fieldnames expected by plot_graphs.py
            fieldnames = [
                "Time (s)",
                "Altitude MSL (ft)",
                "Replay Command Time (s)",
                "Replay Pitch Command (-)",
                "Replay Roll Command (-)",
                "Replay Rudder Command (-)",
                "Replay Throttle Command (%)",
                "Indicated Airspeed (kts)",
                "Indicated Altitude (ft)",
                "Rate of Climb/Descent (fpm)",
                "Pitch Angle (deg)",
                "Bank Angle (deg)",
                "Magnetic Heading (deg)",
                "Pitch Rate (deg/s)",
                "Yaw Rate (deg/s)",
                "Roll Rate (deg/s)",
                "Propeller 1 RPM",
                "Engine 1 RPM",
                "Throttle 1 Position (%)",
                "Elevator Trim Angle (rad)",
                "Elevator Trim (%)",
                "Aileron Trim (%)",
                "Rudder Trim (%)",

                "Elevator Surface (rad)",
                "Left Aileron Surface (rad)",
                "Right Aileron Surface (rad)",
                "Rudder Surface (rad)",

                "Mixture 1 Position (%)",
                "Propeller 1 Lever (%)",
                "Propeller Blade Angle (rad)",
                "Engine Torque (ft lb)",
                "Engine Manifold Pressure (psi)",
                "Engine Fuel Air Ratio (-)",

                "Joystick Pitch Position (mm)",
                "Joystick Roll Position (mm)",
                "Rudder Pedal Position (mm)",
                "Joystick Pitch Force (N)",
                "Joystick Roll Force (N)",
                "Rudder Pedal Force (N)",
                "Pitch Virtual Force 0x51 (N)",
                "Roll Virtual Force 0x51 (N)",
                "Yaw Virtual Force 0x51 (N)",
                "Pitch Virtual Force (N)",
                "Roll Virtual Force (N)",
                "Yaw Virtual Force (N)",
                "Gross Wt (kg)",
                "Total Fuel Quantity (kg)",
                "CG Pos in X Axis (mm)",
                "CG Pos in Y Axis (mm)",
                "Baro Press (hPa)",
                "OAT (deg C)",
                "Wind Dir (mag,deg)",
                "Wind Spd (kt)",
                "Fuel Flow (pph)",
                "EGT (deg C)",
                "Ground Speed (kt)",
                "Mach Number (-)",
                "Pressure Alt (ft)",
                "Radar Alt (ft)",
                "Height of Terrain (ft)",
                "Aircraft Angle of Attack (deg)",
                "Sideslip Angle (deg)",
                "Track Angle (deg)",
                "X Body Acceleration (g)",
                "Y Body Acceleration (g)",
                "Z Body Acceleration (g)",
                # Dual-compatible alias columns for CLI analysis tools
                f"Pitch_{prefix.capitalize()}_Pos",
                f"Pitch_{prefix.capitalize()}_Force",
                f"Roll_{prefix.capitalize()}_Pos",
                f"Roll_{prefix.capitalize()}_Force",
                f"Rudder_{prefix.capitalize()}_Pos",
                f"Rudder_{prefix.capitalize()}_Force",
                f"Airspeed_{prefix.capitalize()}",
                f"Altitude_{prefix.capitalize()}",
                f"Pitch_Angle_{prefix.capitalize()}",
                f"Bank_Angle_{prefix.capitalize()}",
                f"Heading_{prefix.capitalize()}",
                f"Engine_RPM_{prefix.capitalize()}",
                f"Throttle_{prefix.capitalize()}"
            ]

            with open(csv_path, "w", newline="", encoding="utf-8") as f:
                writer = csv.DictWriter(f, fieldnames=fieldnames)
                writer.writeheader()

                for i in range(len(self.rec_time)):
                    tel = self.p3d_tele_buf[i] if i < len(self.p3d_tele_buf) else {}
                    t_sec = self.rec_time[i]
                    p_pos = self.p_pos_buf[i]
                    p_frc = self.p_force_buf[i]
                    r_pos = self.r_pos_buf[i]
                    r_frc = self.r_force_buf[i]
                    y_pos = self.y_pos_buf[i]
                    y_frc = self.y_force_buf[i]

                    ias = tel.get("airspeed_indicated", 80.0)
                    alt = tel.get("indicated_altitude", 3000.0)
                    v_spd = tel.get("vertical_speed", 0.0)
                    pitch = tel.get("pitch_angle", 0.0)
                    bank = tel.get("bank_angle", 0.0)
                    hdg = tel.get("true_heading", 48.4)
                    rpm = tel.get("engine_rpm", 0.0) or (tel.get("eng_rpms", [0.0])[0])
                    throt = float(tel.get("engine_throttle_percent", float("nan")))

                    # Keep the measured 0x21 and virtual 0x51 channels separate.
                    applied_samples = getattr(self, "applied_force_buf", [])
                    virtual_samples = getattr(self, "virtual_force_buf", [])
                    unknown = (float("nan"),) * 3
                    applied = applied_samples[i] if i < len(applied_samples) else ((p_frc, r_frc, y_frc) if self.test_mode == "manual" else unknown)
                    virtual = virtual_samples[i] if i < len(virtual_samples) else ((p_frc, r_frc, y_frc) if self.test_mode == "auto" else unknown)
                    # Convert normalized load cell force to true Newtons matching virtual force:
                    p_app = applied[0] * 100.0 if math.isfinite(applied[0]) and abs(applied[0]) <= 5.0 else applied[0]
                    r_app = applied[1] * 60.0 if math.isfinite(applied[1]) and abs(applied[1]) <= 5.0 else applied[1]
                    y_app = applied[2] * 80.0 if math.isfinite(applied[2]) and abs(applied[2]) <= 5.0 else applied[2]

                    commands = getattr(self, "replay_command_buf", [])
                    command = commands[i] if i < len(commands) else (float("nan"),) * 4
                    row_dict = {
                        "Time (s)": round(t_sec, 9),
                        "Altitude MSL (ft)": tel.get("pressure_alt", float("nan")),
                        "Replay Command Time (s)": command[0],
                        "Replay Pitch Command (-)": command[1],
                        "Replay Roll Command (-)": command[2],
                        "Replay Rudder Command (-)": command[3],
                        "Replay Throttle Command (%)": command[4] if len(command)>4 else float("nan"),
                        "Indicated Airspeed (kts)": round(float(ias), 2),
                        "Indicated Altitude (ft)": round(float(alt), 1),
                        "Rate of Climb/Descent (fpm)": round(float(v_spd), 1),
                        "Pitch Angle (deg)": round(float(pitch), 2),
                        "Bank Angle (deg)": round(float(bank), 2),
                        "Magnetic Heading (deg)": round(float(hdg), 2),
                        "Pitch Rate (deg/s)": round(float(tel.get("pitch_rate", 0.0)), 2),
                        "Yaw Rate (deg/s)": round(float(tel.get("yaw_rate", 0.0)), 2),
                        "Roll Rate (deg/s)": round(float(tel.get("roll_rate", 0.0)), 2),
                        "Propeller 1 RPM": round(float(tel.get("prop_rpms", [float("nan")])[0]), 0),
                        "Engine 1 RPM": round(float(rpm), 0),
                        "Throttle 1 Position (%)": round(float(throt), 1),
                        "Elevator Trim Angle (rad)": tel.get("trim_elevator_angle_rad",float("nan")),
                        "Elevator Trim (%)": tel.get("trim_elevator_percent",float("nan")),
                        "Aileron Trim (%)": tel.get("trim_aileron_percent",float("nan")),
                        "Rudder Trim (%)": tel.get("trim_rudder_percent",float("nan")),

                        "Elevator Surface (rad)": tel.get("surface_elevator_rad",float("nan")),
                        "Left Aileron Surface (rad)": tel.get("surface_aileron_left_rad",float("nan")),
                        "Right Aileron Surface (rad)": tel.get("surface_aileron_right_rad",float("nan")),
                        "Rudder Surface (rad)": tel.get("surface_rudder_rad",float("nan")),

                        "Mixture 1 Position (%)": tel.get("engine_mixture_percent",float("nan")),
                        "Propeller 1 Lever (%)": tel.get("engine_propeller_percent",float("nan")),
                        "Propeller Blade Angle (rad)": tel.get("propeller_blade_angle_rad",float("nan")),
                        "Engine Torque (ft lb)": tel.get("engine_torque_ft_lb",float("nan")),
                        "Engine Manifold Pressure (psi)": tel.get("engine_manifold_pressure_psi_1",float("nan")),
                        "Engine Fuel Air Ratio (-)": tel.get("engine_fuel_air_ratio",float("nan")),

                        "Joystick Pitch Position (mm)": round(float(p_pos) * 100.0, 2),
                        "Joystick Roll Position (mm)": round(float(r_pos) * 100.0, 2),
                        "Rudder Pedal Position (mm)": round(float(y_pos) * 100.0, 2),
                        "Joystick Pitch Force (N)": round(float(p_app), 3),
                        "Joystick Roll Force (N)": round(float(r_app), 3),
                        "Rudder Pedal Force (N)": round(float(y_app), 3),
                        "Pitch Virtual Force 0x51 (N)": round(float(virtual[0]), 3),
                        "Roll Virtual Force 0x51 (N)": round(float(virtual[1]), 3),
                        "Yaw Virtual Force 0x51 (N)": round(float(virtual[2]), 3),
                        "Pitch Virtual Force (N)": round(float(virtual[0]), 3),
                        "Roll Virtual Force (N)": round(float(virtual[1]), 3),
                        "Yaw Virtual Force (N)": round(float(virtual[2]), 3),
                        "Gross Wt (kg)": round(float(tel.get("total_weight", 850.0)), 1),
                        "Total Fuel Quantity (kg)": round(float(tel.get("total_fuel_quantity", 110.0)), 1),
                        "CG Pos in X Axis (mm)": 0.0,
                        "CG Pos in Y Axis (mm)": 0.0,
                        "Baro Press (hPa)": round(float(tel.get("baro_pressure", 1013.25)), 1),
                        "OAT (deg C)": round(float(tel.get("ambient_temp", 15.0)), 1),
                        "Wind Dir (mag,deg)": round(float(tel.get("ambient_wind_dir", 0.0)), 1),
                        "Wind Spd (kt)": round(float(tel.get("ambient_wind_spd", 0.0)), 1),
                        "Fuel Flow (pph)": round(float(tel.get("fuelFlow", 60.0)), 1),
                        "EGT (deg C)": round(float(tel.get("egt", 500.0)), 1),
                        "Ground Speed (kt)": round(float(tel.get("ground_speed", ias)), 2),
                        "Mach Number (-)": round(float(tel.get("mach_number", 0.12)), 3),
                        "Pressure Alt (ft)": round(float(tel.get("pressure_alt", alt)), 1),
                        "Radar Alt (ft)": round(float(tel.get("radar_alt", alt - 750.0)), 1),
                        "Height of Terrain (ft)": 750.0,
                        "Aircraft Angle of Attack (deg)": round(float(tel.get("aoa", 2.0)), 2),
                        "Sideslip Angle (deg)": round(float(tel.get("sideslip", 0.0)), 2),
                        "Track Angle (deg)": round(float(tel.get("track_angle", hdg)), 2),
                        "X Body Acceleration (g)": round(float(tel.get("accelX", 0.0)), 2),
                        "Y Body Acceleration (g)": round(float(tel.get("accelY", 0.0)), 2),
                        "Z Body Acceleration (g)": round(float(tel.get("accelZ", 1.0)), 2),
                        # Dual-compatible alias columns
                        f"Pitch_{prefix.capitalize()}_Pos": round(float(p_pos), 4),
                        f"Pitch_{prefix.capitalize()}_Force": round(float(p_app), 3),
                        f"Roll_{prefix.capitalize()}_Pos": round(float(r_pos), 4),
                        f"Roll_{prefix.capitalize()}_Force": round(float(r_app), 3),
                        f"Rudder_{prefix.capitalize()}_Pos": round(float(y_pos), 4),
                        f"Rudder_{prefix.capitalize()}_Force": round(float(y_app), 3),
                        f"Airspeed_{prefix.capitalize()}": round(float(ias), 2),
                        f"Altitude_{prefix.capitalize()}": round(float(alt), 1),
                        f"Pitch_Angle_{prefix.capitalize()}": round(float(pitch), 2),
                        f"Bank_Angle_{prefix.capitalize()}": round(float(bank), 2),
                        f"Heading_{prefix.capitalize()}": round(float(hdg), 2),
                        f"Engine_RPM_{prefix.capitalize()}": round(float(rpm), 0),
                        f"Throttle_{prefix.capitalize()}": round(float(throt), 1),
                    }
                    writer.writerow(row_dict)

            print(f"[OK] CSV saved successfully with full QTG columns: {len(self.rec_time)} rows.")

            # Save Initial Conditions JSON
            json_filename = os.path.splitext(csv_path)[0] + "_init_conditions.json"
            snapshot = self.manual_t0_snapshot if self.test_mode == "manual" else self.auto_t0_snapshot
            init_dict = {
                self.test_mode: snapshot,
                "manual_ref_csv": self.active_manual_ref_path if self.test_mode == "auto" else "",
                "replay_axis_offsets": getattr(self, "replay_axis_offsets", (0.0, 0.0, 0.0)),
                "replay_axis_leads_s": getattr(self,"replay_axis_leads",(LEAD_MS/1000.0,)*3),
                "replay_timing_source": getattr(self,"replay_timing_source",None)
            }
            if getattr(self,'control_method','replay')=='pid':
                init_dict['pid_run']={key:value for key,value in getattr(self,'_session_context',{}).items()
                    if key in ('initial_conditions','pid_conditions','pid_profile','pid_guidance_initialization','pid_workflow','pid_initial_readback','pid_temperature_check','pid_aircraft_settling','pid_stabilization','pid_forwarding_check','pid_complete','pid_fault','duration_s')}
            with open(json_filename, "w", encoding="utf-8") as jf:
                json.dump(init_dict, jf, indent=2, default=str)
            print(f"[OK] Initial conditions JSON saved: {json_filename}")

            # Also update local pitch_roll_rudder_telemetry.csv for fast analysis tools
            local_directory = Path(CURRENT_DIR)/"Output"/"recordings"
            local_directory.mkdir(parents=True,exist_ok=True)
            local_csv = str(local_directory/"pitch_roll_rudder_telemetry.csv")
            try:
                import shutil
                shutil.copyfile(csv_path, local_csv)
            except Exception:
                pass

        except Exception as e:
            QMessageBox.critical(self, "Save Error", f"Could not save CSV file:\n{e}")
            return

        # Generate Official 6-Page QTG PDF Report with progress bar and auto-open
        pdf_path = None
        progress_dialog: Optional[QProgressDialog] = None
        try:
            print("[REPORT] Generating official 6-page evaluation PDF report via plot_graphs.py...")
            self.lbl_rec_status.setText("GENERATING PDF REPORT...")
            self.lbl_rec_status.setStyleSheet("color: #89b4fa;")
            self.lbl_rec_progress.setText("Rendering 6-Page QTG PDF Report... Please wait")
            self.rec_progress.setRange(0, 0)  # Animated marquee progress bar

            progress_dialog = QProgressDialog(self)
            progress_dialog.setLabelText("Generating 6-Page Evaluation PDF Report...\nPlease wait.")
            progress_dialog.setWindowTitle("Generating PDF Report")
            progress_dialog.setWindowModality(Qt.WindowModality.WindowModal)
            progress_dialog.setCancelButton(None)
            progress_dialog.setRange(0, 0)
            progress_dialog.setMinimumDuration(0)
            progress_dialog.setValue(0)
            progress_dialog.show()
            QApplication.processEvents()

            pdf_path = plot_graphs.plot_aligned_comparison(csv_path, manual_csv_path=self.active_manual_ref_path,
                test_reference=getattr(self,"_session_context",{}).get("test_reference"), control_method=getattr(self,"control_method","replay"), validation_csv_path=getattr(self,"validation_csv",None), run_fault=getattr(self,"pid_fault",None), selected_graphs=getattr(self,"report_graphs",None), initial_conditions=getattr(self,"report_initial_conditions",None), pid_target_ias=getattr(self,"pid_conditions",{}).get("target_ias",80))

            if progress_dialog is not None:
                progress_dialog.close()
            self.rec_progress.setRange(0, self.target_duration_sec)
            self.rec_progress.setValue(self.target_duration_sec)

            if pdf_path and os.path.exists(pdf_path):
                print(f"[SUCCESS] Official QTG Report generated: {pdf_path}")
                self.lbl_rec_status.setText("COMPLETED - REPORT OPENED")
                self.lbl_rec_status.setStyleSheet("color: #a6e3a1;")
                self.lbl_rec_progress.setText(f"Report ready: {os.path.basename(pdf_path)}")
                try:
                    os.startfile(pdf_path)
                    print(f"[OPEN] Automatically opened report: {pdf_path}")
                except Exception as open_err:
                    print(f"[OPEN NOTE] Could not automatically open PDF: {open_err}")
            else:
                self.lbl_rec_status.setText("COMPLETED")
                self.lbl_rec_status.setStyleSheet("color: #a6e3a1;")
        except Exception as e:
            if progress_dialog is not None:
                try:
                    progress_dialog.close()
                except Exception:
                    pass
            self.rec_progress.setRange(0, self.target_duration_sec)
            self.rec_progress.setValue(self.target_duration_sec)
            print(f"[REPORT NOTE] plot_graphs generator note: {e}")
            self.lbl_rec_status.setText("COMPLETED (Report note)")
            self.lbl_rec_progress.setText(f"Report note: {e}")

        # Refresh dropdown so this new manual is immediately selectable
        self._refresh_manual_reference_selector()

    # =====================================================================
    # MANUAL REFERENCE SELECTOR (Dropdown Helpers)
    # =====================================================================
    def _refresh_manual_reference_selector(self):
        self.cmb_manual_reference.blockSignals(True)
        self.cmb_manual_reference.clear()

        # Scan for real manual flight test CSVs in data directory
        def is_valid_flight(p):
            name_l = p.name.lower()
            if any(k in name_l for k in ["diagnostic", "commissioning", "legacy_plot", "test_qtg"]):
                return False
            return "manual" in name_l and p.suffix == ".csv"

        candidates = [p for p in self.data_dir.glob("*.csv") if is_valid_flight(p)]
        if not candidates:
            candidates = [p for p in Path(CURRENT_DIR).glob("*.csv") if is_valid_flight(p)]

        # Sort by last modification time so the most recent flight is first
        candidates.sort(key=lambda p: p.stat().st_mtime, reverse=True)

        if candidates:
            for p in candidates:
                self.cmb_manual_reference.addItem(p.name, userData=str(p))
            self.selected_ref_csv = str(candidates[0])
            self.active_manual_ref_path = str(candidates[0])
            self.lbl_manual_ref_info.setText(f"Ready: {candidates[0].name}")
        else:
            self.cmb_manual_reference.addItem("No saved Manual runs found. Perform a Manual Test first.")
            self.selected_ref_csv = None
            self.active_manual_ref_path = None
            self.lbl_manual_ref_info.setText("No Manual reference available.")

        self.cmb_manual_reference.blockSignals(False)

    def _on_manual_reference_selection_changed(self, index: int):
        path = self.cmb_manual_reference.currentData()
        if path and os.path.exists(path):
            self.selected_ref_csv = path
            self.active_manual_ref_path = path
            self.lbl_manual_ref_info.setText(f"Selected: {Path(path).name}")

    def _open_data_folder(self):
        os.startfile(str(self.data_dir))


def main():
    if "--self-test" in sys.argv:
        from tools.packaging_check import run
        sys.exit(run(sys.argv))
    if getattr(sys,"frozen",False):
        log_dir=Path(CURRENT_DIR)/"logs"
        log_dir.mkdir(parents=True,exist_ok=True)
        log_stream=open(log_dir/"application.log","a",encoding="utf-8",buffering=1)
        sys.stdout=log_stream
        sys.stderr=log_stream
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    window = SuperMushakLoggerApp()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
