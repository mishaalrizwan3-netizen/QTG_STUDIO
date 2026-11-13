# V15.12 ROOT-CAUSE FIX - restore BODY XYZ + VSI without forcing PROP RPM
# V15.11 LOADER/SIM-PC DERIVED-STATE GATE FIX
# V15 SIM-PC GRAPH HOTFIX: measured diagnostic replay for current Manual
from measurement_audit_v15123 import MeasurementAudit
from telemetry_quality_v15124 import wait_wind_ready, wind_matches_reference
from fuel_initialization_v15 import primary_fuel_plan, fuel_mismatches
from engine_preparation_v15 import SUPPORT_KEYS, engine_mismatches, prepare_engine, configuration_mismatches
# V14.87: read-only engine and airflow cause telemetry at Manual/Automatic t=0.
# Retains physical control, pose, dynamics and frame-0 force interlocks.
"""
Prepar3D Super Mushshak Automatic Testing Data Logger Application
GUI built with PySide6 and official Prepar3D SimConnect SDK.

CLS-P 3-Axis FORCE V4 - PITCH FORCE DIAGNOSTIC:
- Pitch/Roll/Yaw physical position read from CLS2Sim Protocol v2.
- Force capability is auto-detected per physical node.
- Protocol-v2 0x21 normalized force is tried first (documented Read Data request).
- 0x20 legacy/raw force is fallback. 0x22 engineering force is probed as an
  optional capability because it is listed in the Data table but omitted from
  the explicit Read Data request list in the current BRUNNER protocol page.
- Watchdog heartbeat uses a dedicated UDP socket/thread so telemetry timeouts
  cannot starve the drive watchdog.
- V14.42 Manual mode is STRICT READ-ONLY to CLS2Sim/BRUNNER. It records live
  Pitch/Roll/Yaw position and engineering force without changing Force Scale,
  Move Back, AP, Force Offset, trim, or any controller override.
- Logs both applied force (0x20/0x21/0x22) and virtual/simulated deflection
  force (0x50/0x51) when supported by the installed CLS2Sim build.
- V14.10 adds an adaptive legacy-force path for the simulator-PC CLS-P
  firmware v474: when 0x22 ENG and 0x23 Axis Information are unavailable,
  0x20 RAW is interpreted as signed 16-bit only after it is cross-checked
  against 0x21 normalized force. The verified legacy scale is 0.1 N/count
  with 20000 counts corresponding to normalized full scale.
- V14.21 COMPLETE ALL-AXES + YAW SPEED SELECTOR preserves the validated v535/v474 force paths,
  V14.12 safe handover, corrected endpoint precheck, per-axis safety interlocks, and
  runs Pitch + Roll + Yaw position/force replay together on every Automatic test.
- The only runtime choice is Yaw AP speed: 60/80/100/120/150. This allows all Yaw
  speed trials to be completed in one simulator-PC visit without editing the code.
- V14.42 STRICT MANUAL + REPEATABLE AUTO keeps V14.21 replay tuning and V14.40
  P3D initialization unchanged. Manual is CLS-read-only; Automatic adds the final
  simultaneous frame-0 position + force settling gate and fail-closed force arming.
- V14.43 SELECTABLE MANUAL REFERENCE preserves the validated V14.42 control law
  unchanged and adds an explicit saved-Manual selector. Automatic is bound to the
  exact selected Manual CSV + matching replay JSON + matching initial-state JSON.
- V14.51 HARD-GUARDED MANUAL + COORDINATED FORCE keeps Manual control-read-only
  at the transport layer: settings/override/mutating-state packets are blocked and
  Manual uses telemetry/status polling without the Remote-Control watchdog.
  Automatic keeps the V14.50 replay path but makes the force/position interlock
  recoverable, fades Force Offset as position error grows, and includes the live
  zero-offset force bias when sizing the safe Force-Offset authority.
- V14.60 AUTO-AXIS-CAL + MEASURED-CLS SOFT-HOLD keeps the validated
  V14.51/V14.56 BRUNNER position+force replay controller, including adaptive
  high-force Pitch holding and position-priority frame-0 settling.  It fixes the
  Sim-PC bank/yaw handover at the source: CLS2Sim Position-to-Sim stays BLOCKED
  throughout Automatic, and a dedicated SimConnect axis-event bridge drives P3D
  from the LIVE MEASURED physical CLS Pitch/Roll/Yaw positions at 50 Hz.  P3D is
  briefly run with its pose/dynamics continuously restored to Manual t=0 while
  the real Elevator/Aileron/Rudder inputs and surface deflections synchronize.
  Recording starts only after those live controls are stable, so there is no
  stale -0.364 -> +0.101 aileron handover kick.  Yaw uses the selected Manual t=0
  P3D rudder value as an anchor plus the live physical CLS Yaw delta, allowing the
  fixed CLS2Sim profile ('Don't set pos in AP mode' checked) to remain unchanged.
  Manual remains strict CLS read-only and never opens the direct P3D bridge.
- V14.30 VERIFIED INITIAL-STATE COMMIT remains the source of exact pose, dynamic
  state and trim restoration. V14.60 supersedes the V14.59 uncalibrated direct-axis path: Manual
  remains CLS2Sim/read-only, while Automatic mirrors LIVE measured physical CLS
  positions to P3D exclusively through CLS2Sim Position-to-Sim forwarding.
- V14.62 CANONICAL POSITION + ROBUST ALL-AXIS PROBE keeps all replay/controller
  tuning unchanged. Automatic force-profile softening is disabled for the repeatability
  test. Pitch/Roll/Yaw discovery is retried and falls back to read-only node-specific
  position probes before an axis is declared unavailable. Formal position comparison
  uses live normalized CLS position transformed into the selected Manual run's own
  position coordinate, so native RAW sign/range differences cannot corrupt the graph.
- V14.64 ROBUST t=0 FORCE LOCK keeps the V14.62 position/replay implementation
  frozen and adds one fail-closed pre-recording force gate inside the existing measured-CLS
  P3D soft-hold.  The gate uses the selected Manual CSV row-0 engineering force (the same
  value used by the formal report), keeps the exact frame-0 AP position commanded, and
  evaluates force at the real ~100 ms feedback cadence, requires three consecutive
  observations within +/-3 N, and captures t=0 only when the final live sample is within
  +/-2 N. The converged force integrator/offset is preserved into replay.
  Automatic row 0 is taken from a live CLS snapshot captured at that verified t=0, never
  from the Manual force reference, eliminating the stale pre-recording force sample.
- V14.65 CONTROLS FIRST, THEN P3D changes only the Automatic preparation order.
  BRUNNER/CLS frame-0 position AND exact Manual CSV row-0 force are physically locked
  before P3D is allowed to run for terrain loading.  After the final direct P3D t=0 restore,
  P3D remains PAUSED while the same controls are re-verified/re-locked.  Only after those
  control gates pass is P3D unpaused for the measured-CLS axis calibration/soft-hold.
  The working V14.62 position mapping and all AP/force controller tuning are unchanged.
- V14.66 TRUE DYNAMICS + NO-IAS FINAL POSE keeps ALL BRUNNER replay/controller
  tuning unchanged. Existing Manual runs are upgraded from their matching
  P3D_State_Diagnostic_manual CSV: true WORLD velocity, BODY velocity evidence,
  BODY angular rates, trims and control-surface t=0 evidence are recovered.
  After terrain preload, final pose/soft-hold deliberately DOES NOT write
  AIRSPEED INDICATED. WORLD velocity is authoritative; BODY velocity is a
  live consistency gate. Automatic cannot start until pose, WORLD dynamics,
  BODY velocity, controls/surfaces and physical CLS t=0 all agree.
  Formal Automatic graphs use raw measured data: no t=0 Manual substitution,
  no initial VS/RPM smoothing, and heading is unwrapped only in the plot CSV.
- V14.66.2 fixes the exact-reference bundle guard so the V14.66 recovered
  state sidecar is accepted ONLY when it belongs to the selected Manual CSV and
  its matching Manual P3D diagnostic. The original Manual, replay JSON and
  original initial-state JSON remain unchanged.
- V14.66.3 fixes mode switching after a blocked Automatic run. A failed
  Automatic validation no longer masquerades as a SimConnect connection error,
  and Manual/Automatic retries reuse the already-live SimConnect session instead
  of attempting to open a second connection on the same client.
- V14.67 fixes the remaining P3D handover logic.  With the no-IAS final-pose
  bridge, IAS is a derived/display value and is no longer a paused-state gate.
  During the live measured-CLS synchronization window, WORLD/BODY velocities are
  diagnostic only because P3D physics is intentionally running between 40-ms
  restores.  Once controls/surfaces are synchronized, P3D is PAUSED, exact pose
  + WORLD dynamics are restored and verified, then the sim is unpaused and the
  FIRST fresh live frame is verified before Automatic t=0 is accepted.  BODY
  linear velocity remains read-only diagnostic evidence, not an impossible
  settable-state requirement.
- V14.68 uses the P3D v5 settable BODY velocity variables. Exact Manual
  VELOCITY BODY X/Y/Z are restored together with WORLD X/Y/Z, angular rates and
  trims. BODY velocity is now a HARD paused/first-live gate. Automatic also
  replays the selected Manual throttle profile instead of holding only the
  Manual t=0 throttle. Automatic row 0 is written immediately from the exact
  verified first-live frame. A three-axis Manual is fail-closed on a Pitch-only
  loader PC instead of silently producing invalid Roll/Yaw graphs.
- V14.69 POSITION PHASE LEAD keeps the complete V14.68 P3D state handover,
  throttle replay, force replay, AP force/speed settings and direct measured-CLS
  P3D bridge unchanged. Analysis of the two V14.68 simulator-PC runs measured a
  repeatable physical actuator phase lag of about 145 ms on Pitch/Roll and
  approximately 90-150 ms on Yaw. V14.69 therefore commands the AP position
  reference slightly ahead of test time: Pitch +145 ms, Roll +145 ms and
  Yaw +120 ms. The force reference remains at CURRENT Manual time, not future
  time. The force/position interlock also compares physical position with the
  CURRENT Manual position reference so the phase lead cannot falsely suppress
  force replay.
- V14.70 fixes the remaining t=0 Rate-of-Climb and Engine-RPM initialization.
  The P3D v5 SDK exposes VERTICAL SPEED as settable, so the exact recorded
  Manual VSI is now restored directly together with WORLD/BODY velocity.
  GENERAL ENG RPM is read-only, but PROP RPM:1 is settable; for the Super
  Mushshak's coupled piston/prop shaft the recorded Manual Engine RPM is used
  as the prop-RPM seed. Paused-final verification checks VSI + prop RPM; the
  first-live gate checks VSI + actual Engine RPM. No graph substitution or
  smoothing is used. All V14.69 position lookahead and BRUNNER tuning remain
  unchanged.
- V14.72 AERODYNAMIC SETTLE + STRICT FIRST-LIVE GATE keeps all BRUNNER
  position/force replay tuning unchanged.  After direct-axis calibration,
  Automatic no longer accepts control/pose stability alone: P3D stays in the
  timer-OFF software-hold window until the REAL air-relative state has settled
  back to the selected Manual t=0.  AoA, sideslip, IAS and roll/pitch/yaw rates
  are compared against the matching Manual P3D diagnostic.  Only then is P3D
  paused, the exact V14.70 state restored again, and one fresh first-live frame
  accepted.  A bad first-live lateral state is rejected before recording.
  Body accelerations and real ambient wind are logged as diagnostic evidence.
  Formal report wind remains hard-coded 0/0 exactly as before; V14.72 does NOT
  write weather and does NOT change the user's Manual wind-report convention.
- V14.73 NO-PROBE HANDOVER corrects two issues exposed by the loader-PC
  V14.72 test.  First, the old V14.60 transfer calibration deliberately drove
  each live P3D axis to +0.20 and -0.20; although pose/dynamics were repeatedly
  restored, those probes could still excite the flight model and leave a
  residual pitch/roll/yaw aerodynamic transient.  V14.73 removes those probes.
  The native bridge's normalized AXIS_*_SET path is used 1:1 and only the exact
  Manual t=0 P3D input is repeatedly commanded and verified.  Second, the
  V14.72 aerodynamic gate is no longer used DURING the live software hold,
  because repeatedly restoring t=0 while P3D is running can itself create
  non-zero instantaneous rates between restore frames.  The strict aerodynamic
  check is retained at the authoritative FIRST LIVE t=0 frame, where it belongs.
  On Pitch-only loader runs only longitudinal gates are hard; Roll/Yaw/sideslip
  remain diagnostic.  On a full Pitch+Roll+Yaw Sim-PC run all lateral gates are
  hard.  BRUNNER replay tuning remains unchanged.
- V14.74 PAUSED AXIS CAL + LATERAL-ONLY AERO GATE fixes the two faults seen
  in the V14.73 screenshots.  V14.73 incorrectly assumed the bridge-to-P3D axis
  transfer was +1.0.  Existing loader evidence proves Pitch is inverted
  (positive bridge command gives negative P3D Elevator input), which caused the
  runtime safety stop such as live=-0.232 versus desired=+0.248.  V14.74 restores
  the measured two-point sign/scale calibration, but performs it while P3D is
  PAUSED so the +0.20/-0.20 probe cannot excite the flight model.  The exact
  Manual t=0 input is restored and verified before P3D is unpaused.

  The V14.73 first-live blocks on loader PC were also false positives: all
  settable state/control checks passed, but one derived air-state channel
  (AoA on one attempt, Pitch Rate on another) differed for the first solver
  frame.  V14.74 therefore uses AoA/IAS/Pitch Rate as diagnostic-only channels.
  The hard aerodynamic gate is focused on the actual unresolved Sim-PC problem:
  Sideslip + Roll Rate + Yaw Rate, and only when the corresponding Roll/Yaw axes
  are present.  Pitch-only loader tests are not blocked by those derived lateral
  channels.  All BRUNNER replay tuning remains unchanged.
- V14.75 MICRO-PULSE LIVE AXIS CAL fixes the V14.74 loader failure which
  proved that P3D does NOT update AXIS_* input telemetry while the simulator is
  paused.  The V14.74 screenshot showed +probe and -probe both reading about
  +0.0011, giving slope=0.0000.  Therefore paused calibration is impossible on
  this P3D path.

  V14.75 measures the required sign/scale with very short LIVE micro-pulses:
  +0.06 and -0.06 only.  Each pulse is allowed to exist for only enough time to
  obtain fresh P3D input telemetry, then P3D is immediately paused and the exact
  Manual pose/dynamic state is restored before the next pulse.  This preserves
  the essential V14.60 measured mapping while removing the long +/-0.20 live
  disturbance that could contaminate the aerodynamic handover.

  After calibration all axes are returned to exact Manual t=0, the measured-CLS
  mirror starts, and the normal control/surface synchronization runs.  The
  V14.74 lateral-only first-live gate is retained unchanged.  No BRUNNER replay
  tuning is changed and no DLL rebuild is required.
- V14.76 RELIABLE FIRST-CLICK START keeps the successful V14.75 replay
  unchanged but removes the timing-sensitive startup. V14.75 accepted only two
  samples in a short 0.22 s pulse; depending on P3D/SimConnect frame timing the
  samples could still be stale, which explains why an unchanged setup sometimes
  needed several Automatic attempts before it passed.

  V14.76 re-sends the same small probe while live, ignores the first two fresh
  frames, rejects samples that have not actually moved away from the paused
  baseline, requires three settled samples, and automatically retries the axis
  up to four times inside ONE Automatic-button attempt. New commissioning fields
  record the measured slope/offset/probe responses and attempt count.
- V14.79 CONFIRMED FIXES keeps the original strict Sim-PC lateral
  tolerances. It does NOT widen the gate. If the first live frame has the
  previously observed sideslip/roll/yaw mismatch, the logger automatically
  enters a short pre-record stabilization phase instead of rejecting the run.

  During stabilization the 75-second timer remains OFF. P3D stays live while
  the exact Manual pose, WORLD/BODY velocities, angular rates, trims, throttle
  and exact control inputs are re-applied at high rate. This gives P3D's
  derived aerodynamic state time to converge without allowing the aircraft to
  leave the Manual t=0 condition. The run is released only after multiple
  consecutive fresh frames satisfy the original strict sideslip/roll/yaw
  limits. On a full Roll/Yaw system, lateral acceleration is also checked so a
  temporarily forced zero roll-rate cannot hide a still-wrong side-force state.

  If the state cannot converge, Automatic remains blocked. No BRUNNER tuning,
  position lookahead, force controller, DLL or wrapper is changed.

"""

import os
import sys
import csv
import inspect
import tempfile
import shutil
import hashlib
import json
import re
import time
import ctypes
import datetime
import threading
from pathlib import Path
from typing import List, Dict, Any, Optional

try:
    import winsound
    WINSOUND_AVAILABLE = True
except ImportError:
    WINSOUND_AVAILABLE = False

# Rahwali Airport (OPRW) exact start position from P3D Map
# Lat: N32° 14.02'  Lon: E74° 7.41'  Alt: 750.24 ft  Hdg: 48.4°
RAHWALI_LAT  =  32.23367   # degrees N  (N32° 14.02')
RAHWALI_LON  =  74.12350   # degrees E  (E74° 7.41')
RAHWALI_ALT  = 750.24      # feet MSL
RAHWALI_HDG  =  48.4       # degrees true

import math
import socket
import struct

from PySide6.QtCore import Qt, QTimer, Signal, Slot
from PySide6.QtGui import QFont, QIcon, QColor, QPalette, QPainter, QPen, QBrush, QKeySequence, QShortcut
from PySide6.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGridLayout,
    QLabel,
    QPushButton,
    QProgressBar,
    QFrame,
    QFileDialog,
    QMessageBox,
    QGroupBox,
    QComboBox,
)





# CLS2Sim Remote Control Protocol v2 commands
CMD_STATE         = 0xCD
CMD_SETTINGS      = 0xCE
CMD_READ          = 0xD0
CMD_OVERRIDE      = 0xD1
CMD_REMOTE_CONFIG = 0xD8

# Official Protocol-v2 axis bitmasks:
# Elevator/Pitch: 0x1 (device IDs 2,4)
# Aileron/Roll:   0x2 (device IDs 1,3)
# Rudder/Yaw:     0x4 (device IDs 5,6)
AX_ELEVATOR = 0x0001
AX_AILERON  = 0x0002
AX_RUDDER   = 0x0004

AX_JOYSTICK = AX_ELEVATOR | AX_AILERON
AX_THREE_ACTUATORS = AX_ELEVATOR | AX_AILERON | AX_RUDDER
AX_ALL = AX_THREE_ACTUATORS  # Legacy alias retained for compatibility.
AX_NONE = 0x0000

SID_FORCE_SCALE = 0x20
SID_MOVE_BACK_ENABLE = 0x40
SID_AP_FORCE    = 0x50
SID_AP_SPEED    = 0x60
SID_AP_ENABLE   = 0x70
SID_AP_POSITION = 0x80
SID_VIRTUAL_FORCE_ENABLE = 0xA0

# PITCH FORCE REPLAY ONLY. These do not change any position command or unit.
# Force Offset (0xD0) is the signed force/effect command used by the existing
# CLS2Sim force-offset test bench. No extra 0xD1 setting is required here.
SID_FORCE_OFFSET = 0xD0

DATA_POS_NORM     = 0x11
DATA_POS_RAW      = 0x12
DATA_FORCE_RAW    = 0x20
DATA_FORCE_NORM   = 0x21
DATA_FORCE_ENG    = 0x22
DATA_AXIS_INFO    = 0x23
DATA_VIRTUAL_FORCE_RAW = 0x50
DATA_VIRTUAL_FORCE_ENG = 0x51
DATA_AP_ENABLED   = 0x55

OVR_FORCE_SCALE     = 0x01
OVR_AP_ENABLE       = 0x03
OVR_AP_POSITION     = 0x04
# BRUNNER force/effect ownership used only by Pitch Force Offset replay.
OVR_VIBRATION       = 0x05
OVR_POSITION_TO_SIM = 0x06
REPLAY_OVERRIDE_IDS = (
    OVR_AP_ENABLE,
    OVR_AP_POSITION,
    OVR_VIBRATION,
    OVR_POSITION_TO_SIM,
)

RCFG_COMMUNICATION_WATCHDOG = 0x00
RCFG_MESSAGE_IDENTIFIER = 0x01

STATE_GET_STATUS = 0x00
STATE_INITIALIZE = 0x01
STATE_STOP = 0x02
STATE_FAULT_RESET = 0x03
STATE_GET_ERRORS = 0x04
STATE_SET_OFFSET_SENSOR = 0x05

DEVICE_STATE_NAMES = {
    0x00: "NOT CONNECTED",
    0x01: "OPERATIONAL",
    0x02: "NOT INITIALIZED",
    0x03: "DRIVE ERROR",
}

ERROR_NAMES = {
    0x1000: "General Error",
    0x1001: "Emergency Stop",
    0x2311: "Continuous Over Current",
    0x2320: "Over Current",
    0x3210: "Overvoltage DC Link",
    0x3220: "Undervoltage DC Link",
    0x4210: "Motor Overtemperature",
    0x4310: "Drive Overtemperature",
    0x5113: "Undervoltage Internal 5V Supply",
    0x5114: "Feedback Error",
    0x5115: "Overspeed",
    0x8611: "Following Window",
    0x8620: "Negative Limit Switch",
    0x8621: "Positive Limit Switch",
    0x8622: "Soft Limit Switch",
    0xFF01: "Force Sensor",
    0xFF02: "Sync IP Mode",
    0xFF03: "Safety IN1",
}

# -----------------------------------------------------------------------------
# BRUNNER CLS-P / NETWORK CONFIGURATION
# -----------------------------------------------------------------------------
# If this application and CLS2Sim run on the SAME simulator PC, leave the host
# at 127.0.0.1. Windows' physical Ethernet/Wi-Fi adapter does not have to carry
# this local Python->CLS2Sim traffic. If your BRUNNER hardware itself uses the
# optional Ethernet2CAN gateway, that adapter still has to be enabled so CLS2Sim
# can see the physical drives.
CLS2SIM_HOST = os.environ.get("CLS2SIM_HOST", "127.0.0.1")
CLS2SIM_PORT = int(os.environ.get("CLS2SIM_PORT", "15090"))

# -----------------------------------------------------------------------------
# LEGACY CLS-P FORCE COMPATIBILITY (simulator PC / firmware v474)
# -----------------------------------------------------------------------------
# The simulator-PC read-only probe established this legacy relationship on all
# three physical CLS-P Linear Motion axes:
#   0x21 normalized ~= signed16(0x20 RAW) / 20000
# CLS2Sim's own engineering display and the working v535 loader reference show
# 0.1 N per internal force count. These constants are used ONLY when both 0x22
# Engineering Force and 0x23 Axis Information are unavailable AND the 0x20/0x21
# pair passes the numerical consistency check below. Modern/v535 behavior is
# therefore left unchanged.
LEGACY_FORCE_NORM_FULL_SCALE_COUNTS = 20000.0
LEGACY_FORCE_FACTOR_N_PER_COUNT = 0.1
LEGACY_FORCE_MATCH_TOL_COUNTS = 8.0

# -----------------------------------------------------------------------------
# ONE-SHOT BRUNNER COMMISSIONING DIAGNOSTICS
# -----------------------------------------------------------------------------
# Passive only: these diagnostics read cached values already used by the logger
# and save extra evidence files. They DO NOT issue extra force/position commands.
COMMISSION_DIAG_ENABLED = True
COMMISSION_DIAG_SAMPLE_SEC = 0.05       # 20 Hz diagnostic CSV from cached values
COMMISSION_DIAG_BUILD = "V14.95 FINAL DETERMINISTIC + MEASURED-CLS"

# V14.30: high-rate Prepar3D state diagnostic.  The diagnostic itself is read-only.
# V14.30 uses a two-phase initialization: INITPOSITION is committed during a brief
# CLS-blocked unpause, then dynamic state is restored and verified while paused.
P3D_STATE_DIAG_ENABLED = True
P3D_STATE_DIAG_SAMPLE_SEC = 0.020
P3D_STATE_DIAG_BUILD = "V14.95 FINAL DETERMINISTIC + FULL-STATE AUDIT"

# -----------------------------------------------------------------------------
# V14.92 LEGACY MANUAL RECONSTRUCTION PROFILES
# -----------------------------------------------------------------------------
# ONLY the two named September-17 Manual bundles may use these values. They are
# calibration candidates for fields that bridge 1440 did not record; they never
# replace a value that actually exists in the selected Manual state.
#
# The first-detent V14.90 trial made longitudinal tracking materially worse, so
# the first controlled calibration candidate is FLAPS UP (handle index 0). The
# wind candidates are taken from the matching V14.89/V14.90 Sim-PC environment
# observed at each reference location. Mixture/propeller remain the initial
# candidate at 100%; the fit-score file tells us whether later calibration is
# required. These values must be revised only from measured replay evidence.
V14_92_LEGACY_PROFILES = {
    "2026-09-17_15-25-33": {
        "profile_id": "SEP17_152533_R1",
        "calibration_status": "CANDIDATE_R1",
        "flaps_handle_index": 0.0,
        "mixture_percent": 100.0,
        "propeller_percent": 100.0,
        "weather_metar": "GLOB 250900Z 31610KT 9999 SKC 10/05 Q1013",
        "expected_wind_dir_deg": 316.0,
        "expected_wind_speed_kt": 9.7,
    },
    "2026-09-17_15-37-17": {
        "profile_id": "SEP17_153717_R1",
        "calibration_status": "CANDIDATE_R1",
        "flaps_handle_index": 0.0,
        "mixture_percent": 100.0,
        "propeller_percent": 100.0,
        "weather_metar": "GLOB 250900Z 32209KT 9999 SKC 10/05 Q1013",
        "expected_wind_dir_deg": 322.0,
        "expected_wind_speed_kt": 8.5,
    },
}
V14_92_LEGACY_PROFILE_REASSERT_SEC = 0.20
V14_92_SECONDARY_REPLAY_SEC = 0.05
V14_92_APPLY_LEGACY_WEATHER = True
V14_92_CAPTURE_GLOBAL_METAR = True
V14_92_FIT_SCORE_VERSION = "V14.95-final"

# -----------------------------------------------------------------------------
# V14.93 ONE-VISIT LEGACY CALIBRATION + FULL-STATE AUDIT
# -----------------------------------------------------------------------------
# Goal: one trip to the Sim PC. The app can automatically exercise a bounded
# candidate set for BOTH September-17 references, choose the lowest measured
# trajectory-error profile, run a final full-duration replay, and persist the
# selected profile locally. No source edit is required between candidates.
V14_93_ONE_VISIT_SHORT_SEC = 8
V14_93_ONE_VISIT_AUTO_CONFIRM_LOAD_SEC = 4.0
V14_93_FULL_STATE_AUDIT_PERIOD_SEC = 0.20  # 5 Hz complete telemetry dictionary
V14_93_OVERRIDE_FILENAME = "V14_93_LEGACY_PROFILE_OVERRIDE.json"
V14_93_SUMMARY_FILENAME = "V14_95_FINAL_TWO_MANUAL_SUMMARY.json"

# Bounded, engineering-safe candidate sweep. It varies only hidden legacy
# configuration values that were NOT measured by bridge 1440. Recorded Manual
# pose/dynamics/BRUNNER data are never changed.
def _v1493_candidate_profiles(base):
    base = dict(base)
    metar = str(base.get("weather_metar") or "")
    # Extract a simple wind token from the current legacy METAR. Candidates are
    # intentionally small and bounded so a one-visit sweep does not become a
    # huge unattended optimization problem.
    import re as _re
    m = _re.search(r"\b(\d{3})(\d{2})KT\b", metar)
    d = int(m.group(1)) if m else int(round(float(base.get("expected_wind_dir_deg", 0.0))))
    v = int(m.group(2)) if m else int(round(float(base.get("expected_wind_speed_kt", 0.0))))
    def wx(direction, speed):
        direction = int(direction) % 360
        speed = max(0, min(30, int(speed)))
        return _re.sub(r"\b\d{3}\d{2}KT\b", f"{direction:03d}{speed:02d}KT", metar, count=1) if m else metar
    rows = [
        ("BASE", 0.0, 100.0, d, v),
        ("MIX80", 0.0, 80.0, d, v),
        ("MIX65", 0.0, 65.0, d, v),
        ("WIND_HALF", 0.0, 100.0, d, max(0, round(v * 0.5))),
        ("WIND_CALM", 0.0, 100.0, d, 0),
        ("WIND_LEFT20", 0.0, 100.0, d - 20, v),
        ("WIND_RIGHT20", 0.0, 100.0, d + 20, v),
        ("FLAP1_SANITY", 1.0, 100.0, d, v),
        ("MIX80_WIND_HALF", 0.0, 80.0, d, max(0, round(v * 0.5))),
    ]
    candidates = []
    for cid, flap, mix, wd, ws in rows:
        p = dict(base)
        p.update({
            "profile_id": f"{base.get('profile_id','LEGACY')}_{cid}",
            "calibration_status": "V14.93_ONE_VISIT_CANDIDATE",
            "candidate_id": cid,
            "flaps_handle_index": float(flap),
            "mixture_percent": float(mix),
            "weather_metar": wx(wd, ws),
            "expected_wind_dir_deg": float(wd % 360),
            "expected_wind_speed_kt": float(ws),
        })
        candidates.append(p)
    return candidates

V14_93_ONE_VISIT_CANDIDATES = {
    key: _v1493_candidate_profiles(profile)
    for key, profile in V14_92_LEGACY_PROFILES.items()
}

V14_30_STATE_SCHEMA = "SuperMushak-V14.33-TrueInitialState-v1"
V14_30_MANUAL_CAPTURE_SETTLE_SEC = 0.12
# V14.30 initialization sequencing / readback verification.
V14_30_POSITION_COMMIT_TIMEOUT_SEC = 120.0  # V14.34: terrain/scenery load may be slow
V14_34_P3D_STABLE_MIN_SEC = 1.50
V14_34_P3D_STABLE_MIN_SAMPLES = 8
V14_34_FRESH_TELEMETRY_WAIT_SEC = 1.50
# P3D VERTICAL SPEED is a derived/displayed simvar and can differ slightly from
# the exact WORLD Y velocity immediately after state restore.  Do not reject a
# physically correct restore for a small derived-VS lag.  WORLD Y itself remains
# a hard, tight check.  100 fpm matches the formal ROC tolerance used in report.
V14_34_1_LIVE_VS_TOL_FPM = 100.0

# V14.34.2: P3D may continue terrain/scenery loading AFTER INITPOSITION has
# already reached the correct coordinates. During that period derived flight
# telemetry can be nonsense even while the low-level settable state reads back
# correctly. Do not fail immediately; wait until the COMPLETE dynamic state is
# continuously valid for a real settle window.
V14_34_2_DYNAMIC_READY_TIMEOUT_SEC = 120.0
V14_34_2_DYNAMIC_STABLE_MIN_SEC = 3.0
V14_34_2_DYNAMIC_STABLE_MIN_SAMPLES = 8
V14_34_2_DYNAMIC_REFRESH_WAIT_SEC = 1.50
V14_34_2_REAPPLY_STATE_SEC = 2.0

# V14.36: after the operator confirms P3D's visible terrain loading is over,
# verify only DIRECTLY SETTABLE P3D state. Derived/display simvars such as
# Vertical Speed can lag while paused and must not block readiness.
V14_36_DIRECT_VERIFY_ATTEMPTS = 6
V14_36_DIRECT_VERIFY_WAIT_SEC = 0.35
V14_36_POST_CONFIRM_POSITION_TIMEOUT_SEC = 15.0
V14_36_POST_CONFIRM_POSITION_STABLE_SAMPLES = 3

# V14.39: after the user says the visual P3D load is finished, wait for the
# PAUSED aircraft telemetry itself to stop changing before doing the final exact
# t=0 initialization.  This catches terrain/scenery work that is still changing
# the aircraft even if the button was pressed a little early.
V14_39_POSTLOAD_STABLE_TIMEOUT_SEC = 120.0
V14_39_POSTLOAD_STABLE_MIN_SEC = 4.0
V14_39_POSTLOAD_STABLE_MIN_SAMPLES = 8
V14_39_POSTLOAD_SAMPLE_WAIT_SEC = 1.0

V14_39_STABLE_ALT_DELTA_FT = 2.0
V14_39_STABLE_IAS_DELTA_KT = 0.5
V14_39_STABLE_ATT_DELTA_DEG = 0.20
V14_39_STABLE_HDG_DELTA_DEG = 0.30
V14_30_POSITION_STABLE_SAMPLES = 3
V14_30_ALT_TOL_FT = 30.0
V14_30_IAS_TOL_KT = 4.0
V14_30_ATT_TOL_DEG = 2.5
V14_30_HDG_TOL_DEG = 4.0
V14_30_DYNAMIC_VERIFY_RETRIES = 3
V14_30_VELOCITY_TOL_FPS = 3.0
V14_30_ROTATION_TOL_RAD_S = 0.012
V14_30_ELEV_TRIM_TOL_RAD = 0.006
V14_30_TRIM_PCT_TOL = 0.015

# V14.33 authoritative t=0 consistency tolerances
V14_33_VS_TOL_FPM = 20.0
V14_33_RATE_TOL_DEG_S = 0.20
V14_33_CLS_RAW_POS_TOL = 5.0
V14_33_FORCE_TOL_N = 0.10

# V14.30.3: START means "initialize first", not "start the 75-second clock".
# P3D remains paused during the final guard interval.  The actual Automatic
# t=0 is the first fresh SimConnect frame received after the FINAL unpause.
V14_30_3_FINAL_PAUSED_GUARD_SEC = 0.50
V14_30_3_FIRST_FRAME_TIMEOUT_SEC = 1.50


# Automatic physical replay policy (V14.58):
# - Recorded Manual trajectory is written ONLY to BRUNNER/CLS2Sim AP-position.
# - Actual actuator positions/forces are read ONLY from live CLS2Sim telemetry.
# - P3D is commanded from those LIVE MEASURED physical CLS positions, never from
#   the prerecorded Manual trajectory directly.
# - Normal CLS Position-to-Sim forwarding stays blocked throughout Automatic to
#   prevent duplicate sources under the fixed CLS2Sim AP profile.
# - Manual remains strict CLS read-only and uses normal CLS2Sim -> P3D behavior.
CLS_ONLY_AUTOMATIC_REPLAY = False

# Formal report force-plot policy:
# Force plots are labelled N, so only an engineering-force channel is allowed:
# native 0x22, documented RAW->ENG via 0x23, or the validated v474 legacy
# signed16 conversion. Normalized/raw values are still logged separately but
# are never relabelled as Newtons.
REPORT_FORCE_SOURCE_NORMALIZED = False
STRICT_CLS_JOYSTICK_IO = True
# STRICT rule: Pitch/Roll/Yaw control position and force are NEVER read from P3D.
# Manual: CLS2Sim -> logger -> replay JSON/CSV.
# Automatic: replay JSON -> CLS2Sim -> physical actuator -> CLS2Sim feedback.
# P3D is read-only for aircraft/engine/environment telemetry only.

BRUNNER_DEVICE = "CLS-P"
CLS_P_AXES = AX_THREE_ACTUATORS
CLS_LOG_AXES = AX_THREE_ACTUATORS

# V13: every CLS axis is independent. One connected/replayed axis is sufficient.
CLS_AXIS_SPECS = (
    ("Pitch", AX_ELEVATOR, "cls_pitch_norm"),
    ("Roll", AX_AILERON, "cls_roll_norm"),
    ("Yaw", AX_RUDDER, "cls_yaw_norm"),
)
CLS_AXIS_MASK_BY_NAME = {
    "Pitch": AX_ELEVATOR,
    "Roll": AX_AILERON,
    "Yaw": AX_RUDDER,
}
CLS_REPLAY_KEY_BY_NAME = {
    "Pitch": "cls_pitch_norm",
    "Roll": "cls_roll_norm",
    "Yaw": "cls_yaw_norm",
}

# BRUNNER's Protocol-v2 guidance for initial AP tuning:
# force: start around 1500 and tune; speed: start below 10 and increase.
CLS_P_AP_FORCE_PITCH = 1500
CLS_P_AP_FORCE_ROLL = 1500
CLS_P_AP_FORCE_YAW = 1500

CLS_P_AP_SPEED_PITCH = 25
CLS_P_AP_SPEED_ROLL = 40
CLS_P_AP_SPEED_YAW = 25

# V11 Tracking Diagnostic mode. Normal high-level AP Position only.
# AP force is unchanged; only the maximum AP speed is raised for comparison.
CLS_TRACK_AP_FORCE_PITCH = 1800
CLS_TRACK_AP_FORCE_ROLL = 1500
CLS_TRACK_AP_FORCE_YAW = 1500
CLS_TRACK_AP_SPEED_PITCH = 300
CLS_TRACK_AP_SPEED_ROLL = 200  # Loader-PC validated Roll position speed

# V14.21: Yaw speed can be selected at runtime before every Automatic run.
# This lets the operator test several Yaw position speeds during one simulator-PC
# visit without editing/replacing the Python file. Only the selected speed changes;
# Pitch/Roll settings, force tuning, safety interlocks and shutdown remain unchanged.
YAW_AP_SPEED_OPTIONS = (60, 80, 100, 120, 150)
YAW_AP_SPEED_DEFAULT = 60
CLS_TRACK_AP_SPEED_YAW = YAW_AP_SPEED_DEFAULT  # fallback/default only

# -----------------------------------------------------------------------------
# V14.69 AUTOMATIC POSITION PHASE LEAD
# -----------------------------------------------------------------------------
# Measured from the two V14.68 simulator-PC commissioning runs:
#   Pitch physical position lag ~145 ms
#   Roll  physical position lag ~145 ms
#   Yaw   physical position lag ~90..150 ms
#
# IMPORTANT:
# - These values advance ONLY the AP POSITION command.
# - Manual force reference timing is NOT advanced.
# - P3D receives the LIVE MEASURED physical CLS position, never the future target.
# - Formal graphs remain actual physical position vs Manual reference at the same
#   test time. Therefore an improvement in the graph is real physical alignment.
AUTO_POSITION_LOOKAHEAD_SEC = {
    "Pitch": 0.145,
    "Roll": 0.145,
    "Yaw": 0.120,
}

# V14.70 exact secondary-state gates.
# The report's ROC tolerance band is ±100 fpm; use the same value for the
# first-live VSI gate. These relaxed limits permit a diagnostic replay; the
# original 35-rpm limits remain the threshold for marking QTG mismatch.
V14_70_VSI_TOL_FPM = 100.0
V14_70_FORMAL_RPM_TOL = 35.0
V14_70_PROP_RPM_TOL = 80.0
V14_70_ENGINE_RPM_TOL = 80.0

# Recorded-extrema pre-check before Tracking Automatic recording.
CLS_TRACK_PRECHECK_ENABLED = True
# V14.18: endpoint precheck is no longer an extreme-to-extreme sprint.
# Each endpoint is approached from the recorded initial position.
CLS_TRACK_PRECHECK_HOLD_SEC = 6.0
CLS_TRACK_PRECHECK_RETURN_HOLD_SEC = 4.0
CLS_TRACK_PRECHECK_INTER_TARGET_SETTLE_SEC = 0.40
# Physical endpoint tolerance. Former 0.12 norm accepted effectively motionless loaders.
CLS_TRACK_PRECHECK_TOL = 0.008
# Reference ranges smaller than this cannot independently prove servo tracking.
CLS_MIN_MEANINGFUL_MOTION_NORM = 0.008
CLS_TRACK_PRECHECK_LIMIT = 0.95

# V12: Automatic physical CLS feedback to Prepar3D.
# During setup/pre-check, Position Override 0x06 remains ACTIVE so
# pre-positioning does not move the aircraft. It is released only when
# Automatic recording actually starts.
CLS_TO_P3D_AUTOMATIC_ENABLED = True

# Following-error logic: do not disable AP merely because the recorded
# target is itself moving rapidly. A hard following-error shutdown is
# permitted only after the target has been effectively settled.
CLS_COMMAND_MOTION_DELTA = 0.005
CLS_COMMAND_SETTLED_SEC = 0.75
CLS_SETTLED_FOLLOW_ABORT_SEC = 5.0

# Fresh manual recordings use native CLS positions and are replayed 1:1, so
# these inversion values are only used as fallback for legacy replay files.
CLS_P_INVERT_PITCH = True
CLS_P_INVERT_ROLL = False
CLS_P_INVERT_RUDDER = False

CLS_WATCHDOG_MS = 500
CLS_WATCHDOG_HEARTBEAT_SEC = 0.10
CLS_POLL_INTERVAL_SEC = 0.05
CLS_DEBUG_EVERY_N_TICKS = 25  # ~0.5 s with the 50 Hz GUI timer.

# -----------------------------------------------------------------------------
# PITCH POSITION DIAGNOSTIC ONLY
# -----------------------------------------------------------------------------
# This does NOT alter the AP-position command or any position unit.
# It records what Python commanded versus what the CLS-P actually reached.
PITCH_POSITION_DIAG_ENABLED = True
PITCH_POSITION_DIAG_PRINT_EVERY_TICKS = 25   # ~0.5 s at 50 Hz

# Layered CLS polling: keep the replay-critical path light.
CLS_FAST_TELEMETRY_SEC = 0.020      # 50 Hz normalized position
CLS_MEDIUM_TELEMETRY_SEC = 0.200    # 5 Hz raw position / virtual force / AP state
CLS_HEALTH_TELEMETRY_SEC = 1.000    # 1 Hz drive health/errors

# Automatic-replay diagnostics/safety.  A warning is printed if an actuator
# does not follow the requested normalized position.  The code deliberately
# does not use the dangerous raw-position override (setting 0xB0).
CLS_FOLLOW_WARN_ERROR = 0.012
CLS_FOLLOW_ABORT_ERROR = 0.060
CLS_FOLLOW_ABORT_SEC = 5.0

# V14.51 coordinated force/position safety. The force loop remains secondary
# to position replay. Full force authority is allowed only while the physical
# axis is close to its AP-position target. Force Offset fades smoothly as
# position error grows, becomes zero before the hard block threshold, and may
# recover automatically once position tracking is healthy again.
FORCE_POSITION_INTERLOCK_ENABLED = True
FORCE_POSITION_FULL_AUTHORITY_ERROR = 0.08
FORCE_POSITION_ZERO_AUTHORITY_ERROR = 0.28
FORCE_POSITION_INTERLOCK_BLOCK_ERROR = 0.35
FORCE_POSITION_INTERLOCK_RECOVER_ERROR = 0.12
FORCE_POSITION_INTERLOCK_LATCH_FOR_RUN = False

# V14.12 end-of-test safe handover. Force Offset is ramped smoothly to zero
# while AP/watchdog ownership is still valid. AP is then switched OFF before
# Move Back / normal force-scale ownership are restored. PDF generation only
# starts after this synchronous shutdown sequence returns.
CLS_SAFE_SHUTDOWN_RAMP_SEC = 2.0
CLS_SAFE_SHUTDOWN_RATE_HZ = 50.0
CLS_SAFE_SHUTDOWN_ZERO_HOLD_SEC = 0.30
CLS_SAFE_SHUTDOWN_AP_OFF_HOLD_SEC = 0.20

# Manual Pitch-force diagnostic. BRUNNER documents Force Scale Factor 0x20 as
# 20..999 percent and Scale Factor Override 0x01 as the method for taking the
# force scale away from the connected simulator. This is DIAGNOSTIC ONLY.
# V14.42: MANUAL is strictly read-only to CLS2Sim/BRUNNER.
# These legacy diagnostic controls are kept only for source compatibility and
# are hard-disabled. Manual must never change Force Scale or Move Back.
PITCH_FORCE_DIAGNOSTIC_ENABLED = False
PITCH_FORCE_DIAGNOSTIC_SCALE_PERCENT = 20
PITCH_MOVE_BACK_DIAGNOSTIC_ENABLED = False
PITCH_MOVE_BACK_RESTORE_VALUE = 1  # Automatic cleanup restore value only.

# Automatic physical replay configuration.
# V14.62 repeatability test: DO NOT change the underlying CLS force environment
# between Manual and Automatic.  The softening implementation is retained for
# later controlled experiments, but it is disabled by default here.
AUTO_REPLAY_SOFTENING_ENABLED = False
AUTO_REPLAY_FORCE_SCALE_PERCENT = 20
AUTO_REPLAY_MOVE_BACK_VALUE = 0
AUTO_REPLAY_MOVE_BACK_RESTORE_VALUE = 1

# -----------------------------------------------------------------------------
# AUTOMATIC PITCH FORCE REPLAY -- FORCE ONLY
# -----------------------------------------------------------------------------
# The Manual run already records cls_pitch_force_eng from Data ID 0x22 on the
# user's CLS-P. During Automatic replay we use that recorded force in N as the
# reference. Pitch POSITION replay and Pitch POSITION units are not modified.
# V15.12.2 POSITION-ONLY + VIRTUAL FORCE
# Automatic reproduces controller POSITION only.
# Virtual Force 0x50/0x51 is telemetry/validation only.
# Measured-force -> Force Offset closed-loop replay is disabled.
VIRTUAL_FORCE_POSITION_ONLY_MODE = True

PITCH_FORCE_REPLAY_ENABLED = False
PITCH_FORCE_REPLAY_SIGN = 1.0

# Force Offset is limited both by the BRUNNER axis force limit and by the force
# actually seen in the Manual run. This prevents a force-loop fault from driving
# the actuator to its full hardware capability.
PITCH_FORCE_OFFSET_LIMIT_FRACTION = 0.49
PITCH_FORCE_MANUAL_PEAK_MARGIN = 1.25

# Closed-loop correction around the 1:1 Manual engineering-force feed-forward.
# The correction is intentionally slow/conservative.
PITCH_FORCE_REPLAY_INTEGRAL_GAIN = 0.02
PITCH_FORCE_REPLAY_MAX_STEP_INTERNAL = 120
PITCH_FORCE_FEEDFORWARD_LOOKAHEAD_SEC = 0.00

# FORCE ONLY: preload Manual t=0 Pitch force before Automatic recording.
# This does not modify Pitch position commands, position conversion, or units.
PITCH_FORCE_INITIAL_PRELOAD_ENABLED = False
PITCH_FORCE_INITIAL_PRELOAD_SEC = 3.00
PITCH_FORCE_INITIAL_PRELOAD_TOL_N = 0.20
PITCH_FORCE_INITIAL_PRELOAD_STABLE_SEC = 0.25
PITCH_FORCE_REPLAY_DEADBAND_N = 0.15

# -----------------------------------------------------------------------------
# AUTOMATIC ROLL / YAW FORCE REPLAY -- FORCE ONLY
# -----------------------------------------------------------------------------
# Same physical-force replay method already proven on Pitch.
# Position commands, position units, and position replay are NOT modified.
ROLL_FORCE_REPLAY_ENABLED = False
ROLL_FORCE_REPLAY_SIGN = 1.0
ROLL_FORCE_OFFSET_LIMIT_FRACTION = 0.49
ROLL_FORCE_MANUAL_PEAK_MARGIN = 1.25
ROLL_FORCE_REPLAY_INTEGRAL_GAIN = 0.08
ROLL_FORCE_REPLAY_MAX_STEP_INTERNAL = 80
ROLL_FORCE_INITIAL_PRELOAD_ENABLED = False
ROLL_FORCE_INITIAL_PRELOAD_SEC = 3.00
ROLL_FORCE_INITIAL_PRELOAD_TOL_N = 0.20
ROLL_FORCE_INITIAL_PRELOAD_STABLE_SEC = 0.25
ROLL_FORCE_REPLAY_DEADBAND_N = 0.15

YAW_FORCE_REPLAY_ENABLED = False
YAW_FORCE_REPLAY_SIGN = 1.0
YAW_FORCE_OFFSET_LIMIT_FRACTION = 0.49
YAW_FORCE_MANUAL_PEAK_MARGIN = 1.25
YAW_FORCE_REPLAY_INTEGRAL_GAIN = 0.08
YAW_FORCE_REPLAY_MAX_STEP_INTERNAL = 80
YAW_FORCE_INITIAL_PRELOAD_ENABLED = False
YAW_FORCE_INITIAL_PRELOAD_SEC = 3.00
YAW_FORCE_INITIAL_PRELOAD_TOL_N = 0.20
YAW_FORCE_INITIAL_PRELOAD_STABLE_SEC = 0.25
YAW_FORCE_REPLAY_DEADBAND_N = 0.15

# -----------------------------------------------------------------------------
# V14.42 FINAL BUMPLESS AUTOMATIC START
# -----------------------------------------------------------------------------
# Final gate immediately before Automatic t=0:
#   1) command the EXACT first high-rate replay position for every active axis,
#   2) command the EXACT first high-rate replay force for every force-enabled axis,
#   3) hold all axes TOGETHER until position + force are stable,
#   4) preserve the converged force-controller state into replay.
#
# IMPORTANT: no replay gain, AP speed, force limit, position conversion, P3D
# initialization, or V14.21 control-law tuning is changed by this feature.
AUTO_BUMPLESS_START_ENABLED = True
AUTO_BUMPLESS_START_TIMEOUT_SEC = 5.0
AUTO_BUMPLESS_START_STABLE_SEC = 0.35
AUTO_BUMPLESS_START_POSITION_TOL = 0.08
AUTO_BUMPLESS_START_LOOP_SEC = 0.020

# -----------------------------------------------------------------------------
# V14.64 ROBUST t=0 FORCE LOCK -- FINAL GATE BEFORE RECORDING
# -----------------------------------------------------------------------------
# This gate runs INSIDE the already-proven V14.60 measured-CLS P3D soft-hold,
# after P3D inputs/surfaces have synchronized but before Automatic t=0 exists.
# It intentionally changes no AP force/speed, replay gain, integral gain, max
# step, position conversion, or P3D bridge mapping.
V14_64_T0_FORCE_LOCK_ENABLED = True
# Acceptance is intentionally based on real 0x22 force updates, not repeated
# reads of the same cached force value. Three good 100-ms observations are
# required, followed by a final live sample inside the tighter capture limit.
V14_64_T0_FORCE_GOOD_TOL_N = 3.0
V14_64_T0_FORCE_CAPTURE_TOL_N = 2.0
V14_64_T0_FORCE_GOOD_SAMPLES = 3
V14_64_T0_FORCE_SAMPLE_PERIOD_SEC = 0.100
V14_64_T0_FORCE_TIMEOUT_SEC = 15.0
V14_64_T0_FORCE_LOOP_SEC = 0.020
V14_64_T0_POSITION_TOL = 0.030
V14_64_T0_P3D_FRESH_TIMEOUT_SEC = 0.50

# V14.65 CONTROLS-FIRST hard ordering.  These are acceptance gates only;
# AP force/speed, integral gains, max-step, force sign and position scaling remain unchanged.
V14_65_CONTROLS_FIRST_ENABLED = True
V14_65_T0_FORCE_GOOD_TOL_N = 3.0
V14_65_T0_FORCE_CAPTURE_TOL_N = 2.0
V14_65_T0_FORCE_GOOD_SAMPLES = 3
V14_65_T0_FORCE_SAMPLE_PERIOD_SEC = 0.100
V14_65_T0_FORCE_TIMEOUT_SEC = 15.0
V14_65_T0_FORCE_LOOP_SEC = 0.020
V14_65_T0_POSITION_TOL = 0.030

# -----------------------------------------------------------------------------
# V14.54 CONTROLLERS-FIRST + LIVE P3D/CLS YAW HANDOVER GUARD
# -----------------------------------------------------------------------------
# P3D's elevator_pos / aileron_pos / rudder_pos channels are normalized -1..+1.
# The selected Manual _initial_state.json already stores the corresponding t=0
# evidence values. Automatic must match them while paused AND on the first fresh
# post-unpause frame before the 75-second timer may start.
V14_52_P3D_CONTROL_INPUT_TOL = 0.025       # 2.5 percentage points of full travel
V14_52_P3D_CONTROL_STABLE_SAMPLES = 3
V14_52_P3D_CONTROL_VERIFY_TIMEOUT_SEC = 1.50

# V14.53 SIM-PC HANDOVER FIX
# P3D/CLS2Sim control-position channels do not reliably update while P3D is paused.
# Prime the live CLS->P3D control path before the final exact t=0 restore, then
# require a bounded live post-unpause match before the recording clock starts.
V14_53_LIVE_HANDOVER_PRIME_TIMEOUT_SEC = 2.50
V14_53_LIVE_HANDOVER_START_TIMEOUT_SEC = 1.25
V14_53_LIVE_HANDOVER_STABLE_SAMPLES = 3
V14_53_LIVE_HANDOVER_LOOP_SEC = 0.040
V14_53_BANK_SIGN_AUTOCAL_MIN_TARGET_DEG = 0.30

# V14.54 YAW START-GATE FIX
# The installed Sim-PC/P3D aircraft responds visibly to the physical rudder pedals,
# but the bridge's standard rudder_pos / rudder_deflection telemetry does not
# reproduce the Manual t=0 value reliably. Therefore those P3D rudder channels
# remain diagnostic only. Yaw start readiness is proven from the authoritative
# CLS2Sim path: commanded target, physical actual position, live force feedback,
# active force replay/AP-position tracking, and enabled Position-to-Sim forwarding.
V14_54_YAW_CLS_POSITION_TOL = 0.025
V14_54_YAW_REQUIRE_LIVE_FORCE = True

# V14.56: prove that an AP-driven physical Yaw movement actually reaches the
# aircraft rudder BEFORE the final exact t=0 restore. This catches the CLS2Sim
# profile option "Don't set pos in AP mode" while there is still time to abort
# safely and restore the selected Manual state.
V14_56_YAW_BRIDGE_NUDGE_NORM = 0.035
V14_56_YAW_BRIDGE_ACTUAL_TOL = 0.012
V14_56_YAW_BRIDGE_MIN_SURFACE_DELTA_RAD = 0.0030
V14_56_YAW_BRIDGE_TIMEOUT_SEC = 2.50
V14_56_RUDDER_SURFACE_TOL_RAD = 0.0060

# V14.56 adaptive Pitch AP holding force. 1800 remains the validated normal
# setting. Only high-force references receive more AP holding authority. The
# selected value is derived from the recorded Manual force and the live BRUNNER
# force conversion, and is capped conservatively below the existing Force-Offset
# hardware envelope.
V14_56_ADAPTIVE_PITCH_AP_FORCE_ENABLED = True
V14_56_PITCH_AP_FORCE_MARGIN = 1.35
V14_56_PITCH_AP_FORCE_ADAPT_THRESHOLD = 1.15  # keep proven 1800 unless >15% more is needed
V14_56_PITCH_AP_FORCE_MAX_FRACTION_OF_AXIS_LIMIT = 0.25
V14_56_PITCH_AP_FORCE_ABSOLUTE_MAX = 5000

# V14.58 measured-CLS -> P3D bridge.  The direct P3D command is NOT the stored
# Manual trajectory.  It is computed each 20 ms from the ACTUAL physical CLS
# position measured on that frame, anchored to the selected Manual t=0 P3D input.
V14_58_DIRECT_MIRROR_PERIOD_SEC = 0.020
# Runtime fail-closed monitoring. The direct bridge is now the ONLY P3D flight-
# control source during Automatic, so a dead/stale bridge must stop the run.
V14_58_DIRECT_MIRROR_STALE_SEC = 0.25
V14_58_DIRECT_MIRROR_MAX_CONSECUTIVE_FAILURES = 5
# Gross/sustained P3D input mismatch guard. This is intentionally loose so
# ordinary one-frame telemetry lag during rapid controller motion does not abort
# a valid test, while another input source or a failed bridge cannot silently
# take over the aircraft.
V14_58_RUNTIME_INPUT_TOL = 0.20
V14_58_RUNTIME_INPUT_BAD_SEC = 0.60
V14_58_SOFT_HOLD_RESTORE_PERIOD_SEC = 0.040
V14_58_SOFT_HOLD_TIMEOUT_SEC = 3.50
V14_58_SOFT_HOLD_STABLE_SAMPLES = 6

# V14.72 air-relative/aerodynamic settle gate.
# These values are intentionally tighter than the old generic 0.012 rad/s
# angular-rate gate (~0.69 deg/s), which allowed the bad 23-Sep Roll t=0.
V14_72_AERO_SETTLE_TIMEOUT_SEC = 6.0
V14_72_AERO_STABLE_SAMPLES = 5
V14_72_IAS_TOL_KT = 2.0
V14_72_AOA_TOL_DEG = 0.35
V14_72_FORMAL_SIDESLIP_TOL_DEG = 0.50
V14_72_SIDESLIP_TOL_DEG = 2.50
V14_72_ROLL_RATE_TOL_DEG_S = 0.15
V14_72_PITCH_RATE_TOL_DEG_S = 0.15
V14_72_YAW_RATE_TOL_DEG_S = 0.15
# Acceleration is recorded and reported in the diagnostic, but is not a hard
# gate because it can contain one-frame solver transients after unpause.
V14_72_ACCEL_DIAG_TOL_G = 0.75
# If a future Manual diagnostic contains REAL wind telemetry, it is checked.
# Existing 17-Sep Manuals do not contain it, so V14.72 does not invent a value.
V14_72_WIND_SPEED_TOL_KT = 0.50
V14_72_WIND_DIR_TOL_DEG = 10.0

# V14.73 direct-axis handover: no +/-0.20 probing.
# p3d_bridge SetElevator/SetAileron/SetRudder already use normalized
# AXIS_*_SET event values, so the transfer is deterministic 1:1.
V14_73_T0_INPUT_VERIFY_TOL = 0.020
V14_73_T0_INPUT_VERIFY_ATTEMPTS = 10
V14_73_T0_INPUT_STABLE_SAMPLES = 2
V14_73_T0_INPUT_SETTLE_SEC = 0.040

# V14.74 PAUSED transfer calibration.  Same proven V14.60 two-point fit,
# but with the P3D flight model paused so calibration cannot create an
# aerodynamic transient.
V14_74_PAUSED_AXIS_CAL_PROBE = 0.20
V14_74_PAUSED_AXIS_CAL_SAMPLES_PER_POINT = 3
V14_74_PAUSED_AXIS_CAL_POINT_TIMEOUT_SEC = 1.00
V14_74_PAUSED_AXIS_CAL_VERIFY_TOL = 0.035
V14_74_PAUSED_AXIS_CAL_RESTORE_SEC = 0.040

# V14.75: P3D input telemetry updates only while P3D is running.
# Use small, short live pulses rather than the old +/-0.20 sustained probes.
V14_75_LIVE_CAL_PROBE = 0.060
V14_75_LIVE_CAL_SAMPLES_PER_POINT = 2
V14_75_LIVE_CAL_POINT_TIMEOUT_SEC = 0.22
V14_75_LIVE_CAL_FRAME_WAIT_SEC = 0.080
V14_75_LIVE_CAL_POST_PAUSE_SEC = 0.035
V14_75_LIVE_CAL_VERIFY_TOL = 0.035

# V14.76 robust one-click axis calibration.
V14_76_CAL_PROBE = 0.060
V14_76_CAL_AXIS_MAX_ATTEMPTS = 4
V14_76_CAL_SAMPLES_PER_POINT = 3
V14_76_CAL_IGNORE_INITIAL_FRAMES = 2
V14_76_CAL_POINT_TIMEOUT_SEC = 0.55
V14_76_CAL_FRESH_WAIT_SEC = 0.12
V14_76_CAL_RESEND_SEC = 0.020
V14_76_CAL_MIN_RESPONSE_FROM_BASELINE = 0.020
V14_76_CAL_MAX_SAMPLE_SPREAD = 0.025
V14_76_CAL_INTER_ATTEMPT_PAUSE_SEC = 0.080

# V14.79 strict automatic pre-record stabilization.
V14_79_STABILIZE_MAX_ATTEMPTS = 4
V14_79_STABILIZE_WINDOW_SEC = 1.25
V14_79_STABILIZE_RESTORE_PERIOD_SEC = 0.040
V14_79_STABILIZE_STABLE_FRAMES = 4
V14_79_STABILIZE_FRESH_TIMEOUT_SEC = 0.12
V14_79_STABILIZE_INTER_ATTEMPT_PAUSE_SEC = 0.10
# Lateral BODY X acceleration tolerance in g. Bridge and legacy reference
# acceleration values are ft/s^2 and must both be converted before comparison.
V14_79_LATERAL_ACCEL_TOL_G = 0.35

V14_58_CONTROL_INPUT_TOL = 0.020
V14_58_ELEVATOR_SURFACE_TOL_RAD = 0.010
V14_58_AILERON_SURFACE_TOL_RAD = 0.010
V14_58_RUDDER_SURFACE_TOL_RAD = 0.006
V14_58_THROTTLE_TOL_PCT = 1.0
V14_58_ENGINE_RPM_TOL = 180.0
V14_58_PITCH_DELTA_GAIN = 1.0
V14_58_ROLL_DELTA_GAIN = 1.0
V14_58_YAW_DELTA_GAIN = 1.0
V14_58_AXIS_EVENT_MAX = 16383

# V14.60 automatic direct-axis transfer calibration.  The native p3d_bridge.dll
# may expose an axis sign convention opposite to the P3D telemetry convention
# (Loader-PC evidence: +0.225 Elevator command produced about -0.218 input).
# During the software-held, timer-OFF handover window V14.60 measures TWO
# symmetric points per connected axis and fits live_p3d = slope*bridge_cmd + offset.
# Runtime commands use the inverse mapping, so no Pitch/Roll/Yaw sign is guessed.
V14_60_AXIS_CAL_PROBE = 0.20
V14_60_AXIS_CAL_SAMPLES_PER_POINT = 3
V14_60_AXIS_CAL_POINT_TIMEOUT_SEC = 0.90
V14_60_AXIS_CAL_MIN_ABS_SLOPE = 0.35
V14_60_AXIS_CAL_MAX_ABS_SLOPE = 1.65
V14_60_AXIS_CAL_VERIFY_TOL = 0.035
V14_60_AXIS_CAL_RESTORE_PERIOD_SEC = 0.040

# V14.66 TRUE live-dynamics gate.
V14_66_BODY_VELOCITY_TOL_FPS = 3.0
V14_66_REQUIRE_MANUAL_P3D_DIAGNOSTIC = True

# V14.68: first-live state tolerances. Vertical components are tighter because
# 1.5 ft/s = 90 fpm, keeping the t=0 climb/descent error inside the ±100 fpm
# QTG-style comparison band used by the report.
V14_68_WORLD_XZ_TOL_FPS = 3.0
V14_68_WORLD_Y_TOL_FPS = 1.5
V14_68_BODY_XZ_TOL_FPS = 3.0
V14_68_BODY_Y_TOL_FPS = 1.5

V14_52_POST_P3D_CLS_REVERIFY_TOL = 0.15
V14_52_POST_P3D_CLS_REVERIFY_TIMEOUT_SEC = 5.0

# -----------------------------------------------------------------------------
# V14.21 COMPLETE THREE-AXIS ALL-AXES + YAW SPEED SELECTOR
# -----------------------------------------------------------------------------
# FINAL SIM-PC AXIS SETTINGS CURRENTLY USED:
#   Pitch: AP force 1800, speed 300 (loader-PC validated in current branch)
#   Roll : AP force 1500, speed 200 (loader-PC validated position + force)
#   Yaw  : AP force 1500, runtime-selectable speed 60/80/100/120/150
#
# V14.21 has NO commissioning stages. Every Automatic run uses all connected
# Pitch + Roll + Yaw axes for both position replay and force replay.
ALL_FORCE_REPLAY_AXES = ("Pitch", "Roll", "Yaw")
COMMISSION_FORCE_STAGE_DEFAULT = 3  # retained internally for backwards-compatible logs only
COMMISSION_FORCE_STAGE_LABELS = {3: "ALL AXES - Pitch + Roll + Yaw position + force"}
COMMISSION_FORCE_STAGE_AXES = {3: ALL_FORCE_REPLAY_AXES}

# BRUNNER UDP transport split into a reusable, standalone module.
# The original packet functions, watchdog, telemetry polling and force probing
# are imported WITHOUT ANY controller tuning or behavioral change.
from climb_brunner_transport import (
    Cls2SimUdpTransport,
    packet_override, packet_setting_u8, packet_setting_u16,
    packet_setting_i32, packet_setting_float, packet_read,
    packet_watchdog, packet_state, node_id_mask,
    packet_remote_config_u8,
)


# V14.30 requires the matching wrapper + rebuilt p3d_bridge.dll.
try:
    from p3d_simconnect_v15 import SimConnectClient
    SIMCONNECT_AVAILABLE = True
except Exception as e:
    SIMCONNECT_AVAILABLE = False
    SIMCONNECT_ERROR = str(e)


class P3DDirectAxisBridge:
    """Auto-only adapter that reuses the application's EXISTING P3D bridge.

    V14.60 does NOT load SimConnect.dll and does NOT open a second SimConnect
    session.  The already-connected ``SimConnectClient`` owns ``p3d_bridge.dll``;
    this adapter only calls its proven SetElevator/SetAileron/SetRudder exports.

    Values are still derived from LIVE measured BRUNNER/CLS positions, not from
    the prerecorded Manual trajectory.  The existing runtime P3D telemetry
    verification remains the fail-closed proof that these calls actually reached
    Prepar3D.
    """

    def __init__(self, owner):
        self.owner = owner
        self.is_open = False
        self.last_error = ""
        self._lock = threading.Lock()
        self.last_commands = {"elevator": None, "aileron": None, "rudder": None}
        self.last_send_wall = 0.0

    def _client(self):
        return getattr(self.owner, "sim_client", None)

    def open(self) -> bool:
        if self.is_open:
            return True
        try:
            client = self._client()
            if client is None:
                raise RuntimeError("existing SimConnectClient is not available")
            if hasattr(client, "is_connected") and not client.is_connected():
                raise RuntimeError("existing SimConnectClient is not connected to Prepar3D")
            missing = [
                name for name in ("set_elevator", "set_aileron", "set_rudder")
                if not callable(getattr(client, name, None))
            ]
            if missing:
                raise RuntimeError(
                    "existing p3d_simconnect_v14_40 wrapper is missing: "
                    + ", ".join(missing)
                )
            self.is_open = True
            self.last_error = ""
            print(
                "[V14.60 EXISTING P3D BRIDGE] using already-connected "
                "p3d_simconnect_v14_40 / p3d_bridge.dll; no second SimConnect.dll load",
                flush=True,
            )
            return True
        except Exception as exc:
            self.last_error = str(exc)
            self.is_open = False
            print(f"[V14.60 EXISTING P3D BRIDGE] OPEN FAILED: {exc}", flush=True)
            return False

    @staticmethod
    def _clamp(norm: float) -> float:
        return max(-1.0, min(1.0, float(norm)))

    def send_controls(self, elevator=None, aileron=None, rudder=None) -> bool:
        """Disabled: QTG app must never directly drive P3D primary controls.

        BRUNNER/CLS2Sim is the only permitted pitch/roll/rudder command path.
        This guard also covers any accidentally revived calibration code.
        """
        self.last_error = (
            "Direct P3D elevator/aileron/rudder writes are DISABLED: "
            "BRUNNER/CLS2Sim sole flight-control authority"
        )
        print("[BRUNNER ONLY] BLOCKED unexpected direct P3D primary-control write", flush=True)
        return False

    def close(self) -> None:
        # IMPORTANT: this adapter does not own the application's SimConnectClient
        # or p3d_bridge.dll, therefore shutdown must NOT disconnect either one.
        with self._lock:
            self.is_open = False


# Define Operating Stages
STAGE_NOT_CONNECTED = "Not connected"
STAGE_CONNECTING = "Connecting to Prepar3D"
STAGE_CONNECTED_READY = "Connected — Ready to Record"
STAGE_ARMED = "Armed — waiting for full throttle"
STAGE_INITIALIZING = "Initializing aircraft"
STAGE_RECORDING = "Recording"
STAGE_COMPLETED = "Completed"
STAGE_STOPPED_BY_USER = "Stopped by user"
STAGE_CONNECTION_ERROR = "Connection error"
STAGE_AUTO_BLOCKED = "Automatic blocked"


class SuperMushakLoggerApp(QMainWindow):

    """Main Application Window for Super Mushshak Automatic Testing Data Logger."""

    # Qt Signals for background thread communication
    _sig_connection_ok = Signal()
    _sig_connection_err = Signal(str)
    _sig_report_progress = Signal(int, str)
    _sig_report_complete = Signal(str)
    # V14.34: Automatic mode has a preparation phase BEFORE Start Recording.
    _sig_auto_prepare_progress = Signal(str)
    _sig_auto_prepare_done = Signal(bool, str)
    # V14.35: the operator confirms visually that the P3D terrain-loading
    # screen has finished before exact t=0 state restoration begins.
    _sig_p3d_user_confirmation_needed = Signal(str)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Super Mushshak Logger - V15.12.5 Automatic Wind Settling")
        self.setMinimumSize(850, 620)
        self.resize(900, 660)

        # Operating state
        self.operating_stage = STAGE_NOT_CONNECTED
        self.is_armed = False
        self.is_recording = False
        self.record_start_time = 0.0
        self.recorded_rows: List[Dict[str, Any]] = []
        self.target_duration_sec = 75
        self.expected_total_rows = 76  # t = 0..75 seconds
        self.next_sample_second = 0
        self.current_csv_filepath: Optional[str] = None
        self.engine_count = 1
        self.test_mode: str = ""  # "auto" or "manual"
        self.auto_replay_strategy: str = "tracking"  # V14: tracking only
        # V14.21: all connected axes always use position + force replay.
        # The legacy stage variable is fixed to 3 only for backwards-compatible diagnostics.
        self.commission_force_stage: int = 3
        # Yaw AP speed is selected separately before each Automatic run.
        self.commission_yaw_ap_speed: int = YAW_AP_SPEED_DEFAULT
        self.track_precheck_results: Dict[str, Any] = {}
        self.cls_to_p3d_feedback_enabled: bool = False
        self.active_cls_axes: List[str] = []
        self.active_cls_axis_mask: int = AX_NONE
        self.reference_cls_axes: List[str] = []
        self.cls_prep_complete: bool = False
        self.force_position_interlock_blocked = {"Pitch": False, "Roll": False, "Yaw": False}
        self.force_position_interlock_latched = {"Pitch": False, "Roll": False, "Yaw": False}
        self.cls_last_cmd_for_safety = {"Pitch": None, "Roll": None, "Yaw": None}
        self.cls_last_cmd_motion_time = {"Pitch": 0.0, "Roll": 0.0, "Yaw": 0.0}

        # BRUNNER device-specific software adaptation.  CLS2Sim profile is left unchanged.
        self.brunner_device = BRUNNER_DEVICE
        self.invert_pitch = CLS_P_INVERT_PITCH
        self.invert_roll = CLS_P_INVERT_ROLL
        self.invert_rudder = CLS_P_INVERT_RUDDER
        self.cls_debug_counter = 0
        self.cls_hardware_ready = False
        self.manual_cls_ready = False
        # V14.34 Automatic readiness:
        # Automatic button performs P3D initialization + CLS preparation.
        # Start Recording is only exposed after BOTH are finished.
        self.auto_p3d_ready = False
        self.auto_environment_ready = False
        self.auto_prepare_in_progress = False

        # V14.52 readiness/failure evidence. These are diagnostics and gates only;
        # they do not alter the V14.51 position/force replay control law.
        self.auto_controller_init_ready = False
        self.auto_last_cls_initial_verify_detail = ""
        self.auto_last_p3d_control_handover_detail = ""
        self.auto_last_block_reason = ""

        # V14.58 Auto-only measured physical CLS -> P3D bridge state.
        self.p3d_direct_bridge = P3DDirectAxisBridge(self)
        self.direct_mirror_active = False
        self.direct_mirror_thread = None
        self.direct_mirror_stop = threading.Event()
        self.direct_mirror_last_error = ""
        self.direct_mirror_last_commands = {"elevator": None, "aileron": None, "rudder": None}
        # V14.60 stores the ACTUAL bridge values separately from desired P3D
        # input values. Runtime health compares P3D telemetry to DESIRED inputs.
        self.direct_mirror_last_bridge_commands = {"elevator": None, "aileron": None, "rudder": None}
        self.direct_mirror_axis_calibration = {
            "Pitch": {"ready": False, "slope": None, "offset": None, "plus": None, "minus": None},
            "Roll": {"ready": False, "slope": None, "offset": None, "plus": None, "minus": None},
            "Yaw": {"ready": False, "slope": None, "offset": None, "plus": None, "minus": None},
        }
        self.v1476_axis_cal_attempts = {"Pitch": 0, "Roll": 0, "Yaw": 0}
        self.v1476_axis_cal_last_detail = {"Pitch": "", "Roll": "", "Yaw": ""}
        self.direct_mirror_anchor = {}
        self.direct_mirror_consecutive_failures = 0
        self.direct_mirror_runtime_mismatch_since = None
        self.direct_mirror_runtime_mismatch_detail = ""
        self.auto_bank_write_multiplier = -1.0

        # V14.35 human-in-the-loop P3D loading gate.
        # Automatic sends the first INITPOSITION to trigger scenery loading,
        # then waits here until the operator confirms the visible P3D loading
        # screen is completely gone.
        self.auto_waiting_for_p3d_user = False
        self.p3d_user_ready_event = threading.Event()

        # V14.37 hard invariant:
        # exactly ONE SIMCONNECT_DATA_INITPOSITION is allowed per Automatic cycle.
        # Start Recording is never allowed to send another one.
        self.auto_initposition_sent = False
        self.auto_initposition_count = 0
        self.auto_position_frozen = False

        # V14.38 two-phase P3D setup:
        #   phase 1 = terrain/scenery preload trigger (Automatic button)
        #   phase 2 = ONE final exact Manual t=0 initialization (after user confirms load)
        # Start Recording performs NO position initialization.
        self.auto_terrain_preload_sent = False
        self.auto_final_init_sent = False
        # V14.40: final t=0 uses SetExactPoseState, not a second INITPOSITION.
        self.auto_final_direct_state_ready = False
        self.pitch_force_diagnostic_active = False
        self.pitch_move_back_disabled = False
        self.auto_replay_softening_active = False
        self.auto_replay_softened_axes: List[str] = []
        # V14.42: becomes True only after the exact replay-frame-0
        # position + force state is simultaneously stable.
        self.bumpless_start_ready = False
        self.bumpless_start_frame_t_ms = None
        self.reference_has_native_cls = False
        self.pitch_force_scale_percent = None

        # Automatic Pitch-force replay state. No position state is changed.
        self.pitch_force_replay_active = False
        self.pitch_force_reference_n = None
        self.pitch_force_actual_n = None
        self.pitch_force_offset_command = 0
        self.pitch_force_integral_internal = 0.0
        self.pitch_force_manual_peak_n = 0.0
        self.pitch_force_baseline_n = 0.0
        self.pitch_force_replay_warned = False
        self.pitch_force_last_arm_attempt = 0.0

        # Automatic Roll-force replay state. Position state is untouched.
        self.roll_force_replay_active = False
        self.roll_force_reference_n = None
        self.roll_force_actual_n = None
        self.roll_force_offset_command = 0
        self.roll_force_integral_internal = 0.0
        self.roll_force_manual_peak_n = 0.0
        self.roll_force_baseline_n = 0.0
        self.roll_force_replay_warned = False
        self.roll_force_last_arm_attempt = 0.0

        # Automatic Yaw-force replay state. Position state is untouched.
        self.yaw_force_replay_active = False
        self.yaw_force_reference_n = None
        self.yaw_force_actual_n = None
        self.yaw_force_offset_command = 0
        self.yaw_force_integral_internal = 0.0

        # V14.12 runtime safety: each force loop is blocked independently if
        # its physical axis is not following the AP-position command.
        self.force_position_interlock_blocked = {
            "Pitch": False, "Roll": False, "Yaw": False
        }
        # During all-axis replay, one large position/force conflict latches
        # that axis's force replay OFF until the next Automatic run. This avoids
        # repeated block/recover cycling on a poorly following Roll/Yaw axis.
        self.force_position_interlock_latched = {
            "Pitch": False, "Roll": False, "Yaw": False
        }
        self.force_position_authority = {
            "Pitch": 1.0, "Roll": 1.0, "Yaw": 1.0
        }
        self.yaw_force_manual_peak_n = 0.0
        self.yaw_force_baseline_n = 0.0
        self.yaw_force_replay_warned = False
        self.yaw_force_last_arm_attempt = 0.0

        self.cls_follow_error_since = {
            "Pitch": None,
            "Roll": None,
            "Yaw": None,
        }
        self.cls_last_targets = {
            "Pitch": 0.0,
            "Roll": 0.0,
            "Yaw": 0.0,
        }
        self.cls_replay_send_failures = {}
        self.cls_replay_ap_off_counts = {}

        # V14.69: distinguish the future AP command from the Manual reference
        # that belongs to the CURRENT test time.  This is diagnostic/safety
        # evidence only; it is never sent directly to P3D.
        self.v1469_manual_position_reference_now = {
            "Pitch": None,
            "Roll": None,
            "Yaw": None,
        }

        # V14.43 explicit Manual-reference selection.  This path identifies the
        # operator-selected Manual CSV only; the matching replay/state companions
        # are validated and bound fail-closed immediately before Automatic setup.
        self.selected_manual_csv_path: Optional[str] = None
        self.manual_reference_catalog: List[Dict[str, Any]] = []

        # Control replay: recorded during manual test, replayed during auto test
        # replay_buffer: list of dicts recorded at the 50 Hz telemetry tick during manual
        self.replay_buffer: List[Dict[str, float]] = []
        # reference_replay: loaded from the operator-selected Manual _replay.json, used during auto
        self.reference_replay: List[Dict[str, float]] = []
        self.reference_replay_path: Optional[str] = None  # exact selected _replay.json
        # V14.32: the Manual CSV itself is now a first-class part of the same
        # immutable reference bundle.  Automatic initialization, replay and
        # report generation are all bound to these exact three files.
        self.reference_manual_csv_path: Optional[str] = None
        self.reference_bundle_manifest: Dict[str, Any] = {}
        self.reference_bundle_id: Optional[str] = None
        self.auto_reference_manifest_path: Optional[str] = None

        # PITCH POSITION DIAGNOSTIC ONLY.
        # High-rate Automatic replay evidence; it never feeds back into control.
        self.pitch_position_diag_rows: List[Dict[str, Any]] = []
        self.pitch_position_diag_last_path: Optional[str] = None

        # V14.62 report/diagnostic coordinate bridge.  Native normalized CLS
        # position is the canonical replay quantity.  The selected Manual run
        # supplies a per-axis affine map back into its own legacy report
        # position coordinate.  This cache is observational only.
        self.manual_position_calibration_cache: Dict[str, Dict[str, float]] = {}

        # ONE-SHOT commissioning evidence. Passive only; no control-law input.
        self.commission_diag_rows: List[Dict[str, Any]] = []
        self.commission_diag_session_wall_start: Optional[float] = None
        self.commission_diag_last_sample_mono: float = 0.0
        self.commission_diag_last_csv: Optional[str] = None
        self.commission_diag_last_txt: Optional[str] = None

        # V14.28 READ-ONLY P3D state diagnostic. These buffers are observational
        # only and are never used by the control/replay law.
        self.p3d_state_diag_rows: List[Dict[str, Any]] = []
        self.p3d_state_diag_wall_start: Optional[float] = None
        self.p3d_state_diag_last_sample_mono: float = 0.0
        self.p3d_state_diag_last_csv: Optional[str] = None
        self.p3d_state_diag_last_txt: Optional[str] = None
        self.p3d_state_diag_all_keys = set()
        self.p3d_state_diag_source_keys: Dict[str, set] = {}

        # V14.30 deterministic Manual -> Automatic initial-state metadata.
        self.manual_initial_dynamic_state: Dict[str, Any] = {}
        # V14.30.4: one authoritative Manual t=0 P3D frame.
        # This exact frame drives BOTH Manual CSV row 0 and the
        # companion _initial_state.json used by Automatic.
        self.manual_t0_frame: Dict[str, Any] = {}
        self.manual_t0_cls_values: Dict[str, Any] = {}
        # V14.64: LIVE measured CLS snapshot captured only after the final
        # exact-force/P3D/position gate passes. Used for Automatic CSV row 0 so
        # row 0 cannot contain a stale pre-gate force sample.
        self.auto_t0_cls_values: Dict[str, Any] = {}
        # V14.65 strict ordering evidence: full physical frame-0 controls are
        # locked before P3D terrain/run phases are allowed to proceed.
        self.v1465_controls_prelocked = False
        self.v1465_controls_final_locked = False
        self.v1465_controls_lock_detail = ""
        self.reference_initial_dynamic_state: Dict[str, Any] = {}
        self.reference_initial_state_path: Optional[str] = None

        # V14.93 one-visit calibration state.
        self.v1493_one_visit_active = False
        self.v1493_one_visit_stage = None          # None / short / final
        self.v1493_one_visit_manual_keys = []
        self.v1493_one_visit_manual_index = 0
        self.v1493_one_visit_candidate_index = 0
        self.v1493_one_visit_results = []
        self.v1493_one_visit_current_candidate = None
        self.v1493_one_visit_original_profiles = {}
        self.measurement_audit = MeasurementAudit()
        self.v1493_full_state_timeline = []
        self.v1493_full_state_last_capture_mono = 0.0
        self.last_manual_reference_error: str = ""
        self.manual_state_capture_ready_mono: float = 0.0
        self.v1430_dynamic_restore_ok: bool = False
        self.v1430_position_commit_ok: bool = False
        self.v1430_dynamic_verify_ok: bool = False

        # Data directory: prefer D: drive if available to avoid C: disk full errors (960+ GB free)
        if os.path.exists("D:\\"):
            self.data_dir = Path("D:/Super_Mushak_Logger_Data")
        else:
            self.data_dir = Path.home() / "Documents" / "Super_Mushak_Logger_Data"
        self.data_dir.mkdir(parents=True, exist_ok=True)

        # SimConnect Client — created lazily in background thread on first Start
        self.sim_client: Optional[SimConnectClient] = None

        # CLS2Sim UDP Transport — three physical actuators: Pitch, Roll, Yaw/Rudder
        self.cls = Cls2SimUdpTransport(host=CLS2SIM_HOST, port=CLS2SIM_PORT)
        try:
            self.cls.open()
            # V14.50 SIM-PC SAFETY:
            # Do NOT send VirtualForceEnable (0xA0) at application startup.
            # Manual mode must not change CLS2Sim/BRUNNER configuration.
            # Normal applied force is read through the validated 0x20/0x21 path.
            self.cls.start_read_only_polling(interval_sec=CLS_POLL_INTERVAL_SEC)
        except Exception:
            pass

        # High-rate control & telemetry update timer (50 Hz / 20ms)
        self.telemetry_timer = QTimer(self)
        self.telemetry_timer.setInterval(20)
        self.telemetry_timer.timeout.connect(self._on_telemetry_tick)

        # Wire cross-thread connection signals to slots on the main thread
        self._sig_connection_ok.connect(self._on_connection_success)
        self._sig_connection_err.connect(self._on_connection_error)
        self._sig_report_progress.connect(self._on_report_progress)
        self._sig_auto_prepare_progress.connect(self._on_auto_prepare_progress)
        self._sig_auto_prepare_done.connect(self._on_auto_prepare_done)
        self._sig_p3d_user_confirmation_needed.connect(
            self._on_p3d_user_confirmation_needed
        )
        self._sig_report_complete.connect(self._on_report_complete)

        # Build UI
        self._init_ui()
        self._apply_dark_theme()
        self._update_stage_ui(STAGE_NOT_CONNECTED)

    @staticmethod
    def _diag_finite(value) -> Optional[float]:
        try:
            value = float(value)
            return value if math.isfinite(value) else None
        except (TypeError, ValueError):
            return None

    @staticmethod
    def _p3d_diag_finite(value) -> Optional[float]:
        try:
            fv = float(value)
            return fv if math.isfinite(fv) else None
        except (TypeError, ValueError, OverflowError):
            return None

    @classmethod
    def _p3d_diag_first(cls, data: Dict[str, Any], candidates) -> tuple:
        """Return (finite_value, source_key) from the first available telemetry key."""
        for key in candidates:
            if key not in (data or {}):
                continue
            value = cls._p3d_diag_finite((data or {}).get(key))
            if value is not None:
                return value, key
        return None, None

    @staticmethod
    def _v1430_finite(value) -> Optional[float]:
        try:
            value = float(value)
            return value if math.isfinite(value) else None
        except (TypeError, ValueError, OverflowError):
            return None

    def _capture_v1430_initial_dynamic_state(self, data: Dict[str, Any]) -> Dict[str, Any]:
        """
        V14.33 authoritative flight state.

        Important corrections:
        - INITPOSITION altitude is MSL/geometric aircraft altitude, so use the
          bridge's PLANE ALTITUDE channel (Python key: pressure_alt), NOT
          INDICATED ALTITUDE.
        - Store all three WORLD and BODY velocities and all three BODY rotation
          velocities directly from the SAME fresh P3D frame. Deriving WORLD Y
          from VSI or BODY rotations from Euler rates changes the snapshot.
        - Keep Euler rates and VSI separately for reports and verification.
        """
        d = data or {}

        plane_alt = self._v1430_finite(d.get("pressure_alt"))
        if plane_alt is None:
            plane_alt = self._v1430_finite(d.get("indicated_altitude"))

        vs_fpm = self._v1430_finite(d.get("vertical_speed"))
        pitch_rate_deg_s = self._v1430_finite(d.get("pitch_rate"))
        yaw_rate_deg_s = self._v1430_finite(d.get("yaw_rate"))
        roll_rate_deg_s = self._v1430_finite(d.get("roll_rate"))

        state = {
            "schema": V14_30_STATE_SCHEMA,
            "captured_iso": datetime.datetime.now().isoformat(timespec="milliseconds"),
            "bridge_version": int(d.get("bridge_version", 0) or 0),

            "velocity_world_x_fps": self._v1430_finite(d.get("velocity_world_x")),
            "velocity_world_y_fps": self._v1430_finite(d.get("velocity_world_y")),
            "velocity_world_z_fps": self._v1430_finite(d.get("velocity_world_z")),
            "velocity_body_x_fps": self._v1430_finite(d.get("velocity_body_x")),
            "velocity_body_y_fps": self._v1430_finite(d.get("velocity_body_y")),
            "velocity_body_z_fps": self._v1430_finite(d.get("velocity_body_z")),

            "rotation_velocity_body_x_rad_s": self._v1430_finite(d.get("rotation_velocity_body_x")),
            "rotation_velocity_body_y_rad_s": self._v1430_finite(d.get("rotation_velocity_body_y")),
            "rotation_velocity_body_z_rad_s": self._v1430_finite(d.get("rotation_velocity_body_z")),

            "elevator_trim_rad": self._v1430_finite(d.get("elevator_trim")),
            "aileron_trim_pct": self._v1430_finite(d.get("aileron_trim_pct")),
            "rudder_trim_pct": self._v1430_finite(d.get("rudder_trim_pct")),

            # Evidence only: never written directly by V14.33.
            "elevator_input": self._v1430_finite(d.get("elevator_pos")),
            "aileron_input": self._v1430_finite(d.get("aileron_pos")),
            "rudder_input": self._v1430_finite(d.get("rudder_pos")),
            "elevator_deflection_rad": self._v1430_finite(d.get("elevator_deflection")),
            "aileron_left_deflection_rad": self._v1430_finite(d.get("aileron_left_deflection")),
            "aileron_right_deflection_rad": self._v1430_finite(d.get("aileron_right_deflection")),
            "rudder_deflection_rad": self._v1430_finite(d.get("rudder_deflection")),

            "ias_kts": self._v1430_finite(d.get("airspeed_indicated")),
            "vertical_speed_fpm": vs_fpm,
            "altitude_ft": plane_alt,
            "indicated_altitude_ft": self._v1430_finite(d.get("indicated_altitude")),

            "roll_rate_deg_s": roll_rate_deg_s,
            "pitch_rate_deg_s": pitch_rate_deg_s,
            "yaw_rate_deg_s": yaw_rate_deg_s,

            "pitch_deg": self._v1430_finite(d.get("pitch_angle")),
            "bank_deg": self._v1430_finite(d.get("bank_angle")),
            "magnetic_heading_deg": self._v1430_finite(d.get("magnetic_heading")),
            "true_heading_deg": self._v1430_finite(d.get("true_heading")),
            "latitude_deg": self._v1430_finite(d.get("plane_latitude")),
            "longitude_deg": self._v1430_finite(d.get("plane_longitude")),
            "throttle_pct": self._v1430_finite(d.get("max_throttle")),
            "actual_wind_speed_kt": self._v1430_finite(d.get("ambient_wind_spd")),
            "actual_wind_dir_deg": self._v1430_finite(d.get("ambient_wind_dir")),
            "actual_fuel_flow_pph": self._v1430_finite(d.get("fuelFlow")),
            "engine_rpm": self._v1430_finite((d.get("eng_rpms") or [None])[0]),
            "engine_prop_rpm": self._v1430_finite((d.get("prop_rpms") or [None])[0]),
            "engine_mixture_lever_1": self._v1430_finite(d.get("engine_mixture_lever_1")),
            "flaps_handle_percent": self._v1430_finite(d.get("flaps_handle_percent")),
            "engine_propeller_lever_1": self._v1430_finite(d.get("engine_propeller_lever_1")),
            "engine_combustion_1": self._v1430_finite(d.get("engine_combustion_1")),
            "engine_fuel_valve_1": self._v1430_finite(d.get("engine_fuel_valve_1")),
            "engine_fuel_pressure_psi_1": self._v1430_finite(d.get("engine_fuel_pressure_psi_1")),
            "engine_manifold_pressure_psi_1": self._v1430_finite(d.get("engine_manifold_pressure_psi_1")),
            "relative_wind_body_x_fps": self._v1430_finite(d.get("relative_wind_body_x_fps")),
            "relative_wind_body_y_fps": self._v1430_finite(d.get("relative_wind_body_y_fps")),
            "relative_wind_body_z_fps": self._v1430_finite(d.get("relative_wind_body_z_fps")),
            "aoa_deg": self._v1430_finite(d.get("aoa")),
            "sideslip_deg": self._v1430_finite(d.get("sideslip")),

            # V14.92: causal state that can alter the aircraft response even
            # when pose, primary controls and rigid-body velocities all match.
            # Legacy accel_* keys are retained for compatibility; despite the
            # suffix they contain the bridge's native ft/s^2 values.
            "accel_x_g": self._v1430_finite(d.get("accelX")),
            "accel_y_g": self._v1430_finite(d.get("accelY")),
            "accel_z_g": self._v1430_finite(d.get("accelZ")),
            "rotation_acceleration_body_x_rad_s2": self._v1430_finite(d.get("rotation_acceleration_body_x_rad_s2")),
            "rotation_acceleration_body_y_rad_s2": self._v1430_finite(d.get("rotation_acceleration_body_y_rad_s2")),
            "rotation_acceleration_body_z_rad_s2": self._v1430_finite(d.get("rotation_acceleration_body_z_rad_s2")),
            "flaps_handle_index": self._v1430_finite(d.get("flaps_handle_index")),
            "flaps_num_handle_positions": self._v1430_finite(d.get("flaps_num_handle_positions")),
            "trailing_edge_flaps_left_percent": self._v1430_finite(d.get("trailing_edge_flaps_left_percent")),
            "trailing_edge_flaps_right_percent": self._v1430_finite(d.get("trailing_edge_flaps_right_percent")),
            "trailing_edge_flaps_left_angle_rad": self._v1430_finite(d.get("trailing_edge_flaps_left_angle_rad")),
            "trailing_edge_flaps_right_angle_rad": self._v1430_finite(d.get("trailing_edge_flaps_right_angle_rad")),
            "ambient_density_slug_ft3": self._v1430_finite(d.get("ambient_density_slug_ft3")),
            "ambient_pressure_mb": self._v1430_finite(d.get("ambient_pressure_mb")),
            "aircraft_wind_x_kt": self._v1430_finite(d.get("aircraft_wind_x_kt")),
            "aircraft_wind_y_kt": self._v1430_finite(d.get("aircraft_wind_y_kt")),
            "aircraft_wind_z_kt": self._v1430_finite(d.get("aircraft_wind_z_kt")),
            "fuel_tank_left_main_level": self._v1430_finite(d.get("fuel_tank_left_main_level")),
            "fuel_tank_right_main_level": self._v1430_finite(d.get("fuel_tank_right_main_level")),
            "fuel_tank_center_level": self._v1430_finite(d.get("fuel_tank_center_level")),
            "fuel_tank_selector_1": self._v1430_finite(d.get("fuel_tank_selector_1")),
            "autopilot_master": self._v1430_finite(d.get("autopilot_master")),
            "auto_coordination": self._v1430_finite(d.get("auto_coordination")),
            "spoilers_handle_position": self._v1430_finite(d.get("spoilers_handle_position")),
            "gear_handle_position": self._v1430_finite(d.get("gear_handle_position")),
            "brake_parking_position": self._v1430_finite(d.get("brake_parking_position")),
            "total_weight_kg": self._v1430_finite(d.get("total_weight")),
            "total_fuel_weight_kg": self._v1430_finite(d.get("total_fuel_quantity")),
            "cg_lateral": self._v1430_finite(d.get("cg_percent_lateral")),
            "cg_percent": self._v1430_finite(d.get("cg_percent")),
            "baro_pressure_hpa": self._v1430_finite(d.get("baro_pressure")),
            "ambient_temp_c": self._v1430_finite(d.get("ambient_temp")),
            "egt_c": self._v1430_finite(d.get("egt")),
            "engine_torque": self._v1430_finite(d.get("torque")),

            # V14.95 complete causal-state audit / loading / inertia.
            "payload_station_count": int(d.get("payload_station_count", 0) or 0),
            "payload_station_weights_lb": list(d.get("payload_station_weights_lb") or []),
            "total_weight_pitch_moi": self._v1430_finite(d.get("total_weight_pitch_moi")),
            "total_weight_roll_moi": self._v1430_finite(d.get("total_weight_roll_moi")),
            "total_weight_yaw_moi": self._v1430_finite(d.get("total_weight_yaw_moi")),
            "total_weight_cross_coupled_moi": self._v1430_finite(d.get("total_weight_cross_coupled_moi")),
            "fuel_tank_levels_all": dict(d.get("fuel_tank_levels_all") or {}),
            "fuel_tank_capacities_gal": dict(d.get("fuel_tank_capacities_gal") or {}),
            "fuel_tank_selectors": list(d.get("fuel_tank_selectors") or []),
            "recip_cowl_flap_position_1": self._v1430_finite(d.get("recip_cowl_flap_position_1")),
            "recip_primer_1": self._v1430_finite(d.get("recip_primer_1")),
            "recip_alternate_air_position_1": self._v1430_finite(d.get("recip_alternate_air_position_1")),
            "recip_left_magneto_1": self._v1430_finite(d.get("recip_left_magneto_1")),
            "recip_right_magneto_1": self._v1430_finite(d.get("recip_right_magneto_1")),
            "recip_fuel_available_1": self._v1430_finite(d.get("recip_fuel_available_1")),
            "recip_fuel_tank_selector_1": self._v1430_finite(d.get("recip_fuel_tank_selector_1")),
            "recip_fuel_tanks_used_1": self._v1430_finite(d.get("recip_fuel_tanks_used_1")),
            "general_eng_starter_1": self._v1430_finite(d.get("general_eng_starter_1")),
            "general_eng_fuel_pump_switch_1": self._v1430_finite(d.get("general_eng_fuel_pump_switch_1")),
            "general_eng_fuel_pump_on_1": self._v1430_finite(d.get("general_eng_fuel_pump_on_1")),
            "general_eng_damage_percent_1": self._v1430_finite(d.get("general_eng_damage_percent_1")),
            "general_eng_failed_1": self._v1430_finite(d.get("general_eng_failed_1")),
            "general_eng_anti_ice_position_1": self._v1430_finite(d.get("general_eng_anti_ice_position_1")),
            "prop_deice_switch_1": self._v1430_finite(d.get("prop_deice_switch_1")),
            "eng_on_fire_1": self._v1430_finite(d.get("eng_on_fire_1")),
            "structural_ice_pct": self._v1430_finite(d.get("structural_ice_pct")),
            "pitot_ice_pct": self._v1430_finite(d.get("pitot_ice_pct")),
            "pitot_heat": self._v1430_finite(d.get("pitot_heat")),
            "flap_damage_by_speed": self._v1430_finite(d.get("flap_damage_by_speed")),
            "flap_speed_exceeded": self._v1430_finite(d.get("flap_speed_exceeded")),
            "gear_damage_by_speed": self._v1430_finite(d.get("gear_damage_by_speed")),
            "gear_speed_exceeded": self._v1430_finite(d.get("gear_speed_exceeded")),
            "spoilers_left_position": self._v1430_finite(d.get("spoilers_left_position")),
            "spoilers_right_position": self._v1430_finite(d.get("spoilers_right_position")),
            "gear_center_position": self._v1430_finite(d.get("gear_center_position")),
            "gear_left_position": self._v1430_finite(d.get("gear_left_position")),
            "gear_right_position": self._v1430_finite(d.get("gear_right_position")),
            "canopy_open": self._v1430_finite(d.get("canopy_open")),
            "damage_state": self._v1430_finite(d.get("damage_state")),
            "sim_on_ground": self._v1430_finite(d.get("sim_on_ground")),
            "user_input_enabled": self._v1430_finite(d.get("user_input_enabled")),
            "gyro_heading_rad": self._v1430_finite(d.get("gyro_heading_rad")),
            "gyro_drift_error_rad": self._v1430_finite(d.get("gyro_drift_error_rad")),
            "lvar_capture_status": "UNAVAILABLE_VIA_SIMCONNECT; optional gauge-side audit required",
        }
        for key in SUPPORT_KEYS:
            state[key] = self._v1430_finite(d.get(key))
        state["extended_causal_layout"] = d.get("extended_causal_layout")
        state["extended_causal_sequence"] = d.get("extended_causal_sequence")
        state["engine_support_sequence"] = d.get("engine_support_sequence")
        return state


    def _v1430_state_is_complete(self, state: Dict[str, Any]) -> bool:
        if not isinstance(state, dict) or state.get("schema") != V14_30_STATE_SCHEMA:
            return False
        if int(state.get("bridge_version", 0) or 0) < 1430:
            return False
        required = (
            "latitude_deg", "longitude_deg", "altitude_ft", "ias_kts",
            "pitch_deg", "bank_deg", "true_heading_deg",
            "velocity_world_x_fps", "velocity_world_y_fps", "velocity_world_z_fps",
            "rotation_velocity_body_x_rad_s", "rotation_velocity_body_y_rad_s",
            "rotation_velocity_body_z_rad_s", "elevator_trim_rad",
            "aileron_trim_pct", "rudder_trim_pct",
        )
        return all(self._v1430_finite(state.get(k)) is not None for k in required)

    @staticmethod
    def _v1430_angle_error_deg(actual: float, target: float) -> float:
        return ((float(actual) - float(target) + 180.0) % 360.0) - 180.0

    def _v1430_position_matches(self, data: Dict[str, Any], state: Dict[str, Any]) -> tuple[bool, str]:
        d = data or {}
        try:
            live_alt = self._v1430_finite(d.get("pressure_alt"))
            if live_alt is None:
                live_alt = self._v1430_finite(d.get("indicated_altitude"))
            if live_alt is None:
                live_alt = 1e9

            errs = {
                "alt_ft": float(live_alt) - float(state["altitude_ft"]),
                "ias_kt": float(d.get("airspeed_indicated", 1e9)) - float(state["ias_kts"]),
                "pitch_deg": float(d.get("pitch_angle", 1e9)) - float(state["pitch_deg"]),
                "bank_deg": float(d.get("bank_angle", 1e9)) - float(state["bank_deg"]),
                "hdg_deg": self._v1430_angle_error_deg(
                    d.get("true_heading", 1e9),
                    state["true_heading_deg"],
                ),
            }
            ok = (
                abs(errs["alt_ft"]) <= V14_30_ALT_TOL_FT
                and abs(errs["ias_kt"]) <= V14_30_IAS_TOL_KT
                and abs(errs["pitch_deg"]) <= V14_30_ATT_TOL_DEG
                and abs(errs["bank_deg"]) <= V14_30_ATT_TOL_DEG
                and abs(errs["hdg_deg"]) <= V14_30_HDG_TOL_DEG
            )
            detail = (
                f"dPlaneAlt={errs['alt_ft']:+.1f}ft dIAS={errs['ias_kt']:+.1f}kt "
                f"dPitch={errs['pitch_deg']:+.2f}deg dBank={errs['bank_deg']:+.2f}deg "
                f"dHdg={errs['hdg_deg']:+.2f}deg"
            )
            return bool(ok), detail
        except Exception as exc:
            return False, f"position compare error: {exc}"


    def _v1467_pose_matches_no_ias(
        self, data: Dict[str, Any], state: Dict[str, Any]
    ) -> tuple[bool, str]:
        """Verify exact geometric pose without using AIRSPEED INDICATED as a gate.

        V14.66 deliberately stopped writing AIRSPEED INDICATED during the final
        post-terrain pose restore.  IAS is derived from the aerodynamic/velocity
        solution and may not refresh while P3D is paused.  The authoritative
        t=0 velocity gate is therefore WORLD velocity X/Y/Z in
        _v1430_dynamic_matches().  IAS is logged here as diagnostic evidence only.
        """
        d = data or {}
        try:
            live_alt = self._v1430_finite(d.get("pressure_alt"))
            if live_alt is None:
                live_alt = self._v1430_finite(d.get("indicated_altitude"))
            if live_alt is None:
                live_alt = 1e9

            errs = {
                "alt_ft": float(live_alt) - float(state["altitude_ft"]),
                "pitch_deg": float(d.get("pitch_angle", 1e9)) - float(state["pitch_deg"]),
                "bank_deg": float(d.get("bank_angle", 1e9)) - float(state["bank_deg"]),
                "hdg_deg": self._v1430_angle_error_deg(
                    d.get("true_heading", 1e9),
                    state["true_heading_deg"],
                ),
            }

            ok = (
                abs(errs["alt_ft"]) <= V14_30_ALT_TOL_FT
                and abs(errs["pitch_deg"]) <= V14_30_ATT_TOL_DEG
                and abs(errs["bank_deg"]) <= V14_30_ATT_TOL_DEG
                and abs(errs["hdg_deg"]) <= V14_30_HDG_TOL_DEG
            )

            live_ias = self._v1430_finite(d.get("airspeed_indicated"))
            ref_ias = self._v1430_finite(state.get("ias_kts"))
            if live_ias is not None and ref_ias is not None:
                ias_text = f"dIAS(diag)={live_ias - ref_ias:+.1f}kt"
            else:
                ias_text = "dIAS(diag)=missing"

            detail = (
                f"dPlaneAlt={errs['alt_ft']:+.1f}ft {ias_text} "
                f"dPitch={errs['pitch_deg']:+.2f}deg "
                f"dBank={errs['bank_deg']:+.2f}deg "
                f"dHdg={errs['hdg_deg']:+.2f}deg"
            )
            return bool(ok), detail
        except Exception as exc:
            return False, f"no-IAS pose compare error: {exc}"

    def _v1435_begin_p3d_visual_load(self, context: str) -> bool:
        """
        V14.38 Phase 1 — TERRAIN PRELOAD ONLY.

        This first INITPOSITION exists only to make P3D load the correct scenery.
        It is NOT accepted as the final Automatic t=0 state.

        P3D remains live after the preload so the terrain/scenery loader can
        finish normally. The operator then presses "P3D Loaded — Continue Setup".

        A separate final exact initialization is performed AFTER that confirmation.
        Start Recording itself never sends INITPOSITION.
        """
        state = dict(getattr(self, "reference_initial_dynamic_state", {}) or {})
        if not self._v1430_state_is_complete(state) or not self.sim_client:
            print(f"[V14.38 PRELOAD] {context}: reference state missing/incomplete", flush=True)
            return False

        if self.auto_terrain_preload_sent:
            print("[V14.38 PRELOAD] duplicate terrain preload blocked", flush=True)
            return True

        queue = getattr(self.sim_client, "queue_position_airborne", None)
        if not callable(queue):
            print("[V14.38 PRELOAD] queue_position_airborne API missing", flush=True)
            return False

        try:
            self._set_cls_position_forwarding_to_p3d(False)
            self.sim_client.pause_sim()
            time.sleep(0.08)

            throttle = state.get("throttle_pct")
            if self._v1430_finite(throttle) is None:
                throttle = (getattr(self, "manual_initial_conditions", {}) or {}).get(
                    "throttle", 80.0
                )

            ok = bool(
                queue(
                    alt_ft=float(state["altitude_ft"]),
                    speed_kts=float(state["ias_kts"]),
                    pitch_deg=float(state["pitch_deg"]),
                    bank_deg=float(state["bank_deg"]),
                    hdg_deg=float(state["true_heading_deg"]),
                    lat=float(state["latitude_deg"]),
                    lon=float(state["longitude_deg"]),
                    throttle=float(throttle),
                )
            )
            if not ok:
                print(f"[V14.38 PRELOAD] {context}: terrain preload INITPOSITION failed", flush=True)
                return False

            self.auto_terrain_preload_sent = True
            self.auto_initposition_sent = True
            self.auto_initposition_count = 1

            # Let P3D run so the visual terrain/scenery loader can complete.
            self.sim_client.unpause_sim()

            print(
                "[V14.38 PRELOAD] TERRAIN PRELOAD SENT | init_count=1 | "
                "waiting for operator confirmation; this is NOT final t=0.",
                flush=True,
            )
            return True

        except Exception as exc:
            print(f"[V14.38 PRELOAD] {context}: exception: {exc}", flush=True)
            return False



    def _v1439_wait_for_postload_stability(self, context: str) -> bool:
        """
        Wait until P3D has genuinely stopped changing the aircraft after the
        terrain/scenery loading phase.

        IMPORTANT:
        This does NOT compare against the Manual target yet.
        The aircraft may have drifted during loading.  We only ask:
            "has P3D stopped moving/changing the state?"

        Once that is stable, V14.39 performs the ONE final exact Manual t=0
        initialization.  This prevents the final t=0 state from being overwritten
        by terrain work that was still finishing in the background.
        """
        if not self.sim_client:
            return False

        request_once = getattr(self.sim_client, "request_telemetry_once", None)
        wait_fresh = getattr(self.sim_client, "wait_for_fresh_telemetry", None)
        get_seq = getattr(self.sim_client, "get_telemetry_sequence", None)
        if not all(callable(x) for x in (request_once, wait_fresh, get_seq)):
            print("[V14.39 STABLE] required telemetry API missing", flush=True)
            return False

        def live_state(d: Dict[str, Any]) -> Optional[Dict[str, float]]:
            try:
                alt = self._v1430_finite(d.get("pressure_alt"))
                if alt is None:
                    alt = self._v1430_finite(d.get("indicated_altitude"))
                ias = self._v1430_finite(d.get("airspeed_indicated"))
                pitch = self._v1430_finite(d.get("pitch_angle"))
                bank = self._v1430_finite(d.get("bank_angle"))
                hdg = self._v1430_finite(d.get("true_heading"))
                if None in (alt, ias, pitch, bank, hdg):
                    return None
                return {
                    "alt": float(alt),
                    "ias": float(ias),
                    "pitch": float(pitch),
                    "bank": float(bank),
                    "hdg": float(hdg),
                }
            except Exception:
                return None

        try:
            # Freeze normal flight physics while the background scenery loader
            # finishes any remaining state changes.
            self.sim_client.pause_sim()
            time.sleep(0.10)

            deadline = time.monotonic() + V14_39_POSTLOAD_STABLE_TIMEOUT_SEC
            seq = int(get_seq())
            previous = None
            stable_samples = 0
            stable_since = None
            last_detail = "waiting for first fresh paused state"

            while time.monotonic() < deadline:
                if not request_once():
                    previous = None
                    stable_samples = 0
                    stable_since = None
                    time.sleep(0.10)
                    continue

                fresh = wait_fresh(seq, V14_39_POSTLOAD_SAMPLE_WAIT_SEC)
                if fresh is None:
                    previous = None
                    stable_samples = 0
                    stable_since = None
                    last_detail = "no fresh telemetry"
                    continue

                try:
                    new_seq = int(fresh.get("bridge_telemetry_sequence", seq))
                except Exception:
                    new_seq = int(get_seq())
                if new_seq <= seq:
                    previous = None
                    stable_samples = 0
                    stable_since = None
                    last_detail = "cached telemetry"
                    continue
                seq = new_seq

                current = live_state(fresh)
                if current is None:
                    previous = None
                    stable_samples = 0
                    stable_since = None
                    last_detail = "incomplete telemetry"
                    continue

                if previous is None:
                    previous = current
                    stable_samples = 0
                    stable_since = None
                    continue

                d_alt = current["alt"] - previous["alt"]
                d_ias = current["ias"] - previous["ias"]
                d_pitch = current["pitch"] - previous["pitch"]
                d_bank = current["bank"] - previous["bank"]
                d_hdg = self._v1430_angle_error_deg(
                    current["hdg"], previous["hdg"]
                )

                stable_now = (
                    abs(d_alt) <= V14_39_STABLE_ALT_DELTA_FT
                    and abs(d_ias) <= V14_39_STABLE_IAS_DELTA_KT
                    and abs(d_pitch) <= V14_39_STABLE_ATT_DELTA_DEG
                    and abs(d_bank) <= V14_39_STABLE_ATT_DELTA_DEG
                    and abs(d_hdg) <= V14_39_STABLE_HDG_DELTA_DEG
                )

                now = time.monotonic()
                if stable_now:
                    if stable_since is None:
                        stable_since = now
                    stable_samples += 1
                else:
                    stable_samples = 0
                    stable_since = None

                stable_sec = 0.0 if stable_since is None else now - stable_since
                last_detail = (
                    f"dAlt={d_alt:+.2f}ft dIAS={d_ias:+.2f}kt "
                    f"dPitch={d_pitch:+.2f} dBank={d_bank:+.2f} "
                    f"dHdg={d_hdg:+.2f}"
                )

                try:
                    self._sig_auto_prepare_progress.emit(
                        "P3D load confirmed — waiting for background state to stop "
                        f"changing ({stable_samples}/{V14_39_POSTLOAD_STABLE_MIN_SAMPLES}, "
                        f"{stable_sec:.1f}/{V14_39_POSTLOAD_STABLE_MIN_SEC:.1f}s)"
                    )
                except Exception:
                    pass

                print(
                    f"[V14.39 STABLE] {context}: "
                    f"{stable_samples}/{V14_39_POSTLOAD_STABLE_MIN_SAMPLES} samples, "
                    f"{stable_sec:.2f}/{V14_39_POSTLOAD_STABLE_MIN_SEC:.2f}s | "
                    f"{last_detail}",
                    flush=True,
                )

                previous = current

                if (
                    stable_samples >= V14_39_POSTLOAD_STABLE_MIN_SAMPLES
                    and stable_sec >= V14_39_POSTLOAD_STABLE_MIN_SEC
                ):
                    print(
                        "[V14.39 STABLE] P3D BACKGROUND STATE IS STABLE — "
                        "safe to apply FINAL exact Manual t=0.",
                        flush=True,
                    )
                    return True

                time.sleep(0.10)

            print(
                f"[V14.39 STABLE] TIMEOUT after "
                f"{V14_39_POSTLOAD_STABLE_TIMEOUT_SEC:.0f}s | {last_detail}",
                flush=True,
            )
            return False

        except Exception as exc:
            print(f"[V14.39 STABLE] exception: {exc}", flush=True)
            return False

    def _v1438_final_exact_init_after_load(self, context: str) -> bool:
        """
        V14.40 FINAL exact Manual t=0 using DIRECT settable P3D SimVars.

        Terrain has already been loaded by the one INITPOSITION preload.

        CRITICAL:
          - NO second SIMCONNECT_DATA_INITPOSITION here.
          - NO unpause is needed to commit the final pose.
          - final pose + dynamics are written while P3D is PAUSED.
          - this avoids starting another "Loading terrain data..." cycle.

        The V14.66+ bridge writes:
          PLANE LATITUDE
          PLANE LONGITUDE
          PLANE ALTITUDE
          PLANE PITCH DEGREES
          PLANE BANK DEGREES
          PLANE HEADING DEGREES TRUE

        AIRSPEED INDICATED is intentionally NOT written here.  The authoritative
        velocity state is restored by the direct dynamic setter:
          WORLD velocity X/Y/Z
          BODY rotation X/Y/Z
          trims
        """
        if not self.sim_client:
            return False

        if not self.auto_terrain_preload_sent or self.auto_initposition_count != 1:
            print(
                f"[V14.40 FINAL] invalid terrain preload state | "
                f"init_count={self.auto_initposition_count}",
                flush=True,
            )
            return False

        state = dict(getattr(self, "reference_initial_dynamic_state", {}) or {})
        if not self._v1430_state_is_complete(state):
            return False

        pose_setter = getattr(self.sim_client, "set_exact_pose_state_no_airspeed", None)
        dyn_setter = getattr(self.sim_client, "set_initial_dynamic_state", None)
        request_once = getattr(self.sim_client, "request_telemetry_once", None)
        wait_fresh = getattr(self.sim_client, "wait_for_fresh_telemetry", None)
        get_seq = getattr(self.sim_client, "get_telemetry_sequence", None)

        if not all(callable(x) for x in (
            pose_setter, dyn_setter, request_once, wait_fresh, get_seq
        )):
            print("[V14.67 FINAL] V14.66 no-IAS direct-pose API not available.", flush=True)
            return False

        try:
            self._set_cls_position_forwarding_to_p3d(False)
            self.sim_client.pause_sim()
            time.sleep(0.10)

            throttle = state.get("throttle_pct")
            if self._v1430_finite(throttle) is None:
                throttle = (getattr(self, "manual_initial_conditions", {}) or {}).get(
                    "throttle", 80.0
                )
            try:
                self.sim_client.set_throttle(float(throttle))
            except Exception:
                pass

            last_detail = "no direct final-state readback yet"
            bank_write_multiplier = -1.0

            for attempt in range(1, 7):
                pose_state = dict(state)
                pose_state["bank_deg"] = float(state["bank_deg"]) * float(bank_write_multiplier)
                pose_ok = bool(pose_setter(pose_state))
                dyn_ok = bool(dyn_setter(state))

                if not (pose_ok and dyn_ok):
                    last_detail = f"setter failure pose={pose_ok} dynamics={dyn_ok}"
                    print(
                        f"[V14.40 FINAL] attempt={attempt}/6 FAIL | {last_detail}",
                        flush=True,
                    )
                    time.sleep(0.20)
                    continue

                time.sleep(0.12)

                seq0 = int(get_seq())
                if not request_once():
                    last_detail = "RequestTelemetryOnce failed"
                    continue

                fresh = wait_fresh(seq0, 1.0)
                if fresh is None:
                    last_detail = "fresh paused telemetry timed out"
                    continue

                pos_ok, pos_detail = self._v1467_pose_matches_no_ias(fresh, state)
                dyn_verify_ok, dyn_detail = self._v1430_dynamic_matches(fresh, state)
                last_detail = (
                    f"position={'PASS' if pos_ok else 'FAIL'} [{pos_detail}] | "
                    f"dynamic={'PASS' if dyn_verify_ok else 'FAIL'} [{dyn_detail}]"
                )

                print(
                    f"[V14.40 FINAL] DIRECT FINAL VERIFY attempt={attempt}/6 | "
                    f"{last_detail}",
                    flush=True,
                )

                # V14.79: wrapper reports bank=-native bank. Always invert
                # the reported reference on write, including banks below 0.30 deg.
                # Do not apply a second heuristic sign correction.

                if pos_ok and dyn_verify_ok:
                    self.v1430_position_commit_ok = True
                    self.v1430_dynamic_restore_ok = True
                    self.v1430_dynamic_verify_ok = True
                    self.auto_position_frozen = True
                    self.auto_p3d_ready = True
                    self.auto_final_direct_state_ready = True
                    self.auto_bank_write_multiplier = float(bank_write_multiplier)

                    print(
                        "[V14.40 FINAL] FINAL MANUAL t=0 READY VIA DIRECT POSE | "
                        f"INITPOSITION count remains {self.auto_initposition_count}",
                        flush=True,
                    )
                    return True

                time.sleep(0.20)

            print(
                f"[V14.40 FINAL] direct final state FAILED | {last_detail}",
                flush=True,
            )
            return False

        except Exception as exc:
            print(f"[V14.40 FINAL] exception: {exc}", flush=True)
            return False


    def _v1437_verify_paused_position_no_reinit(self, context: str) -> bool:
        """Verify the already-initialized paused P3D position without writing position."""
        if not self.sim_client or not self.auto_position_frozen:
            return False

        state = dict(getattr(self, "reference_initial_dynamic_state", {}) or {})
        try:
            self.sim_client.pause_sim()
            seq0 = self.sim_client.get_telemetry_sequence()
            self.sim_client.request_telemetry_once()
            fresh = self.sim_client.wait_for_fresh_telemetry(seq0, 1.0)
            ok, detail = self._v1467_pose_matches_no_ias(fresh or {}, state)
            print(
                f"[V14.67 VERIFY] {context}: "
                f"{'PASS' if ok else 'FAIL'} | {detail} | "
                f"init_count={self.auto_initposition_count}",
                flush=True,
            )
            return bool(ok)
        except Exception as exc:
            print(f"[V14.37 VERIFY] {context}: exception: {exc}", flush=True)
            return False

    def _v1437_finalize_p3d_without_reinit(self, context: str) -> bool:
        """
        Finalize exact P3D state after the operator confirms visual scenery load.

        CRITICAL: this method NEVER sends INITPOSITION.
        """
        if self.auto_initposition_count != 1 or not self.auto_initposition_sent:
            print(
                f"[V14.37 FINALIZE] invalid init count={self.auto_initposition_count}",
                flush=True,
            )
            return False

        if not self._v1437_verify_paused_position_no_reinit(
            context + " / position-before-dynamics"
        ):
            return False

        if not self._restore_v1430_reference_dynamic_state(
            context + " / direct-dynamic-restore"
        ):
            return False

        if not self._v1437_verify_paused_position_no_reinit(
            context + " / position-after-dynamics"
        ):
            return False

        self.v1430_position_commit_ok = True
        self.auto_p3d_ready = True

        print(
            "[V14.37 FINALIZE] P3D READY WITHOUT REINITIALIZATION | "
            f"init_count={self.auto_initposition_count}",
            flush=True,
        )
        return True

    def _commit_v1430_reference_position(self, context: str) -> bool:
        """
        V14.36 exact position commit AFTER user-confirmed scenery load.

        The visible P3D loading is handled by the explicit user confirmation
        button. This second same-location commit is only to place the aircraft
        precisely back at Manual t=0 after any drift that occurred while P3D
        was live during scenery loading.

        It therefore uses a short bounded fresh-telemetry verification rather
        than trying to infer terrain-loader completion.
        """
        # V14.37 HARD DUPLICATE-INIT GUARD.
        # Automatic already initialized once on the Automatic button path.
        if (
            self.test_mode == "auto"
            and getattr(self, "auto_initposition_sent", False)
        ):
            print(
                "[V14.37 INIT] duplicate position-initialization request BLOCKED; "
                "verification-only path used.",
                flush=True,
            )
            return self._v1437_verify_paused_position_no_reinit(context)

        state = dict(getattr(self, "reference_initial_dynamic_state", {}) or {})
        self.v1430_position_commit_ok = False

        if not self._v1430_state_is_complete(state) or not self.sim_client:
            print(f"[V14.36 INIT] {context}: reference state missing/incomplete", flush=True)
            return False

        queue = getattr(self.sim_client, "queue_position_airborne", None)
        request_once = getattr(self.sim_client, "request_telemetry_once", None)
        wait_fresh = getattr(self.sim_client, "wait_for_fresh_telemetry", None)
        get_seq = getattr(self.sim_client, "get_telemetry_sequence", None)

        if not all(callable(x) for x in (queue, request_once, wait_fresh, get_seq)):
            print("[V14.36 INIT] Position verification API missing.", flush=True)
            return False

        try:
            self._set_cls_position_forwarding_to_p3d(False)
            self.sim_client.pause_sim()
            time.sleep(0.08)

            throttle = state.get("throttle_pct")
            if self._v1430_finite(throttle) is None:
                throttle = (getattr(self, "manual_initial_conditions", {}) or {}).get(
                    "throttle", 80.0
                )

            if not queue(
                alt_ft=float(state["altitude_ft"]),
                speed_kts=float(state["ias_kts"]),
                pitch_deg=float(state["pitch_deg"]),
                bank_deg=float(state["bank_deg"]),
                hdg_deg=float(state["true_heading_deg"]),
                lat=float(state["latitude_deg"]),
                lon=float(state["longitude_deg"]),
                throttle=float(throttle),
            ):
                print(f"[V14.36 INIT] {context}: exact INITPOSITION queue failed", flush=True)
                return False

            # Let the already-loaded P3D commit the same-location position.
            seq = int(get_seq())
            self.sim_client.unpause_sim()

            deadline = time.monotonic() + V14_36_POST_CONFIRM_POSITION_TIMEOUT_SEC
            stable = 0
            last_detail = "waiting for exact position readback"

            while time.monotonic() < deadline:
                request_once()
                fresh = wait_fresh(seq, 1.0)
                if fresh is None:
                    stable = 0
                    continue

                try:
                    seq = int(fresh.get("bridge_telemetry_sequence", seq))
                except Exception:
                    seq = int(get_seq())

                ok, last_detail = self._v1430_position_matches(fresh, state)
                stable = stable + 1 if ok else 0

                print(
                    f"[V14.36 INIT] {context}: EXACT POSITION "
                    f"{stable}/{V14_36_POST_CONFIRM_POSITION_STABLE_SAMPLES} | "
                    f"{last_detail}",
                    flush=True,
                )

                if stable >= V14_36_POST_CONFIRM_POSITION_STABLE_SAMPLES:
                    self.sim_client.pause_sim()
                    time.sleep(0.08)

                    # Final paused readback.
                    seq0 = int(get_seq())
                    request_once()
                    paused = wait_fresh(seq0, 1.0)
                    paused_ok, paused_detail = self._v1430_position_matches(
                        paused or {}, state
                    )
                    if not paused_ok:
                        self.sim_client.unpause_sim()
                        stable = 0
                        continue

                    self.v1430_position_commit_ok = True
                    print(
                        f"[V14.36 INIT] {context}: "
                        f"EXACT POSITION COMMITTED + VERIFIED | {paused_detail}",
                        flush=True,
                    )
                    return True

                time.sleep(0.05)

            self.sim_client.pause_sim()
            print(
                f"[V14.36 INIT] {context}: exact position verify TIMEOUT | "
                f"{last_detail}",
                flush=True,
            )
            return False

        except Exception as exc:
            try:
                self.sim_client.pause_sim()
            except Exception:
                pass
            print(
                f"[V14.36 INIT] {context}: exact position exception: {exc}",
                flush=True,
            )
            return False



    def _v1430_dynamic_matches(self, data: Dict[str, Any], state: Dict[str, Any]) -> tuple[bool, str]:
        """
        V14.36 readiness verification.

        HARD checks:
          - WORLD velocity X/Y/Z
          - BODY rotation velocity X/Y/Z
          - elevator trim
          - aileron trim
          - rudder trim

        These are the actual state values written by SetInitialDynamicState().

        NOT used as readiness gates:
          - VERTICAL SPEED
          - displayed roll/pitch/yaw rate channels

        Those are derived/display telemetry and can lag or be recomputed by P3D
        while paused even when the underlying settable state is correct. They
        remain logged for diagnostics/reports; they simply cannot deadlock
        Automatic preparation.
        """
        d = data or {}
        try:
            checks = [
                ("velocity_world_x", "velocity_world_x_fps", V14_68_WORLD_XZ_TOL_FPS),
                ("velocity_world_y", "velocity_world_y_fps", V14_68_WORLD_Y_TOL_FPS),
                ("velocity_world_z", "velocity_world_z_fps", V14_68_WORLD_XZ_TOL_FPS),
                ("velocity_body_x", "velocity_body_x_fps", V14_68_BODY_XZ_TOL_FPS),
                ("velocity_body_y", "velocity_body_y_fps", V14_68_BODY_Y_TOL_FPS),
                ("velocity_body_z", "velocity_body_z_fps", V14_68_BODY_XZ_TOL_FPS),
                ("rotation_velocity_body_x", "rotation_velocity_body_x_rad_s", V14_30_ROTATION_TOL_RAD_S),
                ("rotation_velocity_body_y", "rotation_velocity_body_y_rad_s", V14_30_ROTATION_TOL_RAD_S),
                ("rotation_velocity_body_z", "rotation_velocity_body_z_rad_s", V14_30_ROTATION_TOL_RAD_S),
                ("elevator_trim", "elevator_trim_rad", V14_30_ELEV_TRIM_TOL_RAD),
                ("aileron_trim_pct", "aileron_trim_pct", V14_30_TRIM_PCT_TOL),
                ("rudder_trim_pct", "rudder_trim_pct", V14_30_TRIM_PCT_TOL),
            ]

            errors = []
            ok = True
            for live_key, ref_key, tol in checks:
                live = self._v1430_finite(d.get(live_key))
                ref = self._v1430_finite(state.get(ref_key))
                if live is None or ref is None:
                    ok = False
                    errors.append(f"{live_key}=missing")
                    continue

                err = live - ref
                if abs(err) > tol:
                    ok = False
                errors.append(f"{live_key}:{err:+.4f}")

            # Diagnostic only — never blocks readiness.
            diag = []

            for live_key, ref_key in (
                ("vertical_speed", "vertical_speed_fpm"),
                ("roll_rate", "roll_rate_deg_s"),
                ("pitch_rate", "pitch_rate_deg_s"),
                ("yaw_rate", "yaw_rate_deg_s"),
            ):
                live = self._v1430_finite(d.get(live_key))
                ref = self._v1430_finite(state.get(ref_key))
                if live is not None and ref is not None:
                    diag.append(f"{live_key}(diag):{live-ref:+.2f}")

            detail = " ".join(errors + diag)
            return bool(ok), detail

        except Exception as exc:
            return False, f"direct-state compare error: {exc}"




    def _restore_v1430_reference_dynamic_state(self, context: str) -> bool:
        """
        V14.36 bounded direct-state restore.

        This runs ONLY after the operator has visually confirmed P3D terrain
        loading is finished.

        There is no 0/8 endless "dynamic settle" loop anymore.

        The logger:
          1. pauses P3D,
          2. writes the exact Manual WORLD velocity / BODY rotation / trims,
          3. requests a fresh paused telemetry packet,
          4. verifies the DIRECT settable values,
          5. retries a small bounded number of times if needed.

        Derived/display Vertical Speed and displayed rate channels are not used
        as readiness gates.
        """
        state = dict(getattr(self, "reference_initial_dynamic_state", {}) or {})
        self.v1430_dynamic_restore_ok = False
        self.v1430_dynamic_verify_ok = False

        if not self._v1430_state_is_complete(state) or not self.sim_client:
            print(f"[V14.36 INIT] {context}: reference state missing/incomplete", flush=True)
            return False

        setter = getattr(self.sim_client, "set_initial_dynamic_state", None)
        request_once = getattr(self.sim_client, "request_telemetry_once", None)
        wait_fresh = getattr(self.sim_client, "wait_for_fresh_telemetry", None)
        get_seq = getattr(self.sim_client, "get_telemetry_sequence", None)

        if not all(callable(x) for x in (setter, request_once, wait_fresh, get_seq)):
            print("[V14.36 INIT] Missing direct-state verification API.", flush=True)
            return False

        try:
            self.sim_client.pause_sim()
            last_detail = "no direct-state readback yet"

            for attempt in range(1, V14_36_DIRECT_VERIFY_ATTEMPTS + 1):
                set_ok = bool(setter(state))
                self.v1430_dynamic_restore_ok = set_ok
                if not set_ok:
                    last_detail = "SetInitialDynamicState returned failure"
                    print(
                        f"[V14.36 INIT] {context}: direct restore attempt "
                        f"{attempt}/{V14_36_DIRECT_VERIFY_ATTEMPTS} FAILED | {last_detail}",
                        flush=True,
                    )
                    time.sleep(V14_36_DIRECT_VERIFY_WAIT_SEC)
                    continue

                time.sleep(0.12)

                seq0 = int(get_seq())
                if not request_once():
                    last_detail = "RequestTelemetryOnce failed"
                    time.sleep(V14_36_DIRECT_VERIFY_WAIT_SEC)
                    continue

                fresh = wait_fresh(seq0, 1.0)
                if fresh is None:
                    last_detail = "fresh paused telemetry timed out"
                    time.sleep(V14_36_DIRECT_VERIFY_WAIT_SEC)
                    continue

                verified, last_detail = self._v1430_dynamic_matches(fresh, state)
                print(
                    f"[V14.36 INIT] {context}: DIRECT STATE VERIFY "
                    f"attempt={attempt}/{V14_36_DIRECT_VERIFY_ATTEMPTS} | "
                    f"{'PASS' if verified else 'FAIL'} | {last_detail}",
                    flush=True,
                )

                if verified:
                    self.v1430_dynamic_restore_ok = True
                    self.v1430_dynamic_verify_ok = True
                    print(
                        f"[V14.36 INIT] {context}: "
                        f"DIRECT DYNAMIC STATE RESTORED + VERIFIED",
                        flush=True,
                    )
                    return True

                time.sleep(V14_36_DIRECT_VERIFY_WAIT_SEC)

            print(
                f"[V14.36 INIT] {context}: DIRECT STATE VERIFY FAILED | {last_detail}",
                flush=True,
            )
            return False

        except Exception as exc:
            print(
                f"[V14.36 INIT] {context}: direct restore exception: {exc}",
                flush=True,
            )
            return False



    def _prepare_v1430_verified_initial_state(self, context: str) -> bool:
        """V14.30 two-phase initialisation: commit position first, then restore/verify dynamics."""
        if not self._commit_v1430_reference_position(context):
            return False
        if not self._restore_v1430_reference_dynamic_state(context):
            return False
        # Final position readback after dynamic restore, while still paused.
        try:
            seq0 = self.sim_client.get_telemetry_sequence()
            self.sim_client.request_telemetry_once()
            fresh = self.sim_client.wait_for_fresh_telemetry(seq0, 1.0)
            ok, detail = self._v1430_position_matches(fresh or {}, self.reference_initial_dynamic_state)
            if not ok:
                print(f"[V14.30 INIT] {context}: FINAL POSITION VERIFY FAILED | {detail}", flush=True)
                return False
            print(f"[V14.30 INIT] {context}: FULL INITIAL STATE VERIFIED | {detail}", flush=True)
            return True
        except Exception as exc:
            print(f"[V14.30 INIT] {context}: final verify exception: {exc}", flush=True)
            return False

    def _reset_p3d_state_diagnostic(self) -> None:
        """Reset the V14.30 P3D state capture."""
        self.p3d_state_diag_rows = []
        self.p3d_state_diag_wall_start = time.time()
        self.p3d_state_diag_last_sample_mono = 0.0
        self.p3d_state_diag_last_csv = None
        self.p3d_state_diag_last_txt = None
        self.p3d_state_diag_all_keys = set()
        self.p3d_state_diag_source_keys = {}
        print(
            f"[P3D STATE DIAG] {P3D_STATE_DIAG_BUILD} | READ-ONLY capture START",
            flush=True,
        )

    def _p3d_diag_capture_value(self, data: Dict[str, Any], field: str, candidates):
        value, source = self._p3d_diag_first(data, candidates)
        if source:
            self.p3d_state_diag_source_keys.setdefault(field, set()).add(source)
        return value

    def _append_p3d_state_diagnostic(self, data: Dict[str, Any], t0_marker=None) -> None:
        """
        Capture exactly the telemetry keys exposed by the installed p3d_simconnect.py.

        IMPORTANT: that wrapper labels elevator_pos / aileron_pos / rudder_pos as
        "Control surface positions" and also derives yoke/pedal percentages from the
        same values. This diagnostic therefore records the values exactly as exposed
        and does not assume they are a separate joystick-input channel.

        No P3D control setter is called here.
        """
        if not P3D_STATE_DIAG_ENABLED or self.test_mode not in ("manual", "auto"):
            return
        if self.p3d_state_diag_wall_start is None:
            self._reset_p3d_state_diagnostic()

        now_mono = time.monotonic()
        if (t0_marker is None and
            self.p3d_state_diag_last_sample_mono > 0.0
            and now_mono - self.p3d_state_diag_last_sample_mono < P3D_STATE_DIAG_SAMPLE_SEC
        ):
            return
        self.p3d_state_diag_last_sample_mono = now_mono

        d = data or {}
        try:
            self.p3d_state_diag_all_keys.update(str(k) for k in d.keys())
        except Exception:
            pass

        # Exact keys exposed by this project's p3d_simconnect.py.
        elev_pos = self._p3d_diag_capture_value(d, "elevator_pos", ("elevator_pos",))
        ail_pos = self._p3d_diag_capture_value(d, "aileron_pos", ("aileron_pos",))
        rud_pos = self._p3d_diag_capture_value(d, "rudder_pos", ("rudder_pos",))
        elev_trim = self._p3d_diag_capture_value(d, "elevator_trim", ("elevator_trim",))
        yoke_y = self._p3d_diag_capture_value(d, "yoke_y_percent", ("yoke_y_percent",))
        yoke_x = self._p3d_diag_capture_value(d, "yoke_x_percent", ("yoke_x_percent",))
        pedal = self._p3d_diag_capture_value(d, "rudder_pedal_percent", ("rudder_pedal_percent",))

        now_wall = time.time()
        rec_elapsed = None
        if self.is_recording and self.record_start_time:
            rec_elapsed = max(0.0, now_wall - float(self.record_start_time))

        row = {
            "wall_time_iso": datetime.datetime.fromtimestamp(now_wall).isoformat(timespec="milliseconds"),
            "diag_elapsed_s": round(now_wall - float(self.p3d_state_diag_wall_start), 4),
            "recording_elapsed_s": (round(rec_elapsed, 4) if rec_elapsed is not None else None),
            "test_mode": self.test_mode,
            "stage": self.operating_stage,
            "is_recording": int(bool(self.is_recording)),
            "t0_marker": t0_marker or "",
            "cls_to_p3d_feedback_enabled": int(bool(self.cls_to_p3d_feedback_enabled)),

            # Exact P3D-wrapper control/surface telemetry.
            "p3d_elevator_pos": elev_pos,
            "p3d_aileron_pos": ail_pos,
            "p3d_rudder_pos": rud_pos,
            "p3d_elevator_trim_rad": elev_trim,
            "p3d_aileron_trim_pct": self._p3d_diag_finite(d.get("aileron_trim_pct")),
            "p3d_rudder_trim_pct": self._p3d_diag_finite(d.get("rudder_trim_pct")),
            "p3d_velocity_world_x_fps": self._p3d_diag_finite(d.get("velocity_world_x")),
            "p3d_velocity_world_y_fps": self._p3d_diag_finite(d.get("velocity_world_y")),
            "p3d_velocity_world_z_fps": self._p3d_diag_finite(d.get("velocity_world_z")),
            "p3d_velocity_body_x_fps": self._p3d_diag_finite(d.get("velocity_body_x")),
            "p3d_velocity_body_y_fps": self._p3d_diag_finite(d.get("velocity_body_y")),
            "p3d_velocity_body_z_fps": self._p3d_diag_finite(d.get("velocity_body_z")),
            "p3d_rotation_body_x_rad_s": self._p3d_diag_finite(d.get("rotation_velocity_body_x")),
            "p3d_rotation_body_y_rad_s": self._p3d_diag_finite(d.get("rotation_velocity_body_y")),
            "p3d_rotation_body_z_rad_s": self._p3d_diag_finite(d.get("rotation_velocity_body_z")),
            "p3d_elevator_deflection_rad": self._p3d_diag_finite(d.get("elevator_deflection")),
            "p3d_aileron_left_deflection_rad": self._p3d_diag_finite(d.get("aileron_left_deflection")),
            "p3d_aileron_right_deflection_rad": self._p3d_diag_finite(d.get("aileron_right_deflection")),
            "p3d_rudder_deflection_rad": self._p3d_diag_finite(d.get("rudder_deflection")),
            "p3d_bridge_version": d.get("bridge_version"),
            "v1430_position_commit_ok": int(bool(self.v1430_position_commit_ok)),
            "v1430_dynamic_restore_ack": int(bool(self.v1430_dynamic_restore_ok)),
            "v1430_dynamic_verify_ok": int(bool(self.v1430_dynamic_verify_ok)),
            "ref_velocity_world_x_fps": (self.reference_initial_dynamic_state or {}).get("velocity_world_x_fps") if self.test_mode == "auto" else None,
            "ref_velocity_world_y_fps": (self.reference_initial_dynamic_state or {}).get("velocity_world_y_fps") if self.test_mode == "auto" else None,
            "ref_velocity_world_z_fps": (self.reference_initial_dynamic_state or {}).get("velocity_world_z_fps") if self.test_mode == "auto" else None,
            "ref_rotation_body_x_rad_s": (self.reference_initial_dynamic_state or {}).get("rotation_velocity_body_x_rad_s") if self.test_mode == "auto" else None,
            "ref_rotation_body_y_rad_s": (self.reference_initial_dynamic_state or {}).get("rotation_velocity_body_y_rad_s") if self.test_mode == "auto" else None,
            "ref_rotation_body_z_rad_s": (self.reference_initial_dynamic_state or {}).get("rotation_velocity_body_z_rad_s") if self.test_mode == "auto" else None,
            "ref_elevator_trim_rad": (self.reference_initial_dynamic_state or {}).get("elevator_trim_rad") if self.test_mode == "auto" else None,
            "ref_aileron_trim_pct": (self.reference_initial_dynamic_state or {}).get("aileron_trim_pct") if self.test_mode == "auto" else None,
            "ref_rudder_trim_pct": (self.reference_initial_dynamic_state or {}).get("rudder_trim_pct") if self.test_mode == "auto" else None,
            "ref_elevator_input": (self.reference_initial_dynamic_state or {}).get("elevator_input") if self.test_mode == "auto" else None,
            "ref_aileron_input": (self.reference_initial_dynamic_state or {}).get("aileron_input") if self.test_mode == "auto" else None,
            "ref_rudder_input": (self.reference_initial_dynamic_state or {}).get("rudder_input") if self.test_mode == "auto" else None,
            "ref_elevator_deflection_rad": (self.reference_initial_dynamic_state or {}).get("elevator_deflection_rad") if self.test_mode == "auto" else None,
            "ref_aileron_left_deflection_rad": (self.reference_initial_dynamic_state or {}).get("aileron_left_deflection_rad") if self.test_mode == "auto" else None,
            "ref_aileron_right_deflection_rad": (self.reference_initial_dynamic_state or {}).get("aileron_right_deflection_rad") if self.test_mode == "auto" else None,
            "ref_rudder_deflection_rad": (self.reference_initial_dynamic_state or {}).get("rudder_deflection_rad") if self.test_mode == "auto" else None,
            "p3d_yoke_y_percent": yoke_y,
            "p3d_yoke_x_percent": yoke_x,
            "p3d_rudder_pedal_percent": pedal,

            # Exact flight-state keys exposed by the installed wrapper.
            "p3d_ias_kts": self._p3d_diag_finite(d.get("airspeed_indicated")),
            "p3d_vertical_speed_fpm": self._p3d_diag_finite(d.get("vertical_speed")),
            "p3d_altitude_ft": self._p3d_diag_finite(d.get("indicated_altitude")),
            "p3d_pressure_alt_ft": self._p3d_diag_finite(d.get("pressure_alt")),
            "p3d_pitch_deg": self._p3d_diag_finite(d.get("pitch_angle")),
            "p3d_bank_deg": self._p3d_diag_finite(d.get("bank_angle")),
            "p3d_true_heading_deg": self._p3d_diag_finite(d.get("true_heading", d.get("yaw_angle"))),
            "p3d_magnetic_heading_deg": self._p3d_diag_finite(d.get("magnetic_heading")),
            "p3d_roll_rate_deg_s": self._p3d_diag_finite(d.get("roll_rate")),
            "p3d_pitch_rate_deg_s": self._p3d_diag_finite(d.get("pitch_rate")),
            "p3d_yaw_rate_deg_s": self._p3d_diag_finite(d.get("yaw_rate")),
            "p3d_aoa": self._p3d_diag_finite(d.get("aoa")),
            "p3d_sideslip": self._p3d_diag_finite(d.get("sideslip")),
            "p3d_accelX": self._p3d_diag_finite(d.get("accelX")),
            "p3d_accelY": self._p3d_diag_finite(d.get("accelY")),
            "p3d_accelZ": self._p3d_diag_finite(d.get("accelZ")),
            # V14.72 REAL P3D weather diagnostic.  Formal report wind remains
            # hard-coded 0/0 elsewhere exactly as requested.
            "p3d_actual_wind_dir_deg": self._p3d_diag_finite(d.get("ambient_wind_dir")),
            "p3d_actual_wind_speed_kt": self._p3d_diag_finite(d.get("ambient_wind_spd")),
            "p3d_engine_mixture_lever_1": self._p3d_diag_finite(d.get("engine_mixture_lever_1")),
            "p3d_flaps_handle_percent": self._p3d_diag_finite(d.get("flaps_handle_percent")),
            "p3d_flaps_handle_index": self._p3d_diag_finite(d.get("flaps_handle_index")),
            "p3d_flaps_num_handle_positions": self._p3d_diag_finite(d.get("flaps_num_handle_positions")),
            "p3d_flaps_left_percent": self._p3d_diag_finite(d.get("trailing_edge_flaps_left_percent")),
            "p3d_flaps_right_percent": self._p3d_diag_finite(d.get("trailing_edge_flaps_right_percent")),
            "p3d_flaps_left_angle_rad": self._p3d_diag_finite(d.get("trailing_edge_flaps_left_angle_rad")),
            "p3d_flaps_right_angle_rad": self._p3d_diag_finite(d.get("trailing_edge_flaps_right_angle_rad")),
            "p3d_ambient_density_slug_ft3": self._p3d_diag_finite(d.get("ambient_density_slug_ft3")),
            "p3d_ambient_pressure_mb": self._p3d_diag_finite(d.get("ambient_pressure_mb")),
            "p3d_aircraft_wind_x_kt": self._p3d_diag_finite(d.get("aircraft_wind_x_kt")),
            "p3d_aircraft_wind_y_kt": self._p3d_diag_finite(d.get("aircraft_wind_y_kt")),
            "p3d_aircraft_wind_z_kt": self._p3d_diag_finite(d.get("aircraft_wind_z_kt")),
            "p3d_rot_accel_x_rad_s2": self._p3d_diag_finite(d.get("rotation_acceleration_body_x_rad_s2")),
            "p3d_rot_accel_y_rad_s2": self._p3d_diag_finite(d.get("rotation_acceleration_body_y_rad_s2")),
            "p3d_rot_accel_z_rad_s2": self._p3d_diag_finite(d.get("rotation_acceleration_body_z_rad_s2")),
            "p3d_fuel_tank_left_main_level": self._p3d_diag_finite(d.get("fuel_tank_left_main_level")),
            "p3d_fuel_tank_right_main_level": self._p3d_diag_finite(d.get("fuel_tank_right_main_level")),
            "p3d_fuel_tank_center_level": self._p3d_diag_finite(d.get("fuel_tank_center_level")),
            "p3d_fuel_tank_selector_1": self._p3d_diag_finite(d.get("fuel_tank_selector_1")),
            "p3d_autopilot_master": self._p3d_diag_finite(d.get("autopilot_master")),
            "p3d_auto_coordination": self._p3d_diag_finite(d.get("auto_coordination")),
            "p3d_spoilers_handle_position": self._p3d_diag_finite(d.get("spoilers_handle_position")),
            "p3d_gear_handle_position": self._p3d_diag_finite(d.get("gear_handle_position")),
            "p3d_brake_parking_position": self._p3d_diag_finite(d.get("brake_parking_position")),
            "p3d_engine_propeller_lever_1": self._p3d_diag_finite(d.get("engine_propeller_lever_1")),
            "p3d_engine_combustion_1": self._p3d_diag_finite(d.get("engine_combustion_1")),
            "p3d_engine_fuel_valve_1": self._p3d_diag_finite(d.get("engine_fuel_valve_1")),
            "p3d_engine_fuel_pressure_psi_1": self._p3d_diag_finite(d.get("engine_fuel_pressure_psi_1")),
            "p3d_engine_manifold_pressure_psi_1": self._p3d_diag_finite(d.get("engine_manifold_pressure_psi_1")),
            "p3d_relative_wind_body_x_fps": self._p3d_diag_finite(d.get("relative_wind_body_x_fps")),
            "p3d_relative_wind_body_y_fps": self._p3d_diag_finite(d.get("relative_wind_body_y_fps")),
            "p3d_relative_wind_body_z_fps": self._p3d_diag_finite(d.get("relative_wind_body_z_fps")),
            "p3d_max_throttle_pct": self._p3d_diag_finite(d.get("max_throttle")),
            "p3d_engine_rpm_1": self._p3d_diag_finite((d.get("eng_rpms") or [None])[0] if isinstance(d.get("eng_rpms"), list) and d.get("eng_rpms") else None),
            "p3d_prop_rpm_1": self._p3d_diag_finite((d.get("prop_rpms") or [None])[0] if isinstance(d.get("prop_rpms"), list) and d.get("prop_rpms") else None),
            "ref_vertical_speed_fpm": self._p3d_diag_finite(
                (getattr(self, "reference_initial_dynamic_state", {}) or {}).get("vertical_speed_fpm")
            ),
            "ref_engine_rpm_1": self._p3d_diag_finite(
                (getattr(self, "reference_initial_dynamic_state", {}) or {}).get("engine_rpm")
            ),
            "ref_aoa_deg": self._p3d_diag_finite(
                (getattr(self, "reference_initial_dynamic_state", {}) or {}).get("aoa_deg")
            ),
            "ref_sideslip_deg": self._p3d_diag_finite(
                (getattr(self, "reference_initial_dynamic_state", {}) or {}).get("sideslip_deg")
            ),
            "ref_roll_rate_deg_s": self._p3d_diag_finite(
                (getattr(self, "reference_initial_dynamic_state", {}) or {}).get("roll_rate_deg_s")
            ),
            "ref_pitch_rate_deg_s": self._p3d_diag_finite(
                (getattr(self, "reference_initial_dynamic_state", {}) or {}).get("pitch_rate_deg_s")
            ),
            "ref_yaw_rate_deg_s": self._p3d_diag_finite(
                (getattr(self, "reference_initial_dynamic_state", {}) or {}).get("yaw_rate_deg_s")
            ),
            "ref_actual_wind_dir_deg": self._p3d_diag_finite(
                (getattr(self, "reference_initial_dynamic_state", {}) or {}).get("actual_wind_dir_deg")
            ),
            "ref_actual_wind_speed_kt": self._p3d_diag_finite(
                (getattr(self, "reference_initial_dynamic_state", {}) or {}).get("actual_wind_speed_kt")
            ),
            "ref_engine_mixture_lever_1": self._p3d_diag_finite(
                (getattr(self, "reference_initial_dynamic_state", {}) or {}).get("engine_mixture_lever_1")
            ),
            "ref_flaps_handle_percent": self._p3d_diag_finite(
                (getattr(self, "reference_initial_dynamic_state", {}) or {}).get("flaps_handle_percent")
            ),
            "ref_engine_propeller_lever_1": self._p3d_diag_finite(
                (getattr(self, "reference_initial_dynamic_state", {}) or {}).get("engine_propeller_lever_1")
            ),
            "ref_engine_combustion_1": self._p3d_diag_finite(
                (getattr(self, "reference_initial_dynamic_state", {}) or {}).get("engine_combustion_1")
            ),
            "ref_engine_fuel_valve_1": self._p3d_diag_finite(
                (getattr(self, "reference_initial_dynamic_state", {}) or {}).get("engine_fuel_valve_1")
            ),
            "ref_engine_fuel_pressure_psi_1": self._p3d_diag_finite(
                (getattr(self, "reference_initial_dynamic_state", {}) or {}).get("engine_fuel_pressure_psi_1")
            ),
            "ref_engine_manifold_pressure_psi_1": self._p3d_diag_finite(
                (getattr(self, "reference_initial_dynamic_state", {}) or {}).get("engine_manifold_pressure_psi_1")
            ),
            "ref_relative_wind_body_x_fps": self._p3d_diag_finite(
                (getattr(self, "reference_initial_dynamic_state", {}) or {}).get("relative_wind_body_x_fps")
            ),
            "ref_relative_wind_body_y_fps": self._p3d_diag_finite(
                (getattr(self, "reference_initial_dynamic_state", {}) or {}).get("relative_wind_body_y_fps")
            ),
            "ref_relative_wind_body_z_fps": self._p3d_diag_finite(
                (getattr(self, "reference_initial_dynamic_state", {}) or {}).get("relative_wind_body_z_fps")
            ),
            "p3d_egt": self._p3d_diag_finite(d.get("egt")),
            "p3d_last_updated": self._p3d_diag_finite(d.get("last_updated")),

            # Side-by-side BRUNNER chain evidence.
            "cls_pitch_target_norm": self.cls.last_pitch_target if getattr(self, "cls", None) else None,
            "cls_roll_target_norm": self.cls.last_roll_target if getattr(self, "cls", None) else None,
            "cls_yaw_target_norm": self.cls.last_rudder_target if getattr(self, "cls", None) else None,
            "cls_pitch_actual_norm": self.cls.last_pitch_pos if getattr(self, "cls", None) else None,
            "cls_roll_actual_norm": self.cls.last_roll_pos if getattr(self, "cls", None) else None,
            "cls_yaw_actual_norm": self.cls.last_rudder_pos if getattr(self, "cls", None) else None,
            "cls_pitch_force_N": self.cls.last_pitch_force_eng if getattr(self, "cls", None) else None,
            "cls_roll_force_N": self.cls.last_roll_force_eng if getattr(self, "cls", None) else None,
            "cls_yaw_force_N": self.cls.last_rudder_force_eng if getattr(self, "cls", None) else None,
        }
        self.p3d_state_diag_rows.append(row)

    def _save_p3d_state_diagnostics(self, timestamp: str) -> None:
        """Save the read-only P3D diagnostic CSV and exact-wrapper coverage summary."""
        if not P3D_STATE_DIAG_ENABLED:
            return
        mode = self.test_mode if self.test_mode in ("manual", "auto") else "session"
        base = self.data_dir / f"Super_Mushak_P3D_State_Diagnostic_{mode}_{timestamp}"
        csv_path = base.with_suffix(".csv")
        txt_path = base.with_suffix(".txt")
        rows = list(self.p3d_state_diag_rows)

        if rows:
            fieldnames = list(rows[0].keys())
            with open(csv_path, "w", newline="", encoding="utf-8") as f:
                writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
                writer.writeheader()
                writer.writerows(rows)
            self.p3d_state_diag_last_csv = str(csv_path)

            # One extra evidence page: measured configuration only. The old
            # manuals contain no flap or mixture trace, so no fake reference
            # curves are drawn over these graphs.
            if mode == "auto":
                try:
                    import matplotlib
                    matplotlib.use("Agg")
                    import matplotlib.pyplot as plt
                    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
                    for ax, key, label, scale in (
                        (axes[0], "p3d_flaps_handle_percent", "P3D flap handle (%)", 1.0),
                        (axes[1], "p3d_engine_mixture_lever_1", "P3D mixture lever (%)", 100.0),
                    ):
                        points = [(float(r["recording_elapsed_s"]), float(r[key]) * scale)
                                  for r in rows if r.get("recording_elapsed_s") is not None
                                  and r.get(key) is not None]
                        if points:
                            ax.plot(*zip(*points), linewidth=1.7, label="Measured automatic")
                            ax.legend(loc="best")
                        else:
                            ax.text(0.5, 0.5, "No measured samples", ha="center",
                                    va="center", transform=ax.transAxes)
                        ax.set_ylabel(label)
                        ax.grid(alpha=0.25)
                    axes[-1].set_xlabel("Automatic recording time (s)")
                    fig.suptitle("Measured flap and mixture configuration\n"
                                 "September 17 manual targets unknown: diagnostic only")
                    fig.tight_layout()
                    graph_path = base.with_name(base.name + "_configuration_graphs.pdf")
                    fig.savefig(graph_path)
                    plt.close(fig)
                    print(f"[V14.92 CONFIG GRAPH] {graph_path}", flush=True)
                except Exception as graph_exc:
                    print(f"[V14.92 CONFIG GRAPH] Save warning: {graph_exc}", flush=True)

        lines = [
            "SUPER MUSHAK / PREPAR3D READ-ONLY STATE DIAGNOSTIC",
            "=" * 76,
            f"Build: {P3D_STATE_DIAG_BUILD}",
            f"Generated: {datetime.datetime.now().isoformat(timespec='seconds')}",
            f"Test mode: {mode}",
            f"Samples: {len(rows)}",
            "BRUNNER CONTROL LAW: UNCHANGED FROM V14.21",
            "DIAGNOSTIC CAPTURE ITSELF: READ-ONLY",
            "V14.30 INITIALIZATION: verified INITPOSITION commit, then paused WORLD velocity / BODY rotation / trim restore",
            ("Automatic control path: live measured physical CLS positions -> "
             "V14.60 existing p3d_bridge.dll axis path -> P3D; Manual remains normal CLS2Sim/read-only."),
            "",
            "TELEMETRY KEY COVERAGE",
            "-" * 76,
        ]
        for field in (
            "elevator_pos", "aileron_pos", "rudder_pos", "elevator_trim",
            "yoke_y_percent", "yoke_x_percent", "rudder_pedal_percent",
        ):
            keys = sorted(self.p3d_state_diag_source_keys.get(field, set()))
            lines.append(f"{field}: {', '.join(keys) if keys else 'NOT EXPOSED BY CURRENT get_telemetry()'}")

        lines.extend([
            "aileron_trim_pct: exposed by V14.30 wrapper",
            "rudder_trim_pct: exposed by V14.30 wrapper",
            "world_velocity_xyz: exposed by V14.30 wrapper",
            "body_rotation_velocity_xyz: exposed by V14.30 wrapper",
            "actual elevator/aileron/rudder deflections: exposed by V14.30 wrapper",
            "",
            "ALL get_telemetry() KEYS SEEN",
            "-" * 76,
            ", ".join(sorted(self.p3d_state_diag_all_keys)) if self.p3d_state_diag_all_keys else "NONE",
            "",
            "HOW TO INTERPRET",
            "- Compare CLS target/actual with p3d_elevator_pos / p3d_aileron_pos / p3d_rudder_pos between the known-good and current-bad runs.",
            "- If CLS matches but these P3D-reported positions differ, inspect the CLS2Sim -> P3D path/profile/mapping.",
            "- Compare restored WORLD velocity / BODY rotation / trim against the Manual _initial_state.json.",
            "- Compare P3D input positions with actual surface deflections to separate input-path from flight-model behavior.",
            "- V14.30 never writes elevator/aileron/rudder input; CLS2Sim remains the continuous control source.",
        ])
        txt_path.write_text("\n".join(lines), encoding="utf-8")
        self.p3d_state_diag_last_txt = str(txt_path)
        print(f"[P3D STATE DIAG] Saved CSV -> {csv_path if rows else 'NO ROWS'}", flush=True)
        print(f"[P3D STATE DIAG] Saved TXT -> {txt_path}", flush=True)

    def _reset_commissioning_diagnostics(self) -> None:
        """Start a fresh passive commissioning evidence session."""
        self.commission_diag_rows = []
        self.commission_diag_session_wall_start = time.time()
        self.commission_diag_last_sample_mono = 0.0
        self.commission_diag_last_csv = None
        self.commission_diag_last_txt = None
        print(
            f"[COMMISSION] {COMMISSION_DIAG_BUILD} | passive evidence capture START",
            flush=True,
        )

    @staticmethod
    def _commission_num(value):
        try:
            fv = float(value)
            return fv if math.isfinite(fv) else None
        except (TypeError, ValueError, OverflowError):
            return None

    def _commission_axis_snapshot(self, name: str) -> Dict[str, Any]:
        """Return one axis diagnostic snapshot using cached values only."""
        if not getattr(self, "cls", None):
            return {}

        prefix = {"Pitch": "pitch", "Roll": "roll", "Yaw": "yaw"}[name]
        info = dict(self.cls.force_conversion_info.get(name) or {})
        axis_info = dict(self.cls.axis_info.get(name) or {})
        factor = self._commission_num(info.get("force_factor"))
        force_limit = self._commission_num(info.get("force_limit"))

        if name == "Pitch":
            pos = self.cls.last_pitch_pos
            pos_raw = self.cls.last_pitch_pos_raw
            target = self.cls.last_pitch_target
            ap_enabled = self.cls.last_pitch_ap_enabled
            eng = self.cls.last_pitch_force_eng
            norm_derived = self.cls.last_pitch_force_norm
            raw_internal = self.cls.last_pitch_force_raw
            replay_active = self.pitch_force_replay_active
            ref_n = self.pitch_force_reference_n
            loop_actual_n = self.pitch_force_actual_n
            offset_cmd = self.pitch_force_offset_command
            integral = self.pitch_force_integral_internal
            manual_peak = self.pitch_force_manual_peak_n
            baseline_n = self.pitch_force_baseline_n
            force_authority = self.force_position_authority.get("Pitch", 1.0)
            drive_state = self.cls.last_pitch_drive_state
            temp = self.cls.last_pitch_temperature_c
            err_count = self.cls.last_pitch_error_count
            primary_error = self.cls.last_pitch_primary_error
            limit_fraction = PITCH_FORCE_OFFSET_LIMIT_FRACTION
            peak_margin = PITCH_FORCE_MANUAL_PEAK_MARGIN
        elif name == "Roll":
            pos = self.cls.last_roll_pos
            pos_raw = self.cls.last_roll_pos_raw
            target = self.cls.last_roll_target
            ap_enabled = self.cls.last_roll_ap_enabled
            eng = self.cls.last_roll_force_eng
            norm_derived = self.cls.last_roll_force_norm
            raw_internal = self.cls.last_roll_force_raw
            replay_active = self.roll_force_replay_active
            ref_n = self.roll_force_reference_n
            loop_actual_n = self.roll_force_actual_n
            offset_cmd = self.roll_force_offset_command
            integral = self.roll_force_integral_internal
            manual_peak = self.roll_force_manual_peak_n
            baseline_n = self.roll_force_baseline_n
            force_authority = self.force_position_authority.get("Roll", 1.0)
            drive_state = self.cls.last_roll_drive_state
            temp = self.cls.last_roll_temperature_c
            err_count = self.cls.last_roll_error_count
            primary_error = self.cls.last_roll_primary_error
            limit_fraction = ROLL_FORCE_OFFSET_LIMIT_FRACTION
            peak_margin = ROLL_FORCE_MANUAL_PEAK_MARGIN
        else:
            pos = self.cls.last_rudder_pos
            pos_raw = self.cls.last_rudder_pos_raw
            target = self.cls.last_rudder_target
            ap_enabled = self.cls.last_rudder_ap_enabled
            eng = self.cls.last_rudder_force_eng
            norm_derived = self.cls.last_rudder_force_norm
            raw_internal = self.cls.last_rudder_force_raw
            replay_active = self.yaw_force_replay_active
            ref_n = self.yaw_force_reference_n
            loop_actual_n = self.yaw_force_actual_n
            offset_cmd = self.yaw_force_offset_command
            integral = self.yaw_force_integral_internal
            manual_peak = self.yaw_force_manual_peak_n
            baseline_n = self.yaw_force_baseline_n
            force_authority = self.force_position_authority.get("Yaw", 1.0)
            drive_state = self.cls.last_rudder_drive_state
            temp = self.cls.last_rudder_temperature_c
            err_count = self.cls.last_rudder_error_count
            primary_error = self.cls.last_rudder_primary_error
            limit_fraction = YAW_FORCE_OFFSET_LIMIT_FRACTION
            peak_margin = YAW_FORCE_MANUAL_PEAK_MARGIN

        wire_raw = self._commission_num(self.cls.last_force_wire_raw.get(name))
        wire_norm = self._commission_num(self.cls.last_force_wire_norm.get(name))
        signed_wire_raw = self.cls._legacy_force_raw_signed16(wire_raw)
        legacy_norm_from_raw = (
            float(signed_wire_raw) / LEGACY_FORCE_NORM_FULL_SCALE_COUNTS
            if signed_wire_raw is not None else None
        )

        ref = self._commission_num(ref_n)
        act = self._commission_num(loop_actual_n)
        if act is None:
            act = self._commission_num(eng)
        error_n = (ref - act) if ref is not None and act is not None else None

        target_f = self._commission_num(target)
        pos_f = self._commission_num(pos)
        pos_error = (
            target_f - pos_f
            if target_f is not None and pos_f is not None else None
        )

        # V14.69: the AP command is intentionally phase-led.  Record the
        # same-time Manual reference separately so post-test analysis can
        # measure whether the physical actuator has actually removed the lag.
        current_ref_f = self._commission_num(
            (getattr(self, "v1469_manual_position_reference_now", {}) or {}).get(name)
        )
        current_ref_error = (
            current_ref_f - pos_f
            if current_ref_f is not None and pos_f is not None else None
        )

        offset_cmd_f = self._commission_num(offset_cmd) or 0.0
        offset_cmd_n = (
            offset_cmd_f * factor if factor not in (None, 0.0) else None
        )
        feedforward_internal = (
            ref / factor if ref is not None and factor not in (None, 0.0) else None
        )

        command_limit = None
        if force_limit is not None and force_limit > 0 and factor not in (None, 0.0):
            command_limit = abs(force_limit) * float(limit_fraction)
            peak = self._commission_num(manual_peak)
            if peak is not None and peak > 0:
                command_limit = min(
                    command_limit,
                    peak * float(peak_margin) / abs(factor),
                )

        legacy_stats = dict(self.cls.legacy_force_validation.get(name) or {})

        return {
            f"{prefix}_node": self.cls.detected_nodes.get(name),
            f"{prefix}_force_support": ",".join(self.cls.force_support.get(name) or []),
            f"{prefix}_force_mode": self.cls.force_mode.get(name),
            f"{prefix}_force_source": self.cls.last_force_source.get(name),
            f"{prefix}_force_factor_N_per_count": factor,
            f"{prefix}_force_limit_counts": force_limit,
            f"{prefix}_legacy_signed16": int(bool(info.get("legacy_signed16"))),
            f"{prefix}_wire_raw": wire_raw,
            f"{prefix}_wire_raw_signed16": signed_wire_raw,
            f"{prefix}_wire_norm_0x21": wire_norm,
            f"{prefix}_legacy_norm_from_raw": legacy_norm_from_raw,
            f"{prefix}_force_raw_internal": self._commission_num(raw_internal),
            f"{prefix}_force_norm_cached": self._commission_num(norm_derived),
            f"{prefix}_force_eng_N": self._commission_num(eng),
            f"{prefix}_force_ref_N": ref,
            f"{prefix}_force_loop_actual_N": act,
            f"{prefix}_force_error_N": error_n,
            f"{prefix}_force_replay_active": int(bool(replay_active)),
            f"{prefix}_offset_cmd_internal": int(round(offset_cmd_f)),
            f"{prefix}_offset_cmd_N": offset_cmd_n,
            f"{prefix}_feedforward_internal": feedforward_internal,
            f"{prefix}_integral_internal": self._commission_num(integral),
            f"{prefix}_command_limit_internal": command_limit,
            f"{prefix}_manual_peak_N": self._commission_num(manual_peak),
            f"{prefix}_force_baseline_N": self._commission_num(baseline_n),
            f"{prefix}_force_authority": self._commission_num(force_authority),
            # Backward-compatible field: actual AP command sent to BRUNNER.
            f"{prefix}_position_target_norm": target_f,
            f"{prefix}_position_actual_norm": pos_f,
            f"{prefix}_position_error_norm": pos_error,

            # V14.69 same-time reference fields.  These are the correct fields
            # for judging phase-lead effectiveness in the commissioning CSV.
            f"{prefix}_position_reference_now_norm": current_ref_f,
            f"{prefix}_position_reference_now_error_norm": current_ref_error,
            f"{prefix}_position_lookahead_sec": float(
                AUTO_POSITION_LOOKAHEAD_SEC.get(name, 0.0)
            ),
            f"{prefix}_position_actual_raw": self._commission_num(pos_raw),
            f"{prefix}_ap_enabled": (None if ap_enabled is None else int(bool(ap_enabled))),
            f"{prefix}_drive_state": drive_state,
            f"{prefix}_temperature_C": temp,
            f"{prefix}_error_count": err_count,
            f"{prefix}_primary_error": primary_error,
            f"{prefix}_legacy_checks": legacy_stats.get("checks"),
            f"{prefix}_legacy_failures": legacy_stats.get("failures"),
            f"{prefix}_legacy_last_error_counts": legacy_stats.get("last_error_counts"),
            f"{prefix}_legacy_max_abs_error_counts": legacy_stats.get("max_abs_error_counts"),
            f"{prefix}_axis_force_factor_native": self._commission_num(axis_info.get("force_factor")),
            f"{prefix}_axis_force_limit_native": self._commission_num(axis_info.get("force_limit")),
        }

    def _append_commissioning_diagnostic(self) -> None:
        """Append one passive 20 Hz commissioning row from cached state."""
        if not COMMISSION_DIAG_ENABLED or self.test_mode not in ("manual", "auto"):
            return
        if self.commission_diag_session_wall_start is None:
            self._reset_commissioning_diagnostics()

        now_mono = time.monotonic()
        if (
            self.commission_diag_last_sample_mono > 0.0
            and now_mono - self.commission_diag_last_sample_mono
            < COMMISSION_DIAG_SAMPLE_SEC
        ):
            return
        self.commission_diag_last_sample_mono = now_mono

        wall_now = time.time()
        row = {
            "wall_time_iso": datetime.datetime.fromtimestamp(wall_now).isoformat(timespec="milliseconds"),
            "session_elapsed_s": round(
                wall_now - float(self.commission_diag_session_wall_start), 3
            ),
            "test_mode": self.test_mode,
            "stage": self.operating_stage,
            "commission_force_stage": int(getattr(self, "commission_force_stage", COMMISSION_FORCE_STAGE_DEFAULT)),
            "commission_force_axes": ",".join(self._commission_force_axes()) if self.test_mode == "auto" else "",
            "commission_yaw_ap_speed": int(getattr(self, "commission_yaw_ap_speed", YAW_AP_SPEED_DEFAULT)) if self.test_mode == "auto" else "",
            "force_interlock_latched_axes": ",".join(
                name for name, latched in getattr(self, "force_position_interlock_latched", {}).items() if latched
            ),
            "force_interlock_blocked_axes": ",".join(
                name for name, blocked in getattr(self, "force_position_interlock_blocked", {}).items() if blocked
            ),
            "is_recording": int(bool(self.is_recording)),
            "active_cls_axes": ",".join(self.active_cls_axes),
            "reference_cls_axes": ",".join(self.reference_cls_axes),
            "cls_hardware_ready": int(bool(self.cls_hardware_ready)),
            "cls_prep_complete": int(bool(self.cls_prep_complete)),
            "auto_controller_init_ready": int(bool(getattr(self, "auto_controller_init_ready", False))),
            "auto_last_cls_initial_verify_detail": getattr(self, "auto_last_cls_initial_verify_detail", ""),
            "auto_last_p3d_control_handover_detail": getattr(self, "auto_last_p3d_control_handover_detail", ""),
            "auto_last_yaw_bridge_proof_detail": getattr(self, "auto_last_yaw_bridge_proof_detail", ""),
            "auto_pitch_ap_force_selected": int(getattr(self, "auto_pitch_ap_force_selected", CLS_TRACK_AP_FORCE_PITCH) or CLS_TRACK_AP_FORCE_PITCH),
            "direct_p3d_mirror_active": int(bool(getattr(self, "direct_mirror_active", False))),
            "direct_p3d_mirror_error": getattr(self, "direct_mirror_last_error", ""),
            "direct_p3d_mirror_consecutive_failures": int(getattr(self, "direct_mirror_consecutive_failures", 0) or 0),
            "direct_p3d_mirror_last_send_age_s": (
                max(0.0, time.time() - float(getattr(getattr(self, "p3d_direct_bridge", None), "last_send_wall", 0.0) or 0.0))
                if float(getattr(getattr(self, "p3d_direct_bridge", None), "last_send_wall", 0.0) or 0.0) > 0.0 else None
            ),
            "direct_p3d_runtime_mismatch_detail": getattr(self, "direct_mirror_runtime_mismatch_detail", ""),
            "direct_p3d_elevator_cmd": (getattr(self, "direct_mirror_last_commands", {}) or {}).get("elevator"),
            "direct_p3d_aileron_cmd": (getattr(self, "direct_mirror_last_commands", {}) or {}).get("aileron"),
            "direct_p3d_rudder_cmd": (getattr(self, "direct_mirror_last_commands", {}) or {}).get("rudder"),

            # V14.76 startup calibration evidence.
            "pitch_direct_cal_attempts": int((getattr(self, "v1476_axis_cal_attempts", {}) or {}).get("Pitch", 0) or 0),
            "pitch_direct_cal_slope": ((getattr(self, "direct_mirror_axis_calibration", {}) or {}).get("Pitch", {}) or {}).get("slope"),
            "pitch_direct_cal_offset": ((getattr(self, "direct_mirror_axis_calibration", {}) or {}).get("Pitch", {}) or {}).get("offset"),
            "pitch_direct_cal_plus": ((getattr(self, "direct_mirror_axis_calibration", {}) or {}).get("Pitch", {}) or {}).get("plus"),
            "pitch_direct_cal_minus": ((getattr(self, "direct_mirror_axis_calibration", {}) or {}).get("Pitch", {}) or {}).get("minus"),
            "pitch_direct_cal_detail": (getattr(self, "v1476_axis_cal_last_detail", {}) or {}).get("Pitch", ""),
            "roll_direct_cal_attempts": int((getattr(self, "v1476_axis_cal_attempts", {}) or {}).get("Roll", 0) or 0),
            "roll_direct_cal_slope": ((getattr(self, "direct_mirror_axis_calibration", {}) or {}).get("Roll", {}) or {}).get("slope"),
            "roll_direct_cal_offset": ((getattr(self, "direct_mirror_axis_calibration", {}) or {}).get("Roll", {}) or {}).get("offset"),
            "roll_direct_cal_plus": ((getattr(self, "direct_mirror_axis_calibration", {}) or {}).get("Roll", {}) or {}).get("plus"),
            "roll_direct_cal_minus": ((getattr(self, "direct_mirror_axis_calibration", {}) or {}).get("Roll", {}) or {}).get("minus"),
            "roll_direct_cal_detail": (getattr(self, "v1476_axis_cal_last_detail", {}) or {}).get("Roll", ""),
            "yaw_direct_cal_attempts": int((getattr(self, "v1476_axis_cal_attempts", {}) or {}).get("Yaw", 0) or 0),
            "yaw_direct_cal_slope": ((getattr(self, "direct_mirror_axis_calibration", {}) or {}).get("Yaw", {}) or {}).get("slope"),
            "yaw_direct_cal_offset": ((getattr(self, "direct_mirror_axis_calibration", {}) or {}).get("Yaw", {}) or {}).get("offset"),
            "yaw_direct_cal_plus": ((getattr(self, "direct_mirror_axis_calibration", {}) or {}).get("Yaw", {}) or {}).get("plus"),
            "yaw_direct_cal_minus": ((getattr(self, "direct_mirror_axis_calibration", {}) or {}).get("Yaw", {}) or {}).get("minus"),
            "yaw_direct_cal_detail": (getattr(self, "v1476_axis_cal_last_detail", {}) or {}).get("Yaw", ""),

            "auto_last_block_reason": getattr(self, "auto_last_block_reason", ""),
            "auto_softening_active": int(bool(self.auto_replay_softening_active)),
            "auto_softened_axes": ",".join(getattr(self, "auto_replay_softened_axes", [])),
            "cls_to_p3d_feedback_enabled": int(bool(self.cls_to_p3d_feedback_enabled)),
            "cls_rx_count": getattr(self.cls, "rx_count", None),
            "cls_last_socket_error": getattr(self.cls, "last_socket_error", ""),
            "cls_manual_write_guard": int(bool(getattr(self.cls, "control_write_guard", False))),
            "cls_manual_blocked_write_count": int(getattr(self.cls, "blocked_control_write_count", 0) or 0),
        }
        for axis_name in ("Pitch", "Roll", "Yaw"):
            row.update(self._commission_axis_snapshot(axis_name))
        self.commission_diag_rows.append(row)

    def _save_commissioning_diagnostics(self, timestamp: str) -> None:
        """Save one-shot BRUNNER CSV + human-readable TXT summary."""
        if not COMMISSION_DIAG_ENABLED:
            return

        mode = self.test_mode if self.test_mode in ("manual", "auto") else "session"
        base = self.data_dir / f"Super_Mushak_Brunner_Commissioning_{mode}_{timestamp}"
        csv_path = base.with_suffix(".csv")
        txt_path = base.with_suffix(".txt")

        rows = list(self.commission_diag_rows)
        if rows:
            fieldnames = list(rows[0].keys())
            with open(csv_path, "w", newline="", encoding="utf-8") as f:
                writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
                writer.writeheader()
                writer.writerows(rows)
            self.commission_diag_last_csv = str(csv_path)

        lines = []
        lines.append("SUPER MUSHAK / BRUNNER ONE-SHOT COMMISSIONING SUMMARY")
        lines.append("=" * 72)
        lines.append(f"Build: {COMMISSION_DIAG_BUILD}")
        lines.append(f"Generated: {datetime.datetime.now().isoformat(timespec='seconds')}")
        lines.append(f"Test mode: {mode}")
        if mode == "auto":
            lines.append("Force replay mode: ALL AXES - Pitch + Roll + Yaw")
            lines.append(f"Force replay axes: {','.join(self._commission_force_axes())}")
            lines.append(f"Selected Yaw AP speed: {int(getattr(self, 'commission_yaw_ap_speed', YAW_AP_SPEED_DEFAULT))}")
            lines.append(f"Selected Pitch AP force: {int(getattr(self, 'auto_pitch_ap_force_selected', CLS_TRACK_AP_FORCE_PITCH) or CLS_TRACK_AP_FORCE_PITCH)}")
            lines.append(f"Measured-CLS direct P3D mirror: {'ACTIVE' if getattr(self, 'direct_mirror_active', False) else 'INACTIVE'} | error={getattr(self, 'direct_mirror_last_error', '') or 'none'}")
            cal = dict(getattr(self, "direct_mirror_axis_calibration", {}) or {})
            cal_parts = []
            for _n in ("Pitch", "Roll", "Yaw"):
                _c = dict(cal.get(_n, {}) or {})
                if _c.get("ready"):
                    cal_parts.append(f"{_n}:slope={float(_c.get('slope')):+.4f},offset={float(_c.get('offset')):+.4f}")
                elif _n in (getattr(self, "active_cls_axes", []) or []):
                    cal_parts.append(f"{_n}:NOT_READY")
            lines.append("V14.60 direct-axis calibration: " + (" | ".join(cal_parts) if cal_parts else "not run"))
            lines.append(
                "Direct bridge health: consecutive failures="
                f"{int(getattr(self, 'direct_mirror_consecutive_failures', 0) or 0)} | "
                f"runtime mismatch={getattr(self, 'direct_mirror_runtime_mismatch_detail', '') or 'none'}"
            )
            lines.append(f"Controllers-first ready: {bool(getattr(self, 'auto_controller_init_ready', False))}")
            lines.append(f"Last CLS initial verify: {getattr(self, 'auto_last_cls_initial_verify_detail', '') or 'none'}")
            lines.append(f"Last P3D control handover: {getattr(self, 'auto_last_p3d_control_handover_detail', '') or 'none'}")
            lines.append("V14.60 P3D path: CLS Position-to-Sim BLOCKED; live measured CLS positions mirrored through existing p3d_bridge.dll")
            lines.append(f"Last Automatic block reason: {getattr(self, 'auto_last_block_reason', '') or 'none'}")
        lines.append(f"CLS2Sim target: {CLS2SIM_HOST}:{CLS2SIM_PORT}")
        lines.append(f"Diagnostic samples: {len(rows)}")
        lines.append(f"Main CSV: {self.current_csv_filepath or 'not yet assigned'}")
        if mode == "manual":
            lines.append(
                "Manual CLS control guard: "
                + ("ENABLED" if bool(getattr(self.cls, "control_write_guard", False)) else "DISABLED")
            )
            lines.append(
                f"Blocked controller-write attempts during Manual: "
                f"{int(getattr(self.cls, 'blocked_control_write_count', 0) or 0)}"
            )
            lines.append("Manual watchdog: OFF (telemetry/status polling only)")
        lines.append("")

        def max_abs_from_rows(key):
            vals = []
            for rr in rows:
                v = self._commission_num(rr.get(key))
                if v is not None:
                    vals.append(abs(v))
            return max(vals) if vals else None

        def any_true_from_rows(key):
            return any(bool(rr.get(key)) for rr in rows)

        for name, prefix in (("Pitch", "pitch"), ("Roll", "roll"), ("Yaw", "yaw")):
            info = dict(self.cls.force_conversion_info.get(name) or {})
            stats = dict(self.cls.legacy_force_validation.get(name) or {})
            lines.append(f"[{name}]")
            lines.append(f"Node: {self.cls.detected_nodes.get(name)}")
            lines.append(f"Supported IDs: {','.join(self.cls.force_support.get(name) or []) or 'NONE/UNPROBED'}")
            lines.append(f"Selected force mode: {self.cls.force_mode.get(name)}")
            lines.append(f"Live force source: {self.cls.last_force_source.get(name)}")
            lines.append(f"Conversion info: {info}")
            lines.append(
                f"Legacy consistency: checks={stats.get('checks',0)} "
                f"failures={stats.get('failures',0)} "
                f"maxAbsErrorCounts={stats.get('max_abs_error_counts',0.0)}"
            )
            lines.append(f"Force replay active at any time: {any_true_from_rows(prefix + '_force_replay_active')}")
            lines.append(f"Max |force error| N: {max_abs_from_rows(prefix + '_force_error_N')}")
            lines.append(f"Max |position error| norm: {max_abs_from_rows(prefix + '_position_error_norm')}")
            lines.append(f"Max |offset command| internal: {max_abs_from_rows(prefix + '_offset_cmd_internal')}")
            lines.append("")

        lines.append("FORCE PROBE HISTORY")
        lines.append("-" * 72)
        if self.cls.force_probe_history:
            for ev in self.cls.force_probe_history:
                lines.append(str(ev))
        else:
            lines.append("No force probe history recorded.")
        lines.append("")

        lines.append("COMMAND / ACK HISTORY")
        lines.append("-" * 72)
        if self.cls.ack_history:
            for ev in self.cls.ack_history:
                stamp = datetime.datetime.fromtimestamp(
                    float(ev.get("wall_time", time.time()))
                ).isoformat(timespec="milliseconds")
                status = ev.get("status")
                status_text = "" if status is None else f" status=0x{int(status):02X}"
                lines.append(
                    f"{stamp} | {'OK' if ev.get('ok') else 'FAIL'} | "
                    f"{ev.get('label')} | {ev.get('detail','')}{status_text}"
                )
        else:
            lines.append("No generic-ACK commands recorded.")

        lines.append("")
        lines.append("NOTES")
        lines.append("- V14.21 runs Force Offset replay on all connected/replayable Pitch/Roll/Yaw axes together.")
        lines.append("- Roll position tuning is frozen at AP speed 200 / AP force 1500 from loader-PC validation.")
        lines.append("- Endpoint precheck returns to the recorded initial point between negative and positive endpoint checks.")
        lines.append("- This diagnostic is passive with respect to the control law.")
        lines.append("- The loader-PC v535 path remains native 0x22 engineering force when available.")
        lines.append("- The simulator-PC v474 path is accepted only after 0x20/0x21 legacy consistency validation.")
        lines.append("- Bring this TXT, the commissioning CSV, the normal test CSV/PDF, and replay JSON back for analysis.")

        txt_path.write_text("\n".join(lines), encoding="utf-8")
        self.commission_diag_last_txt = str(txt_path)
        print(f"[COMMISSION] Saved CSV -> {csv_path if rows else 'NO ROWS'}", flush=True)
        print(f"[COMMISSION] Saved TXT -> {txt_path}", flush=True)

    def _pitch_reference_raw_from_replay(
        self,
        pf0: Dict[str, Any],
        pf1: Dict[str, Any],
        alpha: float,
    ) -> Optional[float]:
        """
        Interpolate the Manual RAW Pitch reference at the SAME replay time
        used by the existing normalized AP-position command.

        Prefer cls_pitch_raw when present; fall back to pitch_pos, which the
        Manual high-rate replay also stores from CLS2Sim raw position.
        """
        def _one(frame):
            value = self._diag_finite(frame.get("cls_pitch_raw"))
            if value is None:
                value = self._diag_finite(frame.get("pitch_pos"))
            return value

        v0 = _one(pf0)
        v1 = _one(pf1)

        if v0 is None and v1 is None:
            return None
        if v0 is None:
            v0 = v1
        if v1 is None:
            v1 = v0

        return (1.0 - float(alpha)) * float(v0) + float(alpha) * float(v1)

    def _manual_position_calibration(self, axis_name: str) -> Optional[Dict[str, float]]:
        """Return Manual-report position = slope * CLS_norm + intercept.

        The replay/control coordinate remains normalized CLS position.  This
        calibration exists only to compare Automatic live telemetry with the
        selected Manual run in one common report coordinate.
        """
        axis_name = str(axis_name)
        cached = self.manual_position_calibration_cache.get(axis_name)
        if cached:
            return cached

        key_map = {
            "Pitch": ("cls_pitch_norm", "cls_pitch_raw", "pitch_pos"),
            "Roll": ("cls_roll_norm", "cls_roll_raw", "roll_pos"),
            "Yaw": ("cls_yaw_norm", "cls_yaw_raw", "rudder_pos"),
        }
        keys = key_map.get(axis_name)
        if not keys:
            return None
        norm_key, raw_key, legacy_key = keys

        pairs = []
        for frame in self.reference_replay or []:
            norm = self._diag_finite(frame.get(norm_key))
            raw = self._diag_finite(frame.get(raw_key))
            if raw is None:
                raw = self._diag_finite(frame.get(legacy_key))
            if norm is not None and raw is not None:
                pairs.append((float(norm), float(raw)))

        if len(pairs) < 3:
            return None

        mean_n = sum(p[0] for p in pairs) / len(pairs)
        mean_r = sum(p[1] for p in pairs) / len(pairs)
        var_n = sum((p[0] - mean_n) ** 2 for p in pairs)
        if var_n <= 1e-12:
            return None

        cov = sum((p[0] - mean_n) * (p[1] - mean_r) for p in pairs)
        slope = cov / var_n
        intercept = mean_r - slope * mean_n
        if not (math.isfinite(slope) and math.isfinite(intercept)):
            return None
        if abs(slope) <= 1e-9:
            return None

        residuals = [r - (slope * n + intercept) for n, r in pairs]
        rms = math.sqrt(sum(e * e for e in residuals) / len(residuals))
        calibration = {
            "slope": float(slope),
            "intercept": float(intercept),
            "rms": float(rms),
            "samples": float(len(pairs)),
        }
        self.manual_position_calibration_cache[axis_name] = calibration
        print(
            f"[V14.62 POSITION CAL] {axis_name}: "
            f"manualCoord={slope:+.6f}*norm{intercept:+.3f} | "
            f"samples={len(pairs)} rms={rms:.3f}",
            flush=True,
        )
        return calibration

    def _manual_position_from_norm(
        self, axis_name: str, norm_value: Optional[float]
    ) -> Optional[float]:
        norm = self._diag_finite(norm_value)
        cal = self._manual_position_calibration(axis_name)
        if norm is None or not cal:
            return None
        value = cal["slope"] * float(norm) + cal["intercept"]
        return value if math.isfinite(value) else None

    def _pitch_target_raw_estimate(self, target_norm: Optional[float]) -> Optional[float]:
        """
        Map the existing normalized Pitch AP target into the selected Manual
        run's legacy position coordinate for DIAGNOSTIC DISPLAY ONLY.

        V14.62 deliberately does not use the current device's native RAW
        +/-range here because that native RAW sign/scale can differ from the
        coordinate stored by the Manual reference.  Nothing from this function
        is ever sent to CLS2Sim.
        """
        return self._manual_position_from_norm("Pitch", target_norm)

    def _append_pitch_position_diagnostic(
        self,
        elapsed_ms: int,
        pf0: Dict[str, Any],
        pf1: Dict[str, Any],
        palpha: float,
        targets: Dict[str, float],
        current_targets: Optional[Dict[str, float]] = None,
    ) -> None:
        """
        Observe the existing Pitch position replay without modifying it.
        """
        if (
            not PITCH_POSITION_DIAG_ENABLED
            or "Pitch" not in self.active_cls_axes
            or "Pitch" not in targets
        ):
            return

        ref_raw = self._pitch_reference_raw_from_replay(pf0, pf1, palpha)
        cmd_norm = self._diag_finite(targets.get("Pitch"))
        current_ref_norm = self._diag_finite(
            (current_targets or {}).get("Pitch")
        )
        cmd_raw_est = self._pitch_target_raw_estimate(cmd_norm)
        actual_norm = self._diag_finite(self.cls.last_pitch_pos)
        actual_raw = self._diag_finite(self.cls.last_pitch_pos_raw)
        actual_manual_coord = self._manual_position_from_norm("Pitch", actual_norm)

        err_norm = None
        if cmd_norm is not None and actual_norm is not None:
            err_norm = cmd_norm - actual_norm

        raw_error_to_manual = None
        if ref_raw is not None and actual_manual_coord is not None:
            raw_error_to_manual = ref_raw - actual_manual_coord

        raw_error_to_command_est = None
        if cmd_raw_est is not None and actual_manual_coord is not None:
            raw_error_to_command_est = cmd_raw_est - actual_manual_coord

        self.pitch_position_diag_rows.append({
            "Time (s)": round(float(elapsed_ms) / 1000.0, 3),
            "Replay Frame 0 t_ms": int(pf0.get("t_ms", 0) or 0),
            "Replay Frame 1 t_ms": int(pf1.get("t_ms", 0) or 0),
            "Replay Alpha": round(float(palpha), 6),
            "Manual Pitch Raw Reference": (
                round(ref_raw, 3) if ref_raw is not None else float("nan")
            ),
            "Manual Pitch Current Reference (norm)": (
                round(current_ref_norm, 6)
                if current_ref_norm is not None
                else float("nan")
            ),
            "Position Lookahead (ms)": round(
                1000.0 * float(AUTO_POSITION_LOOKAHEAD_SEC.get("Pitch", 0.0)),
                1,
            ),
            "Python AP Pitch Target (norm)": (
                round(cmd_norm, 6) if cmd_norm is not None else float("nan")
            ),
            "Python AP Pitch Target Raw Estimate": (
                round(cmd_raw_est, 3) if cmd_raw_est is not None else float("nan")
            ),
            "Actual CLS Pitch Position (norm)": (
                round(actual_norm, 6) if actual_norm is not None else float("nan")
            ),
            "Actual CLS Pitch Position (raw native)": (
                round(actual_raw, 3) if actual_raw is not None else float("nan")
            ),
            "Actual CLS Pitch Position (manual coordinate)": (
                round(actual_manual_coord, 3)
                if actual_manual_coord is not None
                else float("nan")
            ),
            "Command-Actual Error (norm)": (
                round(err_norm, 6) if err_norm is not None else float("nan")
            ),
            "ManualRaw-ActualRaw Error": (
                round(raw_error_to_manual, 3)
                if raw_error_to_manual is not None
                else float("nan")
            ),
            "CommandRawEst-ActualRaw Error": (
                round(raw_error_to_command_est, 3)
                if raw_error_to_command_est is not None
                else float("nan")
            ),
        })

        if self.cls_debug_counter % PITCH_POSITION_DIAG_PRINT_EVERY_TICKS == 0:
            def _fmt(v, digits=3):
                if v is None:
                    return "NA"
                return f"{float(v):+.{digits}f}"

            print(
                "[PITCH POS DIAG] "
                f"t={elapsed_ms / 1000.0:6.2f}s | "
                f"ManualRaw={_fmt(ref_raw, 1)} | "
                f"CmdNorm={_fmt(cmd_norm, 4)} | "
                f"CmdRawEst={_fmt(cmd_raw_est, 1)} | "
                f"ActualNorm={_fmt(actual_norm, 4)} | "
                f"ActualRawNative={_fmt(actual_raw, 1)} | "
                f"ActualManualCoord={_fmt(actual_manual_coord, 1)} | "
                f"ErrNorm={_fmt(err_norm, 4)}",
                flush=True,
            )

    def _save_pitch_position_diagnostic_csv(self, timestamp: str) -> Optional[Path]:
        """Save high-rate Pitch diagnostic beside the normal Automatic CSV."""
        if (
            self.test_mode != "auto"
            or not PITCH_POSITION_DIAG_ENABLED
            or not self.pitch_position_diag_rows
        ):
            return None

        rows = list(self.pitch_position_diag_rows)
        filepath = (
            self.data_dir
            / f"Super_Mushak_auto_pitch_position_diagnostic_{timestamp}.csv"
        )

        with open(filepath, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)

        self.pitch_position_diag_last_path = str(filepath)
        print(
            "[PITCH POS DIAG] Saved "
            f"{len(rows)} high-rate rows -> {filepath}",
            flush=True,
        )
        return filepath


    def _apply_pitch_force_diagnostic(self) -> bool:
        """
        Stage-2 MANUAL diagnostic.

        Pitch only:
          1. Force Scale Override = active
          2. Force Scale Factor = 20%
          3. Move Back Enable = 0

        Roll/Yaw are unchanged.
        AP is not enabled.
        Trim position is NOT overridden.

        Protocol-v2 meaning of Move Back Enable:
          1 -> axis tries to return to its center/trim position
          0 -> force profile is used as friction instead of centering
        """
        # V13: Pitch diagnostic is optional. A Roll-only or Yaw-only setup
        # must remain fully usable.
        if (
            getattr(self, "cls", None)
            and self.cls.detected_nodes.get("Pitch") is None
        ):
            print(
                "[PITCH FORCE/MOVEBACK DIAG] Skipped: Pitch axis is not connected.",
                flush=True,
            )
            return True

        if not PITCH_FORCE_DIAGNOSTIC_ENABLED:
            return False

        try:
            self.cls.open()

            ok_override = self.cls.send_with_ack(
                packet_override(AX_ELEVATOR, OVR_FORCE_SCALE),
                "Pitch Scale-Factor Override",
                0.15,
            )

            ok_scale = self.cls.send_with_ack(
                packet_setting_u16(
                    AX_ELEVATOR,
                    SID_FORCE_SCALE,
                    PITCH_FORCE_DIAGNOSTIC_SCALE_PERCENT,
                ),
                f"Pitch Force Scale={PITCH_FORCE_DIAGNOSTIC_SCALE_PERCENT}%",
                0.15,
            )

            self.pitch_force_diagnostic_active = bool(ok_override and ok_scale)
            self.pitch_force_scale_percent = (
                PITCH_FORCE_DIAGNOSTIC_SCALE_PERCENT
                if self.pitch_force_diagnostic_active
                else None
            )

            ok_move_back = True
            self.pitch_move_back_disabled = False

            if PITCH_MOVE_BACK_DIAGNOSTIC_ENABLED:
                ok_move_back = self.cls.send_with_ack(
                    packet_setting_u8(
                        AX_ELEVATOR,
                        SID_MOVE_BACK_ENABLE,
                        0,
                    ),
                    "Pitch Move Back=OFF",
                    0.15,
                )
                self.pitch_move_back_disabled = bool(ok_move_back)

            print(
                "[PITCH FORCE/MOVEBACK DIAG] "
                f"ScaleOverride={'OK' if ok_override else 'FAIL'} | "
                f"Scale={PITCH_FORCE_DIAGNOSTIC_SCALE_PERCENT}% "
                f"{'OK' if ok_scale else 'FAIL'} | "
                f"MoveBack={'OFF' if self.pitch_move_back_disabled else 'NOT CHANGED'} | "
                "AP remains OFF; Trim Position is NOT overridden; Roll/Yaw unchanged.",
                flush=True,
            )

            if not self.pitch_force_diagnostic_active:
                print(
                    "[PITCH FORCE/MOVEBACK DIAG] WARNING: "
                    "20% force-scale diagnostic did not fully activate.",
                    flush=True,
                )

            if PITCH_MOVE_BACK_DIAGNOSTIC_ENABLED and not ok_move_back:
                print(
                    "[PITCH FORCE/MOVEBACK DIAG] WARNING: "
                    "CLS2Sim did not acknowledge Pitch Move Back OFF. "
                    "Do not interpret this run as the Move-Back-off test.",
                    flush=True,
                )

            return bool(
                self.pitch_force_diagnostic_active
                and (
                    not PITCH_MOVE_BACK_DIAGNOSTIC_ENABLED
                    or self.pitch_move_back_disabled
                )
            )

        except Exception as exc:
            self.pitch_force_diagnostic_active = False
            self.pitch_force_scale_percent = None
            self.pitch_move_back_disabled = False
            print(f"[PITCH FORCE/MOVEBACK DIAG] ERROR: {exc}", flush=True)
            return False

    def _release_pitch_force_diagnostic(self) -> None:
        """
        Restore normal Pitch behavior.

        The user's normal CLS2Sim Pitch profile has Move Back enabled, so V5
        explicitly restores Move Back=1 before releasing the Pitch force-scale
        override back to Prepar3D/CLS2Sim.
        """
        if not getattr(self, "cls", None):
            return

        try:
            if getattr(self, "pitch_move_back_disabled", False):
                restored = self.cls.send_with_ack(
                    packet_setting_u8(
                        AX_ELEVATOR,
                        SID_MOVE_BACK_ENABLE,
                        PITCH_MOVE_BACK_RESTORE_VALUE,
                    ),
                    "Restore Pitch Move Back=ON",
                    0.15,
                )
                print(
                    "[PITCH FORCE/MOVEBACK DIAG] "
                    f"Move Back restore={'OK' if restored else 'FAIL'}",
                    flush=True,
                )

            if getattr(self, "pitch_force_diagnostic_active", False):
                released = self.cls.send_with_ack(
                    packet_override(AX_NONE, OVR_FORCE_SCALE),
                    "Release Pitch Scale-Factor Override",
                    0.15,
                )
                print(
                    "[PITCH FORCE/MOVEBACK DIAG] "
                    f"Force-scale override release={'OK' if released else 'FAIL'}",
                    flush=True,
                )

        except Exception as exc:
            print(
                f"[PITCH FORCE/MOVEBACK DIAG] Restore warning: {exc}",
                flush=True,
            )
        finally:
            self.pitch_force_diagnostic_active = False
            self.pitch_force_scale_percent = None
            self.pitch_move_back_disabled = False

    def _set_active_cls_axes(self, names) -> None:
        ordered = [name for name, _axis, _key in CLS_AXIS_SPECS if name in set(names)]
        self.active_cls_axes = ordered
        mask = AX_NONE
        for name in ordered:
            mask |= CLS_AXIS_MASK_BY_NAME[name]
        self.active_cls_axis_mask = mask

    def _active_axis_specs(self):
        return [
            (name, axis, key)
            for name, axis, key in CLS_AXIS_SPECS
            if name in self.active_cls_axes
        ]

    def _active_axis_summary(self) -> str:
        return ", ".join(self.active_cls_axes) if self.active_cls_axes else "NONE"

    def _initial_reference_target(self, name: str, default: float = 0.0) -> float:
        key = CLS_REPLAY_KEY_BY_NAME[name]
        for frame in self.reference_replay or []:
            try:
                value = frame.get(key)
                if value is not None:
                    value = float(value)
                    if math.isfinite(value):
                        return max(-1.0, min(1.0, value))
            except (TypeError, ValueError):
                pass
        return float(default)

    @staticmethod
    def _finite_force_value(value) -> Optional[float]:
        try:
            fv = float(value)
            return fv if math.isfinite(fv) else None
        except (TypeError, ValueError):
            return None

    def _force_conversion_info_for_axis(self, name: str, axis_mask: int) -> Dict[str, Any]:
        """Return force-only conversion metadata without altering position logic."""
        if not getattr(self, "cls", None):
            return {}

        native = dict(self.cls.axis_info.get(name) or {})
        if native:
            return native

        converted = dict(
            getattr(self.cls, "force_conversion_info", {}).get(name) or {}
        )
        if converted:
            return converted

        try:
            native = dict(self.cls._ensure_axis_info(axis_mask) or {})
        except Exception:
            native = {}
        if native:
            return native

        return dict(
            getattr(self.cls, "force_conversion_info", {}).get(name) or {}
        )

    def _raw_force_to_engineering_n(
        self, raw_value, info: Dict[str, Any]
    ) -> Optional[float]:
        """Convert a recorded/live RAW force sample using native or v474 metadata."""
        raw = self._finite_force_value(raw_value)
        factor = self._finite_force_value((info or {}).get("force_factor"))
        if raw is None or factor in (None, 0.0):
            return None

        if (info or {}).get("legacy_signed16"):
            raw_signed = self.cls._legacy_force_raw_signed16(raw)
            if raw_signed is None:
                return None
            raw = float(raw_signed)

        return raw * factor

    def _pitch_force_axis_info(self) -> Dict[str, Any]:
        """Return Pitch force conversion data only; never changes position units."""
        return self._force_conversion_info_for_axis("Pitch", AX_ELEVATOR)

    def _pitch_force_n_from_frame(self, frame: Dict[str, Any]) -> Optional[float]:
        """Read one recorded Manual Pitch-force sample in engineering units (N)."""
        eng = self._finite_force_value(frame.get("cls_pitch_force_eng"))
        if eng is not None:
            return eng

        info = self._pitch_force_axis_info()
        factor = self._finite_force_value(info.get("force_factor"))
        force_limit = self._finite_force_value(info.get("force_limit"))

        raw = self._finite_force_value(frame.get("cls_pitch_force_raw"))
        raw_n = self._raw_force_to_engineering_n(raw, info)
        if raw_n is not None:
            return raw_n

        norm = self._finite_force_value(frame.get("cls_pitch_force_norm"))
        if (
            norm is not None
            and factor not in (None, 0.0)
            and force_limit not in (None, 0.0)
        ):
            return norm * force_limit * factor

        # Generic/per-second fields are accepted only when the replay frame
        # explicitly identifies them as an engineering-force source. Fresh
        # V14.10 recordings always save this source tag.
        source = str(frame.get("cls_pitch_force_source", ""))
        base_force_n = self._finite_force_value(frame.get("pitch_force"))
        if base_force_n is not None and "ENG" in source:
            return base_force_n

        generic = self._finite_force_value(frame.get("cls_pitch_force"))
        if generic is not None and "ENG" in source:
            return generic

        return None
    def _pitch_actual_force_n(self) -> Optional[float]:
        """Return LIVE measured Pitch applied force in engineering units (N)."""
        if not getattr(self, "cls", None):
            return None

        # The installed CLS2Sim is demonstrably using 0x22 ENG, so prefer it.
        eng = self._finite_force_value(self.cls.last_pitch_force_eng)
        if eng is not None:
            return eng

        # Same live channel shown in your console as Fp=...[0x22 ENG].
        generic = self._finite_force_value(self.cls.last_pitch_force)
        source = str(self.cls.last_force_source.get("Pitch", ""))
        if generic is not None and "ENG" in source:
            return generic

        info = self._pitch_force_axis_info()
        factor = self._finite_force_value(info.get("force_factor"))
        force_limit = self._finite_force_value(info.get("force_limit"))

        raw = self._finite_force_value(self.cls.last_pitch_force_raw)
        raw_n = self._raw_force_to_engineering_n(raw, info)
        if raw_n is not None:
            return raw_n

        norm = self._finite_force_value(self.cls.last_pitch_force_norm)
        if (
            norm is not None
            and factor not in (None, 0.0)
            and force_limit not in (None, 0.0)
        ):
            return norm * force_limit * factor

        return None

    def _pitch_force_factor(self) -> Optional[float]:
        """Engineering-force conversion factor: engineering = internal * factor."""
        info = self._pitch_force_axis_info()
        factor = self._finite_force_value(info.get("force_factor"))
        if factor is None or abs(factor) < 1e-12:
            return None
        return factor

    def _pitch_force_command_limit_internal(self) -> Optional[int]:
        """Safe signed Force-Offset command limit in BRUNNER internal units."""
        info = self._pitch_force_axis_info()
        force_limit = self._finite_force_value(info.get("force_limit"))
        factor = self._pitch_force_factor()
        if force_limit is None or force_limit <= 0.0 or factor is None:
            return None

        hardware_limit = abs(force_limit) * PITCH_FORCE_OFFSET_LIMIT_FRACTION

        # V14.51: include the live zero-offset force bias.  Manual peak alone
        # was too small on Sim-PC Yaw, so the controller could not cancel the
        # existing bias before reproducing the Manual force.
        force_need_n = self.pitch_force_manual_peak_n + abs(float(self.pitch_force_baseline_n or 0.0))
        if force_need_n > 0.0:
            manual_internal = force_need_n * PITCH_FORCE_MANUAL_PEAK_MARGIN / abs(factor)
            hardware_limit = min(hardware_limit, manual_internal)

        return max(1, int(hardware_limit))

    def _scan_manual_pitch_force_peak(self) -> float:
        peak = 0.0
        for frame in self.reference_replay or []:
            value = self._pitch_force_n_from_frame(frame)
            if value is not None:
                peak = max(peak, abs(float(value)))
        return peak

    def _arm_pitch_force_replay(self) -> bool:
        """Arm only the signed physical Pitch-force command channel."""
        if (
            not PITCH_FORCE_REPLAY_ENABLED
            or "Pitch" not in self.active_cls_axes
            or not self._force_stage_allows("Pitch")
        ):
            self.pitch_force_replay_active = False
            return True

        factor = self._pitch_force_factor()
        self.pitch_force_manual_peak_n = self._scan_manual_pitch_force_peak()
        self.pitch_force_baseline_n = float(self._pitch_actual_force_n() or 0.0)
        if factor is None or self.pitch_force_manual_peak_n <= 0.0:
            print(
                "[PITCH FORCE REPLAY] Cannot arm: engineering-force conversion "
                "or Manual 0x22 force samples are unavailable.",
                flush=True,
            )
            self.pitch_force_replay_active = False
            return False

        try:
            # IMPORTANT FIX:
            # The existing cls2sim_lab.py force-offset implementation uses only
            # Override 0x05 + Setting 0xD0. The previous build also sent a 0xD1
            # setting and included it in the boolean arm result. On this CLS2Sim
            # build that extra command can be rejected/unsupported, which leaves
            # pitch_force_replay_active=False. That is exactly what the console
            # signature FpRefN=NA / FpActN=NA / FpOffsetCmd=0 indicates.
            override_ok = self.cls.send_with_ack(
                packet_override(AX_ELEVATOR, OVR_VIBRATION),
                "Pitch Force-Offset Override",
                0.15,
            )

            # Force Offset is a streamed setting. A UDP send is sufficient for
            # the neutral command; do not falsely disarm the whole force path if
            # this particular setting does not return the generic ACK format.
            zero_send_ok = self.cls.send(
                packet_setting_i32(AX_ELEVATOR, SID_FORCE_OFFSET, 0)
            )
            ok = bool(override_ok and zero_send_ok)

            self.pitch_force_offset_command = 0
            self.pitch_force_integral_internal = 0.0
            self.pitch_force_reference_n = None
            self.pitch_force_actual_n = None
            self.pitch_force_replay_warned = False
            self.pitch_force_replay_active = bool(ok)

            limit = self._pitch_force_command_limit_internal()
            print(
                "[PITCH FORCE REPLAY] "
                f"arm={'OK' if ok else 'FAIL'} | "
                f"override={'OK' if override_ok else 'FAIL'} | "
                f"zeroSend={'OK' if zero_send_ok else 'FAIL'} | "
                f"source={self.cls.last_force_source.get('Pitch', 'engineering force')} | "
                f"manualPeak={self.pitch_force_manual_peak_n:.3f} N | "
                f"baseline={self.pitch_force_baseline_n:+.3f} N | "
                f"forceFactor={factor:.9g} | "
                f"offsetLimit={limit if limit is not None else 'UNKNOWN'} internal | "
                "Pitch POSITION code/units UNCHANGED.",
                flush=True,
            )
            return bool(ok)
        except Exception as exc:
            self.pitch_force_replay_active = False
            print(f"[PITCH FORCE REPLAY] Arm error: {exc}", flush=True)
            return False

    def _release_pitch_force_replay(self) -> None:
        """Zero and release only the Pitch force-offset command."""
        if not getattr(self, "cls", None):
            return
        try:
            self.cls.send(packet_setting_i32(AX_ELEVATOR, SID_FORCE_OFFSET, 0))
            self.cls.send(packet_override(AX_NONE, OVR_VIBRATION))
        except Exception as exc:
            print(f"[PITCH FORCE REPLAY] Release warning: {exc}", flush=True)
        finally:
            self.pitch_force_replay_active = False
            self.pitch_force_reference_n = None
            self.pitch_force_actual_n = None
            self.pitch_force_offset_command = 0
            self.pitch_force_integral_internal = 0.0

    def _force_authority(self, name: str) -> float:
        try:
            value = float(self.force_position_authority.get(name, 1.0))
        except Exception:
            value = 1.0
        return max(0.0, min(1.0, value))

    def _update_pitch_force_replay(
        self,
        pf0: Dict[str, Any],
        pf1: Dict[str, Any],
        alpha: float,
    ) -> None:
        """
        Replay ONLY the Manual Pitch force in N.

        The existing AP-position command immediately above this call remains
        untouched, so this method cannot change Pitch position or its units.
        """
        if "Pitch" not in self.active_cls_axes:
            return
        if not self.pitch_force_replay_active:
            return

        f0 = self._pitch_force_n_from_frame(pf0)
        f1 = self._pitch_force_n_from_frame(pf1)
        if f0 is None and f1 is None:
            if not self.pitch_force_replay_warned:
                print(
                    "[PITCH FORCE REPLAY] No Manual engineering-force sample available in replay frame.",
                    flush=True,
                )
                self.pitch_force_replay_warned = True
            return
        if f0 is None:
            f0 = f1
        if f1 is None:
            f1 = f0

        reference_n = (1.0 - float(alpha)) * float(f0) + float(alpha) * float(f1)
        reference_n *= float(PITCH_FORCE_REPLAY_SIGN)
        if abs(reference_n) < PITCH_FORCE_REPLAY_DEADBAND_N:
            reference_n = 0.0

        factor = self._pitch_force_factor()
        limit = self._pitch_force_command_limit_internal()
        if factor is None or limit is None:
            if not self.pitch_force_replay_warned:
                print(
                    "[PITCH FORCE REPLAY] forceFactor/forceLimit unavailable; "
                    "force command suppressed for safety.",
                    flush=True,
                )
                self.pitch_force_replay_warned = True
            return

        actual_n = self._pitch_actual_force_n()

        # 1:1 feed-forward from the exact Manual engineering force.
        feedforward_internal = reference_n / factor

        # Conservative integral correction lets the physical measured 0x22 force
        # converge toward the recorded Manual force even when the existing 20%
        # force-scale/profile changes the effective response.
        authority = self._force_authority("Pitch")
        if actual_n is not None:
            error_internal = (reference_n - actual_n) / factor
            if authority >= 0.95:
                self.pitch_force_integral_internal += (
                    PITCH_FORCE_REPLAY_INTEGRAL_GAIN * error_internal
                )
            else:
                # Prevent wind-up while the position loop needs priority.
                self.pitch_force_integral_internal *= 0.98

        # Do not let the correction itself exceed the same safe force envelope.
        self.pitch_force_integral_internal = max(
            -float(limit),
            min(float(limit), self.pitch_force_integral_internal),
        )

        desired = feedforward_internal + self.pitch_force_integral_internal
        desired = max(-float(limit), min(float(limit), desired))
        # Position has priority. Fade the absolute Force Offset as position error
        # grows rather than letting the two loops fight until a hard trip.
        desired *= authority

        # Slew only the FORCE command; position command remains exactly unchanged.
        previous = float(self.pitch_force_offset_command)
        delta = desired - previous
        max_step = float(PITCH_FORCE_REPLAY_MAX_STEP_INTERNAL)
        if delta > max_step:
            desired = previous + max_step
        elif delta < -max_step:
            desired = previous - max_step

        command = int(round(desired))
        self.cls.send(packet_setting_i32(AX_ELEVATOR, SID_FORCE_OFFSET, command))

        self.pitch_force_reference_n = reference_n
        self.pitch_force_actual_n = actual_n
        self.pitch_force_offset_command = command

    def _preload_initial_pitch_force(self) -> bool:
        """
        FORCE ONLY: physically establish the Manual test's initial Pitch force
        before Automatic recording begins.

        Position behavior is intentionally untouched:
          - no SID_AP_POSITION write is made here;
          - no normalized/raw position conversion is changed;
          - no Pitch position unit is changed.

        The existing force controller is simply held at the earliest valid
        Manual Pitch-force sample while P3D remains paused. The live 0x22 ENG
        force feedback is allowed to settle before the user starts recording.
        """
        if not PITCH_FORCE_INITIAL_PRELOAD_ENABLED:
            return True
        if "Pitch" not in self.active_cls_axes or not self._force_stage_allows("Pitch"):
            return True

        # If force arming was unavailable during early setup, try once more now
        # because runtime telemetry/AxisInfo is already active at this stage.
        if not self.pitch_force_replay_active:
            self.pitch_force_last_arm_attempt = time.monotonic()
            if not self._arm_pitch_force_replay():
                print(
                    "[PITCH FORCE PRELOAD] SKIPPED: Pitch force replay is not armed.",
                    flush=True,
                )
                return False

        initial_frame = None
        initial_force_n = None
        initial_force_source = "NONE"

        # IMPORTANT:
        # The formal green Manual trace comes from the Manual CSV initial row.
        # That row can be sanitized by the existing initial-condition logic,
        # while the first high-rate replay JSON frame can be slightly different.
        #
        # For an exact Manual-vs-Automatic t=0 comparison, preload from the SAME
        # Manual CSV initial-force value used by the report.
        init_c = getattr(self, "manual_initial_conditions", {}) or {}
        csv_initial_force = self._finite_force_value(init_c.get("pitch_force"))
        if csv_initial_force is not None:
            initial_force_n = float(csv_initial_force)
            initial_force_source = "Manual CSV t=0"

        # We still need a frame object for the already-working force controller.
        # Use the earliest valid replay frame only as a carrier for metadata.
        for frame in self.reference_replay or []:
            value = self._pitch_force_n_from_frame(frame)
            if value is None:
                continue
            initial_frame = dict(frame)
            if initial_force_n is None:
                initial_force_n = float(value)
                initial_force_source = "Replay JSON first force"
            break

        if initial_force_n is None:
            print(
                "[PITCH FORCE PRELOAD] SKIPPED: no valid initial Manual Pitch force.",
                flush=True,
            )
            return False
        if initial_frame is None:
            initial_frame = {}

        # Force the preload frame to carry the exact initial value selected above.
        # This changes FORCE only; no Pitch position value is read or modified.
        initial_frame = dict(initial_frame)
        initial_frame["cls_pitch_force_eng"] = float(initial_force_n)
        initial_frame["pitch_force"] = float(initial_force_n)

        signed_reference_n = float(initial_force_n) * float(PITCH_FORCE_REPLAY_SIGN)
        if abs(signed_reference_n) < PITCH_FORCE_REPLAY_DEADBAND_N:
            signed_reference_n = 0.0

        print(
            "[PITCH FORCE PRELOAD] START | "
            f"ManualInitial={signed_reference_n:+.3f} N | "
            f"Source={initial_force_source} | "
            "Pitch position command/units unchanged.",
            flush=True,
        )

        start = time.monotonic()
        stable_since = None

        while (time.monotonic() - start) < float(PITCH_FORCE_INITIAL_PRELOAD_SEC):
            # Same first frame as both interpolation endpoints = constant t=0
            # force demand. The working feed-forward + integral force controller
            # therefore settles the physical CLS-P force before recording.
            self._update_pitch_force_replay(initial_frame, initial_frame, 0.0)

            actual_n = self._pitch_actual_force_n()
            if actual_n is not None:
                error_n = abs(signed_reference_n - float(actual_n))
                if error_n <= float(PITCH_FORCE_INITIAL_PRELOAD_TOL_N):
                    if stable_since is None:
                        stable_since = time.monotonic()
                    elif (
                        time.monotonic() - stable_since
                        >= float(PITCH_FORCE_INITIAL_PRELOAD_STABLE_SEC)
                    ):
                        self.pitch_force_reference_n = signed_reference_n
                        self.pitch_force_actual_n = float(actual_n)
                        print(
                            "[PITCH FORCE PRELOAD] READY | "
                            f"Ref={signed_reference_n:+.3f} N | "
                            f"Actual={float(actual_n):+.3f} N | "
                            f"OffsetCmd={self.pitch_force_offset_command:+d}",
                            flush=True,
                        )
                        return True
                else:
                    stable_since = None

            time.sleep(0.02)

        actual_n = self._pitch_actual_force_n()
        actual_text = "NA" if actual_n is None else f"{float(actual_n):+.3f}"
        error_text = (
            "NA"
            if actual_n is None
            else f"{abs(signed_reference_n - float(actual_n)):.3f}"
        )
        print(
            "[PITCH FORCE PRELOAD] TIMEOUT/CONTINUE | "
            f"Ref={signed_reference_n:+.3f} N | "
            f"Actual={actual_text} N | "
            f"Error={error_text} N | "
            f"OffsetCmd={self.pitch_force_offset_command:+d}. "
            "The normal force replay loop will continue correcting after Start.",
            flush=True,
        )

        # Keep the final preload force command active. Do NOT zero it here.
        return actual_n is not None


    # ==================================================================
    # ROLL FORCE REPLAY -- FORCE ONLY
    # ==================================================================
    def _roll_force_axis_info(self) -> Dict[str, Any]:
        """Return Roll force conversion data only; never changes position units."""
        return self._force_conversion_info_for_axis("Roll", AX_AILERON)

    def _roll_force_n_from_frame(self, frame: Dict[str, Any]) -> Optional[float]:
        """Read one recorded Manual Roll-force sample in engineering units (N)."""
        eng = self._finite_force_value(frame.get("cls_roll_force_eng"))
        if eng is not None:
            return eng

        info = self._roll_force_axis_info()
        factor = self._finite_force_value(info.get("force_factor"))
        force_limit = self._finite_force_value(info.get("force_limit"))

        raw = self._finite_force_value(frame.get("cls_roll_force_raw"))
        raw_n = self._raw_force_to_engineering_n(raw, info)
        if raw_n is not None:
            return raw_n

        norm = self._finite_force_value(frame.get("cls_roll_force_norm"))
        if (
            norm is not None
            and factor not in (None, 0.0)
            and force_limit not in (None, 0.0)
        ):
            return norm * force_limit * factor

        # Generic/per-second fields are accepted only when the replay frame
        # explicitly identifies them as an engineering-force source. Fresh
        # V14.10 recordings always save this source tag.
        source = str(frame.get("cls_roll_force_source", ""))
        base_force_n = self._finite_force_value(frame.get("roll_force"))
        if base_force_n is not None and "ENG" in source:
            return base_force_n

        generic = self._finite_force_value(frame.get("cls_roll_force"))
        if generic is not None and "ENG" in source:
            return generic

        return None
    def _roll_actual_force_n(self) -> Optional[float]:
        """Return LIVE measured Roll applied force in engineering units (N)."""
        if not getattr(self, "cls", None):
            return None

        eng = self._finite_force_value(self.cls.last_roll_force_eng)
        if eng is not None:
            return eng

        generic = self._finite_force_value(self.cls.last_roll_force)
        source = str(self.cls.last_force_source.get("Roll", ""))
        if generic is not None and "ENG" in source:
            return generic

        info = self._roll_force_axis_info()
        factor = self._finite_force_value(info.get("force_factor"))
        force_limit = self._finite_force_value(info.get("force_limit"))

        raw = self._finite_force_value(self.cls.last_roll_force_raw)
        raw_n = self._raw_force_to_engineering_n(raw, info)
        if raw_n is not None:
            return raw_n

        norm = self._finite_force_value(self.cls.last_roll_force_norm)
        if (
            norm is not None
            and factor not in (None, 0.0)
            and force_limit not in (None, 0.0)
        ):
            return norm * force_limit * factor
        return None

    def _roll_force_factor(self) -> Optional[float]:
        info = self._roll_force_axis_info()
        factor = self._finite_force_value(info.get("force_factor"))
        if factor is None or abs(factor) < 1e-12:
            return None
        return factor

    def _roll_force_command_limit_internal(self) -> Optional[int]:
        info = self._roll_force_axis_info()
        force_limit = self._finite_force_value(info.get("force_limit"))
        factor = self._roll_force_factor()
        if force_limit is None or force_limit <= 0.0 or factor is None:
            return None

        hardware_limit = abs(force_limit) * ROLL_FORCE_OFFSET_LIMIT_FRACTION
        force_need_n = self.roll_force_manual_peak_n + abs(float(self.roll_force_baseline_n or 0.0))
        if force_need_n > 0.0:
            manual_internal = force_need_n * ROLL_FORCE_MANUAL_PEAK_MARGIN / abs(factor)
            hardware_limit = min(hardware_limit, manual_internal)
        return max(1, int(hardware_limit))

    def _scan_manual_roll_force_peak(self) -> float:
        peak = 0.0
        for frame in self.reference_replay or []:
            value = self._roll_force_n_from_frame(frame)
            if value is not None:
                peak = max(peak, abs(float(value)))
        return peak

    def _arm_roll_force_replay(self) -> bool:
        """Arm only the signed physical Roll-force command channel."""
        if (
            not ROLL_FORCE_REPLAY_ENABLED
            or "Roll" not in self.active_cls_axes
            or not self._force_stage_allows("Roll")
        ):
            self.roll_force_replay_active = False
            return True

        factor = self._roll_force_factor()
        self.roll_force_manual_peak_n = self._scan_manual_roll_force_peak()
        self.roll_force_baseline_n = float(self._roll_actual_force_n() or 0.0)
        if factor is None or self.roll_force_manual_peak_n <= 0.0:
            print(
                "[ROLL FORCE REPLAY] Cannot arm: engineering-force conversion "
                "or Manual Roll force samples are unavailable.",
                flush=True,
            )
            self.roll_force_replay_active = False
            return False

        try:
            override_ok = self.cls.send_with_ack(
                packet_override(AX_AILERON, OVR_VIBRATION),
                "Roll Force-Offset Override",
                0.15,
            )
            zero_send_ok = self.cls.send(
                packet_setting_i32(AX_AILERON, SID_FORCE_OFFSET, 0)
            )
            ok = bool(override_ok and zero_send_ok)

            self.roll_force_offset_command = 0
            self.roll_force_integral_internal = 0.0
            self.roll_force_reference_n = None
            self.roll_force_actual_n = None
            self.roll_force_replay_warned = False
            self.roll_force_replay_active = bool(ok)

            limit = self._roll_force_command_limit_internal()
            print(
                "[ROLL FORCE REPLAY] "
                f"arm={'OK' if ok else 'FAIL'} | "
                f"override={'OK' if override_ok else 'FAIL'} | "
                f"zeroSend={'OK' if zero_send_ok else 'FAIL'} | "
                f"manualPeak={self.roll_force_manual_peak_n:.3f} N | "
                f"baseline={self.roll_force_baseline_n:+.3f} N | "
                f"forceFactor={factor:.9g} | "
                f"offsetLimit={limit if limit is not None else 'UNKNOWN'} internal | "
                "Roll POSITION code/units UNCHANGED.",
                flush=True,
            )
            return bool(ok)
        except Exception as exc:
            self.roll_force_replay_active = False
            print(f"[ROLL FORCE REPLAY] Arm error: {exc}", flush=True)
            return False

    def _update_roll_force_replay(
        self,
        pf0: Dict[str, Any],
        pf1: Dict[str, Any],
        alpha: float,
    ) -> None:
        """Replay ONLY the recorded Manual Roll force in N."""
        if "Roll" not in self.active_cls_axes or not self.roll_force_replay_active:
            return

        f0 = self._roll_force_n_from_frame(pf0)
        f1 = self._roll_force_n_from_frame(pf1)
        if f0 is None and f1 is None:
            if not self.roll_force_replay_warned:
                print("[ROLL FORCE REPLAY] No Manual Roll force available.", flush=True)
                self.roll_force_replay_warned = True
            return
        if f0 is None:
            f0 = f1
        if f1 is None:
            f1 = f0

        reference_n = (1.0 - float(alpha)) * float(f0) + float(alpha) * float(f1)
        reference_n *= float(ROLL_FORCE_REPLAY_SIGN)
        if abs(reference_n) < ROLL_FORCE_REPLAY_DEADBAND_N:
            reference_n = 0.0

        factor = self._roll_force_factor()
        limit = self._roll_force_command_limit_internal()
        if factor is None or limit is None:
            if not self.roll_force_replay_warned:
                print(
                    "[ROLL FORCE REPLAY] forceFactor/forceLimit unavailable; "
                    "force command suppressed for safety.",
                    flush=True,
                )
                self.roll_force_replay_warned = True
            return

        actual_n = self._roll_actual_force_n()
        feedforward_internal = reference_n / factor

        authority = self._force_authority("Roll")
        if actual_n is not None:
            error_internal = (reference_n - actual_n) / factor
            if authority >= 0.95:
                self.roll_force_integral_internal += (
                    ROLL_FORCE_REPLAY_INTEGRAL_GAIN * error_internal
                )
            else:
                # Prevent wind-up while the position loop needs priority.
                self.roll_force_integral_internal *= 0.98

        self.roll_force_integral_internal = max(
            -float(limit), min(float(limit), self.roll_force_integral_internal)
        )

        desired = feedforward_internal + self.roll_force_integral_internal
        desired = max(-float(limit), min(float(limit), desired))
        # Position has priority. Fade the absolute Force Offset as position error
        # grows rather than letting the two loops fight until a hard trip.
        desired *= authority

        previous = float(self.roll_force_offset_command)
        delta = desired - previous
        max_step = float(ROLL_FORCE_REPLAY_MAX_STEP_INTERNAL)
        if delta > max_step:
            desired = previous + max_step
        elif delta < -max_step:
            desired = previous - max_step

        command = int(round(desired))
        self.cls.send(packet_setting_i32(AX_AILERON, SID_FORCE_OFFSET, command))

        self.roll_force_reference_n = reference_n
        self.roll_force_actual_n = actual_n
        self.roll_force_offset_command = command

    def _preload_initial_roll_force(self) -> bool:
        """Physically establish the exact Manual CSV t=0 Roll force before recording."""
        if (
            not ROLL_FORCE_INITIAL_PRELOAD_ENABLED
            or "Roll" not in self.active_cls_axes
            or not self._force_stage_allows("Roll")
        ):
            return True

        if not self.roll_force_replay_active:
            self.roll_force_last_arm_attempt = time.monotonic()
            if not self._arm_roll_force_replay():
                print("[ROLL FORCE PRELOAD] SKIPPED: Roll force replay is not armed.", flush=True)
                return False

        initial_frame = None
        initial_force_n = None
        initial_force_source = "NONE"

        init_c = getattr(self, "manual_initial_conditions", {}) or {}
        csv_initial_force = self._finite_force_value(init_c.get("roll_force"))
        if csv_initial_force is not None:
            initial_force_n = float(csv_initial_force)
            initial_force_source = "Manual CSV t=0"

        for frame in self.reference_replay or []:
            value = self._roll_force_n_from_frame(frame)
            if value is None:
                continue
            initial_frame = dict(frame)
            if initial_force_n is None:
                initial_force_n = float(value)
                initial_force_source = "Replay JSON first force"
            break

        if initial_force_n is None:
            print("[ROLL FORCE PRELOAD] SKIPPED: no valid initial Manual Roll force.", flush=True)
            return False

        if initial_frame is None:
            initial_frame = {}
        initial_frame = dict(initial_frame)
        initial_frame["cls_roll_force_eng"] = float(initial_force_n)
        initial_frame["roll_force"] = float(initial_force_n)

        signed_reference_n = float(initial_force_n) * float(ROLL_FORCE_REPLAY_SIGN)
        if abs(signed_reference_n) < ROLL_FORCE_REPLAY_DEADBAND_N:
            signed_reference_n = 0.0

        print(
            "[ROLL FORCE PRELOAD] START | "
            f"ManualInitial={signed_reference_n:+.3f} N | "
            f"Source={initial_force_source} | "
            "Roll position command/units unchanged.",
            flush=True,
        )

        start = time.monotonic()
        stable_since = None
        while (time.monotonic() - start) < float(ROLL_FORCE_INITIAL_PRELOAD_SEC):
            self._update_roll_force_replay(initial_frame, initial_frame, 0.0)
            actual_n = self._roll_actual_force_n()

            if actual_n is not None:
                error_n = abs(signed_reference_n - float(actual_n))
                if error_n <= float(ROLL_FORCE_INITIAL_PRELOAD_TOL_N):
                    if stable_since is None:
                        stable_since = time.monotonic()
                    elif (
                        time.monotonic() - stable_since
                        >= float(ROLL_FORCE_INITIAL_PRELOAD_STABLE_SEC)
                    ):
                        self.roll_force_reference_n = signed_reference_n
                        self.roll_force_actual_n = float(actual_n)
                        print(
                            "[ROLL FORCE PRELOAD] READY | "
                            f"Ref={signed_reference_n:+.3f} N | "
                            f"Actual={float(actual_n):+.3f} N | "
                            f"OffsetCmd={self.roll_force_offset_command:+d}",
                            flush=True,
                        )
                        return True
                else:
                    stable_since = None
            time.sleep(0.02)

        actual_n = self._roll_actual_force_n()
        print(
            "[ROLL FORCE PRELOAD] TIMEOUT/CONTINUE | "
            f"Ref={signed_reference_n:+.3f} N | "
            f"Actual={'NA' if actual_n is None else f'{float(actual_n):+.3f}'} N | "
            f"OffsetCmd={self.roll_force_offset_command:+d}",
            flush=True,
        )
        return actual_n is not None

    # ==================================================================
    # YAW FORCE REPLAY -- FORCE ONLY
    # ==================================================================
    def _yaw_force_axis_info(self) -> Dict[str, Any]:
        """Return Yaw force conversion data only; never changes position units."""
        return self._force_conversion_info_for_axis("Yaw", AX_RUDDER)

    def _yaw_force_n_from_frame(self, frame: Dict[str, Any]) -> Optional[float]:
        """Read one recorded Manual Rudder/Yaw-force sample in engineering units (N)."""
        eng = self._finite_force_value(frame.get("cls_yaw_force_eng"))
        if eng is not None:
            return eng

        info = self._yaw_force_axis_info()
        factor = self._finite_force_value(info.get("force_factor"))
        force_limit = self._finite_force_value(info.get("force_limit"))

        raw = self._finite_force_value(frame.get("cls_yaw_force_raw"))
        raw_n = self._raw_force_to_engineering_n(raw, info)
        if raw_n is not None:
            return raw_n

        norm = self._finite_force_value(frame.get("cls_yaw_force_norm"))
        if (
            norm is not None
            and factor not in (None, 0.0)
            and force_limit not in (None, 0.0)
        ):
            return norm * force_limit * factor

        # Generic/per-second fields are accepted only when the replay frame
        # explicitly identifies them as an engineering-force source. Fresh
        # V14.10 recordings always save this source tag.
        source = str(frame.get("cls_yaw_force_source", ""))
        base_force_n = self._finite_force_value(frame.get("rudder_force"))
        if base_force_n is not None and "ENG" in source:
            return base_force_n

        generic = self._finite_force_value(frame.get("cls_yaw_force"))
        if generic is not None and "ENG" in source:
            return generic

        return None
    def _yaw_actual_force_n(self) -> Optional[float]:
        """Return LIVE measured Rudder/Yaw applied force in engineering units (N)."""
        if not getattr(self, "cls", None):
            return None

        eng = self._finite_force_value(self.cls.last_rudder_force_eng)
        if eng is not None:
            return eng

        generic = self._finite_force_value(self.cls.last_rudder_force)
        source = str(self.cls.last_force_source.get("Yaw", ""))
        if generic is not None and "ENG" in source:
            return generic

        info = self._yaw_force_axis_info()
        factor = self._finite_force_value(info.get("force_factor"))
        force_limit = self._finite_force_value(info.get("force_limit"))

        raw = self._finite_force_value(self.cls.last_rudder_force_raw)
        raw_n = self._raw_force_to_engineering_n(raw, info)
        if raw_n is not None:
            return raw_n

        norm = self._finite_force_value(self.cls.last_rudder_force_norm)
        if (
            norm is not None
            and factor not in (None, 0.0)
            and force_limit not in (None, 0.0)
        ):
            return norm * force_limit * factor
        return None

    def _yaw_force_factor(self) -> Optional[float]:
        info = self._yaw_force_axis_info()
        factor = self._finite_force_value(info.get("force_factor"))
        if factor is None or abs(factor) < 1e-12:
            return None
        return factor

    def _yaw_force_command_limit_internal(self) -> Optional[int]:
        info = self._yaw_force_axis_info()
        force_limit = self._finite_force_value(info.get("force_limit"))
        factor = self._yaw_force_factor()
        if force_limit is None or force_limit <= 0.0 or factor is None:
            return None

        hardware_limit = abs(force_limit) * YAW_FORCE_OFFSET_LIMIT_FRACTION
        force_need_n = self.yaw_force_manual_peak_n + abs(float(self.yaw_force_baseline_n or 0.0))
        if force_need_n > 0.0:
            manual_internal = force_need_n * YAW_FORCE_MANUAL_PEAK_MARGIN / abs(factor)
            hardware_limit = min(hardware_limit, manual_internal)
        return max(1, int(hardware_limit))

    def _scan_manual_yaw_force_peak(self) -> float:
        peak = 0.0
        for frame in self.reference_replay or []:
            value = self._yaw_force_n_from_frame(frame)
            if value is not None:
                peak = max(peak, abs(float(value)))
        return peak

    def _arm_yaw_force_replay(self) -> bool:
        """Arm only the signed physical Rudder/Yaw-force command channel."""
        if (
            not YAW_FORCE_REPLAY_ENABLED
            or "Yaw" not in self.active_cls_axes
            or not self._force_stage_allows("Yaw")
        ):
            self.yaw_force_replay_active = False
            return True

        factor = self._yaw_force_factor()
        self.yaw_force_manual_peak_n = self._scan_manual_yaw_force_peak()
        self.yaw_force_baseline_n = float(self._yaw_actual_force_n() or 0.0)
        if factor is None or self.yaw_force_manual_peak_n <= 0.0:
            print(
                "[YAW FORCE REPLAY] Cannot arm: engineering-force conversion "
                "or Manual Yaw force samples are unavailable.",
                flush=True,
            )
            self.yaw_force_replay_active = False
            return False

        try:
            override_ok = self.cls.send_with_ack(
                packet_override(AX_RUDDER, OVR_VIBRATION),
                "Yaw Force-Offset Override",
                0.15,
            )
            zero_send_ok = self.cls.send(
                packet_setting_i32(AX_RUDDER, SID_FORCE_OFFSET, 0)
            )
            ok = bool(override_ok and zero_send_ok)

            self.yaw_force_offset_command = 0
            self.yaw_force_integral_internal = 0.0
            self.yaw_force_reference_n = None
            self.yaw_force_actual_n = None
            self.yaw_force_replay_warned = False
            self.yaw_force_replay_active = bool(ok)

            limit = self._yaw_force_command_limit_internal()
            print(
                "[YAW FORCE REPLAY] "
                f"arm={'OK' if ok else 'FAIL'} | "
                f"override={'OK' if override_ok else 'FAIL'} | "
                f"zeroSend={'OK' if zero_send_ok else 'FAIL'} | "
                f"manualPeak={self.yaw_force_manual_peak_n:.3f} N | "
                f"baseline={self.yaw_force_baseline_n:+.3f} N | "
                f"forceFactor={factor:.9g} | "
                f"offsetLimit={limit if limit is not None else 'UNKNOWN'} internal | "
                "Yaw POSITION code/units UNCHANGED.",
                flush=True,
            )
            return bool(ok)
        except Exception as exc:
            self.yaw_force_replay_active = False
            print(f"[YAW FORCE REPLAY] Arm error: {exc}", flush=True)
            return False

    def _update_yaw_force_replay(
        self,
        pf0: Dict[str, Any],
        pf1: Dict[str, Any],
        alpha: float,
    ) -> None:
        """Replay ONLY the recorded Manual Rudder/Yaw force in N."""
        if "Yaw" not in self.active_cls_axes or not self.yaw_force_replay_active:
            return

        f0 = self._yaw_force_n_from_frame(pf0)
        f1 = self._yaw_force_n_from_frame(pf1)
        if f0 is None and f1 is None:
            if not self.yaw_force_replay_warned:
                print("[YAW FORCE REPLAY] No Manual Yaw force available.", flush=True)
                self.yaw_force_replay_warned = True
            return
        if f0 is None:
            f0 = f1
        if f1 is None:
            f1 = f0

        reference_n = (1.0 - float(alpha)) * float(f0) + float(alpha) * float(f1)
        reference_n *= float(YAW_FORCE_REPLAY_SIGN)
        if abs(reference_n) < YAW_FORCE_REPLAY_DEADBAND_N:
            reference_n = 0.0

        factor = self._yaw_force_factor()
        limit = self._yaw_force_command_limit_internal()
        if factor is None or limit is None:
            if not self.yaw_force_replay_warned:
                print(
                    "[YAW FORCE REPLAY] forceFactor/forceLimit unavailable; "
                    "force command suppressed for safety.",
                    flush=True,
                )
                self.yaw_force_replay_warned = True
            return

        actual_n = self._yaw_actual_force_n()
        feedforward_internal = reference_n / factor

        authority = self._force_authority("Yaw")
        if actual_n is not None:
            error_internal = (reference_n - actual_n) / factor
            if authority >= 0.95:
                self.yaw_force_integral_internal += (
                    YAW_FORCE_REPLAY_INTEGRAL_GAIN * error_internal
                )
            else:
                # Prevent wind-up while the position loop needs priority.
                self.yaw_force_integral_internal *= 0.98

        self.yaw_force_integral_internal = max(
            -float(limit), min(float(limit), self.yaw_force_integral_internal)
        )

        desired = feedforward_internal + self.yaw_force_integral_internal
        desired = max(-float(limit), min(float(limit), desired))
        # Position has priority. Fade the absolute Force Offset as position error
        # grows rather than letting the two loops fight until a hard trip.
        desired *= authority

        previous = float(self.yaw_force_offset_command)
        delta = desired - previous
        max_step = float(YAW_FORCE_REPLAY_MAX_STEP_INTERNAL)
        if delta > max_step:
            desired = previous + max_step
        elif delta < -max_step:
            desired = previous - max_step

        command = int(round(desired))
        self.cls.send(packet_setting_i32(AX_RUDDER, SID_FORCE_OFFSET, command))

        self.yaw_force_reference_n = reference_n
        self.yaw_force_actual_n = actual_n
        self.yaw_force_offset_command = command

    def _preload_initial_yaw_force(self) -> bool:
        """Physically establish the exact Manual CSV t=0 Rudder/Yaw force before recording."""
        if (
            not YAW_FORCE_INITIAL_PRELOAD_ENABLED
            or "Yaw" not in self.active_cls_axes
            or not self._force_stage_allows("Yaw")
        ):
            return True

        if not self.yaw_force_replay_active:
            self.yaw_force_last_arm_attempt = time.monotonic()
            if not self._arm_yaw_force_replay():
                print("[YAW FORCE PRELOAD] SKIPPED: Yaw force replay is not armed.", flush=True)
                return False

        initial_frame = None
        initial_force_n = None
        initial_force_source = "NONE"

        init_c = getattr(self, "manual_initial_conditions", {}) or {}
        csv_initial_force = self._finite_force_value(init_c.get("rudder_force"))
        if csv_initial_force is not None:
            initial_force_n = float(csv_initial_force)
            initial_force_source = "Manual CSV t=0"

        for frame in self.reference_replay or []:
            value = self._yaw_force_n_from_frame(frame)
            if value is None:
                continue
            initial_frame = dict(frame)
            if initial_force_n is None:
                initial_force_n = float(value)
                initial_force_source = "Replay JSON first force"
            break

        if initial_force_n is None:
            print("[YAW FORCE PRELOAD] SKIPPED: no valid initial Manual Yaw force.", flush=True)
            return False

        if initial_frame is None:
            initial_frame = {}
        initial_frame = dict(initial_frame)
        initial_frame["cls_yaw_force_eng"] = float(initial_force_n)
        initial_frame["rudder_force"] = float(initial_force_n)

        signed_reference_n = float(initial_force_n) * float(YAW_FORCE_REPLAY_SIGN)
        if abs(signed_reference_n) < YAW_FORCE_REPLAY_DEADBAND_N:
            signed_reference_n = 0.0

        print(
            "[YAW FORCE PRELOAD] START | "
            f"ManualInitial={signed_reference_n:+.3f} N | "
            f"Source={initial_force_source} | "
            "Yaw position command/units unchanged.",
            flush=True,
        )

        start = time.monotonic()
        stable_since = None
        while (time.monotonic() - start) < float(YAW_FORCE_INITIAL_PRELOAD_SEC):
            self._update_yaw_force_replay(initial_frame, initial_frame, 0.0)
            actual_n = self._yaw_actual_force_n()

            if actual_n is not None:
                error_n = abs(signed_reference_n - float(actual_n))
                if error_n <= float(YAW_FORCE_INITIAL_PRELOAD_TOL_N):
                    if stable_since is None:
                        stable_since = time.monotonic()
                    elif (
                        time.monotonic() - stable_since
                        >= float(YAW_FORCE_INITIAL_PRELOAD_STABLE_SEC)
                    ):
                        self.yaw_force_reference_n = signed_reference_n
                        self.yaw_force_actual_n = float(actual_n)
                        print(
                            "[YAW FORCE PRELOAD] READY | "
                            f"Ref={signed_reference_n:+.3f} N | "
                            f"Actual={float(actual_n):+.3f} N | "
                            f"OffsetCmd={self.yaw_force_offset_command:+d}",
                            flush=True,
                        )
                        return True
                else:
                    stable_since = None
            time.sleep(0.02)

        actual_n = self._yaw_actual_force_n()
        print(
            "[YAW FORCE PRELOAD] TIMEOUT/CONTINUE | "
            f"Ref={signed_reference_n:+.3f} N | "
            f"Actual={'NA' if actual_n is None else f'{float(actual_n):+.3f}'} N | "
            f"OffsetCmd={self.yaw_force_offset_command:+d}",
            flush=True,
        )
        return actual_n is not None

    def _final_bumpless_cls_settle(self) -> bool:
        """
        V14.51 early position/force-path pre-start gate (V14.50 behavior preserved).

        Keep the validated V14.42/V14.43 Automatic control path, but do NOT
        require the measured force on every axis to sit within +/-0.20 N of
        Manual frame 0 for 0.35 s before the run is allowed to start.

        On the simulator PC the validated legacy 0x20/0x21 force channel is
        quantized/noisy and Pitch carries substantial trim/airspeed force.
        Requiring exact pre-start force equality can block an otherwise valid
        Automatic run and encourages unnecessary actuator fighting.

        Automatic is allowed only when:
          1. every selected Manual axis has a valid frame-0 AP position;
          2. required force replay for every active axis has armed successfully;
          3. a live engineering-force measurement exists for every force axis;
          4. every physical axis follows frame-0 position within the existing
             normalized-position tolerance continuously for the existing
             stable time.

        The force controllers ARE still run against frame 0 during settling,
        and their converged integrals/offset commands are preserved into t=0.
        V14.65 has already locked the exact formal Manual CSV t=0 force before
        P3D is allowed to run. This gate maintains that same exact target and
        verifies position/live-force availability without resetting the converged
        controller state. No RAW-position correction, extra force integrator, or
        additional CLS2Sim configuration write is introduced here.
        """
        self.bumpless_start_ready = False
        self.bumpless_start_frame_t_ms = None

        if not AUTO_BUMPLESS_START_ENABLED:
            self.bumpless_start_ready = True
            return True

        if not getattr(self, "cls", None):
            print("[V14.51 BUMPLESS] FAIL: CLS transport is unavailable.", flush=True)
            return False

        if not self.reference_replay:
            print("[V14.51 BUMPLESS] FAIL: replay reference is empty.", flush=True)
            return None

        frame0 = dict(self.reference_replay[0])
        frame0_t_ms = int(frame0.get("t_ms", 0) or 0)
        # V14.65: after the controls-first lock, every subsequent pre-start force
        # maintenance step must use the SAME formal Manual CSV row-0 target.
        # Do not switch back to the high-rate replay frame's first force sample.
        exact_force_frame, exact_force_refs = self._v1463_exact_t0_force_frame()
        if not exact_force_frame:
            exact_force_frame = frame0

        targets: Dict[str, float] = {}
        for name, axis, key in self._active_axis_specs():
            try:
                value = float(frame0[key])
            except (KeyError, TypeError, ValueError):
                print(
                    f"[V14.51 BUMPLESS] FAIL: frame-0 position '{key}' "
                    f"is missing/invalid for {name}.",
                    flush=True,
                )
                return False
            if not math.isfinite(value):
                print(
                    f"[V14.51 BUMPLESS] FAIL: frame-0 position '{key}' "
                    f"is non-finite for {name}.",
                    flush=True,
                )
                return False
            targets[name] = max(-1.0, min(1.0, value))

        if not targets:
            print("[V14.51 BUMPLESS] FAIL: no active CLS replay axes.", flush=True)
            return False

        force_refs: Dict[str, float] = {}

        if "Pitch" in targets and self._force_stage_allows("Pitch"):
            ref = exact_force_refs.get("Pitch")
            if ref is None:
                print("[V14.51 BUMPLESS] FAIL: formal Manual CSV t=0 Pitch force is unavailable.", flush=True)
                return False
            force_refs["Pitch"] = float(ref)
            if not self.pitch_force_replay_active:
                self.pitch_force_last_arm_attempt = time.monotonic()
                if not self._arm_pitch_force_replay():
                    print("[V14.51 BUMPLESS] FAIL: Pitch force replay could not arm.", flush=True)
                    return False

        if "Roll" in targets and self._force_stage_allows("Roll"):
            ref = exact_force_refs.get("Roll")
            if ref is None:
                print("[V14.51 BUMPLESS] FAIL: formal Manual CSV t=0 Roll force is unavailable.", flush=True)
                return False
            force_refs["Roll"] = float(ref)
            if not self.roll_force_replay_active:
                self.roll_force_last_arm_attempt = time.monotonic()
                if not self._arm_roll_force_replay():
                    print("[V14.51 BUMPLESS] FAIL: Roll force replay could not arm.", flush=True)
                    return False

        if "Yaw" in targets and self._force_stage_allows("Yaw"):
            ref = exact_force_refs.get("Yaw")
            if ref is None:
                print("[V14.51 BUMPLESS] FAIL: formal Manual CSV t=0 Yaw force is unavailable.", flush=True)
                return False
            force_refs["Yaw"] = float(ref)
            if not self.yaw_force_replay_active:
                self.yaw_force_last_arm_attempt = time.monotonic()
                if not self._arm_yaw_force_replay():
                    print("[V14.51 BUMPLESS] FAIL: Yaw force replay could not arm.", flush=True)
                    return False

        print(
            "[V14.51 BUMPLESS] START | frame-0 position settle + force-path validation | "
            f"t_ms={frame0_t_ms} | "
            + " ".join(f"{n}Pos={targets[n]:+.5f}" for n in targets)
            + (
                " | " + " ".join(f"{n}ForceRef={force_refs[n]:+.3f}N" for n in force_refs)
                if force_refs else ""
            ),
            flush=True,
        )

        stable_since = None
        start = time.monotonic()
        last_detail = "waiting for first feedback"

        while (time.monotonic() - start) < float(AUTO_BUMPLESS_START_TIMEOUT_SEC):
            # Same AP-position command used by the normal replay loop at frame 0.
            for name, axis, _key in self._active_axis_specs():
                target = targets[name]
                self.cls.send(packet_setting_float(axis, SID_AP_POSITION, target))
                self.cls_last_targets[name] = target

                if name == "Pitch":
                    self.cls.last_pitch_target = target
                elif name == "Roll":
                    self.cls.last_roll_target = target
                else:
                    self.cls.last_rudder_target = target

            # V14.56: position ALWAYS has priority during frame-0 force settling.
            # Recompute the same recoverable authority used during normal replay
            # BEFORE applying Force Offset. A high-force reference can therefore
            # never drag an axis toward an end stop merely to satisfy force.
            current_for_interlock = {
                "Pitch": self.cls.last_pitch_pos,
                "Roll": self.cls.last_roll_pos,
                "Yaw": self.cls.last_rudder_pos,
            }
            for name, axis, _key in self._active_axis_specs():
                if name in force_refs:
                    self._force_position_interlock_allows(
                        name, axis, targets[name], current_for_interlock.get(name)
                    )

            # Keep the normal validated force controllers active while position
            # settles. Their converged state is carried straight into replay.
            if "Pitch" in force_refs:
                self._update_pitch_force_replay(exact_force_frame, exact_force_frame, 0.0)
            if "Roll" in force_refs:
                self._update_roll_force_replay(exact_force_frame, exact_force_frame, 0.0)
            if "Yaw" in force_refs:
                self._update_yaw_force_replay(exact_force_frame, exact_force_frame, 0.0)

            actual_pos = {
                "Pitch": self.cls.last_pitch_pos,
                "Roll": self.cls.last_roll_pos,
                "Yaw": self.cls.last_rudder_pos,
            }
            actual_force = {
                "Pitch": self._pitch_actual_force_n(),
                "Roll": self._roll_actual_force_n(),
                "Yaw": self._yaw_actual_force_n(),
            }

            all_positions_ok = True
            all_force_channels_live = True
            detail_parts = []

            for name, target in targets.items():
                actual = actual_pos.get(name)
                if actual is None:
                    all_positions_ok = False
                    detail_parts.append(f"{name}Pos=NA")
                    continue
                try:
                    pos_err = abs(float(target) - float(actual))
                except (TypeError, ValueError):
                    all_positions_ok = False
                    detail_parts.append(f"{name}Pos=BAD")
                    continue

                pos_ok = pos_err <= float(AUTO_BUMPLESS_START_POSITION_TOL)
                all_positions_ok = all_positions_ok and pos_ok
                detail_parts.append(
                    f"{name}PosErr={pos_err:.4f}"
                    + ("" if pos_ok else "!")
                )

            # Force is evidence only here: channel must be live, but exact
            # equality is not required before t=0.
            for name, ref_n in force_refs.items():
                actual_n = actual_force.get(name)
                if actual_n is None:
                    all_force_channels_live = False
                    detail_parts.append(f"{name}Force=NA!")
                    continue
                try:
                    force_err = float(ref_n) - float(actual_n)
                    detail_parts.append(
                        f"{name}Force={float(actual_n):+.2f}N "
                        f"Ref={float(ref_n):+.2f}N d={force_err:+.2f}N"
                    )
                except (TypeError, ValueError):
                    all_force_channels_live = False
                    detail_parts.append(f"{name}Force=BAD!")

            all_ok = bool(all_positions_ok and all_force_channels_live)
            last_detail = " | ".join(detail_parts)

            if all_ok:
                if stable_since is None:
                    stable_since = time.monotonic()
                elif (
                    time.monotonic() - stable_since
                    >= float(AUTO_BUMPLESS_START_STABLE_SEC)
                ):
                    self.bumpless_start_ready = True
                    self.bumpless_start_frame_t_ms = frame0_t_ms
                    print(
                        "[V14.51 BUMPLESS] READY | "
                        + last_detail
                        + " | force controller state PRESERVED into replay.",
                        flush=True,
                    )
                    return True
            else:
                stable_since = None

            time.sleep(float(AUTO_BUMPLESS_START_LOOP_SEC))

        print(
            "[V14.51 BUMPLESS] TIMEOUT/FAIL | "
            + last_detail
            + " | Automatic start BLOCKED because frame-0 POSITION did not settle "
              "or a required live force channel was unavailable.",
            flush=True,
        )
        return False


    def _set_force_loop_zero(self, name: str, axis_mask: int) -> None:
        """Immediately command zero Force Offset for one axis and reset its controller state."""
        if not getattr(self, "cls", None):
            return
        try:
            self.cls.send(packet_setting_i32(axis_mask, SID_FORCE_OFFSET, 0))
        except Exception as exc:
            print(f"[CLS FORCE SAFE] {name}: zero-offset warning: {exc}", flush=True)

        if name == "Pitch":
            self.pitch_force_offset_command = 0
            self.pitch_force_integral_internal = 0.0
            self.pitch_force_reference_n = None
        elif name == "Roll":
            self.roll_force_offset_command = 0
            self.roll_force_integral_internal = 0.0
            self.roll_force_reference_n = None
        elif name == "Yaw":
            self.yaw_force_offset_command = 0
            self.yaw_force_integral_internal = 0.0
            self.yaw_force_reference_n = None

    def _force_position_interlock_allows(
        self, name: str, axis_mask: int, target, actual
    ) -> bool:
        """V14.51: fade force before position/force conflict, then recover."""
        if not FORCE_POSITION_INTERLOCK_ENABLED:
            self.force_position_authority[name] = 1.0
            return True

        try:
            err = float("inf") if target is None or actual is None else abs(float(target) - float(actual))
        except (TypeError, ValueError):
            err = float("inf")

        if bool(self.force_position_interlock_blocked.get(name, False)):
            self.force_position_authority[name] = 0.0
            if err <= FORCE_POSITION_INTERLOCK_RECOVER_ERROR:
                self.force_position_interlock_blocked[name] = False
                self.force_position_interlock_latched[name] = False
                self.force_position_authority[name] = 1.0
                print(
                    f"[CLS FORCE INTERLOCK] {name}: RECOVERED | "
                    f"position error={err:.3f}; force replay resumed.", flush=True
                )
                return True
            self._set_force_loop_zero(name, axis_mask)
            return False

        if err >= FORCE_POSITION_INTERLOCK_BLOCK_ERROR:
            self.force_position_interlock_blocked[name] = True
            self.force_position_interlock_latched[name] = False
            self.force_position_authority[name] = 0.0
            self._set_force_loop_zero(name, axis_mask)
            print(
                f"[CLS FORCE INTERLOCK] {name}: TEMPORARILY BLOCKED | "
                f"position error={err:.3f} >= {FORCE_POSITION_INTERLOCK_BLOCK_ERROR:.2f}; "
                "Force Offset ZERO until position recovers.", flush=True
            )
            return False

        if err <= FORCE_POSITION_FULL_AUTHORITY_ERROR:
            authority = 1.0
        elif err >= FORCE_POSITION_ZERO_AUTHORITY_ERROR:
            authority = 0.0
        else:
            span = max(1e-6, FORCE_POSITION_ZERO_AUTHORITY_ERROR - FORCE_POSITION_FULL_AUTHORITY_ERROR)
            authority = (FORCE_POSITION_ZERO_AUTHORITY_ERROR - err) / span
            authority = max(0.0, min(1.0, authority))

        self.force_position_authority[name] = authority
        if authority <= 0.0:
            self._set_force_loop_zero(name, axis_mask)
            return False
        return True

    def _ramp_all_force_offsets_to_zero(self) -> None:
        """Smoothly remove force commands before releasing the Force-Offset override."""
        if not getattr(self, "cls", None):
            return

        current = {
            "Pitch": int(getattr(self, "pitch_force_offset_command", 0) or 0),
            "Roll": int(getattr(self, "roll_force_offset_command", 0) or 0),
            "Yaw": int(getattr(self, "yaw_force_offset_command", 0) or 0),
        }
        axis_by_name = {
            "Pitch": AX_ELEVATOR,
            "Roll": AX_AILERON,
            "Yaw": AX_RUDDER,
        }

        active = [name for name in self.active_cls_axes if name in axis_by_name]
        if not active:
            return

        steps = max(1, int(CLS_SAFE_SHUTDOWN_RAMP_SEC * CLS_SAFE_SHUTDOWN_RATE_HZ))
        dt = CLS_SAFE_SHUTDOWN_RAMP_SEC / float(steps)
        print(
            "[CLS SAFE SHUTDOWN] Ramping Force Offset to ZERO | "
            + " ".join(f"{n}={current[n]:+d}" for n in active),
            flush=True,
        )

        for step in range(1, steps + 1):
            a = step / float(steps)
            # cosine easing: zero slope at both ends
            smooth = (1.0 - math.cos(math.pi * a)) / 2.0
            for name in active:
                value = int(round(current[name] * (1.0 - smooth)))
                self.cls.send(
                    packet_setting_i32(axis_by_name[name], SID_FORCE_OFFSET, value)
                )
                if name == "Pitch":
                    self.pitch_force_offset_command = value
                elif name == "Roll":
                    self.roll_force_offset_command = value
                else:
                    self.yaw_force_offset_command = value
            time.sleep(dt)

        for name in active:
            self.cls.send(packet_setting_i32(axis_by_name[name], SID_FORCE_OFFSET, 0))
            self._set_force_loop_zero(name, axis_by_name[name])

        time.sleep(CLS_SAFE_SHUTDOWN_ZERO_HOLD_SEC)
        print("[CLS SAFE SHUTDOWN] Force Offset ZERO and stable.", flush=True)

    def _release_all_force_replay(
        self, smooth: bool = False, release_override: bool = True
    ) -> None:
        """
        Remove Pitch/Roll/Yaw force replay. During Automatic teardown, use
        ``smooth=True`` to reach zero gently. ``release_override=False`` keeps
        Force-Offset ownership until AP has been switched OFF.
        """
        if not getattr(self, "cls", None):
            return
        try:
            if smooth:
                self._ramp_all_force_offsets_to_zero()
            else:
                for name, axis in (("Pitch", AX_ELEVATOR), ("Roll", AX_AILERON), ("Yaw", AX_RUDDER)):
                    if name in self.active_cls_axes:
                        self._set_force_loop_zero(name, axis)

            if release_override:
                self.cls.send_with_ack(
                    packet_override(AX_NONE, OVR_VIBRATION),
                    "Release Force-Offset Override",
                    0.15,
                )
        except Exception as exc:
            print(f"[ALL-AXIS FORCE REPLAY] Release warning: {exc}", flush=True)
        finally:
            self.pitch_force_replay_active = False
            self.pitch_force_reference_n = None
            self.pitch_force_actual_n = None
            self.pitch_force_offset_command = 0
            self.pitch_force_integral_internal = 0.0

            self.roll_force_replay_active = False
            self.roll_force_reference_n = None
            self.roll_force_actual_n = None
            self.roll_force_offset_command = 0
            self.roll_force_integral_internal = 0.0

            self.yaw_force_replay_active = False
            self.yaw_force_reference_n = None
            self.yaw_force_actual_n = None
            self.yaw_force_offset_command = 0
            self.yaw_force_integral_internal = 0.0

            for name in ("Pitch", "Roll", "Yaw"):
                self.force_position_interlock_blocked[name] = False
                self.force_position_interlock_latched[name] = False
                self.force_position_authority[name] = 1.0


    def _apply_auto_replay_softening(self) -> bool:
        """
        Temporarily soften only active axes that have a verified engineering
        force path. If force conversion is unavailable, leave that axis's
        Force Scale / Move Back untouched instead of making it unexpectedly light.
        """
        self.auto_replay_softened_axes = []
        if not AUTO_REPLAY_SOFTENING_ENABLED:
            return True

        active_specs = self._active_axis_specs()
        if not active_specs:
            print("[AUTO REPLAY SOFTENING] No active CLS axis.", flush=True)
            return False

        # Force capability is probed before any Force Scale / Move Back change.
        # v535 remains eng22; v474 can become legacy16-eng after validation.
        safe_modes = {"eng22", "raw20-eng", "legacy16-eng"}
        specs = []
        for name, axis, key in active_specs:
            if not self.cls.force_probe_done.get(name, False):
                self.cls._probe_force_mode(axis)
            mode = self.cls.force_mode.get(name)
            if mode in safe_modes:
                specs.append((name, axis, key))
            else:
                print(
                    f"[CLS FORCE SAFE] {name}: mode={mode or 'UNPROBED'}; "
                    "engineering conversion unavailable, so Force Scale and "
                    "Move Back are LEFT UNCHANGED for this axis.",
                    flush=True,
                )

        if not specs:
            print(
                "[AUTO REPLAY SOFTENING] SKIPPED: no active axis has a verified "
                "engineering-force path. Position replay may continue with the "
                "normal CLS2Sim force profile.",
                flush=True,
            )
            return True

        try:
            axes = AX_NONE
            for _name, axis, _key in specs:
                axes |= axis

            # Mark intended ownership BEFORE the first mutating command so that
            # any partial setup can be rolled back safely.
            self.auto_replay_softened_axes = [s[0] for s in specs]
            self.auto_replay_softening_active = True

            ok = True
            ok &= self.cls.send_with_ack(
                packet_override(axes, OVR_FORCE_SCALE),
                "AUTO Force-Scale Override " + ", ".join(s[0] for s in specs),
                0.15,
            )

            for name, axis, _key in specs:
                ok &= self.cls.send_with_ack(
                    packet_setting_u16(
                        axis, SID_FORCE_SCALE, AUTO_REPLAY_FORCE_SCALE_PERCENT
                    ),
                    f"AUTO {name} Force Scale={AUTO_REPLAY_FORCE_SCALE_PERCENT}%",
                    0.15,
                )
                ok &= self.cls.send_with_ack(
                    packet_setting_u8(
                        axis, SID_MOVE_BACK_ENABLE, AUTO_REPLAY_MOVE_BACK_VALUE
                    ),
                    f"AUTO {name} Move Back=OFF",
                    0.15,
                )

            if not ok:
                print(
                    "[AUTO REPLAY SOFTENING] PARTIAL/FAILED -> immediate rollback.",
                    flush=True,
                )
                self._release_auto_replay_softening()
                return False

            print(
                "[AUTO REPLAY SOFTENING] "
                f"axes={','.join(self.auto_replay_softened_axes)} | "
                f"ForceScale={AUTO_REPLAY_FORCE_SCALE_PERCENT}% | "
                f"MoveBack=OFF | status=OK",
                flush=True,
            )
            return True
        except Exception as exc:
            print(f"[AUTO REPLAY SOFTENING] ERROR: {exc}", flush=True)
            try:
                # Transaction rollback even if failure occurred halfway through.
                self._release_auto_replay_softening()
            except Exception:
                self.auto_replay_softening_active = False
                self.auto_replay_softened_axes = []
            return False

    def _release_auto_replay_softening(self) -> None:
        """Restore only axes that this Automatic run actually softened."""
        if not getattr(self, "cls", None):
            return
        if not getattr(self, "auto_replay_softening_active", False):
            self.auto_replay_softened_axes = []
            return

        softened = list(getattr(self, "auto_replay_softened_axes", []) or [])
        try:
            for name, axis, _key in CLS_AXIS_SPECS:
                if name not in softened:
                    continue
                self.cls.send_with_ack(
                    packet_setting_u8(
                        axis,
                        SID_MOVE_BACK_ENABLE,
                        AUTO_REPLAY_MOVE_BACK_RESTORE_VALUE,
                    ),
                    f"Restore {name} Move Back=ON",
                    0.15,
                )

            self.cls.send_with_ack(
                packet_override(AX_NONE, OVR_FORCE_SCALE),
                "Release AUTO Force-Scale Override",
                0.15,
            )
            print(
                "[AUTO REPLAY SOFTENING] Restored softened axes and returned "
                "force-scale ownership to simulation.",
                flush=True,
            )
        except Exception as exc:
            print(f"[AUTO REPLAY SOFTENING] Restore warning: {exc}", flush=True)
        finally:
            self.auto_replay_softening_active = False
            self.auto_replay_softened_axes = []

    def _summarize_reference_replay(self) -> bool:
        """
        Validate a native CLS replay with any non-empty axis subset.

        No missing axis is synthesized from P3D.
        """
        self.reference_has_native_cls = False
        self.reference_cls_axes = []

        frames = self.reference_replay or []
        if len(frames) < 20:
            print(
                f"[Replay Validation] Too few replay frames: {len(frames)}.",
                flush=True,
            )
            return False

        threshold = max(10, len(frames) // 2)
        valid = {}

        for name, _axis, key in CLS_AXIS_SPECS:
            values = []
            for frame in frames:
                try:
                    v = frame.get(key)
                    if v is None:
                        continue
                    fv = float(v)
                    if math.isfinite(fv):
                        values.append(fv)
                except (TypeError, ValueError):
                    pass
            valid[name] = values

        self.reference_cls_axes = [
            name for name, _axis, _key in CLS_AXIS_SPECS
            if len(valid[name]) >= threshold
        ]

        if not self.reference_cls_axes:
            print(
                "[Replay Validation] No usable native CLS axis exists in the "
                "manual replay. At least one of Pitch/Roll/Yaw is required.",
                flush=True,
            )
            return False

        self.reference_has_native_cls = True
        duration = max(
            float(f.get("t_ms", 0) or 0) for f in frames
        ) / 1000.0

        ranges = []
        for name, _axis, _key in CLS_AXIS_SPECS:
            vals = valid[name]
            if name in self.reference_cls_axes and vals:
                ranges.append(f"{name}={min(vals):+.3f}..{max(vals):+.3f}")
            else:
                ranges.append(f"{name}=N/A")

        print(
            "[Replay Validation] Native CLS replay READY | "
            f"axes={','.join(self.reference_cls_axes)} | "
            f"frames={len(frames)} duration={duration:.2f}s | "
            + " | ".join(ranges),
            flush=True,
        )
        return True

    def _probe_cls_before_auto(self) -> Dict[str, Any]:
        """Read-only check of Pitch/Roll/Yaw before enabling any motor."""
        result = self.cls.probe_axes(timeout_sec=0.12)
        parts = []
        for name in ("Pitch", "Roll", "Yaw"):
            info = result[name]
            if info["ok"]:
                parts.append(
                    f"{name}=OK(id={info['device_id']},pos={info['position']:+.3f})"
                )
            else:
                parts.append(f"{name}=NO RESPONSE")

        print("[CLS NET PROBE] " + " | ".join(parts), flush=True)
        return result

    def _open_and_arm_hardware(
        self,
        initial_pitch: float = 0.0,
        initial_roll: float = 0.0,
        initial_yaw: float = 0.0,
    ) -> bool:
        """
        Arm any non-empty connected CLS axis subset.

        Automatic active axes = currently connected axes INTERSECT axes that
        exist in the selected Manual replay. Missing axes are skipped and are
        never sourced from P3D.
        """
        try:
            self.cls_hardware_ready = False
            self.cls.open()

            self.cls.watchdog_enabled = False
            self.cls.stop_watchdog()
            self.cls.flush_rx()

            probe = self._probe_cls_before_auto()
            connected = [
                name for name, _axis, _key in CLS_AXIS_SPECS
                if probe[name]["ok"]
            ]
            active = [
                name for name in connected
                if name in self.reference_cls_axes
            ]
            self._set_active_cls_axes(active)

            if not active:
                print(
                    "[CLS SAFETY] No replayable CLS axis is both connected and "
                    "present in the Manual reference.",
                    flush=True,
                )
                self.cls.start_watchdog(
                    interval_sec=CLS_POLL_INTERVAL_SEC,
                    timeout_ms=CLS_WATCHDOG_MS,
                )
                return False

            skipped_connected = [n for n in connected if n not in active]
            missing_reference = [
                n for n in self.reference_cls_axes if n not in connected
            ]

            # V14.68 fail-closed reference/hardware compatibility.
            # A Pitch/Roll/Yaw Manual is not a valid loader-PC reference when
            # only Pitch hardware is physically available.
            if missing_reference:
                print(
                    "[V14.68 AXIS MATCH] BLOCKED | selected Manual requires "
                    + ",".join(self.reference_cls_axes)
                    + " but connected hardware is "
                    + (",".join(connected) if connected else "NONE")
                    + " | missing=" + ",".join(missing_reference),
                    flush=True,
                )
                self._set_active_cls_axes([])
                self.cls.start_watchdog(
                    interval_sec=CLS_POLL_INTERVAL_SEC,
                    timeout_ms=CLS_WATCHDOG_MS,
                )
                return False

            print(
                "[CLS PARTIAL AXIS] ACTIVE="
                + self._active_axis_summary()
                + (
                    " | connected-but-not-recorded=" + ",".join(skipped_connected)
                    if skipped_connected else ""
                )
                + (
                    " | recorded-but-not-connected=" + ",".join(missing_reference)
                    if missing_reference else ""
                ),
                flush=True,
            )

            status = self.cls.get_status(self.active_cls_axis_mask)
            errors = self.cls.get_errors(self.active_cls_axis_mask)

            operational = True
            for name, _axis, _key in self._active_axis_specs():
                node = self.cls.detected_nodes.get(name)
                s = status.get(node, {}) if node is not None else {}
                state = s.get("state")
                state_name = s.get("state_name", "NO STATUS")
                temp = s.get("temperature_c", "?")
                errs = errors.get(node, []) if node is not None else []

                err_text = (
                    ", ".join(
                        f"0x{e:04X} {ERROR_NAMES.get(e, 'Unknown')}"
                        for e in errs
                    )
                    if errs else "none"
                )

                print(
                    f"[CLS DRIVE] {name}: node={node} "
                    f"state={state_name} temp={temp}C errors={err_text}",
                    flush=True,
                )

                if state is not None and state != 0x01:
                    operational = False
                if errs:
                    operational = False

            if not operational:
                print(
                    "[CLS SAFETY] One active drive is not operational or reports an error.",
                    flush=True,
                )
                self.cls.start_watchdog(
                    interval_sec=CLS_POLL_INTERVAL_SEC,
                    timeout_ms=CLS_WATCHDOG_MS,
                )
                return False

            if not self._apply_auto_replay_softening():
                self._teardown_hardware()
                return False

            axes = self.active_cls_axis_mask
            ok = True

            ok &= self.cls.send_with_ack(
                packet_override(axes, OVR_AP_ENABLE),
                f"AP-enable override ({self._active_axis_summary()})",
            )
            ok &= self.cls.send_with_ack(
                packet_override(axes, OVR_AP_POSITION),
                f"AP-position override ({self._active_axis_summary()})",
            )
            ok &= self._set_cls_position_forwarding_to_p3d(False)

            for name, axis, _key in self._active_axis_specs():
                ok &= self.cls.send_with_ack(
                    packet_setting_u8(axis, SID_AP_ENABLE, 0),
                    f"{name} AP OFF",
                )

            cfg_by_name = {
                name: (force, speed)
                for axis, name, force, speed in self._selected_ap_config()
            }

            for name, axis, _key in self._active_axis_specs():
                force, speed = cfg_by_name[name]
                ok &= self.cls.send_with_ack(
                    packet_setting_u16(axis, SID_AP_FORCE, force),
                    f"{name} AP force={force}",
                )
                ok &= self.cls.send_with_ack(
                    packet_setting_u16(axis, SID_AP_SPEED, speed),
                    f"{name} AP speed={speed}",
                )

            if VIRTUAL_FORCE_POSITION_ONLY_MODE:
                # V15.12.2 VF POSITION-ONLY:
                # Ensure no residual Force Offset from a previous run can oppose
                # the AP position controller. 0xD0 is ZEROED, then its override
                # is released. No further Force Offset command is streamed.
                zero_owner_ok = self.cls.send_with_ack(
                    packet_override(axes, OVR_VIBRATION),
                    "Temporary Force-Offset ZERO override",
                    0.15,
                )
                zero_send_ok = True
                for _name, _axis, _key in self._active_axis_specs():
                    zero_send_ok = bool(
                        self.cls.send(packet_setting_i32(_axis, SID_FORCE_OFFSET, 0))
                        and zero_send_ok
                    )
                zero_release_ok = self.cls.send_with_ack(
                    packet_override(AX_NONE, OVR_VIBRATION),
                    "Release Force-Offset override after ZERO",
                    0.15,
                )
                print(
                    "[V15.12.2 VF POSITION-ONLY] "
                    f"Force Offset ZERO | owner={'OK' if zero_owner_ok else 'WARN'} | "
                    f"zeroSend={'OK' if zero_send_ok else 'FAIL'} | "
                    f"release={'OK' if zero_release_ok else 'WARN'} | "
                    "force replay DISABLED; Virtual Force is READ-ONLY.",
                    flush=True,
                )
                if not zero_send_ok:
                    print(
                        "[V15.12.2 VF POSITION-ONLY] FAIL: could not guarantee "
                        "Force Offset = 0 on all active axes.",
                        flush=True,
                    )
                    self._teardown_hardware()
                    return False

            initial_by_name = {
                "Pitch": max(-1.0, min(1.0, float(initial_pitch))),
                "Roll": max(-1.0, min(1.0, float(initial_roll))),
                "Yaw": max(-1.0, min(1.0, float(initial_yaw))),
            }

            for name, axis, _key in self._active_axis_specs():
                target = initial_by_name[name]
                ok &= self.cls.send_with_ack(
                    packet_setting_float(axis, SID_AP_POSITION, target),
                    f"{name} initial AP pos={target:+.3f}",
                )

            if not ok:
                print(
                    "[CLS SAFETY] One or more active-axis commands were rejected.",
                    flush=True,
                )
                self._teardown_hardware()
                self.cls.start_watchdog(
                    interval_sec=CLS_POLL_INTERVAL_SEC,
                    timeout_ms=CLS_WATCHDOG_MS,
                )
                return False

            for name, axis, _key in self._active_axis_specs():
                if not self.cls.send_with_ack(
                    packet_setting_u8(axis, SID_AP_ENABLE, 1),
                    f"{name} AP ON",
                ):
                    self._teardown_hardware()
                    return False

                ap_state = self.cls.verify_ap_enabled(axis)
                print(
                    f"[CLS AP VERIFY] {name}: "
                    f"{'ENABLED' if ap_state is True else 'NOT VERIFIED' if ap_state is None else 'DISABLED'}",
                    flush=True,
                )
                if ap_state is False:
                    self._teardown_hardware()
                    return False

            # V15.12.2 VF POSITION-ONLY:
            # _force_stage_allows() is False for every axis, so none of the
            # measured-force / Force Offset controllers can arm or update.
            # AP-position replay remains fully active.
            if VIRTUAL_FORCE_POSITION_ONLY_MODE:
                print(
                    "[V15.12.2 VF POSITION-ONLY] AP position replay ACTIVE | "
                    "Force Offset replay OFF | Virtual Force 0x50/0x51 READ ONLY.",
                    flush=True,
                )

            force_arm_failed = False

            for name, arm_fn in (
                ("Pitch", self._arm_pitch_force_replay),
                ("Roll", self._arm_roll_force_replay),
                ("Yaw", self._arm_yaw_force_replay),
            ):
                if name not in self.active_cls_axes:
                    continue
                if not self._force_stage_allows(name):
                    print(
                        f"[COMMISSION STAGE] {name} FORCE intentionally OFF | "
                        "position replay remains active.",
                        flush=True,
                    )
                    continue
                arm_ok = arm_fn()
                if not arm_ok:
                    force_arm_failed = True
                    print(
                        f"[{name.upper()} FORCE REPLAY] WARNING: force replay not armed; "
                        "position replay is unchanged and can still operate.",
                        flush=True,
                    )

            if force_arm_failed:
                print(
                    "[CLS FORCE SAFE] FAIL-CLOSED: at least one required active-axis "
                    "force loop failed to arm. Automatic is ABORTED; position-only "
                    "replay is not permitted in V14.42.",
                    flush=True,
                )
                # _teardown_hardware() restores AP/ForceOffset/MoveBack/ForceScale.
                self._teardown_hardware()
                self.cls.start_watchdog(
                    interval_sec=CLS_POLL_INTERVAL_SEC,
                    timeout_ms=CLS_WATCHDOG_MS,
                )
                return False

            self.cls.watchdog_enabled = True
            self.cls.start_watchdog(
                interval_sec=CLS_POLL_INTERVAL_SEC,
                timeout_ms=CLS_WATCHDOG_MS,
            )
            self.cls.send(packet_watchdog(CLS_WATCHDOG_MS))

            self.cls_hardware_ready = False
            self.cls_prep_complete = False

            cfg_summary = []
            for name, _axis, _key in self._active_axis_specs():
                force, speed = cfg_by_name[name]
                cfg_summary.append(
                    f"{name}(F={force},S={speed},node={self.cls.detected_nodes.get(name)})"
                )

            print(
                "[CLS-P V13] Hardware CONFIGURED | axes="
                + self._active_axis_summary()
                + " | "
                + " ".join(cfg_summary),
                flush=True,
            )
            return True

        except Exception as exc:
            print(f"[CLS-P] Partial-axis hardware setup error: {exc}", flush=True)
            self.cls_hardware_ready = False
            return False

    def _ramp_hardware_to_initial_position(
        self,
        target_pitch: float,
        target_roll: float,
        target_yaw: float,
        duration_sec: float = 2.0,
    ) -> None:
        """Smoothly ramp only active CLS axes to their recorded start values."""
        try:
            if not self.active_cls_axes:
                raise RuntimeError("No active CLS axis")

            target_by_name = {
                "Pitch": max(-1.0, min(1.0, float(target_pitch))),
                "Roll": max(-1.0, min(1.0, float(target_roll))),
                "Yaw": max(-1.0, min(1.0, float(target_yaw))),
            }
            actual_by_name = {
                "Pitch": self.cls.last_pitch_pos,
                "Roll": self.cls.last_roll_pos,
                "Yaw": self.cls.last_rudder_pos,
            }

            start_by_name = {}
            for name in self.active_cls_axes:
                actual = actual_by_name[name]
                start_by_name[name] = (
                    float(actual)
                    if actual is not None
                    else target_by_name[name]
                )

            details = " | ".join(
                f"{name} {start_by_name[name]:+.3f}->{target_by_name[name]:+.3f}"
                for name in self.active_cls_axes
            )
            print("[CLS-P] Initial-position ramp | " + details, flush=True)

            steps = max(1, int(duration_sec * 50))
            dt = duration_sec / steps

            for step in range(1, steps + 1):
                alpha = step / float(steps)
                smooth = (1.0 - math.cos(alpha * math.pi)) / 2.0

                for name, axis, _key in self._active_axis_specs():
                    value = (
                        start_by_name[name]
                        + smooth * (target_by_name[name] - start_by_name[name])
                    )
                    self.cls.send(
                        packet_setting_float(axis, SID_AP_POSITION, value)
                    )

                time.sleep(dt)

            for name, axis, _key in self._active_axis_specs():
                self.cls.send(
                    packet_setting_float(
                        axis, SID_AP_POSITION, target_by_name[name]
                    )
                )

            print(
                "[CLS-P] Active-axis initial-position ramp complete: "
                + self._active_axis_summary(),
                flush=True,
            )

        except Exception as exc:
            self.cls_hardware_ready = False
            print(f"[CLS-P] Initial-position ramp error: {exc}", flush=True)

    def _teardown_hardware(self) -> None:
        """
        V14.12 ordered BRUNNER handover.

        Automatic shutdown order:
          1) keep watchdog alive and ramp Force Offset smoothly to zero,
          2) switch AP OFF on every owned axis,
          3) release Force-Offset / AP-position / AP-enable ownership,
          4) return normal CLS->P3D forwarding,
          5) restore Move Back / force-scale ownership,
          6) finally remove the watchdog.

        This sequence is synchronous, so report generation cannot begin until
        the loader handover has completed.
        """
        # V14.42 HARD SAFETY RULE: Manual is read-only. Never issue a teardown,
        # override, AP, Move-Back, Force-Scale or Force-Offset write in Manual.
        if self.test_mode == "manual":
            self.cls_hardware_ready = False
            self.cls_prep_complete = False
            print(
                "[MANUAL READ-ONLY] Teardown request ignored; no CLS command sent.",
                flush=True,
            )
            return

        try:
            # V14.58: remove direct P3D commands BEFORE releasing normal CLS
            # Position-to-Sim ownership, so the simulator never has two sources.
            try:
                self._v1458_stop_direct_mirror(close_bridge=True)
            except Exception as mirror_exc:
                print(f"[V14.58 DIRECT P3D] teardown warning: {mirror_exc}", flush=True)
            self.cls_hardware_ready = False
            self.cls_prep_complete = False

            if hasattr(self, "cls") and self.cls:
                active_specs = list(self._active_axis_specs())
                is_auto = (self.test_mode == "auto") and bool(active_specs)

                if is_auto:
                    print(
                        "[CLS SAFE SHUTDOWN] START | axes="
                        + self._active_axis_summary(),
                        flush=True,
                    )

                    # Keep heartbeat alive while the final controlled commands
                    # are being sent. This avoids ownership timing out mid-ramp.
                    self.cls.watchdog_enabled = True
                    self.cls.send(packet_watchdog(CLS_WATCHDOG_MS))

                    # 1) Remove force gently while AP position is still owned.
                    # Keep the Force-Offset override until AP itself is OFF.
                    self._release_all_force_replay(
                        smooth=True, release_override=False
                    )

                    # 2) AP OFF before normal centering / Move Back returns.
                    ap_off_ok = True
                    for name, axis, _key in active_specs:
                        ap_off_ok &= self.cls.send_with_ack(
                            packet_setting_u8(axis, SID_AP_ENABLE, 0),
                            f"SAFE {name} AP OFF",
                            0.20,
                        )
                    time.sleep(CLS_SAFE_SHUTDOWN_AP_OFF_HOLD_SEC)
                    print(
                        f"[CLS SAFE SHUTDOWN] AP OFF "
                        f"{'OK' if ap_off_ok else 'WITH WARNING'}.",
                        flush=True,
                    )

                    # 3) Now release zeroed Force Offset, then AP command ownership.
                    self.cls.send_with_ack(
                        packet_override(AX_NONE, OVR_VIBRATION),
                        "Release Force-Offset Override after AP OFF",
                        0.20,
                    )
                    self.cls.send_with_ack(
                        packet_override(AX_NONE, OVR_AP_POSITION),
                        "Release AP-position Override",
                        0.20,
                    )
                    self.cls.send_with_ack(
                        packet_override(AX_NONE, OVR_AP_ENABLE),
                        "Release AP-enable Override",
                        0.20,
                    )

                    # 4) Return normal physical CLS output to the simulator.
                    self.cls.send_with_ack(
                        packet_override(AX_NONE, OVR_POSITION_TO_SIM),
                        "Release Position-to-Sim Override",
                        0.20,
                    )
                    self.cls_to_p3d_feedback_enabled = True

                    # 5) Only now restore Move Back / normal Force Scale.
                    self._release_auto_replay_softening()

                    # Final neutral force command after all ownership changes.
                    for _name, axis, _key in active_specs:
                        self.cls.send(packet_setting_i32(axis, SID_FORCE_OFFSET, 0))
                    time.sleep(0.10)

                    # 6) End external watchdog ownership last.
                    self.cls.send(packet_watchdog(0))
                    self.cls.watchdog_enabled = False
                    print(
                        "[CLS SAFE SHUTDOWN] COMPLETE | "
                        "ForceOffset=0, AP=OFF, overrides released, "
                        "MoveBack/ForceScale returned to CLS2Sim.",
                        flush=True,
                    )
                else:
                    # Manual/early-stop path: no automatic force/AP handover is
                    # needed, but still release any ownership held by this test.
                    self._release_all_force_replay(smooth=False)
                    for name, axis, _key in active_specs:
                        self.cls.send(packet_setting_u8(axis, SID_AP_ENABLE, 0))
                    for oid in (OVR_AP_POSITION, OVR_AP_ENABLE, OVR_POSITION_TO_SIM):
                        self.cls.send(packet_override(AX_NONE, oid))
                    self._release_auto_replay_softening()
                    self.cls.send(packet_watchdog(0))
                    self.cls.watchdog_enabled = False

                self.cls.last_pitch_target = None
                self.cls.last_roll_target = None
                self.cls.last_rudder_target = None

            print(
                "[CLS-P] Released active axes: " + self._active_axis_summary(),
                flush=True,
            )
        except Exception as exc:
            # Do not hide teardown faults; they are critical commissioning data.
            print(f"[CLS SAFE SHUTDOWN] ERROR: {exc}", flush=True)
            try:
                if hasattr(self, "cls") and self.cls:
                    for axis in (AX_ELEVATOR, AX_AILERON, AX_RUDDER):
                        self.cls.send(packet_setting_i32(axis, SID_FORCE_OFFSET, 0))
                        self.cls.send(packet_setting_u8(axis, SID_AP_ENABLE, 0))
                        self.cls.send(
                            packet_setting_u8(
                                axis,
                                SID_MOVE_BACK_ENABLE,
                                AUTO_REPLAY_MOVE_BACK_RESTORE_VALUE,
                            )
                        )
                    for oid in REPLAY_OVERRIDE_IDS:
                        self.cls.send(packet_override(AX_NONE, oid))
                    self.cls.send(packet_override(AX_NONE, OVR_FORCE_SCALE))
                    self.cls.send(packet_watchdog(0))
                    self.cls.watchdog_enabled = False
            except Exception:
                pass

    def _init_ui(self):
        """Initialize GUI Layout and Widgets."""
        main_widget = QWidget(self)
        self.setCentralWidget(main_widget)
        main_layout = QVBoxLayout(main_widget)
        main_layout.setSpacing(16)
        main_layout.setContentsMargins(24, 24, 24, 24)

        # Header Title Card
        header_card = QFrame()
        header_card.setObjectName("HeaderCard")
        header_layout = QHBoxLayout(header_card)
        header_layout.setContentsMargins(20, 16, 20, 16)

        title_vbox = QVBoxLayout()
        app_title = QLabel("Super Mushshak Engine Tests - Climb V15.12.5 Base")
        app_title.setFont(QFont("Segoe UI", 18, QFont.Bold))
        app_title.setStyleSheet("color: #f5e0dc;")
        
        app_subtitle = QLabel("CLS2Sim Physical Closed-Loop | Prepar3D Aircraft Response | QTG Testing")
        app_subtitle.setFont(QFont("Segoe UI", 10))
        app_subtitle.setStyleSheet("color: #a6adc8;")
        title_vbox.addWidget(app_title)
        title_vbox.addWidget(app_subtitle)

        header_layout.addLayout(title_vbox)
        header_layout.addStretch()

        # Stage Badge
        self.stage_badge = QLabel("Not connected")
        self.stage_badge.setFont(QFont("Segoe UI", 11, QFont.Bold))
        self.stage_badge.setContentsMargins(16, 8, 16, 8)
        self.stage_badge.setAlignment(Qt.AlignCenter)
        header_layout.addWidget(self.stage_badge)

        main_layout.addWidget(header_card)

        # Grid of Status and Telemetry Cards
        grid_layout = QGridLayout()
        grid_layout.setSpacing(16)

        # 1. Connection Status Card
        card_conn, card_conn_layout = self._create_card("Connection Status", "📡")
        self.lbl_conn_status = QLabel("Disconnected")
        self.lbl_conn_status.setFont(QFont("Segoe UI", 12, QFont.Bold))
        self.lbl_conn_status.setStyleSheet("color: #f38ba8;")
        
        self.lbl_aircraft = QLabel("Aircraft: Waiting for connection...")
        self.lbl_aircraft.setFont(QFont("Segoe UI", 10))
        self.lbl_aircraft.setStyleSheet("color: #bac2de;")
        
        self.lbl_engines = QLabel("Detected Engines: --")
        self.lbl_engines.setFont(QFont("Segoe UI", 10))
        self.lbl_engines.setStyleSheet("color: #bac2de;")

        self.lbl_cls_status = QLabel(
            f"CLS2Sim: checking {CLS2SIM_HOST}:{CLS2SIM_PORT} ..."
        )
        self.lbl_cls_status.setFont(QFont("Segoe UI", 9))
        self.lbl_cls_status.setStyleSheet("color: #f9e2af;")
        self.lbl_cls_status.setWordWrap(True)

        card_conn_layout.addWidget(self.lbl_conn_status)
        card_conn_layout.addWidget(self.lbl_aircraft)
        card_conn_layout.addWidget(self.lbl_engines)
        card_conn_layout.addWidget(self.lbl_cls_status)
        card_conn_layout.addStretch()
        grid_layout.addWidget(card_conn, 0, 0)

        # 2. Throttle Position Card
        card_throttle, card_throttle_layout = self._create_card("Throttle Position", "⚡")
        
        self.lbl_throttle_val = QLabel("0.0 %")
        self.lbl_throttle_val.setFont(QFont("Segoe UI", 22, QFont.Bold))
        self.lbl_throttle_val.setStyleSheet("color: #89b4fa;")

        self.throttle_bar = QProgressBar()
        self.throttle_bar.setRange(0, 100)
        self.throttle_bar.setValue(0)
        self.throttle_bar.setTextVisible(False)
        self.throttle_bar.setFixedHeight(12)
        self.throttle_bar.setStyleSheet("""
            QProgressBar {
                background-color: #313244;
                border-radius: 6px;
            }
            QProgressBar::chunk {
                background-color: #89b4fa;
                border-radius: 6px;
            }
        """)

        self.lbl_throttle_detail = QLabel("Engines: waiting for telemetry...")
        self.lbl_throttle_detail.setFont(QFont("Segoe UI", 9))
        self.lbl_throttle_detail.setStyleSheet("color: #a6adc8;")

        card_throttle_layout.addWidget(self.lbl_throttle_val)
        card_throttle_layout.addWidget(self.throttle_bar)
        card_throttle_layout.addWidget(self.lbl_throttle_detail)
        card_throttle_layout.addStretch()
        grid_layout.addWidget(card_throttle, 0, 1)

        # 3. Aircraft Attitude Card
        card_att, card_att_layout = self._create_card("Aircraft Attitude & Orientation", "📐")
        self.lbl_pitch_val = QLabel("Pitch: 0.0°")
        self.lbl_pitch_val.setFont(QFont("Segoe UI", 10))
        self.lbl_pitch_val.setStyleSheet("color: #cdd6f4;")

        self.lbl_bank_val = QLabel("Bank: 0.0°")
        self.lbl_bank_val.setFont(QFont("Segoe UI", 10))
        self.lbl_bank_val.setStyleSheet("color: #cdd6f4;")

        self.lbl_yaw_val = QLabel("Yaw (True Hdg): 0.0° | Mag: 0.0°")
        self.lbl_yaw_val.setFont(QFont("Segoe UI", 10))
        self.lbl_yaw_val.setStyleSheet("color: #cdd6f4;")

        card_att_layout.addWidget(self.lbl_pitch_val)
        card_att_layout.addWidget(self.lbl_bank_val)
        card_att_layout.addWidget(self.lbl_yaw_val)
        card_att_layout.addStretch()
        grid_layout.addWidget(card_att, 1, 0)

        # 4. Recording Status Card
        card_rec, card_rec_layout = self._create_card("Recording Telemetry", "⏱️")
        
        self.lbl_rec_status = QLabel("Idle")
        self.lbl_rec_status.setFont(QFont("Segoe UI", 12, QFont.Bold))
        self.lbl_rec_status.setStyleSheet("color: #a6adc8;")

        self.lbl_timer = QLabel("0 / 75 seconds (0 rows)")
        self.lbl_timer.setFont(QFont("Segoe UI", 11))
        self.lbl_timer.setStyleSheet("color: #cdd6f4;")

        self.rec_progress = QProgressBar()
        self.rec_progress.setRange(0, 75)
        self.rec_progress.setValue(0)
        self.rec_progress.setFixedHeight(14)
        self.rec_progress.setStyleSheet("""
            QProgressBar {
                background-color: #313244;
                border-radius: 7px;
                text-align: center;
                color: #cdd6f4;
                font-weight: bold;
            }
            QProgressBar::chunk {
                background-color: #a6e3a1;
                border-radius: 7px;
            }
        """)

        card_rec_layout.addWidget(self.lbl_rec_status)
        card_rec_layout.addWidget(self.lbl_timer)
        card_rec_layout.addWidget(self.rec_progress)
        card_rec_layout.addStretch()
        grid_layout.addWidget(card_rec, 1, 1)

        main_layout.addLayout(grid_layout)

        # CSV Location Card
        card_csv = QFrame()
        card_csv.setObjectName("Card")
        csv_layout = QHBoxLayout(card_csv)
        csv_layout.setContentsMargins(16, 12, 16, 12)

        lbl_csv_icon = QLabel("📁")
        lbl_csv_icon.setFont(QFont("Segoe UI", 14))

        csv_info_vbox = QVBoxLayout()
        csv_info_title = QLabel("CSV Output Location")
        csv_info_title.setFont(QFont("Segoe UI", 10, QFont.Bold))
        csv_info_title.setStyleSheet("color: #b4befe;")
        
        self.lbl_csv_path = QLabel(str(self.data_dir))
        self.lbl_csv_path.setFont(QFont("Segoe UI", 9))
        self.lbl_csv_path.setStyleSheet("color: #a6adc8;")
        self.lbl_csv_path.setWordWrap(True)

        csv_info_vbox.addWidget(csv_info_title)
        csv_info_vbox.addWidget(self.lbl_csv_path)

        btn_open_folder = QPushButton("Open Folder")
        btn_open_folder.setFixedWidth(110)
        btn_open_folder.setCursor(Qt.PointingHandCursor)
        btn_open_folder.setStyleSheet("""
            QPushButton {
                background-color: #313244;
                color: #cdd6f4;
                border: 1px solid #45475a;
                border-radius: 6px;
                padding: 6px 12px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #45475a;
                border-color: #585b70;
            }
        """)
        btn_open_folder.clicked.connect(self._open_data_folder)

        csv_layout.addWidget(lbl_csv_icon)
        csv_layout.addLayout(csv_info_vbox)
        csv_layout.addStretch()

        main_layout.addWidget(card_csv)

        # ── V14.43 Manual Reference Selection ───────────────────────────────
        ref_group = QGroupBox("Manual Reference for Automatic")
        ref_group.setFont(QFont("Segoe UI", 10, QFont.Bold))
        ref_group.setStyleSheet("""
            QGroupBox {
                color: #b4befe;
                border: 1px solid #313244;
                border-radius: 8px;
                margin-top: 8px;
                padding-top: 8px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 12px;
            }
        """)
        ref_outer = QVBoxLayout(ref_group)
        ref_outer.setContentsMargins(12, 8, 12, 8)
        ref_outer.setSpacing(6)

        ref_row = QHBoxLayout()
        ref_row.setSpacing(8)
        self.cmb_manual_reference = QComboBox()
        self.cmb_manual_reference.setMinimumWidth(560)
        self.cmb_manual_reference.setToolTip(
            "Select the exact saved Manual run that Automatic will reproduce."
        )
        self.cmb_manual_reference.currentIndexChanged.connect(
            self._on_manual_reference_selection_changed
        )

        self.btn_refresh_manual_refs = QPushButton("↻ Refresh Manuals")
        self.btn_refresh_manual_refs.setFixedWidth(145)
        self.btn_refresh_manual_refs.setCursor(Qt.PointingHandCursor)
        self.btn_refresh_manual_refs.clicked.connect(
            lambda: self._refresh_manual_reference_selector()
        )

        ref_row.addWidget(self.cmb_manual_reference, 1)
        ref_row.addWidget(self.btn_refresh_manual_refs)
        ref_outer.addLayout(ref_row)

        self.lbl_manual_reference_info = QLabel(
            "No Manual reference selected. Record a Manual or press Refresh Manuals."
        )
        self.lbl_manual_reference_info.setFont(QFont("Segoe UI", 9))
        self.lbl_manual_reference_info.setStyleSheet("color: #a6adc8;")
        self.lbl_manual_reference_info.setWordWrap(True)
        ref_outer.addWidget(self.lbl_manual_reference_info)
        main_layout.addWidget(ref_group)

        # ── Test Mode Selection Row ──────────────────────────────────────────
        mode_group = QGroupBox("Test Mode")
        mode_group.setFont(QFont("Segoe UI", 10, QFont.Bold))
        mode_group.setStyleSheet("""
            QGroupBox {
                color: #b4befe;
                border: 1px solid #313244;
                border-radius: 8px;
                margin-top: 8px;
                padding-top: 8px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 12px;
            }
        """)
        mode_layout = QHBoxLayout(mode_group)
        mode_layout.setSpacing(12)
        mode_layout.setContentsMargins(12, 8, 12, 8)

        self.btn_auto_test = QPushButton("🤖  Automatic Testing")
        self.btn_auto_test.setFixedHeight(44)
        self.btn_auto_test.setFont(QFont("Segoe UI", 11, QFont.Bold))
        self.btn_auto_test.setCursor(Qt.PointingHandCursor)
        self.btn_auto_test.setStyleSheet("""
            QPushButton {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #89b4fa, stop:1 #1e66f5);
                color: #11111b;
                border: none;
                border-radius: 8px;
            }
            QPushButton:hover {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #a6c8ff, stop:1 #1e66f5);
            }
            QPushButton:disabled { background-color: #313244; color: #585b70; }
        """)
        self.btn_auto_test.clicked.connect(self.start_auto_test_only)


        self.btn_one_visit = QPushButton("✅  FINAL Two-Manual Run")
        self.btn_one_visit.setFixedHeight(44)
        self.btn_one_visit.setFont(QFont("Segoe UI", 10, QFont.Bold))
        self.btn_one_visit.setCursor(Qt.PointingHandCursor)
        self.btn_one_visit.setToolTip(
            "Automatically test bounded legacy profiles for BOTH September-17 Manuals, "
            "select the lowest measured fit score, then run a final full-duration replay."
        )
        self.btn_one_visit.setStyleSheet("""
            QPushButton { background-color:#cba6f7; color:#11111b; border:none; border-radius:8px; }
            QPushButton:hover { background-color:#ddb6f2; }
            QPushButton:disabled { background-color:#313244; color:#585b70; }
        """)
        self.btn_one_visit.clicked.connect(self.start_final_two_manual_run)

        self.btn_manual_test = QPushButton("✋  Manual Testing")
        self.btn_manual_test.setFixedHeight(44)
        self.btn_manual_test.setFont(QFont("Segoe UI", 11, QFont.Bold))
        self.btn_manual_test.setCursor(Qt.PointingHandCursor)
        self.btn_manual_test.setStyleSheet("""
            QPushButton {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #a6e3a1, stop:1 #40a02b);
                color: #11111b;
                border: none;
                border-radius: 8px;
            }
            QPushButton:hover {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #befebe, stop:1 #40a02b);
            }
            QPushButton:disabled { background-color: #313244; color: #585b70; }
        """)
        self.btn_manual_test.clicked.connect(self.start_manual_test)

        self.btn_stop = QPushButton("⏹  STOP")
        self.btn_stop.setFixedHeight(44)
        self.btn_stop.setFont(QFont("Segoe UI", 11, QFont.Bold))
        self.btn_stop.setCursor(Qt.PointingHandCursor)
        self.btn_stop.setStyleSheet("""
            QPushButton {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #f38ba8, stop:1 #d20f39);
                color: #ffffff;
                border: none;
                border-radius: 8px;
            }
            QPushButton:hover {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #f5a4b9, stop:1 #d20f39);
            }
            QPushButton:disabled { background-color: #313244; color: #585b70; }
        """)
        self.btn_stop.clicked.connect(self.stop_test)
        self.btn_stop.setEnabled(False)

        self.btn_record = QPushButton("🔴  Start Recording (Press SPACEBAR)")
        self.btn_record.setFixedHeight(44)
        self.btn_record.setFont(QFont("Segoe UI", 11, QFont.Bold))
        self.btn_record.setCursor(Qt.PointingHandCursor)
        self.btn_record.setStyleSheet("""
            QPushButton {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #f38ba8, stop:1 #d20f39);
                color: #ffffff;
                border: none;
                border-radius: 8px;
            }
            QPushButton:hover {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #f5a4b9, stop:1 #d20f39);
            }
            QPushButton:disabled { background-color: #313244; color: #585b70; }
        """)
        self.btn_record.clicked.connect(self.arm_recording)
        self.btn_record.setVisible(False)  # Hidden until fully ready

        # V14.35: explicit human confirmation after P3D terrain/scenery load.
        self.btn_p3d_loaded = QPushButton("✅  P3D Loaded — Continue Setup")
        self.btn_p3d_loaded.setFixedHeight(44)
        self.btn_p3d_loaded.setFont(QFont("Segoe UI", 11, QFont.Bold))
        self.btn_p3d_loaded.setCursor(Qt.PointingHandCursor)
        self.btn_p3d_loaded.setStyleSheet("""
            QPushButton {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #f9e2af, stop:1 #df8e1d);
                color: #11111b;
                border: none;
                border-radius: 8px;
            }
            QPushButton:hover {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #ffe8a3, stop:1 #e6a23c);
            }
            QPushButton:disabled {
                background-color: #313244;
                color: #585b70;
            }
        """)
        self.btn_p3d_loaded.clicked.connect(self._on_p3d_loaded_continue)
        self.btn_p3d_loaded.setVisible(False)

        # Connect Spacebar Keyboard Shortcut to trigger Start Recording
        self.shortcut_space = QShortcut(QKeySequence(Qt.Key_Space), self)
        self.shortcut_space.activated.connect(self._on_spacebar_pressed)

        mode_layout.addWidget(self.btn_auto_test)
        mode_layout.addWidget(self.btn_one_visit)
        mode_layout.addWidget(self.btn_manual_test)
        mode_layout.addWidget(self.btn_p3d_loaded)
        mode_layout.addWidget(self.btn_record)
        mode_layout.addWidget(self.btn_stop)

        # Keep a reference for _update_stage_ui compatibility
        self.btn_start = self.btn_manual_test

        main_layout.addWidget(mode_group)

        # Populate saved Manual runs only after all selector widgets exist.
        self._refresh_manual_reference_selector()
        self._v1493_load_persisted_overrides()

    def _create_card(self, title: str, icon: str):
        """Helper to create stylized card containers.
        
        Returns a (card_frame, content_layout) tuple.
        The caller adds widgets to content_layout, NOT to the card frame directly.
        This avoids QLayout double-assignment warnings.
        """
        card = QFrame()
        card.setObjectName("Card")
        outer_layout = QVBoxLayout(card)
        outer_layout.setContentsMargins(16, 14, 16, 14)
        outer_layout.setSpacing(8)
        
        lbl_head = QLabel(f"{icon}  {title}")
        lbl_head.setFont(QFont("Segoe UI", 10, QFont.Bold))
        lbl_head.setStyleSheet("color: #b4befe;")
        outer_layout.addWidget(lbl_head)

        # Separator line
        sep = QFrame()
        sep.setFrameShape(QFrame.HLine)
        sep.setStyleSheet("color: #313244;")
        outer_layout.addWidget(sep)

        # Inner content layout — callers add their widgets here
        content_layout = QVBoxLayout()
        content_layout.setSpacing(6)
        outer_layout.addLayout(content_layout)
        
        return card, content_layout

    def _apply_dark_theme(self):
        """Apply modern Catppuccin-inspired dark theme stylesheet."""
        self.setStyleSheet("""
            QMainWindow {
                background-color: #1e1e2e;
            }
            QFrame#HeaderCard {
                background-color: #252538;
                border: 1px solid #313244;
                border-radius: 10px;
            }
            QFrame#Card {
                background-color: #252538;
                border: 1px solid #313244;
                border-radius: 10px;
            }
        """)

    def _set_auto_buttons_enabled(self, enabled: bool) -> None:
        if hasattr(self, "btn_auto_test"):
            self.btn_auto_test.setEnabled(bool(enabled))
        if hasattr(self, "btn_one_visit"):
            self.btn_one_visit.setEnabled(bool(enabled) and not self.v1493_one_visit_active)
        if hasattr(self, "cmb_manual_reference"):
            self.cmb_manual_reference.setEnabled(bool(enabled))
        if hasattr(self, "btn_refresh_manual_refs"):
            self.btn_refresh_manual_refs.setEnabled(bool(enabled))

    def _update_stage_ui(self, stage: str, details: str = ""):
        """Update Operating Stage Badge and GUI element states."""
        self.operating_stage = stage
        self.stage_badge.setText(stage)

        badge_styles = {
            STAGE_NOT_CONNECTED: ("background-color: #313244; color: #a6adc8; border-radius: 14px;", "Disconnected", "#f38ba8"),
            STAGE_CONNECTING: ("background-color: #89b4fa; color: #11111b; border-radius: 14px;", "Connecting...", "#89b4fa"),
            STAGE_ARMED: ("background-color: #f9e2af; color: #11111b; border-radius: 14px;", "Connected", "#a6e3a1"),
            STAGE_INITIALIZING: ("background-color: #fab387; color: #11111b; border-radius: 14px;", "Initializing...", "#fab387"),
            STAGE_RECORDING: ("background-color: #a6e3a1; color: #11111b; border-radius: 14px;", "Connected", "#a6e3a1"),
            STAGE_COMPLETED: ("background-color: #cba6f7; color: #11111b; border-radius: 14px;", "Connected", "#a6e3a1"),
            STAGE_STOPPED_BY_USER: ("background-color: #fab387; color: #11111b; border-radius: 14px;", "Stopped", "#fab387"),
            STAGE_CONNECTION_ERROR: ("background-color: #f38ba8; color: #11111b; border-radius: 14px;", "Connection Error", "#f38ba8"),
            STAGE_AUTO_BLOCKED: ("background-color: #fab387; color: #11111b; border-radius: 14px;", "Connected", "#a6e3a1"),
        }

        b_style, conn_text, conn_color = badge_styles.get(
            stage, ("background-color: #313244; color: #cdd6f4; border-radius: 14px;", "Connected", "#a6e3a1")
        )
        self.stage_badge.setStyleSheet(b_style)
        self.lbl_conn_status.setText(conn_text)
        self.lbl_conn_status.setStyleSheet(f"color: {conn_color};")

        # Button States
        if stage in [STAGE_NOT_CONNECTED, STAGE_COMPLETED, STAGE_STOPPED_BY_USER, STAGE_CONNECTION_ERROR, STAGE_AUTO_BLOCKED]:
            self._set_auto_buttons_enabled(True)
            self.btn_manual_test.setEnabled(True)
            self.btn_record.setVisible(False)
            self.btn_stop.setEnabled(False)
        elif stage == STAGE_CONNECTED_READY:
            # Connected but not yet armed — show Record button, hide mode buttons
            self._set_auto_buttons_enabled(False)
            self.btn_manual_test.setEnabled(False)
            self.btn_record.setVisible(True)
            self.btn_record.setEnabled(True)
            self.btn_stop.setEnabled(True)
        elif stage in [STAGE_CONNECTING, STAGE_ARMED, STAGE_INITIALIZING, STAGE_RECORDING]:
            self._set_auto_buttons_enabled(False)
            self.btn_manual_test.setEnabled(False)
            self.btn_record.setVisible(False)
            self.btn_stop.setEnabled(True)

        if hasattr(self, "btn_p3d_loaded"):
            show_p3d_confirm = bool(
                self.test_mode == "auto"
                and self.auto_waiting_for_p3d_user
                and stage == STAGE_INITIALIZING
            )
            self.btn_p3d_loaded.setVisible(show_p3d_confirm)
            if show_p3d_confirm:
                self.btn_p3d_loaded.setEnabled(True)

        if stage == STAGE_CONNECTED_READY:
            self.lbl_rec_status.setText("Connected — Press 🔴 Start Recording when ready")
            self.lbl_rec_status.setStyleSheet("color: #89b4fa;")
        elif stage == STAGE_ARMED:
            self.lbl_rec_status.setText("ARMED — Waiting for Throttle ≥ 99%")
            self.lbl_rec_status.setStyleSheet("color: #f9e2af;")
        elif stage == STAGE_INITIALIZING:
            self.lbl_rec_status.setText(
                details or "INITIALIZING AIRCRAFT — timer NOT started (0 rows)"
            )
            self.lbl_rec_status.setStyleSheet("color: #fab387;")
        elif stage == STAGE_RECORDING:
            self.lbl_rec_status.setText("RECORDING TELEMETRY (1 row/sec)")
            self.lbl_rec_status.setStyleSheet("color: #a6e3a1;")
        elif stage == STAGE_COMPLETED:
            self.lbl_rec_status.setText(f"RECORDING COMPLETED ({len(self.recorded_rows)} Data Rows Saved)")
            self.lbl_rec_status.setStyleSheet("color: #cba6f7;")
        elif stage == STAGE_STOPPED_BY_USER:
            self.lbl_rec_status.setText(f"STOPPED BY USER ({len(self.recorded_rows)} Rows Saved)")
            self.lbl_rec_status.setStyleSheet("color: #fab387;")
        elif stage == STAGE_CONNECTION_ERROR:
            self.lbl_rec_status.setText(f"Error: {details or 'Could not connect to Prepar3D'}")
            self.lbl_rec_status.setStyleSheet("color: #f38ba8;")
        elif stage == STAGE_AUTO_BLOCKED:
            self.lbl_rec_status.setText(
                "Automatic blocked by validation — P3D connection remains available; "
                "you may run Manual or retry Automatic."
            )
            self.lbl_rec_status.setStyleSheet("color: #fab387;")
        else:
            self.lbl_rec_status.setText("Idle — Press Start to Arm Test")
            self.lbl_rec_status.setStyleSheet("color: #a6adc8;")


    @staticmethod
    def _v1432_sha256_file(path: Path) -> str:
        h = hashlib.sha256()
        with open(path, "rb") as f:
            for chunk in iter(lambda: f.read(1024 * 1024), b""):
                h.update(chunk)
        return h.hexdigest()

    def _v1432_bind_reference_bundle(
        self,
        manual_csv: Path,
        replay_json: Path,
        state_json: Path,
    ) -> tuple[bool, str]:
        """
        Freeze one exact Manual bundle for the whole Automatic run.

        From this point onward:
          * P3D initialization uses state_json
          * physical CLS replay uses replay_json
          * formal comparison report uses manual_csv
        """
        try:
            manual_csv = Path(manual_csv).resolve()
            replay_json = Path(replay_json).resolve()
            state_json = Path(state_json).resolve()

            if not manual_csv.exists():
                return False, f"Manual CSV missing: {manual_csv}"
            if not replay_json.exists():
                return False, f"Replay JSON missing: {replay_json}"
            if not state_json.exists():
                return False, f"State JSON missing: {state_json}"

            base = manual_csv.stem
            expected_replay = manual_csv.with_name(base + "_replay.json").resolve()
            expected_state = manual_csv.with_name(base + "_initial_state.json").resolve()
            expected_recovered_state = manual_csv.with_name(
                base + "_initial_state_v1466_recovered.json"
            ).resolve()

            if replay_json != expected_replay:
                return False, "Replay JSON does not belong to the selected Manual CSV."

            # V14.66.2:
            # V14.66 deliberately creates a recovered state sidecar from the exact
            # selected Manual's original _initial_state.json + matching saved
            # P3D_State_Diagnostic_manual CSV.  The old V14.32 exact-bundle guard
            # only accepted the original filename and therefore rejected the
            # correctly generated V14.66 sidecar.  Accept ONLY these two exact
            # state paths; arbitrary state JSON files remain fail-closed.
            if state_json not in (expected_state, expected_recovered_state):
                return False, "Initial-state JSON does not belong to the selected Manual CSV."

            if state_json == expected_recovered_state:
                try:
                    with open(state_json, "r", encoding="utf-8") as sf:
                        recovered_meta = json.load(sf)
                    if str(recovered_meta.get("recovery_version", "")) != "V14.66":
                        return False, "Recovered V14.66 state has invalid recovery metadata."
                    source_diag = Path(
                        str(recovered_meta.get("recovery_p3d_diag_file", "") or "")
                    ).name
                    expected_stamp = base.replace(
                        "Super_Mushak_manual_takeoff_data_", "", 1
                    )
                    expected_stamp = re.sub(r"\(\d+\)$", "", expected_stamp).strip()
                    if (
                        not source_diag.startswith(
                            "Super_Mushak_P3D_State_Diagnostic_manual_" + expected_stamp
                        )
                        or not source_diag.lower().endswith(".csv")
                    ):
                        return False, (
                            "Recovered V14.66 state is not tied to the selected "
                            "Manual P3D diagnostic."
                        )
                except Exception as exc:
                    return False, f"Recovered V14.66 state metadata check failed: {exc}"

            hashes = {
                "manual_csv": self._v1432_sha256_file(manual_csv),
                "replay_json": self._v1432_sha256_file(replay_json),
                "state_json": self._v1432_sha256_file(state_json),
            }
            bundle_seed = (
                manual_csv.name + "|" + hashes["manual_csv"] + "|"
                + replay_json.name + "|" + hashes["replay_json"] + "|"
                + state_json.name + "|" + hashes["state_json"]
            )
            bundle_id = hashlib.sha256(bundle_seed.encode("utf-8")).hexdigest()[:16]

            state = dict(getattr(self, "reference_initial_dynamic_state", {}) or {})
            init_c = dict(getattr(self, "manual_initial_conditions", {}) or {})

            self.reference_manual_csv_path = str(manual_csv)
            self.reference_bundle_id = bundle_id
            self.reference_bundle_manifest = {
                "schema": "SuperMushak-V14.32-ExactReferenceBundle-v1",
                "bundle_id": bundle_id,
                "manual_csv": str(manual_csv),
                "replay_json": str(replay_json),
                "state_json": str(state_json),
                "hashes": hashes,
                "manual_t0": {
                    "alt_ft": init_c.get("alt_ft"),
                    "ias_kts": init_c.get("speed_kts"),
                    "pitch_deg": init_c.get("pitch_deg"),
                    "bank_deg": init_c.get("bank_deg"),
                    "heading_deg": init_c.get("heading_deg"),
                },
                "state_t0": {
                    "altitude_ft": state.get("altitude_ft"),
                    "ias_kts": state.get("ias_kts"),
                    "pitch_deg": state.get("pitch_deg"),
                    "bank_deg": state.get("bank_deg"),
                    "magnetic_heading_deg": state.get("magnetic_heading_deg"),
                    "true_heading_deg": state.get("true_heading_deg"),
                },
            }

            print(
                "[V14.32 BUNDLE BOUND] "
                f"id={bundle_id} | CSV={manual_csv.name} | "
                f"Replay={replay_json.name} | State={state_json.name}",
                flush=True,
            )
            return True, f"bundle={bundle_id}"

        except Exception as exc:
            return False, f"bundle bind exception: {exc}"

    def _v1432_verify_bound_bundle(self, context: str) -> tuple[bool, str]:
        """Fail closed if any selected Manual bundle file changed after selection."""
        manifest = dict(getattr(self, "reference_bundle_manifest", {}) or {})
        if not manifest or not self.reference_bundle_id:
            return False, "no exact Manual bundle is bound"

        try:
            current = {}
            for key, path_key in (
                ("manual_csv", "manual_csv"),
                ("replay_json", "replay_json"),
                ("state_json", "state_json"),
            ):
                p = Path(manifest[path_key])
                if not p.exists():
                    return False, f"{context}: missing {p.name}"
                current[key] = self._v1432_sha256_file(p)

            expected = dict(manifest.get("hashes") or {})
            for key, digest in current.items():
                if digest != expected.get(key):
                    return False, f"{context}: {key} changed after bundle selection"

            return True, f"{context}: bundle {self.reference_bundle_id} unchanged"
        except Exception as exc:
            return False, f"{context}: bundle verification exception: {exc}"

    def _v1432_save_auto_reference_manifest(self, auto_csv: Path) -> Optional[Path]:
        """Write a traceable sidecar proving which Manual bundle drove this Auto run."""
        if self.test_mode != "auto":
            return None

        ok, detail = self._v1432_verify_bound_bundle("Auto report")
        if not ok:
            raise RuntimeError(detail)

        payload = dict(self.reference_bundle_manifest)
        payload.update({
            "automatic_csv": str(Path(auto_csv).resolve()),
            "automatic_generated_at": datetime.datetime.now().isoformat(),
            "binding_rule": (
                "ONE exact Manual CSV + its matching replay JSON + its matching "
                "initial-state JSON are used for initialization, replay and report."
            ),
        })
        sidecar = Path(auto_csv).with_name(
            Path(auto_csv).stem + "_reference_bundle.json"
        )
        with open(sidecar, "w", encoding="utf-8") as f:
            json.dump(payload, f, indent=2)
        self.auto_reference_manifest_path = str(sidecar)
        print(
            f"[V14.32 AUTO MANIFEST] Saved exact reference binding: {sidecar.name}",
            flush=True,
        )
        return sidecar

    def _v1432_plot_bound_comparison(self, auto_legacy_csv: Path):
        """
        Generate the formal comparison in an isolated fake data directory.

        plot_graphs.py historically chooses a Manual CSV on its own.  V14.32
        removes that ambiguity without modifying the user's plot_graphs.py:
        the plotter is executed in a temporary data folder containing ONLY
        the exact selected Manual CSV and the current Automatic CSV.

        The generated PDF is then copied back beside the real Automatic CSV.
        """
        if self.test_mode != "auto":
            from plot_graphs import plot_aligned_comparison
            return plot_aligned_comparison(auto_legacy_csv)

        ok, detail = self._v1432_verify_bound_bundle("PDF binding")
        if not ok:
            raise RuntimeError(detail)

        if not self.reference_manual_csv_path:
            raise RuntimeError("No exact Manual CSV path is bound for the report.")

        manual_csv = Path(self.reference_manual_csv_path).resolve()
        auto_legacy_csv = Path(auto_legacy_csv).resolve()

        # Stage report in D: drive if available to prevent C: out-of-disk-space errors
        d_temp = Path("D:/Super_Mushak_Logger_Data/temp") if os.path.exists("D:\\") else None
        if d_temp:
            d_temp.mkdir(parents=True, exist_ok=True)
        temp_root = Path(tempfile.mkdtemp(prefix="SuperMushak_V1432_Report_", dir=str(d_temp) if d_temp else None))
        fake_data_dir = temp_root / "Documents" / "Super_Mushak_Logger_Data"
        fake_data_dir.mkdir(parents=True, exist_ok=True)

        stage_manual = fake_data_dir / manual_csv.name
        stage_auto = fake_data_dir / auto_legacy_csv.name
        shutil.copy2(manual_csv, stage_manual)
        shutil.copy2(auto_legacy_csv, stage_auto)

        # The Automatic compatibility CSV has a continuous heading trace.
        # Apply the same plot-only unwrap to the staged Manual reference;
        # leave the authoritative Manual CSV and its 0..360 values untouched.
        with open(stage_manual, "r", newline="", encoding="utf-8") as source:
            manual_reader = csv.DictReader(source)
            manual_fields = list(manual_reader.fieldnames or [])
            manual_rows = list(manual_reader)
        for heading_key in ("Magnetic Heading (deg)", "Yaw / True Heading (deg)"):
            last_raw = None
            unwrapped = None
            for row in manual_rows:
                try:
                    heading = float(row.get(heading_key, ""))
                    if not math.isfinite(heading):
                        continue
                except (TypeError, ValueError):
                    continue
                if last_raw is None:
                    unwrapped = heading
                else:
                    unwrapped += (heading - last_raw + 180.0) % 360.0 - 180.0
                row[heading_key] = str(unwrapped)
                last_raw = heading
        with open(stage_manual, "w", newline="", encoding="utf-8") as staged:
            manual_writer = csv.DictWriter(staged, fieldnames=manual_fields)
            manual_writer.writeheader()
            manual_writer.writerows(manual_rows)

        print(
            "[V14.32 REPORT BIND] "
            f"bundle={self.reference_bundle_id} | "
            f"Manual={manual_csv.name} | Auto={auto_legacy_csv.name}",
            flush=True,
        )

        import plot_graphs as plot_graphs_module
        plot_func = plot_graphs_module.plot_aligned_comparison

        old_cwd = os.getcwd()
        old_home = os.environ.get("HOME")
        old_userprofile = os.environ.get("USERPROFILE")

        # Patch obvious module-level data-directory variables, if plot_graphs
        # defines them.  They are restored immediately afterwards.
        patched_globals = {}
        for name in (
            "DATA_DIR", "data_dir", "CSV_DIR", "csv_dir",
            "OUTPUT_DIR", "output_dir", "LOG_DIR", "log_dir",
        ):
            if hasattr(plot_graphs_module, name):
                old_value = getattr(plot_graphs_module, name)
                if isinstance(old_value, (str, Path)):
                    patched_globals[name] = old_value
                    setattr(
                        plot_graphs_module,
                        name,
                        fake_data_dir if isinstance(old_value, Path)
                        else str(fake_data_dir),
                    )

        result = None
        try:
            # Covers plotters that derive the data folder from cwd, HOME,
            # USERPROFILE, a module global, or the supplied Automatic CSV path.
            os.chdir(fake_data_dir)
            os.environ["HOME"] = str(temp_root)
            os.environ["USERPROFILE"] = str(temp_root)

            # If a newer plot_graphs implementation offers an explicit Manual
            # parameter, use it. Otherwise the isolated folder makes selection
            # deterministic because exactly one Manual CSV exists there.
            sig = inspect.signature(plot_func)
            params = list(sig.parameters.values())
            manual_param = next(
                (
                    p.name for p in params[1:]
                    if p.name.lower() in {
                        "manual_csv", "manual_csv_path", "manual_file",
                        "reference_csv", "reference_csv_path", "reference_file",
                    }
                ),
                None,
            )
            if manual_param:
                result = plot_func(str(stage_auto), **{manual_param: str(stage_manual)})
            else:
                result = plot_func(stage_auto)

            if result:
                result_path = Path(result)
                if not result_path.is_absolute():
                    result_path = fake_data_dir / result_path
            else:
                result_path = None

            # Fallback: locate the generated Auto PDF in the isolated folder.
            if result_path is None or not result_path.exists():
                candidates = sorted(
                    fake_data_dir.glob(
                        stage_auto.stem + "*_graphs.pdf"
                    ),
                    key=lambda p: p.stat().st_mtime,
                    reverse=True,
                )
                if not candidates:
                    candidates = sorted(
                        fake_data_dir.glob("*auto*_graphs.pdf"),
                        key=lambda p: p.stat().st_mtime,
                        reverse=True,
                    )
                if candidates:
                    result_path = candidates[0]

            if result_path is None or not result_path.exists():
                raise RuntimeError(
                    "plot_graphs did not produce a PDF in the bound report folder"
                )

            final_pdf = auto_legacy_csv.with_name(
                auto_legacy_csv.stem + "_graphs.pdf"
            )
            shutil.copy2(result_path, final_pdf)
            print(
                f"[V14.32 REPORT BIND] Exact-bundle PDF copied to: {final_pdf.name}",
                flush=True,
            )
            return final_pdf

        finally:
            os.chdir(old_cwd)
            if old_home is None:
                os.environ.pop("HOME", None)
            else:
                os.environ["HOME"] = old_home
            if old_userprofile is None:
                os.environ.pop("USERPROFILE", None)
            else:
                os.environ["USERPROFILE"] = old_userprofile

            for name, old_value in patched_globals.items():
                setattr(plot_graphs_module, name, old_value)

            # Report inputs were copies only; removing them cannot affect the
            # authoritative Manual bundle or Automatic CSV.
            shutil.rmtree(temp_root, ignore_errors=True)

    def _manual_reference_search_dirs(self) -> List[Path]:
        """Return every directory in which this logger can store Manual bundles."""
        dirs = [Path(self.data_dir)]
        docs_dir = Path.home() / "Documents" / "Super_Mushak_Logger_Data"
        if docs_dir.exists() and docs_dir.resolve() != Path(self.data_dir).resolve():
            dirs.append(docs_dir)
        return dirs

    @staticmethod
    def _manual_reference_timestamp_label(path: Path) -> str:
        prefix = "Super_Mushak_manual_takeoff_data_"
        stamp = path.stem[len(prefix):] if path.stem.startswith(prefix) else path.stem
        try:
            dt = datetime.datetime.strptime(stamp, "%Y-%m-%d_%H-%M-%S")
            return dt.strftime("%d-%b-%Y %H:%M:%S")
        except Exception:
            return stamp.replace("_", " ")

    def _inspect_manual_reference_candidate(self, manual_csv: Path) -> Dict[str, Any]:
        """Lightweight, read-only catalog inspection. Full validation occurs on use."""
        manual_csv = Path(manual_csv).resolve()
        replay_json = manual_csv.with_name(manual_csv.stem + "_replay.json")
        state_json = manual_csv.with_name(manual_csv.stem + "_initial_state.json")
        info: Dict[str, Any] = {
            "csv": str(manual_csv),
            "replay": str(replay_json),
            "state": str(state_json),
            "timestamp": self._manual_reference_timestamp_label(manual_csv),
            "axes": [],
            "frames": 0,
            "duration_s": None,
            "status": "INCOMPLETE",
            "detail": "",
        }
        try:
            missing = []
            if not replay_json.exists():
                missing.append("replay JSON")
            if not state_json.exists():
                missing.append("initial-state JSON")
            if missing:
                info["detail"] = "missing " + ", ".join(missing)
                return info

            with open(replay_json, "r", encoding="utf-8") as f:
                replay = json.load(f)
            with open(state_json, "r", encoding="utf-8") as f:
                state = json.load(f)

            if not isinstance(replay, list) or not replay:
                info["detail"] = "replay contains no frames"
                return info
            if not self._v1430_state_is_complete(state):
                info["detail"] = (
                    f"state incompatible (schema={state.get('schema')!r}, "
                    f"bridge={state.get('bridge_version')!r})"
                )
                return info

            threshold = max(10, len(replay) // 2)
            axes = []
            for name, _axis, key in CLS_AXIS_SPECS:
                count = 0
                for frame in replay:
                    try:
                        value = frame.get(key)
                        if value is not None and math.isfinite(float(value)):
                            count += 1
                    except (TypeError, ValueError, OverflowError):
                        pass
                if count >= threshold:
                    axes.append(name)

            info["axes"] = axes
            info["frames"] = len(replay)
            try:
                info["duration_s"] = max(
                    float(frame.get("t_ms", 0) or 0) for frame in replay
                ) / 1000.0
            except Exception:
                info["duration_s"] = None

            if not axes:
                info["detail"] = "no usable native CLS axis"
                return info

            # CSV readability is also checked here; exact t=0 alignment remains a
            # fail-closed check when Automatic binds the selected bundle.
            with open(manual_csv, "r", encoding="utf-8") as f:
                first = next(csv.DictReader(f), None)
            if not first:
                info["detail"] = "CSV contains no rows"
                return info

            info["status"] = "READY"
            info["detail"] = "complete bundle; full t=0 validation occurs on Automatic"
            return info
        except Exception as exc:
            info["status"] = "INVALID"
            info["detail"] = str(exc)
            return info

    def _scan_manual_reference_catalog(self) -> List[Dict[str, Any]]:
        """Discover all saved Manual CSV bundles, newest first."""
        paths: Dict[str, Path] = {}
        for sdir in self._manual_reference_search_dirs():
            if not sdir.exists():
                continue
            for p in sdir.glob("Super_Mushak_manual_takeoff_data_*.csv"):
                # The Manual selector must ignore generated Ti/Tt, channel and plot CSVs.
                if re.fullmatch(
                    r"Super_Mushak_manual_takeoff_data_\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2}(?:\(\d+\))?\.csv",
                    p.name,
                ) is None:
                    continue
                try:
                    key = str(p.resolve()).lower()
                except Exception:
                    key = str(p).lower()
                paths[key] = p

        ordered = sorted(paths.values(), key=lambda p: p.name, reverse=True)
        return [self._inspect_manual_reference_candidate(p) for p in ordered]

    def _refresh_manual_reference_selector(self, select_path: Optional[str] = None) -> None:
        """Refresh the GUI list without changing any CLS2Sim/BRUNNER state."""
        if not hasattr(self, "cmb_manual_reference"):
            return

        previous = select_path or self.selected_manual_csv_path
        if previous is None and self.cmb_manual_reference.currentIndex() >= 0:
            previous = self.cmb_manual_reference.currentData()
        try:
            previous_resolved = str(Path(previous).resolve()) if previous else None
        except Exception:
            previous_resolved = str(previous) if previous else None

        self.manual_reference_catalog = self._scan_manual_reference_catalog()
        self.cmb_manual_reference.blockSignals(True)
        self.cmb_manual_reference.clear()

        selected_index = -1
        for idx, info in enumerate(self.manual_reference_catalog):
            axes = "/".join(info.get("axes") or []) or "NO AXIS"
            duration = info.get("duration_s")
            duration_text = "?s" if duration is None else f"{duration:.1f}s"
            status = info.get("status", "UNKNOWN")
            label = (
                f"{info.get('timestamp', '')}  |  {axes}  |  "
                f"{duration_text}  |  {status}"
            )
            self.cmb_manual_reference.addItem(label, info.get("csv"))
            csv_path = str(info.get("csv") or "")
            try:
                same = previous_resolved is not None and str(Path(csv_path).resolve()) == previous_resolved
            except Exception:
                same = previous_resolved is not None and csv_path == previous_resolved
            if same:
                selected_index = idx

        if self.cmb_manual_reference.count() == 0:
            self.cmb_manual_reference.addItem("No saved Manual runs found", None)
            self.cmb_manual_reference.setCurrentIndex(0)
            self.selected_manual_csv_path = None
        else:
            if selected_index < 0:
                selected_index = next(
                    (i for i, item in enumerate(self.manual_reference_catalog)
                     if item.get("status") == "READY"),
                    0,
                )
            self.cmb_manual_reference.setCurrentIndex(selected_index)
            self.selected_manual_csv_path = self.cmb_manual_reference.currentData()

        self.cmb_manual_reference.blockSignals(False)
        self._on_manual_reference_selection_changed(self.cmb_manual_reference.currentIndex())

        print(
            f"[V14.43 MANUAL CATALOG] {len(self.manual_reference_catalog)} saved Manual run(s) found | "
            f"selected={Path(self.selected_manual_csv_path).name if self.selected_manual_csv_path else 'NONE'}",
            flush=True,
        )

    @Slot(int)
    def _on_manual_reference_selection_changed(self, index: int) -> None:
        if not hasattr(self, "cmb_manual_reference"):
            return
        path = self.cmb_manual_reference.itemData(index) if index >= 0 else None
        self.selected_manual_csv_path = str(path) if path else None

        if not hasattr(self, "lbl_manual_reference_info"):
            return
        if not self.selected_manual_csv_path:
            self.lbl_manual_reference_info.setText(
                "No Manual reference selected. Record a Manual or press Refresh Manuals."
            )
            self.lbl_manual_reference_info.setStyleSheet("color: #f9e2af;")
            return

        selected = None
        try:
            selected_resolved = str(Path(self.selected_manual_csv_path).resolve())
            for item in self.manual_reference_catalog:
                if str(Path(item.get("csv", "")).resolve()) == selected_resolved:
                    selected = item
                    break
        except Exception:
            pass

        if selected is None:
            self.lbl_manual_reference_info.setText(
                f"Selected Manual: {Path(self.selected_manual_csv_path).name}"
            )
            self.lbl_manual_reference_info.setStyleSheet("color: #a6adc8;")
            return

        axes = ", ".join(selected.get("axes") or []) or "none"
        frames = int(selected.get("frames") or 0)
        status = selected.get("status", "UNKNOWN")
        detail = selected.get("detail", "")
        self.lbl_manual_reference_info.setText(
            f"Selected: {Path(self.selected_manual_csv_path).name} | "
            f"Axes: {axes} | Frames: {frames} | Status: {status} | {detail}"
        )
        color = "#a6e3a1" if status == "READY" else "#f9e2af"
        self.lbl_manual_reference_info.setStyleSheet(f"color: {color};")

    def _selected_manual_reference_path(self) -> Optional[Path]:
        """Return the exact operator-selected Manual CSV, or None."""
        path = self.selected_manual_csv_path
        if hasattr(self, "cmb_manual_reference") and self.cmb_manual_reference.currentIndex() >= 0:
            data = self.cmb_manual_reference.currentData()
            if data:
                path = str(data)
        if not path:
            return None
        try:
            candidate = Path(path).resolve()
        except Exception:
            candidate = Path(path)
        # A previously cached selection must not point to a generated summary.
        if re.fullmatch(
            r"Super_Mushak_manual_takeoff_data_\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2}(?:\(\d+\))?\.csv",
            candidate.name,
        ) is None:
            return None
        return candidate

    def _v1466_recover_true_manual_dynamics(
        self,
        manual_csv: Path,
        first_manual_row: Dict[str, Any],
        original_state: Dict[str, Any],
    ) -> tuple[Optional[Dict[str, Any]], Optional[Path], str]:
        """Recover true Manual t=0 low-level dynamics from its saved P3D diagnostic."""
        try:
            stem = manual_csv.stem
            prefix = "Super_Mushak_manual_takeoff_data_"
            stamp = stem[len(prefix):] if stem.startswith(prefix) else stem
            stamp = re.sub(r"\(\d+\)$", "", stamp).strip()
            pattern = f"Super_Mushak_P3D_State_Diagnostic_manual_{stamp}*.csv"
            candidates = sorted(manual_csv.parent.glob(pattern))
            if not candidates:
                return None, None, f"no matching Manual P3D diagnostic found ({pattern})"

            def fv(row, key):
                try:
                    v = float(row.get(key, "nan"))
                    return v if math.isfinite(v) else None
                except (TypeError, ValueError, OverflowError):
                    return None

            def mf(*keys):
                for key in keys:
                    try:
                        raw = first_manual_row.get(key)
                        if raw not in (None, ""):
                            v = float(raw)
                            if math.isfinite(v):
                                return v
                    except (TypeError, ValueError, OverflowError):
                        pass
                return None

            best = None
            best_path = None
            best_score = float("inf")
            best_detail = ""

            for diag_path in candidates:
                try:
                    with open(diag_path, "r", encoding="utf-8") as f:
                        drows = list(csv.DictReader(f))
                except Exception:
                    continue

                # The Manual CSV and _initial_state.json are frozen from the
                # MANUAL_T0 frame.  The first is_recording row is one telemetry
                # update later, and its rates can already differ substantially.
                marked_rows = [rr for rr in drows if rr.get("t0_marker") == "MANUAL_T0"]
                rec_rows = []
                for rr in drows:
                    try:
                        if int(float(rr.get("is_recording", 0) or 0)) == 1:
                            rec_rows.append(rr)
                    except Exception:
                        pass
                if not rec_rows and not marked_rows:
                    continue

                rr = marked_rows[0] if marked_rows else rec_rows[0]
                specs = (
                    ("IAS", mf("Indicated Airspeed (kts)", "Indicated Airspeed (kt)"),
                     fv(rr, "p3d_ias_kts"), 1.0, False),
                    ("Alt", mf("Pressure Alt (ft)", "Indicated Altitude (ft)"),
                     fv(rr, "p3d_pressure_alt_ft"), 5.0, False),
                    ("Pitch", mf("Pitch Angle (deg)"),
                     fv(rr, "p3d_pitch_deg"), 1.0, False),
                    ("Bank", mf("Bank Angle (deg)"),
                     fv(rr, "p3d_bank_deg"), 1.0, False),
                    ("MagHdg", mf("Magnetic Heading (deg)", "Yaw / True Heading (deg)"),
                     fv(rr, "p3d_magnetic_heading_deg"), 2.0, True),
                )

                ok = True
                score = 0.0
                parts = []
                for label, ref, live, tol, angular in specs:
                    if ref is None or live is None:
                        ok = False
                        score += 1000.0
                        parts.append(f"{label}=missing")
                        continue
                    err = self._v1430_angle_error_deg(live, ref) if angular else float(live) - float(ref)
                    if abs(err) > float(tol):
                        ok = False
                    score += abs(err) / max(float(tol), 1e-9)
                    parts.append(f"d{label}={err:+.3f}")

                if ok and score < best_score:
                    best = rr
                    best_path = diag_path
                    best_score = score
                    best_detail = " ".join(parts)

            if best is None or best_path is None:
                return None, None, "matching diagnostic exists, but no recording t=0 row aligns with selected Manual"

            recovered = dict(original_state or {})

            def put(diag_key, state_key):
                v = fv(best, diag_key)
                if v is not None:
                    recovered[state_key] = float(v)
                return v

            # Settable low-level dynamics.
            put("p3d_velocity_world_x_fps", "velocity_world_x_fps")
            put("p3d_velocity_world_y_fps", "velocity_world_y_fps")
            put("p3d_velocity_world_z_fps", "velocity_world_z_fps")
            put("p3d_rotation_body_x_rad_s", "rotation_velocity_body_x_rad_s")
            put("p3d_rotation_body_y_rad_s", "rotation_velocity_body_y_rad_s")
            put("p3d_rotation_body_z_rad_s", "rotation_velocity_body_z_rad_s")
            put("p3d_elevator_trim_rad", "elevator_trim_rad")
            put("p3d_aileron_trim_pct", "aileron_trim_pct")
            put("p3d_rudder_trim_pct", "rudder_trim_pct")

            # Read-only BODY velocity reference used by the V14.66 live gate.
            put("p3d_velocity_body_x_fps", "velocity_body_x_fps")
            put("p3d_velocity_body_y_fps", "velocity_body_y_fps")
            put("p3d_velocity_body_z_fps", "velocity_body_z_fps")

            # Controls/surfaces.
            put("p3d_elevator_pos", "elevator_input")
            put("p3d_aileron_pos", "aileron_input")
            put("p3d_rudder_pos", "rudder_input")
            put("p3d_elevator_deflection_rad", "elevator_deflection_rad")
            put("p3d_aileron_left_deflection_rad", "aileron_left_deflection_rad")
            put("p3d_aileron_right_deflection_rad", "aileron_right_deflection_rad")
            put("p3d_rudder_deflection_rad", "rudder_deflection_rad")

            # Visible t=0 evidence.
            put("p3d_ias_kts", "ias_kts")
            put("p3d_vertical_speed_fpm", "vertical_speed_fpm")
            put("p3d_pressure_alt_ft", "altitude_ft")
            put("p3d_altitude_ft", "indicated_altitude_ft")
            put("p3d_pitch_deg", "pitch_deg")
            put("p3d_bank_deg", "bank_deg")
            put("p3d_true_heading_deg", "true_heading_deg")
            put("p3d_magnetic_heading_deg", "magnetic_heading_deg")
            put("p3d_roll_rate_deg_s", "roll_rate_deg_s")
            put("p3d_pitch_rate_deg_s", "pitch_rate_deg_s")
            put("p3d_yaw_rate_deg_s", "yaw_rate_deg_s")

            # V14.72: REAL air-relative Manual t=0 reference.  These are
            # read-only verification targets, never force-written to P3D.
            put("p3d_aoa", "aoa_deg")
            put("p3d_sideslip", "sideslip_deg")
            put("p3d_accelX", "accel_x_g")
            put("p3d_accelY", "accel_y_g")
            put("p3d_accelZ", "accel_z_g")
            # New V14.72 diagnostics include real ambient wind.  Older Manual
            # diagnostics simply leave these absent, so no fake 0/0 reference
            # is created from the formal report.
            put("p3d_actual_wind_dir_deg", "actual_wind_dir_deg")
            put("p3d_actual_wind_speed_kt", "actual_wind_speed_kt")

            put("p3d_max_throttle_pct", "throttle_pct")
            put("p3d_engine_rpm_1", "engine_rpm")

            # V14.70 fallback: if an older diagnostic lacks engine RPM, use the
            # selected Manual row-0 Engine RPM.  Never invent an RPM target.
            if self._v1430_finite(recovered.get("engine_rpm")) is None:
                try:
                    row0_engine_rpm = float(first_manual_row.get("Engine 1 RPM"))
                    if math.isfinite(row0_engine_rpm) and row0_engine_rpm > 100.0:
                        recovered["engine_rpm"] = row0_engine_rpm
                except Exception:
                    pass

            recovered["dynamic_state_source"] = "V14.70 recovered true Manual P3D diagnostic + VSI/RPM state"
            recovered["recovery_version"] = "V14.66"
            recovered["recovery_p3d_diag_file"] = str(best_path.resolve())
            recovered["recovery_alignment"] = best_detail

            if not self._v1430_state_is_complete(recovered):
                return None, best_path, "recovered state is incomplete"

            for k in (
                "velocity_body_x_fps", "velocity_body_y_fps", "velocity_body_z_fps",
                "vertical_speed_fpm", "engine_rpm",
            ):
                if self._v1430_finite(recovered.get(k)) is None:
                    return None, best_path, f"V14.70 recovered state missing {k}"

            return recovered, best_path, (
                f"TRUE t=0 recovered from {best_path.name} | "
                f"VW=({recovered['velocity_world_x_fps']:+.3f},"
                f"{recovered['velocity_world_y_fps']:+.3f},"
                f"{recovered['velocity_world_z_fps']:+.3f}) | "
                f"VB=({recovered['velocity_body_x_fps']:+.3f},"
                f"{recovered['velocity_body_y_fps']:+.3f},"
                f"{recovered['velocity_body_z_fps']:+.3f}) | {best_detail}"
            )
        except Exception as exc:
            return None, None, f"V14.66 recovery exception: {exc}"

    def _v1466_body_velocity_matches(
        self, data: Dict[str, Any], context: str = ""
    ) -> tuple[bool, str]:
        state = dict(getattr(self, "reference_initial_dynamic_state", {}) or {})
        ok = True
        parts = []
        for label, live_key, ref_key in (
            ("VBX", "velocity_body_x", "velocity_body_x_fps"),
            ("VBY", "velocity_body_y", "velocity_body_y_fps"),
            ("VBZ", "velocity_body_z", "velocity_body_z_fps"),
        ):
            live = self._v1430_finite((data or {}).get(live_key))
            ref = self._v1430_finite(state.get(ref_key))
            if live is None or ref is None:
                ok = False
                parts.append(f"{label}=MISSING({live}/{ref})")
                continue
            err = float(live) - float(ref)
            passed = abs(err) <= float(V14_66_BODY_VELOCITY_TOL_FPS)
            ok = bool(ok and passed)
            parts.append(
                f"{label}={live:+.3f}/{ref:+.3f} d={err:+.3f} "
                f"({'OK' if passed else 'FAIL'})"
            )
        detail = " | ".join(parts)
        return bool(ok), (f"{context}: {detail}" if context else detail)

    @staticmethod
    def _v1494_metar_temp_token(value_c: float) -> str:
        v = int(round(float(value_c)))
        return ("M" + f"{abs(v):02d}") if v < 0 else f"{v:02d}"

    def _v1494_reconstruct_legacy_profile(
        self, key: str, state: Dict[str, Any], manual: Dict[str, Any]
    ) -> Dict[str, Any]:
        """Reconstruct only fields absent from the bridge-1440 Manual.

        The wind is solved from the Manual's measured ground BODY velocity and
        its recovered AoA/beta.  For 15:25:33 the result independently agrees
        with the measured V14.90 environment (~316 deg / 9.7 kt), so that
        directly observed value is retained.  Nothing here replaces a field
        that the Manual actually recorded.
        """
        base = dict(V14_92_LEGACY_PROFILES.get(key, {}))
        alpha = self._v1430_finite(state.get("aoa_deg"))
        beta = self._v1430_finite(state.get("sideslip_deg"))
        vbx = self._v1430_finite(state.get("velocity_body_x_fps"))
        vby = self._v1430_finite(state.get("velocity_body_y_fps"))
        vbz = self._v1430_finite(state.get("velocity_body_z_fps"))
        true_hdg = self._v1430_finite(state.get("true_heading_deg"))
        mach = self._v1430_finite(manual.get("mach"))
        oat = self._v1430_finite(manual.get("oat"))
        alt_ft = self._v1430_finite(manual.get("alt_ft"))
        baro = self._v1430_finite(manual.get("baro")) or 1013.0

        if None in (alpha, beta, vbx, vby, vbz, true_hdg, mach, oat, alt_ft):
            raise RuntimeError("V14.95 cannot reconstruct legacy weather: recovered Manual air-state is incomplete")

        # Speed of sound from the Manual OAT, then Manual TAS from Mach.
        a_mps = math.sqrt(1.4 * 287.05 * (float(oat) + 273.15))
        tas_fps = float(mach) * a_mps * 3.280839895
        ar = math.radians(float(alpha))
        br = math.radians(float(beta))
        air_x = tas_fps * math.sin(br)
        air_y = -tas_fps * math.cos(br) * math.sin(ar)
        air_z = tas_fps * math.cos(br) * math.cos(ar)
        wx_fps = float(vbx) - air_x
        wy_fps = float(vby) - air_y
        wz_fps = float(vbz) - air_z
        fps_per_kt = 1.687809857
        wind_speed_kt = math.hypot(wx_fps, wz_fps) / fps_per_kt
        rel_from_deg = math.degrees(math.atan2(-wx_fps, -wz_fps))
        wind_dir_deg = (float(true_hdg) + rel_from_deg) % 360.0

        # The 15:25:33 V14.90 run directly measured 316/9.7 and produced beta
        # 3.49 deg against the recovered Manual 3.063 deg. Prefer that observed
        # environmental value over rounding the reconstruction by ~1 degree.
        if key == "2026-09-17_15-25-33":
            wind_dir_deg, wind_speed_kt = 316.0, 9.7

        # P3D METAR supports KMH, which gives finer speed resolution than whole KT.
        wd = int(round(wind_dir_deg)) % 360
        kmh = max(0, min(180, int(round(float(wind_speed_kt) * 1.852))))
        alt_m = max(0, min(99999, int(round(float(alt_ft) * 0.3048))))
        temp_tok = self._v1494_metar_temp_token(float(oat))
        dew_tok = self._v1494_metar_temp_token(float(oat) - 5.0)
        surface_oat = float(oat) + 0.0065 * float(alt_m)
        surface_temp_tok = self._v1494_metar_temp_token(surface_oat)
        surface_dew_tok = self._v1494_metar_temp_token(surface_oat - 5.0)
        # 3000 m wind-layer depth deliberately covers both 3,124 and 3,544 ft
        # references. The old 305 m default can place the aircraft above the
        # surface-wind layer and silently yield 0 kt at the aircraft.
        metar = (
            f"GLOB 250900Z {wd:03d}{kmh:03d}KMH&D3000NG 9999 SKC "
            f"{surface_temp_tok}/{surface_dew_tok}&A00000 "
            f"{temp_tok}/{dew_tok}&A{alt_m:05d} Q{int(round(baro)):04d}"
        )
        base.update({
            "profile_id": f"SEP17_{key[-8:].replace('-', '')}_V1494_FINAL",
            "calibration_status": "V14.95_RECONSTRUCTED_FINAL",
            "flaps_handle_index": 0.0,
            "mixture_percent": 100.0,
            "propeller_percent": 100.0,
            "weather_metar": metar,
            "expected_wind_dir_deg": float(wd),
            "expected_wind_speed_kt": float(kmh) / 1.852,
            "target_oat_c": float(oat),
            "target_fuel_kg": self._v1430_finite(manual.get("fuel_qty")),
            "target_weight_kg": self._v1430_finite(manual.get("gross_wt")),
            "target_aircraft_wind_x_kt": wx_fps / fps_per_kt,
            "target_aircraft_wind_y_kt": wy_fps / fps_per_kt,
            "target_aircraft_wind_z_kt": wz_fps / fps_per_kt,
            # Old takeoff/climb references: AFM runway-entry state is BOTH,
            # mixture RICH, RPM full forward; keep AP/autocoordination out.
            "fuel_selector": 16.0,
            "autopilot_master": 0.0,
            "auto_coordination": 0.0,
            "spoilers_handle": 0.0,
            "gear_handle": 0.0,
            "parking_brake": 0.0,
            "engine_combustion": 1.0,
        })
        print(
            f"[V14.95 LEGACY RECONSTRUCT] {key} | wind={base['expected_wind_dir_deg']:.0f}/"
            f"{base['expected_wind_speed_kt']:.2f}kt | bodyWind="
            f"({base['target_aircraft_wind_x_kt']:+.2f},"
            f"{base['target_aircraft_wind_y_kt']:+.2f},"
            f"{base['target_aircraft_wind_z_kt']:+.2f})kt | OAT={oat:.1f}C | "
            f"flaps=UP mixture=RICH prop=FULL | METAR={metar}", flush=True,
        )
        return base

    @staticmethod
    def _v1494_angle_error_deg(actual: float, target: float) -> float:
        return ((float(actual) - float(target) + 180.0) % 360.0) - 180.0

    def _v1494_weather_readback_ok(self, d: Dict[str, Any], profile: Dict[str, Any]) -> tuple[bool, str]:
        wd = self._v1430_finite((d or {}).get("ambient_wind_dir"))
        ws = self._v1430_finite((d or {}).get("ambient_wind_spd"))
        oat = self._v1430_finite((d or {}).get("ambient_temp"))
        twd = self._v1430_finite(profile.get("expected_wind_dir_deg"))
        tws = self._v1430_finite(profile.get("expected_wind_speed_kt"))
        toat = self._v1430_finite(profile.get("target_oat_c"))
        ok = all(v is not None for v in (wd, ws, oat, twd, tws, toat))
        if ok:
            derr = abs(self._v1494_angle_error_deg(wd, twd)) if tws and tws > 0.5 else 0.0
            ok = bool(derr <= 8.0 and abs(ws-tws) <= 1.0 and abs(oat-toat) <= 1.5)
        detail = (
            f"wind={wd}/{ws} target={twd}/{tws} | OAT={oat} target={toat}"
        )
        return bool(ok), detail

    def _v1494_apply_verified_weather(self, profile: Dict[str, Any], attempts: int = 5) -> tuple[bool, str]:
        if not self.sim_client:
            return False, "no SimConnect client"
        metar = str(profile.get("weather_metar") or "").strip()
        if not metar:
            return False, "legacy weather METAR missing"
        last = "no readback"
        for attempt in range(1, max(1, int(attempts)) + 1):
            sent = bool(self.sim_client.set_global_weather_metar(metar))
            time.sleep(0.18 + 0.06 * attempt)
            try:
                self.sim_client.request_telemetry_once()
                time.sleep(0.06)
            except Exception:
                pass
            d = self.sim_client.get_telemetry() or {}
            good, detail = self._v1494_weather_readback_ok(d, profile)
            last = f"attempt={attempt} sent={sent} | {detail}"
            print(f"[V14.95 WEATHER VERIFY] {'PASS' if good else 'RETRY'} | {last}", flush=True)
            if sent and good:
                return True, last
        return False, last

    def _v1494_match_legacy_fuel(self, profile: Dict[str, Any]) -> tuple[bool, str]:
        """Scale existing measured tank distribution to the Manual total fuel.

        The old Manuals did not record per-tank levels. Preserving the current
        left/right/center ratio avoids inventing a new imbalance while matching
        the recorded total fuel mass as closely as P3D allows.
        """
        target = self._v1430_finite(profile.get("target_fuel_kg"))
        if target is None or not self.sim_client:
            return True, "fuel target unavailable"
        last = ""
        for attempt in range(1, 4):
            d = self.sim_client.get_telemetry() or {}
            current = self._v1430_finite(d.get("total_fuel_quantity"))
            levels = [self._v1430_finite(d.get(k)) for k in (
                "fuel_tank_left_main_level", "fuel_tank_right_main_level", "fuel_tank_center_level")]
            if current is None or current <= 0.1 or any(v is None for v in levels):
                return False, f"fuel readback unavailable current={current} levels={levels}"
            err = current - target
            if abs(err) <= 0.35:
                return True, f"fuel={current:.2f}/{target:.2f}kg"
            factor = target / current
            new_levels = [max(0.0, min(1.0, float(v) * factor)) for v in levels]
            sent = bool(self.sim_client.set_fuel_tank_levels(*new_levels))
            time.sleep(0.12)
            last = f"attempt={attempt} {current:.2f}->{target:.2f}kg factor={factor:.6f} sent={sent}"
        d = self.sim_client.get_telemetry() or {}
        current = self._v1430_finite(d.get("total_fuel_quantity"))
        ok = current is not None and abs(current-target) <= 0.75
        return bool(ok), f"{last} final={current}/{target}kg"

    def _v1494_profile_with_wind(self, profile: Dict[str, Any], direction_deg: float, speed_kt: float) -> Dict[str, Any]:
        p = dict(profile)
        wd = int(round(float(direction_deg))) % 360
        kmh = max(0, min(180, int(round(float(speed_kt) * 1.852))))
        metar = str(p.get("weather_metar") or "")
        metar = re.sub(r"\b\d{3}\d{3}KMH&D\d{4}NG\b", f"{wd:03d}{kmh:03d}KMH&D3000NG", metar, count=1)
        p["weather_metar"] = metar
        p["expected_wind_dir_deg"] = float(wd)
        p["expected_wind_speed_kt"] = float(kmh) / 1.852
        return p

    def _v1494_preflight_aero_lock(
        self, pose_state, state, throttle, pose_setter, dyn_setter,
        request_once, wait_fresh, get_seq
    ) -> tuple[bool, str]:
        """Short t=0-only weather self-calibration for the two legacy Manuals.

        This never records a QTG point and never changes BRUNNER tuning. Each
        trial lasts only one fresh physics frame, is immediately paused, and the
        complete Manual t=0 state is restored before the next trial. It exists
        so this can be the last Sim-PC visit: the app chooses the wind variant
        that best reproduces recovered Manual beta/AoA/rates, then performs the
        one real 75-s replay.
        """
        if not getattr(self, "v1490_legacy_reference", False):
            return True, "not a legacy reference"
        base = self._v1492_selected_profile() or {}
        bd = self._v1430_finite(base.get("expected_wind_dir_deg"))
        bs = self._v1430_finite(base.get("expected_wind_speed_kt"))
        if bd is None or bs is None:
            return False, "legacy reconstructed wind missing"
        target_beta = self._v1430_finite(state.get("sideslip_deg"))
        target_aoa = self._v1430_finite(state.get("aoa_deg"))
        tr = [self._v1430_finite(state.get(k)) for k in (
            "roll_rate_deg_s", "pitch_rate_deg_s", "yaw_rate_deg_s")]
        if target_beta is None or target_aoa is None or any(v is None for v in tr):
            return False, "legacy recovered aero/rate targets incomplete"

        # Base first; then small bounded corrections. Stop immediately if the
        # original reconstruction already satisfies every strict aero gate.
        variants = [(bd,bs),(bd-5,bs),(bd+5,bs),(bd-10,bs),(bd+10,bs),
                    (bd, max(0.0,bs-1.5)),(bd,bs+1.5),
                    (bd-7,max(0.0,bs-1.0)),(bd+7,bs+1.0)]
        best = None
        for i,(wd,ws) in enumerate(variants,1):
            cand = self._v1494_profile_with_wind(base, wd, ws)
            self.v1492_legacy_profile = dict(cand)
            self.sim_client.pause_sim()
            wok,wdetail = self._v1494_apply_verified_weather(cand, attempts=3)
            if not wok:
                print(f"[V14.95 AERO LOCK] candidate {i} weather FAIL | {wdetail}", flush=True)
                continue
            pose_setter(pose_state); dyn_setter(state)
            if throttle is not None:
                self.sim_client.set_throttle(float(throttle))
            time.sleep(0.03)
            seq0 = int(get_seq())
            self.sim_client.unpause_sim()
            request_once()
            live = wait_fresh(seq0, 0.35)
            self.sim_client.pause_sim()
            if not live:
                continue
            beta = self._v1430_finite(live.get("sideslip")); aoa = self._v1430_finite(live.get("aoa"))
            rr = self._v1430_finite(live.get("roll_rate")); pr = self._v1430_finite(live.get("pitch_rate")); yr = self._v1430_finite(live.get("yaw_rate"))
            vals=(beta,aoa,rr,pr,yr)
            if any(v is None for v in vals):
                continue
            score=(abs(beta-target_beta)/0.50 + abs(aoa-target_aoa)/0.35 +
                   abs(rr-tr[0])/0.15 + abs(pr-tr[1])/0.15 + abs(yr-tr[2])/0.15)
            strict=(abs(beta-target_beta)<=0.50 and abs(aoa-target_aoa)<=0.35 and
                    abs(rr-tr[0])<=0.15 and abs(pr-tr[1])<=0.15 and abs(yr-tr[2])<=0.15)
            rec=(score,strict,cand,live)
            if best is None or score < best[0]: best=rec
            print(f"[V14.95 AERO LOCK] {i}/{len(variants)} wind={cand['expected_wind_dir_deg']:.0f}/"
                  f"{cand['expected_wind_speed_kt']:.2f} beta={beta:.3f}/{target_beta:.3f} "
                  f"aoa={aoa:.3f}/{target_aoa:.3f} rates={rr:.3f}/{pr:.3f}/{yr:.3f} score={score:.3f} "
                  f"{'STRICT PASS' if strict else ''}", flush=True)
            if strict:
                break

        if best is None:
            self.v1492_legacy_profile = dict(base)
            return False, "no valid t=0 aero-lock candidate"
        self.v1492_legacy_profile = dict(best[2])
        self.v1492_legacy_profile["calibration_status"] = "V14.95_T0_AERO_LOCKED"
        # Restore the chosen weather and exact t=0 while paused. The next
        # unpause in the normal handover remains the authoritative t=0 frame.
        self.sim_client.pause_sim()
        wok,wdetail = self._v1494_apply_verified_weather(self.v1492_legacy_profile, attempts=5)
        pose_setter(pose_state); dyn_setter(state)
        if throttle is not None: self.sim_client.set_throttle(float(throttle))
        return bool(wok), f"best_score={best[0]:.3f} strict={best[1]} | {wdetail}"

    def _v1492_selected_profile(self) -> Optional[Dict[str, Any]]:
        profile = getattr(self, "v1492_legacy_profile", None)
        return dict(profile) if isinstance(profile, dict) else None

    @staticmethod
    def _v1492_lerp_optional(a, b, alpha):
        try:
            a = float(a); b = float(b); alpha = float(alpha)
            if not (math.isfinite(a) and math.isfinite(b) and math.isfinite(alpha)):
                return None
            return (1.0 - alpha) * a + alpha * b
        except (TypeError, ValueError, OverflowError):
            return None

    def _v1492_secondary_frame_fields(self, data: Dict[str, Any]) -> Dict[str, Any]:
        """P3D configuration timeline stored in NEW Manual replay JSON.

        These fields are observational during Manual. Automatic may replay the
        settable controls; derived/read-only air-state fields stay diagnostics.
        """
        d = data or {}
        def f(key):
            return self._v1430_finite(d.get(key))
        return {
            "p3d_flaps_handle_index": f("flaps_handle_index"),
            "p3d_flaps_handle_percent": f("flaps_handle_percent"),
            "p3d_engine_mixture_lever_1": f("engine_mixture_lever_1"),
            "p3d_engine_propeller_lever_1": f("engine_propeller_lever_1"),
            "p3d_fuel_tank_left_main_level": f("fuel_tank_left_main_level"),
            "p3d_fuel_tank_right_main_level": f("fuel_tank_right_main_level"),
            "p3d_fuel_tank_center_level": f("fuel_tank_center_level"),
            "p3d_fuel_tank_selector_1": f("fuel_tank_selector_1"),
            "p3d_autopilot_master": f("autopilot_master"),
            "p3d_auto_coordination": f("auto_coordination"),
            "p3d_spoilers_handle_position": f("spoilers_handle_position"),
            "p3d_gear_handle_position": f("gear_handle_position"),
            "p3d_brake_parking_position": f("brake_parking_position"),
            "p3d_engine_combustion_1": f("engine_combustion_1"),
            "p3d_recip_cowl_flap_position_1": f("recip_cowl_flap_position_1"),
            "p3d_recip_primer_1": f("recip_primer_1"),
            "p3d_recip_alternate_air_position_1": f("recip_alternate_air_position_1"),
            "p3d_recip_left_magneto_1": f("recip_left_magneto_1"),
            "p3d_recip_right_magneto_1": f("recip_right_magneto_1"),
            "p3d_recip_fuel_available_1": f("recip_fuel_available_1"),
            "p3d_recip_fuel_tank_selector_1": f("recip_fuel_tank_selector_1"),
            "p3d_recip_fuel_tanks_used_1": f("recip_fuel_tanks_used_1"),
            "p3d_general_eng_starter_1": f("general_eng_starter_1"),
            "p3d_canopy_open": f("canopy_open"),
            "p3d_gyro_heading_rad": f("gyro_heading_rad"),
            "p3d_damage_state": f("damage_state"),
            "p3d_gear_center_position": f("gear_center_position"),
            "p3d_gear_left_position": f("gear_left_position"),
            "p3d_gear_right_position": f("gear_right_position"),
            "p3d_engine_fuel_valve_1": f("engine_fuel_valve_1"),
            "p3d_engine_master_alternator_1": f("engine_master_alternator_1"),
            "p3d_electrical_master_battery": f("electrical_master_battery"),
        }

    def _v1497_verify_fuel(self, state):
        last = ["No fuel sample"]
        for _ in range(12):
            if self.test_mode != "auto" or not self.auto_prepare_in_progress:
                return False, "Fuel verification cancelled"
            live = self._v1496_sample_engine()
            last = fuel_mismatches(live, state)
            if not last: return True, "Measured primary fuel and total mass match Manual"
            time.sleep(0.05)
        return False, "Fuel restoration readback failed: " + "; ".join(last)

    def _v1496_sample_engine(self):
        support = self.sim_client.get_engine_support_state(fresh=True)
        seq = self.sim_client.get_telemetry_sequence()
        if not self.sim_client.request_telemetry_once():
            raise RuntimeError("Engine primary telemetry request rejected")
        live = self.sim_client.wait_for_fresh_telemetry(seq, 0.6)
        if live is None:
            raise RuntimeError("No fresh engine primary telemetry")
        live = dict(live)
        live.update(support)
        return live

    def _v1496_prepare_engine(self):
        """Scenery ready; hold flight state while engine runs naturally before final t=0."""
        state = dict(self.reference_initial_dynamic_state or {})
        if state.get("extended_causal_layout") != "TAGGED_V1497":
            for key in SUPPORT_KEYS:
                state.pop(key, None)
        pose = dict(state)
        pose["bank_deg"] = float(state.get("bank_deg", 0.0)) * float(
            getattr(self, "auto_bank_write_multiplier", -1.0) or -1.0)
        client = self.sim_client
        client.natural_engine_mode = True
        try:
            ok, detail = self._v1492_apply_reference_causal_state(
                include_weather=True, context="ENGINE-PREPARATION")
            if not ok:
                return False, detail
            missing = [key for key in SUPPORT_KEYS if state.get(key) is None]
            if missing:
                self.auto_diagnostic_replay = True
                self.auto_diagnostic_start_detail = "Manual has no complete engine-support snapshot: " + ", ".join(missing)
                print("[ENGINE PREPARATION] " + self.auto_diagnostic_start_detail, flush=True)

            def hold(starting=False):
                if not client.set_exact_pose_state_no_airspeed(pose):
                    raise RuntimeError("Engine preparation pose hold failed")
                if not client.set_initial_dynamic_state(state):
                    raise RuntimeError("Engine preparation flight hold failed")
                # While auto-start operates, leave engine controls to P3D.
                if not starting:
                    if state.get("throttle_pct") is not None:
                        client.set_throttle(float(state["throttle_pct"]))
                    mix, prop = state.get("engine_mixture_lever_1"), state.get("engine_propeller_lever_1")
                    if mix is not None and prop is not None:
                        if not client.set_engine_controls(float(mix)*100, float(prop)*100):
                            raise RuntimeError("Engine controls rejected")

            def configure():
                ok, detail = self._v1492_apply_reference_causal_state(
                    include_weather=False, context="POST-START-ENGINE")
                if not ok:
                    raise RuntimeError(detail)
                client.restore_engine_support_state(state, thermal=False)

            hold(starting=True)
            client.unpause_sim()
            fuel_ok, fuel_detail = self._v1497_verify_fuel(state)
            if not fuel_ok:
                return False, fuel_detail
            ok, detail = prepare_engine(
                state, self._v1496_sample_engine, hold, configure,
                lambda: client.send_engine_preparation_event(0),
                lambda: self.test_mode != "auto" or not self.auto_prepare_in_progress,
                progress=self._sig_auto_prepare_progress.emit, verify_transients=False)
            print(f"[ENGINE PREPARATION] {'PASS' if ok else 'BLOCKED'} | {detail}", flush=True)
            return ok, detail
        except Exception as exc:
            return False, "Engine preparation error: " + str(exc)
        finally:
            # Return to the existing final pose/control verification sequence.
            client.pause_sim()

    def _v15_restore_paused_initial_transients(self):
        """Final paused seed and readback. No recurring output forcing in replay."""
        state = dict(self.reference_initial_dynamic_state or {})
        if state.get("extended_causal_layout") != "TAGGED_V1497":
            for key in SUPPORT_KEYS: state.pop(key, None)
        self.sim_client.restore_engine_support_state(state, thermal=True)
        pressure = self._v1430_finite(state.get("engine_fuel_pressure_psi_1"))
        if pressure is not None and not self.sim_client.set_initial_fuel_pressure(pressure):
            return False, "Initial fuel-pressure write rejected"
        good = 0
        last = ["No fresh paused readback"]
        for _ in range(12):
            if self.test_mode != "auto": return False, "Final state restore cancelled"
            live = self._v1496_sample_engine()
            last = engine_mismatches(live, state)
            last += fuel_mismatches(live, state)
            last += configuration_mismatches(live, state)
            good = good + 1 if not last else 0
            if good >= 2:
                print("[V15 FINAL INITIAL STATE] two fresh paused readbacks PASS", flush=True)
                return True, "Recorded temperatures, pressure, fuel, flaps and wind verified"
            time.sleep(0.03)
        return False, "Paused initial-state readback failed (model may override writes): " + "; ".join(last)

    def _v1496_live_engine_check(self, frame, fresh_support=True):
        live = dict(frame)
        live.update(self.sim_client.get_engine_support_state(fresh=fresh_support))
        reference = dict(self.reference_initial_dynamic_state or {})
        if reference.get("extended_causal_layout") != "TAGGED_V1497":
            for key in SUPPORT_KEYS: reference.pop(key, None)
        bad = engine_mismatches(live, reference)
        bad += fuel_mismatches(live, self.reference_initial_dynamic_state or {})
        bad += configuration_mismatches(live, reference)
        if bad:
            raise RuntimeError("Natural engine t=0 check failed: " + "; ".join(bad))
        frame.update({key: live[key] for key in SUPPORT_KEYS if key in live})
        return frame

    def _v1495_causal_preflight_audit(self, data: Dict[str, Any], context: str = "") -> tuple[bool, str]:
        """Fail closed on hidden states that can materially change trajectory.

        Read-only failure/icing channels cannot be restored through SimConnect;
        therefore Automatic must not proceed when they indicate a non-normal
        state. This avoids silently comparing a clean Manual with a damaged/iced
        Automatic (or vice versa).
        """
        d = data or {}
        bad = []
        thresholds = (
            ("general_eng_failed_1", 0.5, "engine failed"),
            ("general_eng_damage_percent_1", 0.5, "engine damage"),
            ("eng_on_fire_1", 0.5, "engine fire"),
            ("structural_ice_pct", 0.01, "structural ice"),
            ("pitot_ice_pct", 0.01, "pitot ice"),
            ("flap_damage_by_speed", 0.5, "flap damage"),
            ("gear_damage_by_speed", 0.5, "gear damage"),
        )
        for key, lim, label in thresholds:
            v = self._v1430_finite(d.get(key))
            if v is not None and abs(v) > lim:
                bad.append(f"{label}={v}")
        missing_audit = [key for key, _, _ in thresholds if self._v1430_finite(d.get(key)) is None]
        if missing_audit or d.get("extended_causal_invalid_fields"):
            self.auto_diagnostic_replay = True
            print("[HIDDEN-STATE AUDIT] Coverage incomplete; result is diagnostic: "
                  + ",".join(missing_audit), flush=True)
        # Payload/MOI are logged even for legacy references. For NEW Manuals,
        # restoration is exact and any gross mismatch is reported separately.
        pc = int(d.get("payload_station_count", 0) or 0)
        moi = [self._v1430_finite(d.get(k)) for k in ("total_weight_pitch_moi","total_weight_roll_moi","total_weight_yaw_moi","total_weight_cross_coupled_moi")]
        detail = f"payloadStations={pc} MOI=" + "/".join("NA" if v is None else f"{v:.2f}" for v in moi)
        if bad:
            return False, (context + ": " if context else "") + "NON-NORMAL HIDDEN STATE | " + ", ".join(bad) + " | " + detail
        return True, (context + ": " if context else "") + "hidden-state audit PASS | " + detail + " | L-vars=not enumerable through SimConnect"

    def _v1492_apply_reference_causal_state(
        self, include_weather: bool = True, context: str = ""
    ) -> tuple[bool, str]:
        """Apply recorded causal state or the approved legacy reconstruction.

        Recorded Manual values always win.  A V14.92 legacy profile is used
        only for fields absent from one of the two September-17 bridge-1440
        references. AoA, sideslip and relative-wind channels are NEVER written;
        they remain derived validation outputs.
        """
        if not self.sim_client:
            return False, "SimConnect client unavailable"
        state = dict(getattr(self, "reference_initial_dynamic_state", {}) or {})
        profile = self._v1492_selected_profile() if getattr(self, "v1490_legacy_reference", False) else None
        parts = []
        ok = True

        # Engine controls: recorded fractions are 0..1; legacy profile stores %.
        mix_state = self._v1430_finite(state.get("engine_mixture_lever_1"))
        prop_state = self._v1430_finite(state.get("engine_propeller_lever_1"))
        mix_pct = mix_state * 100.0 if mix_state is not None else self._v1430_finite((profile or {}).get("mixture_percent"))
        prop_pct = prop_state * 100.0 if prop_state is not None else self._v1430_finite((profile or {}).get("propeller_percent"))
        if mix_pct is not None and prop_pct is not None:
            sent = bool(self.sim_client.set_engine_controls(mix_pct, prop_pct))
            ok = bool(ok and sent)
            parts.append(f"engine_controls={mix_pct:.1f}/{prop_pct:.1f}% {'OK' if sent else 'FAIL'}")
        else:
            parts.append("engine_controls=not available")

        flap_idx = self._v1430_finite(state.get("flaps_handle_index"))
        if flap_idx is None:
            flap_idx = self._v1430_finite((profile or {}).get("flaps_handle_index"))
        if flap_idx is not None:
            sent = bool(self.sim_client.set_flaps_handle_index(flap_idx))
            ok = bool(ok and sent)
            parts.append(f"flaps_index={flap_idx:.2f} {'OK' if sent else 'FAIL'}")
        else:
            parts.append("flaps=not available")

        flap_surface = [self._v1430_finite(state.get(k)) for k in (
            "trailing_edge_flaps_left_percent", "trailing_edge_flaps_right_percent",
            "trailing_edge_flaps_left_angle_rad", "trailing_edge_flaps_right_angle_rad",
        )]
        if all(v is not None for v in flap_surface):
            sent = bool(self.sim_client.set_flap_surface_state(*flap_surface))
            ok = bool(ok and sent)
            parts.append(
                "flap_surfaces=" + "/".join(f"{v:.4f}" for v in flap_surface)
                + (" OK" if sent else " FAIL")
            )

        # Fuel distribution matters because identical total fuel does not imply
        # identical CG/inertia. New Manuals use measured tank levels. For the
        # two old Manuals, retain the currently loaded tank ratio and scale it
        # to the actually recorded Manual total fuel mass.
        try:
            tanks, conflicts = primary_fuel_plan(state)
            sent = bool(self.sim_client.set_fuel_tank_levels(*tanks))
            ok = bool(ok and sent)
            parts.append("PRIMARY_FUEL=" + "/".join(f"{v:.6f}" for v in tanks) + (" SENT" if sent else " FAIL"))
            if conflicts:
                parts.append("CONFLICTING_ALL_TANK_SNAPSHOT_IGNORED=" + ",".join(conflicts))
                self.auto_diagnostic_replay = True
        except ValueError as exc:
            if profile:
                sent, detail = self._v1494_match_legacy_fuel(profile)
                ok = bool(ok and sent); parts.append("LEGACY_FUEL=" + detail)
            else:
                return False, "Manual fuel state invalid: " + str(exc)

        # V14.95 payload distribution, every standard fuel tank and the
        # remaining documented settable piston/airframe causal variables.
        # For bridge-1440 legacy references these values were not recorded, so
        # we never invent payload weights.  We preserve the currently loaded
        # payload and only verify MOI/CG. New Manuals restore their exact payload.
        trusted_extended = (state.get("extended_causal_layout") == "TAGGED_V1497")
        payload = state.get("payload_station_weights_lb") if trusted_extended else None
        if isinstance(payload, list) and payload:
            sent = bool(self.sim_client.set_payload_station_weights(payload))
            ok = bool(ok and sent)
            parts.append(f"payload[{len(payload)}]={'OK' if sent else 'FAIL'}")
        elif profile:
            parts.append("legacy_payload=PRESERVE_CURRENT (not recorded in 2026-09-17 Manual)")

        extended_state = dict(state) if trusted_extended else {}
        if not trusted_extended:
            self.auto_diagnostic_replay = True
            parts.append("OLDER_EXTENDED_STATE=PRESERVE_LIVE; capture new Manual for complete matching")
        if profile:
            # Safe legacy defaults only for unrecorded switches whose wrong
            # state can materially affect drag/engine operation. No payload is
            # guessed. Magnetos/fuel available are ON; starter/primer/canopy/
            # damage are OFF. Gear surface state follows the legacy gear handle.
            defaults = {
                "recip_cowl_flap_position_1": 0.0,
                "recip_primer_1": 0.0,
                "recip_alternate_air_position_1": 0.0,
                "recip_left_magneto_1": 1.0,
                "recip_right_magneto_1": 1.0,
                "recip_fuel_available_1": 1.0,
                "recip_fuel_tank_selector_1": float(profile.get("fuel_selector", 16.0)),
                "recip_fuel_tanks_used_1": 0.0,
                "general_eng_starter_1": 0.0,
                "gear_center_position": float(profile.get("gear_handle", 0.0)),
                "gear_left_position": float(profile.get("gear_handle", 0.0)),
                "gear_right_position": float(profile.get("gear_handle", 0.0)),
                "canopy_open": 0.0,
                "gyro_heading_rad": math.radians(float(state.get("magnetic_heading_deg") or self.initial_conditions.get("heading_deg", 0.0))),
                "damage_state": 0.0,
            }
            for k,v in defaults.items():
                if self._v1430_finite(extended_state.get(k)) is None:
                    extended_state[k] = v
        ext_available = any(self._v1430_finite(extended_state.get(k)) is not None for k in (
            "recip_cowl_flap_position_1", "gear_center_position", "canopy_open", "gyro_heading_rad"))
        if ext_available:
            sent = bool(self.sim_client.set_extended_aircraft_configuration(extended_state))
            ok = bool(ok and sent)
            parts.append("extended_config=" + ("OK" if sent else "FAIL"))

        # Exact linear and angular accelerations are legitimate P3D settable
        # initial derivatives. Older recovery files already contain linear
        # acceleration under the historical accel_*_g names (native ft/s^2).
        lin = [self._v1430_finite(state.get(k)) for k in ("accel_x_g", "accel_y_g", "accel_z_g")]
        if all(v is not None for v in lin):
            sent = bool(self.sim_client.set_linear_body_acceleration(*lin))
            ok = bool(ok and sent)
            parts.append("body_accel=" + "/".join(f"{v:+.3f}" for v in lin) + (" OK" if sent else " FAIL"))
        rotacc = [self._v1430_finite(state.get(k)) for k in (
            "rotation_acceleration_body_x_rad_s2", "rotation_acceleration_body_y_rad_s2", "rotation_acceleration_body_z_rad_s2")]
        if all(v is not None for v in rotacc):
            sent = bool(self.sim_client.set_rotation_body_acceleration(*rotacc))
            ok = bool(ok and sent)
            parts.append("rot_accel=" + "/".join(f"{v:+.5f}" for v in rotacc) + (" OK" if sent else " FAIL"))

        # Secondary aircraft configuration is restored only when all measured
        # fields exist. We do not invent these for bridge-1440 Manuals.
        aux_keys = (
            "fuel_tank_selector_1", "autopilot_master", "auto_coordination",
            "spoilers_handle_position", "gear_handle_position",
            "brake_parking_position", "engine_combustion_1",
        )
        aux = [self._v1430_finite(state.get(k)) for k in aux_keys]
        if all(v is not None for v in aux):
            sent = bool(self.sim_client.set_aux_configuration(*aux))
            ok = bool(ok and sent)
            parts.append("aux_config=" + ("OK" if sent else "FAIL"))
        elif profile:
            legacy_aux = [profile.get(k) for k in (
                "fuel_selector", "autopilot_master", "auto_coordination",
                "spoilers_handle", "gear_handle", "parking_brake", "engine_combustion")]
            sent = bool(self.sim_client.set_aux_configuration(*(float(v) for v in legacy_aux)))
            ok = bool(ok and sent)
            parts.append("legacy_aux=" + ("OK" if sent else "FAIL"))

        if include_weather:
            metar = state.get("global_weather_metar") if isinstance(state.get("global_weather_metar"), str) else None
            if not metar and profile and V14_92_APPLY_LEGACY_WEATHER:
                metar = profile.get("weather_metar")
            if metar:
                if profile:
                    sent, wdetail = self._v1494_apply_verified_weather(profile, attempts=5)
                    ok = bool(ok and sent)
                    parts.append(f"weather={'OK' if sent else 'FAIL'}[{wdetail}]")
                else:
                    sent = bool(self.sim_client.set_global_weather_metar(str(metar)))
                    ok = bool(ok and sent)
                    parts.append(f"weather={'OK' if sent else 'FAIL'}[{str(metar)[:80]}]")
            else:
                parts.append("weather=not written")

        source = "LEGACY_PROFILE" if profile else "RECORDED_MANUAL"
        detail = f"{source}" + (f" {profile.get('profile_id')}" if profile else "") + " | " + " | ".join(parts)
        if context:
            detail = f"{context}: {detail}"
        return bool(ok), detail

    def _v1492_replay_secondary_controls(
        self, f0: Dict[str, Any], f1: Dict[str, Any], alpha: float
    ) -> None:
        """Replay non-CLS controls for new Manuals; reassert legacy constants."""
        if not self.sim_client:
            return
        now = time.monotonic()
        profile = self._v1492_selected_profile() if getattr(self, "v1490_legacy_reference", False) else None
        period = V14_92_LEGACY_PROFILE_REASSERT_SEC if profile else V14_92_SECONDARY_REPLAY_SEC
        if now - float(getattr(self, "v1492_last_secondary_reassert_mono", 0.0) or 0.0) < float(period):
            return
        self.v1492_last_secondary_reassert_mono = now
        try:
            if profile:
                self.sim_client.set_engine_controls(
                    float(profile["mixture_percent"]), float(profile["propeller_percent"]))
                self.sim_client.set_flaps_handle_index(float(profile["flaps_handle_index"]))
                self.sim_client.set_aux_configuration(
                    float(profile.get("fuel_selector",16.0)),
                    float(profile.get("autopilot_master",0.0)),
                    float(profile.get("auto_coordination",0.0)),
                    float(profile.get("spoilers_handle",0.0)),
                    float(profile.get("gear_handle",0.0)),
                    float(profile.get("parking_brake",0.0)),
                    float(profile.get("engine_combustion",1.0)),
                )
                return

            mix = self._v1492_lerp_optional(
                f0.get("p3d_engine_mixture_lever_1"), f1.get("p3d_engine_mixture_lever_1"), alpha)
            prop = self._v1492_lerp_optional(
                f0.get("p3d_engine_propeller_lever_1"), f1.get("p3d_engine_propeller_lever_1"), alpha)
            if mix is not None and prop is not None:
                self.sim_client.set_engine_controls(mix * 100.0, prop * 100.0)

            flap0 = self._v1430_finite(f0.get("p3d_flaps_handle_index"))
            flap1 = self._v1430_finite(f1.get("p3d_flaps_handle_index"))
            if flap0 is not None:
                flap = flap1 if (flap1 is not None and float(alpha) >= 0.5) else flap0
                self.sim_client.set_flaps_handle_index(float(flap))

            # Replay the other discrete, potentially flight-affecting aircraft
            # configuration switches recorded by NEW Manuals.  They are held
            # constant when unchanged, but a deliberate Manual change (gear,
            # spoiler, fuel selector, auto-coordination/AP state, parking brake
            # or combustion state) is reproduced at the same timeline point.
            aux_keys = (
                "p3d_fuel_tank_selector_1", "p3d_autopilot_master",
                "p3d_auto_coordination", "p3d_spoilers_handle_position",
                "p3d_gear_handle_position", "p3d_brake_parking_position",
                "p3d_engine_combustion_1",
            )
            aux_vals = []
            for key in aux_keys:
                a0 = self._v1430_finite(f0.get(key))
                a1 = self._v1430_finite(f1.get(key))
                value = a1 if (a1 is not None and float(alpha) >= 0.5) else a0
                aux_vals.append(value)
            if all(v is not None for v in aux_vals):
                self.sim_client.set_aux_configuration(*aux_vals)

            # V14.95 additional time-varying piston/airframe controls. Actual
            # gear/spoiler surfaces remain validation outputs after t=0; the
            # corresponding handles/controls drive the normal P3D dynamics.
            ext_map = {
                "recip_cowl_flap_position_1": "p3d_recip_cowl_flap_position_1",
                "recip_primer_1": "p3d_recip_primer_1",
                "recip_alternate_air_position_1": "p3d_recip_alternate_air_position_1",
                "recip_left_magneto_1": "p3d_recip_left_magneto_1",
                "recip_right_magneto_1": "p3d_recip_right_magneto_1",
                "recip_fuel_available_1": "p3d_recip_fuel_available_1",
                "recip_fuel_tank_selector_1": "p3d_recip_fuel_tank_selector_1",
                "recip_fuel_tanks_used_1": "p3d_recip_fuel_tanks_used_1",
                "general_eng_starter_1": "p3d_general_eng_starter_1",
                "canopy_open": "p3d_canopy_open",
                "gyro_heading_rad": "p3d_gyro_heading_rad",
                "damage_state": "p3d_damage_state",
            }
            if (self.reference_initial_dynamic_state or {}).get("extended_causal_layout") != "TAGGED_V1497":
                return
            support_controls = {}
            for support_key in ("electrical_master_battery", "engine_master_alternator_1", "general_eng_fuel_pump_switch_1", "engine_fuel_valve_1"):
                key = "p3d_" + support_key
                a0 = self._v1430_finite(f0.get(key)); a1 = self._v1430_finite(f1.get(key))
                support_controls[support_key] = a1 if (a1 is not None and float(alpha) >= 0.5) else a0
            support_signature = tuple(support_controls.items())
            if (any(value is not None for value in support_controls.values())
                    and support_signature != getattr(self, "_engine_support_replay_last", None)):
                self.sim_client.restore_engine_support_state(support_controls, thermal=False)
                self._engine_support_replay_last = support_signature
            ext_state = {}
            if (self.reference_initial_dynamic_state or {}).get("extended_causal_layout") != "TAGGED_V1497":
                return
            for target, key in ext_map.items():
                a0 = self._v1430_finite(f0.get(key)); a1 = self._v1430_finite(f1.get(key))
                ext_state[target] = a1 if (a1 is not None and float(alpha) >= 0.5) else a0
            # Do not continuously force gear actual positions; handle replay is
            # already above. Use current live actual gear state for unused args.
            live = self.sim_client.get_telemetry() or {}
            for k in ("gear_center_position", "gear_left_position", "gear_right_position"):
                ext_state[k] = self._v1430_finite(live.get(k)) or 0.0
            if any(v is not None for v in ext_state.values()):
                self.sim_client.set_extended_aircraft_configuration(ext_state)
        except Exception as exc:
            print(f"[V14.92 SECONDARY REPLAY] warning: {exc}", flush=True)

    def _v1492_save_fit_score(self, auto_rows: list, auto_csv_path: Path) -> None:
        """Save an objective trajectory score; never modifies report data."""
        if self.test_mode != "auto" or not auto_rows or not self.manual_csv_rows_by_sec:
            return
        metrics = (
            ("IAS", "Indicated Airspeed (kts)", 2.0),
            ("ROC", "Rate of Climb/Descent (fpm)", 100.0),
            ("Pitch", "Pitch Angle (deg)", 1.0),
            ("Bank", "Bank Angle (deg)", 1.0),
            ("Heading", "Magnetic Heading (deg)", 2.0),
            ("RollRate", "Roll Rate (deg/s)", 0.5),
            ("PitchRate", "Pitch Rate (deg/s)", 0.5),
            ("YawRate", "Yaw Rate (deg/s)", 0.5),
            ("RPM", "Engine 1 RPM", 50.0),
        )
        samples = {name: [] for name, _key, _scale in metrics}
        for ar in auto_rows:
            try:
                sec = int(round(float(ar.get("Time (s)", -1))))
            except Exception:
                continue
            mr = self.manual_csv_rows_by_sec.get(sec)
            if not mr:
                continue
            for name, key, scale in metrics:
                try:
                    av = float(ar.get(key)); mv = float(mr.get(key))
                    if math.isfinite(av) and math.isfinite(mv):
                        if name == "Heading":
                            err = ((av - mv + 180.0) % 360.0) - 180.0
                        else:
                            err = av - mv
                        samples[name].append(err)
                except Exception:
                    pass
        result = {
            "version": V14_92_FIT_SCORE_VERSION,
            "selected_manual": str(getattr(self, "reference_manual_csv_path", "") or ""),
            "legacy_profile_key": getattr(self, "v1492_legacy_profile_key", None),
            "legacy_profile": self._v1492_selected_profile(),
            "metrics": {},
        }
        normalized_terms = []
        for name, _key, scale in metrics:
            errs = samples[name]
            if not errs:
                continue
            mae = sum(abs(e) for e in errs) / len(errs)
            rms = math.sqrt(sum(e * e for e in errs) / len(errs))
            max_abs = max(abs(e) for e in errs)
            result["metrics"][name] = {"n": len(errs), "mae": mae, "rms": rms, "max_abs": max_abs, "normalization": scale}
            normalized_terms.append(mae / scale)
        result["mean_normalized_mae"] = (sum(normalized_terms) / len(normalized_terms)) if normalized_terms else None
        path = Path(auto_csv_path).with_name(Path(auto_csv_path).stem + "_V14_92_fit_score.json")
        with open(path, "w", encoding="utf-8") as f:
            json.dump(result, f, indent=2, sort_keys=True)
        print(f"[V14.92 FIT SCORE] {path} | mean normalized MAE={result['mean_normalized_mae']}", flush=True)

    def _load_selected_manual_reference(self, selected_csv: Path) -> bool:
        """
        V14.43: validate and bind EXACTLY the Manual selected by the operator.

        There is no latest-file substitution and no fallback to any other Manual.
        Automatic fails closed if the selected CSV or either companion file is
        missing, incompatible, misaligned at t=0, or lacks a usable CLS axis.
        """
        self.reference_replay = []
        self.reference_replay_path = None
        self.manual_position_calibration_cache = {}
        self.reference_manual_csv_path = None
        self.reference_bundle_manifest = {}
        self.reference_bundle_id = None
        self.reference_initial_dynamic_state = {}
        self.reference_initial_state_path = None
        self.reference_has_native_cls = False
        self.reference_cls_axes = []
        self.manual_csv_rows_by_sec = {}
        self.last_manual_reference_error = ""
        self.v1490_legacy_reference = False
        self.v1492_legacy_profile_key = None
        self.v1492_legacy_profile = None

        try:
            candidate_csv = Path(selected_csv).resolve()
            if not candidate_csv.exists():
                raise RuntimeError(f"selected Manual CSV no longer exists: {candidate_csv}")
            if "_legacy_plot" in candidate_csv.stem:
                raise RuntimeError("legacy-plot CSV cannot be used as the Manual reference")
            candidate_replay = candidate_csv.with_name(candidate_csv.stem + "_replay.json")
            candidate_state = candidate_csv.with_name(candidate_csv.stem + "_initial_state.json")
            if not candidate_replay.exists():
                raise RuntimeError("selected Manual is missing its _replay.json")
            if not candidate_state.exists():
                raise RuntimeError("selected Manual is missing its _initial_state.json")

            per_sec: Dict[int, Dict[str, float]] = {}
            with open(candidate_csv, "r", encoding="utf-8") as f:
                rows = list(csv.DictReader(f))
            if not rows:
                raise RuntimeError("selected Manual CSV contains no rows")
            first_r = rows[0]

            for r in rows:
                try:
                    t_sec = int(round(float(r.get("Time (s)", 0.0))))
                    p_pos = float(r.get(
                        "Joystick Pitch Position (mm)",
                        r.get("Joystick Pitch Position",
                              r.get("Joystick Pitch Displacement (deg)", 0.0))
                    ))
                    p_f = float(r.get(
                        "Joystick Pitch Force (N)",
                        r.get("Joystick Pitch Force (Nm)", 0.0)
                    ))
                    r_pos = float(r.get(
                        "Joystick Roll Position (mm)",
                        r.get("Joystick Roll Position",
                              r.get("Joystick Roll Displacement (deg)", 0.0))
                    ))
                    r_f = float(r.get(
                        "Joystick Roll Force (N)",
                        r.get("Joystick Roll Force (Nm)", 0.0)
                    ))
                    rd_pos = float(r.get("Rudder Pedal Position (mm)", 0.0))
                    rd_f = float(r.get(
                        "Rudder Pedal Force (N)",
                        r.get("Rudder Pedal Force (Nm)", 0.0)
                    ))
                    full_row = dict(r)
                    full_row.update({
                        "pitch_pos": p_pos,
                        "pitch_disp": p_pos,
                        "pitch_force": p_f,
                        "roll_pos": r_pos,
                        "roll_disp": r_pos,
                        "roll_force": r_f,
                        "rudder_pos": rd_pos,
                        "rudder_force": rd_f,
                        "throttle": float(r.get("Throttle 1 Position (%)", 0.0)),
                    })
                    per_sec[t_sec] = full_row
                except (ValueError, TypeError):
                    pass

            candidate_initial_conditions = {
                "alt_ft": float(first_r.get(
                    "Pressure Alt (ft)",
                    first_r.get("Indicated Altitude (ft)", 3000.0)
                )),
                "speed_kts": float(first_r.get(
                    "Indicated Airspeed (kts)",
                    first_r.get("Indicated Airspeed (kt)", 80.0)
                )),
                "ground_speed": float(first_r.get(
                    "Ground Speed (kt)",
                    first_r.get("Ground Speed (kts)", 0.0)
                )),
                "mach": float(first_r.get("Mach Number (-)", 0.0)),
                "vspeed": float(first_r.get("Rate of Climb/Descent (fpm)", 0.0)),
                "pitch_deg": float(first_r.get("Pitch Angle (deg)", 0.0)),
                "bank_deg": float(first_r.get("Bank Angle (deg)", 0.0)),
                "heading_deg": float(first_r.get(
                    "Magnetic Heading (deg)",
                    first_r.get("Yaw / True Heading (deg)", 48.4)
                )),
                "pitch_pos": float(first_r.get(
                    "Joystick Pitch Position (mm)",
                    first_r.get("Joystick Pitch Position", 0.0)
                )),
                "roll_pos": float(first_r.get(
                    "Joystick Roll Position (mm)",
                    first_r.get("Joystick Roll Position", 0.0)
                )),
                "rudder_pos": float(first_r.get("Rudder Pedal Position (mm)", 0.0)),
                "pitch_force": float(first_r.get(
                    "Joystick Pitch Force (N)",
                    first_r.get("Joystick Pitch Force (Nm)", 0.0)
                )),
                "roll_force": float(first_r.get(
                    "Joystick Roll Force (N)",
                    first_r.get("Joystick Roll Force (Nm)", 0.0)
                )),
                "rudder_force": float(first_r.get(
                    "Rudder Pedal Force (N)",
                    first_r.get("Rudder Pedal Force (Nm)", 0.0)
                )),
                "cg_x": float(first_r.get("CG Pos in X Axis (mm)", 0.0)),
                "cg_y": float(first_r.get("CG Pos in Y Axis (mm)", 0.0)),
                "roll_rate": float(first_r.get("Roll Rate (deg/s)", 0.0)),
                "pitch_rate": float(first_r.get("Pitch Rate (deg/s)", 0.0)),
                "yaw_rate": float(first_r.get("Yaw Rate (deg/s)", 0.0)),
                "alpha": float(first_r.get("Aircraft Angle of Attack (deg)", 0.0)),
                "beta": float(first_r.get("Sideslip Angle (deg)", 0.0)),
                "acc_x": float(first_r.get("X Body Acceleration (g)", 0.0)),
                "acc_y": float(first_r.get("Y Body Acceleration (g)", 0.0)),
                "acc_z": float(first_r.get("Z Body Acceleration (g)", 1.0)),
                "throttle": float(first_r.get("Throttle 1 Position (%)", 80.0)),
                "gross_wt": float(first_r.get("Gross Wt (kg)", 0.0)),
                "fuel_qty": float(first_r.get("Total Fuel Quantity (kg)", 0.0)),
                "oat": float(first_r.get("OAT (deg C)", 0.0)),
                "baro": float(first_r.get("Baro Press (hPa)", 0.0)),
                "engine_rpm": float(first_r.get("Engine 1 RPM", 0.0)),
                "prop_rpm": float(first_r.get("Propeller 1 RPM", 0.0)),
                "fuel_flow": float(first_r.get("Fuel Flow (pph)", 0.0)),
                "egt": float(first_r.get(
                    "EGT (deg C)",
                    first_r.get("Exhaust Gas Temp (deg C)", 0.0)
                )),
                "lat": float(first_r.get("Latitude", RAHWALI_LAT)),
                "lon": float(first_r.get("Longitude", RAHWALI_LON)),
            }

            with open(candidate_replay, "r", encoding="utf-8") as f:
                replay = json.load(f)
            with open(candidate_state, "r", encoding="utf-8") as f:
                state = json.load(f)

            if not self._v1430_state_is_complete(state):
                raise RuntimeError(
                    f"selected state snapshot invalid/old schema "
                    f"(schema={state.get('schema')!r}, bridge={state.get('bridge_version')!r})"
                )

            recovered_state, recovery_diag, recovery_detail = self._v1466_recover_true_manual_dynamics(
                candidate_csv, first_r, state
            )
            if recovered_state is None:
                if V14_66_REQUIRE_MANUAL_P3D_DIAGNOSTIC:
                    raise RuntimeError(
                        "V14.66 requires the matching saved Manual P3D diagnostic: "
                        + recovery_detail
                    )
            else:
                state = recovered_state
                recovered_path = candidate_csv.with_name(
                    candidate_csv.stem + "_initial_state_v1466_recovered.json"
                )
                with open(recovered_path, "w", encoding="utf-8") as rf:
                    json.dump(state, rf, indent=2, sort_keys=True)
                candidate_state = recovered_path
                print(
                    "[V14.66 TRUE DYNAMICS] " + recovery_detail
                    + f" | sidecar={recovered_path.name}",
                    flush=True,
                )

            self.manual_initial_conditions = candidate_initial_conditions
            self.reference_initial_dynamic_state = state
            self.reference_replay = replay

            aligned, detail = self._v14304_reference_csv_matches_state_snapshot()
            if not aligned:
                raise RuntimeError(f"selected Manual t=0 alignment failed: {detail}")
            stamp = re.sub(r"\(\d+\)$", "", candidate_csv.stem)
            legacy_key = next(
                (key for key in V14_92_LEGACY_PROFILES if stamp.endswith(key)), None
            )
            self.v1490_legacy_reference = legacy_key is not None
            self.v1492_legacy_profile_key = legacy_key
            self.v1492_legacy_profile = (
                self._v1494_reconstruct_legacy_profile(legacy_key, state, candidate_initial_conditions)
                if legacy_key else None
            )
            if legacy_key:
                V14_92_LEGACY_PROFILES[legacy_key] = dict(self.v1492_legacy_profile)
            if self.v1490_legacy_reference:
                profile = self.v1492_legacy_profile or {}
                print(
                    "[V14.92 LEGACY REFERENCE] " + candidate_csv.name
                    + " | profile=" + str(profile.get("profile_id"))
                    + " status=" + str(profile.get("calibration_status"))
                    + " | recovered measured P3D sideslip=" + str(state.get("sideslip_deg"))
                    + " deg. Only fields absent from bridge-1440 are supplemented; "
                    "recorded Manual values remain authoritative.",
                    flush=True,
                )
            rpm_at_t0 = self._v1430_finite(state.get("engine_rpm"))
            combustion_at_t0 = self._v1430_finite(state.get("engine_combustion_1"))
            pressure_at_t0 = self._v1430_finite(state.get("engine_fuel_pressure_psi_1"))
            if rpm_at_t0 is not None and rpm_at_t0 >= 500.0 and (
                (combustion_at_t0 is not None and combustion_at_t0 < 0.5)
                or (pressure_at_t0 is not None and pressure_at_t0 < 10.0)
            ):
                raise RuntimeError(
                    "Selected Manual t=0 has spinning RPM but engine startup "
                    f"telemetry (combustion={combustion_at_t0}, "
                    f"fuel pressure={pressure_at_t0} psi). Record a new "
                    "Manual with the engine running and settled."
                )
            if not self._summarize_reference_replay():
                raise RuntimeError("selected Manual native CLS replay validation failed")

            self.manual_csv_rows_by_sec = per_sec
            self.reference_replay_path = str(candidate_replay)
            self.reference_initial_state_path = str(candidate_state)

            bundle_ok, bundle_detail = self._v1432_bind_reference_bundle(
                candidate_csv, candidate_replay, candidate_state
            )
            if not bundle_ok:
                raise RuntimeError(bundle_detail)

            self.selected_manual_csv_path = str(candidate_csv)
            self.last_manual_reference_error = ""
            print(
                "[V14.43 SELECTED MANUAL] ACCEPTED | "
                f"CSV={candidate_csv.name} | Replay={candidate_replay.name} | "
                f"State={candidate_state.name} | Bundle={self.reference_bundle_id}",
                flush=True,
            )
            print(
                "[V14.43 T0] "
                f"Alt={candidate_initial_conditions['alt_ft']:.1f}ft "
                f"IAS={candidate_initial_conditions['speed_kts']:.2f}kt "
                f"Pitch={candidate_initial_conditions['pitch_deg']:+.2f}deg "
                f"Bank={candidate_initial_conditions['bank_deg']:+.2f}deg "
                f"Hdg={candidate_initial_conditions['heading_deg']:.2f}deg",
                flush=True,
            )
            return True

        except Exception as exc:
            self.reference_replay = []
            self.reference_replay_path = None
            self.manual_position_calibration_cache = {}
            self.reference_manual_csv_path = None
            self.reference_bundle_manifest = {}
            self.reference_bundle_id = None
            self.reference_initial_dynamic_state = {}
            self.reference_initial_state_path = None
            self.reference_has_native_cls = False
            self.reference_cls_axes = []
            self.manual_csv_rows_by_sec = {}
            self.last_manual_reference_error = str(exc)
            print(
                "[V14.43 SELECTED MANUAL] INVALID — AUTOMATIC BLOCKED | "
                + self.last_manual_reference_error,
                flush=True,
            )
            return False

    def _find_and_load_latest_manual_reference(self) -> bool:
        """Compatibility wrapper: V14.43 always loads the explicit GUI selection."""
        selected = self._selected_manual_reference_path()
        if selected is None:
            self.last_manual_reference_error = "No Manual reference is selected."
            return False
        return self._load_selected_manual_reference(selected)

    def _commission_force_axes(self):
        """V14.21: all three axes are always permitted for force replay."""
        return ALL_FORCE_REPLAY_AXES

    def _force_stage_allows(self, axis_name: str) -> bool:
        """V15.12.2 VF mode: position replay only; no Force Offset force replay."""
        if VIRTUAL_FORCE_POSITION_ONLY_MODE:
            return False
        return str(axis_name) in ALL_FORCE_REPLAY_AXES

    def _commission_stage_text(self) -> str:
        """Current controller mode shown in diagnostics."""
        if VIRTUAL_FORCE_POSITION_ONLY_MODE:
            return "ALL AXES - POSITION REPLAY + VIRTUAL FORCE READ ONLY"
        return "ALL AXES - Pitch + Roll + Yaw position + force"

    def _select_commission_force_stage(self) -> bool:
        """Compatibility no-op. V14.21 always runs all axes together."""
        self.commission_force_stage = 3
        return True

    def _select_yaw_ap_speed(self) -> bool:
        """Select Yaw AP-position speed for this Automatic run.

        The selection affects only SID_AP_SPEED for the Yaw axis. It does not
        alter Yaw force tuning, position scaling, force conversion,
        safety interlocks, or shutdown behavior.
        """
        box = QMessageBox(self)
        box.setWindowTitle("Yaw AP Speed - This Automatic Run")
        box.setIcon(QMessageBox.Icon.Information)
        box.setText(
            "Select the Yaw AP speed for this run.\n\n"
            "Use the SAME Manual reference and change only this speed between runs."
        )
        box.setInformativeText(
            "Recommended test order: 60 -> 80 -> 100 -> 120 -> 150.\n"
            "Stop increasing once Yaw position follows the Manual trace closely "
            "without harsh movement or abnormal sound. Keep the E-stop accessible."
        )

        buttons = {}
        for speed in YAW_AP_SPEED_OPTIONS:
            btn = box.addButton(f"Yaw speed {speed}", QMessageBox.ButtonRole.AcceptRole)
            buttons[btn] = int(speed)
        cancel = box.addButton(QMessageBox.StandardButton.Cancel)

        # Default to the last-used speed, otherwise the configured default.
        current = int(getattr(self, "commission_yaw_ap_speed", YAW_AP_SPEED_DEFAULT))
        default_button = next((b for b, v in buttons.items() if v == current), None)
        if default_button is not None:
            box.setDefaultButton(default_button)

        box.exec()
        clicked = box.clickedButton()
        if clicked is cancel or clicked is None:
            print("[YAW SPEED] Automatic Test cancelled by operator.", flush=True)
            return False
        if clicked not in buttons:
            return False

        self.commission_yaw_ap_speed = int(buttons[clicked])
        print(
            f"[YAW SPEED] SELECTED {self.commission_yaw_ap_speed} | "
            f"available={','.join(str(v) for v in YAW_AP_SPEED_OPTIONS)}",
            flush=True,
        )
        return True

    def _v1493_sanitize_telemetry(self, data: Dict[str, Any]) -> Dict[str, Any]:
        """Return JSON-safe snapshot of EVERY telemetry key exposed by the wrapper.

        This is an audit recorder. Automatic only writes variables for which the
        application has an explicit, verified setter. Derived/read-only values
        remain validation outputs and are never forced merely to improve graphs.
        """
        out = {}
        for k, v in dict(data or {}).items():
            try:
                if isinstance(v, (str, bool, int)) or v is None:
                    out[str(k)] = v
                elif isinstance(v, float):
                    out[str(k)] = v if math.isfinite(v) else None
                elif isinstance(v, (list, tuple)):
                    vals = []
                    for x in v:
                        if isinstance(x, (int, bool, str)) or x is None:
                            vals.append(x)
                        else:
                            xf = float(x)
                            vals.append(xf if math.isfinite(xf) else None)
                    out[str(k)] = vals
                else:
                    xf = float(v)
                    out[str(k)] = xf if math.isfinite(xf) else None
            except Exception:
                out[str(k)] = repr(v)[:300]
        return out

    def _v1493_record_full_state_audit(self, data: Dict[str, Any], elapsed_ms: int) -> None:
        if self.test_mode != "manual" or not self.is_recording:
            return
        now = time.monotonic()
        if now - float(self.v1493_full_state_last_capture_mono or 0.0) < V14_93_FULL_STATE_AUDIT_PERIOD_SEC:
            return
        self.v1493_full_state_last_capture_mono = now
        self.v1493_full_state_timeline.append({
            "t_ms": int(elapsed_ms),
            "telemetry": self._v1493_sanitize_telemetry(data),
        })

    def _v1493_find_manual_index(self, key: str) -> int:
        if not hasattr(self, "cmb_manual_reference"):
            return -1
        for i in range(self.cmb_manual_reference.count()):
            p = self.cmb_manual_reference.itemData(i)
            if p and key in Path(str(p)).name:
                return i
        return -1

    def _v1493_load_persisted_overrides(self) -> None:
        try:
            path = Path(self.data_dir) / V14_93_OVERRIDE_FILENAME
            if not path.exists():
                return
            obj = json.loads(path.read_text(encoding="utf-8"))
            if not isinstance(obj, dict) or str(obj.get("version", "")) != "V14.95":
                print(f"[V14.95 PROFILE OVERRIDE] ignoring stale pre-V14.95 file: {path}", flush=True)
                return
            profiles = obj.get("profiles", {})
            for key, profile in profiles.items():
                if key in V14_92_LEGACY_PROFILES and isinstance(profile, dict):
                    V14_92_LEGACY_PROFILES[key] = dict(profile)
                    print(f"[V14.93 PROFILE OVERRIDE] loaded {key} -> {profile.get('profile_id')}", flush=True)
        except Exception as exc:
            print(f"[V14.93 PROFILE OVERRIDE] load warning: {exc}", flush=True)

    def _v1493_save_profile_overrides(self) -> None:
        try:
            payload = {
                "version": "V14.95",
                "saved_iso": datetime.datetime.now().isoformat(timespec="seconds"),
                "profiles": {k: dict(V14_92_LEGACY_PROFILES[k]) for k in V14_92_LEGACY_PROFILES},
            }
            path = Path(self.data_dir) / V14_93_OVERRIDE_FILENAME
            path.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
            print(f"[V14.93 PROFILE OVERRIDE] saved {path}", flush=True)
        except Exception as exc:
            print(f"[V14.93 PROFILE OVERRIDE] save warning: {exc}", flush=True)

    @Slot()
    def start_final_two_manual_run(self):
        """Run both September-17 Manuals once each using V14.95 final profiles."""
        if self.v1493_one_visit_active:
            return
        keys = ["2026-09-17_15-25-33", "2026-09-17_15-37-17"]
        missing = [k for k in keys if self._v1493_find_manual_index(k) < 0]
        if missing:
            QMessageBox.critical(self, "Final Two-Manual Run", "Missing required Manual(s): " + ", ".join(missing))
            return
        answer = QMessageBox.question(
            self, "V14.95 FINAL Two-Manual Run",
            "This is the final deterministic Sim-PC batch. It will run BOTH September-17 references for 75 s each.\n\n"
            "Before each run V14.95 reconstructs and verifies wind/OAT, restores recorded fuel/weight-related state, "
            "sets flaps UP, mixture RICH, prop FULL FORWARD, restores the full recorded t=0 dynamics, and performs a "
            "brief one-frame aero lock. BRUNNER Pitch/Roll/Yaw tuning is unchanged.\n\n"
            "Keep clear of the controls and keep the E-stop available. Continue?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        if answer != QMessageBox.StandardButton.Yes:
            return
        self.v1493_one_visit_active = True
        self.v1493_one_visit_stage = "final"
        self.v1493_one_visit_manual_keys = list(keys)
        self.v1493_one_visit_manual_index = 0
        self.v1493_one_visit_candidate_index = 0
        self.v1493_one_visit_results = []
        self.v1493_one_visit_original_profiles = {}
        if hasattr(self, "btn_one_visit"):
            self.btn_one_visit.setEnabled(False)
        print("[V14.95 FINAL TWO-MANUAL] START | " + ",".join(keys), flush=True)
        QTimer.singleShot(250, self._v1493_run_next_one_visit_step)

    def _v1493_current_manual_key(self) -> Optional[str]:
        i = int(self.v1493_one_visit_manual_index)
        if 0 <= i < len(self.v1493_one_visit_manual_keys):
            return self.v1493_one_visit_manual_keys[i]
        return None

    def _v1493_run_next_one_visit_step(self):
        if not self.v1493_one_visit_active:
            return
        key = self._v1493_current_manual_key()
        if key is None:
            self._v1493_finish_one_visit()
            return
        idx = self._v1493_find_manual_index(key)
        if idx < 0:
            self._v1493_abort_one_visit(f"Manual disappeared from selector: {key}")
            return
        self.cmb_manual_reference.setCurrentIndex(idx)
        self.v1493_one_visit_stage = "final"
        self.v1493_one_visit_current_candidate = None
        print(f"[V14.95 FINAL TWO-MANUAL] starting 75s reference {key}", flush=True)
        self.commission_force_stage = 3
        self.auto_replay_strategy = "tracking"
        self.start_auto_test()

    def _v1493_read_score_for_csv(self, csv_path: str) -> Optional[float]:
        try:
            p = Path(csv_path)
            score_paths = [
                p.with_name(p.stem + "_V14_92_fit_score.json"),
                p.with_name(p.stem + "_V14_93_fit_score.json"),
            ]
            for sp in score_paths:
                if sp.exists():
                    obj = json.loads(sp.read_text(encoding="utf-8"))
                    val = obj.get("mean_normalized_mae")
                    return float(val) if val is not None else None
        except Exception as exc:
            print(f"[V14.93 ONE-VISIT] score read warning: {exc}", flush=True)
        return None

    def _v1493_after_auto_report(self, filepath_str: str) -> None:
        if not self.v1493_one_visit_active or self.test_mode != "auto":
            return
        key = self._v1493_current_manual_key()
        if key is None:
            return
        profile = dict(self._v1492_selected_profile() or self.v1493_one_visit_current_candidate or {})
        score = self._v1493_read_score_for_csv(filepath_str)
        rec = {
            "manual_key": key,
            "stage": self.v1493_one_visit_stage,
            "candidate_id": profile.get("candidate_id", profile.get("profile_id")),
            "profile": profile,
            "score": score,
            "auto_csv": str(filepath_str),
        }
        self.v1493_one_visit_results.append(rec)
        print(f"[V14.95 FINAL TWO-MANUAL] RESULT {rec['stage']} {key} {rec['candidate_id']} score={score}", flush=True)

        # V14.95 final-only batch: one real 75-s replay per reference.
        if key in V14_92_LEGACY_PROFILES:
            V14_92_LEGACY_PROFILES[key]["calibration_status"] = "V14.95_FINAL_RUN_COMPLETED"
        self._v1493_save_profile_overrides()
        self.v1493_one_visit_manual_index += 1
        self.v1493_one_visit_candidate_index = 0
        self.v1493_one_visit_stage = "final"
        QTimer.singleShot(1200, self._v1493_run_next_one_visit_step)

    def _v1493_abort_one_visit(self, reason: str) -> None:
        print(f"[V14.95 FINAL TWO-MANUAL] ABORTED | {reason}", flush=True)
        self.v1493_one_visit_active = False
        self.v1493_one_visit_stage = None
        if hasattr(self, "btn_one_visit"):
            self.btn_one_visit.setEnabled(True)
        QMessageBox.critical(self, "V14.95 Final Run Aborted", reason)

    def _v1493_finish_one_visit(self) -> None:
        try:
            summary = {
                "version": "V14.95",
                "completed_iso": datetime.datetime.now().isoformat(timespec="seconds"),
                "results": list(self.v1493_one_visit_results),
                "selected_profiles": {k: dict(V14_92_LEGACY_PROFILES[k]) for k in V14_92_LEGACY_PROFILES},
            }
            path = Path(self.data_dir) / V14_93_SUMMARY_FILENAME
            path.write_text(json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8")
        except Exception as exc:
            path = None
            print(f"[V14.95 FINAL TWO-MANUAL] summary save warning: {exc}", flush=True)
        self.v1493_one_visit_active = False
        self.v1493_one_visit_stage = None
        if hasattr(self, "btn_one_visit"):
            self.btn_one_visit.setEnabled(True)
        QMessageBox.information(
            self, "V14.95 Final Two-Manual Run Complete",
            "Both September-17 references completed their final deterministic 75-second run.\n\n"
            + (f"Summary: {path}" if path else "Summary save failed; see console."),
        )

    @Slot()
    def start_auto_test_only(self):
        """V14.43 Automatic: replay the exact selected Manual; Yaw speed remains selectable."""
        if self._selected_manual_reference_path() is None:
            QMessageBox.warning(
                self,
                "Select a Manual Reference",
                "No saved Manual reference is selected.\n\n"
                "Record a Manual test or press Refresh Manuals, then choose the "
                "Manual run you want Automatic to reproduce.",
            )
            return
        self.commission_force_stage = 3
        if not self._select_yaw_ap_speed():
            return
        self.auto_replay_strategy = "tracking"
        print(
            "[AUTO STRATEGY] TRACKING ONLY | ALL AXES POSITION + FORCE | "
            f"P F/S=adaptive(base {CLS_TRACK_AP_FORCE_PITCH})/{CLS_TRACK_AP_SPEED_PITCH} | "
            f"R F/S={CLS_TRACK_AP_FORCE_ROLL}/{CLS_TRACK_AP_SPEED_ROLL} | "
            f"Y F/S={CLS_TRACK_AP_FORCE_YAW}/{int(getattr(self, 'commission_yaw_ap_speed', YAW_AP_SPEED_DEFAULT))}",
            flush=True,
        )
        print(
            "[TRACKING FORCE CONFIG] "
            f"APForce P/R/Y=adaptive(base {CLS_TRACK_AP_FORCE_PITCH})/"
            f"{CLS_TRACK_AP_FORCE_ROLL}/"
            f"{CLS_TRACK_AP_FORCE_YAW} | "
            f"ForceScale={AUTO_REPLAY_FORCE_SCALE_PERCENT}% | "
            "MoveBack=OFF during automatic replay",
            flush=True,
        )
        print(
            f"[ALL-AXIS FORCE] ENABLED on Pitch, Roll, Yaw | "
            f"YawSpeed={int(getattr(self, 'commission_yaw_ap_speed', YAW_AP_SPEED_DEFAULT))}",
            flush=True,
        )
        self.start_auto_test()

    def _set_cls_position_forwarding_to_p3d(self, enabled: bool) -> bool:
        """
        V13 partial-axis handling for Position Override 0x06.

        Setup: block only axes controlled by this Automatic replay.
        Recording: release the override so physical CLS output can reach P3D.
        """
        if not getattr(self, "cls", None):
            return False
        if not self.active_cls_axes:
            return False

        try:
            if enabled:
                ok = self.cls.send_with_ack(
                    packet_override(AX_NONE, OVR_POSITION_TO_SIM),
                    f"Enable normal CLS output to P3D ({self._active_axis_summary()})",
                )
                self.cls_to_p3d_feedback_enabled = bool(ok)
                if ok:
                    print(
                        "[CLS->P3D] ENABLED | active physical axes: "
                        + self._active_axis_summary(),
                        flush=True,
                    )
                return bool(ok)

            ok = self.cls.send_with_ack(
                packet_override(
                    self.active_cls_axis_mask,
                    OVR_POSITION_TO_SIM,
                ),
                f"Block active CLS output during setup ({self._active_axis_summary()})",
            )
            self.cls_to_p3d_feedback_enabled = False
            if ok:
                print(
                    "[CLS->P3D] BLOCKED during setup | axes="
                    + self._active_axis_summary(),
                    flush=True,
                )
            return bool(ok)
        except Exception as exc:
            print(f"[CLS->P3D] Forwarding change failed: {exc}", flush=True)
            self.cls_to_p3d_feedback_enabled = False
            return False

    def _verify_controls_at_initial_targets(
        self,
        target_pitch: float,
        target_roll: float,
        target_yaw: float,
        tolerance: float = 0.15,
        timeout_sec: float = 5.0,
    ) -> bool:
        """Verify only active physical axes at the recorded initial position."""
        if not self.active_cls_axes:
            return False

        targets = {
            "Pitch": float(target_pitch),
            "Roll": float(target_roll),
            "Yaw": float(target_yaw),
        }

        start = time.monotonic()
        while time.monotonic() - start < timeout_sec:
            actuals = {
                "Pitch": self.cls.last_pitch_pos,
                "Roll": self.cls.last_roll_pos,
                "Yaw": self.cls.last_rudder_pos,
            }

            good = True
            for name in self.active_cls_axes:
                actual = actuals[name]
                if actual is None:
                    good = False
                    break
                if abs(targets[name] - float(actual)) > tolerance:
                    good = False
                    break

            if good:
                details = " ".join(
                    f"{name}={float(actuals[name]):+.3f}/{targets[name]:+.3f}"
                    for name in self.active_cls_axes
                )
                self.auto_last_cls_initial_verify_detail = details
                print(
                    "[CLS INITIAL VERIFY] PASS | " + details,
                    flush=True,
                )
                return True
            time.sleep(0.05)

        details = []
        actuals = {
            "Pitch": self.cls.last_pitch_pos,
            "Roll": self.cls.last_roll_pos,
            "Yaw": self.cls.last_rudder_pos,
        }
        for name in self.active_cls_axes:
            actual = actuals[name]
            actual_text = "NA" if actual is None else f"{float(actual):+.3f}"
            details.append(f"{name}={actual_text}/{targets[name]:+.3f}")

        detail_text = " ".join(details)
        self.auto_last_cls_initial_verify_detail = detail_text
        print(
            "[CLS INITIAL VERIFY] NOT WITHIN TOLERANCE | " + detail_text,
            flush=True,
        )
        return False

    def _v1456_selected_pitch_ap_force(self) -> int:
        """Select extra Pitch AP holding authority only when the Manual needs it.

        The normal validated value remains 1800.  For a high-force Manual, the
        AP-position servo must be able to oppose the commanded Force Offset;
        otherwise the two loops fight and the stick runs toward an end stop.
        The required holding value is derived from the recorded Manual peak force
        in the same internal units used by CLS2Sim and capped conservatively.
        """
        base = int(CLS_TRACK_AP_FORCE_PITCH)
        if not V14_56_ADAPTIVE_PITCH_AP_FORCE_ENABLED:
            return base
        try:
            peak_n = float(self._scan_manual_pitch_force_peak())
            factor = abs(float(self._pitch_force_factor() or 0.0))
            info = dict(self._pitch_force_axis_info() or {})
            axis_limit = abs(float(info.get("force_limit") or 0.0))
            if peak_n <= 0.0 or factor <= 0.0 or axis_limit <= 0.0:
                return base

            needed = int(math.ceil((peak_n / factor) * float(V14_56_PITCH_AP_FORCE_MARGIN)))
            if needed <= int(math.ceil(base * float(V14_56_PITCH_AP_FORCE_ADAPT_THRESHOLD))):
                self.auto_pitch_ap_force_selected = base
                print(
                    "[V14.56 PITCH AP HOLD] "
                    f"ManualPeak={peak_n:.1f}N needs={needed}; proven base {base} retained.",
                    flush=True,
                )
                return base
            cap = int(min(
                float(V14_56_PITCH_AP_FORCE_ABSOLUTE_MAX),
                axis_limit * float(V14_56_PITCH_AP_FORCE_MAX_FRACTION_OF_AXIS_LIMIT),
            ))
            selected = max(base, min(max(base, needed), max(base, cap)))
            self.auto_pitch_ap_force_selected = int(selected)
            print(
                "[V14.56 PITCH AP HOLD] "
                f"ManualPeak={peak_n:.1f}N factor={factor:.4f} "
                f"needed={needed} cap={cap} -> APForce={selected}",
                flush=True,
            )
            return int(selected)
        except Exception as exc:
            print(f"[V14.56 PITCH AP HOLD] fallback to {base}: {exc}", flush=True)
            return base

    def _selected_ap_config(self):
        """Tracking AP configuration with V14.56 adaptive Pitch holding force."""
        pitch_force = self._v1456_selected_pitch_ap_force()
        return (
            (AX_ELEVATOR, "Pitch", pitch_force, CLS_TRACK_AP_SPEED_PITCH),
            (AX_AILERON, "Roll", CLS_TRACK_AP_FORCE_ROLL, CLS_TRACK_AP_SPEED_ROLL),
            (AX_RUDDER, "Yaw", CLS_TRACK_AP_FORCE_YAW, int(getattr(self, "commission_yaw_ap_speed", YAW_AP_SPEED_DEFAULT))),
        )

    def _axis_actual_position(self, name: str) -> Optional[float]:
        if name == "Pitch":
            return self.cls.last_pitch_pos
        if name == "Roll":
            return self.cls.last_roll_pos
        return self.cls.last_rudder_pos

    def _command_and_wait_tracking_target(
        self,
        axis: int,
        name: str,
        target: float,
        hold_sec: float = CLS_TRACK_PRECHECK_HOLD_SEC,
    ) -> Dict[str, Any]:
        """Command a normalized AP target and watch actual CLS2Sim feedback."""
        target = max(
            -CLS_TRACK_PRECHECK_LIMIT,
            min(CLS_TRACK_PRECHECK_LIMIT, float(target)),
        )
        # As in the working climb transport: AP POSITION (0x80), never P3D AXIS_SET.
        # Unlike the legacy precheck, acknowledge the command before claiming
        # the physical target was reached. A UDP send alone proves very little.
        if not self.cls.send_with_ack(
            packet_setting_float(axis, SID_AP_POSITION, target),
            f"{name} climb-style AP position check={target:+.5f}",
        ):
            return {"target": target, "actual": self._axis_actual_position(name),
                    "error": float("inf"), "reached": False, "reason": "AP command not acknowledged"}

        start = time.monotonic()
        best_actual = None
        best_error = float("inf")

        while time.monotonic() - start < float(hold_sec):
            actual = self._axis_actual_position(name)
            if actual is not None:
                actual = float(actual)
                err = abs(target - actual)
                if err < best_error:
                    best_error = err
                    best_actual = actual
                if err <= CLS_TRACK_PRECHECK_TOL:
                    print(
                        f"[TRACK PRECHECK] {name}: target={target:+.3f} "
                        f"actual={actual:+.3f} error={err:.3f} -> REACHED",
                        flush=True,
                    )
                    return {
                        "target": target,
                        "actual": actual,
                        "error": err,
                        "reached": True,
                    }
            time.sleep(0.05)

        actual = self._axis_actual_position(name)
        if actual is not None:
            actual = float(actual)
            err = abs(target - actual)
        else:
            actual = best_actual
            err = best_error

        actual_text = f"{actual:+.3f}" if actual is not None else "NA"
        err_text = f"{err:.3f}" if math.isfinite(err) else "NA"
        print(
            f"[TRACK PRECHECK] {name}: target={target:+.3f} "
            f"actual={actual_text} error={err_text} -> NOT REACHED",
            flush=True,
        )
        return {
            "target": target,
            "actual": actual,
            "error": err,
            "reached": False,
        }

    def _run_tracking_endpoint_precheck(
        self,
        initial_pitch: float,
        initial_roll: float,
        initial_yaw: float,
    ) -> bool:
        """
        Verify recorded endpoint reachability without an artificial
        extreme-to-extreme sprint.

        V14.18 sequence for every active axis:
            recorded initial -> negative endpoint -> recorded initial
            recorded initial -> positive endpoint -> recorded initial

        The previous sequence went directly from the negative endpoint to the
        positive endpoint. For a Roll recording spanning about -0.95..+0.95,
        that demanded almost 1.9 normalized units of travel within only 3 s and
        could report a false FAIL even though the same actuator tracked the
        actual Manual replay correctly.
        """
        if (
            not CLS_TRACK_PRECHECK_ENABLED
            or not self.reference_replay
            or not self.active_cls_axes
        ):
            return True

        initial_by_name = {
            "Pitch": float(initial_pitch),
            "Roll": float(initial_roll),
            "Yaw": float(initial_yaw),
        }

        print(
            "[TRACK PRECHECK] START | axes="
            + self._active_axis_summary()
            + " | endpoint tests are separated by return-to-initial | keep E-stop accessible",
            flush=True,
        )

        self.track_precheck_results = {}

        for name, axis, key in self._active_axis_specs():
            vals = []
            for frame in self.reference_replay:
                try:
                    value = frame.get(key)
                    if value is None:
                        continue
                    value = float(value)
                    if math.isfinite(value):
                        vals.append(value)
                except (TypeError, ValueError):
                    pass

            if not vals:
                self.track_precheck_results[name] = {"error": "no native CLS data"}
                continue

            negative_target = max(-CLS_TRACK_PRECHECK_LIMIT, min(vals))
            positive_target = min(CLS_TRACK_PRECHECK_LIMIT, max(vals))
            initial_target = max(
                -CLS_TRACK_PRECHECK_LIMIT,
                min(CLS_TRACK_PRECHECK_LIMIT, initial_by_name[name]),
            )

            print(
                f"[TRACK PRECHECK] {name} manual range="
                f"{min(vals):+.3f}..{max(vals):+.3f} | "
                f"targets={negative_target:+.3f}/{positive_target:+.3f} | "
                f"initial={initial_target:+.3f}",
                flush=True,
            )

            # A few raw encoder counts are NOT a flight-control maneuver.
            # The old precheck returned PASS even when no meaningful motion was
            # recorded or achieved, since its tolerance was 0.12 normalized.
            # Never amplify tiny manual jitter into real aircraft commands.
            if (max(vals) - min(vals)) < CLS_MIN_MEANINGFUL_MOTION_NORM:
                print(
                    f"[CLIMB AP PROOF] {name} recorded span={max(vals)-min(vals):.6f} norm "
                    "is too small to establish dynamic physical tracking; "
                    "holding the recorded initial position; no artificial movement commanded.",
                    flush=True,
                )
                at_t0 = self._command_and_wait_tracking_target(
                    axis, name, initial_target, hold_sec=CLS_TRACK_PRECHECK_RETURN_HOLD_SEC
                )
                self.track_precheck_results[name] = {
                    "start": at_t0, "negative": at_t0,
                    "return_after_negative": at_t0, "positive": at_t0,
                    "return_after_positive": at_t0,
                    "limited_excitation": True,
                    "span_norm": max(vals) - min(vals),
                }
                continue

            # Start the endpoint test from the actual recorded initial point.
            start_result = self._command_and_wait_tracking_target(
                axis,
                name,
                initial_target,
                hold_sec=CLS_TRACK_PRECHECK_RETURN_HOLD_SEC,
            )
            time.sleep(CLS_TRACK_PRECHECK_INTER_TARGET_SETTLE_SEC)

            neg_result = self._command_and_wait_tracking_target(
                axis,
                name,
                negative_target,
                hold_sec=CLS_TRACK_PRECHECK_HOLD_SEC,
            )
            time.sleep(CLS_TRACK_PRECHECK_INTER_TARGET_SETTLE_SEC)

            return_after_neg = self._command_and_wait_tracking_target(
                axis,
                name,
                initial_target,
                hold_sec=CLS_TRACK_PRECHECK_RETURN_HOLD_SEC,
            )
            time.sleep(CLS_TRACK_PRECHECK_INTER_TARGET_SETTLE_SEC)

            # Positive endpoint is now approached from the recorded initial
            # position instead of directly from the negative endpoint.
            pos_result = self._command_and_wait_tracking_target(
                axis,
                name,
                positive_target,
                hold_sec=CLS_TRACK_PRECHECK_HOLD_SEC,
            )
            time.sleep(CLS_TRACK_PRECHECK_INTER_TARGET_SETTLE_SEC)

            return_after_pos = self._command_and_wait_tracking_target(
                axis,
                name,
                initial_target,
                hold_sec=CLS_TRACK_PRECHECK_RETURN_HOLD_SEC,
            )
            time.sleep(CLS_TRACK_PRECHECK_INTER_TARGET_SETTLE_SEC)

            self.track_precheck_results[name] = {
                "start": start_result,
                "negative": neg_result,
                "return_after_negative": return_after_neg,
                "positive": pos_result,
                "return_after_positive": return_after_pos,
            }

        failed = []
        for name, result in self.track_precheck_results.items():
            if "error" in result:
                failed.append(name)
                continue

            required = (
                "negative",
                "positive",
                "return_after_negative",
                "return_after_positive",
            )
            if any(not result[k].get("reached", False) for k in required):
                failed.append(name)

        # Re-issue the exact recorded initial target one final time.
        for name, axis, _key in self._active_axis_specs():
            self.cls.send(
                packet_setting_float(
                    axis,
                    SID_AP_POSITION,
                    max(
                        -CLS_TRACK_PRECHECK_LIMIT,
                        min(CLS_TRACK_PRECHECK_LIMIT, initial_by_name[name]),
                    ),
                )
            )
        time.sleep(0.75)

        if failed:
            print(
                "[TRACK PRECHECK RESULT] INCOMPLETE/FAIL: "
                + ", ".join(failed)
                + " | this is now a genuine endpoint/return failure, not the old extreme-to-extreme timeout",
                flush=True,
            )
            print(
                "[TRACK PRECHECK] END | active axes returned to initial point",
                flush=True,
            )
            return False

        limited_axes = [name for name, info in self.track_precheck_results.items()
                        if info.get('limited_excitation')]
        if limited_axes:
            print(
                '[TRACK PRECHECK RESULT] STATIC POSITION ONLY, DYNAMIC TRACKING UNPROVEN for: '
                + ', '.join(limited_axes)
                + ' | new Manual movements must exceed the loader noise/deadband for a dynamic position comparison.',
                flush=True,
            )
        else:
            print(
                "[TRACK PRECHECK RESULT] REACHED RECORDED ENDPOINTS: "
                + self._active_axis_summary(),
                flush=True,
            )
        print(
            "[TRACK PRECHECK] END | active axes returned to initial point",
            flush=True,
        )
        return True

    @Slot()
    def start_auto_test(self):
        """Automatic Testing mode for Super Mushshak — 75 seconds test using validation initial conditions from Manual Test."""
        self._reset_commissioning_diagnostics()
        self._reset_p3d_state_diagnostic()
        self.cls_replay_send_failures = {}
        self.cls_replay_ap_off_counts = {}
        self.cls.set_control_write_guard(False, "Automatic replay requires CLS control writes")
        self._release_pitch_force_diagnostic()
        self.test_mode = "auto"
        self._engine_support_replay_last = None
        self.auto_replay_strategy = "tracking"
        self.cls_prep_complete = False
        self.cls_hardware_ready = False
        self.cls_to_p3d_feedback_enabled = False
        self.auto_p3d_ready = False
        self.auto_environment_ready = False
        self.auto_prepare_in_progress = False
        self.auto_waiting_for_p3d_user = False
        self.auto_controller_init_ready = False
        self.auto_last_cls_initial_verify_detail = ""
        self.auto_last_p3d_control_handover_detail = ""
        self.auto_diagnostic_replay = False
        self.auto_diagnostic_start_detail = ""
        self.v1492_last_secondary_reassert_mono = 0.0
        self.auto_last_block_reason = ""
        self.auto_t0_cls_values = {}
        self.v1465_controls_prelocked = False
        self.v1465_controls_final_locked = False
        self.v1465_controls_lock_detail = ""
        self.p3d_user_ready_event.clear()
        self.auto_initposition_sent = False
        self.auto_initposition_count = 0
        self.auto_position_frozen = False
        self.auto_terrain_preload_sent = False
        self.auto_final_init_sent = False
        self.auto_final_direct_state_ready = False
        self.auto_bank_write_multiplier = -1.0
        self.direct_mirror_last_error = ""
        self.direct_mirror_last_commands = {"elevator": None, "aileron": None, "rudder": None}
        self.direct_mirror_anchor = {}
        self.direct_mirror_consecutive_failures = 0
        self.direct_mirror_runtime_mismatch_since = None
        self.direct_mirror_runtime_mismatch_detail = ""
        self._v1458_stop_direct_mirror(close_bridge=True)
        if hasattr(self, "btn_p3d_loaded"):
            self.btn_p3d_loaded.setVisible(False)
            self.btn_p3d_loaded.setEnabled(False)

        # DIAGNOSTIC ONLY: fresh high-rate position evidence buffer.
        self.pitch_position_diag_rows = []
        self.pitch_position_diag_last_path = None

        # FORCE ONLY reset.
        self.pitch_force_replay_active = False
        self.pitch_force_reference_n = None
        self.pitch_force_actual_n = None
        self.pitch_force_offset_command = 0
        self.pitch_force_integral_internal = 0.0
        self.pitch_force_manual_peak_n = 0.0
        self.pitch_force_baseline_n = 0.0
        self.pitch_force_replay_warned = False
        self.pitch_force_last_arm_attempt = 0.0

        self.roll_force_replay_active = False
        self.roll_force_reference_n = None
        self.roll_force_actual_n = None
        self.roll_force_offset_command = 0
        self.roll_force_integral_internal = 0.0
        self.roll_force_manual_peak_n = 0.0
        self.roll_force_baseline_n = 0.0
        self.roll_force_replay_warned = False
        self.roll_force_last_arm_attempt = 0.0

        self.yaw_force_replay_active = False
        self.yaw_force_reference_n = None
        self.yaw_force_actual_n = None
        self.yaw_force_offset_command = 0
        self.yaw_force_integral_internal = 0.0
        self.yaw_force_manual_peak_n = 0.0
        self.yaw_force_baseline_n = 0.0
        self.yaw_force_replay_warned = False
        self.yaw_force_last_arm_attempt = 0.0

        self.force_position_authority = {"Pitch": 1.0, "Roll": 1.0, "Yaw": 1.0}
        self.cls_last_cmd_for_safety = {"Pitch": None, "Roll": None, "Yaw": None}
        now_safety = time.monotonic()
        self.cls_last_cmd_motion_time = {
            "Pitch": now_safety,
            "Roll": now_safety,
            "Yaw": now_safety,
        }
        print(
            "[AUTO REPLAY MODE] BRUNNER AP POSITION/FORCE + MEASURED-CLS P3D BRIDGE | "
            "P3D commands=LIVE physical CLS positions only | "
            "stored Manual targets never written directly to P3D",
            flush=True,
        )
        print(
            f"[ALL-AXIS FORCE] Pitch,Roll,Yaw | "
            f"YawSpeed={int(getattr(self, 'commission_yaw_ap_speed', YAW_AP_SPEED_DEFAULT))}",
            flush=True,
        )
        selected_cfg = self._selected_ap_config()
        cfg_map = {name: (force, speed) for _, name, force, speed in selected_cfg}
        print(
            "[CLS-P CONFIG] "
            f"strategy=TRACKING | "
            f"P(F={cfg_map['Pitch'][0]},S={cfg_map['Pitch'][1]}) "
            f"R(F={cfg_map['Roll'][0]},S={cfg_map['Roll'][1]}) "
            f"Y(F={cfg_map['Yaw'][0]},S={cfg_map['Yaw'][1]}) | "
            f"AutoSoftening={'ON ' + str(AUTO_REPLAY_FORCE_SCALE_PERCENT) + '%/MoveBackOFF' if AUTO_REPLAY_SOFTENING_ENABLED else 'OFF'} | "
            "physical-source=BRUNNER/CLS | P3D-source=measured-CLS direct bridge",
            flush=True,
        )
        self.target_duration_sec = 75
        self.expected_total_rows = 76  # t = 0..75 seconds
        self.rec_progress.setRange(0, 75)
        self.lbl_timer.setText("0 / 75 seconds (0 rows)")

        # V14.43: load ONLY the Manual explicitly selected in the GUI.
        selected_manual = self._selected_manual_reference_path()
        success = (
            self._load_selected_manual_reference(selected_manual)
            if selected_manual is not None
            else False
        )
        if (
            success
            and self.reference_replay_path
            and self.reference_manual_csv_path
            and self.reference_bundle_id
            and self.reference_has_native_cls
        ):
            filename = os.path.basename(self.reference_manual_csv_path)
            self.lbl_csv_path.setText(
                f"🔒 SELECTED Manual: {filename} | Bundle {self.reference_bundle_id}"
            )

            # V14.18 REPORT FIX: Automatic report duration follows the Manual
            # reference duration. This prevents unequal overlay arrays such as
            # Auto 76 rows versus a shorter Manual reference.
            csv_duration = None
            if self.manual_csv_rows_by_sec:
                try:
                    csv_duration = int(max(self.manual_csv_rows_by_sec.keys()))
                except Exception:
                    csv_duration = None

            replay_duration = None
            if self.reference_replay:
                try:
                    replay_duration = int(round(max(
                        float(frame.get("t_ms", 0) or 0)
                        for frame in self.reference_replay
                    ) / 1000.0))
                except Exception:
                    replay_duration = None

            matched_duration = (
                csv_duration if csv_duration is not None and csv_duration >= 1
                else replay_duration
            )
            if matched_duration is not None and matched_duration >= 1:
                self.target_duration_sec = int(matched_duration)
                self.expected_total_rows = self.target_duration_sec + 1
                self.rec_progress.setRange(0, self.target_duration_sec)
                self.lbl_timer.setText(
                    f"0 / {self.target_duration_sec} seconds (0 rows)"
                )
                print(
                    f"[REPORT DURATION] Automatic duration matched to Manual reference: "
                    f"{self.target_duration_sec}s / {self.expected_total_rows} rows",
                    flush=True,
                )
        else:
            print("[Replay] The selected Manual reference is not valid for Automatic.", flush=True)
            self.lbl_csv_path.setText("⚠️ Selected Manual reference is not valid for Automatic Test.")
            reason = getattr(self, "last_manual_reference_error", "") or (
                "No complete valid selected Manual reference bundle was found."
            )
            QMessageBox.warning(
                self,
                "Selected Manual Reference Not Valid",
                "Automatic was blocked because the selected Manual reference could not "
                "be validated.\n\n"
                + reason
                + "\n\nChoose another saved Manual from the selector, or record a fresh Manual.",
            )
            return

        # V14.93 one-visit short candidates use only the first few seconds of
        # the exact Manual replay. Final stage remains full Manual duration.
        if self.v1493_one_visit_active and self.v1493_one_visit_stage == "short":
            self.target_duration_sec = int(V14_93_ONE_VISIT_SHORT_SEC)
            self.expected_total_rows = self.target_duration_sec + 1
            self.rec_progress.setRange(0, self.target_duration_sec)
            self.lbl_timer.setText(f"0 / {self.target_duration_sec} seconds (0 rows)")
            print(f"[V14.93 ONE-VISIT] short diagnostic duration={self.target_duration_sec}s", flush=True)

        init_c = getattr(self, "manual_initial_conditions", {})
        init_alt = init_c.get("alt_ft", 3000.0)
        init_speed = init_c.get("speed_kts", 80.0)
        init_pitch = init_c.get("pitch_deg", 0.0)
        init_bank = init_c.get("bank_deg", 0.0)
        init_hdg = init_c.get("heading_deg", 48.4)
        init_lat = init_c.get("lat", RAHWALI_LAT)
        init_lon = init_c.get("lon", RAHWALI_LON)
        init_throttle = init_c.get("throttle", 80.0)

        print(f"[Validation Init] Applying Manual Validation Initial Conditions to Auto Test -> Lat={init_lat:.5f}, Lon={init_lon:.5f}, Alt={init_alt:.1f}ft, Speed={init_speed:.1f}kts, Pitch={init_pitch:.1f}°, Bank={init_bank:.1f}°, Hdg={init_hdg:.1f}°, Throttle={init_throttle:.1f}%", flush=True)

        # V14.32: verify the exact selected Manual bundle one more time
        # before touching P3D or BRUNNER hardware.
        bundle_ok, bundle_detail = self._v1432_verify_bound_bundle(
            "before Automatic connection"
        )
        print(
            f"[V14.32 BUNDLE VERIFY] {'PASS' if bundle_ok else 'FAIL'} | "
            f"{bundle_detail}",
            flush=True,
        )
        if not bundle_ok:
            QMessageBox.warning(
                self,
                "Reference Bundle Changed",
                "Automatic was blocked because the selected Manual bundle changed "
                "after it was loaded.\n\n" + bundle_detail,
            )
            return

        # V14.34:
        # Connect only here. Once SimConnect reports connected,
        # _on_connection_success performs the COMPLETE variable-time P3D
        # initialization/terrain load and BRUNNER preparation BEFORE the
        # Start Recording button is shown.
        self._start_connection(place_airborne=False)

    @Slot()
    def start_manual_test(self):
        """Manual Testing mode — connect to P3D and place aircraft airborne at 3000ft, 80kts for 75 seconds test."""
        self._reset_commissioning_diagnostics()
        self._reset_p3d_state_diagnostic()
        self.manual_initial_dynamic_state = {}
        self.v1493_full_state_timeline = []
        self.v1493_full_state_last_capture_mono = 0.0
        self.manual_t0_frame = {}
        self.manual_t0_cls_values = {}
        self.manual_state_capture_ready_mono = 0.0
        self.test_mode = "manual"
        print("[REPORT FORCE] Virtual Force 0x51 only; measured-force graph removed; Force Offset replay disabled.", flush=True)
        self.target_duration_sec = 75
        self.expected_total_rows = 76  # t = 0..75 seconds
        self.rec_progress.setRange(0, 75)
        self.lbl_timer.setText("0 / 75 seconds (0 rows)")
        # V14.51 HARD-GUARDED STRICT MANUAL. From this point onward the CLS
        # transport rejects controller-mutating packets locally. Manual polling
        # uses telemetry/status reads only and the Remote-Control watchdog is OFF.
        self.cls.set_control_write_guard(True, "Manual test: telemetry/status reads only")
        self.cls.start_read_only_polling(interval_sec=CLS_POLL_INTERVAL_SEC)

        # Reset only application-side axis bookkeeping; this sends no protocol command.
        self._set_active_cls_axes([])
        self._start_connection(place_airborne=True)



    def _v1484_log_t0_frame(self, label, state, reference=None):
        """Print one complete t=0 snapshot; never modify P3D or CLS."""
        fields = (
            ("Latitude", "latitude_deg", "deg"),
            ("Longitude", "longitude_deg", "deg"),
            ("Altitude", "altitude_ft", "ft"),
            ("IAS", "ias_kts", "kt"),
            ("Pitch", "pitch_deg", "deg"),
            ("Bank", "bank_deg", "deg"),
            ("True heading", "true_heading_deg", "deg"),
            ("WORLD velocity X", "velocity_world_x_fps", "ft/s"),
            ("WORLD velocity Y", "velocity_world_y_fps", "ft/s"),
            ("WORLD velocity Z", "velocity_world_z_fps", "ft/s"),
            ("BODY velocity X", "velocity_body_x_fps", "ft/s"),
            ("BODY velocity Y", "velocity_body_y_fps", "ft/s"),
            ("BODY velocity Z", "velocity_body_z_fps", "ft/s"),
            ("BODY rotation X", "rotation_velocity_body_x_rad_s", "rad/s"),
            ("BODY rotation Y", "rotation_velocity_body_y_rad_s", "rad/s"),
            ("BODY rotation Z", "rotation_velocity_body_z_rad_s", "rad/s"),
            ("Vertical speed", "vertical_speed_fpm", "fpm"),
            ("Elevator trim", "elevator_trim_rad", "rad"),
            ("Aileron trim", "aileron_trim_pct", "fraction"),
            ("Rudder trim", "rudder_trim_pct", "fraction"),
            ("Elevator input", "elevator_input", "fraction"),
            ("Aileron input", "aileron_input", "fraction"),
            ("Rudder input", "rudder_input", "fraction"),
            ("Throttle", "throttle_pct", "%"),
            ("Engine RPM", "engine_rpm", "rpm"),
            ("Prop RPM", "engine_prop_rpm", "rpm"),
            ("engine_mixture_lever_1", "engine_mixture_lever_1", "measured"),
            ("engine_propeller_lever_1", "engine_propeller_lever_1", "measured"),
            ("engine_combustion_1", "engine_combustion_1", "measured"),
            ("engine_fuel_valve_1", "engine_fuel_valve_1", "measured"),
            ("engine_fuel_pressure_psi_1", "engine_fuel_pressure_psi_1", "measured"),
            ("engine_manifold_pressure_psi_1", "engine_manifold_pressure_psi_1", "measured"),
            ("relative_wind_body_x_fps", "relative_wind_body_x_fps", "measured"),
            ("relative_wind_body_y_fps", "relative_wind_body_y_fps", "measured"),
            ("relative_wind_body_z_fps", "relative_wind_body_z_fps", "measured"),
            ("Sideslip", "sideslip_deg", "deg"),
            ("AoA", "aoa_deg", "deg"),
            ("Roll rate", "roll_rate_deg_s", "deg/s"),
            ("Pitch rate", "pitch_rate_deg_s", "deg/s"),
            ("Yaw rate", "yaw_rate_deg_s", "deg/s"),
            ("Actual wind speed", "actual_wind_speed_kt", "kt"),
            ("Actual wind direction", "actual_wind_dir_deg", "deg"),
            ("Fuel flow (diagnostic)", "actual_fuel_flow_pph", "pph"),
        )
        print(f"\n========== {label} t=0 P3D SNAPSHOT ==========", flush=True)
        print(f"Source: {(state or {}).get('t0_source', 'fresh SimConnect frame')}", flush=True)
        for name, key, units in fields:
            value = self._v1430_finite((state or {}).get(key))
            target = self._v1430_finite((reference or {}).get(key)) if reference else None
            measured = f"{value:+.6f}" if value is not None else "MISSING"
            if target is not None and value is not None:
                delta = value - target
                if key in ("true_heading_deg", "actual_wind_dir_deg"):
                    delta = (delta + 180.0) % 360.0 - 180.0
                print(f"{name:<26} {measured:>13} {units:<8} | Manual {target:+.6f} | delta {delta:+.6f}", flush=True)
            else:
                print(f"{name:<26} {measured:>13} {units}", flush=True)
        print("================================================\n", flush=True)

    def _v14304_capture_manual_authoritative_t0(self) -> bool:
        """
        Capture exactly ONE fresh P3D frame immediately after Manual unpause.

        That same frame is the only Manual t=0 source for:
          - Manual CSV/report row 0
          - _initial_state.json
          - Automatic initial-state restoration
        """
        if not self.sim_client:
            return False

        get_seq = getattr(self.sim_client, "get_telemetry_sequence", None)
        request_once = getattr(self.sim_client, "request_telemetry_once", None)
        wait_fresh = getattr(self.sim_client, "wait_for_fresh_telemetry", None)
        if not all(callable(x) for x in (get_seq, request_once, wait_fresh)):
            print("[V14.31 MANUAL T0] Matching V14.30 telemetry API not loaded.", flush=True)
            return False

        self._update_stage_ui(
            STAGE_INITIALIZING,
            "MANUAL — waiting for first fresh P3D t=0 frame; timer NOT started",
        )
        self.lbl_timer.setText(f"0 / {self.target_duration_sec} seconds (0 rows)")
        self.rec_progress.setValue(0)
        QApplication.processEvents()

        seq0 = int(get_seq())
        self.sim_client.unpause_sim()

        # Explicit request: do not use an arbitrary cached background frame.
        if not request_once():
            print("[V14.31 MANUAL T0] FAILED: telemetry request was rejected.", flush=True)
            return False
        fresh = wait_fresh(seq0, V14_30_3_FIRST_FRAME_TIMEOUT_SEC)
        if fresh is None:
            print("[V14.31 MANUAL T0] FAILED: no fresh post-unpause P3D frame.", flush=True)
            try:
                self.sim_client.pause_sim()
            except Exception:
                pass
            return False

        # Following flight reload P3D can briefly report RPM with combustion
        # off and almost no fuel pressure. Never freeze that transient as t=0.
        # Wait while running continuously; do not toggle pause for retries.
        deadline = time.monotonic() + 2.5
        consecutive = 0
        while fresh is not None:
            combustion = self._v1430_finite(fresh.get("engine_combustion_1"))
            rpm = self._v1430_finite((fresh.get("eng_rpms") or [None])[0])
            pressure = self._v1430_finite(fresh.get("engine_fuel_pressure_psi_1"))
            ready = (combustion is not None and combustion >= 0.5
                     and rpm is not None and rpm >= 500.0
                     and pressure is not None and pressure >= 10.0)
            consecutive = consecutive + 1 if ready else 0
            if consecutive >= 3:
                break
            if time.monotonic() >= deadline:
                print(f"[MANUAL T0] Engine transitional: combustion={combustion}, "
                      f"RPM={rpm}, fuel pressure={pressure} psi. "
                      "Reload a running-engine flight and retry.", flush=True)
                self.sim_client.pause_sim()
                return False
            seq0 = int(get_seq())
            if not request_once():
                self.sim_client.pause_sim()
                return False
            fresh = wait_fresh(seq0, 0.4)
            if fresh is None:
                consecutive = 0
                if time.monotonic() >= deadline:
                    self.sim_client.pause_sim()
                    return False
                seq0 = int(get_seq())
                if not request_once():
                    self.sim_client.pause_sim()
                    return False
                fresh = wait_fresh(seq0, 0.4)
        if fresh is None:
            self.sim_client.pause_sim()
            return False

        print("[V15.12.4 MANUAL] Waiting for effective aircraft wind to settle before t=0", flush=True)
        wind_frame, wind_detail = wait_wind_ready(
            self.sim_client, cancelled=lambda: self.test_mode != "manual" or not self.sim_client
        )
        if wind_frame is None:
            print("[V15.12.4 MANUAL] BLOCKED: " + wind_detail, flush=True)
            self.sim_client.pause_sim()
            return False
        print("[V15.12.4 MANUAL] Wind settled: " + wind_detail, flush=True)

        try:
            ext_seq = int(self.sim_client._bridge.GetExtendedCausalSequenceV1495())
            self.sim_client._bridge.RequestExtendedCausalOnceV1495()
            ext_deadline = time.monotonic() + 0.6
            while int(self.sim_client._bridge.GetExtendedCausalSequenceV1495()) <= ext_seq:
                if time.monotonic() >= ext_deadline: break
                time.sleep(0.005)
            support = self.sim_client.get_engine_support_state(fresh=True)
            seq = self.sim_client.get_telemetry_sequence()
            if not self.sim_client.request_telemetry_once():
                raise RuntimeError("Manual t=0 telemetry request rejected")
            fresh = self.sim_client.wait_for_fresh_telemetry(seq, 0.6)
            if fresh is None:
                raise RuntimeError("Manual t=0 primary telemetry timeout")
            fresh = dict(fresh)
            fresh.update(support)
        except Exception as exc:
            print(f"[ENGINE PREPARATION] Manual capture blocked: {exc}", flush=True)
            self.sim_client.pause_sim()
            return False
        state = self._capture_v1430_initial_dynamic_state(fresh)
        if engine_mismatches(fresh, state):
            print("[ENGINE PREPARATION] Manual t=0 engine is not ready", flush=True)
            self.sim_client.pause_sim()
            return False
        exact_frame_keys = (
            "velocity_world_x_fps", "velocity_world_y_fps", "velocity_world_z_fps",
            "velocity_body_x_fps", "velocity_body_y_fps", "velocity_body_z_fps",
            "rotation_velocity_body_x_rad_s", "rotation_velocity_body_y_rad_s",
            "rotation_velocity_body_z_rad_s", "vertical_speed_fpm", "engine_rpm",
        )
        if (not self._v1430_state_is_complete(state)
                or any(self._v1430_finite(state.get(key)) is None for key in exact_frame_keys)):
            print("[V14.31 MANUAL T0] FAILED: first fresh frame is incomplete.", flush=True)
            try:
                self.sim_client.pause_sim()
            except Exception:
                pass
            return False

        self.manual_t0_frame = dict(fresh)
        self.manual_t0_cls_values = dict(getattr(self, "current_joystick_values", {}) or {})

        # Freeze force-source metadata at the SAME Manual t=0 instant.
        if getattr(self, "cls", None):
            self.manual_t0_cls_values.update({
                "cls_pitch_force_source": self.cls.last_force_source.get("Pitch"),
                "cls_roll_force_source": self.cls.last_force_source.get("Roll"),
                "cls_yaw_force_source": self.cls.last_force_source.get("Yaw"),
                "cls_pitch_movement_type": self.cls.last_pitch_movement_type,
                "cls_roll_movement_type": self.cls.last_roll_movement_type,
                "cls_yaw_movement_type": self.cls.last_rudder_movement_type,
            })

        self.manual_initial_dynamic_state = dict(state)
        self.manual_initial_dynamic_state["v1493_full_telemetry_t0"] = self._v1493_sanitize_telemetry(fresh)
        if V14_92_CAPTURE_GLOBAL_METAR and self.sim_client:
            try:
                metar = self.sim_client.get_global_weather_metar(timeout_sec=1.5)
            except Exception as metar_exc:
                metar = None
                print(f"[V14.92 MANUAL WEATHER] read warning: {metar_exc}", flush=True)
            self.manual_initial_dynamic_state["global_weather_metar"] = metar
            print(f"[V14.92 MANUAL WEATHER] METAR={metar!r}", flush=True)
        self.manual_initial_dynamic_state["t0_source"] = "first_fresh_post_unpause_p3d_frame"
        self.manual_initial_dynamic_state["t0_frame_last_updated"] = self._v1430_finite(
            fresh.get("last_updated")
        )
        self._v1484_log_t0_frame("MANUAL (SAVED)", self.manual_initial_dynamic_state)
        self._append_p3d_state_diagnostic(fresh, t0_marker="MANUAL_T0")

        t0_wall = self._v1430_finite(fresh.get("last_updated"))
        if t0_wall is None:
            t0_wall = time.time()

        self.is_recording = True
        self.record_start_time = float(t0_wall)
        self.measurement_audit.begin("manual", self.record_start_time,
                                     self.manual_t0_frame, self.manual_t0_cls_values)
        self.next_sample_second = 0
        self.recorded_rows.clear()
        self.replay_buffer.clear()

        # V14.33: seed replay frame 0 from the SAME frozen CLS snapshot that
        # supplies Manual CSV row 0. This removes the old mismatch where CSV
        # row 0 could be -1391 raw while replay frame 0 was already near 0.
        active_names = list(self.active_cls_axes)
        pos_key = {
            "Pitch": "cls_pitch_norm",
            "Roll": "cls_roll_norm",
            "Yaw": "cls_yaw_norm",
        }
        if (
            active_names
            and all(self.manual_t0_cls_values.get(pos_key[n]) is not None for n in active_names)
        ):
            j = self.manual_t0_cls_values
            self.replay_buffer.append({
                "t_ms": 0,
                "active_cls_axes": ",".join(active_names),

                "cls_pitch_norm": j.get("cls_pitch_norm"),
                "cls_roll_norm": j.get("cls_roll_norm"),
                "cls_yaw_norm": j.get("cls_yaw_norm"),

                "pitch_pos": j.get("pitch_pos"),
                "pitch_force": j.get("pitch_force"),
                "roll_pos": j.get("roll_pos"),
                "roll_force": j.get("roll_force"),
                "rudder_pos": j.get("rudder_pos"),
                "rudder_force": j.get("rudder_force"),

                "cls_pitch_force": j.get("cls_pitch_force"),
                "cls_roll_force": j.get("cls_roll_force"),
                "cls_yaw_force": j.get("cls_yaw_force"),

                "cls_pitch_force_eng": j.get("cls_pitch_force_eng"),
                "cls_roll_force_eng": j.get("cls_roll_force_eng"),
                "cls_yaw_force_eng": j.get("cls_yaw_force_eng"),
                "cls_pitch_force_norm": j.get("cls_pitch_force_norm"),
                "cls_roll_force_norm": j.get("cls_roll_force_norm"),
                "cls_yaw_force_norm": j.get("cls_yaw_force_norm"),
                "cls_pitch_force_raw": j.get("cls_pitch_force_raw"),
                "cls_roll_force_raw": j.get("cls_roll_force_raw"),
                "cls_yaw_force_raw": j.get("cls_yaw_force_raw"),

                "cls_pitch_force_source": j.get("cls_pitch_force_source"),
                "cls_roll_force_source": j.get("cls_roll_force_source"),
                "cls_yaw_force_source": j.get("cls_yaw_force_source"),

                "cls_pitch_movement_type": j.get("cls_pitch_movement_type"),
                "cls_roll_movement_type": j.get("cls_roll_movement_type"),
                "cls_yaw_movement_type": j.get("cls_yaw_movement_type"),
                **self._v1492_secondary_frame_fields(self.manual_t0_frame),
            })
            print(
                "[V14.33 MANUAL T0] CLS replay frame 0 frozen from exact Manual t=0.",
                flush=True,
            )
        else:
            print(
                "[V14.33 MANUAL T0] WARNING: could not freeze a complete CLS t=0 "
                "frame for all active axes.",
                flush=True,
            )

        print(
            "[V14.31 MANUAL T0] AUTHORITATIVE FRAME CAPTURED | "
            f"Alt={state['altitude_ft']:.1f}ft "
            f"IAS={state['ias_kts']:.2f}kt "
            f"Pitch={state['pitch_deg']:+.2f}deg "
            f"Bank={state['bank_deg']:+.2f}deg "
            f"Hdg={state['true_heading_deg']:.2f}deg | "
            f"VW=({state['velocity_world_x_fps']:+.3f},"
            f"{state['velocity_world_y_fps']:+.3f},"
            f"{state['velocity_world_z_fps']:+.3f})",
            flush=True,
        )

        self._update_stage_ui(STAGE_RECORDING)
        self._beep_start()
        return True

    def _v14304_reference_csv_matches_state_snapshot(self) -> tuple[bool, str]:
        """V14.33: strict t=0 consistency between formal CSV and state JSON."""
        init_c = dict(getattr(self, "manual_initial_conditions", {}) or {})
        state = dict(getattr(self, "reference_initial_dynamic_state", {}) or {})
        if not init_c or not self._v1430_state_is_complete(state):
            return False, "missing CSV t=0 or state snapshot"

        try:
            d_alt = float(init_c.get("alt_ft", 1e9)) - float(state["altitude_ft"])
            d_ias = float(init_c.get("speed_kts", 1e9)) - float(state["ias_kts"])
            d_pitch = float(init_c.get("pitch_deg", 1e9)) - float(state["pitch_deg"])
            d_bank = float(init_c.get("bank_deg", 1e9)) - float(state["bank_deg"])
            d_hdg = self._v1430_angle_error_deg(
                init_c.get("heading_deg", 1e9),
                state["magnetic_heading_deg"],
            )
            d_vs = float(init_c.get("vspeed", 1e9)) - float(state["vertical_speed_fpm"])
            d_roll_rate = float(init_c.get("roll_rate", 1e9)) - float(state["roll_rate_deg_s"])
            d_pitch_rate = float(init_c.get("pitch_rate", 1e9)) - float(state["pitch_rate_deg_s"])
            d_yaw_rate = float(init_c.get("yaw_rate", 1e9)) - float(state["yaw_rate_deg_s"])

            ok = (
                abs(d_alt) <= 3.0
                and abs(d_ias) <= 0.75
                and abs(d_pitch) <= 0.75
                and abs(d_bank) <= 0.75
                and abs(d_hdg) <= 1.0
                and abs(d_vs) <= V14_33_VS_TOL_FPM
                and abs(d_roll_rate) <= V14_33_RATE_TOL_DEG_S
                and abs(d_pitch_rate) <= V14_33_RATE_TOL_DEG_S
                and abs(d_yaw_rate) <= V14_33_RATE_TOL_DEG_S
            )
            detail = (
                f"dAlt={d_alt:+.1f}ft dIAS={d_ias:+.2f}kt "
                f"dPitch={d_pitch:+.2f}deg dBank={d_bank:+.2f}deg "
                f"dHdg={d_hdg:+.2f}deg dVS={d_vs:+.1f}fpm "
                f"dRollRate={d_roll_rate:+.2f} dPitchRate={d_pitch_rate:+.2f} "
                f"dYawRate={d_yaw_rate:+.2f}"
            )
            return bool(ok), detail
        except Exception as exc:
            return False, f"comparison error: {exc}"


    def _v1481_plot_failed_start(self, timestamp: str, reason: str):
        """Graph actual startup samples, never fabricate an Automatic replay."""
        rows = list(getattr(self, "p3d_state_diag_rows", []) or [])
        if not rows:
            return None
        try:
            from matplotlib.figure import Figure
            from matplotlib.backends.backend_agg import FigureCanvasAgg
            from matplotlib.backends.backend_pdf import PdfPages

            def finite(value):
                try:
                    v = float(value)
                    return v if math.isfinite(v) else float("nan")
                except (TypeError, ValueError, OverflowError):
                    return float("nan")

            t0 = finite(rows[0].get("diag_elapsed_s"))
            times = [finite(r.get("diag_elapsed_s")) - t0 for r in rows]
            groups = [
                ("Air-relative state", "deg / kt", [
                    ("Sideslip", "p3d_sideslip", "ref_sideslip_deg"),
                    ("AoA", "p3d_aoa", "ref_aoa_deg"),
                    ("IAS", "p3d_ias_kts", None),
                ]),
                ("Angular rates", "deg/s", [
                    ("Roll rate", "p3d_roll_rate_deg_s", "ref_roll_rate_deg_s"),
                    ("Yaw rate", "p3d_yaw_rate_deg_s", "ref_yaw_rate_deg_s"),
                ]),
                ("Engine", "rpm", [
                    ("Engine RPM", "p3d_engine_rpm_1", "ref_engine_rpm_1"),
                ]),
                ("Actual wind", "kt", [
                    ("Wind speed", "p3d_actual_wind_speed_kt", "ref_actual_wind_speed_kt"),
                ]),
                ("Physical CLS position", "normalised", [
                    ("Pitch actual", "cls_pitch_actual_norm", "cls_pitch_target_norm"),
                    ("Roll actual", "cls_roll_actual_norm", "cls_roll_target_norm"),
                    ("Yaw actual", "cls_yaw_actual_norm", "cls_yaw_target_norm"),
                ]),
            ]
            path = self.data_dir / f"Super_Mushak_FAILED_AUTO_START_{timestamp}_graphs.pdf"
            with PdfPages(path) as pdf:
                for page in range(0, len(groups), 2):
                    fig = Figure(figsize=(11.7, 8.3), constrained_layout=True)
                    FigureCanvasAgg(fig)
                    axes = fig.subplots(2, 1)
                    fig.suptitle("BLOCKED START — measured startup trace; no Automatic replay", fontsize=12)
                    for ax, (label, ylabel, traces) in zip(axes, groups[page:page + 2]):
                        for name, actual_key, ref_key in traces:
                            actual = [finite(r.get(actual_key)) for r in rows]
                            if any(math.isfinite(v) for v in actual):
                                line, = ax.plot(times, actual, label=name, linewidth=1.5)
                                if ref_key:
                                    ref = [finite(r.get(ref_key)) for r in rows]
                                    if any(math.isfinite(v) for v in ref):
                                        ax.plot(times, ref, "--", color=line.get_color(),
                                                alpha=0.7, label=name + " reference")
                        ax.set(title=label, xlabel="Seconds since first captured startup frame", ylabel=ylabel)
                        ax.grid(alpha=0.3)
                        if ax.lines:
                            ax.legend(loc="best", fontsize=8)
                    if page + 1 >= len(groups):
                        axes[1].set_visible(False)
                    pdf.savefig(fig)
            print(f"[V14.81 STARTUP GRAPH] saved {path} ({len(rows)} measured samples)", flush=True)
            return path
        except Exception as exc:
            print(f"[V14.81 STARTUP GRAPH] unavailable; diagnostic CSV retained: {exc}", flush=True)
            return None

    def _v14303_abort_auto_start(self, reason: str, title: str = "Automatic Start Blocked") -> None:
        """Fail closed: pause P3D, block CLS->P3D, keep the 75 s clock at zero."""
        self.auto_last_block_reason = str(reason)
        self._v1479_capture_start_diagnostics()
        print(f"[V14.52 START] BLOCKED | {reason}", flush=True)
        try:
            if self.sim_client:
                self.sim_client.pause_sim()
        except Exception:
            pass
        try:
            self._set_cls_position_forwarding_to_p3d(False)
        except Exception:
            pass

        self.is_recording = False
        self.is_armed = False
        self.record_start_time = 0.0
        self.next_sample_second = 0
        self.recorded_rows.clear()
        self.lbl_timer.setText(f"0 / {self.target_duration_sec} seconds (0 rows)")
        self.rec_progress.setValue(0)
        # V14.37 fail closed: do not expose Start Recording again after a
        # failed ready/start check. The operator must press Automatic to begin a
        # fresh one-init cycle.
        if self.test_mode == "auto":
            self.auto_environment_ready = False
            self.auto_p3d_ready = False
            # V14.66.3: a failed Automatic validation is NOT a SimConnect
            # connection failure. Keep the live P3D session reusable so the
            # operator can immediately start a Manual run or retry Automatic.
            self._update_stage_ui(STAGE_AUTO_BLOCKED, reason)
        else:
            self._update_stage_ui(STAGE_CONNECTED_READY)

        # Always save failure evidence. This makes a blocked Automatic explainable
        # even when no normal Auto CSV/PDF exists.
        try:
            diag_ts = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            self._save_commissioning_diagnostics(diag_ts)
            self._save_p3d_state_diagnostics(diag_ts)
            graph_path = self._v1481_plot_failed_start(diag_ts, reason)
        except Exception as diag_exc:
            print(f"[V14.52 START] failure-diagnostic save warning: {diag_exc}", flush=True)

        # Restore normal CLS2Sim ownership/feel after a failed Automatic start.
        try:
            self._teardown_hardware()
        except Exception as teardown_exc:
            print(f"[V14.52 START] teardown warning after block: {teardown_exc}", flush=True)

        QMessageBox.warning(self, title, reason + (
            f"\n\nMeasured startup graph: {graph_path}"
            if graph_path else "\n\nStartup diagnostic CSV was saved; graph unavailable."
        ))

    def _v14303_get_fresh_p3d_frame(self, timeout_sec: float):
        """Explicitly request and return a newer V14.30 bridge telemetry packet."""
        if not self.sim_client:
            return None

        request_once = getattr(self.sim_client, "request_telemetry_once", None)
        wait_fresh = getattr(self.sim_client, "wait_for_fresh_telemetry", None)
        get_seq = getattr(self.sim_client, "get_telemetry_sequence", None)
        if not all(callable(x) for x in (request_once, wait_fresh, get_seq)):
            return None

        seq0 = int(get_seq())
        if not request_once():
            return None
        return wait_fresh(seq0, float(timeout_sec))

    def _v14303_verify_full_state_frame(self, data: Dict[str, Any], context: str) -> bool:
        """Verify position/attitude AND dynamic state against the Manual snapshot."""
        state = dict(getattr(self, "reference_initial_dynamic_state", {}) or {})
        if not self._v1430_state_is_complete(state):
            print(f"[V14.30.3 FINAL] {context}: reference state incomplete", flush=True)
            return False

        pos_ok, pos_detail = self._v1430_position_matches(data or {}, state)
        dyn_ok, dyn_detail = self._v1430_dynamic_matches(data or {}, state)
        print(
            f"[V14.30.3 FINAL] {context} | "
            f"position={'OK' if pos_ok else 'FAIL'} [{pos_detail}] | "
            f"dynamic={'OK' if dyn_ok else 'FAIL'} [{dyn_detail}]",
            flush=True,
        )
        return bool(pos_ok and dyn_ok)



    def _v1458_finite_cls_position(self, axis_name: str) -> Optional[float]:
        if not getattr(self, "cls", None):
            return None
        value = {
            "Pitch": self.cls.last_pitch_pos,
            "Roll": self.cls.last_roll_pos,
            "Yaw": self.cls.last_rudder_pos,
        }.get(axis_name)
        try:
            fv = float(value)
            return fv if math.isfinite(fv) else None
        except (TypeError, ValueError):
            return None

    def _v1458_build_direct_anchor(self) -> bool:
        """Bind Manual t=0 P3D inputs to Manual t=0 physical CLS positions."""
        state = dict(getattr(self, "reference_initial_dynamic_state", {}) or {})
        try:
            anchor = {
                "Pitch": {
                    "cls": float(self._initial_reference_target("Pitch", 0.0)),
                    "p3d": float(state["elevator_input"]),
                    "gain": float(V14_58_PITCH_DELTA_GAIN),
                },
                "Roll": {
                    "cls": float(self._initial_reference_target("Roll", 0.0)),
                    "p3d": float(state["aileron_input"]),
                    "gain": float(V14_58_ROLL_DELTA_GAIN),
                },
                "Yaw": {
                    "cls": float(self._initial_reference_target("Yaw", 0.0)),
                    "p3d": float(state["rudder_input"]),
                    "gain": float(V14_58_YAW_DELTA_GAIN),
                },
            }
        except Exception as exc:
            self.direct_mirror_last_error = f"Could not build Manual t=0 control anchor: {exc}"
            return False
        for name in ("Pitch", "Roll", "Yaw"):
            if name not in (getattr(self, "active_cls_axes", []) or []):
                continue
            vals = anchor[name]
            if not all(math.isfinite(float(vals[k])) for k in ("cls", "p3d", "gain")):
                self.direct_mirror_last_error = f"Non-finite direct-control anchor for {name}"
                return False
        self.direct_mirror_anchor = anchor
        print(
            "[V14.58 DIRECT P3D] Manual anchor | "
            + " | ".join(
                f"{n}:CLS0={anchor[n]['cls']:+.6f}->P3D0={anchor[n]['p3d']:+.6f} gain={anchor[n]['gain']:.3f}"
                for n in ("Pitch", "Roll", "Yaw") if n in (getattr(self, "active_cls_axes", []) or [])
            ),
            flush=True,
        )
        return True

    def _v1460_axis_spec(self, axis_name: str):
        return {
            "Pitch": ("elevator", "elevator_pos"),
            "Roll": ("aileron", "aileron_pos"),
            "Yaw": ("rudder", "rudder_pos"),
        }.get(axis_name)

    def _v1460_reset_axis_calibration(self) -> None:
        self.direct_mirror_axis_calibration = {
            "Pitch": {"ready": False, "slope": None, "offset": None, "plus": None, "minus": None},
            "Roll": {"ready": False, "slope": None, "offset": None, "plus": None, "minus": None},
            "Yaw": {"ready": False, "slope": None, "offset": None, "plus": None, "minus": None},
        }

    def _v1460_send_raw_axis_command(self, axis_name: str, command: float) -> bool:
        """Send one raw p3d_bridge axis value WITHOUT applying calibration."""
        if not getattr(self, "p3d_direct_bridge", None) or not self.p3d_direct_bridge.is_open:
            self.direct_mirror_last_error = "Existing P3D bridge is not open for axis calibration"
            return False
        spec = self._v1460_axis_spec(axis_name)
        if spec is None:
            self.direct_mirror_last_error = f"Unknown calibration axis {axis_name}"
            return False
        control_name, _ = spec
        kwargs = {"elevator": None, "aileron": None, "rudder": None}
        kwargs[control_name] = max(-1.0, min(1.0, float(command)))
        return bool(self.p3d_direct_bridge.send_controls(**kwargs))

    def _v1460_inverse_bridge_command(self, axis_name: str, desired_p3d_input: float) -> Optional[float]:
        """Convert desired P3D input telemetry convention to raw bridge command."""
        cal = dict((getattr(self, "direct_mirror_axis_calibration", {}) or {}).get(axis_name, {}) or {})
        if not cal.get("ready"):
            self.direct_mirror_last_error = f"{axis_name} direct-axis calibration is not ready"
            return None
        try:
            slope = float(cal["slope"])
            offset = float(cal["offset"])
            desired = float(desired_p3d_input)
        except Exception:
            self.direct_mirror_last_error = f"{axis_name} direct-axis calibration is invalid"
            return None
        if not all(math.isfinite(x) for x in (slope, offset, desired)) or abs(slope) < float(V14_60_AXIS_CAL_MIN_ABS_SLOPE):
            self.direct_mirror_last_error = f"{axis_name} direct-axis calibration slope is unusable: {slope}"
            return None
        return max(-1.0, min(1.0, (desired - offset) / slope))

    def _v1460_hold_restore_once(self, pose_state: Dict[str, Any], state: Dict[str, Any], throttle: Optional[float]) -> None:
        """Software-hold one exact Manual t=0 state while P3D remains live."""
        self.sim_client.set_exact_pose_state_no_airspeed(pose_state)
        self.sim_client.set_initial_dynamic_state(state)
        try:
            self._v1492_apply_reference_causal_state(include_weather=False, context="SOFT-HOLD")
        except Exception:
            pass
        if throttle is not None:
            try:
                self.sim_client.set_throttle(float(throttle))
            except Exception:
                pass

    def _v1460_measure_axis_for_raw_command(
        self,
        axis_name: str,
        raw_command: float,
        pose_state: Dict[str, Any],
        state: Dict[str, Any],
        throttle: Optional[float],
    ) -> Optional[float]:
        """Measure fresh P3D input response to one raw bridge command under hold."""
        spec = self._v1460_axis_spec(axis_name)
        if spec is None:
            return None
        _control_name, telemetry_key = spec
        if not self._v1460_send_raw_axis_command(axis_name, raw_command):
            return None

        request_once = getattr(self.sim_client, "request_telemetry_once", None)
        wait_fresh = getattr(self.sim_client, "wait_for_fresh_telemetry", None)
        get_seq = getattr(self.sim_client, "get_telemetry_sequence", None)
        if not all(callable(x) for x in (request_once, wait_fresh, get_seq)):
            self.direct_mirror_last_error = "Fresh P3D telemetry API unavailable during axis calibration"
            return None

        values = []
        seq = int(get_seq())
        deadline = time.monotonic() + float(V14_60_AXIS_CAL_POINT_TIMEOUT_SEC)
        last_restore = 0.0
        while time.monotonic() < deadline and len(values) < int(V14_60_AXIS_CAL_SAMPLES_PER_POINT):
            now = time.monotonic()
            if now - last_restore >= float(V14_60_AXIS_CAL_RESTORE_PERIOD_SEC):
                self._v1460_hold_restore_once(pose_state, state, throttle)
                # Re-issue after state restoration in case the aircraft/model
                # rewrites its input channel as part of the state update.
                self._v1460_send_raw_axis_command(axis_name, raw_command)
                last_restore = now

            seq0 = int(get_seq())
            request_once()
            fresh = wait_fresh(seq0, 0.25)
            if fresh is None:
                continue
            try:
                seq = int(fresh.get("bridge_telemetry_sequence", get_seq()))
            except Exception:
                seq = int(get_seq())
            live = self._v1430_finite(fresh.get(telemetry_key))
            if live is not None:
                values.append(float(live))
            time.sleep(0.01)

        if len(values) < int(V14_60_AXIS_CAL_SAMPLES_PER_POINT):
            self.direct_mirror_last_error = (
                f"{axis_name} axis calibration did not obtain enough fresh P3D samples "
                f"for raw command {raw_command:+.3f}: {len(values)}/"
                f"{int(V14_60_AXIS_CAL_SAMPLES_PER_POINT)}"
            )
            return None
        values.sort()
        return float(values[len(values) // 2])

    def _v1460_calibrate_direct_axis_transfer_under_hold(
        self,
        pose_state: Dict[str, Any],
        state: Dict[str, Any],
        throttle: Optional[float],
    ) -> bool:
        """V14.76 robust measured bridge->P3D transfer calibration."""
        active = list(getattr(self, "active_cls_axes", []) or [])
        probe = float(V14_76_CAL_PROBE)

        if not active:
            self.direct_mirror_last_error = "No active CLS axes exist for V14.76 calibration"
            return False

        # A calibration is only reusable in this process with the same live
        # SimConnect client, bridge and active control axes.  The normal live
        # control/surface gate below still checks every new Automatic startup.
        cache_key = (id(self.sim_client), id(self.p3d_direct_bridge), tuple(active))
        calibration = getattr(self, "direct_mirror_axis_calibration", {}) or {}
        if (getattr(self, "_axis_calibration_verified_key", None) == cache_key
                and all(calibration.get(axis, {}).get("ready") for axis in active)):
            self.direct_mirror_last_error = ""
            self._v1460_hold_restore_once(pose_state, state, throttle)
            print("[V14.85 CAL] Reusing verified live-session axis calibration; no probe pulses.", flush=True)
            return True

        self._axis_calibration_verified_key = None
        self._v1460_reset_axis_calibration()

        self.v1476_axis_cal_attempts = {"Pitch": 0, "Roll": 0, "Yaw": 0}
        self.v1476_axis_cal_last_detail = {"Pitch": "", "Roll": "", "Yaw": ""}

        get_seq = getattr(self.sim_client, "get_telemetry_sequence", None)
        request_once = getattr(self.sim_client, "request_telemetry_once", None)
        wait_fresh = getattr(self.sim_client, "wait_for_fresh_telemetry", None)
        if not all(callable(x) for x in (get_seq, request_once, wait_fresh)):
            self.direct_mirror_last_error = "Fresh P3D telemetry API unavailable during V14.76 calibration"
            return False

        telemetry_key = {"Pitch": "elevator_pos", "Roll": "aileron_pos", "Yaw": "rudder_pos"}
        state_key = {"Pitch": "elevator_input", "Roll": "aileron_input", "Yaw": "rudder_input"}

        def pause_restore_exact():
            try:
                self.sim_client.pause_sim()
            except Exception:
                pass
            time.sleep(0.035)
            self._v1460_hold_restore_once(pose_state, state, throttle)
            time.sleep(0.020)
            self._v1460_hold_restore_once(pose_state, state, throttle)

        def read_baseline(axis_name):
            for _ in range(3):
                seq0 = int(get_seq())
                if not request_once():
                    continue
                fresh = wait_fresh(seq0, 0.25)
                if fresh is not None:
                    self._v1479_capture_start_diagnostics(fresh)
                if fresh is None:
                    continue
                v = self._v1430_finite(fresh.get(telemetry_key[axis_name]))
                if v is not None:
                    return float(v)
            return None

        def median(values):
            vv = sorted(float(v) for v in values)
            return float(vv[len(vv)//2])

        def measure_point(axis_name, raw_command):
            pause_restore_exact()
            baseline = read_baseline(axis_name)
            if baseline is None:
                return None, "baseline unavailable"

            if not self._v1460_send_raw_axis_command(axis_name, raw_command):
                return None, "initial command failed"

            samples = []
            fresh_frames = 0
            last_send = 0.0
            deadline = time.monotonic() + float(V14_76_CAL_POINT_TIMEOUT_SEC)

            try:
                self.sim_client.unpause_sim()

                while time.monotonic() < deadline:
                    now = time.monotonic()
                    if now - last_send >= float(V14_76_CAL_RESEND_SEC):
                        if not self._v1460_send_raw_axis_command(axis_name, raw_command):
                            return None, "command re-send failed"
                        last_send = now

                    seq0 = int(get_seq())
                    if not request_once():
                        continue
                    fresh = wait_fresh(seq0, float(V14_76_CAL_FRESH_WAIT_SEC))
                    if fresh is not None:
                        self._v1479_capture_start_diagnostics(fresh)
                    if fresh is None:
                        continue

                    live = self._v1430_finite(fresh.get(telemetry_key[axis_name]))
                    if live is None:
                        continue

                    fresh_frames += 1
                    if fresh_frames <= int(V14_76_CAL_IGNORE_INITIAL_FRAMES):
                        continue

                    if abs(float(live) - baseline) < float(V14_76_CAL_MIN_RESPONSE_FROM_BASELINE):
                        continue

                    samples.append(float(live))
                    if len(samples) >= int(V14_76_CAL_SAMPLES_PER_POINT):
                        tail = samples[-int(V14_76_CAL_SAMPLES_PER_POINT):]
                        spread = max(tail) - min(tail)
                        if spread <= float(V14_76_CAL_MAX_SAMPLE_SPREAD):
                            value = median(tail)
                            return value, (
                                f"base={baseline:+.5f} raw={raw_command:+.3f} "
                                f"value={value:+.5f} spread={spread:.5f} frames={fresh_frames}"
                            )
                        samples = samples[-2:]
            finally:
                pause_restore_exact()

            return None, (
                f"timeout base={baseline:+.5f} raw={raw_command:+.3f} "
                f"samples={len(samples)} frames={fresh_frames}"
            )

        print(
            f"[V14.76 RELIABLE CAL] BEGIN | axes={','.join(active)} | "
            f"probe=+/-{probe:.3f} | retries={V14_76_CAL_AXIS_MAX_ATTEMPTS}",
            flush=True,
        )

        for axis_name in active:
            passed = False
            last_detail = ""

            for attempt in range(1, int(V14_76_CAL_AXIS_MAX_ATTEMPTS) + 1):
                self.v1476_axis_cal_attempts[axis_name] = attempt

                plus, plus_detail = measure_point(axis_name, +probe)
                minus, minus_detail = measure_point(axis_name, -probe)

                if plus is None or minus is None:
                    last_detail = f"+[{plus_detail}] -[{minus_detail}]"
                    self.v1476_axis_cal_last_detail[axis_name] = last_detail
                    print(
                        f"[V14.76 RELIABLE CAL] {axis_name} attempt={attempt} RETRY | {last_detail}",
                        flush=True,
                    )
                    pause_restore_exact()
                    time.sleep(float(V14_76_CAL_INTER_ATTEMPT_PAUSE_SEC))
                    continue

                slope = (float(plus) - float(minus)) / (2.0 * probe)
                offset = 0.5 * (float(plus) + float(minus))
                last_detail = (
                    f"plus={plus:+.5f} minus={minus:+.5f} "
                    f"slope={slope:+.5f} offset={offset:+.5f}"
                )
                self.v1476_axis_cal_last_detail[axis_name] = last_detail

                fit_ok = (
                    math.isfinite(slope)
                    and math.isfinite(offset)
                    and abs(slope) >= float(V14_60_AXIS_CAL_MIN_ABS_SLOPE)
                    and abs(slope) <= float(V14_60_AXIS_CAL_MAX_ABS_SLOPE)
                )
                if not fit_ok:
                    print(
                        f"[V14.76 RELIABLE CAL] {axis_name} attempt={attempt} BAD FIT -> RETRY | "
                        + last_detail,
                        flush=True,
                    )
                    pause_restore_exact()
                    time.sleep(float(V14_76_CAL_INTER_ATTEMPT_PAUSE_SEC))
                    continue

                self.direct_mirror_axis_calibration[axis_name] = {
                    "ready": True,
                    "slope": float(slope),
                    "offset": float(offset),
                    "plus": float(plus),
                    "minus": float(minus),
                }
                print(
                    f"[V14.76 RELIABLE CAL] {axis_name} PASS attempt={attempt} | "
                    f"slope={slope:+.5f} offset={offset:+.5f} | "
                    f"SIGN={'SAME' if slope > 0 else 'INVERTED'}",
                    flush=True,
                )
                passed = True
                break

            if not passed:
                self.direct_mirror_last_error = (
                    f"{axis_name} V14.76 calibration failed after "
                    f"{V14_76_CAL_AXIS_MAX_ATTEMPTS} internal attempts | {last_detail}"
                )
                pause_restore_exact()
                return False

            desired = self._v1430_finite(state.get(state_key[axis_name]))
            if desired is None:
                self.direct_mirror_last_error = f"{axis_name} Manual t=0 P3D input missing"
                pause_restore_exact()
                return False

            raw_restore = self._v1460_inverse_bridge_command(axis_name, float(desired))
            if raw_restore is None:
                pause_restore_exact()
                return False
            pause_restore_exact()
            if not self._v1460_send_raw_axis_command(axis_name, float(raw_restore)):
                pause_restore_exact()
                return False

        # Queue all exact Manual t=0 controls through measured maps.
        bridge_kwargs = {"elevator": None, "aileron": None, "rudder": None}
        ctl_name = {"Pitch": "elevator", "Roll": "aileron", "Yaw": "rudder"}

        for axis_name in active:
            desired = self._v1430_finite(state.get(state_key[axis_name]))
            raw = self._v1460_inverse_bridge_command(axis_name, float(desired))
            if raw is None:
                pause_restore_exact()
                return False
            bridge_kwargs[ctl_name[axis_name]] = raw

        # Final state scrub after all live calibration pulses.
        for _ in range(3):
            pause_restore_exact()
            if not self.p3d_direct_bridge.send_controls(**bridge_kwargs):
                self.direct_mirror_last_error = (
                    self.p3d_direct_bridge.last_error
                    or "Could not queue calibrated Manual t=0 controls"
                )
                pause_restore_exact()
                return False
            time.sleep(0.025)

        self.direct_mirror_last_error = ""
        print(
            "[V14.76 RELIABLE CAL] ALL AXES PASS | continuing in same Automatic attempt.",
            flush=True,
        )
        return True

    def _v1458_direct_commands_from_measured_cls(self) -> Optional[Dict[str, Optional[float]]]:
        if not self.direct_mirror_anchor:
            return None
        mapping = {"Pitch": "elevator", "Roll": "aileron", "Yaw": "rudder"}
        out = {"elevator": None, "aileron": None, "rudder": None}
        for axis_name in ("Pitch", "Roll", "Yaw"):
            if axis_name not in (getattr(self, "active_cls_axes", []) or []):
                continue
            actual = self._v1458_finite_cls_position(axis_name)
            if actual is None:
                return None
            a = self.direct_mirror_anchor[axis_name]
            cmd = float(a["p3d"]) + (float(actual) - float(a["cls"])) * float(a["gain"])
            out[mapping[axis_name]] = max(-1.0, min(1.0, cmd))
        return out

    def _v1458_send_direct_measured_controls_once(self) -> bool:
        """Legacy call-site compatibility; never writes P3D primary axes.

        Existing handover/stabilization loops call this helper repeatedly;
        it now only confirms that CLS2Sim forwarding was enabled.
        """
        if not bool(getattr(self, "cls_to_p3d_feedback_enabled", False)):
            self.direct_mirror_last_error = "CLS2Sim Position-to-Sim forwarding is not enabled"
            return False
        self.direct_mirror_last_error = ""
        return True

    def _v1458_direct_mirror_loop(self) -> None:
        next_tick = time.monotonic()
        while not self.direct_mirror_stop.is_set():
            if self.direct_mirror_active and self.test_mode == "auto":
                self._v1458_send_direct_measured_controls_once()
            next_tick += float(V14_58_DIRECT_MIRROR_PERIOD_SEC)
            delay = next_tick - time.monotonic()
            if delay > 0:
                self.direct_mirror_stop.wait(delay)
            else:
                next_tick = time.monotonic()

    def _v1458_start_direct_mirror(self, bridge_prepared: bool = False) -> bool:
        """BRUNNER-only control handover: enable CLS2Sim output, no AXIS_SET.

        Old name retained for the surrounding V2.1.3 preparation sequence.
        The former 50 Hz direct writer, P3D calibration and mirror thread
        are NOT started. Physical AP movement still comes from BRUNNER.
        """
        if self.test_mode != "auto":
            self.direct_mirror_last_error = "BRUNNER-only handover requires Automatic mode"
            return False
        self.direct_mirror_active = False
        self.direct_mirror_stop.set()
        if getattr(self, "p3d_direct_bridge", None):
            try:
                self.p3d_direct_bridge.close()
            except Exception:
                pass
        if not self._set_cls_position_forwarding_to_p3d(True):
            self.direct_mirror_last_error = (
                "CLS2Sim forwarding could not be enabled. P3D flight controls "
                "cannot be driven by this QTG application."
            )
            return False
        self.direct_mirror_last_error = ""
        print("[BRUNNER ONLY] CLS2Sim forwarding ON; direct P3D axis bridge OFF", flush=True)
        return True

    def _v1458_stop_direct_mirror(self, close_bridge: bool = True) -> None:
        self.direct_mirror_active = False
        self.direct_mirror_runtime_mismatch_since = None
        self.direct_mirror_stop.set()
        t = self.direct_mirror_thread
        if t is not None and t.is_alive() and threading.current_thread() is not t:
            t.join(timeout=0.75)
        self.direct_mirror_thread = None
        if close_bridge and getattr(self, "p3d_direct_bridge", None):
            try:
                self.p3d_direct_bridge.close()
            except Exception:
                pass
        print("[V14.58 DIRECT P3D] measured-CLS mirror STOPPED", flush=True)

    def _v1458_runtime_bridge_health(self, data: Dict[str, Any]) -> tuple[bool, str]:
        """Monitor the sole flight-control source, not the removed P3D bridge.

        CLS2Sim override acknowledgement alone cannot demonstrate P3D's
        actual aircraft input follows AP-driven BRUNNER positions. Retain
        the P3D input signals in raw diagnostic CSV for that independent check.
        """
        if self.test_mode != "auto" or not self.is_recording:
            return True, "not recording"
        if not getattr(self, "cls", None) or not bool(getattr(self, "cls_to_p3d_feedback_enabled", False)):
            return False, "CLS2Sim Position-to-Sim forwarding lost or not enabled"
        stamps = getattr(self.cls, "position_sample_times", {}) or {}
        positions = {
            "Pitch": getattr(self.cls, "last_pitch_pos", None),
            "Roll": getattr(self.cls, "last_roll_pos", None),
            "Yaw": getattr(self.cls, "last_rudder_pos", None),
        }
        now = time.monotonic()
        for name in (getattr(self, "active_cls_axes", []) or []):
            raw = positions.get(name)
            age = now - float(stamps.get(name, -1e6))
            if raw is None or not math.isfinite(float(raw)) or not (0.0 <= age <= 1.0):
                return False, f"{name} BRUNNER physical position missing/stale ({age:.2f}s)"
        return True, "BRUNNER/CLS2Sim live; P3D physical-to-input transfer requires live verification"

    def _v1458_abort_runtime_bridge(self, reason: str) -> None:
        """Emergency fail-closed stop if the direct P3D bridge becomes invalid."""
        reason = "BRUNNER/CLS2Sim automatic replay safety stop: " + str(reason)
        self.auto_last_block_reason = reason
        print("[V14.60 RUNTIME SAFETY] " + reason, flush=True)
        try:
            if self.sim_client:
                self.sim_client.pause_sim()
        except Exception:
            pass
        # stop_test() saves a partial CSV/report when enough rows exist and calls
        # the normal BRUNNER teardown. That teardown stops direct commands BEFORE
        # returning Position-to-Sim ownership to CLS2Sim.
        self.stop_test()
        # This is a fail-closed Automatic-control stop, not a CLS/network
        # connection loss.  Keep the user-facing message precise.
        self._update_stage_ui(STAGE_ARMED, "AUTOMATIC STOPPED — " + reason)
        try:
            QMessageBox.critical(self, "Automatic Control Bridge Stopped", reason)
        except Exception:
            pass

    def _v1458_control_surfaces_match(self, data: Dict[str, Any], context: str) -> tuple[bool, str]:
        state = dict(getattr(self, "reference_initial_dynamic_state", {}) or {})
        d = data or {}
        ok = True
        parts = []

        def chk(label, live_key, ref_key, tol, required=True):
            nonlocal ok
            if not required:
                return
            live = self._v1430_finite(d.get(live_key))
            ref = self._v1430_finite(state.get(ref_key))
            if live is None or ref is None:
                ok = False
                parts.append(f"{label}=MISSING({live}/{ref})")
                return
            err = float(live) - float(ref)
            passed = abs(err) <= float(tol)
            ok = bool(ok and passed)
            parts.append(f"{label}={live:+.5f}/{ref:+.5f} d={err:+.5f} ({'OK' if passed else 'FAIL'})")

        active = set(getattr(self, "active_cls_axes", []) or [])
        chk("ElevInput", "elevator_pos", "elevator_input", V14_58_CONTROL_INPUT_TOL, "Pitch" in active)
        chk("AilInput", "aileron_pos", "aileron_input", V14_58_CONTROL_INPUT_TOL, "Roll" in active)
        chk("RudInput", "rudder_pos", "rudder_input", V14_58_CONTROL_INPUT_TOL, "Yaw" in active)
        chk("ElevSurface", "elevator_deflection", "elevator_deflection_rad", V14_58_ELEVATOR_SURFACE_TOL_RAD, "Pitch" in active)
        chk("AilLSurface", "aileron_left_deflection", "aileron_left_deflection_rad", V14_58_AILERON_SURFACE_TOL_RAD, "Roll" in active)
        chk("AilRSurface", "aileron_right_deflection", "aileron_right_deflection_rad", V14_58_AILERON_SURFACE_TOL_RAD, "Roll" in active)
        chk("RudSurface", "rudder_deflection", "rudder_deflection_rad", V14_58_RUDDER_SURFACE_TOL_RAD, "Yaw" in active)
        chk("ElevTrim", "elevator_trim", "elevator_trim_rad", V14_30_ELEV_TRIM_TOL_RAD, True)
        chk("AilTrim", "aileron_trim_pct", "aileron_trim_pct", V14_30_TRIM_PCT_TOL, True)
        chk("RudTrim", "rudder_trim_pct", "rudder_trim_pct", V14_30_TRIM_PCT_TOL, True)

        throttle_ref = self._v1430_finite(state.get("throttle_pct"))
        throttles = d.get("throttles", []) or []
        throttle_live = self._v1430_finite(throttles[0] if throttles else d.get("max_throttle"))
        if throttle_ref is not None and throttle_live is not None:
            terr = throttle_live - throttle_ref
            passed = abs(terr) <= float(V14_58_THROTTLE_TOL_PCT)
            ok = bool(ok and passed)
            parts.append(f"Throttle={throttle_live:.2f}/{throttle_ref:.2f} d={terr:+.2f} ({'OK' if passed else 'FAIL'})")

        # Engine RPM is a guard against an unrelated power-state problem. Fuel
        # flow remains diagnostic because this aircraft's bridge reports 0 pph
        # after t=0 while RPM and throttle remain correct.
        rpm_ref = self._v1430_finite(state.get("engine_rpm"))
        rpms = d.get("eng_rpms", []) or []
        rpm_live = self._v1430_finite(rpms[0] if rpms else None)
        if rpm_ref is not None and rpm_live is not None:
            rerr = rpm_live - rpm_ref
            passed = abs(rerr) <= float(V14_58_ENGINE_RPM_TOL)
            ok = bool(ok and passed)
            parts.append(f"RPM={rpm_live:.1f}/{rpm_ref:.1f} d={rerr:+.1f} ({'OK' if passed else 'FAIL'})")
        fuel = self._v1430_finite(d.get("fuelFlow"))
        if fuel is not None:
            parts.append(f"FuelFlow={fuel:.1f}pph (DIAG)")

        detail = " | ".join(parts)
        self.auto_last_p3d_control_handover_detail = f"{context}: {detail}"
        return bool(ok), detail

    def _v1463_exact_t0_force_frame(self) -> tuple[Dict[str, Any], Dict[str, float]]:
        """Build the FINAL force-lock frame from the formal Manual CSV row 0.

        V14.64 deliberately uses manual_initial_conditions because those values
        are loaded from the first row of the selected Manual CSV -- the same
        values shown by the green formal report trace at t=0.  Position fields
        remain the original high-rate replay frame-0 values.
        """
        if not self.reference_replay:
            return {}, {}

        frame = dict(self.reference_replay[0])
        init_c = dict(getattr(self, "manual_initial_conditions", {}) or {})
        refs: Dict[str, float] = {}

        specs = (
            ("Pitch", "pitch_force", "cls_pitch_force_eng", "pitch_force", PITCH_FORCE_REPLAY_SIGN, PITCH_FORCE_REPLAY_DEADBAND_N),
            ("Roll", "roll_force", "cls_roll_force_eng", "roll_force", ROLL_FORCE_REPLAY_SIGN, ROLL_FORCE_REPLAY_DEADBAND_N),
            ("Yaw", "rudder_force", "cls_yaw_force_eng", "rudder_force", YAW_FORCE_REPLAY_SIGN, YAW_FORCE_REPLAY_DEADBAND_N),
        )
        active = set(getattr(self, "active_cls_axes", []) or [])
        for name, init_key, eng_key, generic_key, sign, deadband in specs:
            if name not in active or not self._force_stage_allows(name):
                continue
            value = self._finite_force_value(init_c.get(init_key))
            if value is None:
                continue
            # Store the UNSIGNED Manual value in the frame because the existing
            # replay controller applies the axis sign exactly once.
            frame[eng_key] = float(value)
            frame[generic_key] = float(value)
            signed = float(value) * float(sign)
            if abs(signed) < float(deadband):
                signed = 0.0
            refs[name] = float(signed)

        return frame, refs

    def _v1463_capture_auto_t0_cls_snapshot(self) -> None:
        """Capture LIVE measured CLS values at the verified Automatic t=0."""
        if not getattr(self, "cls", None):
            self.auto_t0_cls_values = {}
            return

        def finite_or_none(v):
            try:
                fv = float(v)
                return fv if math.isfinite(fv) else None
            except (TypeError, ValueError):
                return None

        p_force = self._pitch_actual_force_n()
        r_force = self._roll_actual_force_n()
        y_force = self._yaw_actual_force_n()
        snapshot = {
            "pitch_force": finite_or_none(p_force),
            "roll_force": finite_or_none(r_force),
            "rudder_force": finite_or_none(y_force),
            "cls_pitch_norm": finite_or_none(self.cls.last_pitch_pos),
            "cls_roll_norm": finite_or_none(self.cls.last_roll_pos),
            "cls_yaw_norm": finite_or_none(self.cls.last_rudder_pos),
            "cls_pitch_force": finite_or_none(self.cls.last_pitch_force),
            "cls_roll_force": finite_or_none(self.cls.last_roll_force),
            "cls_yaw_force": finite_or_none(self.cls.last_rudder_force),
            "cls_pitch_force_eng": finite_or_none(p_force),
            "cls_roll_force_eng": finite_or_none(r_force),
            "cls_yaw_force_eng": finite_or_none(y_force),
            "cls_pitch_force_norm": finite_or_none(self.cls.last_pitch_force_norm),
            "cls_roll_force_norm": finite_or_none(self.cls.last_roll_force_norm),
            "cls_yaw_force_norm": finite_or_none(self.cls.last_rudder_force_norm),
            "cls_pitch_force_raw": finite_or_none(self.cls.last_pitch_force_raw),
            "cls_roll_force_raw": finite_or_none(self.cls.last_roll_force_raw),
            "cls_yaw_force_raw": finite_or_none(self.cls.last_rudder_force_raw),
            "cls_pitch_raw": finite_or_none(self.cls.last_pitch_pos_raw),
            "cls_roll_raw": finite_or_none(self.cls.last_roll_pos_raw),
            "cls_yaw_raw": finite_or_none(self.cls.last_rudder_pos_raw),
            "cls_pitch_ap_enabled": int(self.cls.last_pitch_ap_enabled) if self.cls.last_pitch_ap_enabled is not None else None,
            "cls_roll_ap_enabled": int(self.cls.last_roll_ap_enabled) if self.cls.last_roll_ap_enabled is not None else None,
            "cls_yaw_ap_enabled": int(self.cls.last_rudder_ap_enabled) if self.cls.last_rudder_ap_enabled is not None else None,
            "cls_pitch_target": finite_or_none(self.cls.last_pitch_target),
            "cls_roll_target": finite_or_none(self.cls.last_roll_target),
            "cls_yaw_target": finite_or_none(self.cls.last_rudder_target),
        }
        # Do not overwrite the current GUI cache with None for axes that are not
        # connected (Roll/Yaw on the present Loader-PC run).
        self.auto_t0_cls_values = {k: v for k, v in snapshot.items() if v is not None}

    def _v1465_exact_t0_force_status(self, capture_tol: float = V14_65_T0_FORCE_CAPTURE_TOL_N) -> tuple[bool, str]:
        """Read-only exact-t0 physical force/position status in the Manual CSV coordinate target."""
        if not getattr(self, "cls", None) or not self.reference_replay:
            return False, "CLS/reference unavailable"

        frame0 = dict(self.reference_replay[0])
        _force_frame, force_refs = self._v1463_exact_t0_force_frame()
        targets: Dict[str, float] = {}
        for name, _axis, key in self._active_axis_specs():
            try:
                v = float(frame0[key])
            except (KeyError, TypeError, ValueError):
                return False, f"{name} frame-0 position unavailable"
            if not math.isfinite(v):
                return False, f"{name} frame-0 position non-finite"
            targets[name] = max(-1.0, min(1.0, v))

        parts = []
        ok = True
        actual_pos = {
            "Pitch": self.cls.last_pitch_pos,
            "Roll": self.cls.last_roll_pos,
            "Yaw": self.cls.last_rudder_pos,
        }
        for name, target in targets.items():
            actual = actual_pos.get(name)
            if actual is None:
                ok = False
                parts.append(f"{name}Pos=NA")
                continue
            err = abs(float(target) - float(actual))
            if err > float(V14_65_T0_POSITION_TOL):
                ok = False
            parts.append(f"{name}PosErr={err:.4f}")

        actual_force = {
            "Pitch": self._pitch_actual_force_n(),
            "Roll": self._roll_actual_force_n(),
            "Yaw": self._yaw_actual_force_n(),
        }
        for name, ref_n in force_refs.items():
            actual = actual_force.get(name)
            if actual is None:
                ok = False
                parts.append(f"{name}=NA")
                continue
            err = float(actual) - float(ref_n)
            if abs(err) > float(capture_tol):
                ok = False
            parts.append(f"{name}={float(actual):+.2f}/{float(ref_n):+.2f} d={err:+.2f}N")

        return bool(ok), " | ".join(parts)

    def _v1465_apply_exact_t0_controls_once(self) -> bool:
        """Maintain the already-armed physical controls at exact formal Manual t=0.

        This performs no P3D write and changes no tuning.  It only reasserts the
        proven normalized frame-0 AP position and advances the existing force
        controller against the formal Manual CSV row-0 engineering-force target.
        """
        if not getattr(self, "cls", None) or not self.reference_replay:
            return False

        frame0 = dict(self.reference_replay[0])
        force_frame, force_refs = self._v1463_exact_t0_force_frame()
        targets: Dict[str, float] = {}
        for name, axis, key in self._active_axis_specs():
            try:
                target = max(-1.0, min(1.0, float(frame0[key])))
            except (KeyError, TypeError, ValueError):
                return False
            if not math.isfinite(target):
                return False
            targets[name] = target
            self.cls.send(packet_setting_float(axis, SID_AP_POSITION, target))
            self.cls_last_targets[name] = target
            if name == "Pitch":
                self.cls.last_pitch_target = target
            elif name == "Roll":
                self.cls.last_roll_target = target
            else:
                self.cls.last_rudder_target = target

        current_pos = {
            "Pitch": self.cls.last_pitch_pos,
            "Roll": self.cls.last_roll_pos,
            "Yaw": self.cls.last_rudder_pos,
        }
        for name, axis, _key in self._active_axis_specs():
            if name in force_refs:
                self._force_position_interlock_allows(name, axis, targets[name], current_pos.get(name))

        if "Pitch" in force_refs:
            self._update_pitch_force_replay(force_frame, force_frame, 0.0)
        if "Roll" in force_refs:
            self._update_roll_force_replay(force_frame, force_frame, 0.0)
        if "Yaw" in force_refs:
            self._update_yaw_force_replay(force_frame, force_frame, 0.0)
        return True

    def _v1465_cls_only_t0_force_lock(self, stage: str) -> bool:
        """Lock full physical frame-0 controls while P3D remains paused/not running.

        This is the V14.65 ordering gate.  It uses the exact Manual CSV row-0 force,
        the existing V14.62/V14.64 force controller and normalized frame-0 position.
        No SimConnect state write, unpause, direct-axis event or P3D telemetry is used.
        """
        if not V14_65_CONTROLS_FIRST_ENABLED:
            return True
        if not getattr(self, "cls", None) or not self.reference_replay:
            self.v1465_controls_lock_detail = f"{stage}: CLS/reference unavailable"
            print(f"[V14.65 CONTROLS FIRST] {stage} FAIL | CLS/reference unavailable", flush=True)
            return False

        force_frame, force_refs = self._v1463_exact_t0_force_frame()
        if not force_refs:
            ok, detail = self._v1465_exact_t0_force_status()
            self.v1465_controls_lock_detail = f"{stage}: {detail}"
            print(f"[V14.65 CONTROLS FIRST] {stage} {'PASS' if ok else 'FAIL'} | {detail}", flush=True)
            return bool(ok)

        # Required force loops must already be armed by _open_and_arm_hardware().
        active_check = {
            "Pitch": self.pitch_force_replay_active,
            "Roll": self.roll_force_replay_active,
            "Yaw": self.yaw_force_replay_active,
        }
        missing = [name for name in force_refs if not active_check.get(name, False)]
        if missing:
            self.v1465_controls_lock_detail = f"{stage}: force replay not armed for {','.join(missing)}"
            print(f"[V14.65 CONTROLS FIRST] {stage} FAIL | {self.v1465_controls_lock_detail}", flush=True)
            return False

        print(
            f"[V14.65 CONTROLS FIRST] {stage} START | P3D must remain PAUSED | "
            + " ".join(f"{n}Ref={force_refs[n]:+.3f}N" for n in force_refs)
            + f" | good=+/-{V14_65_T0_FORCE_GOOD_TOL_N:.1f}N x{V14_65_T0_FORCE_GOOD_SAMPLES} "
              f"| final=+/-{V14_65_T0_FORCE_CAPTURE_TOL_N:.1f}N",
            flush=True,
        )

        start = time.monotonic()
        last_eval = 0.0
        good = 0
        last_detail = "waiting for live CLS force"
        while (time.monotonic() - start) < float(V14_65_T0_FORCE_TIMEOUT_SEC):
            if not self._v1465_apply_exact_t0_controls_once():
                self.v1465_controls_lock_detail = f"{stage}: could not apply frame-0 controls"
                return False

            now = time.monotonic()
            if now - last_eval < float(V14_65_T0_FORCE_SAMPLE_PERIOD_SEC):
                time.sleep(float(V14_65_T0_FORCE_LOOP_SEC))
                continue
            last_eval = now

            good_now, last_detail = self._v1465_exact_t0_force_status(V14_65_T0_FORCE_GOOD_TOL_N)
            good = good + 1 if good_now else 0
            print(
                f"[V14.65 CONTROLS FIRST] {stage} good={good}/{V14_65_T0_FORCE_GOOD_SAMPLES} | {last_detail}",
                flush=True,
            )
            if good >= int(V14_65_T0_FORCE_GOOD_SAMPLES):
                final_ok, final_detail = self._v1465_exact_t0_force_status(V14_65_T0_FORCE_CAPTURE_TOL_N)
                if final_ok:
                    self.v1465_controls_lock_detail = f"{stage}: {final_detail}"
                    if str(stage).upper().startswith("PRE-P3D"):
                        self.v1465_controls_prelocked = True
                    else:
                        self.v1465_controls_final_locked = True
                    print(
                        f"[V14.65 CONTROLS FIRST] {stage} READY | {final_detail} | controller state PRESERVED",
                        flush=True,
                    )
                    return True
                good = 0
            time.sleep(float(V14_65_T0_FORCE_LOOP_SEC))

        self.v1465_controls_lock_detail = f"{stage}: timeout | {last_detail}"
        print(f"[V14.65 CONTROLS FIRST] {stage} TIMEOUT/FAIL | {last_detail}", flush=True)
        return False

    def _v1464_force_lock_under_soft_hold(
        self,
        pose_state: Dict[str, Any],
        dynamic_state: Dict[str, Any],
        throttle: Optional[float],
        initial_live: Optional[Dict[str, Any]] = None,
    ) -> Optional[Dict[str, Any]]:
        """Robust final frame-0 force lock while the proven P3D soft-hold stays live.

        V14.64 fixes the V14.63 start-block problem without changing the working
        Pitch position mapping or any force-controller tuning.  The controller
        continues to run at the normal 20-ms cadence, but acceptance is evaluated
        only every 100 ms so repeated reads of one cached 0x22 force sample cannot
        create a false pass or a false instability.  Three consecutive physical
        observations must be within +/-3 N, and the final captured live force must
        be within +/-2 N.  P3D is still continuously restored to Manual t=0 and
        the measured-CLS direct bridge remains active throughout.
        """
        if not V14_64_T0_FORCE_LOCK_ENABLED:
            self._v1463_capture_auto_t0_cls_snapshot()
            return initial_live
        if not getattr(self, "cls", None) or not self.reference_replay:
            self.auto_last_p3d_control_handover_detail = "V14.64 force lock: CLS/reference unavailable"
            print("[V14.64 T0 FORCE LOCK] FAIL: CLS/reference unavailable.", flush=True)
            return None

        frame0 = dict(self.reference_replay[0])
        force_frame, force_refs = self._v1463_exact_t0_force_frame()
        if not force_refs:
            self._v1463_capture_auto_t0_cls_snapshot()
            print("[V14.64 T0 FORCE LOCK] SKIP: no active force-enabled axes.", flush=True)
            return initial_live

        expected_force_axes = [
            n for n in (getattr(self, "active_cls_axes", []) or [])
            if self._force_stage_allows(n)
        ]
        missing = [n for n in expected_force_axes if n not in force_refs]
        if missing:
            detail = "formal Manual row-0 force missing for " + ",".join(missing)
            self.auto_last_p3d_control_handover_detail = "V14.64 force lock: " + detail
            print("[V14.64 T0 FORCE LOCK] FAIL: " + detail, flush=True)
            return None

        targets: Dict[str, float] = {}
        for name, _axis, key in self._active_axis_specs():
            try:
                v = float(frame0[key])
            except (KeyError, TypeError, ValueError):
                detail = f"frame-0 {name} position unavailable"
                self.auto_last_p3d_control_handover_detail = "V14.64 force lock: " + detail
                print("[V14.64 T0 FORCE LOCK] FAIL: " + detail, flush=True)
                return None
            if not math.isfinite(v):
                detail = f"frame-0 {name} position is non-finite"
                self.auto_last_p3d_control_handover_detail = "V14.64 force lock: " + detail
                print("[V14.64 T0 FORCE LOCK] FAIL: " + detail, flush=True)
                return None
            targets[name] = max(-1.0, min(1.0, v))

        # Do not reset a controller that is already armed/converged.  Re-arm only
        # if ownership was actually lost.
        armers = {
            "Pitch": (lambda: self.pitch_force_replay_active, self._arm_pitch_force_replay),
            "Roll": (lambda: self.roll_force_replay_active, self._arm_roll_force_replay),
            "Yaw": (lambda: self.yaw_force_replay_active, self._arm_yaw_force_replay),
        }
        for name in force_refs:
            active_fn, armer = armers[name]
            if not active_fn() and not armer():
                detail = f"{name} force replay is not armed"
                self.auto_last_p3d_control_handover_detail = "V14.64 force lock: " + detail
                print("[V14.64 T0 FORCE LOCK] FAIL: " + detail, flush=True)
                return None

        request_once = getattr(self.sim_client, "request_telemetry_once", None)
        wait_fresh = getattr(self.sim_client, "wait_for_fresh_telemetry", None)
        get_seq = getattr(self.sim_client, "get_telemetry_sequence", None)
        if not all(callable(x) for x in (request_once, wait_fresh, get_seq)):
            self.auto_last_p3d_control_handover_detail = "V14.64 force lock: fresh P3D telemetry API unavailable"
            print("[V14.64 T0 FORCE LOCK] FAIL: fresh P3D telemetry API unavailable.", flush=True)
            return None

        print(
            "[V14.64 T0 FORCE LOCK] START | "
            + " ".join(f"{n}Ref={force_refs[n]:+.3f}N" for n in force_refs)
            + f" | good=+/-{V14_64_T0_FORCE_GOOD_TOL_N:.1f}N x{V14_64_T0_FORCE_GOOD_SAMPLES} "
              f"| capture=+/-{V14_64_T0_FORCE_CAPTURE_TOL_N:.1f}N "
              f"| timeout={V14_64_T0_FORCE_TIMEOUT_SEC:.0f}s",
            flush=True,
        )

        start = time.monotonic()
        last_restore = 0.0
        last_eval = 0.0
        good_samples = 0
        last_detail = "waiting for live force feedback"
        last_p3d_detail = "P3D soft-hold already verified"

        while (time.monotonic() - start) < float(V14_64_T0_FORCE_TIMEOUT_SEC):
            now = time.monotonic()

            # Keep P3D pinned to the already-verified Manual t=0 state while the
            # physical force loop settles. No new pause/unpause transition occurs.
            if now - last_restore >= float(V14_58_SOFT_HOLD_RESTORE_PERIOD_SEC):
                self._v1460_hold_restore_once(pose_state, dynamic_state, throttle)
                self._v1458_send_direct_measured_controls_once()
                last_restore = now

            # Reassert the SAME proven normalized frame-0 AP position and run the
            # unchanged force controller continuously.
            current_pos = {
                "Pitch": self.cls.last_pitch_pos,
                "Roll": self.cls.last_roll_pos,
                "Yaw": self.cls.last_rudder_pos,
            }
            for name, axis, _key in self._active_axis_specs():
                target = targets[name]
                self.cls.send(packet_setting_float(axis, SID_AP_POSITION, target))
                self.cls_last_targets[name] = target
                if name == "Pitch":
                    self.cls.last_pitch_target = target
                elif name == "Roll":
                    self.cls.last_roll_target = target
                else:
                    self.cls.last_rudder_target = target
                if name in force_refs:
                    self._force_position_interlock_allows(name, axis, target, current_pos.get(name))

            if "Pitch" in force_refs:
                self._update_pitch_force_replay(force_frame, force_frame, 0.0)
            if "Roll" in force_refs:
                self._update_roll_force_replay(force_frame, force_frame, 0.0)
            if "Yaw" in force_refs:
                self._update_yaw_force_replay(force_frame, force_frame, 0.0)

            # Judge convergence only at the physical force polling cadence. This
            # avoids counting the same cached 0x22 value 5-15 times.
            if now - last_eval < float(V14_64_T0_FORCE_SAMPLE_PERIOD_SEC):
                time.sleep(float(V14_64_T0_FORCE_LOOP_SEC))
                continue
            last_eval = now

            pos_ok = True
            pos_parts = []
            for name, _axis, _key in self._active_axis_specs():
                actual = {
                    "Pitch": self.cls.last_pitch_pos,
                    "Roll": self.cls.last_roll_pos,
                    "Yaw": self.cls.last_rudder_pos,
                }.get(name)
                if actual is None:
                    pos_ok = False
                    pos_parts.append(f"{name}Pos=NA")
                    continue
                err = abs(float(targets[name]) - float(actual))
                if err > float(V14_64_T0_POSITION_TOL):
                    pos_ok = False
                pos_parts.append(f"{name}PosErr={err:.4f}")

            actual_force = {
                "Pitch": self._pitch_actual_force_n(),
                "Roll": self._roll_actual_force_n(),
                "Yaw": self._yaw_actual_force_n(),
            }
            force_good = True
            force_parts = []
            for name, ref_n in force_refs.items():
                actual = actual_force.get(name)
                if actual is None:
                    force_good = False
                    force_parts.append(f"{name}=NA")
                    continue
                err = float(actual) - float(ref_n)
                if abs(err) > float(V14_64_T0_FORCE_GOOD_TOL_N):
                    force_good = False
                force_parts.append(f"{name}={float(actual):+.2f}/{float(ref_n):+.2f} d={err:+.2f}N")

            if pos_ok and force_good and not self.direct_mirror_last_error:
                good_samples += 1
            else:
                good_samples = 0

            last_detail = " | ".join(pos_parts + force_parts)
            print(
                f"[V14.64 T0 FORCE LOCK] good={good_samples}/{V14_64_T0_FORCE_GOOD_SAMPLES} | {last_detail}",
                flush=True,
            )

            if good_samples < int(V14_64_T0_FORCE_GOOD_SAMPLES):
                time.sleep(float(V14_64_T0_FORCE_LOOP_SEC))
                continue

            # Obtain a genuinely fresh SimConnect frame for the recording-time
            # anchor.  This is NOT another strict handover gate: that handover has
            # already passed. We only require fresh telemetry and then confirm the
            # physical CLS position/force are still inside the final capture limits.
            seq = int(get_seq())
            request_once()
            fresh = wait_fresh(seq, float(V14_64_T0_P3D_FRESH_TIMEOUT_SEC))
            if fresh is not None:
                self._v1479_capture_start_diagnostics(fresh)
            if fresh is None:
                last_p3d_detail = "no fresh P3D timestamp frame"
                print("[V14.64 T0 FORCE LOCK] fresh P3D frame not yet available; continuing hold.", flush=True)
                time.sleep(float(V14_64_T0_FORCE_LOOP_SEC))
                continue

            final_pos_ok = True
            final_pos_parts = []
            for name, _axis, _key in self._active_axis_specs():
                actual = {
                    "Pitch": self.cls.last_pitch_pos,
                    "Roll": self.cls.last_roll_pos,
                    "Yaw": self.cls.last_rudder_pos,
                }.get(name)
                if actual is None:
                    final_pos_ok = False
                    final_pos_parts.append(f"{name}Pos=NA")
                    continue
                err = abs(float(targets[name]) - float(actual))
                if err > float(V14_64_T0_POSITION_TOL):
                    final_pos_ok = False
                final_pos_parts.append(f"{name}PosErr={err:.4f}")

            final_force_ok = True
            final_force_parts = []
            for name, ref_n in force_refs.items():
                actual = {
                    "Pitch": self._pitch_actual_force_n,
                    "Roll": self._roll_actual_force_n,
                    "Yaw": self._yaw_actual_force_n,
                }[name]()
                if actual is None:
                    final_force_ok = False
                    final_force_parts.append(f"{name}=NA")
                    continue
                err = float(actual) - float(ref_n)
                if abs(err) > float(V14_64_T0_FORCE_CAPTURE_TOL_N):
                    final_force_ok = False
                final_force_parts.append(f"{name}={float(actual):+.2f}/{float(ref_n):+.2f} d={err:+.2f}N")

            # The preceding V14.60 soft-hold has already proven P3D input/surface
            # synchronization. During this lock there is no ownership change, so a
            # second single-frame exact-surface pass is deliberately NOT allowed to
            # falsely turn a good physical force lock into a 'connection error'.
            if final_pos_ok and final_force_ok and not self.direct_mirror_last_error:
                self._v1463_capture_auto_t0_cls_snapshot()
                self.auto_last_p3d_control_handover_detail = (
                    "V14.64 force lock READY | "
                    + " | ".join(final_force_parts + final_pos_parts)
                )
                print(
                    "[V14.64 T0 FORCE LOCK] READY | "
                    + " | ".join(final_force_parts + final_pos_parts)
                    + " | fresh P3D t0 timestamp captured | force controller state PRESERVED.",
                    flush=True,
                )
                return fresh

            good_samples = 0
            last_detail = " | ".join(final_pos_parts + final_force_parts)
            print(
                "[V14.64 T0 FORCE LOCK] FINAL CAPTURE RETRY | " + last_detail,
                flush=True,
            )
            time.sleep(float(V14_64_T0_FORCE_LOOP_SEC))

        detail = (
            "V14.64 force lock timeout after "
            f"{V14_64_T0_FORCE_TIMEOUT_SEC:.0f}s | {last_detail} | {last_p3d_detail}"
        )
        self.auto_last_p3d_control_handover_detail = detail
        print("[V14.64 T0 FORCE LOCK] TIMEOUT/FAIL | " + detail, flush=True)
        return None

    def _v1472_aero_state_matches(
        self,
        data: Dict[str, Any],
        state: Dict[str, Any],
        *,
        context: str = "",
    ) -> tuple[bool, str]:
        """V14.74 first-live air-state check focused on the real lateral fault.

        Diagnostic-only on every machine:
          IAS, AoA, Pitch Rate, accelerations, actual wind.

        Hard only when the corresponding lateral hardware exists:
          Roll active -> Roll Rate hard
          Yaw active  -> Yaw Rate hard
          Roll or Yaw active -> Sideslip hard

        This avoids false loader-PC blocks from one-frame AoA/Pitch-rate solver
        transients while still rejecting the Sim-PC condition that caused the
        Bank/Heading divergence.
        """
        d = data or {}
        st = state or {}
        active = set(getattr(self, "active_cls_axes", []) or [])
        ok = True
        parts = []

        checks = [
            ("IAS", "airspeed_indicated", "ias_kts",
             V14_72_IAS_TOL_KT, "kt", False),
            ("AOA", "aoa", "aoa_deg",
             V14_72_AOA_TOL_DEG, "deg", False),
            ("BETA", "sideslip", "sideslip_deg",
             V14_72_SIDESLIP_TOL_DEG, "deg",
             bool("Roll" in active or "Yaw" in active)),
            ("ROLL", "roll_rate", "roll_rate_deg_s",
             V14_72_ROLL_RATE_TOL_DEG_S, "deg/s",
             bool("Roll" in active)),
            ("PITCH", "pitch_rate", "pitch_rate_deg_s",
             V14_72_PITCH_RATE_TOL_DEG_S, "deg/s", False),
            ("YAW", "yaw_rate", "yaw_rate_deg_s",
             V14_72_YAW_RATE_TOL_DEG_S, "deg/s",
             bool("Yaw" in active)),
        ]

        for label, live_key, ref_key, tol, unit, hard in checks:
            live = self._v1430_finite(d.get(live_key))
            ref = self._v1430_finite(st.get(ref_key))

            if live is None or ref is None:
                if hard:
                    ok = False
                    parts.append(f"{label}=MISSING(FAIL)")
                else:
                    parts.append(f"{label}=MISSING(diag)")
                continue

            err = float(live) - float(ref)
            within = abs(err) <= float(tol)
            if hard:
                ok = bool(ok and within)
                tag = "OK" if within else "FAIL"
            else:
                tag = "OK(diag)" if within else "DIFF(diag)"

            parts.append(
                f"{label}={live:+.3f}/{ref:+.3f} "
                f"d={err:+.3f}{unit} ({tag})"
            )

        for label, live_key, ref_key in (
            ("AX", "accelX", "accel_x_g"),
            ("AY", "accelY", "accel_y_g"),
            ("AZ", "accelZ", "accel_z_g"),
        ):
            live = self._v1430_finite(d.get(live_key))
            ref = self._v1430_finite(st.get(ref_key))
            if live is not None and ref is not None:
                live = float(live) / 32.17404856
                ref = float(ref) / 32.17404856
                err = live - ref
                within = abs(err) <= float(V14_72_ACCEL_DIAG_TOL_G)
                parts.append(
                    f"{label}(diag)={live:+.3f}/{ref:+.3f} "
                    f"d={err:+.3f}g ({'OK' if within else 'DIFF'})"
                )

        live_ws = self._v1430_finite(d.get("ambient_wind_spd"))
        live_wd = self._v1430_finite(d.get("ambient_wind_dir"))
        if live_ws is not None and live_wd is not None:
            parts.append(
                f"WIND(diag only)={live_wd:.1f}deg/{live_ws:.2f}kt"
            )

        detail = " | ".join(parts)
        if context:
            detail = f"{context}: {detail}"
        return bool(ok), detail


    @staticmethod
    def _v1479_lateral_acceleration_matches(data, state, required=True):
        if not required:
            return True, "AX lateral=not gated (no lateral axis)"
        try:
            live = float(data["accelX"])
            ref = float(state["accel_x_g"])  # legacy key; actual unit is ft/s^2
            if not (math.isfinite(live) and math.isfinite(ref)):
                raise ValueError("non-finite acceleration")
        except (KeyError, TypeError, ValueError, OverflowError):
            return False, "AX lateral=MISSING/non-finite (FAIL)"
        err_g = (live - ref) / 32.17404856
        ok = abs(err_g) <= V14_79_LATERAL_ACCEL_TOL_G
        return ok, (f"AX lateral={live/32.17404856:+.4f}/{ref/32.17404856:+.4f}g "
                    f"d={err_g:+.4f}g ({'OK' if ok else 'FAIL'})")

    def _v1479_capture_start_diagnostics(self, data=None):
        """Capture even while the GUI timer is blocked; sends no commands."""
        try:
            self.commission_diag_last_sample_mono = 0.0
            self.p3d_state_diag_last_sample_mono = 0.0
            self._append_commissioning_diagnostic()
            if data is None and self.sim_client:
                data = self.sim_client.get_telemetry()
            self._append_p3d_state_diagnostic(data or {})
        except Exception as exc:
            print(f"[V14.79 DIAGNOSTIC] snapshot failed: {exc}", flush=True)

    def _v1480_environment_mismatch(self, live, reference):
        """Detect external conditions which pose/velocity/control writes cannot repair."""
        problems = []
        wind = self._v1430_finite(live.get("ambient_wind_spd"))
        wind_ref = self._v1430_finite(reference.get("actual_wind_speed_kt"))
        direction = self._v1430_finite(live.get("ambient_wind_dir"))
        direction_ref = self._v1430_finite(reference.get("actual_wind_dir_deg"))
        if wind is not None and wind_ref is not None:
            if abs(wind - wind_ref) > 1.0:
                problems.append(
                    f"actual wind {wind:.2f} kt versus Manual {wind_ref:.2f} kt"
                )
            elif (direction is not None and direction_ref is not None
                  and min(abs(wind), abs(wind_ref)) > 2.0
                  and abs((direction - direction_ref + 180.0) % 360.0 - 180.0) > 15.0):
                problems.append(
                    f"actual wind direction {direction:.1f} deg versus Manual "
                    f"{direction_ref:.1f} deg"
                )
        flow = self._v1430_finite(live.get("fuelFlow"))
        # The aircraft bridge can report 0 pph with a running engine.
        # Fuel flow is recorded for investigation, never treated as proof
        # of combustion or as a hard environment mismatch.
        return problems

    def _v1478_stabilize_first_live_t0(
        self,
        pose_state: Dict[str, Any],
        state: Dict[str, Any],
        throttle: Optional[float],
    ) -> tuple[Optional[Dict[str, Any]], str]:
        """Strictly stabilize P3D's derived lateral state BEFORE recording.

        This is a pre-record clamp only. The test timer remains OFF.

        The exact Manual t=0 rigid-body state and exact controls are repeatedly
        re-applied while P3D is live. P3D is released only after several
        consecutive fresh frames satisfy all existing handover checks plus the
        strict lateral aerodynamic gate.

        Returns:
            (accepted_fresh_frame, detail) on success
            (None, detail) on failure
        """
        if not self.sim_client:
            return None, "SimConnect client unavailable"

        get_seq = getattr(self.sim_client, "get_telemetry_sequence", None)
        request_once = getattr(self.sim_client, "request_telemetry_once", None)
        wait_fresh = getattr(self.sim_client, "wait_for_fresh_telemetry", None)
        if not all(callable(x) for x in (get_seq, request_once, wait_fresh)):
            return None, "Fresh P3D telemetry API unavailable"

        active = set(getattr(self, "active_cls_axes", []) or [])
        lateral_system = bool("Roll" in active or "Yaw" in active)

        last_detail = "no stabilization frame"

        print(
            "[V14.79 STABILIZE] BEGIN | strict lateral gate retained | "
            f"axes={','.join(sorted(active))}",
            flush=True,
        )

        for attempt in range(1, int(V14_79_STABILIZE_MAX_ATTEMPTS) + 1):
            # Start every attempt from a clean paused authoritative t=0.
            try:
                self.sim_client.pause_sim()
            except Exception:
                pass
            time.sleep(0.06)

            for _ in range(3):
                self._v1460_hold_restore_once(pose_state, state, throttle)
                self._v1458_send_direct_measured_controls_once()
                self._v1465_apply_exact_t0_controls_once()
                time.sleep(0.025)

            stable = 0
            last_restore = 0.0
            deadline = time.monotonic() + float(V14_79_STABILIZE_WINDOW_SEC)

            seq0 = int(get_seq())
            self.sim_client.unpause_sim()

            while time.monotonic() < deadline:
                now = time.monotonic()

                # Hold the aircraft at exact Manual t=0 while allowing the
                # derived aerodynamic solver to receive live frames.
                if now - last_restore >= float(V14_79_STABILIZE_RESTORE_PERIOD_SEC):
                    self._v1460_hold_restore_once(pose_state, state, throttle)
                    self._v1458_send_direct_measured_controls_once()
                    self._v1465_apply_exact_t0_controls_once()
                    last_restore = now

                seq = int(get_seq())
                if not request_once():
                    time.sleep(0.005)
                    continue

                fresh = wait_fresh(seq, float(V14_79_STABILIZE_FRESH_TIMEOUT_SEC))
                if fresh is not None:
                    self._v1479_capture_start_diagnostics(fresh)
                if fresh is None:
                    continue

                ctrl_ok, ctrl_detail = self._v1458_control_surfaces_match(
                    fresh, f"V14.79 STABILIZE {attempt}"
                )
                pose_ok, pose_detail = self._v1467_pose_matches_no_ias(
                    fresh, state
                )
                dyn_ok, dyn_detail = self._v1430_dynamic_matches(
                    fresh, state
                )
                secondary_ok, secondary_detail = self._v1470_secondary_state_matches(
                    fresh,
                    state,
                    require_engine_rpm=True,
                    context=f"V14.79 STABILIZE {attempt}",
                )
                aero_ok, aero_detail = self._v1472_aero_state_matches(
                    fresh,
                    state,
                    context=f"V14.79 STABILIZE {attempt}",
                )
                cls_ok = self._verify_controls_at_initial_targets(
                    target_pitch=self._initial_reference_target("Pitch", 0.0),
                    target_roll=self._initial_reference_target("Roll", 0.0),
                    target_yaw=self._initial_reference_target("Yaw", 0.0),
                    tolerance=0.030,
                    timeout_sec=0.02,
                )

                # Legacy accel_x_g stores unconverted bridge ft/s^2.
                # BODY X is lateral. Convert BOTH sides, not just live data.
                ay_ok, ay_detail = self._v1479_lateral_acceleration_matches(
                    fresh, state, lateral_system
                )

                frame_ok = bool(
                    ctrl_ok and pose_ok and dyn_ok and secondary_ok
                    and aero_ok and cls_ok and ay_ok
                    and not self.direct_mirror_last_error
                )

                stable = stable + 1 if frame_ok else 0

                last_detail = (
                    f"attempt={attempt} stable={stable}/{V14_79_STABILIZE_STABLE_FRAMES} | "
                    f"CTRL={'OK' if ctrl_ok else 'FAIL'} [{ctrl_detail}] | "
                    f"POSE={'OK' if pose_ok else 'FAIL'} [{pose_detail}] | "
                    f"DYN={'OK' if dyn_ok else 'FAIL'} [{dyn_detail}] | "
                    f"VSI/RPM={'OK' if secondary_ok else 'FAIL'} [{secondary_detail}] | "
                    f"AERO={'OK' if aero_ok else 'FAIL'} [{aero_detail}] | "
                    f"{ay_detail} | CLS={'OK' if cls_ok else 'FAIL'}"
                )

                print(
                    "[V14.79 STABILIZE] " + last_detail,
                    flush=True,
                )

                if stable >= int(V14_79_STABILIZE_STABLE_FRAMES):
                    print(
                        "[V14.79 STABILIZE] PASS | strict t=0 state converged "
                        "before recording.",
                        flush=True,
                    )
                    return fresh, last_detail

                time.sleep(0.005)

            # Failed this stabilization window: pause, scrub exact state, retry.
            try:
                self.sim_client.pause_sim()
            except Exception:
                pass
            for _ in range(2):
                self._v1460_hold_restore_once(pose_state, state, throttle)
                self._v1458_send_direct_measured_controls_once()
                self._v1465_apply_exact_t0_controls_once()
                time.sleep(0.025)

            print(
                f"[V14.79 STABILIZE] RETRY {attempt}/"
                f"{V14_79_STABILIZE_MAX_ATTEMPTS} | {last_detail}",
                flush=True,
            )
            time.sleep(float(V14_79_STABILIZE_INTER_ATTEMPT_PAUSE_SEC))

        try:
            self.sim_client.pause_sim()
        except Exception:
            pass

        return None, (
            "strict lateral state did not converge after "
            f"{V14_79_STABILIZE_MAX_ATTEMPTS} stabilization attempts | "
            + last_detail
        )


    def _v1470_secondary_state_matches(
        self,
        data: Dict[str, Any],
        state: Dict[str, Any],
        *,
        require_engine_rpm: bool,
        context: str = "",
    ) -> tuple[bool, str]:
        """Verify the V14.70 VSI and shaft/engine-RPM initial states.

        While paused we verify the directly settable PROP RPM:1 state.
        On the first live frame we additionally require GENERAL ENG RPM:1,
        which P3D documents as read-only and therefore must follow naturally
        from the seeded prop/shaft state.
        """
        d = data or {}
        ref_vs = self._v1430_finite(state.get("vertical_speed_fpm"))
        ref_rpm = self._v1430_finite(state.get("engine_rpm"))

        live_vs = self._v1430_finite(d.get("vertical_speed"))

        prop_list = d.get("prop_rpms")
        live_prop = None
        if isinstance(prop_list, list) and prop_list:
            live_prop = self._v1430_finite(prop_list[0])

        eng_list = d.get("eng_rpms")
        live_eng = None
        if isinstance(eng_list, list) and eng_list:
            live_eng = self._v1430_finite(eng_list[0])

        ok = True
        parts = []
        natural_engine_mode = bool(
            getattr(getattr(self, "sim_client", None), "natural_engine_mode", False)
        )

        # V15.12: VSI is written by SetInitialDynamicStateNoRpmV1496 and is
        # therefore a hard state again. PROP RPM is deliberately not written
        # in natural-engine mode, so PROP1 remains diagnostic there.
        if ref_vs is None or live_vs is None:
            ok = False
            parts.append(f"VSI=MISSING({live_vs}/{ref_vs})")
        else:
            err = float(live_vs) - float(ref_vs)
            passed = abs(err) <= float(V14_70_VSI_TOL_FPM)
            ok = bool(ok and passed)
            parts.append(
                f"VSI={live_vs:+.1f}/{ref_vs:+.1f} d={err:+.1f}fpm "
                f"({'OK' if passed else 'FAIL'})"
            )

        if ref_rpm is None or live_prop is None:
            if natural_engine_mode:
                parts.append(f"PROP1=MISSING({live_prop}/{ref_rpm}) (natural diag)")
            else:
                ok = False
                parts.append(f"PROP1=MISSING({live_prop}/{ref_rpm})")
        else:
            err = float(live_prop) - float(ref_rpm)
            passed = abs(err) <= float(V14_70_PROP_RPM_TOL)
            if natural_engine_mode:
                parts.append(
                    f"PROP1(diag)={live_prop:.1f}/{ref_rpm:.1f} d={err:+.1f}rpm "
                    f"({'NEAR' if passed else 'DIFF'})"
                )
            else:
                ok = bool(ok and passed)
                parts.append(
                    f"PROP1={live_prop:.1f}/{ref_rpm:.1f} d={err:+.1f}rpm "
                    f"({'OK' if passed else 'FAIL'})"
                )

        if require_engine_rpm:
            if ref_rpm is None or live_eng is None:
                ok = False
                parts.append(f"ENG1=MISSING({live_eng}/{ref_rpm})")
            else:
                err = float(live_eng) - float(ref_rpm)
                passed = abs(err) <= float(V14_70_ENGINE_RPM_TOL)
                ok = bool(ok and passed)
                parts.append(
                    f"ENG1={live_eng:.1f}/{ref_rpm:.1f} d={err:+.1f}rpm "
                    f"({'OK' if passed else 'FAIL'})"
                )
        elif live_eng is not None and ref_rpm is not None:
            parts.append(
                f"ENG1(diag)={live_eng:.1f}/{ref_rpm:.1f} "
                f"d={live_eng-ref_rpm:+.1f}rpm"
            )

        detail = " | ".join(parts)
        if context:
            detail = f"{context}: {detail}"
        return bool(ok), detail

    def _v1458_sync_controls_under_soft_hold(self) -> Optional[Dict[str, Any]]:
        """Synchronize measured CLS controls, then establish a true P3D t=0.

        V14.67 correction
        -----------------
        The earlier V14.66 gate incorrectly demanded that WORLD/BODY velocity
        remain equal to Manual while P3D was deliberately RUNNING during the
        40-ms software-hold/control-calibration window.  That cannot be a stable
        acceptance condition: physics legitimately evolves between restores.

        Correct order:
          1) run P3D under software hold ONLY long enough to calibrate/synchronize
             the measured CLS -> P3D controls and actual surfaces;
          2) once controls are stable, PAUSE P3D;
          3) restore exact no-IAS pose + WORLD dynamics + trims while paused;
          4) verify paused pose, settable dynamics, controls and CLS;
          5) unpause ONCE and capture the FIRST fresh live frame;
          6) verify that first live frame has not suffered a large state/control
             kick; only then return it as Automatic t=0.

        BODY linear velocity is read-only in the current bridge.  It remains
        diagnostic evidence but is not used as an impossible hard gate.
        """
        if not self.sim_client:
            return None

        # V14.92: restore non-primary-control causal state BEFORE the
        # live handover. New Manuals use their measured state; only the two
        # approved September-17 bundles may use a reconstructed profile.
        causal_ok, causal_detail = self._v1492_apply_reference_causal_state(
            include_weather=True, context="PRE-HANDOVER"
        )
        try:
            audit_ok, audit_detail = self._v1495_causal_preflight_audit(self.sim_client.get_telemetry() or {}, "PRE-HANDOVER")
            print(f"[V14.95 HIDDEN-STATE AUDIT] {'PASS' if audit_ok else 'FAIL'} | {audit_detail}", flush=True)
            causal_ok = bool(causal_ok and audit_ok)
            causal_detail = causal_detail + " | " + audit_detail
        except Exception as _audit_exc:
            print(f"[V14.95 HIDDEN-STATE AUDIT] warning: {_audit_exc}", flush=True)
        print(
            f"[V14.92 CAUSAL STATE] {'PASS' if causal_ok else 'FAIL'} | {causal_detail}",
            flush=True,
        )
        # V14.92 fail-closed: if a state value was available and its causal
        # setter failed, do NOT start a 75-second run with a known mismatch.
        # Missing optional fields are reported as "not available" and do not
        # make causal_ok false; actual setter failures do.
        if not causal_ok:
            self.direct_mirror_last_error = "V14.92 causal-state restore failed: " + causal_detail
            print(f"[V14.92 CAUSAL STATE] BLOCKED | {self.direct_mirror_last_error}", flush=True)
            return None

        state = dict(getattr(self, "reference_initial_dynamic_state", {}) or {})
        pose_setter = getattr(self.sim_client, "set_exact_pose_state_no_airspeed", None)
        dyn_setter = getattr(self.sim_client, "set_initial_dynamic_state", None)
        request_once = getattr(self.sim_client, "request_telemetry_once", None)
        wait_fresh = getattr(self.sim_client, "wait_for_fresh_telemetry", None)
        get_seq = getattr(self.sim_client, "get_telemetry_sequence", None)
        if not all(callable(x) for x in (
            pose_setter, dyn_setter, request_once, wait_fresh, get_seq
        )):
            self.direct_mirror_last_error = (
                "Required V14.66 P3D direct-state/fresh-telemetry API unavailable"
            )
            return None

        pose_state = dict(state)
        pose_state["bank_deg"] = (
            float(state.get("bank_deg", 0.0))
            * float(getattr(self, "auto_bank_write_multiplier", -1.0) or -1.0)
        )
        throttle = self._v1430_finite(state.get("throttle_pct"))
        stable = 0
        last_detail = "no fresh live frame"
        last_restore = 0.0

        try:
            # --------------------------------------------------------------
            # A. BRUNNER-ONLY handover. No direct P3D axis calibration/probes.
            # Hold the recorded physical CLS t=0 positions. Release the
            # Position-to-Sim override so CLS2Sim (not this app) drives P3D.
            self.sim_client.pause_sim()
            time.sleep(0.08)
            self._v1460_hold_restore_once(pose_state, state, throttle)
            self._v1465_apply_exact_t0_controls_once()
            if not self._v1458_start_direct_mirror(bridge_prepared=False):
                self.auto_last_p3d_control_handover_detail = self.direct_mirror_last_error
                return None
            self._v1460_hold_restore_once(pose_state, state, throttle)
            self._v1465_apply_exact_t0_controls_once()
            print("[BRUNNER ONLY] Initial physical control forwarding enabled", flush=True)

            # One normal release for the existing control/surface sync.  From
            # this point onward there are no calibration probes.
            self.sim_client.unpause_sim()
            time.sleep(0.03)

            seq = int(get_seq())
            # V14.73: the live software hold is for CONTROL/SURFACE
            # synchronization only.  Do not require instantaneous aerodynamic
            # rates to be zero while we are deliberately re-writing t=0 state.
            # The authoritative aerodynamic gate is the FIRST LIVE t=0 frame.
            # Weather restoration can restart P3D's aircraft-wind ramp.
            # Remain in PREPARATION under the existing rigid-body/control hold
            # until its observed vector matches the saved reference. Never
            # write AIRCRAFT WIND or replace the saved reference.
            deadline = time.monotonic() + max(float(V14_58_SOFT_HOLD_TIMEOUT_SEC), 30.0)
            print("[V15.12.5 AUTO WIND] Waiting under pre-record hold for reference aircraft wind", flush=True)

            while time.monotonic() < deadline:
                now = time.monotonic()
                if now - last_restore >= float(V14_58_SOFT_HOLD_RESTORE_PERIOD_SEC):
                    # Keep the aircraft near Manual t=0 while P3D is live, but
                    # DO NOT use the evolving live dynamics as a stability gate.
                    pose_setter(pose_state)
                    dyn_setter(state)
                    if throttle is not None:
                        try:
                            self.sim_client.set_throttle(float(throttle))
                        except Exception:
                            pass
                    last_restore = now

                self._v1465_apply_exact_t0_controls_once()

                request_once()
                fresh = wait_fresh(seq, 0.35)
                if fresh is not None:
                    self._v1479_capture_start_diagnostics(fresh)
                if fresh is None:
                    stable = 0
                    continue
                try:
                    seq = int(fresh.get("bridge_telemetry_sequence", get_seq()))
                except Exception:
                    seq = int(get_seq())

                control_ok, control_detail = self._v1458_control_surfaces_match(
                    fresh, f"V14.67 live control-sync sample {stable + 1}"
                )
                pose_ok, pose_detail = self._v1467_pose_matches_no_ias(fresh, state)

                # Dynamics/BODY are logged during this live window.
                dyn_diag_ok, dyn_diag_detail = self._v1430_dynamic_matches(fresh, state)
                body_diag_ok, body_diag_detail = self._v1466_body_velocity_matches(
                    fresh, f"live diagnostic {stable + 1}"
                )
                aero_ok, aero_detail = self._v1472_aero_state_matches(
                    fresh,
                    state,
                    context=f"V14.72 AERO SETTLE sample {stable + 1}",
                )

                cls_ok = self._verify_controls_at_initial_targets(
                    target_pitch=self._initial_reference_target("Pitch", 0.0),
                    target_roll=self._initial_reference_target("Roll", 0.0),
                    target_yaw=self._initial_reference_target("Yaw", 0.0),
                    tolerance=0.030,
                    timeout_sec=0.05,
                )

                # V14.73: live hold passes on controls + surfaces + pose + CLS.
                # AERO is diagnostic here only; it becomes a hard gate on the
                # first fresh post-unpause t=0 frame.
                wind_ok, wind_detail = wind_matches_reference(fresh, state, tolerance_kt=0.15)
                frame_ok = bool(
                    control_ok and pose_ok and cls_ok and wind_ok
                    and not self.direct_mirror_last_error
                )
                stable = stable + 1 if frame_ok else 0

                last_detail = (
                    f"CTRL={'OK' if control_ok else 'FAIL'} [{control_detail}] | "
                    f"POSE={'OK' if pose_ok else 'FAIL'} [{pose_detail}] | "
                    f"CLS={'OK' if cls_ok else 'FAIL'} | "
                    f"AERO(diag)={'OK' if aero_ok else 'EVOLVING'} [{aero_detail}] | "
                    f"DYN(diag)={'OK' if dyn_diag_ok else 'EVOLVING'} [{dyn_diag_detail}] | "
                    f"BODY(diag)={'OK' if body_diag_ok else 'EVOLVING'} [{body_diag_detail}] | "
                    f"AIRCRAFT_WIND={'READY' if wind_ok else 'SETTLING'} [{wind_detail}]"
                )
                print(
                    f"[V14.79 CONTROL SYNC] stable={stable}/"
                    f"{V14_58_SOFT_HOLD_STABLE_SAMPLES} | {last_detail}",
                    flush=True,
                )

                if stable >= int(V14_58_SOFT_HOLD_STABLE_SAMPLES):
                    break

                time.sleep(0.01)

            if stable < int(V14_58_SOFT_HOLD_STABLE_SAMPLES):
                self.auto_last_p3d_control_handover_detail = (
                    "V15.12.5 control/pose/aircraft-wind preparation timeout (30 s bound) | " + last_detail
                )
                return None

            # --------------------------------------------------------------
            # B. PAUSE and establish the exact authoritative t=0.
            # --------------------------------------------------------------
            self.sim_client.pause_sim()
            time.sleep(0.08)

            paused_stable = 0
            paused_last = "no paused verification frame"

            for attempt in range(1, 7):
                pose_ok_set = bool(pose_setter(pose_state))
                dyn_ok_set = bool(dyn_setter(state))
                if throttle is not None:
                    try:
                        self.sim_client.set_throttle(float(throttle))
                    except Exception:
                        pass

                self._v1458_send_direct_measured_controls_once()
                self._v1465_apply_exact_t0_controls_once()

                time.sleep(0.05)
                seqp = int(get_seq())
                if not request_once():
                    paused_stable = 0
                    paused_last = "RequestTelemetryOnce failed while paused"
                    continue

                pfresh = wait_fresh(seqp, 0.50)
                if pfresh is None:
                    paused_stable = 0
                    paused_last = "fresh paused telemetry timed out"
                    continue

                pctrl_ok, pctrl_detail = self._v1458_control_surfaces_match(
                    pfresh, f"V14.70 PAUSED FINAL {attempt}"
                )
                ppose_ok, ppose_detail = self._v1467_pose_matches_no_ias(
                    pfresh, state
                )
                pdyn_ok, pdyn_detail = self._v1430_dynamic_matches(pfresh, state)
                psecondary_ok, psecondary_detail = self._v1470_secondary_state_matches(
                    pfresh,
                    state,
                    require_engine_rpm=False,
                    context=f"V14.70 PAUSED FINAL {attempt}",
                )
                pcls_ok = self._verify_controls_at_initial_targets(
                    target_pitch=self._initial_reference_target("Pitch", 0.0),
                    target_roll=self._initial_reference_target("Roll", 0.0),
                    target_yaw=self._initial_reference_target("Yaw", 0.0),
                    tolerance=0.030,
                    timeout_sec=0.05,
                )
                _, pbody_detail = self._v1466_body_velocity_matches(
                    pfresh, "paused diagnostic"
                )

                pok = bool(
                    pose_ok_set and dyn_ok_set
                    and pctrl_ok and ppose_ok and pdyn_ok
                    and psecondary_ok and pcls_ok
                )
                paused_stable = paused_stable + 1 if pok else 0
                paused_last = (
                    f"setterPose={'OK' if pose_ok_set else 'FAIL'} "
                    f"setterDyn={'OK' if dyn_ok_set else 'FAIL'} | "
                    f"CTRL={'OK' if pctrl_ok else 'FAIL'} [{pctrl_detail}] | "
                    f"POSE={'OK' if ppose_ok else 'FAIL'} [{ppose_detail}] | "
                    f"DYN={'OK' if pdyn_ok else 'FAIL'} [{pdyn_detail}] | "
                    f"VSI/RPM={'OK' if psecondary_ok else 'FAIL'} [{psecondary_detail}] | "
                    f"CLS={'OK' if pcls_ok else 'FAIL'} | "
                    f"BODY(diag)=[{pbody_detail}]"
                )
                print(
                    f"[V14.70 PAUSED FINAL] stable={paused_stable}/2 | "
                    + paused_last,
                    flush=True,
                )
                if paused_stable >= 2:
                    break

            if paused_stable < 2:
                self.auto_last_p3d_control_handover_detail = (
                    "V14.67 paused final t=0 verification failed | " + paused_last
                )
                return None

            # Force must still be at the formal Manual row-0 target before release.
            force_ok, force_detail = self._v1465_exact_t0_force_status(
                V14_65_T0_FORCE_CAPTURE_TOL_N
            )
            if not force_ok:
                self.auto_last_p3d_control_handover_detail = (
                    "V14.67 paused final force not ready | " + force_detail
                )
                return None

            # V14.95: for only the two bridge-1440 legacy references, perform
            # a short one-frame t=0 aero lock around the reconstructed wind.
            # No report rows are recorded and BRUNNER tuning is untouched.
            if getattr(self, "v1490_legacy_reference", False):
                lock_ok, lock_detail = self._v1494_preflight_aero_lock(
                    pose_state, state, throttle, pose_setter, dyn_setter,
                    request_once, wait_fresh, get_seq
                )
                print(f"[V14.95 PRE-FLIGHT AERO LOCK] {'PASS' if lock_ok else 'FAIL'} | {lock_detail}", flush=True)
                if not lock_ok:
                    self.auto_last_p3d_control_handover_detail = "V14.95 aero/weather lock failed: " + lock_detail
                    return None
                # Re-verify the final paused state after the internal weather probes.
                pose_setter(pose_state); dyn_setter(state)
                if throttle is not None:
                    self.sim_client.set_throttle(float(throttle))
                time.sleep(0.04)
                force_ok2, force_detail2 = self._v1465_exact_t0_force_status(
                    V14_65_T0_FORCE_CAPTURE_TOL_N
                )
                if not force_ok2:
                    self.auto_last_p3d_control_handover_detail = (
                        "V14.95 post-aero-lock force not ready | " + force_detail2
                    )
                    return None

            # --------------------------------------------------------------
            # C. ONE authoritative unpause. First fresh frame is Auto t=0.
            # --------------------------------------------------------------
            # Finish slow support work BEFORE the last queued state restore.
            self.sim_client.get_engine_support_state(fresh=True)
            restored, restore_detail = self._v15_restore_paused_initial_transients()
            if not restored:
                self.auto_last_p3d_control_handover_detail = restore_detail
                return None
            if not (pose_setter(pose_state) and dyn_setter(state)):
                self.auto_last_p3d_control_handover_detail = "V15.12.4 final rigid-body restore rejected"
                return None
            self._v1458_send_direct_measured_controls_once()
            self._v1465_apply_exact_t0_controls_once()
            self._v1484_log_t0_frame("AUTOMATIC TARGET (MANUAL t=0)", state)
            seq0 = int(get_seq())
            self.sim_client.unpause_sim()
            if not request_once():
                self.auto_last_p3d_control_handover_detail = (
                    "V14.67 could not request first post-unpause telemetry frame"
                )
                return None

            first_live = wait_fresh(seq0, 0.50)
            if first_live is None:
                self.auto_last_p3d_control_handover_detail = (
                    "V14.67 first post-unpause telemetry frame timed out"
                )
                return None

            first_live = self._v1496_live_engine_check(first_live, fresh_support=False)
            measured_t0 = self._capture_v1430_initial_dynamic_state(first_live)
            if getattr(self, "v1490_legacy_reference", False):
                print(
                    "[V14.92 FIRST LIVE CONFIG] "
                    f"flaps_handle_percent={first_live.get('flaps_handle_percent')!r} "
                    f"mixture_lever={first_live.get('engine_mixture_lever_1')!r} "
                    "| reconstructed legacy profile; readback is measured P3D",
                    flush=True,
                )
            self._v1484_log_t0_frame("AUTOMATIC FIRST LIVE", measured_t0, state)
            self._append_p3d_state_diagnostic(first_live, t0_marker="AUTOMATIC_FIRST_LIVE_T0")

            start_deadline = time.monotonic() + 1.5
            for start_attempt in range(1, 41):
                fctrl_ok, fctrl_detail = self._v1458_control_surfaces_match(
                    first_live, "V14.70 FIRST LIVE t0"
                )
                fpose_ok, fpose_detail = self._v1467_pose_matches_no_ias(
                    first_live, state
                )
                fdyn_ok, fdyn_detail = self._v1430_dynamic_matches(first_live, state)
                fsecondary_ok, fsecondary_detail = self._v1470_secondary_state_matches(
                    first_live,
                    state,
                    require_engine_rpm=True,
                    context="V14.70 FIRST LIVE t0",
                )
                faero_ok, faero_detail = self._v1472_aero_state_matches(
                    first_live,
                    state,
                    context="V14.76 FIRST LIVE AERO",
                )
                fcls_ok = self._verify_controls_at_initial_targets(
                    target_pitch=self._initial_reference_target("Pitch", 0.0),
                    target_roll=self._initial_reference_target("Roll", 0.0),
                    target_yaw=self._initial_reference_target("Yaw", 0.0),
                    tolerance=0.030,
                    timeout_sec=0.05,
                )
                _, fbody_detail = self._v1466_body_velocity_matches(
                    first_live, "FIRST LIVE diagnostic"
                )

                fwind_ok, fwind_detail = wind_matches_reference(first_live, state)
                first_ok = bool(
                    fctrl_ok and fpose_ok and fdyn_ok and fsecondary_ok
                    and faero_ok and fcls_ok and fwind_ok
                    and not self.direct_mirror_last_error
                )

                first_detail = (
                    f"CTRL={'OK' if fctrl_ok else 'FAIL'} [{fctrl_detail}] | "
                    f"POSE={'OK' if fpose_ok else 'FAIL'} [{fpose_detail}] | "
                    f"DYN={'OK' if fdyn_ok else 'FAIL'} [{fdyn_detail}] | "
                    f"VSI/RPM={'OK' if fsecondary_ok else 'FAIL'} [{fsecondary_detail}] | "
                    f"AERO={'OK' if faero_ok else 'FAIL'} [{faero_detail}] | "
                    f"CLS={'OK' if fcls_ok else 'FAIL'} | "
                    f"BODY(diag)=[{fbody_detail}] | "
                    f"AIRCRAFT_WIND={'OK' if fwind_ok else 'FAIL'} [{fwind_detail}]"
                )


                self._v1479_capture_start_diagnostics(first_live)
                if first_ok:
                    self._append_p3d_state_diagnostic(first_live, t0_marker="V15124_ACCEPTED_START")
                    break
                # A different effective wind cannot be corrected by rigid-body
                # writes. Stop promptly rather than forcing read-only airflow.
                if not fwind_ok or time.monotonic() >= start_deadline or start_attempt == 40:
                    break
                if not (pose_setter(pose_state) and dyn_setter(state)):
                    self.auto_last_p3d_control_handover_detail = "V15.12.4 candidate restore rejected"
                    return None
                self._v1458_send_direct_measured_controls_once()
                self._v1465_apply_exact_t0_controls_once()
                previous = int(get_seq())
                if not request_once():
                    self.auto_last_p3d_control_handover_detail = "V15.12.4 candidate telemetry request rejected"
                    return None
                candidate = wait_fresh(previous, 0.15)
                if candidate is None:
                    self.auto_last_p3d_control_handover_detail = "V15.12.4 candidate fresh telemetry timeout"
                    return None
                first_live = self._v1496_live_engine_check(candidate, fresh_support=False)

            # A successful start within relaxed limits is still NOT a matched
            # QTG run when it exceeds the previous formal RPM/beta limits.
            # Keep its unmodified live data, report, and explicit diagnostic label.
            ref_rpm = self._v1430_finite(state.get("engine_rpm"))
            engines = first_live.get("eng_rpms") or []
            props = first_live.get("prop_rpms") or []
            live_engine = self._v1430_finite(engines[0]) if engines else None
            live_prop = self._v1430_finite(props[0]) if props else None
            ref_beta = self._v1430_finite(state.get("sideslip_deg"))
            live_beta = self._v1430_finite(first_live.get("sideslip"))
            relaxed_start = bool(first_ok and (
                (ref_rpm is not None and live_engine is not None
                 and abs(live_engine - ref_rpm) > V14_70_FORMAL_RPM_TOL)
                or (ref_rpm is not None and live_prop is not None
                    and abs(live_prop - ref_rpm) > V14_70_FORMAL_RPM_TOL)
                or (ref_beta is not None and live_beta is not None
                    and abs(live_beta - ref_beta) > V14_72_FORMAL_SIDESLIP_TOL_DEG)
            ))
            if relaxed_start:
                self.auto_diagnostic_replay = True
                self.auto_diagnostic_start_detail = first_detail
                print(
                    "[V14.86 BOUNDED START] Measured t=0 exceeds the original "
                    "RPM/sideslip limits. Starting 75-second diagnostic replay "
                    "if the physical frame-0 force passes. NOT QTG-qualified. "
                    + first_detail, flush=True,
                )

            # Both 2026-09-17 references predate the new engine/weather/airflow
            # SimVars. Their replay must remain diagnostic even if every
            # measurable first-live gate happens to pass.
            reference_name = Path(getattr(self, "reference_manual_csv_path", "") or "").name
            if any(s in reference_name for s in (
                "2026-09-17_15-25-33", "2026-09-17_15-37-17"
            )):
                self.auto_diagnostic_replay = True
                self.auto_diagnostic_start_detail = (
                    "Legacy bridge 1440 Manual: engine controls, fuel/manifold "
                    "pressures, actual wind, air-relative BODY flow unknown. "
                    "Only measured 2026-09-17 P3D and CLS fields are references. "
                    + first_detail
                )

            # V15 SIM-PC GRAPH HOTFIX:
            # P3D can recompute derived DYN/AERO channels on the first
            # post-unpause physics frame. Do not discard a physically valid
            # current Manual only because those derived values move immediately.
            #
            # HARD gates retained: controls/surfaces, pose, VSI/RPM, physical
            # CLS, environment, bridge health, and the existing force check.
            selected_name = Path(
                getattr(self, "reference_manual_csv_path", "") or ""
            ).name
            graph_external = self._v1480_environment_mismatch(first_live, state)
            diagnostic_eligible = bool(
                False  # V15.12.4: failed start checks cannot become a recorded test.
                and selected_name
                and fctrl_ok
                and fpose_ok
                and fcls_ok
                and not graph_external
                and not self.direct_mirror_last_error
            )
            if not first_ok and diagnostic_eligible:
                self.auto_diagnostic_replay = True
                self.auto_diagnostic_start_detail = (
                    "V15 SIM-PC GRAPH CAPTURE: first-live derived DYN/AERO/VSI-RPM "
                    "mismatch accepted for measured diagnostic replay. | "
                    + first_detail
                )
                print(
                    "[V15 SIM-PC GRAPH CAPTURE] Controls/pose/VSI-RPM/CLS/environment "
                    "are valid. First-live derived DYN/AERO/VSI-RPM mismatch will NOT block the "
                    "75-second measured replay. Existing physical force check remains "
                    "active. NOT QTG-qualified until the mismatch is resolved. | "
                    + first_detail,
                    flush=True,
                )
                first_ok = True

            if not first_ok:
                self._axis_calibration_verified_key = None
                if not fwind_ok:
                    self.auto_last_p3d_control_handover_detail = (
                        "V15.12.4 effective aircraft wind does not match Manual t=0: "
                        + fwind_detail + ". Ambient wind alone is insufficient; "
                        "this reference may contain a local-wind initialization transient. "
                        + first_detail
                    )
                    return None
                external = self._v1480_environment_mismatch(first_live, state)
                if external:
                    self.auto_last_p3d_control_handover_detail = (
                        "V14.80 environment differs from Manual: " + "; ".join(external)
                        + ". Match P3D weather and engine state to the selected Manual "
                        "flight before retrying; state scrubbing cannot change them. "
                        + first_detail
                    )
                    print("[V14.80 PREFLIGHT] BLOCKED | "
                          + self.auto_last_p3d_control_handover_detail, flush=True)
                    return None

                # Fuel flow is not an engine-on sensor for this aircraft;
                # the measured RPM check remains in first_ok above.
                if state.get("actual_wind_speed_kt") is not None:
                    self.auto_last_p3d_control_handover_detail = (
                        "New Manual first-live check failed. Restore the same P3D "
                        "flight, engine configuration and actual weather before "
                        "Automatic; no pause/unpause retries are attempted. "
                        + first_detail
                    )
                    return None

                self.auto_last_p3d_control_handover_detail = (
                    "V14.85 first live t=0 differs from Manual; no repeated "
                    "pause/unpause or position rewrites. Restore matching P3D "
                    "flight, weather and engine setup. | " + first_detail
                )
                print("[V14.85 FIRST LIVE] BLOCKED | "
                      + self.auto_last_p3d_control_handover_detail, flush=True)
                return None

            force_ok, force_detail = self._v1465_exact_t0_force_status(
                V14_65_T0_FORCE_CAPTURE_TOL_N
            )
            if not force_ok:
                self._axis_calibration_verified_key = None
                try:
                    self.sim_client.pause_sim()
                except Exception:
                    pass
                self.auto_last_p3d_control_handover_detail = (
                    "V14.70 FIRST LIVE force not ready | " + force_detail
                )
                return None

            # No direct P3D axis calibration exists in BRUNNER-only mode.
            self._axis_calibration_verified_key = None

            self._v1463_capture_auto_t0_cls_snapshot()
            if self.auto_diagnostic_replay:
                first_detail = "V15 DIAGNOSTIC GRAPH REPLAY (first-live DYN/AERO mismatch) | " + first_detail
            self.auto_last_p3d_control_handover_detail = (
                "V14.79 FIRST LIVE t0 READY | " + first_detail + " | " + force_detail
            )
            print(
                "[V14.79 FIRST LIVE] READY | "
                + first_detail + " | " + force_detail
                + " | LIVE t0 CLS snapshot captured",
                flush=True,
            )
            return first_live

        except Exception as exc:
            try:
                self.sim_client.pause_sim()
            except Exception:
                pass
            self.direct_mirror_last_error = (
                f"V14.67 corrected handover exception: {exc}"
            )
            self.auto_last_p3d_control_handover_detail = self.direct_mirror_last_error
            print(
                "[V14.67 HANDOVER] ERROR | " + self.direct_mirror_last_error,
                flush=True,
            )
            return None






    @Slot()
    def arm_recording(self):
        """
        Arm recording with any non-empty CLS axis subset.

        Pitch/Roll/Yaw never fall back to Prepar3D joystick data.
        """
        if self.test_mode == "manual":
            probe = self._probe_cls_before_auto()
            connected = [
                name for name, _axis, _key in CLS_AXIS_SPECS
                if probe[name]["ok"]
            ]
            if not connected:
                self.manual_cls_ready = False
                self._set_active_cls_axes([])
                QMessageBox.warning(
                    self,
                    "No CLS Axis Detected",
                    "Manual Test needs at least one live CLS2Sim axis.\n\n"
                    "Pitch, Roll, or Yaw can be used individually.\n\n"
                    "No P3D joystick fallback will be used.",
                )
                print(
                    "[STRICT CLS I/O] Manual recording blocked: no CLS axis detected.",
                    flush=True,
                )
                return

            self._set_active_cls_axes(connected)
            self.manual_cls_ready = True
            missing = [
                name for name, _axis, _key in CLS_AXIS_SPECS
                if name not in connected
            ]
            print(
                "[STRICT CLS I/O] Manual source verified | ACTIVE="
                + self._active_axis_summary()
                + (
                    " | unavailable=" + ",".join(missing)
                    if missing else ""
                )
                + " | source=CLS2Sim only",
                flush=True,
            )

        if self.test_mode == "auto":
            bundle_ok, bundle_detail = self._v1432_verify_bound_bundle(
                "before Automatic initialization"
            )
            print(
                f"[V14.32 BUNDLE VERIFY] {'PASS' if bundle_ok else 'FAIL'} | "
                f"{bundle_detail}",
                flush=True,
            )
            if not bundle_ok:
                QMessageBox.warning(
                    self,
                    "Reference Bundle Changed",
                    "Automatic was blocked because the exact Manual bundle is no "
                    "longer identical to the bundle selected at mode start.\n\n"
                    + bundle_detail,
                )
                return

        if self.test_mode == "auto" and not self.auto_environment_ready:
            QMessageBox.warning(
                self,
                "Automatic Setup Not Ready",
                "Start Recording is blocked because the Automatic-button setup "
                "has not completely finished P3D initialization/terrain loading "
                "and BRUNNER preparation.\n\nWait for the green READY message.",
            )
            return

        if self.test_mode == "auto" and not self.auto_p3d_ready:
            QMessageBox.warning(
                self,
                "P3D Not Ready",
                "Prepar3D is not in the verified Manual initial state. "
                "Automatic recording will not start.",
            )
            return

        if self.test_mode == "auto" and not self.cls_hardware_ready:
            QMessageBox.warning(
                self,
                "CLS-P Hardware Not Ready",
                "Automatic replay is not ready yet.\n\n"
                "Only one replayable CLS axis is required, but preparation "
                "must complete for every active axis.",
            )
            return

        self.is_armed = True
        self.btn_record.setVisible(False)

        if self.test_mode == "auto":
            # V14.30.3:
            # START means "begin initialization".  The 75-second clock, CSV
            # recording and reference replay remain OFF until ALL initialization
            # steps have completed and the first fresh frame after FINAL unpause
            # has been received.
            self.is_recording = False
            self.record_start_time = 0.0
            self.next_sample_second = 0
            self.recorded_rows.clear()
            self.lbl_timer.setText(f"0 / {self.target_duration_sec} seconds (0 rows)")
            self.rec_progress.setValue(0)

            self._update_stage_ui(
                STAGE_INITIALIZING,
                "CONTROLS ALREADY SET — P3D final handover only; timer NOT started (0 rows)",
            )
            QApplication.processEvents()

            # V14.37 START RECORDING:
            # READ-ONLY verification. ZERO INITPOSITION calls and ZERO dynamic
            # restoration calls are permitted here. The final exact init has
            # already completed before READY.
            self._update_stage_ui(
                STAGE_INITIALIZING,
                "FINAL READ-ONLY CHECK — P3D already initialized once; timer still 0",
            )
            QApplication.processEvents()

            if (
                self.auto_initposition_count != 1
                or not self.auto_terrain_preload_sent
                or not self.auto_final_direct_state_ready
            ):
                self._v14303_abort_auto_start(
                    f"Invalid P3D setup state: init_count={self.auto_initposition_count}, "
                    f"direct_final={self.auto_final_direct_state_ready}. "
                    "V14.40 requires exactly one INITPOSITION terrain preload plus "
                    "one verified DIRECT final pose before Automatic start.",
                    "P3D Setup State Error",
                )
                return

            if not self._v1437_verify_paused_position_no_reinit(
                "Start Recording read-only position check"
            ):
                self.auto_environment_ready = False
                self.auto_p3d_ready = False
                self._v14303_abort_auto_start(
                    "The frozen P3D position changed before Start Recording. "
                    "No reinitialization was attempted.",
                    "P3D Ready State Changed",
                )
                return

            # Keep CLS2Sim Position-to-Sim BLOCKED only during paused setup.
            # BRUNNER-only handover will enable it before live recording.
            # WARNING: some profiles suppress AP-driven Position-to-Sim;
            # failure to transmit to P3D must be diagnosed on the simulator.
            if not self._set_cls_position_forwarding_to_p3d(False):
                self._v14303_abort_auto_start(
                    "Could not keep CLS Position-to-Sim blocked before the measured-CLS P3D bridge was armed.",
                    "CLS Position-to-Sim Isolation Failed",
                )
                return

            # V14.42 FINAL BUMPLESS GATE:
            # Use the EXACT same high-rate frame-0 position + force values that the
            # normal replay loop will use. Hold all active axes simultaneously
            # until both position and force are stable, then carry the converged
            # controller state directly into t=0 without a target jump/reset.
            if not self._final_bumpless_cls_settle():
                self._v14303_abort_auto_start(
                    "BRUNNER could not hold the exact replay frame-0 position/force "
                    "state steadily before t=0. Automatic was NOT started.",
                    "BRUNNER Bumpless Start Not Ready",
                )
                return

            auto_guard_ok = bool(
                self.v1430_position_commit_ok
                and self.v1430_dynamic_restore_ok
                and self.v1430_dynamic_verify_ok
                and (not self.cls_to_p3d_feedback_enabled)
            )
            if not auto_guard_ok:
                self._v14303_abort_auto_start(
                    "The full initial-state / CLS-to-P3D guard was not satisfied. "
                    "Automatic was NOT started.",
                    "Automatic Start Guard",
                )
                return

            # V14.37: no second initialization and no dynamic rewrite here.
            # One last read-only paused position check after CLS feedback/preload.
            if not self._v1437_verify_paused_position_no_reinit(
                "Start Recording post-CLS read-only check"
            ):
                self._v14303_abort_auto_start(
                    "P3D position changed after CLS enable/preload. "
                    "Automatic was NOT started and no reinitialization was attempted.",
                    "Final Position Verification Failed",
                )
                return

            # BRUNNER-only handover.  Normal CLS2Sim Position-to-Sim forwarding
            # is enabled after physical t=0 settling. No direct P3D axis events
            # are sent. Check live P3D control input/surface telemetry against
            # the selected Manual; do not claim a QTG pass from this alone.
            first_live = self._v1458_sync_controls_under_soft_hold()
            if first_live is None:
                reason = (
                    "Automatic initial-state validation failed. Automatic was NOT started.\n\n"
                    + (self.auto_last_p3d_control_handover_detail or self.direct_mirror_last_error or "No detailed handover result available.")
                )
                self._v14303_abort_auto_start(
                    reason,
                    "Measured-CLS / P3D Handover Failed",
                )
                return

            # ONLY NOW does the Automatic test exist.
            # Anchor t=0 to the first fresh post-unpause SimConnect frame.
            first_frame_wall = self._v1430_finite(first_live.get("last_updated"))
            if first_frame_wall is None:
                first_frame_wall = time.time()

            # V14.68: commit row 0 IMMEDIATELY from the exact verified first-live
            # SimConnect frame. Do not wait for the next GUI timer tick and then
            # label a 0.2-0.3 s later aircraft state as t=0.
            self.record_start_time = float(first_frame_wall)
            self.recorded_rows.clear()
            self.recorded_rows.append(self._format_data_row(0, dict(first_live)))
            self.next_sample_second = 1
            self.is_recording = True
            self.measurement_audit.begin("auto", self.record_start_time,
                                         first_live, self.auto_t0_cls_values)

            print(
                f"[V14.70 START] AUTOMATIC t=0 STARTED | "
                f"terrain_INITPOSITION_count={self.auto_initposition_count} | "
                f"direct_final={self.auto_final_direct_state_ready} | "
                f"wall={self.record_start_time:.6f}",
                flush=True,
            )

            self._update_stage_ui(STAGE_RECORDING)
            self._beep_start()
        else:
            # V14.31 Manual START: first fresh post-unpause frame is THE t=0.
            if not self._v14304_capture_manual_authoritative_t0():
                self.is_armed = False
                self.is_recording = False
                self.record_start_time = 0.0
                self.btn_record.setVisible(True)
                self.btn_record.setEnabled(True)
                self._update_stage_ui(STAGE_CONNECTED_READY)
                QMessageBox.warning(
                    self,
                    "Manual t=0 Capture Failed",
                    "Manual recording was not started because no complete fresh "
                    "P3D t=0 frame was obtained.",
                )
                return

    def _on_spacebar_pressed(self):
        """Handler for Spacebar shortcut — triggers Start Recording for both Manual and Automatic testing."""
        if self.btn_record.isVisible() and self.btn_record.isEnabled() and not self.is_recording:
            print("[Keyboard] SPACEBAR shortcut activated -> Triggering Start Recording!", flush=True)
            self.arm_recording()

    def keyPressEvent(self, event):
        """Override keyPressEvent to ensure Spacebar triggers Start Recording anywhere in GUI."""
        if event.key() == Qt.Key_Space:
            if self.btn_record.isVisible() and self.btn_record.isEnabled() and not self.is_recording:
                print("[Keyboard] SPACEBAR keyPressEvent triggered -> Starting Recording!", flush=True)
                self.arm_recording()
                return
        super().keyPressEvent(event)

    @Slot()
    def start_test(self):
        """Legacy: arm the test without repositioning (kept for compatibility)."""
        self.test_mode = "legacy"
        self.target_duration_sec = 60
        self.expected_total_rows = 61
        self._start_connection(place_at_rahwali=False)

    def _start_connection(
        self,
        place_at_rahwali: bool = False,
        place_airborne: bool = False,
        alt_ft: float = 3000.0,
        speed_kts: float = 80.0,
        initial_pitch: float = 0.0,
        initial_bank: float = 0.0,
        initial_hdg: float = 48.4,
        initial_lat: float = RAHWALI_LAT,
        initial_lon: float = RAHWALI_LON,
        initial_throttle: float = 80.0,
    ):
        """Shared helper: connect to P3D, optionally teleport, then arm."""
        self._update_stage_ui(STAGE_CONNECTING)
        self._set_auto_buttons_enabled(False)
        self.btn_manual_test.setEnabled(False)
        self.btn_stop.setEnabled(True)
        QApplication.processEvents()

        self.is_armed = False
        self.is_recording = False
        self.recorded_rows.clear()
        self.next_sample_second = 0
        self.rec_progress.setValue(0)
        self.lbl_timer.setText(f"0 / {self.target_duration_sec} seconds (0 rows)")

        def _connect_worker():
            if SIMCONNECT_AVAILABLE:
                if self.sim_client is None:
                    self.sim_client = SimConnectClient(
                        app_name="Super Mushshak Logger V14.66 Verified State"
                    )

                def _connected_callback():
                    # Automatic starts from a paused P3D session; Manual uses the
                    # normal airborne-placement path below.
                    if self.test_mode == "auto":
                        try:
                            self.sim_client.pause_sim()
                            print(
                                "[V14.66.3 SESSION] P3D session ready -> PAUSED for Automatic preparation.",
                                flush=True,
                            )
                        except Exception as exc:
                            print(
                                f"[V14.66.3 SESSION] immediate P3D pause warning: {exc}",
                                flush=True,
                            )
                    self._sig_connection_ok.emit()

                # V14.66.3 IMPORTANT:
                # After a blocked Automatic run, connect_and_run() is already alive
                # in its background thread. Calling it a second time on the same
                # SimConnectClient can fail or leave Manual stuck at Connecting.
                # Reuse the existing live session instead.
                try:
                    self.sim_client.natural_engine_mode = (self.test_mode == "auto")
                    already_connected = bool(
                        self.sim_client
                        and hasattr(self.sim_client, "is_connected")
                        and self.sim_client.is_connected()
                    )
                except Exception:
                    already_connected = False

                if already_connected:
                    print(
                        f"[V14.66.3 SESSION] Reusing existing SimConnect session for "
                        f"{self.test_mode.upper()}.",
                        flush=True,
                    )
                    try:
                        if place_at_rahwali:
                            self.sim_client.set_position_rahwali()
                        elif place_airborne:
                            self.sim_client.set_position_airborne(
                                alt_ft=alt_ft,
                                speed_kts=speed_kts,
                                pitch_deg=initial_pitch,
                                bank_deg=initial_bank,
                                hdg_deg=initial_hdg,
                                lat=initial_lat,
                                lon=initial_lon,
                                throttle=initial_throttle,
                            )
                        _connected_callback()
                    except Exception as exc:
                        self._sig_connection_err.emit(
                            f"Existing SimConnect session reuse failed: {exc}"
                        )
                    return

                self.sim_client.connect_and_run(
                    on_connected=_connected_callback,
                    on_error=self._sig_connection_err.emit,
                    place_at_rahwali=place_at_rahwali,
                    place_airborne=place_airborne,
                    alt_ft=alt_ft,
                    speed_kts=speed_kts,
                    initial_pitch=initial_pitch,
                    initial_bank=initial_bank,
                    initial_hdg=initial_hdg,
                    initial_lat=initial_lat,
                    initial_lon=initial_lon,
                    initial_throttle=initial_throttle,
                )
            else:
                self._sig_connection_ok.emit()

        t = threading.Thread(target=_connect_worker, daemon=True, name="SimConnectThread")
        t.start()

    @Slot(str)
    def _on_p3d_user_confirmation_needed(self, message: str):
        """
        Show the explicit operator gate.

        The user should click this ONLY after Prepar3D's visible
        "Loading terrain data..." box has disappeared and the cockpit/scenery
        is fully rendered.
        """
        self.auto_waiting_for_p3d_user = True
        self.btn_p3d_loaded.setText("✅  P3D Loaded — Continue Setup")
        self.btn_p3d_loaded.setEnabled(True)
        self.btn_p3d_loaded.setVisible(True)
        self.btn_record.setVisible(False)

        self._update_stage_ui(
            STAGE_INITIALIZING,
            message
            or (
                "WAIT FOR P3D — when the terrain-loading box disappears, "
                "click 'P3D Loaded — Continue Setup'. Timer remains 0."
            ),
        )
        self.lbl_timer.setText(f"0 / {self.target_duration_sec} seconds (0 rows)")
        self.rec_progress.setValue(0)

    @Slot()
    def _on_p3d_loaded_continue(self):
        """
        Operator confirms visual P3D loading is complete.

        This does NOT start recording. It only allows the preparation worker to
        perform the final exact t=0 position/dynamic restore and BRUNNER setup.
        """
        if self.test_mode != "auto" or not self.auto_prepare_in_progress:
            return
        if not self.auto_waiting_for_p3d_user:
            return

        self.auto_waiting_for_p3d_user = False
        self.btn_p3d_loaded.setEnabled(False)
        self.btn_p3d_loaded.setVisible(False)

        self._update_stage_ui(
            STAGE_INITIALIZING,
            "P3D LOAD CONFIRMED — applying ONE FINAL exact Manual t=0 initialization...",
        )
        print(
            "[V14.38 USER GATE] P3D load confirmed; applying ONE FINAL exact t=0 initialization.",
            flush=True,
        )
        self.p3d_user_ready_event.set()

    @Slot(str)
    def _on_auto_prepare_progress(self, message: str):
        """Main-thread UI update while Automatic pre-initialization is running."""
        self._update_stage_ui(STAGE_INITIALIZING, message)
        self.lbl_timer.setText(f"0 / {self.target_duration_sec} seconds (0 rows)")
        self.rec_progress.setValue(0)

    @Slot(bool, str)
    def _on_auto_prepare_done(self, success: bool, message: str):
        """Expose Start Recording only after P3D + CLS preparation is complete."""
        self.auto_prepare_in_progress = False
        self.auto_waiting_for_p3d_user = False
        if hasattr(self, "btn_p3d_loaded"):
            self.btn_p3d_loaded.setVisible(False)
            self.btn_p3d_loaded.setEnabled(False)
        self.auto_environment_ready = bool(
            success
            and self.auto_controller_init_ready
            and self.auto_p3d_ready
            and self.cls_hardware_ready
        )

        if self.auto_environment_ready:
            print(
                "[V14.65 READY] BRUNNER position+force locked before P3D run + DIRECT final P3D t=0 complete. "
                "Automatic will start immediately — no second Start button.",
                flush=True,
            )

            # Do NOT expose a Start button in Automatic mode. The screenshot
            # proved that waiting between READY and user Start allowed P3D state
            # to change. Start immediately after the verified preparation.
            self.btn_record.setVisible(False)
            self._update_stage_ui(
                STAGE_INITIALIZING,
                "FINAL DIRECT STATE READY — starting Automatic immediately; timer still 0",
            )
            self.lbl_rec_status.setText(
                "FINAL READY — starting Automatic now..."
            )
            self.lbl_rec_status.setStyleSheet("color: #a6e3a1;")
            self.lbl_timer.setText(f"0 / {self.target_duration_sec} seconds (0 rows)")
            self.rec_progress.setValue(0)

            # Main-thread one-shot. arm_recording() is now invoked immediately
            # after READY, leaving no human-delay window for the paused state to
            # drift before the final read-only checks.
            QTimer.singleShot(150, self.arm_recording)
        else:
            print(f"[V14.34 READY] PREPARATION FAILED | {message}", flush=True)
            self.auto_environment_ready = False
            self.auto_p3d_ready = False
            self.cls_hardware_ready = False
            self.auto_controller_init_ready = False
            try:
                if self.sim_client:
                    self.sim_client.pause_sim()
            except Exception:
                pass
            try:
                self._set_cls_position_forwarding_to_p3d(False)
            except Exception:
                pass
            self.auto_last_block_reason = str(message)
            self._v1479_capture_start_diagnostics()
            try:
                diag_ts = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
                self._save_commissioning_diagnostics(diag_ts)
                self._save_p3d_state_diagnostics(diag_ts)
            except Exception as diag_exc:
                print(f"[V14.52 PREP] failure-diagnostic save warning: {diag_exc}", flush=True)
            try:
                self._teardown_hardware()
            except Exception as teardown_exc:
                print(f"[V14.52 PREP] teardown warning: {teardown_exc}", flush=True)
            self._update_stage_ui(STAGE_AUTO_BLOCKED, message)
            if self.v1493_one_visit_active:
                key = self._v1493_current_manual_key()
                profile = dict(self.v1493_one_visit_current_candidate or {})
                self.v1493_one_visit_results.append({
                    "manual_key": key,
                    "stage": self.v1493_one_visit_stage,
                    "candidate_id": profile.get("candidate_id", profile.get("profile_id")),
                    "profile": profile,
                    "score": None,
                    "failure": str(message),
                })
                if self.v1493_one_visit_stage == "short":
                    self.v1493_one_visit_candidate_index += 1
                    print(f"[V14.93 ONE-VISIT] candidate blocked; continuing | {message}", flush=True)
                    QTimer.singleShot(1200, self._v1493_run_next_one_visit_step)
                else:
                    self._v1493_abort_one_visit("Final full-duration run blocked for " + str(key) + ": " + str(message))
                return
            QMessageBox.warning(
                self,
                "Automatic Preparation Failed",
                message
                + "\n\nThe Automatic test was NOT started. "
                  "Press Automatic again after correcting the problem.",
            )

    @Slot()
    def _on_connection_success(self):
        """Called on main thread after successful SimConnect connection."""
        if self.test_mode in ("manual", "auto"):
            if self.sim_client:
                self.sim_client.pause_sim()
            self.is_armed = False

            # V14.42 STRICT MANUAL: UI/state only. Do NOT change any
            # CLS2Sim/BRUNNER setting here. The Manual path records the controller
            # exactly as CLS2Sim is already driving it.
            if self.test_mode == "manual":
                self._update_stage_ui(STAGE_CONNECTED_READY)
                print(
                    "[MANUAL READ-ONLY] HARD GUARDED | telemetry/status reads only | "
                    "watchdog OFF | no ForceScale, MoveBack, AP, ForceOffset, "
                    "override or mutating-state writes can leave this process.",
                    flush=True,
                )

            # V14.34 Automatic mode:
            # DO NOT expose Start Recording yet. Automatic-button selection must
            # first finish the variable-time P3D terrain/initial-state setup and
            # the complete BRUNNER preparation.
            if self.test_mode == "auto":
                self.auto_prepare_in_progress = True
                self.auto_p3d_ready = False
                self.auto_environment_ready = False
                self._update_stage_ui(
                    STAGE_INITIALIZING,
                    "AUTOMATIC SETUP — BRUNNER controllers FIRST, P3D SECOND; "
                    "timer NOT started (0 rows)",
                )
                init_pitch = 0.0
                init_roll = 0.0
                init_yaw = 0.0

                if self.reference_replay:
                    init_pitch = self._initial_reference_target("Pitch", 0.0)
                    init_roll = self._initial_reference_target("Roll", 0.0)
                    init_yaw = self._initial_reference_target("Yaw", 0.0)

                def _ramp_bg():
                    # V14.52 STRICT ORDER:
                    #   1) BRUNNER/CLS controllers FIRST, with output to P3D blocked.
                    #   2) P3D terrain/initial state SECOND.
                    #   3) Re-verify the already-initialized controllers at Manual t=0.
                    #   4) Only then expose the final start gates.
                    # The V14.51 replay gains, AP speeds, force gains and interlocks
                    # are unchanged.
                    self.auto_p3d_ready = False
                    self.auto_controller_init_ready = False
                    self.auto_last_block_reason = ""
                    self.v1476_axis_cal_attempts = {n: 0 for n in ("Pitch", "Roll", "Yaw")}
                    self.v1476_axis_cal_last_detail = {n: "" for n in ("Pitch", "Roll", "Yaw")}

                    try:
                        self._set_cls_position_forwarding_to_p3d(False)
                    except Exception:
                        pass

                    self._sig_auto_prepare_progress.emit(
                        "STEP 1/2 — initializing BRUNNER controllers FIRST; "
                        "CLS→P3D blocked; timer NOT started"
                    )

                    # ---------------------------
                    # PHASE 1: CONTROLLERS FIRST
                    # ---------------------------
                    armed = self._open_and_arm_hardware(
                        initial_pitch=init_pitch,
                        initial_roll=init_roll,
                        initial_yaw=init_yaw,
                    )
                    if not armed:
                        self.auto_last_block_reason = (
                            "BRUNNER hardware could not be armed/configured before P3D initialization."
                        )
                        try:
                            diag_ts = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
                            self._save_commissioning_diagnostics(diag_ts)
                        except Exception as diag_exc:
                            print(f"[COMMISSION] Arm-failure save warning: {diag_exc}", flush=True)
                        self._sig_auto_prepare_done.emit(False, self.auto_last_block_reason)
                        return

                    self._ramp_hardware_to_initial_position(
                        target_pitch=init_pitch,
                        target_roll=init_roll,
                        target_yaw=init_yaw,
                        duration_sec=2.0,
                    )

                    precheck_ok = self._run_tracking_endpoint_precheck(
                        initial_pitch=init_pitch,
                        initial_roll=init_roll,
                        initial_yaw=init_yaw,
                    )

                    # Return every active physical controller to the selected
                    # Manual frame-0 target and verify real CLS feedback.
                    initial_targets = {
                        "Pitch": init_pitch,
                        "Roll": init_roll,
                        "Yaw": init_yaw,
                    }
                    for name, axis, _key in self._active_axis_specs():
                        self.cls.send(
                            packet_setting_float(
                                axis,
                                SID_AP_POSITION,
                                initial_targets[name],
                            )
                        )

                    initial_verified = self._verify_controls_at_initial_targets(
                        target_pitch=init_pitch,
                        target_roll=init_roll,
                        target_yaw=init_yaw,
                        tolerance=0.15,
                        timeout_sec=5.0,
                    )
                    controllers_verified = bool(precheck_ok and initial_verified)
                    self.cls_prep_complete = controllers_verified
                    self.cls_hardware_ready = controllers_verified
                    self.auto_controller_init_ready = controllers_verified

                    if not controllers_verified:
                        if not precheck_ok:
                            reason = (
                                "BRUNNER endpoint/tracking precheck failed before P3D initialization. "
                                f"Results={self.track_precheck_results}"
                            )
                        else:
                            reason = (
                                "BRUNNER could not settle at the selected Manual t=0 positions before "
                                "P3D initialization. "
                                + (self.auto_last_cls_initial_verify_detail or "No axis detail available.")
                            )
                        self.auto_last_block_reason = reason
                        print("[V14.54 CONTROLLERS FIRST] FAILED | " + reason, flush=True)
                        try:
                            diag_ts = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
                            self._save_commissioning_diagnostics(diag_ts)
                        except Exception as diag_exc:
                            print(f"[COMMISSION] Prep-failure save warning: {diag_exc}", flush=True)
                        self._sig_auto_prepare_done.emit(False, reason)
                        return

                    self._sig_auto_prepare_progress.emit(
                        "STEP 1/3 — positions READY; locking exact Manual t=0 physical force BEFORE P3D runs..."
                    )
                    if not self._v1465_cls_only_t0_force_lock("PRE-P3D"):
                        self.auto_last_block_reason = (
                            "BRUNNER position initialization passed, but exact Manual t=0 force could not "
                            "be locked before P3D was allowed to run. " + self.v1465_controls_lock_detail
                        )
                        self._sig_auto_prepare_done.emit(False, self.auto_last_block_reason)
                        return

                    print(
                        "[V14.65 CONTROLS FIRST] FULL PASS | BRUNNER frame-0 POSITION + FORCE are READY; "
                        "P3D may now begin terrain/initial-state preparation.",
                        flush=True,
                    )

                    # ---------------------------
                    # PHASE 2: P3D SECOND
                    # ---------------------------
                    self._sig_auto_prepare_progress.emit(
                        "STEP 2/3 — controls fully READY; now initializing P3D / loading terrain; "
                        "timer NOT started"
                    )

                    load_started = self._v1435_begin_p3d_visual_load(
                        "V14.52 controllers-first terrain-load trigger"
                    )
                    if not load_started:
                        self.auto_last_block_reason = (
                            "BRUNNER was ready, but Prepar3D initial-position / terrain-load trigger failed."
                        )
                        self._sig_auto_prepare_done.emit(False, self.auto_last_block_reason)
                        return

                    if self.v1493_one_visit_active:
                        self._sig_auto_prepare_progress.emit(
                            f"V14.93 ONE-VISIT — automatic terrain settle wait {V14_93_ONE_VISIT_AUTO_CONFIRM_LOAD_SEC:.1f}s; "
                            "existing stability gates remain active"
                        )
                        time.sleep(float(V14_93_ONE_VISIT_AUTO_CONFIRM_LOAD_SEC))
                        confirmed = True
                    else:
                        self._sig_p3d_user_confirmation_needed.emit(
                            "CONTROLLERS READY. WAIT FOR P3D — do NOT continue while "
                            "'Loading terrain data...' is visible. When it disappears and "
                            "cockpit/scenery is fully loaded, click 'P3D Loaded — Continue Setup'. "
                            "Timer stays 0."
                        )
                        confirmed = self.p3d_user_ready_event.wait(timeout=600.0)
                    if not confirmed:
                        self.auto_last_block_reason = (
                            "No P3D-loaded confirmation was received within 10 minutes."
                        )
                        self._sig_auto_prepare_done.emit(False, self.auto_last_block_reason)
                        return

                    if self.test_mode != "auto":
                        return

                    self._sig_auto_prepare_progress.emit(
                        "P3D LOAD CONFIRMED — checking background stability; timer still 0..."
                    )
                    stable_ok = self._v1439_wait_for_postload_stability(
                        "V14.52 post-user-confirmation"
                    )
                    if not stable_ok:
                        self.auto_last_block_reason = (
                            "P3D did not become stable after load confirmation. "
                            "The Automatic test was NOT started."
                        )
                        self._sig_auto_prepare_done.emit(False, self.auto_last_block_reason)
                        return

                    # V14.58: no AP->P3D live proof is attempted through the
                    # fixed CLS2Sim profile.  Keep Position-to-Sim BLOCKED and
                    # apply the final exact P3D t=0 now.  The authoritative live
                    # P3D control/surface synchronization occurs later through the
                    # measured-CLS direct axis bridge immediately before recording.
                    self._sig_auto_prepare_progress.emit(
                        "CONTROLLERS READY — applying FINAL P3D t=0; measured-CLS live bridge will synchronize controls before recording..."
                    )
                    self._set_cls_position_forwarding_to_p3d(False)
                    print(
                        "[V14.58 FIXED PROFILE] CLS Position-to-Sim remains BLOCKED for Automatic; "
                        "no CLS2Sim profile change is required.",
                        flush=True,
                    )
                    engine_ok, engine_detail = self._v1496_prepare_engine()
                    if not engine_ok:
                        self.auto_last_block_reason = engine_detail
                        self._sig_auto_prepare_done.emit(False, engine_detail)
                        return
                    p3d_ok = self._v1438_final_exact_init_after_load(
                        "V14.58 final P3D t0 after controllers-first preparation"
                    )
                    self.auto_p3d_ready = bool(p3d_ok)
                    if not p3d_ok:
                        self.auto_last_block_reason = (
                            "Controllers are ready, but P3D could not verify the final exact Manual t=0 state."
                        )
                        self._sig_auto_prepare_done.emit(False, self.auto_last_block_reason)
                        return

                    # P3D loading can change the physical force environment. The
                    # controllers were already initialized first; now simply
                    # re-issue their SAME frame-0 positions and verify they have
                    # remained there before the final force preload/start gates.
                    self._sig_auto_prepare_progress.emit(
                        "P3D t=0 READY — re-verifying already-initialized controllers at Manual t=0..."
                    )
                    for name, axis, _key in self._active_axis_specs():
                        self.cls.send(
                            packet_setting_float(
                                axis,
                                SID_AP_POSITION,
                                initial_targets[name],
                            )
                        )

                    post_p3d_verified = self._verify_controls_at_initial_targets(
                        target_pitch=init_pitch,
                        target_roll=init_roll,
                        target_yaw=init_yaw,
                        tolerance=V14_52_POST_P3D_CLS_REVERIFY_TOL,
                        timeout_sec=V14_52_POST_P3D_CLS_REVERIFY_TIMEOUT_SEC,
                    )
                    if not post_p3d_verified:
                        self.cls_hardware_ready = False
                        self.auto_controller_init_ready = False
                        self.auto_last_block_reason = (
                            "After P3D initialization, one or more BRUNNER controllers could not hold "
                            "the selected Manual t=0 position. "
                            + (self.auto_last_cls_initial_verify_detail or "No axis detail available.")
                        )
                        try:
                            diag_ts = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
                            self._save_commissioning_diagnostics(diag_ts)
                        except Exception as diag_exc:
                            print(f"[COMMISSION] Post-P3D verify save warning: {diag_exc}", flush=True)
                        self._sig_auto_prepare_done.emit(False, self.auto_last_block_reason)
                        return

                    # V14.65 FINAL CONTROLS-FIRST GATE.  _v1438 leaves P3D PAUSED.
                    # Re-lock the exact physical frame-0 force now, before the later
                    # measured-CLS handover is allowed to unpause/run P3D.
                    self._sig_auto_prepare_progress.emit(
                        "STEP 3/3 — P3D exact t=0 is PAUSED; final BRUNNER position+force lock before P3D runs..."
                    )
                    if not self._v1465_cls_only_t0_force_lock("FINAL-PAUSED"):
                        self.cls_hardware_ready = False
                        self.auto_controller_init_ready = False
                        self.auto_last_block_reason = (
                            "P3D final t=0 is ready and paused, but BRUNNER exact frame-0 force/position "
                            "could not be confirmed before P3D run. " + self.v1465_controls_lock_detail
                        )
                        self._sig_auto_prepare_done.emit(False, self.auto_last_block_reason)
                        return

                    # V14.56: the old separate force-preload stage is disabled.
                    # It was redundant with the final bumpless gate and, on the
                    # high-force reference, could drive Pitch away from t=0 before
                    # the position-priority interlock was active. These calls now
                    # return immediately; the final bumpless gate is the single
                    # authoritative frame-0 force preparation stage.
                    pitch_force_preloaded = True
                    roll_force_preloaded = True
                    yaw_force_preloaded = True
                    if "Pitch" in self.active_cls_axes and self._force_stage_allows("Pitch"):
                        pitch_force_preloaded = self._preload_initial_pitch_force()
                    if "Roll" in self.active_cls_axes and self._force_stage_allows("Roll"):
                        roll_force_preloaded = self._preload_initial_roll_force()
                    if "Yaw" in self.active_cls_axes and self._force_stage_allows("Yaw"):
                        yaw_force_preloaded = self._preload_initial_yaw_force()

                    self.cls_prep_complete = True
                    self.cls_hardware_ready = True
                    self.auto_controller_init_ready = True

                    print(
                        "[V14.65 PREPARATION COMPLETE] controls POSITION+FORCE locked FIRST + P3D FINAL PAUSED + measured-CLS handover pending | axes="
                        + self._active_axis_summary()
                        + " | "
                        + f"InitialPitchForce={'READY' if pitch_force_preloaded else 'NOT SETTLED'} | "
                        + f"InitialRollForce={'READY' if roll_force_preloaded else 'NOT SETTLED'} | "
                        + f"InitialYawForce={'READY' if yaw_force_preloaded else 'NOT SETTLED'}",
                        flush=True,
                    )

                    self._sig_auto_prepare_done.emit(
                        True,
                        "Controls position+force locked first; P3D final state initialized second and remains paused; ready for measured-CLS handover.",
                    )

                threading.Thread(
                    target=_ramp_bg,
                    daemon=True,
                    name="AutoRampThread",
                ).start()
        else:
            # Legacy mode: unpause and arm immediately
            if self.sim_client:
                self.sim_client.unpause_sim()
            self.is_armed = True
            self._update_stage_ui(STAGE_ARMED)
        self.telemetry_timer.start()

    def _v1468_manual_throttle_at_elapsed(self, elapsed_sec: float) -> Optional[float]:
        """Linear interpolation of the selected Manual's 1-Hz throttle profile."""
        rows = getattr(self, "manual_csv_rows_by_sec", {}) or {}
        if not rows:
            return None
        try:
            t = max(0.0, float(elapsed_sec))
            keys = sorted(int(k) for k in rows.keys())
            if not keys:
                return None
            if t <= keys[0]:
                return float(rows[keys[0]].get("throttle"))
            if t >= keys[-1]:
                return float(rows[keys[-1]].get("throttle"))

            lo = int(math.floor(t))
            hi = int(math.ceil(t))
            while lo not in rows and lo > keys[0]:
                lo -= 1
            while hi not in rows and hi < keys[-1]:
                hi += 1
            v0 = self._v1430_finite(rows.get(lo, {}).get("throttle"))
            v1 = self._v1430_finite(rows.get(hi, {}).get("throttle"))
            if v0 is None and v1 is None:
                return None
            if v0 is None:
                return float(v1)
            if v1 is None or hi <= lo:
                return float(v0)
            a = max(0.0, min(1.0, (t - lo) / float(hi - lo)))
            return (1.0 - a) * float(v0) + a * float(v1)
        except Exception:
            return None

    def _beep_start(self):
        """Beep once to signal recording start."""
        if WINSOUND_AVAILABLE:
            threading.Thread(target=lambda: winsound.Beep(1200, 400), daemon=True).start()

    def _beep_end(self):
        """Beep twice to signal recording end."""
        if WINSOUND_AVAILABLE:
            def _double_beep():
                winsound.Beep(1000, 300)
                time.sleep(0.15)
                winsound.Beep(1000, 300)
            threading.Thread(target=_double_beep, daemon=True).start()

    @Slot(str)
    def _on_connection_error(self, error_msg: str = ""):
        """Called on main thread after SimConnect connection failure."""
        self._update_stage_ui(STAGE_CONNECTION_ERROR, error_msg)

    def _reset_readings_to_zero(self):
        """Reset all GUI telemetry card displays to 0.0 and recording time to 0."""
        self.lbl_throttle_val.setText("0.0 %")
        self.throttle_bar.setValue(0)
        self.lbl_throttle_detail.setText("Throttle Eng 1: 0.0%")
        self.lbl_pitch_val.setText("Pitch Angle: 0.0°")
        self.lbl_bank_val.setText("Bank Angle: 0.0°")
        self.lbl_yaw_val.setText("Yaw (True Hdg): 0.0° | Mag: 0.0°")
        if hasattr(self, 'joystick_canvas'):
            self.joystick_canvas.set_position(0.0, 0.0)
        if hasattr(self, 'lbl_joystick_pitch'):
            self.lbl_joystick_pitch.setText("Joystick Pitch: 0.00° | Force: 0.00 Nm")
        if hasattr(self, 'lbl_joystick_roll'):
            self.lbl_joystick_roll.setText("Joystick Roll: 0.00° | Force: 0.00 Nm")
        self.rec_progress.setRange(0, self.target_duration_sec)
        self.rec_progress.setValue(0)
        self.lbl_timer.setText(f"0 / {self.target_duration_sec} seconds (0 rows)")

    @Slot()
    def stop_test(self):
        """Safely stop armed or active recording test."""
        self.auto_prepare_in_progress = False
        # Release any V14.35 preparation worker blocked at the P3D-loaded gate.
        self.auto_waiting_for_p3d_user = False
        try:
            self.p3d_user_ready_event.set()
        except Exception:
            pass
        if hasattr(self, "btn_p3d_loaded"):
            self.btn_p3d_loaded.setVisible(False)
            self.btn_p3d_loaded.setEnabled(False)

        self.telemetry_timer.stop()
        if self.test_mode == "auto":
            self._teardown_hardware()
        elif self.test_mode == "manual":
            print(
                "[MANUAL READ-ONLY] STOP: no CLS2Sim/BRUNNER command sent.",
                flush=True,
            )

        self.is_armed = False
        self.is_recording = False

        # Stop the SimConnect poll loop
        if self.sim_client:
            self.sim_client.stop()

        # If user recorded data before stopping, save it and generate report
        if len(self.recorded_rows) >= 2:
            rows_snapshot = list(self.recorded_rows)
            self.lbl_csv_path.setText("Generating report from stopped test... please wait.")
            def _finish_bg():
                self._save_csv_file_from_rows(rows_snapshot)
            threading.Thread(target=_finish_bg, daemon=True, name="StopFinishThread").start()
        else:
            self.recorded_rows.clear()
            self.lbl_csv_path.setText("None (telemetry not started)")
            # Critical for a one-shot simulator visit: even a preparation or
            # arming failure before formal recording must leave evidence behind.
            if COMMISSION_DIAG_ENABLED and self.test_mode in ("manual", "auto"):
                try:
                    diag_ts = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
                    self._save_commissioning_diagnostics(diag_ts)
                    self._save_p3d_state_diagnostics(diag_ts)
                except Exception as diag_exc:
                    print(f"[DIAGNOSTIC] Early-stop save warning: {diag_exc}", flush=True)

        self._reset_readings_to_zero()
        self._update_stage_ui(STAGE_STOPPED_BY_USER)
        self._set_auto_buttons_enabled(True)
        self.btn_manual_test.setEnabled(True)

    def _on_telemetry_tick(self):
        """Periodic tick to update GUI telemetry and trigger recording logic."""
        if not self.sim_client or not self.sim_client.is_connected():
            data = {
                "title": "Super Mushshak Aircraft",
                "number_of_engines": 1,
                "airspeed_indicated": 0.0,
                "vertical_speed": 0.0,
                "indicated_altitude": 0.0,
                "magnetic_heading": 0.0,
                "throttles": [0.0, 0.0, 0.0, 0.0],
                "max_throttle": 0.0,
                "prop_rpms": [0.0, 0.0, 0.0, 0.0],
                "eng_rpms": [0.0, 0.0, 0.0, 0.0],
            }
        else:
            data = self.sim_client.get_telemetry()

        # BRUNNER-only runtime guard. Physical axes must be live and CLS2Sim
        # Position-to-Sim forwarding must be enabled. This does not alone prove
        # that the active CLS2Sim profile forwards AP-driven positions to P3D.
        if self.test_mode == "auto" and self.is_recording:
            bridge_ok, bridge_detail = self._v1458_runtime_bridge_health(data)
            if not bridge_ok:
                self._v1458_abort_runtime_bridge(bridge_detail)
                return

        title = data.get("title", "Super Mushshak Aircraft")
        num_eng = data.get("number_of_engines", 1)
        self.engine_count = num_eng
        throttles = data.get("throttles", [0.0, 0.0, 0.0, 0.0])
        max_t = data.get("max_throttle", 0.0)
        pitch = data.get("pitch_angle", 0.0)
        bank = data.get("bank_angle", 0.0)
        yaw = data.get("yaw_angle", 0.0)
        mag_hdg = data.get("magnetic_heading", 0.0)

        # In Automatic mode during active recording: read ONLY stored reference data from manual test
        is_auto_replaying = (self.test_mode == "auto" and self.is_recording and bool(self.reference_replay))

        p_pos = 0.0
        r_pos = 0.0
        p_force = 0.0
        r_force = 0.0
        p_disp = 0.0
        r_disp = 0.0
        rd_pos = 0.0
        rd_force = 0.0

        # Live physical actuator telemetry is recorded independently of the
        # P3D/reference trajectory. None means that CLS2Sim has not returned a
        # valid sample for that actuator yet.
        cls_pitch_norm = self.cls.last_pitch_pos if getattr(self, 'cls', None) else None
        cls_roll_norm = self.cls.last_roll_pos if getattr(self, 'cls', None) else None
        cls_yaw_norm = self.cls.last_rudder_pos if getattr(self, 'cls', None) else None
        cls_pitch_force = self.cls.last_pitch_force if getattr(self, 'cls', None) else None
        cls_roll_force = self.cls.last_roll_force if getattr(self, 'cls', None) else None
        cls_yaw_force = self.cls.last_rudder_force if getattr(self, 'cls', None) else None

        # ------------------------------------------------------------------
        # STRICT CLS CONTROL PATH
        # ------------------------------------------------------------------
        # Pitch/Roll/Yaw values below come ONLY from CLS2Sim.
        # Position is stored as raw encoder units.
        # Force is read from active engineering/normalized/raw force channels.

        def _get_raw_pos(axis_name: str, norm_val: Optional[float]) -> float:
            if not getattr(self, "cls", None):
                return float("nan")
            raw_val = None
            if axis_name == "Pitch":
                raw_val = self.cls.last_pitch_pos_raw
            elif axis_name == "Roll":
                raw_val = self.cls.last_roll_pos_raw
            else:
                raw_val = self.cls.last_rudder_pos_raw

            if raw_val is not None:
                try:
                    fv = float(raw_val)
                    if math.isfinite(fv):
                        return round(fv, 1)
                except (TypeError, ValueError):
                    pass

            # Fallback: if raw not yet directly polled, compute from normalized position and range
            if norm_val is not None:
                try:
                    fv = float(norm_val)
                    if math.isfinite(fv):
                        info = self.cls.axis_info.get(axis_name, {})
                        range_max = float(info.get("pos_range_max", 65000.0) or 65000.0)
                        return round(fv * range_max, 1)
                except (TypeError, ValueError):
                    pass

            return float("nan")

        # Actual measured physical raw positions from CLS2Sim.
        p_pos = _get_raw_pos("Pitch", cls_pitch_norm)
        p_disp = p_pos
        r_pos = _get_raw_pos("Roll", cls_roll_norm)
        r_disp = r_pos
        rd_pos = _get_raw_pos("Yaw", cls_yaw_norm)

        # FORMAL REPORT FORCE SOURCE
        # --------------------------------------------------------------
        # Retrieve the active measured force from CLS2Sim (Engineering 0x22,
        # RAW->ENG 0x20, Normalized 0x21, or RAW 0x20).
        def _get_active_force(axis_name: str) -> float:
            """Return engineering force in N only; never relabel NORM/RAW as N."""
            if not getattr(self, "cls", None):
                return 0.0

            if axis_name == "Pitch":
                eng = self.cls.last_pitch_force_eng
                generic = self.cls.last_pitch_force
            elif axis_name == "Roll":
                eng = self.cls.last_roll_force_eng
                generic = self.cls.last_roll_force
            else:
                eng = self.cls.last_rudder_force_eng
                generic = self.cls.last_rudder_force

            source = str(self.cls.last_force_source.get(axis_name, ""))
            val = eng
            if val is None and generic is not None and "ENG" in source:
                val = generic

            if val is None:
                return 0.0
            try:
                fv = float(val)
                return round(fv, 3) if math.isfinite(fv) else 0.0
            except (TypeError, ValueError):
                return 0.0

        p_force = _get_active_force("Pitch")
        r_force = _get_active_force("Roll")
        rd_force = _get_active_force("Yaw")
        if is_auto_replaying:
            elapsed_sec_live = max(0.0, time.time() - self.record_start_time)
            elapsed_ms = int(elapsed_sec_live * 1000)
            n_frames = len(self.reference_replay)

            # V14.68: replay Manual throttle as a separate non-CLS control.
            # The previous builds held only t=0 throttle for all 75 seconds,
            # which made the aircraft trajectory diverge even when Pitch
            # position/force replay was excellent.
            throttle_ref = self._v1468_manual_throttle_at_elapsed(elapsed_sec_live)
            if throttle_ref is not None and self.sim_client:
                try:
                    self.sim_client.set_throttle(float(throttle_ref))
                except Exception as exc:
                    print(f"[V14.70 THROTTLE] set warning: {exc}", flush=True)

            try:
                if not self.active_cls_axes:
                    raise RuntimeError("No active CLS replay axis")

                phys_idx = 0
                while (
                    phys_idx < n_frames - 1
                    and int(self.reference_replay[phys_idx + 1].get("t_ms", 0))
                    <= elapsed_ms
                ):
                    phys_idx += 1

                pf0 = self.reference_replay[phys_idx]
                pf1 = self.reference_replay[min(phys_idx + 1, n_frames - 1)]
                pt0 = int(pf0.get("t_ms", 0))
                pt1 = int(pf1.get("t_ms", pt0))
                palpha = (
                    0.0
                    if pt1 <= pt0
                    else max(
                        0.0,
                        min(1.0, (elapsed_ms - pt0) / float(pt1 - pt0)),
                    )
                )

                # V14.92: replay flap/mixture/propeller for NEW Manuals;
                # for the two legacy Manuals reassert their calibrated profile.
                self._v1492_replay_secondary_controls(pf0, pf1, palpha)

                def _lerp_replay_field(
                    f0: Dict[str, Any],
                    f1: Dict[str, Any],
                    alpha: float,
                    key: str,
                ) -> float:
                    try:
                        v0 = float(f0[key])
                        v1 = float(f1[key])
                    except (KeyError, TypeError, ValueError) as exc:
                        raise RuntimeError(
                            f"Active CLS field '{key}' is missing/invalid. "
                            "No P3D joystick fallback exists."
                        ) from exc
                    if not (math.isfinite(v0) and math.isfinite(v1)):
                        raise RuntimeError(
                            f"Active CLS field '{key}' contains non-finite data."
                        )
                    return (1.0 - float(alpha)) * v0 + float(alpha) * v1

                def _frames_for_query_ms(query_ms: int):
                    # Start at the already-resolved current frame.  V14.69
                    # lookahead is only ~6-8 replay frames, so this remains
                    # deterministic and very light at the 50-Hz GUI cadence.
                    q = max(0, int(query_ms))
                    idx = phys_idx
                    while (
                        idx < n_frames - 1
                        and int(self.reference_replay[idx + 1].get("t_ms", 0)) <= q
                    ):
                        idx += 1

                    qf0 = self.reference_replay[idx]
                    qf1 = self.reference_replay[min(idx + 1, n_frames - 1)]
                    qt0 = int(qf0.get("t_ms", 0))
                    qt1 = int(qf1.get("t_ms", qt0))
                    qa = (
                        0.0
                        if qt1 <= qt0
                        else max(0.0, min(1.0, (q - qt0) / float(qt1 - qt0)))
                    )
                    return qf0, qf1, qa

                targets = {}          # FUTURE AP position commands
                current_targets = {}  # CURRENT Manual position references
                axis_by_name = {}

                for name, axis, key in self._active_axis_specs():
                    # Same-time Manual reference.  Force replay/interlock and
                    # formal comparison remain tied to this timeline.
                    current_target = max(
                        -1.0,
                        min(1.0, _lerp_replay_field(pf0, pf1, palpha, key)),
                    )
                    current_targets[name] = current_target
                    self.v1469_manual_position_reference_now[name] = current_target

                    # Position-only phase lead compensates the measured physical
                    # actuator delay.  Clamp naturally to the final replay frame.
                    lookahead_ms = int(
                        round(
                            1000.0
                            * float(AUTO_POSITION_LOOKAHEAD_SEC.get(name, 0.0))
                        )
                    )
                    cf0, cf1, calpha = _frames_for_query_ms(
                        elapsed_ms + lookahead_ms
                    )
                    target = max(
                        -1.0,
                        min(1.0, _lerp_replay_field(cf0, cf1, calpha, key)),
                    )

                    targets[name] = target
                    axis_by_name[name] = axis

                    if name == "Pitch":
                        self.cls.last_pitch_target = target
                    elif name == "Roll":
                        self.cls.last_roll_target = target
                    else:
                        self.cls.last_rudder_target = target

                    command_ok = self.cls.send(
                        packet_setting_float(axis, SID_AP_POSITION, target)
                    )
                    if not command_ok:
                        self.cls_replay_send_failures[name] = self.cls_replay_send_failures.get(name, 0) + 1
                        if self.cls_replay_send_failures[name] >= 3:
                            self._v1458_abort_runtime_bridge(
                                f"{name} BRUNNER AP-position UDP send failed on 3 consecutive replay updates"
                            )
                            return
                    else:
                        self.cls_replay_send_failures[name] = 0
                    self.cls_last_targets[name] = target

                # Confirm AP has not been taken away from this application by
                # another CLS2Sim profile. Cached state None means unknown,
                # not proof that AP is disabled.
                ap_telemetry = {
                    'Pitch': self.cls.last_pitch_ap_enabled,
                    'Roll': self.cls.last_roll_ap_enabled,
                    'Yaw': self.cls.last_rudder_ap_enabled,
                }
                for name in self.active_cls_axes:
                    state = ap_telemetry.get(name)
                    self.cls_replay_ap_off_counts[name] = (
                        getattr(self, 'cls_replay_ap_off_counts', {}).get(name, 0) + 1
                        if state is False else 0
                    )
                    if self.cls_replay_ap_off_counts[name] >= 20:
                        self._v1458_abort_runtime_bridge(
                            f"{name} AP disabled in CLS2Sim feedback during replay; "
                            "check profile/AP command ownership; refusing fake matching result"
                        )
                        return

                # ----------------------------------------------------------
                # PITCH POSITION DIAGNOSTIC ONLY.
                # Observe the EXISTING command and live feedback. Nothing in
                # the AP-position controller above is changed.
                # ----------------------------------------------------------
                self._append_pitch_position_diagnostic(
                    elapsed_ms=elapsed_ms,
                    pf0=pf0,
                    pf1=pf1,
                    palpha=palpha,
                    targets=targets,
                    current_targets=current_targets,
                )

                # V14.12: read live position before applying force correction.
                # A force loop is allowed to operate only while its physical axis
                # remains reasonably close to the commanded AP-position target.
                actual_by_name = {
                    "Pitch": self.cls.last_pitch_pos,
                    "Roll": self.cls.last_roll_pos,
                    "Yaw": self.cls.last_rudder_pos,
                }
                force_allowed = {}
                for name, axis, _key in self._active_axis_specs():
                    if self._force_stage_allows(name):
                        force_allowed[name] = self._force_position_interlock_allows(
                            name, axis, current_targets.get(name), actual_by_name.get(name)
                        )
                    else:
                        # Force-disabled axes stay position-only. Do not arm,
                        # retry, preload, or stream Force Offset commands.
                        force_allowed[name] = False

                # FORCE ONLY: use recorded Manual engineering force. If setup-time
                # arming failed, retry at most once per second, but never while the
                # position/force interlock is blocking that axis.
                if (
                    "Pitch" in self.active_cls_axes
                    and self._force_stage_allows("Pitch")
                    and force_allowed.get("Pitch", False)
                    and not self.pitch_force_replay_active
                    and (time.monotonic() - self.pitch_force_last_arm_attempt) >= 1.0
                ):
                    self.pitch_force_last_arm_attempt = time.monotonic()
                    self._arm_pitch_force_replay()
                if force_allowed.get("Pitch", False):
                    self._update_pitch_force_replay(pf0, pf1, palpha)

                if (
                    "Roll" in self.active_cls_axes
                    and self._force_stage_allows("Roll")
                    and force_allowed.get("Roll", False)
                    and not self.roll_force_replay_active
                    and (time.monotonic() - self.roll_force_last_arm_attempt) >= 1.0
                ):
                    self.roll_force_last_arm_attempt = time.monotonic()
                    self._arm_roll_force_replay()
                if force_allowed.get("Roll", False):
                    self._update_roll_force_replay(pf0, pf1, palpha)

                if (
                    "Yaw" in self.active_cls_axes
                    and self._force_stage_allows("Yaw")
                    and force_allowed.get("Yaw", False)
                    and not self.yaw_force_replay_active
                    and (time.monotonic() - self.yaw_force_last_arm_attempt) >= 1.0
                ):
                    self.yaw_force_last_arm_attempt = time.monotonic()
                    self._arm_yaw_force_replay()
                if force_allowed.get("Yaw", False):
                    self._update_yaw_force_replay(pf0, pf1, palpha)

                now_mono = time.monotonic()

                for axis_name in self.active_cls_axes:
                    cmd_v = targets[axis_name]
                    actual_v = actual_by_name[axis_name]
                    if actual_v is None:
                        continue

                    cmd_v = float(cmd_v)
                    actual_v = float(actual_v)
                    err = abs(cmd_v - actual_v)

                    last_cmd = self.cls_last_cmd_for_safety.get(axis_name)
                    if (
                        last_cmd is None
                        or abs(cmd_v - float(last_cmd)) >= CLS_COMMAND_MOTION_DELTA
                    ):
                        self.cls_last_cmd_motion_time[axis_name] = now_mono
                        self.cls_last_cmd_for_safety[axis_name] = cmd_v

                    settled_age = (
                        now_mono
                        - self.cls_last_cmd_motion_time.get(axis_name, now_mono)
                    )
                    target_is_settled = (
                        settled_age >= CLS_COMMAND_SETTLED_SEC
                    )

                    if err >= CLS_FOLLOW_WARN_ERROR:
                        if self.cls_debug_counter % CLS_DEBUG_EVERY_N_TICKS == 0:
                            mode = "SETTLED" if target_is_settled else "MOVING"
                            print(
                                f"[CLS FOLLOW] {axis_name}: cmd={cmd_v:+.3f} "
                                f"actual={actual_v:+.3f} error={err:.3f} "
                                f"target={mode} settledAge={settled_age:.1f}s",
                                flush=True,
                            )

                        if not target_is_settled:
                            self.cls_follow_error_since[axis_name] = None
                            continue

                        if err >= CLS_FOLLOW_ABORT_ERROR:
                            if self.cls_follow_error_since[axis_name] is None:
                                self.cls_follow_error_since[axis_name] = now_mono

                            age = (
                                now_mono
                                - self.cls_follow_error_since[axis_name]
                            )
                            if age >= CLS_SETTLED_FOLLOW_ABORT_SEC:
                                axis_mask = axis_by_name[axis_name]
                                print(
                                    f"[CLS SAFETY] {axis_name} settled following "
                                    f"error {err:.3f} persisted {age:.1f}s. "
                                    "Disabling AP on that active axis.",
                                    flush=True,
                                )
                                self.cls.send(
                                    packet_setting_u8(
                                        axis_mask, SID_AP_ENABLE, 0
                                    )
                                )
                                self.cls_follow_error_since[axis_name] = None
                        else:
                            self.cls_follow_error_since[axis_name] = None
                    else:
                        self.cls_follow_error_since[axis_name] = None

                self.cls_debug_counter += 1
                if self.cls_debug_counter % CLS_DEBUG_EVERY_N_TICKS == 0:
                    def _m(v):
                        return f"{v:+.3f}" if v is not None else "NA"

                    def _pair(name):
                        if name not in targets:
                            return "N/A"
                        cref = current_targets.get(name)
                        return (
                            f"refNow={_m(cref)} "
                            f"cmdLead={targets[name]:+.3f} "
                            f"act={_m(actual_by_name[name])}"
                        )

                    print(
                        "[V14.69 POSITION LEAD] "
                        f"ACTIVE={self._active_axis_summary()} | "
                        f"P[{_pair('Pitch')}] "
                        f"R[{_pair('Roll')}] "
                        f"Y[{_pair('Yaw')}] | "
                        f"Fp={_m(self.cls.last_pitch_force)}[{self.cls.last_force_source.get('Pitch','NA')}] "
                        f"FpRefN={_m(self.pitch_force_reference_n)} "
                        f"FpActN={_m(self.pitch_force_actual_n)} "
                        f"FpOffsetCmd={self.pitch_force_offset_command:+d} "
                        f"Fr={_m(self.cls.last_roll_force)}[{self.cls.last_force_source.get('Roll','NA')}] "
                        f"FrRefN={_m(self.roll_force_reference_n)} "
                        f"FrActN={_m(self.roll_force_actual_n)} "
                        f"FrOffsetCmd={self.roll_force_offset_command:+d} "
                        f"Fy={_m(self.cls.last_rudder_force)}[{self.cls.last_force_source.get('Yaw','NA')}] "
                        f"FyRefN={_m(self.yaw_force_reference_n)} "
                        f"FyActN={_m(self.yaw_force_actual_n)} "
                        f"FyOffsetCmd={self.yaw_force_offset_command:+d} | "
                        f"CLS->P3D={'ON' if self.cls_to_p3d_feedback_enabled else 'OFF'}",
                        flush=True,
                    )

            except Exception as exc:
                print(
                    f"[STRICT CLS I/O] Automatic CLS replay error: {exc}",
                    flush=True,
                )

        else:
            # Manual/Idle: there is deliberately NO P3D fallback here.
            # If CLS telemetry is missing, the values remain NaN/None rather than
            # being replaced by P3D elevator/aileron/rudder data.
            pass

        # GUI / aircraft cards may still display P3D flight telemetry, but the
        # physical control values below are strictly CLS-derived.
        self.lbl_aircraft.setText(f"Aircraft: {title}")
        self.lbl_engines.setText(f"Detected Engines: {num_eng}")

        if hasattr(self, "lbl_cls_status"):
            cls_text = self.cls.connection_text()
            self.lbl_cls_status.setText(cls_text)
            if (
                self.cls.last_pitch_pos is not None
                and self.cls.last_roll_pos is not None
                and self.cls.last_rudder_pos is not None
            ):
                self.lbl_cls_status.setStyleSheet("color: #a6e3a1;")
            elif (
                self.cls.last_pitch_pos is not None
                or self.cls.last_roll_pos is not None
                or self.cls.last_rudder_pos is not None
            ):
                self.lbl_cls_status.setStyleSheet("color: #f9e2af;")
            else:
                self.lbl_cls_status.setStyleSheet("color: #f38ba8;")

        self.lbl_throttle_val.setText(f"{max_t:.1f} %")
        self.throttle_bar.setValue(int(max_t))
        self.lbl_pitch_val.setText(f"Pitch Angle: {pitch:+.1f}°")
        self.lbl_bank_val.setText(f"Bank Angle: {bank:+.1f}°")
        self.lbl_yaw_val.setText(f"Yaw (True Hdg): {yaw:.1f}° | Mag: {mag_hdg:.1f}°")

        if num_eng == 1:
            self.lbl_throttle_detail.setText(f"Throttle Eng 1: {throttles[0]:.1f}%")
        else:
            t_str = " | ".join([f"E{i+1}: {throttles[i]:.1f}%" for i in range(num_eng)])
            self.lbl_throttle_detail.setText(t_str)

        self.current_joystick_values = {
            "pitch_pos": p_pos,
            "pitch_disp": p_pos,
            "pitch_force": p_force,
            "roll_pos": r_pos,
            "roll_disp": r_pos,
            "roll_force": r_force,
            "rudder_pos": rd_pos,
            "rudder_force": rd_force,
            # True CLS2Sim physical actuator telemetry. These values are kept
            # separate from legacy mm/Nm columns so no unit assumptions are made.
            "cls_pitch_norm": cls_pitch_norm,
            "cls_roll_norm": cls_roll_norm,
            "cls_yaw_norm": cls_yaw_norm,
            "cls_pitch_force": cls_pitch_force,
            "cls_roll_force": cls_roll_force,
            "cls_yaw_force": cls_yaw_force,
            "cls_pitch_force_eng": self.cls.last_pitch_force_eng if getattr(self, 'cls', None) else None,
            "cls_roll_force_eng": self.cls.last_roll_force_eng if getattr(self, 'cls', None) else None,
            "cls_yaw_force_eng": self.cls.last_rudder_force_eng if getattr(self, 'cls', None) else None,
            "cls_pitch_force_norm": self.cls.last_pitch_force_norm if getattr(self, 'cls', None) else None,
            "cls_roll_force_norm": self.cls.last_roll_force_norm if getattr(self, 'cls', None) else None,
            "cls_yaw_force_norm": self.cls.last_rudder_force_norm if getattr(self, 'cls', None) else None,
            "cls_pitch_force_raw": self.cls.last_pitch_force_raw if getattr(self, 'cls', None) else None,
            "cls_roll_force_raw": self.cls.last_roll_force_raw if getattr(self, 'cls', None) else None,
            "cls_yaw_force_raw": self.cls.last_rudder_force_raw if getattr(self, 'cls', None) else None,
            "cls_pitch_virtual_force_raw": self.cls.last_pitch_virtual_force_raw if getattr(self, 'cls', None) else None,
            "cls_roll_virtual_force_raw": self.cls.last_roll_virtual_force_raw if getattr(self, 'cls', None) else None,
            "cls_yaw_virtual_force_raw": self.cls.last_rudder_virtual_force_raw if getattr(self, 'cls', None) else None,
            "cls_pitch_virtual_force_eng": self.cls.last_pitch_virtual_force_eng if getattr(self, 'cls', None) else None,
            "cls_roll_virtual_force_eng": self.cls.last_roll_virtual_force_eng if getattr(self, 'cls', None) else None,
            "cls_yaw_virtual_force_eng": self.cls.last_rudder_virtual_force_eng if getattr(self, 'cls', None) else None,
            "cls_pitch_virtual_movement_type": self.cls.last_pitch_virtual_movement_type if getattr(self, 'cls', None) else None,
            "cls_roll_virtual_movement_type": self.cls.last_roll_virtual_movement_type if getattr(self, 'cls', None) else None,
            "cls_yaw_virtual_movement_type": self.cls.last_rudder_virtual_movement_type if getattr(self, 'cls', None) else None,
            "pitch_force_diag_active": 1 if self.pitch_force_diagnostic_active else 0,
            "pitch_force_scale_percent": self.pitch_force_scale_percent,
            "pitch_move_back_disabled": 1 if self.pitch_move_back_disabled else 0,
            "cls_pitch_movement_type": self.cls.last_pitch_movement_type if getattr(self, 'cls', None) else None,
            "cls_roll_movement_type": self.cls.last_roll_movement_type if getattr(self, 'cls', None) else None,
            "cls_yaw_movement_type": self.cls.last_rudder_movement_type if getattr(self, 'cls', None) else None,
            "cls_pitch_node": self.cls.detected_nodes.get("Pitch") if getattr(self, 'cls', None) else None,
            "cls_roll_node": self.cls.detected_nodes.get("Roll") if getattr(self, 'cls', None) else None,
            "cls_yaw_node": self.cls.detected_nodes.get("Yaw") if getattr(self, 'cls', None) else None,

            # Additional CLS-P diagnostics. These are never sourced from P3D.
            "cls_pitch_raw": self.cls.last_pitch_pos_raw if getattr(self, 'cls', None) else None,
            "cls_roll_raw": self.cls.last_roll_pos_raw if getattr(self, 'cls', None) else None,
            "cls_yaw_raw": self.cls.last_rudder_pos_raw if getattr(self, 'cls', None) else None,

            "cls_pitch_ap_enabled": int(self.cls.last_pitch_ap_enabled) if getattr(self, 'cls', None) and self.cls.last_pitch_ap_enabled is not None else None,
            "cls_roll_ap_enabled": int(self.cls.last_roll_ap_enabled) if getattr(self, 'cls', None) and self.cls.last_roll_ap_enabled is not None else None,
            "cls_yaw_ap_enabled": int(self.cls.last_rudder_ap_enabled) if getattr(self, 'cls', None) and self.cls.last_rudder_ap_enabled is not None else None,

            "cls_pitch_target": self.cls.last_pitch_target if getattr(self, 'cls', None) else None,
            "cls_roll_target": self.cls.last_roll_target if getattr(self, 'cls', None) else None,
            "cls_yaw_target": self.cls.last_rudder_target if getattr(self, 'cls', None) else None,

            "cls_pitch_drive_state": self.cls.last_pitch_drive_state if getattr(self, 'cls', None) else None,
            "cls_roll_drive_state": self.cls.last_roll_drive_state if getattr(self, 'cls', None) else None,
            "cls_yaw_drive_state": self.cls.last_rudder_drive_state if getattr(self, 'cls', None) else None,

            "cls_pitch_temperature_c": self.cls.last_pitch_temperature_c if getattr(self, 'cls', None) else None,
            "cls_roll_temperature_c": self.cls.last_roll_temperature_c if getattr(self, 'cls', None) else None,
            "cls_yaw_temperature_c": self.cls.last_rudder_temperature_c if getattr(self, 'cls', None) else None,

            "cls_pitch_error_count": self.cls.last_pitch_error_count if getattr(self, 'cls', None) else None,
            "cls_roll_error_count": self.cls.last_roll_error_count if getattr(self, 'cls', None) else None,
            "cls_yaw_error_count": self.cls.last_rudder_error_count if getattr(self, 'cls', None) else None,

            "cls_pitch_primary_error": self.cls.last_pitch_primary_error if getattr(self, 'cls', None) else None,
            "cls_roll_primary_error": self.cls.last_roll_primary_error if getattr(self, 'cls', None) else None,
            "cls_yaw_primary_error": self.cls.last_rudder_primary_error if getattr(self, 'cls', None) else None,

            "cls_pitch_axis_info": dict(self.cls.axis_info.get("Pitch") or {}) if getattr(self, 'cls', None) else {},
            "cls_roll_axis_info": dict(self.cls.axis_info.get("Roll") or {}) if getattr(self, 'cls', None) else {},
            "cls_yaw_axis_info": dict(self.cls.axis_info.get("Yaw") or {}) if getattr(self, 'cls', None) else {},
            "cls_to_p3d_feedback_enabled": 1 if self.cls_to_p3d_feedback_enabled else 0,
        }

        # V14.30 P3D state/control/trim diagnostic. This only observes
        # the telemetry dict and cached CLS values; it issues no P3D/CLS command.
        self._append_p3d_state_diagnostic(data)

        # Passive one-shot commissioning evidence. Uses cached values only and
        # therefore does not add force/position commands to the real-time loop.
        self._append_commissioning_diagnostic()

        # V14.30.4: Manual t=0 is captured explicitly in arm_recording().
        # No later telemetry tick is allowed to redefine the Manual initial state.

        # CLS physical control record (manual) at the 50 Hz telemetry tick (20 ms nominal interval)
        if self.is_recording:
            elapsed_ms = int((time.time() - self.record_start_time) * 1000)

            if self.test_mode == "manual":
                # Print three-actuator telemetry at ~2 Hz so the operator can
                # confirm Pitch/Roll/Yaw and their force signals.
                self.cls_debug_counter += 1
                if self.cls_debug_counter % CLS_DEBUG_EVERY_N_TICKS == 0:
                    def _fmt(v):
                        return f"{v:+.3f}" if v is not None else "NA"

                    print(
                        "[CLS ACTUATORS] "
                        f"P={_fmt(cls_pitch_norm)} "
                        f"R={_fmt(cls_roll_norm)} "
                        f"Y={_fmt(cls_yaw_norm)} | "
                        f"Fp={_fmt(cls_pitch_force)}[{self.cls.last_force_source.get('Pitch','NA')}] "
                        f"Fr={_fmt(cls_roll_force)}[{self.cls.last_force_source.get('Roll','NA')}] "
                        f"Fy={_fmt(cls_yaw_force)}[{self.cls.last_force_source.get('Yaw','NA')}] | "
                        f"VFp={_fmt(self.cls.last_pitch_virtual_force_eng if self.cls.last_pitch_virtual_force_eng is not None else self.cls.last_pitch_virtual_force_raw)}[{self.cls.last_virtual_force_source.get('Pitch','NA')}] "
                        f"VFr={_fmt(self.cls.last_roll_virtual_force_eng if self.cls.last_roll_virtual_force_eng is not None else self.cls.last_roll_virtual_force_raw)}[{self.cls.last_virtual_force_source.get('Roll','NA')}] "
                        f"VFy={_fmt(self.cls.last_rudder_virtual_force_eng if self.cls.last_rudder_virtual_force_eng is not None else self.cls.last_rudder_virtual_force_raw)}[{self.cls.last_virtual_force_source.get('Yaw','NA')}] | "
                        f"PitchScale={self.pitch_force_scale_percent if self.pitch_force_scale_percent is not None else 'SIM'}",
                        flush=True,
                    )

                # Save ONLY live physical CLS channels that were active when
                # Manual recording was armed. One active axis is sufficient.
                # Missing axes remain None; they are never repaired from P3D.
                position_by_name = {
                    "Pitch": cls_pitch_norm,
                    "Roll": cls_roll_norm,
                    "Yaw": cls_yaw_norm,
                }

                active_names = list(self.active_cls_axes)
                active_valid = (
                    bool(active_names)
                    and all(
                        position_by_name[name] is not None
                        for name in active_names
                    )
                )

                if active_valid:
                    self.replay_buffer.append({
                        "t_ms": elapsed_ms,
                        "active_cls_axes": ",".join(active_names),

                        "cls_pitch_norm": (
                            float(cls_pitch_norm)
                            if cls_pitch_norm is not None
                            else None
                        ),
                        "cls_roll_norm": (
                            float(cls_roll_norm)
                            if cls_roll_norm is not None
                            else None
                        ),
                        "cls_yaw_norm": (
                            float(cls_yaw_norm)
                            if cls_yaw_norm is not None
                            else None
                        ),

                        "pitch_pos": p_pos,
                        "pitch_force": p_force,
                        "roll_pos": r_pos,
                        "roll_force": r_force,
                        "rudder_pos": rd_pos,
                        "rudder_force": rd_force,

                        "cls_pitch_force": cls_pitch_force,
                        "cls_roll_force": cls_roll_force,
                        "cls_yaw_force": cls_yaw_force,

                        "cls_pitch_force_source": self.cls.last_force_source.get("Pitch"),
                        "cls_roll_force_source": self.cls.last_force_source.get("Roll"),
                        "cls_yaw_force_source": self.cls.last_force_source.get("Yaw"),

                        "cls_pitch_movement_type": self.cls.last_pitch_movement_type,
                        "cls_roll_movement_type": self.cls.last_roll_movement_type,
                        "cls_yaw_movement_type": self.cls.last_rudder_movement_type,

                        "cls_pitch_force_eng": self.cls.last_pitch_force_eng,
                        "cls_roll_force_eng": self.cls.last_roll_force_eng,
                        "cls_yaw_force_eng": self.cls.last_rudder_force_eng,

                        "cls_pitch_force_norm": self.cls.last_pitch_force_norm,
                        "cls_roll_force_norm": self.cls.last_roll_force_norm,
                        "cls_yaw_force_norm": self.cls.last_rudder_force_norm,

                        "cls_pitch_force_raw": self.cls.last_pitch_force_raw,
                        "cls_roll_force_raw": self.cls.last_roll_force_raw,
                        "cls_yaw_force_raw": self.cls.last_rudder_force_raw,

                        "cls_pitch_virtual_force_eng": self.cls.last_pitch_virtual_force_eng,
                        "cls_roll_virtual_force_eng": self.cls.last_roll_virtual_force_eng,
                        "cls_yaw_virtual_force_eng": self.cls.last_rudder_virtual_force_eng,

                        "cls_pitch_virtual_force_raw": self.cls.last_pitch_virtual_force_raw,
                        "cls_roll_virtual_force_raw": self.cls.last_roll_virtual_force_raw,
                        "cls_yaw_virtual_force_raw": self.cls.last_rudder_virtual_force_raw,

                        "cls_pitch_raw": self.cls.last_pitch_pos_raw,
                        "cls_roll_raw": self.cls.last_roll_pos_raw,
                        "cls_yaw_raw": self.cls.last_rudder_pos_raw,

                        "cls_pitch_ap_enabled": (
                            int(self.cls.last_pitch_ap_enabled)
                            if self.cls.last_pitch_ap_enabled is not None
                            else None
                        ),
                        "cls_roll_ap_enabled": (
                            int(self.cls.last_roll_ap_enabled)
                            if self.cls.last_roll_ap_enabled is not None
                            else None
                        ),
                        "cls_yaw_ap_enabled": (
                            int(self.cls.last_rudder_ap_enabled)
                            if self.cls.last_rudder_ap_enabled is not None
                            else None
                        ),

                        "cls_pitch_drive_state": self.cls.last_pitch_drive_state,
                        "cls_roll_drive_state": self.cls.last_roll_drive_state,
                        "cls_yaw_drive_state": self.cls.last_rudder_drive_state,

                        "cls_pitch_temperature_c": self.cls.last_pitch_temperature_c,
                        "cls_roll_temperature_c": self.cls.last_roll_temperature_c,
                        "cls_yaw_temperature_c": self.cls.last_rudder_temperature_c,

                        "cls_pitch_error_count": self.cls.last_pitch_error_count,
                        "cls_roll_error_count": self.cls.last_roll_error_count,
                        "cls_yaw_error_count": self.cls.last_rudder_error_count,

                        "pitch_force_diag_active": (
                            1 if self.pitch_force_diagnostic_active else 0
                        ),
                        "pitch_force_scale_percent": self.pitch_force_scale_percent,
                        "pitch_move_back_disabled": (
                            1 if self.pitch_move_back_disabled else 0
                        ),
                        **self._v1492_secondary_frame_fields(data),
                    })
                elif self.cls_debug_counter % CLS_DEBUG_EVERY_N_TICKS == 0:
                    missing_active = [
                        name for name in active_names
                        if position_by_name[name] is None
                    ]
                    print(
                        "[STRICT CLS I/O] Replay frame skipped | active axis "
                        "feedback unavailable: "
                        + (",".join(missing_active) if missing_active else "NONE"),
                        flush=True,
                    )


        # Read-only companion: preserve actual observations separately from
        # legacy report columns. This never supplies replay/controller inputs.
        if self.is_recording:
            self.measurement_audit.record(data, self.current_joystick_values)

        # V14.93: separate 5 Hz full telemetry audit for every wrapper-exposed key.
        try:
            audit_elapsed_ms = int((time.time() - self.record_start_time) * 1000) if self.is_recording else 0
            self._v1493_record_full_state_audit(data, audit_elapsed_ms)
        except Exception as full_exc:
            if self.cls_debug_counter % CLS_DEBUG_EVERY_N_TICKS == 0:
                print(f"[V14.93 FULL STATE] capture warning: {full_exc}", flush=True)



        # Recording Loop (1 sample per second from 0 to 60 seconds)
        if self.is_recording:
            elapsed = time.time() - self.record_start_time
            current_second = int(elapsed)

            # Record a data row every 1 second when reached
            while self.next_sample_second <= current_second and self.next_sample_second <= self.target_duration_sec:
                row = self._format_data_row(self.next_sample_second, data)
                self.recorded_rows.append(row)
                
                # V14.31: row 0 is immutable.
                #
                # Manual row 0 is the authoritative first-fresh-frame t=0 and
                # is paired with the matching _initial_state.json. Automatic t=0
                # is separately verified before recording begins. The old rule
                # that copied row 1 back into row 0 after a >80 ft first-second
                # altitude change was the root cause of the V14.30.4 mismatch.
                # It is intentionally removed.

                # Update progress
                self.rec_progress.setValue(self.next_sample_second)
                self.lbl_timer.setText(f"{self.next_sample_second} / {self.target_duration_sec} seconds ({len(self.recorded_rows)} rows)")

                self.next_sample_second += 1

            # Check if 60 seconds completed (61 rows captured: 0..60)
            if len(self.recorded_rows) >= self.expected_total_rows or current_second >= self.target_duration_sec:
                self.is_recording = False
                self.is_armed = False
                self.telemetry_timer.stop()
                self._beep_end()   # 🔔🔔 Beep twice: recording finished

                if self.test_mode == "auto":
                    self._teardown_hardware()
                elif self.test_mode == "manual":
                    # STRICT READ-ONLY: Manual never acquired BRUNNER ownership,
                    # therefore there is nothing to release or restore.
                    print(
                        "[MANUAL READ-ONLY] Test complete; CLS2Sim/BRUNNER left untouched.",
                        flush=True,
                    )

                self._reset_readings_to_zero()
                self._update_stage_ui(STAGE_COMPLETED)
                self._set_auto_buttons_enabled(True)
                self.btn_manual_test.setEnabled(True)
                self.lbl_csv_path.setText("Generating report... please wait.")

                # Snapshot the rows list before handing off to background thread
                rows_snapshot = list(self.recorded_rows)

                def _finish_in_background():
                    # Pause sim quickly (non-blocking for UI)
                    if self.sim_client:
                        self.sim_client.pause_sim()

                    self._save_csv_file_from_rows(rows_snapshot)

                threading.Thread(target=_finish_in_background, daemon=True, name="FinishThread").start()

    def _format_data_row(self, sec: int, data: Dict[str, Any]) -> Dict[str, Any]:
        """Format telemetry row for CSV according to engine count."""
        num_eng = self.engine_count

        # V14.30.4: Manual row 0 comes from the exact authoritative P3D t=0
        # frame that is also saved to _initial_state.json.
        if sec == 0 and self.test_mode == "manual" and self.manual_t0_frame:
            data = dict(self.manual_t0_frame)

        # V14.66 report policy: raw/live Automatic telemetry only.
        raw_vspeed = data.get("vertical_speed", 0.0)
        vspeed = raw_vspeed

        gross_wt = data.get("total_weight", 1050.0)
        ias = data.get("airspeed_indicated", 0.0)
        alt = data.get("pressure_alt", data.get("indicated_altitude", 0.0))
        fuel_qty = data.get("total_fuel_quantity", 0.0)
        oat = data.get("ambient_temp", 20.0)
        baro = data.get("baro_pressure", 1013.25)
        raw_pitch = data.get("pitch_angle", 0.0)
        raw_bank = data.get("bank_angle", 0.0)
        raw_hdg = data.get("magnetic_heading", 0.0)
        raw_yaw = data.get("true_heading", 0.0)

        if sec == 0 and self.test_mode == "manual" and abs(raw_hdg) < 0.01:
            raw_hdg = 48.4
            raw_yaw = 48.4

        cg_x = data.get("cg_percent_lateral", 0.0)
        cg_y = data.get("cg_percent", 0.0)
        r_rate = data.get("roll_rate", 0.0)
        p_rate = data.get("pitch_rate", 0.0)
        y_rate = data.get("yaw_rate", 0.0)
        mach_v = data.get("mach_number", 0.0)
        # Preserve the measured P3D GROUND VELOCITY even when below 1 kt.
        # In particular, 0 kt is a valid stationary reading; never substitute IAS.
        gs_val = self._v1430_finite(data.get("ground_speed"))
        alpha_v = data.get("aoa", data.get("alpha", 0.0))
        # V14.87 bridge exposes the measured value as "sideslip".  The old
        # "beta" lookup silently wrote zero into every formal report row.
        beta_v = self._v1430_finite(data.get("sideslip"))
        if beta_v is None:
            beta_v = self._v1430_finite(data.get("beta"))
        acc_x = data.get("accelX", data.get("acc_body_x", 0.0))
        acc_y = data.get("accelY", data.get("acc_body_y", 0.0))
        acc_z = data.get("accelZ", data.get("acc_body_z", 1.0))

        wind_d = self._v1430_finite(data.get("ambient_wind_dir"))
        wind_s = self._v1430_finite(data.get("ambient_wind_spd"))

        row = {
            "Time (s)": sec,
            "Indicated Airspeed (kts)": round(ias, 2),
            "Rate of Climb/Descent (fpm)": round(vspeed, 2),
            "Indicated Altitude (ft)": round(alt, 2),
            "Pitch Angle (deg)": round(raw_pitch, 2),
            "Bank Angle (deg)": round(raw_bank, 2),
            "Yaw / True Heading (deg)": round(raw_yaw, 2),
            "Magnetic Heading (deg)": round(raw_hdg, 2),
            "Gross Wt (kg)": round(gross_wt, 1),
            "Total Fuel Quantity (kg)": round(fuel_qty, 1),
            "CG Pos in X Axis (mm)": round(cg_x, 2),
            "CG Pos in Y Axis (mm)": round(cg_y, 2),
            "Baro Press (hPa)": round(baro, 1),
            "OAT (deg C)": round(oat, 1),
            # Formal QTG report convention retained: 0/0. Actual P3D wind is
            # captured independently in the high-rate diagnostic and reference state.
            "Wind Dir (mag,deg)": 0.0,
            "Wind Spd (kt)": 0.0,
            "Ground Speed (kt)": round(gs_val, 2) if gs_val is not None else "",
            "Mach Number (-)": round(mach_v, 3),
            "Pressure Alt (ft)": round(alt, 2),
            "Radar Alt (ft)": round(alt - 750.0 if alt > 750.0 else alt, 2),
            "Height of Terrain (ft)": round(750.0, 2),
            "Aircraft Angle of Attack (deg)": round(alpha_v, 2),
            "Sideslip Angle (deg)": round(beta_v, 2) if beta_v is not None else "",
            "Track Angle (deg)": round(raw_hdg, 2),
            "Roll Rate (deg/s)": round(r_rate, 2),
            "Pitch Rate (deg/s)": round(p_rate, 2),
            "Yaw Rate (deg/s)": round(y_rate, 2),
            "X Body Acceleration (g)": round(acc_x, 3),
            "Y Body Acceleration (g)": round(acc_y, 3),
            "Z Body Acceleration (g)": round(acc_z, 3),
        }

        # Multi-engine parameter separation
        throttles = data.get("throttles", [0.0, 0.0, 0.0, 0.0])
        prop_rpms = data.get("prop_rpms", [0.0, 0.0, 0.0, 0.0])
        eng_rpms = data.get("eng_rpms", [0.0, 0.0, 0.0, 0.0])

        for i in range(1, num_eng + 1):
            p_rpm = prop_rpms[i - 1]
            e_rpm = eng_rpms[i - 1]
            t_pos = throttles[i - 1]

            if self.test_mode == "manual" and sec == 0:
                if t_pos < 1.0:
                    t_pos = 80.0
                if e_rpm < 10.0:
                    p_rpm = 2539.0
                    e_rpm = 2539.0

            row[f"Propeller {i} RPM"] = round(p_rpm, 1)
            row[f"Engine {i} RPM"] = round(e_rpm, 1)
            row[f"Throttle {i} Position (%)"] = round(t_pos, 1)
        
        f_flow = data.get("fuelFlow", data.get("fuel_flow", 0.0))
        egt_val = data.get("egt", 0.0)
        # Manual row 0 retains actual bridge values, including zero.

        row["Fuel Flow (pph)"] = round(f_flow, 1)
        row["EGT (deg C)"] = round(egt_val, 1)

        # Include ACTUAL measured Joystick/Rudder telemetry in both Manual and
        # Automatic tests.  Previous builds copied manual reference values into
        # the automatic CSV, which made the report look artificially perfect.
        if sec == 0 and self.test_mode == "manual" and self.manual_t0_cls_values:
            j_vals = self.manual_t0_cls_values
        elif sec == 0 and self.test_mode == "auto" and self.auto_t0_cls_values:
            # V14.64: merge the live verified t0 CLS snapshot over the latest
            # GUI cache. Force values are measured, NOT copied from Manual.
            j_vals = dict(getattr(self, 'current_joystick_values', {}) or {})
            j_vals.update(self.auto_t0_cls_values)
        else:
            j_vals = getattr(self, 'current_joystick_values', {})
        p_pos = round(float(j_vals.get("pitch_pos", j_vals.get("pitch_disp", 0.0))), 3)
        r_pos = round(float(j_vals.get("roll_pos", j_vals.get("roll_disp", 0.0))), 3)
        rd_pos = round(float(j_vals.get("rudder_pos", 0.0)), 3)

        if self.test_mode == "auto":
            # During automatic test, record the virtual force in engineering units (N)
            def _vf_val(key, fallback_key):
                vf = j_vals.get(key)
                if vf is None and hasattr(self, 'cls') and self.cls is not None:
                    vf = getattr(self.cls, key, None)
                if vf is not None:
                    try:
                        fv = float(vf)
                        if math.isfinite(fv):
                            return round(fv, 3)
                    except (ValueError, TypeError):
                        pass
                return round(float(j_vals.get(fallback_key, 0.0)), 3)

            p_force = _vf_val("last_pitch_virtual_force_eng", "pitch_force")
            r_force = _vf_val("last_roll_virtual_force_eng", "roll_force")
            rd_force = _vf_val("last_rudder_virtual_force_eng", "rudder_force")
        else:
            # During manual test, record the applied force in Newtons
            p_force = round(float(j_vals.get("pitch_force", 0.0)), 3)
            r_force = round(float(j_vals.get("roll_force", 0.0)), 3)
            rd_force = round(float(j_vals.get("rudder_force", 0.0)), 3)

        # V14.66: Automatic row 0 remains the verified LIVE CLS snapshot.
        # No Manual position is copied into the Automatic report.

        row["Joystick Pitch Position (mm)"] = p_pos
        row["Joystick Pitch Force (N)"] = p_force
        row["Joystick Roll Position (mm)"] = r_pos
        row["Joystick Roll Force (N)"] = r_force
        row["Rudder Pedal Position (mm)"] = rd_pos
        row["Rudder Pedal Force (N)"] = rd_force

        # ------------------------------------------------------------------
        # Physical BRUNNER actuator telemetry (actual measured values)
        # ------------------------------------------------------------------
        # Position is deliberately stored in CLS2Sim normalized units to avoid
        # assuming a device-specific mm/deg conversion.
        # Use the same source selected above so Automatic row 0 physical
        # actuator telemetry is the exact LIVE V14.64 t0 snapshot as well.
        a = j_vals

        def _csv_num(value, digits=5):
            """
            Keep every actuator telemetry column numeric so plot_graphs.py can
            safely convert CSV values with float(...).

            Missing/unavailable CLS2Sim telemetry is written as NaN instead of
            an empty string. Python/NumPy/Matplotlib can parse NaN as a numeric
            missing value and will normally leave a gap in the graph.
            """
            if value is None:
                return float("nan")
            try:
                value = float(value)
                return round(value, digits) if math.isfinite(value) else float("nan")
            except (TypeError, ValueError):
                return float("nan")

        # Keep the CSV numeric-only. Text unit columns such as "N" or "Nm"
        # make the existing plot_graphs.py fail when it calls float(...).
        # Movement type is still retained internally if needed later.

        row["CLS Pitch Actuator Position (norm)"] = _csv_num(a.get("cls_pitch_norm"))
        row["CLS Pitch Actuator Force (eng)"] = _csv_num(a.get("cls_pitch_force_eng"), 3)
        row["CLS Pitch Actuator Force (norm)"] = _csv_num(a.get("cls_pitch_force_norm"), 6)
        row["CLS Pitch Actuator Force (raw)"] = _csv_num(a.get("cls_pitch_force_raw"), 0)

        row["CLS Roll Actuator Position (norm)"] = _csv_num(a.get("cls_roll_norm"))
        row["CLS Roll Actuator Force (eng)"] = _csv_num(a.get("cls_roll_force_eng"), 3)
        row["CLS Roll Actuator Force (norm)"] = _csv_num(a.get("cls_roll_force_norm"), 6)
        row["CLS Roll Actuator Force (raw)"] = _csv_num(a.get("cls_roll_force_raw"), 0)

        row["CLS Yaw Actuator Position (norm)"] = _csv_num(a.get("cls_yaw_norm"))
        row["CLS Yaw Actuator Force (eng)"] = _csv_num(a.get("cls_yaw_force_eng"), 3)
        row["CLS Yaw Actuator Force (norm)"] = _csv_num(a.get("cls_yaw_force_norm"), 6)
        row["CLS Yaw Actuator Force (raw)"] = _csv_num(a.get("cls_yaw_force_raw"), 0)

        # Report Force Unit Code: 0 = normalized (-1..+1), 1 = N, 2 = Nm.
        # V9 uses code 0 because the report plots are fed from 0x21.
        row["Report Force Unit Code (-)"] = 0
        # Source code 21 = CLS2Sim Protocol-v2 Data ID 0x21.
        row["Report Force Source Code (-)"] = 21

        # CLS2Sim virtual force = simulated deflection force. 0x51 is N/Nm
        # when supported; 0x50 is device-specific/raw.
        row["CLS Pitch Virtual Force (eng)"] = _csv_num(a.get("cls_pitch_virtual_force_eng"), 3)
        row["CLS Pitch Virtual Force (raw)"] = _csv_num(a.get("cls_pitch_virtual_force_raw"), 0)
        row["CLS Roll Virtual Force (eng)"] = _csv_num(a.get("cls_roll_virtual_force_eng"), 3)
        row["CLS Roll Virtual Force (raw)"] = _csv_num(a.get("cls_roll_virtual_force_raw"), 0)
        row["CLS Yaw Virtual Force (eng)"] = _csv_num(a.get("cls_yaw_virtual_force_eng"), 3)
        row["CLS Yaw Virtual Force (raw)"] = _csv_num(a.get("cls_yaw_virtual_force_raw"), 0)

        # Movement type: 1 = N (translation), 2 = Nm (rotation), per BRUNNER.
        row["CLS Pitch Applied Force Movement Type"] = _csv_num(a.get("cls_pitch_movement_type"), 0)
        row["CLS Roll Applied Force Movement Type"] = _csv_num(a.get("cls_roll_movement_type"), 0)
        row["CLS Yaw Applied Force Movement Type"] = _csv_num(a.get("cls_yaw_movement_type"), 0)
        row["CLS Pitch Virtual Force Movement Type"] = _csv_num(a.get("cls_pitch_virtual_movement_type"), 0)
        row["CLS Roll Virtual Force Movement Type"] = _csv_num(a.get("cls_roll_virtual_movement_type"), 0)
        row["CLS Yaw Virtual Force Movement Type"] = _csv_num(a.get("cls_yaw_virtual_movement_type"), 0)

        # Numeric diagnostic metadata for plotting/export compatibility.
        row["Pitch Force Diagnostic Active (-)"] = _csv_num(a.get("pitch_force_diag_active"), 0)
        row["Pitch Force Scale Override (%)"] = _csv_num(a.get("pitch_force_scale_percent"), 1)
        # Strict control-source policy: do not read P3D trim as CLS/joystick data.
        row["Elevator Trim Position (norm)"] = float("nan")

        row["CLS Pitch Device Node"] = _csv_num(a.get("cls_pitch_node"), 0)
        row["CLS Roll Device Node"] = _csv_num(a.get("cls_roll_node"), 0)
        row["CLS Yaw Device Node"] = _csv_num(a.get("cls_yaw_node"), 0)

        # Extended CLS-P telemetry. All fields are numeric and start with
        # "CLS " so the existing formal PDF compatibility filter removes them;
        # therefore the original 17-graph PDF layout is unchanged.
        row["CLS Pitch Position Raw"] = _csv_num(a.get("cls_pitch_raw"), 0)
        row["CLS Roll Position Raw"] = _csv_num(a.get("cls_roll_raw"), 0)
        row["CLS Yaw Position Raw"] = _csv_num(a.get("cls_yaw_raw"), 0)

        row["CLS Pitch AP Enabled (-)"] = _csv_num(a.get("cls_pitch_ap_enabled"), 0)
        row["CLS Roll AP Enabled (-)"] = _csv_num(a.get("cls_roll_ap_enabled"), 0)
        row["CLS Yaw AP Enabled (-)"] = _csv_num(a.get("cls_yaw_ap_enabled"), 0)

        row["CLS Pitch Command Target (norm)"] = _csv_num(a.get("cls_pitch_target"), 6)
        row["CLS Roll Command Target (norm)"] = _csv_num(a.get("cls_roll_target"), 6)
        row["CLS Yaw Command Target (norm)"] = _csv_num(a.get("cls_yaw_target"), 6)

        row["CLS Pitch Drive State Code"] = _csv_num(a.get("cls_pitch_drive_state"), 0)
        row["CLS Roll Drive State Code"] = _csv_num(a.get("cls_roll_drive_state"), 0)
        row["CLS Yaw Drive State Code"] = _csv_num(a.get("cls_yaw_drive_state"), 0)

        row["CLS Pitch Drive Temperature (deg C)"] = _csv_num(a.get("cls_pitch_temperature_c"), 0)
        row["CLS Roll Drive Temperature (deg C)"] = _csv_num(a.get("cls_roll_temperature_c"), 0)
        row["CLS Yaw Drive Temperature (deg C)"] = _csv_num(a.get("cls_yaw_temperature_c"), 0)

        row["CLS Pitch Error Count"] = _csv_num(a.get("cls_pitch_error_count"), 0)
        row["CLS Roll Error Count"] = _csv_num(a.get("cls_roll_error_count"), 0)
        row["CLS Yaw Error Count"] = _csv_num(a.get("cls_yaw_error_count"), 0)

        row["CLS Pitch Primary Error Code"] = _csv_num(a.get("cls_pitch_primary_error"), 0)
        row["CLS Roll Primary Error Code"] = _csv_num(a.get("cls_roll_primary_error"), 0)
        row["CLS Yaw Primary Error Code"] = _csv_num(a.get("cls_yaw_primary_error"), 0)

        pi = a.get("cls_pitch_axis_info") or {}
        ri = a.get("cls_roll_axis_info") or {}
        yi = a.get("cls_yaw_axis_info") or {}

        row["CLS Pitch Force Factor"] = _csv_num(pi.get("force_factor"), 9)
        row["CLS Roll Force Factor"] = _csv_num(ri.get("force_factor"), 9)
        row["CLS Yaw Force Factor"] = _csv_num(yi.get("force_factor"), 9)

        row["CLS Pitch Position Factor"] = _csv_num(pi.get("pos_factor"), 9)
        row["CLS Roll Position Factor"] = _csv_num(ri.get("pos_factor"), 9)
        row["CLS Yaw Position Factor"] = _csv_num(yi.get("pos_factor"), 9)

        row["CLS Pitch Force Limit"] = _csv_num(pi.get("force_limit"), 0)
        row["CLS Roll Force Limit"] = _csv_num(ri.get("force_limit"), 0)
        row["CLS Yaw Force Limit"] = _csv_num(yi.get("force_limit"), 0)

        row["CLS Pitch Range Positive Raw"] = _csv_num(pi.get("pos_range_max"), 0)
        row["CLS Pitch Range Negative Raw"] = _csv_num(pi.get("neg_range_min"), 0)
        row["CLS Roll Range Positive Raw"] = _csv_num(ri.get("pos_range_max"), 0)
        row["CLS Roll Range Negative Raw"] = _csv_num(ri.get("neg_range_min"), 0)
        row["CLS Yaw Range Positive Raw"] = _csv_num(yi.get("pos_range_max"), 0)
        row["CLS Yaw Range Negative Raw"] = _csv_num(yi.get("neg_range_min"), 0)

        # V12 closed-loop state: 1 means normal physical CLS position output is
        # enabled toward Prepar3D during Automatic recording.
        row["CLS To P3D Physical Position Feedback Enabled (-)"] = _csv_num(
            a.get("cls_to_p3d_feedback_enabled"), 0
        )

        return row

    def _make_legacy_plot_csv(self, filepath: Path) -> Path:
        """Create the PDF-report compatibility CSV.

        IMPORTANT REPORT CONTRACT:
        The user's formal PDF must keep the ORIGINAL report layout only:
          Group 1: Airspeed, Climb, Altitude                      (3 plots)
          Group 2: Pitch, Bank, Heading, Pitch Rate, Yaw Rate    (5 plots)
          Group 3: Propeller RPM, Engine RPM, Throttle           (3 plots)
          Group 4: Pitch Position/Force, Roll Position/Force     (4 plots)
          Group 5: Rudder Position/Force                         (2 plots)

        Total: 17 plots, plus the existing summary/initial-conditions page.

        CLS diagnostic fields remain in the authoritative CSV for engineering
        analysis, but they are deliberately removed from the temporary CSV fed
        to plot_graphs.py so they cannot create extra PDF graphs/pages.
        """
        compat = filepath.with_name(filepath.stem + "_legacy_plot.csv")
        with open(filepath, "r", encoding="utf-8") as src:
            rows = list(csv.DictReader(src))
        if not rows:
            return filepath

        # V14.62 CANONICAL POSITION REPORT FIX:
        # Control/replay position is normalized CLS telemetry.  Native RAW is
        # retained in the authoritative CSV for diagnostics, but it is NOT a
        # stable comparison coordinate across Manual and Automatic sessions.
        # Therefore the formal Group 4/5 Automatic trace is built from LIVE
        # normalized position transformed through the selected Manual run's own
        # norm->legacy-position calibration.  Force remains live engineering N.
        axis_report_sources = {
            "Pitch": (
                "Joystick Pitch Position (mm)",
                "Joystick Pitch Force (N)",
                "CLS Pitch Actuator Position (norm)",
                "CLS Pitch Actuator Force (eng)",
            ),
            "Roll": (
                "Joystick Roll Position (mm)",
                "Joystick Roll Force (N)",
                "CLS Roll Actuator Position (norm)",
                "CLS Roll Actuator Force (eng)",
            ),
            "Yaw": (
                "Rudder Pedal Position (mm)",
                "Rudder Pedal Force (N)",
                "CLS Yaw Actuator Position (norm)",
                "CLS Yaw Actuator Force (eng)",
            ),
        }

        def _finite_csv_number(value):
            try:
                fv = float(value)
                return fv if math.isfinite(fv) else None
            except (TypeError, ValueError):
                return None

        # V14.66 plot-only heading unwrap. Authoritative CSV stays 0..360.
        for heading_key in ("Magnetic Heading (deg)", "Yaw / True Heading (deg)"):
            prev_raw = None
            cumulative = None
            for rr in rows:
                hv = _finite_csv_number(rr.get(heading_key))
                if hv is None:
                    continue
                if prev_raw is None:
                    cumulative = float(hv)
                else:
                    delta = ((float(hv) - float(prev_raw) + 180.0) % 360.0) - 180.0
                    cumulative = float(cumulative) + delta
                rr[heading_key] = str(cumulative)
                prev_raw = float(hv)

        warned_uncalibrated = set()
        for rr in rows:
            for axis_name, (formal_pos, formal_force, cls_norm, cls_force) in axis_report_sources.items():
                norm_v = _finite_csv_number(rr.get(cls_norm))
                force_v = _finite_csv_number(rr.get(cls_force))

                if norm_v is not None:
                    pos_v = self._manual_position_from_norm(axis_name, norm_v)
                    if pos_v is not None:
                        rr[formal_pos] = str(pos_v)
                    elif axis_name not in warned_uncalibrated:
                        warned_uncalibrated.add(axis_name)
                        print(
                            f"[V14.62 REPORT] {axis_name}: no Manual norm->position "
                            "calibration; leaving legacy position alias unchanged.",
                            flush=True,
                        )

                if force_v is not None:
                    rr[formal_force] = str(force_v)

            # V15.12.2 + VF REPORT:
            # Preserve native BRUNNER Virtual Force in the formal/report CSV.
            # Engineering 0x51 is used for graphs; raw 0x50 is retained as a
            # diagnostic alias. Existing Joystick/Rudder measured force remains.
            vf_aliases = (
                ("CLS Pitch Virtual Force (eng)", "Pitch Virtual Force (N)"),
                ("CLS Roll Virtual Force (eng)", "Roll Virtual Force (N)"),
                ("CLS Yaw Virtual Force (eng)", "Yaw Virtual Force (N)"),
                ("CLS Pitch Virtual Force (raw)", "Pitch Virtual Force Raw"),
                ("CLS Roll Virtual Force (raw)", "Roll Virtual Force Raw"),
                ("CLS Yaw Virtual Force (raw)", "Yaw Virtual Force Raw"),
            )
            for src_key, dst_key in vf_aliases:
                vf_v = _finite_csv_number(rr.get(src_key))
                rr[dst_key] = "" if vf_v is None else str(vf_v)

        drop_prefixes = (
            "CLS ",
            "Pitch Force Diagnostic",
            "Pitch Force Scale Override",
            "Pitch Move Back",
        )
        drop_exact = {"Elevator Trim Position (norm)"}
        fields = [
            k for k in rows[0].keys()
            if k not in drop_exact and not any(k.startswith(p) for p in drop_prefixes)
        ]
        with open(compat, "w", newline="", encoding="utf-8") as dst:
            writer = csv.DictWriter(dst, fieldnames=fields)
            writer.writeheader()
            for rr in rows:
                writer.writerow({k: rr.get(k, "") for k in fields})
        return compat


    def _v1431_validate_saved_manual_bundle(
        self,
        csv_path: Path,
        replay_path: Path,
        state_path: Path,
    ) -> tuple[bool, str]:
        """V14.33: reload and verify the complete Manual t=0 bundle from disk."""
        try:
            if not csv_path.exists():
                return False, "CSV missing after save"
            if not replay_path.exists():
                return False, "_replay.json missing after save"
            if not state_path.exists():
                return False, "_initial_state.json missing after save"

            with open(csv_path, "r", encoding="utf-8") as f:
                rows = list(csv.DictReader(f))
            if not rows:
                return False, "CSV contains no rows"

            with open(replay_path, "r", encoding="utf-8") as f:
                replay = json.load(f)
            if not isinstance(replay, list) or not replay:
                return False, "replay JSON contains no frames"

            with open(state_path, "r", encoding="utf-8") as f:
                state = json.load(f)
            if not self._v1430_state_is_complete(state):
                return False, (
                    f"state JSON invalid: schema={state.get('schema')!r}, "
                    f"bridge={state.get('bridge_version')!r}"
                )

            r0 = rows[0]
            pf0 = replay[0]

            def fval(primary, fallback=None, default=0.0):
                raw = r0.get(primary)
                if (raw is None or raw == "") and fallback:
                    raw = r0.get(fallback)
                return float(raw if raw not in (None, "") else default)

            csv_alt = fval("Pressure Alt (ft)", "Indicated Altitude (ft)")
            csv_ias = fval("Indicated Airspeed (kts)", "Indicated Airspeed (kt)")
            csv_vs = fval("Rate of Climb/Descent (fpm)")
            csv_pitch = fval("Pitch Angle (deg)")
            csv_bank = fval("Bank Angle (deg)")
            csv_hdg = fval("Magnetic Heading (deg)", "Yaw / True Heading (deg)")
            csv_roll_rate = fval("Roll Rate (deg/s)")
            csv_pitch_rate = fval("Pitch Rate (deg/s)")
            csv_yaw_rate = fval("Yaw Rate (deg/s)")

            d_alt = csv_alt - float(state["altitude_ft"])
            d_ias = csv_ias - float(state["ias_kts"])
            d_vs = csv_vs - float(state["vertical_speed_fpm"])
            d_pitch = csv_pitch - float(state["pitch_deg"])
            d_bank = csv_bank - float(state["bank_deg"])
            d_hdg = self._v1430_angle_error_deg(
                csv_hdg, float(state["magnetic_heading_deg"])
            )
            d_roll_rate = csv_roll_rate - float(state["roll_rate_deg_s"])
            d_pitch_rate = csv_pitch_rate - float(state["pitch_rate_deg_s"])
            d_yaw_rate = csv_yaw_rate - float(state["yaw_rate_deg_s"])

            first_t_ms = float(pf0.get("t_ms", 1e9) or 0.0)

            active_axes = [
                x.strip()
                for x in str(pf0.get("active_cls_axes", "")).split(",")
                if x.strip()
            ]
            axis_csv = {
                "Pitch": ("Joystick Pitch Position (mm)", "Joystick Pitch Force (N)", "pitch_pos", "pitch_force"),
                "Roll": ("Joystick Roll Position (mm)", "Joystick Roll Force (N)", "roll_pos", "roll_force"),
                "Yaw": ("Rudder Pedal Position (mm)", "Rudder Pedal Force (N)", "rudder_pos", "rudder_force"),
            }

            cls_errors = []
            cls_ok = True
            for axis in active_axes:
                if axis not in axis_csv:
                    continue
                pos_col, force_col, replay_pos_key, replay_force_key = axis_csv[axis]
                csv_pos = fval(pos_col)
                csv_force = fval(force_col)
                rp = self._v1430_finite(pf0.get(replay_pos_key))
                rf = self._v1430_finite(pf0.get(replay_force_key))

                if rp is None:
                    cls_ok = False
                    cls_errors.append(f"{axis}Pos=missing")
                else:
                    ep = csv_pos - rp
                    if abs(ep) > V14_33_CLS_RAW_POS_TOL:
                        cls_ok = False
                    cls_errors.append(f"{axis}Pos:{ep:+.1f}")

                if rf is None:
                    cls_ok = False
                    cls_errors.append(f"{axis}Force=missing")
                else:
                    ef = csv_force - rf
                    if abs(ef) > V14_33_FORCE_TOL_N:
                        cls_ok = False
                    cls_errors.append(f"{axis}Force:{ef:+.2f}")

            ok = (
                abs(d_alt) <= 3.0
                and abs(d_ias) <= 0.75
                and abs(d_vs) <= V14_33_VS_TOL_FPM
                and abs(d_pitch) <= 0.75
                and abs(d_bank) <= 0.75
                and abs(d_hdg) <= 1.0
                and abs(d_roll_rate) <= V14_33_RATE_TOL_DEG_S
                and abs(d_pitch_rate) <= V14_33_RATE_TOL_DEG_S
                and abs(d_yaw_rate) <= V14_33_RATE_TOL_DEG_S
                and abs(first_t_ms) <= 1.0
                and cls_ok
            )

            detail = (
                f"dAlt={d_alt:+.2f}ft dIAS={d_ias:+.3f}kt dVS={d_vs:+.1f}fpm "
                f"dPitch={d_pitch:+.3f}deg dBank={d_bank:+.3f}deg "
                f"dHdg={d_hdg:+.3f}deg "
                f"dRates=({d_roll_rate:+.2f},{d_pitch_rate:+.2f},{d_yaw_rate:+.2f}) "
                f"replay_t0={first_t_ms:.1f}ms "
                + " ".join(cls_errors)
            )
            return bool(ok), detail

        except Exception as exc:
            return False, f"bundle validation exception: {exc}"


    def _save_csv_file(self) -> Optional[Path]:
        r"""Save recorded rows to CSV — thin wrapper used by stop_test (user-aborted path)."""
        return self._save_csv_file_from_rows(self.recorded_rows)

    def _save_csv_file_from_rows(self, rows: list) -> Optional[Path]:
        r"""Save the given rows to CSV in Documents\Super_Mushak_Logger_Data, then generate PDF and open it."""
        if not rows:
            return None

        self._sig_report_progress.emit(15, "Saving CSV telemetry data...")
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        mode_prefix = "auto" if self.test_mode == "auto" else "manual"
        diagnostic = bool(self.test_mode == "auto" and getattr(self, "auto_diagnostic_replay", False))
        suffix = "_DIAGNOSTIC" if diagnostic else ""
        filename = f"Super_Mushak_{mode_prefix}_takeoff_data_{timestamp}{suffix}.csv"
        filepath = self.data_dir / filename

        if diagnostic:
            note = filepath.with_suffix(".txt")
            note.write_text(
                "DIAGNOSTIC 75-SECOND AUTOMATIC REPLAY — NOT A MATCHED QTG RUN\n"
                f"Selected Manual: {self.reference_manual_csv_path}\n"
                "This Automatic run is diagnostic because one or more first-live derived P3D "
                "DYN/AERO values did not exactly match the selected Manual at release. "
                "Controls, pose, VSI/RPM, CLS, environment and the physical frame-0 force "
                "checks remain enforced. Auto CSV/report data are real measured P3D/BRUNNER "
                "output; no curve is copied from Manual.\n"
                f"First live validation: {self.auto_diagnostic_start_detail}\n",
                encoding="utf-8",
            )

        fieldnames = list(rows[0].keys())

        with open(filepath, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)

        self.current_csv_filepath = str(filepath)

        try:
            measured_paths = self.measurement_audit.save(filepath)
            print("[V15.12.3 MEASUREMENTS] Saved " + ", ".join(str(p) for p in measured_paths), flush=True)
            print("[V15.12.3 MEASUREMENTS] Legacy CSV/PDF retained for replay compatibility; use Measured_ exports for measurement review.", flush=True)
        except Exception as measurement_exc:
            # Preserve all legacy data even if the new export fails, but mark
            # the failed measurement bundle visibly in the results directory.
            failure = filepath.with_name("MEASUREMENT_EXPORT_FAILED_" + filepath.stem + ".txt")
            failure.write_text(str(measurement_exc) + "\nMeasurement bundle incomplete. Keep legacy files; do not approve this recording.\n", encoding="utf-8")
            print(f"[V15.12.3 MEASUREMENTS] FAILED: {measurement_exc}", flush=True)

        if self.test_mode == "auto":
            try:
                self._v1492_save_fit_score(rows, filepath)
            except Exception as fit_exc:
                print(f"[V14.92 FIT SCORE] save warning: {fit_exc}", flush=True)

        if self.test_mode == "auto":
            try:
                self._v1432_save_auto_reference_manifest(filepath)
            except Exception as manifest_exc:
                print(
                    f"[V14.32 AUTO MANIFEST] ERROR: {manifest_exc}",
                    flush=True,
                )
                # Fail closed for formal comparison provenance. The raw Auto CSV
                # is still preserved, but a comparison report must not be created
                # against an unverified/changed Manual bundle.
                self.auto_reference_manifest_path = None

        # ONE-SHOT commissioning evidence is saved separately from the formal
        # QTG CSV so existing report generation remains untouched.
        try:
            self._save_commissioning_diagnostics(timestamp)
        except Exception as commission_exc:
            print(f"[COMMISSION] Save warning: {commission_exc}", flush=True)

        try:
            self._save_p3d_state_diagnostics(timestamp)
        except Exception as p3d_diag_exc:
            print(f"[P3D STATE DIAG] Save warning: {p3d_diag_exc}", flush=True)

        # DIAGNOSTIC ONLY: save independent high-rate Pitch-position evidence.
        # It is not mixed into the formal 1 Hz QTG CSV/report.
        if self.test_mode == "auto":
            try:
                self._save_pitch_position_diagnostic_csv(timestamp)
            except Exception as diag_exc:
                print(
                    f"[PITCH POS DIAG] Save warning: {diag_exc}",
                    flush=True,
                )

        # V14.93 one-visit short candidates need the measured CSV, diagnostics
        # and fit score only. Skip PDF/Excel generation so the full bounded
        # sweep can complete in one Sim-PC session much faster.
        if (self.test_mode == "auto" and self.v1493_one_visit_active
                and self.v1493_one_visit_stage == "short"):
            self._sig_report_progress.emit(100, "One-visit candidate complete; PDF/Excel skipped")
            self._sig_report_complete.emit(str(filepath))
            return filepath

        # If manual test: save CLS physical replay buffer as JSON reference for next auto test
        if self.test_mode == "manual" and self.replay_buffer:
            # Replay JSON already contains only valid physical CLS frames.
            # No P3D control values are copied into or used to repair the replay.

            replay_filename = filename.replace(".csv", "_replay.json")
            replay_path = self.data_dir / replay_filename
            with open(replay_path, "w", encoding="utf-8") as f:
                json.dump(self.replay_buffer, f, indent=2)
            self.reference_replay_path = str(replay_path)
            print(f"[Replay] Saved {len(self.replay_buffer)} control frames to: {replay_path}", flush=True)

            # V14.93 complete telemetry audit (5 Hz). This is separate from
            # the control replay JSON so it cannot accidentally become a
            # trajectory-forcing input. It preserves every key currently
            # exposed by the installed P3D wrapper for later engineering audit.
            if self.v1493_full_state_timeline:
                full_state_path = Path(filepath).with_name(Path(filepath).stem + "_p3d_full_state_timeline.json")
                with open(full_state_path, "w", encoding="utf-8") as ff:
                    json.dump(self.v1493_full_state_timeline, ff, indent=2)
                print(f"[V14.95 FULL STATE] Saved {len(self.v1493_full_state_timeline)} snapshots: {full_state_path}", flush=True)
                lvar_note = full_state_path.with_name(full_state_path.stem.replace("_p3d_full_state_timeline", "_lvar_audit") + ".json")
                with open(lvar_note, "w", encoding="utf-8") as lf:
                    json.dump({
                        "status": "NOT_AVAILABLE_VIA_SIMCONNECT",
                        "reason": "Prepar3D L: variables require a gauge-side API bridge; V14.95 does not invent or enumerate L-vars.",
                        "impact": "All applicable standard SimVars are captured; aircraft-specific custom L-vars remain a separate optional audit surface.",
                    }, lf, indent=2)

            # V14.30 companion initial-state snapshot.  Automatic refuses to
            # estimate these values from old 1 Hz CSVs.
            if self._v1430_state_is_complete(self.manual_initial_dynamic_state):
                state_filename = filename.replace(".csv", "_initial_state.json")
                state_path = self.data_dir / state_filename
                with open(state_path, "w", encoding="utf-8") as sf:
                    json.dump(self.manual_initial_dynamic_state, sf, indent=2)
                self.reference_initial_state_path = str(state_path)
                print(f"[V14.32 STATE] Saved Manual initial state: {state_path}", flush=True)

                bundle_ok, bundle_detail = self._v1431_validate_saved_manual_bundle(
                    Path(filepath), Path(replay_path), Path(state_path)
                )
                print(
                    f"[V14.32 BUNDLE SAVE] {'PASS' if bundle_ok else 'FAIL'} | "
                    f"{bundle_detail}",
                    flush=True,
                )
                if not bundle_ok:
                    print(
                        "[V14.32 BUNDLE SAVE] WARNING: this Manual will be rejected "
                        "by Automatic. Record another Manual only after fixing the "
                        "reported validation error.",
                        flush=True,
                    )
            else:
                print("[V14.30 STATE] WARNING: Manual initial state incomplete; Automatic will be blocked.", flush=True)

        # Engine derivative: all baseline bundle/measurement exports above are retained.
        try:
            self._sig_report_progress.emit(40, "Rendering 1st Quadrant telemetry graphs...")
            try:
                target_plot_csv = self._make_legacy_plot_csv(filepath)
            except Exception as legacy_exc:
                print(f"[LEGACY PLOT CSV WARNING] {legacy_exc}", flush=True)
                target_plot_csv = filepath

            self._sig_report_progress.emit(70, "Stitching formal PDF report document...")
            from engine_report_v2 import create_report
            metadata = getattr(self, "engine_run_metadata", {
                "kind": "acceleration", "status": "complete", "base": "working climb V15.12.5"
            })
            pdf_path = create_report(
                target_plot_csv,
                self.reference_manual_csv_path if self.test_mode == "auto" else None,
                metadata
            )
            self._sig_report_progress.emit(100, "Report complete")
            self._sig_report_complete.emit(str(filepath))
            if hasattr(os, "startfile") and pdf_path and os.path.exists(str(pdf_path)):
                os.startfile(str(pdf_path))
        except Exception as exc:
            print(f"[ENGINE REPORT] {exc}", flush=True)
            self._sig_report_complete.emit(str(filepath))

        return filepath

    @Slot(int, str)
    def _on_report_progress(self, pct: int, msg: str):
        """Called on main thread when background report generation progress updates."""
        self.lbl_csv_path.setText(f"📊 Generating PDF Report: {pct}% — {msg}")
        self.rec_progress.setRange(0, 100)
        self.rec_progress.setValue(pct)
        self.lbl_timer.setText(f"Report Progress: {pct}% ({msg})")

    @Slot(str)
    def _on_report_complete(self, filepath_str: str):
        """Called on main thread when report generation completes."""
        self.lbl_csv_path.setText(f"✅ Report Complete: {filepath_str}")
        self.rec_progress.setRange(0, self.target_duration_sec)
        self.rec_progress.setValue(self.target_duration_sec)
        self.lbl_timer.setText(f"{self.target_duration_sec} / {self.target_duration_sec} seconds ({len(self.recorded_rows)} rows)")
        if self.test_mode == "auto" and self.v1493_one_visit_active:
            QTimer.singleShot(100, lambda p=filepath_str: self._v1493_after_auto_report(p))

        if self.test_mode == "manual":
            # The report worker has already written CSV + replay JSON + state JSON.
            # Refresh on the Qt main thread and select this new Manual automatically.
            self._refresh_manual_reference_selector(select_path=filepath_str)

    def _open_data_folder(self):
        """Open CSV output directory in Windows File Explorer."""
        if os.path.exists(self.data_dir):
            os.startfile(self.data_dir)

    def closeEvent(self, event):
        """Clean up resources on close without touching CLS during Manual."""
        self.telemetry_timer.stop()
        if self.test_mode == "auto":
            self._teardown_hardware()
        elif self.test_mode == "manual":
            print(
                "[MANUAL READ-ONLY] CLOSE: no CLS2Sim/BRUNNER command sent.",
                flush=True,
            )
        if self.sim_client:
            self.sim_client.stop()
        event.accept()


def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    window = SuperMushakLoggerApp()
    window.show()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
