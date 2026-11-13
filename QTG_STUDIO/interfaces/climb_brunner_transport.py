# -*- coding: utf-8 -*-
"""BRUNNER CLS2Sim Protocol-v2 implementation copied unchanged from the
FSDC Super Mushak V15 climb logger.  Does not import P3D or Qt.

This module provides the exact UDP transport, force/virtual-force telemetry,
read-only guard, AP packet builders, watchdog and health polling.  It is only
the communication layer: the full climb replay, t=0 force lock, tracking,
and P3D handover live in the proven ClimbBrunnerApp base class.

Keep CLS2Sim profile, installed protocol and node mappings aligned with the
working climb simulator; never use on live motors without safety checks.
"""
import os
import math
import socket
import struct
import threading
import time
from typing import Any, Dict, List, Optional

CMD_STATE         = 0xCD

CMD_SETTINGS      = 0xCE

CMD_READ          = 0xD0

CMD_OVERRIDE      = 0xD1

CMD_REMOTE_CONFIG = 0xD8

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

CLS2SIM_HOST = os.environ.get("CLS2SIM_HOST", "127.0.0.1")

CLS2SIM_PORT = int(os.environ.get("CLS2SIM_PORT", "15090"))

LEGACY_FORCE_NORM_FULL_SCALE_COUNTS = 20000.0

LEGACY_FORCE_FACTOR_N_PER_COUNT = 0.1

LEGACY_FORCE_MATCH_TOL_COUNTS = 8.0

CLS_AXIS_SPECS = (
    ("Pitch", AX_ELEVATOR, "cls_pitch_norm"),
    ("Roll", AX_AILERON, "cls_roll_norm"),
    ("Yaw", AX_RUDDER, "cls_yaw_norm"),
)

CLS_WATCHDOG_MS = 500

CLS_WATCHDOG_HEARTBEAT_SEC = 0.10

CLS_FAST_TELEMETRY_SEC = 0.020      # 50 Hz normalized position

CLS_MEDIUM_TELEMETRY_SEC = 0.200    # 5 Hz raw position / virtual force / AP state

CLS_HEALTH_TELEMETRY_SEC = 1.000    # 1 Hz drive health/errors


def packet_override(axis_mask: int, override_id: int) -> bytes:
    """Build one Protocol-v2 override command."""
    return struct.pack("<Iii", CMD_OVERRIDE, int(axis_mask), int(override_id))


def packet_setting_u8(axis_mask: int, setting_id: int, value: int) -> bytes:
    return struct.pack("<IiiB", CMD_SETTINGS, int(axis_mask), int(setting_id), int(value) & 0xFF)


def packet_setting_u16(axis_mask: int, setting_id: int, value: int) -> bytes:
    return struct.pack("<IiiH", CMD_SETTINGS, int(axis_mask), int(setting_id), int(value) & 0xFFFF)


def packet_setting_i32(axis_mask: int, setting_id: int, value: int) -> bytes:
    """Build a signed Int32 setting packet for Pitch Force Offset only."""
    value = max(-2147483648, min(2147483647, int(value)))
    return struct.pack("<Iiii", CMD_SETTINGS, int(axis_mask), int(setting_id), value)


def packet_setting_float(axis_mask: int, setting_id: int, value: float) -> bytes:
    normalized = max(-1.0, min(1.0, float(value)))
    return struct.pack("<Iiif", CMD_SETTINGS, int(axis_mask), int(setting_id), normalized)


def packet_read(axis_mask: int, data_id: int) -> bytes:
    """Build a read-only CLS2Sim telemetry request packet."""
    return struct.pack("<III", CMD_READ, int(axis_mask), int(data_id))


def packet_watchdog(timeout_ms: int) -> bytes:
    """Build the D8/setting-0 watchdog command (zero disables it)."""
    return struct.pack("<IIH", CMD_REMOTE_CONFIG, RCFG_COMMUNICATION_WATCHDOG, int(timeout_ms) & 0xFFFF)


def packet_state(axis_mask: int, state_command: int) -> bytes:
    """Build a Protocol-v2 state-control packet (0xCD)."""
    return struct.pack("<Iii", CMD_STATE, int(axis_mask), int(state_command))


def node_id_mask(node_id: int) -> int:
    """Protocol-v2 Node ID Mode mask: node 11 -> 0x0BFFFFFF."""
    return ((int(node_id) & 0xFF) << 24) | 0x00FFFFFF


def packet_remote_config_u8(setting_id: int, value: int) -> bytes:
    """Build a Protocol-v2 RemoteControlConfig command with a UInt8 value."""
    return struct.pack("<IIB", CMD_REMOTE_CONFIG, int(setting_id), int(value) & 0xFF)


class Cls2SimUdpTransport:
    """
    CLS2Sim Protocol-v2 UDP transport for the three simulator actuators.

    Pitch -> Elevator 0x01 -> nodes 2/4
    Roll  -> Aileron  0x02 -> nodes 1/3
    Yaw   -> Rudder   0x04 -> nodes 5/6

    FORCE V4 strategy
    -----------------
    BRUNNER's current Protocol-v2 Data table documents 0x20, 0x21 and 0x22,
    while the explicit Read Data request list includes 0x20 and 0x21.  For
    compatibility with CLS2Sim 5.5.x and newer builds this class therefore:

      1. discovers the real physical node from normalized position (0x11),
      2. probes 0x21 normalized force first,
      3. probes 0x20 raw/legacy force second,
      4. probes 0x22 engineering force as an optional capability,
      5. probes 0x23 Axis Information only if raw conversion may be useful,
      6. caches the working force method so unsupported IDs are NOT retried on
         every telemetry cycle.

    The watchdog has its own UDP socket and thread.  Telemetry reads can time
    out without preventing the watchdog reset from being sent in time.
    """

    EXPECTED_DEVICE_IDS = {
        AX_ELEVATOR: (2, 4),
        AX_AILERON: (1, 3),
        AX_RUDDER: (5, 6),
    }

    AXIS_NAMES = {
        AX_ELEVATOR: "Pitch",
        AX_AILERON: "Roll",
        AX_RUDDER: "Yaw",
    }

    def __init__(self, host: str = CLS2SIM_HOST, port: int = CLS2SIM_PORT):
        self.address = (host, int(port))
        self._socket: Optional[socket.socket] = None
        self._lock = threading.RLock()

        self._watchdog_socket: Optional[socket.socket] = None
        self._watchdog_stop = threading.Event()
        self._watchdog_thread: Optional[threading.Thread] = None
        self._telemetry_stop = threading.Event()
        self._telemetry_thread: Optional[threading.Thread] = None
        self._medium_stop = threading.Event()
        self._medium_thread: Optional[threading.Thread] = None
        self._health_stop = threading.Event()
        self._health_thread: Optional[threading.Thread] = None
        self.watchdog_enabled = False

        # V14.51 hard Manual guard. When enabled, controller-mutating protocol
        # packets are rejected locally before they can reach CLS2Sim. Telemetry
        # reads plus read-only status/error queries remain allowed.
        self.control_write_guard = False
        self.control_write_guard_reason = ""
        self.blocked_control_write_count = 0
        self.blocked_control_write_labels: List[str] = []

        self.last_pitch_pos: Optional[float] = None
        self.last_roll_pos: Optional[float] = None
        self.last_rudder_pos: Optional[float] = None

        # Additional CLS-P telemetry. The normalized positions above remain the
        # ONLY authoritative Manual-record / Automatic-replay position channels.
        self.last_pitch_pos_raw: Optional[float] = None
        self.last_roll_pos_raw: Optional[float] = None
        self.last_rudder_pos_raw: Optional[float] = None

        self.last_pitch_ap_enabled: Optional[bool] = None
        self.last_roll_ap_enabled: Optional[bool] = None
        self.last_rudder_ap_enabled: Optional[bool] = None

        # Last normalized AP-position target written by THIS application.
        self.last_pitch_target: Optional[float] = None
        self.last_roll_target: Optional[float] = None
        self.last_rudder_target: Optional[float] = None

        # Drive health from Protocol-v2 State Control.
        self.last_pitch_drive_state: Optional[int] = None
        self.last_roll_drive_state: Optional[int] = None
        self.last_rudder_drive_state: Optional[int] = None

        self.last_pitch_temperature_c: Optional[int] = None
        self.last_roll_temperature_c: Optional[int] = None
        self.last_rudder_temperature_c: Optional[int] = None

        self.last_pitch_error_count: int = 0
        self.last_roll_error_count: int = 0
        self.last_rudder_error_count: int = 0

        self.last_pitch_primary_error: Optional[int] = None
        self.last_roll_primary_error: Optional[int] = None
        self.last_rudder_primary_error: Optional[int] = None

        self._static_axis_info_printed = set()

        # Generic display value. Engineering units when available; otherwise
        # normalized or raw, with last_force_source identifying the meaning.
        self.last_pitch_force: Optional[float] = None
        self.last_roll_force: Optional[float] = None
        self.last_rudder_force: Optional[float] = None

        # Explicit force channels so the CSV never has to guess units.
        self.last_pitch_force_eng: Optional[float] = None
        self.last_roll_force_eng: Optional[float] = None
        self.last_rudder_force_eng: Optional[float] = None
        self.last_pitch_force_norm: Optional[float] = None
        self.last_roll_force_norm: Optional[float] = None
        self.last_rudder_force_norm: Optional[float] = None
        self.last_pitch_force_raw: Optional[float] = None
        self.last_roll_force_raw: Optional[float] = None
        self.last_rudder_force_raw: Optional[float] = None

        # Virtual force = simulated deflection force calculated by CLS2Sim.
        # 0x50 is device-specific/raw, 0x51 is N/Nm when supported.
        self.last_pitch_virtual_force_raw: Optional[float] = None
        self.last_roll_virtual_force_raw: Optional[float] = None
        self.last_rudder_virtual_force_raw: Optional[float] = None
        self.last_pitch_virtual_force_eng: Optional[float] = None
        self.last_roll_virtual_force_eng: Optional[float] = None
        self.last_rudder_virtual_force_eng: Optional[float] = None
        self.last_pitch_virtual_movement_type: Optional[int] = None
        self.last_roll_virtual_movement_type: Optional[int] = None
        self.last_rudder_virtual_movement_type: Optional[int] = None
        self.last_virtual_force_source: Dict[str, str] = {
            "Pitch": "UNPROBED", "Roll": "UNPROBED", "Yaw": "UNPROBED"
        }
        self.virtual_force_mode: Dict[str, Optional[str]] = {
            "Pitch": None, "Roll": None, "Yaw": None
        }
        self.virtual_force_probe_done: Dict[str, bool] = {
            "Pitch": False, "Roll": False, "Yaw": False
        }

        self.last_pitch_movement_type: Optional[int] = None
        self.last_roll_movement_type: Optional[int] = None
        self.last_rudder_movement_type: Optional[int] = None

        self.last_force_source: Dict[str, str] = {
            "Pitch": "UNPROBED",
            "Roll": "UNPROBED",
            "Yaw": "UNPROBED",
        }
        # eng22 | raw20-eng | legacy16-eng | norm21 | raw20 | unavailable
        self.force_mode: Dict[str, Optional[str]] = {
            "Pitch": None,
            "Roll": None,
            "Yaw": None,
        }
        self.force_probe_done: Dict[str, bool] = {
            "Pitch": False,
            "Roll": False,
            "Yaw": False,
        }

        self.position_sample_times = {}
        self.last_rx_time: Optional[float] = None
        self.last_socket_error = ""
        self.rx_count = 0
        self.last_read_error: Dict[str, str] = {}

        self.detected_nodes: Dict[str, Optional[int]] = {
            "Pitch": None,
            "Roll": None,
            "Yaw": None,
        }
        self.axis_info: Dict[str, Dict[str, Any]] = {
            "Pitch": {},
            "Roll": {},
            "Yaw": {},
        }

        # Force-only conversion metadata. Native 0x23 data is mirrored here.
        # On legacy v474, where 0x23 is absent, a validated synthetic conversion
        # record is stored here without polluting axis_info (which also carries
        # position-range information used elsewhere).
        self.force_conversion_info: Dict[str, Dict[str, Any]] = {
            "Pitch": {},
            "Roll": {},
            "Yaw": {},
        }

        # One-shot commissioning evidence. These structures are diagnostic only
        # and never participate in the control law.
        self.ack_history: List[Dict[str, Any]] = []
        self.force_probe_history: List[Dict[str, Any]] = []
        self.force_support: Dict[str, List[str]] = {
            "Pitch": [], "Roll": [], "Yaw": []
        }
        self.last_force_wire_raw: Dict[str, Optional[float]] = {
            "Pitch": None, "Roll": None, "Yaw": None
        }
        self.last_force_wire_norm: Dict[str, Optional[float]] = {
            "Pitch": None, "Roll": None, "Yaw": None
        }
        self.legacy_force_validation: Dict[str, Dict[str, Any]] = {
            "Pitch": {"checks": 0, "failures": 0, "max_abs_error_counts": 0.0,
                      "last_error_counts": None},
            "Roll": {"checks": 0, "failures": 0, "max_abs_error_counts": 0.0,
                     "last_error_counts": None},
            "Yaw": {"checks": 0, "failures": 0, "max_abs_error_counts": 0.0,
                    "last_error_counts": None},
        }

        self.device_states: Dict[int, int] = {}
        self.device_temperatures: Dict[int, int] = {}
        self.device_errors: Dict[int, List[int]] = {}

    # ----------------------------- sockets ---------------------------------
    def open(self) -> None:
        with self._lock:
            if self._socket is None:
                self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self._socket.settimeout(0.008)
                try:
                    self._socket.sendto(
                        packet_remote_config_u8(RCFG_MESSAGE_IDENTIFIER, 0),
                        self.address,
                    )
                except OSError as exc:
                    self.last_socket_error = str(exc)

        print(
            f"[CLS NET] UDP target {self.address[0]}:{self.address[1]} "
            "(message identifiers OFF)",
            flush=True,
        )

    def _open_watchdog_socket(self) -> None:
        if self._watchdog_socket is None:
            self._watchdog_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._watchdog_socket.settimeout(0.005)

    def _flush_rx_locked(self) -> None:
        if self._socket is None:
            return
        old_timeout = self._socket.gettimeout()
        try:
            self._socket.settimeout(0.0)
            while True:
                try:
                    self._socket.recvfrom(4096)
                except (BlockingIOError, socket.timeout):
                    break
                except OSError:
                    break
        finally:
            self._socket.settimeout(old_timeout)

    def flush_rx(self) -> None:
        with self._lock:
            self._flush_rx_locked()

    def set_control_write_guard(self, enabled: bool, reason: str = "") -> None:
        self.control_write_guard = bool(enabled)
        self.control_write_guard_reason = str(reason or "")
        if enabled:
            self.blocked_control_write_count = 0
            self.blocked_control_write_labels = []
        print(
            f"[CLS MANUAL GUARD] {'ENABLED' if enabled else 'DISABLED'}"
            + (f" | {self.control_write_guard_reason}" if self.control_write_guard_reason else ""),
            flush=True,
        )

    @staticmethod
    def _packet_is_read_only(packet: bytes) -> bool:
        """Return True only for Protocol-v2 telemetry/status requests."""
        try:
            if not packet or len(packet) < 4:
                return False
            cmd = struct.unpack_from("<I", packet, 0)[0]
            if cmd == CMD_READ:
                return True
            if cmd == CMD_STATE and len(packet) >= 12:
                state_cmd = struct.unpack_from("<i", packet, 8)[0]
                return int(state_cmd) in (STATE_GET_STATUS, STATE_GET_ERRORS)
            return False
        except Exception:
            return False

    def _guard_allows_packet(self, packet: bytes, label: str = "") -> bool:
        if not self.control_write_guard:
            return True
        if self._packet_is_read_only(packet):
            return True
        self.blocked_control_write_count += 1
        text = str(label or "unlabelled controller write")
        if len(self.blocked_control_write_labels) < 25:
            self.blocked_control_write_labels.append(text)
        print(f"[CLS MANUAL GUARD] BLOCKED controller write: {text}", flush=True)
        return False

    def send(self, packet: bytes) -> bool:
        with self._lock:
            if self._socket is None:
                return False
            if not self._guard_allows_packet(packet):
                return False
            try:
                self._socket.sendto(packet, self.address)
                return True
            except OSError as exc:
                self.last_socket_error = str(exc)
                return False

    def _record_ack_event(self, label: str, ok: bool, status=None, detail: str = "") -> None:
        """Store command/ACK evidence without changing command behavior."""
        try:
            self.ack_history.append({
                "wall_time": time.time(),
                "label": str(label),
                "ok": bool(ok),
                "status": status,
                "detail": str(detail or ""),
            })
        except Exception:
            pass

    def send_with_ack(self, packet: bytes, label: str, timeout_sec: float = 0.15) -> bool:
        """Send a setting/override/state command and validate generic status."""
        with self._lock:
            if self._socket is None:
                self._record_ack_event(label, False, None, "socket not open")
                return False
            if not self._guard_allows_packet(packet, label):
                self._record_ack_event(label, False, None, "blocked by Manual read-only guard")
                return False
            self._flush_rx_locked()
            try:
                self._socket.sendto(packet, self.address)
                deadline = time.monotonic() + timeout_sec
                while time.monotonic() < deadline:
                    try:
                        data, _ = self._socket.recvfrom(1024)
                    except socket.timeout:
                        continue
                    except OSError as exc:
                        self.last_socket_error = str(exc)
                        self._record_ack_event(label, False, None, f"socket error: {exc}")
                        return False
                    if len(data) < 3:
                        continue
                    status = data[2]
                    if status == 0:
                        self._record_ack_event(label, True, 0, "OK")
                        print(f"[CLS ACK] {label}: OK", flush=True)
                        return True
                    raw_text = data.hex(' ')
                    self._record_ack_event(
                        label, False, int(status), f"raw={raw_text}"
                    )
                    print(
                        f"[CLS ACK] {label}: ERROR status=0x{status:02X} raw={raw_text}",
                        flush=True,
                    )
                    return False
            except OSError as exc:
                self.last_socket_error = str(exc)
                self._record_ack_event(label, False, None, f"send error: {exc}")
                return False
        self._record_ack_event(label, False, None, "TIMEOUT")
        print(f"[CLS ACK] {label}: TIMEOUT", flush=True)
        return False

    # ------------------------ Protocol-v2 parsing ---------------------------
    @staticmethod
    def _body_size(data_id: int) -> Optional[int]:
        if data_id in (DATA_POS_NORM, DATA_POS_RAW, DATA_FORCE_RAW, DATA_FORCE_NORM, DATA_VIRTUAL_FORCE_RAW):
            return 6  # UInt16 device + 4-byte payload
        if data_id in (DATA_FORCE_ENG, DATA_VIRTUAL_FORCE_ENG):
            return 8  # device + MovementType UInt16 + Float
        if data_id == DATA_AP_ENABLED:
            return 3  # device + UInt8
        if data_id == DATA_AXIS_INFO:
            return 36
        return None

    @classmethod
    def _parse_read_items(cls, data_id: int, data: bytes):
        """Return (status, items). Handles repeated device bodies in one packet."""
        if len(data) < 3:
            return None, []
        status = int(data[2])
        if status != 0:
            return status, []
        body = cls._body_size(data_id)
        if body is None:
            return status, []

        items = []
        offset = 3
        while offset + body <= len(data):
            try:
                dev = struct.unpack_from("<H", data, offset)[0]
                if data_id == DATA_POS_NORM:
                    val = struct.unpack_from("<f", data, offset + 2)[0]
                    if math.isfinite(val):
                        items.append((float(val), int(dev), None))
                elif data_id == DATA_POS_RAW:
                    val = struct.unpack_from("<i", data, offset + 2)[0]
                    items.append((float(val), int(dev), None))
                elif data_id == DATA_FORCE_NORM:
                    val = struct.unpack_from("<f", data, offset + 2)[0]
                    if math.isfinite(val):
                        items.append((float(val), int(dev), None))
                elif data_id == DATA_FORCE_RAW:
                    val = struct.unpack_from("<i", data, offset + 2)[0]
                    items.append((float(val), int(dev), None))
                elif data_id == DATA_VIRTUAL_FORCE_RAW:
                    val = struct.unpack_from("<I", data, offset + 2)[0]
                    items.append((float(val), int(dev), None))
                elif data_id in (DATA_FORCE_ENG, DATA_VIRTUAL_FORCE_ENG):
                    mt = struct.unpack_from("<H", data, offset + 2)[0]
                    val = struct.unpack_from("<f", data, offset + 4)[0]
                    if math.isfinite(val):
                        items.append((float(val), int(dev), int(mt)))
                elif data_id == DATA_AP_ENABLED:
                    items.append((float(data[offset + 2]), int(dev), None))
                elif data_id == DATA_AXIS_INFO:
                    mt = struct.unpack_from("<H", data, offset + 2)[0]
                    force_factor = struct.unpack_from("<f", data, offset + 4)[0]
                    pos_factor = struct.unpack_from("<f", data, offset + 8)[0]
                    force_limit = struct.unpack_from("<I", data, offset + 12)[0]
                    range_valid = struct.unpack_from("<H", data, offset + 16)[0]
                    range_mode = struct.unpack_from("<H", data, offset + 18)[0]
                    pos_range_max = struct.unpack_from("<i", data, offset + 20)[0]
                    neg_range_min = struct.unpack_from("<i", data, offset + 24)[0]
                    pos_range_current = struct.unpack_from("<i", data, offset + 28)[0]
                    neg_range_current = struct.unpack_from("<i", data, offset + 32)[0]
                    info = {
                        "device_id": int(dev),
                        "movement_type": int(mt),
                        "force_factor": float(force_factor),
                        "pos_factor": float(pos_factor),
                        "force_limit": int(force_limit),
                        "range_valid": int(range_valid),
                        "range_mode": int(range_mode),
                        "pos_range_max": int(pos_range_max),
                        "neg_range_min": int(neg_range_min),
                        "pos_range_current": int(pos_range_current),
                        "neg_range_current": int(neg_range_current),
                    }
                    items.append((info, int(dev), int(mt)))
            except struct.error:
                break
            offset += body
        return status, items

    def _read_request(
        self,
        request_mask: int,
        data_id: int,
        expected_ids: Optional[set] = None,
        timeout_sec: float = 0.035,
        diagnostic_key: Optional[str] = None,
    ):
        with self._lock:
            if self._socket is None:
                return None
            self._flush_rx_locked()
            try:
                self._socket.sendto(packet_read(request_mask, data_id), self.address)
                deadline = time.monotonic() + timeout_sec
                while time.monotonic() < deadline:
                    try:
                        data, _ = self._socket.recvfrom(4096)
                    except socket.timeout:
                        continue
                    except OSError as exc:
                        self.last_socket_error = str(exc)
                        return None

                    status, items = self._parse_read_items(data_id, data)
                    if status is None:
                        continue
                    if status != 0:
                        if diagnostic_key:
                            self.last_read_error[diagnostic_key] = (
                                f"status=0x{status:02X} raw={data.hex(' ')}"
                            )
                        return None
                    for parsed in items:
                        dev = parsed[1]
                        if expected_ids and dev not in expected_ids:
                            continue
                        self.last_rx_time = time.monotonic()
                        self.rx_count += 1
                        return parsed
            except OSError as exc:
                self.last_socket_error = str(exc)
                return None

        if diagnostic_key:
            self.last_read_error[diagnostic_key] = "timeout/no matching payload"
        return None

    def _read_single_axis(self, axis_mask: int, data_id: int, timeout_sec: float = 0.050):
        expected = set(self.EXPECTED_DEVICE_IDS.get(axis_mask, ()))
        result = self._read_request(
            axis_mask,
            data_id,
            expected_ids=expected,
            timeout_sec=timeout_sec,
            diagnostic_key=f"axis0x{axis_mask:X}/data0x{data_id:X}",
        )
        if result is not None:
            name = self.AXIS_NAMES.get(axis_mask)
            if name:
                self.detected_nodes[name] = int(result[1])
        return result

    def _read_node(self, node_id: int, data_id: int, timeout_sec: float = 0.040):
        return self._read_request(
            node_id_mask(node_id),
            data_id,
            expected_ids={int(node_id)},
            timeout_sec=timeout_sec,
            diagnostic_key=f"node{node_id}/data0x{data_id:X}",
        )

    def _axis_name_for_mask(self, axis_mask: int) -> str:
        return self.AXIS_NAMES[axis_mask]

    def _ensure_axis_info(self, axis_mask: int) -> Dict[str, Any]:
        name = self._axis_name_for_mask(axis_mask)
        cached = self.axis_info.get(name) or {}
        node = self.detected_nodes.get(name)
        if cached and node and cached.get("device_id") == node:
            return cached
        if node is None:
            p = self._read_single_axis(axis_mask, DATA_POS_NORM, 0.06)
            if p is None:
                return {}
            node = int(p[1])
        info = self._read_node(node, DATA_AXIS_INFO, 0.06)
        if info is None:
            return {}
        self.axis_info[name] = dict(info[0])
        return self.axis_info[name]

    # -------------------------- force handling ------------------------------
    def _set_force_channels(
        self,
        name: str,
        display: Optional[float],
        source: str,
        movement_type: Optional[int],
        eng: Optional[float] = None,
        norm: Optional[float] = None,
        raw: Optional[float] = None,
    ) -> None:
        self.last_force_source[name] = source
        if name == "Pitch":
            self.last_pitch_force = display
            self.last_pitch_force_eng = eng
            self.last_pitch_force_norm = norm
            self.last_pitch_force_raw = raw
            self.last_pitch_movement_type = movement_type
        elif name == "Roll":
            self.last_roll_force = display
            self.last_roll_force_eng = eng
            self.last_roll_force_norm = norm
            self.last_roll_force_raw = raw
            self.last_roll_movement_type = movement_type
        else:
            self.last_rudder_force = display
            self.last_rudder_force_eng = eng
            self.last_rudder_force_norm = norm
            self.last_rudder_force_raw = raw
            self.last_rudder_movement_type = movement_type

    @staticmethod
    def _legacy_force_raw_signed16(value) -> Optional[int]:
        """Return legacy v474 0x20 force as a signed 16-bit count."""
        try:
            iv = int(round(float(value)))
        except (TypeError, ValueError, OverflowError):
            return None

        # Already-signed values are accepted so recorded replay frames remain
        # reusable after this compatibility layer has normalized the raw count.
        if -32768 <= iv <= 32767:
            return iv
        if 32768 <= iv <= 65535:
            return iv - 65536
        return None

    @classmethod
    def _legacy_force_pair_matches(cls, raw_value, norm_value) -> bool:
        """Validate the v474 relation: norm ~= signed16(raw) / 20000."""
        raw_signed = cls._legacy_force_raw_signed16(raw_value)
        if raw_signed is None:
            return False
        try:
            norm = float(norm_value)
        except (TypeError, ValueError, OverflowError):
            return False
        if not math.isfinite(norm) or abs(norm) > 1.5:
            return False

        norm_counts = norm * LEGACY_FORCE_NORM_FULL_SCALE_COUNTS
        tolerance = max(
            LEGACY_FORCE_MATCH_TOL_COUNTS,
            0.05 * abs(float(raw_signed)),
        )
        return abs(norm_counts - float(raw_signed)) <= tolerance

    def _validated_legacy_force_pair(self, node: int, norm, raw):
        """
        Confirm the legacy v474 force representation without guessing.

        The first pair is the normal capability-probe sample. If the axis was
        moving while the two sequential reads were taken, retry a few times and
        accept the legacy path only when 0x20 and 0x21 numerically agree.
        """
        candidates = [(norm, raw)]
        for _ in range(3):
            n = candidates[-1][0]
            r = candidates[-1][1]
            if (
                n is not None
                and r is not None
                and self._legacy_force_pair_matches(r[0], n[0])
            ):
                return n, r
            n = self._read_node(node, DATA_FORCE_NORM, 0.08)
            r = self._read_node(node, DATA_FORCE_RAW, 0.08)
            candidates.append((n, r))

        for n, r in candidates:
            if (
                n is not None
                and r is not None
                and self._legacy_force_pair_matches(r[0], n[0])
            ):
                return n, r
        return None

    def _probe_force_mode(self, axis_mask: int) -> Optional[str]:
        """Probe supported force Data IDs ONCE for the exact connected node."""
        name = self._axis_name_for_mask(axis_mask)
        node = self.detected_nodes.get(name)
        if node is None:
            p = self._read_single_axis(axis_mask, DATA_POS_NORM, 0.06)
            if p is None:
                self.force_probe_done[name] = True
                self.force_mode[name] = "unavailable"
                self.last_force_source[name] = "NO POSITION NODE"
                return None
            node = int(p[1])

        # The explicit BRUNNER Read Data list includes 0x21 and 0x20.
        norm = self._read_node(node, DATA_FORCE_NORM, 0.08)
        raw = self._read_node(node, DATA_FORCE_RAW, 0.08)

        # Optional capabilities listed in the Data table. Probe once, never spam.
        eng = self._read_node(node, DATA_FORCE_ENG, 0.08)
        info = self._read_node(node, DATA_AXIS_INFO, 0.08) if raw is not None else None
        if info is not None:
            native_info = dict(info[0])
            self.axis_info[name] = native_info
            self.force_conversion_info[name] = dict(native_info)

        support = []
        if norm is not None:
            support.append("0x21")
            self.last_force_wire_norm[name] = float(norm[0])
        if raw is not None:
            support.append("0x20")
            self.last_force_wire_raw[name] = float(raw[0])
        if eng is not None:
            support.append("0x22")
        if info is not None:
            support.append("0x23")
        self.force_support[name] = list(support)

        print(
            f"[CLS FORCE PROBE] {name}: node={node} supported="
            f"{','.join(support) if support else 'NONE'}",
            flush=True,
        )
        if not support:
            for did in (DATA_FORCE_NORM, DATA_FORCE_RAW, DATA_FORCE_ENG, DATA_AXIS_INFO):
                key = f"node{node}/data0x{did:X}"
                if key in self.last_read_error:
                    print(f"[CLS FORCE PROBE] {name} {key}: {self.last_read_error[key]}", flush=True)

        # Modern/v535 path: preserve the proven native engineering-force
        # behavior exactly whenever 0x22 is available.
        if eng is not None:
            mode = "eng22"
            self._set_force_channels(
                name, float(eng[0]), "0x22 ENG", eng[2], eng=float(eng[0])
            )

        elif raw is not None and info is not None:
            # Native documented RAW->engineering conversion via 0x23.
            factor = float(info[0].get("force_factor", 0.0))
            mt = info[0].get("movement_type")
            eng_val = float(raw[0]) * factor
            mode = "raw20-eng"
            self._set_force_channels(
                name, eng_val, "0x20 RAW->ENG", mt,
                eng=eng_val, raw=float(raw[0]),
                norm=float(norm[0]) if norm is not None else None,
            )

        elif raw is not None and norm is not None:
            # Legacy simulator-PC / firmware-v474 path. 0x22 and 0x23 are not
            # exposed, but the probe proved that 0x20 is a signed 16-bit count
            # returned inside the 32-bit payload and that 0x21 = count/20000.
            # Never enable this path unless the live pair validates that relation.
            legacy_pair = self._validated_legacy_force_pair(node, norm, raw)
            if legacy_pair is not None:
                legacy_norm, legacy_raw = legacy_pair
                self.last_force_wire_norm[name] = float(legacy_norm[0])
                self.last_force_wire_raw[name] = float(legacy_raw[0])
                raw_signed = self._legacy_force_raw_signed16(legacy_raw[0])
                assert raw_signed is not None
                eng_val = float(raw_signed) * LEGACY_FORCE_FACTOR_N_PER_COUNT
                legacy_info = {
                    "device_id": int(node),
                    "movement_type": None,
                    "force_factor": float(LEGACY_FORCE_FACTOR_N_PER_COUNT),
                    "force_limit": int(LEGACY_FORCE_NORM_FULL_SCALE_COUNTS),
                    "legacy_signed16": True,
                    "source": "v474 0x20/0x21 validated",
                }
                self.force_conversion_info[name] = legacy_info
                mode = "legacy16-eng"
                self._set_force_channels(
                    name,
                    eng_val,
                    "0x20 LEGACY16->ENG",
                    None,
                    eng=eng_val,
                    norm=float(legacy_norm[0]),
                    raw=float(raw_signed),
                )
                print(
                    f"[CLS FORCE LEGACY] {name}: VALIDATED | "
                    f"wireRaw={int(round(float(legacy_raw[0])))} -> "
                    f"signedRaw={raw_signed:+d} | "
                    f"norm={float(legacy_norm[0]):+.6f} | "
                    f"factor={LEGACY_FORCE_FACTOR_N_PER_COUNT:.3f} N/count | "
                    f"force={eng_val:+.3f} N",
                    flush=True,
                )
            else:
                mode = "norm21"
                self._set_force_channels(
                    name, float(norm[0]), "0x21 NORM", None,
                    norm=float(norm[0]), raw=float(raw[0]),
                )
                print(
                    f"[CLS FORCE LEGACY] {name}: 0x20/0x21 did NOT pass "
                    "the v474 consistency check; engineering conversion disabled.",
                    flush=True,
                )

        elif norm is not None:
            # Do NOT relabel normalized force as N/Nm.
            mode = "norm21"
            self._set_force_channels(
                name, float(norm[0]), "0x21 NORM", None, norm=float(norm[0]),
                raw=float(raw[0]) if raw is not None else None,
            )
        elif raw is not None:
            mode = "raw20"
            self._set_force_channels(
                name, float(raw[0]), "0x20 RAW", None, raw=float(raw[0])
            )
        else:
            mode = "unavailable"
            self._set_force_channels(name, None, "UNAVAILABLE", None)

        self.force_mode[name] = mode
        self.force_probe_done[name] = True
        try:
            self.force_probe_history.append({
                "wall_time": time.time(),
                "axis": name,
                "node": int(node),
                "supported": list(support),
                "mode": mode,
                "source": self.last_force_source.get(name),
                "wire_norm": (float(norm[0]) if norm is not None else None),
                "wire_raw": (float(raw[0]) if raw is not None else None),
                "eng22": (float(eng[0]) if eng is not None else None),
                "conversion": dict(self.force_conversion_info.get(name) or {}),
            })
        except Exception:
            pass
        return mode

    def _read_force_axis(self, axis_mask: int):
        name = self._axis_name_for_mask(axis_mask)
        node = self.detected_nodes.get(name)
        if node is None:
            p = self._read_single_axis(axis_mask, DATA_POS_NORM, 0.04)
            if p is None:
                return None
            node = int(p[1])

        if not self.force_probe_done[name]:
            self._probe_force_mode(axis_mask)

        mode = self.force_mode.get(name)
        if mode == "eng22":
            v = self._read_node(node, DATA_FORCE_ENG, 0.04)
            if v is not None:
                self._set_force_channels(
                    name, float(v[0]), "0x22 ENG", v[2], eng=float(v[0])
                )
                return self.last_force_source[name]

        elif mode == "raw20-eng":
            v = self._read_node(node, DATA_FORCE_RAW, 0.04)
            info = self.force_conversion_info.get(name) or self.axis_info.get(name) or {}
            if v is not None and info:
                raw_val = float(v[0])
                self.last_force_wire_raw[name] = raw_val
                eng_val = raw_val * float(info.get("force_factor", 0.0))
                self._set_force_channels(
                    name, eng_val, "0x20 RAW->ENG", info.get("movement_type"),
                    eng=eng_val, raw=raw_val,
                )
                return self.last_force_source[name]

        elif mode == "legacy16-eng":
            v = self._read_node(node, DATA_FORCE_RAW, 0.04)
            if v is not None:
                self.last_force_wire_raw[name] = float(v[0])
                raw_signed = self._legacy_force_raw_signed16(v[0])
                info = self.force_conversion_info.get(name) or {}
                if raw_signed is not None and info:
                    factor = float(info.get("force_factor", LEGACY_FORCE_FACTOR_N_PER_COUNT))
                    force_limit = float(info.get("force_limit", LEGACY_FORCE_NORM_FULL_SCALE_COUNTS))
                    eng_val = float(raw_signed) * factor
                    norm_val = (
                        float(raw_signed) / force_limit
                        if force_limit not in (0.0, -0.0)
                        else None
                    )
                    self._set_force_channels(
                        name,
                        eng_val,
                        "0x20 LEGACY16->ENG",
                        None,
                        eng=eng_val,
                        norm=norm_val,
                        raw=float(raw_signed),
                    )
                    return self.last_force_source[name]

        elif mode == "norm21":
            v = self._read_node(node, DATA_FORCE_NORM, 0.04)
            if v is not None:
                self.last_force_wire_norm[name] = float(v[0])
                self._set_force_channels(
                    name, float(v[0]), "0x21 NORM", None, norm=float(v[0])
                )
                return self.last_force_source[name]

        elif mode == "raw20":
            v = self._read_node(node, DATA_FORCE_RAW, 0.04)
            if v is not None:
                self.last_force_wire_raw[name] = float(v[0])
                self._set_force_channels(
                    name, float(v[0]), "0x20 RAW", None, raw=float(v[0])
                )
                return self.last_force_source[name]
        return None

    def force_capability_text(self) -> str:
        return " | ".join(
            f"{n}:{self.last_force_source.get(n, 'NA')}"
            for n in ("Pitch", "Roll", "Yaw")
        )

    # ------------------------- virtual force -------------------------------
    def enable_virtual_force_reading(self) -> None:
        """Enable Protocol-v2 virtual-force readout (setting 0xA0) on P/R/Y."""
        for axis, name in ((AX_ELEVATOR, "Pitch"), (AX_AILERON, "Roll"), (AX_RUDDER, "Yaw")):
            ok = self.send_with_ack(
                packet_setting_u8(axis, SID_VIRTUAL_FORCE_ENABLE, 1),
                f"{name} VirtualForceEnable=1",
                0.12,
            )
            if not ok:
                self.last_virtual_force_source[name] = "ENABLE FAILED"

    def _set_virtual_force_channels(self, name: str, source: str, raw=None, eng=None, movement_type=None) -> None:
        self.last_virtual_force_source[name] = source
        if name == "Pitch":
            self.last_pitch_virtual_force_raw = raw
            self.last_pitch_virtual_force_eng = eng
            self.last_pitch_virtual_movement_type = movement_type
        elif name == "Roll":
            self.last_roll_virtual_force_raw = raw
            self.last_roll_virtual_force_eng = eng
            self.last_roll_virtual_movement_type = movement_type
        else:
            self.last_rudder_virtual_force_raw = raw
            self.last_rudder_virtual_force_eng = eng
            self.last_rudder_virtual_movement_type = movement_type

    def _probe_virtual_force_mode(self, axis_mask: int) -> Optional[str]:
        name = self._axis_name_for_mask(axis_mask)
        node = self.detected_nodes.get(name)
        if node is None:
            p = self._read_single_axis(axis_mask, DATA_POS_NORM, 0.06)
            if p is None:
                self.virtual_force_probe_done[name] = True
                self.virtual_force_mode[name] = "unavailable"
                self.last_virtual_force_source[name] = "NO POSITION NODE"
                return None
            node = int(p[1])

        # 0x50 is in BRUNNER's explicit Read Data list. 0x51 is documented in
        # the Data-ID table as engineering-unit virtual force and is probed as
        # an optional capability for installed builds that support it.
        raw = self._read_node(node, DATA_VIRTUAL_FORCE_RAW, 0.08)
        eng = self._read_node(node, DATA_VIRTUAL_FORCE_ENG, 0.08)
        support = []
        if raw is not None:
            support.append("0x50")
        if eng is not None:
            support.append("0x51")
        print(
            f"[CLS VIRTUAL FORCE PROBE] {name}: node={node} supported="
            f"{','.join(support) if support else 'NONE'}",
            flush=True,
        )

        if eng is not None:
            mode = "eng51"
            self._set_virtual_force_channels(name, "0x51 ENG", eng=float(eng[0]), movement_type=eng[2], raw=float(raw[0]) if raw is not None else None)
        elif raw is not None:
            mode = "raw50"
            self._set_virtual_force_channels(name, "0x50 RAW", raw=float(raw[0]))
        else:
            mode = "unavailable"
            self._set_virtual_force_channels(name, "UNAVAILABLE")
        self.virtual_force_mode[name] = mode
        self.virtual_force_probe_done[name] = True
        return mode

    def _read_virtual_force_axis(self, axis_mask: int):
        name = self._axis_name_for_mask(axis_mask)
        node = self.detected_nodes.get(name)
        if node is None:
            return None
        if not self.virtual_force_probe_done[name]:
            self._probe_virtual_force_mode(axis_mask)
        mode = self.virtual_force_mode.get(name)
        if mode == "eng51":
            v = self._read_node(node, DATA_VIRTUAL_FORCE_ENG, 0.04)
            if v is not None:
                self._set_virtual_force_channels(name, "0x51 ENG", eng=float(v[0]), movement_type=v[2])
                return self.last_virtual_force_source[name]
        elif mode == "raw50":
            v = self._read_node(node, DATA_VIRTUAL_FORCE_RAW, 0.04)
            if v is not None:
                self._set_virtual_force_channels(name, "0x50 RAW", raw=float(v[0]))
                return self.last_virtual_force_source[name]
        return None

    # ---------------------- position/state diagnostics ----------------------
    def probe_axes(self, timeout_sec: float = 0.12) -> Dict[str, Any]:
        """
        Probe Pitch/Roll/Yaw independently, read-only.

        V14.62: a single missed UDP response must not silently reduce a healthy
        three-axis CLS-P installation to Pitch-only.  Each axis therefore gets
        three normal axis-mask attempts, followed by read-only node-specific
        position probes for the documented candidate node IDs.  An axis is
        declared unavailable only after all of those reads fail.
        """
        self.open()
        result: Dict[str, Any] = {}
        per_try_timeout = max(float(timeout_sec), 0.15)

        for name, axis, _key in CLS_AXIS_SPECS:
            sample = None
            probe_path = "axis-mask"

            for attempt in range(1, 4):
                sample = self._read_single_axis(
                    axis, DATA_POS_NORM, per_try_timeout
                )
                if sample is not None:
                    if attempt > 1:
                        print(
                            f"[CLS AXIS PROBE] {name}: axis-mask detected on "
                            f"attempt {attempt}/3 | node={int(sample[1])}",
                            flush=True,
                        )
                    break
                if attempt < 3:
                    time.sleep(0.02)

            # Some CLS2Sim/firmware combinations answer the physical node more
            # reliably than the generic axis-mask request.  This fallback is
            # READ ONLY (Data 0x11); it changes no controller state.
            if sample is None:
                for node_id in self.EXPECTED_DEVICE_IDS.get(axis, ()):
                    node_sample = self._read_node(
                        int(node_id), DATA_POS_NORM, per_try_timeout
                    )
                    if node_sample is not None:
                        sample = node_sample
                        self.detected_nodes[name] = int(node_sample[1])
                        probe_path = f"node-{int(node_sample[1])}"
                        print(
                            f"[CLS AXIS PROBE] {name}: recovered by "
                            f"node-specific read | node={int(node_sample[1])}",
                            flush=True,
                        )
                        break

            if sample is None:
                self.detected_nodes[name] = None
                if name == "Pitch":
                    self.last_pitch_pos = None
                    self.last_pitch_pos_raw = None
                    self.last_pitch_force = None
                    self.last_pitch_force_norm = None
                    self.last_pitch_force_raw = None
                    self.last_pitch_force_eng = None
                elif name == "Roll":
                    self.last_roll_pos = None
                    self.last_roll_pos_raw = None
                    self.last_roll_force = None
                    self.last_roll_force_norm = None
                    self.last_roll_force_raw = None
                    self.last_roll_force_eng = None
                else:
                    self.last_rudder_pos = None
                    self.last_rudder_pos_raw = None
                    self.last_rudder_force = None
                    self.last_rudder_force_norm = None
                    self.last_rudder_force_raw = None
                    self.last_rudder_force_eng = None

                result[name] = {
                    "ok": False,
                    "position": None,
                    "device_id": None,
                    "probe_path": "failed",
                }
            else:
                self.detected_nodes[name] = int(sample[1])
                result[name] = {
                    "ok": True,
                    "position": float(sample[0]),
                    "device_id": int(sample[1]),
                    "probe_path": probe_path,
                }

        self.refresh_static_axis_information()
        return result

    def detected_axis_mask(self) -> int:
        mask = AX_NONE
        for name, axis, _key in CLS_AXIS_SPECS:
            if self.detected_nodes.get(name) is not None:
                mask |= axis
        return mask

    def _poll_axis_specs(self):
        """
        Once at least one physical node has been positively detected, poll only
        those nodes. Before the first probe, poll all three so discovery works.
        """
        known = [
            (name, axis, key)
            for name, axis, key in CLS_AXIS_SPECS
            if self.detected_nodes.get(name) is not None
        ]
        return known if known else list(CLS_AXIS_SPECS)

    def get_status(self, axis_mask: int = AX_THREE_ACTUATORS) -> Dict[int, Dict[str, Any]]:
        with self._lock:
            if self._socket is None:
                return {}
            self._flush_rx_locked()
            try:
                self._socket.sendto(packet_state(axis_mask, STATE_GET_STATUS), self.address)
                deadline = time.monotonic() + 0.20
                while time.monotonic() < deadline:
                    try:
                        data, _ = self._socket.recvfrom(4096)
                    except socket.timeout:
                        continue
                    if len(data) < 3:
                        continue
                    if data[2] != 0:
                        return {}
                    out: Dict[int, Dict[str, Any]] = {}
                    offset = 3
                    while offset + 6 <= len(data):
                        dev, state, temp = struct.unpack_from("<HHH", data, offset)
                        out[int(dev)] = {
                            "state": int(state),
                            "state_name": DEVICE_STATE_NAMES.get(int(state), f"0x{state:04X}"),
                            "temperature_c": int(temp),
                        }
                        offset += 6
                    self.device_states = {k: v["state"] for k, v in out.items()}
                    self.device_temperatures = {k: v["temperature_c"] for k, v in out.items()}
                    return out
            except OSError as exc:
                self.last_socket_error = str(exc)
                return {}
        return {}

    def get_errors(self, axis_mask: int = AX_THREE_ACTUATORS) -> Dict[int, List[int]]:
        with self._lock:
            if self._socket is None:
                return {}
            self._flush_rx_locked()
            try:
                self._socket.sendto(packet_state(axis_mask, STATE_GET_ERRORS), self.address)
                deadline = time.monotonic() + 0.15
                all_errors: Dict[int, List[int]] = {}
                got_packet = False
                while time.monotonic() < deadline:
                    try:
                        data, _ = self._socket.recvfrom(4096)
                    except socket.timeout:
                        break
                    got_packet = True
                    if len(data) < 3 or data[2] != 0:
                        continue
                    offset = 3
                    while offset + 8 <= len(data):
                        dev, error_code, _timestamp = struct.unpack_from("<HHI", data, offset)
                        all_errors.setdefault(int(dev), []).append(int(error_code))
                        offset += 8
                self.device_errors = all_errors
                return all_errors if got_packet else {}
            except OSError as exc:
                self.last_socket_error = str(exc)
                return {}

    def verify_ap_enabled(self, axis_mask: int) -> Optional[bool]:
        # AP-enabled data is synchronized by CLS2Sim and can be exposed as one
        # value for the selected axis. Do not reject the response merely because
        # its returned device id is not the pilot node discovered from 0x11.
        sample = self._read_request(
            axis_mask,
            DATA_AP_ENABLED,
            expected_ids=None,
            timeout_sec=0.12,
            diagnostic_key=f"axis0x{axis_mask:X}/data0x{DATA_AP_ENABLED:X}",
        )
        if sample is None:
            return None
        return bool(int(round(sample[0])))

    def _set_raw_position(self, name: str, value: Optional[float]) -> None:
        if name == "Pitch":
            self.last_pitch_pos_raw = value
        elif name == "Roll":
            self.last_roll_pos_raw = value
        else:
            self.last_rudder_pos_raw = value

    def _set_ap_state(self, name: str, value: Optional[bool]) -> None:
        if name == "Pitch":
            self.last_pitch_ap_enabled = value
        elif name == "Roll":
            self.last_roll_ap_enabled = value
        else:
            self.last_rudder_ap_enabled = value

    def _update_health_cache(self) -> None:
        """Read drive health only for axes that are currently detected."""
        mask = self.detected_axis_mask()
        if mask == AX_NONE:
            return

        statuses = self.get_status(mask)
        errors = self.get_errors(mask)

        for name, _axis, _key in CLS_AXIS_SPECS:
            node = self.detected_nodes.get(name)
            if node is None:
                continue

            st = statuses.get(node, {})
            err_list = errors.get(node, [])

            state_val = st.get("state")
            temp_val = st.get("temperature_c")
            count = len(err_list)
            primary = int(err_list[0]) if err_list else None

            if name == "Pitch":
                self.last_pitch_drive_state = state_val
                self.last_pitch_temperature_c = temp_val
                self.last_pitch_error_count = count
                self.last_pitch_primary_error = primary
            elif name == "Roll":
                self.last_roll_drive_state = state_val
                self.last_roll_temperature_c = temp_val
                self.last_roll_error_count = count
                self.last_roll_primary_error = primary
            else:
                self.last_rudder_drive_state = state_val
                self.last_rudder_temperature_c = temp_val
                self.last_rudder_error_count = count
                self.last_rudder_primary_error = primary

    def refresh_static_axis_information(self) -> None:
        """Cache Axis Information only for physical nodes that are detected."""
        for name, axis, _key in CLS_AXIS_SPECS:
            if self.detected_nodes.get(name) is None:
                continue

            info = self._ensure_axis_info(axis)
            if not info:
                continue

            key = (name, info.get("device_id"))
            if key in self._static_axis_info_printed:
                continue

            self._static_axis_info_printed.add(key)
            print(
                "[CLS AXIS INFO] "
                f"{name}: node={info.get('device_id')} "
                f"movementType={info.get('movement_type')} "
                f"forceFactor={info.get('force_factor')} "
                f"positionFactor={info.get('pos_factor')} "
                f"forceLimit={info.get('force_limit')} "
                f"rawRange={info.get('neg_range_min')}..{info.get('pos_range_max')}",
                flush=True,
            )

    def connection_text(self) -> str:
        states = []
        for label, value in (
            ("P", self.last_pitch_pos),
            ("R", self.last_roll_pos),
            ("Y", self.last_rudder_pos),
        ):
            states.append(f"{label}{'✓' if value is not None else '✗'}")
        freshness = (
            "no UDP data" if self.last_rx_time is None
            else f"last rx {max(0.0, time.monotonic() - self.last_rx_time):.1f}s"
        )
        return f"CLS2Sim {self.address[0]}:{self.address[1]} | {' '.join(states)} | {freshness}"

    # ---------------------- independent background loops --------------------
    def _telemetry_loop(self, interval_sec: float) -> None:
        """
        Fast CLS telemetry.

        After V13 detects a partial hardware set, only connected axes are polled.
        Missing axes are never replaced by Prepar3D joystick data.
        """
        cycle = 0
        while not self._telemetry_stop.is_set():
            cycle_start = time.monotonic()
            try:
                specs = self._poll_axis_specs()

                for name, axis, _key in specs:
                    sample = self._read_single_axis(axis, DATA_POS_NORM, 0.020)
                    if sample is None:
                        continue
                    value = float(sample[0])
                    self.position_sample_times[name] = time.monotonic()
                    if name == "Pitch":
                        self.last_pitch_pos = value
                    elif name == "Roll":
                        self.last_roll_pos = value
                    else:
                        self.last_rudder_pos = value

                if cycle % 2 == 0:
                    for _name, axis, _key in specs:
                        self._read_force_axis(axis)

                cycle += 1
            except Exception as exc:
                self.last_socket_error = f"fast telemetry: {exc}"

            remain = interval_sec - (time.monotonic() - cycle_start)
            if remain > 0:
                self._telemetry_stop.wait(remain)

    def _medium_telemetry_loop(self, interval_sec: float) -> None:
        """Medium telemetry only for detected CLS axes."""
        while not self._medium_stop.is_set():
            cycle_start = time.monotonic()
            try:
                specs = [
                    spec for spec in CLS_AXIS_SPECS
                    if self.detected_nodes.get(spec[0]) is not None
                ]

                for name, axis, _key in specs:
                    node = self.detected_nodes.get(name)
                    if node is None:
                        continue

                    raw = self._read_node(node, DATA_POS_RAW, 0.025)
                    if raw is not None:
                        self._set_raw_position(name, float(raw[0]))

                    self._read_virtual_force_axis(axis)

                    # One-shot commissioning: on legacy v474, independently
                    # sample the real wire 0x21 value at medium rate so the CSV
                    # contains both wire RAW and wire NORM, not only derived data.
                    if self.force_mode.get(name) == "legacy16-eng":
                        # Read an explicit 0x21/0x20 pair in this same medium
                        # cycle. Using a close pair avoids falsely blaming the
                        # conversion when the control is moving quickly.
                        nwire = self._read_node(node, DATA_FORCE_NORM, 0.025)
                        rwire = self._read_node(node, DATA_FORCE_RAW, 0.025)
                        if nwire is not None:
                            self.last_force_wire_norm[name] = float(nwire[0])
                        if rwire is not None:
                            self.last_force_wire_raw[name] = float(rwire[0])
                        if nwire is not None and rwire is not None:
                            norm_wire = float(nwire[0])
                            raw_wire = float(rwire[0])
                            raw_signed = self._legacy_force_raw_signed16(raw_wire)
                            if raw_signed is not None:
                                err_counts = (
                                    norm_wire * LEGACY_FORCE_NORM_FULL_SCALE_COUNTS
                                    - float(raw_signed)
                                )
                                stats = self.legacy_force_validation[name]
                                stats["checks"] = int(stats.get("checks", 0)) + 1
                                stats["last_error_counts"] = float(err_counts)
                                stats["max_abs_error_counts"] = max(
                                    float(stats.get("max_abs_error_counts", 0.0)),
                                    abs(float(err_counts)),
                                )
                                tol = max(
                                    LEGACY_FORCE_MATCH_TOL_COUNTS,
                                    0.05 * abs(float(raw_signed)),
                                )
                                if abs(float(err_counts)) > tol:
                                    stats["failures"] = int(stats.get("failures", 0)) + 1

                    self._set_ap_state(name, self.verify_ap_enabled(axis))
            except Exception as exc:
                self.last_socket_error = f"medium telemetry: {exc}"

            remain = interval_sec - (time.monotonic() - cycle_start)
            if remain > 0:
                self._medium_stop.wait(remain)

    def _health_telemetry_loop(self, interval_sec: float) -> None:
        """
        SLOW CLS health monitoring (~1 Hz):
          - State Control drive status
          - drive temperature
          - current error codes
          - static Axis Information refresh if a node changes
        """
        while not self._health_stop.is_set():
            cycle_start = time.monotonic()
            try:
                self.refresh_static_axis_information()
                self._update_health_cache()
            except Exception as exc:
                self.last_socket_error = f"health telemetry: {exc}"

            remain = interval_sec - (time.monotonic() - cycle_start)
            if remain > 0:
                self._health_stop.wait(remain)

    def _watchdog_loop(self, interval_sec: float, timeout_ms: int) -> None:
        self._open_watchdog_socket()
        while not self._watchdog_stop.wait(interval_sec):
            if not self.watchdog_enabled:
                continue
            try:
                assert self._watchdog_socket is not None
                self._watchdog_socket.sendto(packet_watchdog(timeout_ms), self.address)
                # Response-to-sender is enabled in CLS2Sim. Drain its tiny ACK so
                # the dedicated watchdog socket never accumulates responses.
                try:
                    self._watchdog_socket.recvfrom(128)
                except socket.timeout:
                    pass
            except OSError as exc:
                self.last_socket_error = f"watchdog: {exc}"

    def start_read_only_polling(
        self,
        interval_sec: float = CLS_FAST_TELEMETRY_SEC,
    ) -> None:
        """
        Start CLS telemetry/status polling WITHOUT the Remote-Control watchdog.
        This is the V14.51 Manual path.
        """
        self.stop_watchdog()
        self.open()
        self.watchdog_enabled = False

        # V15.12.2 + VF:
        # Enable BRUNNER Protocol-v2 Virtual Force readout (Setting 0xA0 = 1)
        # before starting Manual telemetry. The medium telemetry loop already
        # reads 0x50/0x51 and stores both RAW and engineering-unit channels.
        self.enable_virtual_force_reading()

        fast_interval = max(0.015, float(interval_sec))
        self._telemetry_stop.clear()
        self._telemetry_thread = threading.Thread(
            target=self._telemetry_loop, args=(fast_interval,), daemon=True,
            name="CLSFastTelemetryThread",
        )
        self._telemetry_thread.start()

        self._medium_stop.clear()
        self._medium_thread = threading.Thread(
            target=self._medium_telemetry_loop, args=(CLS_MEDIUM_TELEMETRY_SEC,), daemon=True,
            name="CLSMediumTelemetryThread",
        )
        self._medium_thread.start()

        self._health_stop.clear()
        self._health_thread = threading.Thread(
            target=self._health_telemetry_loop, args=(CLS_HEALTH_TELEMETRY_SEC,), daemon=True,
            name="CLSHealthTelemetryThread",
        )
        self._health_thread.start()

        self._watchdog_stop.set()
        self._watchdog_thread = None
        print(
            "[CLS POLLING READ-ONLY] "
            f"FAST={fast_interval:.3f}s | FORCE~{fast_interval * 2.0:.3f}s | "
            f"MEDIUM={CLS_MEDIUM_TELEMETRY_SEC:.3f}s | "
            f"HEALTH={CLS_HEALTH_TELEMETRY_SEC:.3f}s | WATCHDOG=OFF",
            flush=True,
        )

    def start_watchdog(
        self,
        interval_sec: float = CLS_FAST_TELEMETRY_SEC,
        timeout_ms: int = CLS_WATCHDOG_MS,
    ) -> None:
        """
        Start four independent CLS runtime paths:

          FAST      : normalized positions + applied force
          MEDIUM    : raw positions + virtual force + AP state
          HEALTH    : drive state + temperature + errors + axis info
          WATCHDOG  : dedicated heartbeat socket

        The watchdog is never blocked by telemetry requests.
        """
        self.stop_watchdog()
        self.open()

        # V15.12.2 + VF:
        # Keep Virtual Force enabled during Automatic so the report compares
        # Manual 0x51 against live Automatic 0x51.
        self.enable_virtual_force_reading()

        fast_interval = max(0.015, float(interval_sec))

        self._telemetry_stop.clear()
        self._telemetry_thread = threading.Thread(
            target=self._telemetry_loop,
            args=(fast_interval,),
            daemon=True,
            name="CLSFastTelemetryThread",
        )
        self._telemetry_thread.start()

        self._medium_stop.clear()
        self._medium_thread = threading.Thread(
            target=self._medium_telemetry_loop,
            args=(CLS_MEDIUM_TELEMETRY_SEC,),
            daemon=True,
            name="CLSMediumTelemetryThread",
        )
        self._medium_thread.start()

        self._health_stop.clear()
        self._health_thread = threading.Thread(
            target=self._health_telemetry_loop,
            args=(CLS_HEALTH_TELEMETRY_SEC,),
            daemon=True,
            name="CLSHealthTelemetryThread",
        )
        self._health_thread.start()

        self._watchdog_stop.clear()
        self._watchdog_thread = threading.Thread(
            target=self._watchdog_loop,
            args=(CLS_WATCHDOG_HEARTBEAT_SEC, int(timeout_ms)),
            daemon=True,
            name="CLSWatchdogHeartbeatThread",
        )
        self._watchdog_thread.start()

        print(
            "[CLS POLLING] "
            f"FAST={fast_interval:.3f}s | "
            f"FORCE~{fast_interval * 2.0:.3f}s | "
            f"MEDIUM={CLS_MEDIUM_TELEMETRY_SEC:.3f}s | "
            f"HEALTH={CLS_HEALTH_TELEMETRY_SEC:.3f}s | "
            f"WATCHDOG={CLS_WATCHDOG_HEARTBEAT_SEC:.3f}s",
            flush=True,
        )

    def stop_watchdog(self) -> None:
        self._telemetry_stop.set()
        if self._telemetry_thread and self._telemetry_thread.is_alive():
            self._telemetry_thread.join(timeout=1.0)
        self._telemetry_thread = None

        self._medium_stop.set()
        if self._medium_thread and self._medium_thread.is_alive():
            self._medium_thread.join(timeout=1.0)
        self._medium_thread = None

        self._health_stop.set()
        if self._health_thread and self._health_thread.is_alive():
            self._health_thread.join(timeout=1.0)
        self._health_thread = None

        self._watchdog_stop.set()
        if self._watchdog_thread and self._watchdog_thread.is_alive():
            self._watchdog_thread.join(timeout=1.0)
        self._watchdog_thread = None

        if self._watchdog_socket is not None:
            try:
                self._watchdog_socket.close()
            except OSError:
                pass
            self._watchdog_socket = None

    def close(self) -> None:
        self.stop_watchdog()
        with self._lock:
            if self._socket is not None:
                try:
                    self._socket.close()
                except OSError:
                    pass
                self._socket = None
