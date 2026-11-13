"""
QTG Studio — Aircraft Position Setter (SimConnect write subprocess).

This module is launched as a SUBPROCESS by main.py when the user clicks
"Start Manual Recording" or "Run Automated Test".  It opens its own
SimConnect connection, pauses the sim, teleports the aircraft to the
required initial conditions, then unpauses the sim and exits cleanly.

Usage (called programmatically by main.py):
    python qtg_position_setter.py <alt_ft> <ias_kt> <on_ground 0|1> <heading_deg>

Exit codes:
    0  Success — aircraft positioned
    1  SimConnect.dll not found
    2  SimConnect_Open failed
    3  Timeout waiting for sim to stabilise
    4  Bad arguments
"""
from __future__ import annotations
import ctypes
import ctypes.wintypes
import math
import struct
import sys
import time
from pathlib import Path

# ── SimConnect & Bridge DLL location ─────────────────────────────────────────
import os

BASE_DIR = Path(sys.executable).resolve().parent if getattr(sys, "frozen", False) else Path(__file__).resolve().parent.parent
MEIPASS_DIR = Path(getattr(sys, "_MEIPASS", BASE_DIR))

BRIDGE_CANDIDATES = [
    BASE_DIR / "p3d_bridge.dll",
    MEIPASS_DIR / "p3d_bridge.dll",
    Path(__file__).resolve().parent.parent / "p3d_bridge.dll",
]

SIMCONNECT_CANDIDATES = [
    BASE_DIR / "bridge" / "bin" / "qtg_simconnect.dll",
    MEIPASS_DIR / "bridge" / "bin" / "qtg_simconnect.dll",
    Path(__file__).resolve().parent.parent / "bridge" / "bin" / "qtg_simconnect.dll",
    BASE_DIR / "SimConnect.dll",
    MEIPASS_DIR / "SimConnect.dll",
    Path(os.environ.get("WINDIR", "C:\\Windows")) / "System32" / "SimConnect.dll",
    Path(os.environ.get("WINDIR", "C:\\Windows")) / "SysWOW64" / "SimConnect.dll",
    Path(r"C:\Program Files\Lockheed Martin\Prepar3D v5\SimConnect.dll"),
    Path(r"C:\Program Files\Lockheed Martin\Prepar3D v4\SimConnect.dll"),
    Path(r"C:\Users\R&D Lab 4\Desktop\P3D sdk Files\SimConnect.dll"),
    Path(r"C:\Program Files (x86)\Microsoft Games\Microsoft Flight Simulator X\SDK\Core Utilities Kit\SimConnect SDK\lib\SimConnect.dll"),
]

# ── SimConnect constants ──────────────────────────────────────────────────────
SIMCONNECT_OBJECT_ID_USER   = 0
DEF_POSITION                = 10   # unique data-definition ID for write
EVT_SIM_PAUSE               = 20
EVT_SIM_UNPAUSE             = 21
SIMCONNECT_PERIOD_ONCE      = 1
SIMCONNECT_DATATYPE_INITPOSITION = 12  # SDK enum: 13 is MARKERSTATE, not INITPOSITION

OPEN_CONFIGINDEX_LOCAL      = 0

# SIMCONNECT_DATA_INITPOSITION fields (all float64 except OnGround/Airspeed)
class SIMCONNECT_DATA_INITPOSITION(ctypes.Structure):
    _pack_ = 8
    _fields_ = [
        ("Latitude",  ctypes.c_double),   # degrees
        ("Longitude", ctypes.c_double),   # degrees
        ("Altitude",  ctypes.c_double),   # feet MSL
        ("Pitch",     ctypes.c_double),   # degrees (nose up positive)
        ("Bank",      ctypes.c_double),   # degrees (right wing down positive)
        ("Heading",   ctypes.c_double),   # degrees true
        ("OnGround",  ctypes.c_uint32),   # 1 = on ground
        ("Airspeed",  ctypes.c_uint32),   # knots (0 if OnGround)
    ]

# ── Helpers ───────────────────────────────────────────────────────────────────
def find_bridge_dll() -> Path | None:
    for c in BRIDGE_CANDIDATES:
        if c.is_file():
            return c
    return None


def find_simconnect_dll() -> Path | None:
    for c in SIMCONNECT_CANDIDATES:
        if c.is_file():
            return c
    return None


def _log(msg: str):
    print(f"[qtg_position_setter] {msg}", flush=True)


def set_aircraft_position(alt_ft: float, ias_kt: float,
                          on_ground: bool = False,
                          heading_deg: float = 74.0,
                          lat: float = 32.235,
                          lon: float = 74.135) -> int:
    """
    Opens SimConnect / p3d_bridge, pauses the sim, sets the user aircraft to the
    specified initial conditions, waits briefly for the sim to settle,
    then exits.  Returns 0 on success, non-zero on error.
    """
    # ── Path A: Direct Native Bridge (fastest, most reliable) ─────────────
    bridge_path = find_bridge_dll()
    if bridge_path is not None:
        try:
            if hasattr(os, "add_dll_directory"):
                os.add_dll_directory(str(bridge_path.parent))
            bridge = ctypes.CDLL(str(bridge_path))
            if hasattr(bridge, "SetUserPosition"):
                bridge.SetUserPosition.argtypes = [ctypes.POINTER(SIMCONNECT_DATA_INITPOSITION)]
                bridge.SetUserPosition.restype = None
                if hasattr(bridge, "PauseSim"):
                    bridge.PauseSim.restype = None
                    bridge.PauseSim()
                pos = SIMCONNECT_DATA_INITPOSITION(
                    Latitude=float(lat),
                    Longitude=float(lon),
                    Altitude=float(alt_ft),
                    Pitch=0.0,
                    Bank=0.0,
                    Heading=float(heading_deg),
                    OnGround=1 if on_ground else 0,
                    Airspeed=0 if on_ground else int(round(float(ias_kt))),
                )
                bridge.SetUserPosition(ctypes.byref(pos))
                if hasattr(bridge, "SetThrottle"):
                    bridge.SetThrottle.argtypes = [ctypes.c_double]
                    bridge.SetThrottle.restype = None
                    bridge.SetThrottle(ctypes.c_double(100.0 if on_ground else 0.0))
                time.sleep(0.3)
                if hasattr(bridge, "UnpauseSim"):
                    bridge.UnpauseSim.restype = None
                    bridge.UnpauseSim()
                _log(f"Native bridge positioned aircraft: Alt={alt_ft}ft IAS={ias_kt}kt OnGround={on_ground}")
                return 0
        except Exception as bridge_err:
            _log(f"Note: bridge direct position attempt: {bridge_err}")

    # ── Path B: Direct SimConnect.dll fallback ─────────────────────────────
    dll_path = find_simconnect_dll()
    if dll_path is None:
        _log("ERROR: Neither p3d_bridge.dll nor SimConnect.dll found.")
        return 1

    _log(f"Loading SimConnect from {dll_path}")
    try:
        sc = ctypes.CDLL(str(dll_path))
    except OSError as e:
        _log(f"ERROR: Could not load SimConnect.dll: {e}")
        return 1

    # ── Declare required functions ─────────────────────────────────────────
    sc.SimConnect_Open.restype          = ctypes.c_int32   # HRESULT
    sc.SimConnect_Open.argtypes         = [
        ctypes.POINTER(ctypes.c_void_p),  # phSimConnect
        ctypes.c_char_p,                   # szName
        ctypes.c_void_p,                   # hWnd
        ctypes.c_uint32,                   # UserEventWin32
        ctypes.c_void_p,                   # hEventHandle
        ctypes.c_uint32,                   # ConfigIndex
    ]
    sc.SimConnect_Close.restype         = ctypes.c_int32
    sc.SimConnect_Close.argtypes        = [ctypes.c_void_p]
    sc.SimConnect_AddToDataDefinition.restype  = ctypes.c_int32
    sc.SimConnect_AddToDataDefinition.argtypes = [
        ctypes.c_void_p, ctypes.c_uint32,
        ctypes.c_char_p, ctypes.c_char_p,
        ctypes.c_uint32, ctypes.c_float, ctypes.c_uint32,
    ]
    sc.SimConnect_SetDataOnSimObject.restype  = ctypes.c_int32
    sc.SimConnect_SetDataOnSimObject.argtypes = [
        ctypes.c_void_p,  # hSimConnect
        ctypes.c_uint32,  # DefineID
        ctypes.c_uint32,  # ObjectID
        ctypes.c_uint32,  # Flags
        ctypes.c_uint32,  # ArrayCount
        ctypes.c_uint32,  # cbUnitSize
        ctypes.c_void_p,  # pDataSet
    ]
    sc.SimConnect_TransmitClientEvent.restype  = ctypes.c_int32
    sc.SimConnect_TransmitClientEvent.argtypes = [
        ctypes.c_void_p, ctypes.c_uint32, ctypes.c_uint32,
        ctypes.c_uint32, ctypes.c_uint32, ctypes.c_uint32,
    ]
    sc.SimConnect_MapClientEventToSimEvent.restype  = ctypes.c_int32
    sc.SimConnect_MapClientEventToSimEvent.argtypes = [
        ctypes.c_void_p, ctypes.c_uint32, ctypes.c_char_p,
    ]
    sc.SimConnect_CallDispatch.restype  = ctypes.c_int32
    sc.SimConnect_CallDispatch.argtypes = [
        ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p,
    ]

    # ── Open SimConnect ────────────────────────────────────────────────────
    hSim = ctypes.c_void_p(0)
    hr = sc.SimConnect_Open(
        ctypes.byref(hSim),
        b"QTG_PositionSetter",
        None, 0, None, OPEN_CONFIGINDEX_LOCAL
    )
    if hr != 0:
        _log(f"ERROR: SimConnect_Open failed HRESULT=0x{hr & 0xFFFFFFFF:08X}")
        return 2

    _log(f"SimConnect opened  hSim={hSim.value}")

    # Give SimConnect a moment to fully establish
    _pump(sc, hSim, 0.5)

    # ── Register INITPOSITION data definition ──────────────────────────────
    # Use the SDK INITPOSITION datatype (12).
    hr = sc.SimConnect_AddToDataDefinition(
        hSim, DEF_POSITION,
        b"Initial Position", None,
        SIMCONNECT_DATATYPE_INITPOSITION,
        0.0, ctypes.c_uint32(-1).value
    )
    if hr != 0:
        _log(f"ERROR: AddToDataDefinition failed HRESULT=0x{hr & 0xFFFFFFFF:08X}")
        sc.SimConnect_Close(hSim)
        return 2

    # ── Pause sim ─────────────────────────────────────────────────────────
    _log("Pausing sim before position set…")
    sc.SimConnect_MapClientEventToSimEvent(hSim, EVT_SIM_PAUSE,   b"PAUSE_SET")
    sc.SimConnect_MapClientEventToSimEvent(hSim, EVT_SIM_UNPAUSE, b"PAUSE_SET")
    # PAUSE_SET with dwData=1 pauses, =0 unpauses
    sc.SimConnect_TransmitClientEvent(
        hSim, SIMCONNECT_OBJECT_ID_USER, EVT_SIM_PAUSE,
        1,   # dwData = 1 → pause
        0x00000005,   # SIMCONNECT_GROUP_PRIORITY_HIGHEST
        0x00000010,   # SIMCONNECT_EVENT_FLAG_GROUPID_IS_PRIORITY
    )
    _pump(sc, hSim, 0.4)

    # ── Build INITPOSITION struct ──────────────────────────────────────────
    # Use a central European airfield lat/lon (can be any safe position)
    # The aircraft's last known position is kept; we only override alt/speed.
    # To do a full teleport we need to supply lat/lon too.
    # We use 0/0 lat/lon with a large altitude so the sim won't clip it into
    # terrain — or we keep the existing position by reading it first.
    # Safest: supply dummy lat/lon 48.0, 16.0 (Vienna area) if we don't know current.
    pos = SIMCONNECT_DATA_INITPOSITION()
    pos.Latitude  = 33.55     # degrees  (Islamabad / Pakistan area for context)
    pos.Longitude = 73.10     # degrees
    pos.Altitude  = float(alt_ft)
    pos.Pitch     = 0.0
    pos.Bank      = 0.0
    pos.Heading   = float(heading_deg)
    pos.OnGround  = 1 if on_ground else 0
    pos.Airspeed  = 0 if on_ground else int(ias_kt)

    _log(f"Setting position: Alt={alt_ft} ft  IAS={ias_kt} kt  OnGround={on_ground}  HDG={heading_deg}")
    hr = sc.SimConnect_SetDataOnSimObject(
        hSim, DEF_POSITION,
        SIMCONNECT_OBJECT_ID_USER,
        0,   # flags
        0,   # ArrayCount
        ctypes.sizeof(pos),
        ctypes.byref(pos)
    )
    if hr != 0:
        _log(f"WARNING: SetDataOnSimObject returned HRESULT=0x{hr & 0xFFFFFFFF:08X}  (may still work)")
    else:
        _log("SetDataOnSimObject succeeded.")

    # ── Wait for sim to settle ────────────────────────────────────────────
    _pump(sc, hSim, 1.2)

    # ── Unpause sim ───────────────────────────────────────────────────────
    _log("Unpausing sim…")
    sc.SimConnect_TransmitClientEvent(
        hSim, SIMCONNECT_OBJECT_ID_USER, EVT_SIM_UNPAUSE,
        0,   # dwData = 0 → unpause
        0x00000005,
        0x00000010,
    )
    _pump(sc, hSim, 0.5)

    sc.SimConnect_Close(hSim)
    _log("Done — aircraft positioned.  SimConnect closed.")
    return 0


def _pump(sc, hSim, duration_s: float):
    """Drain the SimConnect message queue for duration_s seconds."""
    t0 = time.monotonic()
    _DISPATCH_CB = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.c_uint32, ctypes.c_void_p)
    cb = _DISPATCH_CB(lambda msg, length, ctx: None)
    while time.monotonic() - t0 < duration_s:
        sc.SimConnect_CallDispatch(hSim, cb, None)
        time.sleep(0.02)



def _condition_metar(observation, wind_kt, temperature_c, altitude_ft, wind_direction):
    """Keep existing clouds/visibility/pressure; replace wind and temperature layers."""
    import re
    tokens = observation.split("@@@", 1)[0].split()
    if not tokens:
        raise RuntimeError("Simulator returned an empty weather observation")
    tokens[0] = "GLOB"
    wind = f"{int(round(wind_direction)) % 360:03d}{int(wind_kt):02d}KT&D9999NG"
    def temperature_token(value):
        return ("-" if value < 0 else "") + f"{abs(value):05.2f}".rstrip("0").rstrip(".")
    def temperature_layer(token):
        dewpoint = float(token.split("/", 1)[1].split("&", 1)[0].replace("M", "-"))
        return (f"{temperature_token(temperature_c)}/{temperature_token(min(dewpoint, temperature_c))}"
                f"&A{round(altitude_ft * 0.3048):05d}")
    wind_pattern = r"(?:[0-9]{3}|VRB)[0-9]+(?:G[0-9]+)?(?:KT|MPS|KMH).*"
    temperature_pattern = r"(?:M|-)?[0-9]+(?:\.[0-9]+)?/(?:M|-)?[0-9]+(?:\.[0-9]+)?.*"
    replaced_wind = replaced_temp = False
    result = []
    for token in tokens:
        if re.fullmatch(wind_pattern, token):
            if not replaced_wind: result.append(wind)
            replaced_wind = True
        elif re.fullmatch(temperature_pattern, token):
            if not replaced_temp: result.append(temperature_layer(token))
            replaced_temp = True
        else:
            result.append(token)
    if not replaced_wind or not replaced_temp:
        raise RuntimeError("Cannot safely update the simulator weather observation")
    return " ".join(result).encode("ascii")


def set_initial_conditions(*, alt_ft, ias_kt, on_ground, lat, lon, flaps,
                           throttle, wind_kt, temperature_c, was_paused=False):
    """Apply and read back conditions; raise on missing DLL, rejection or timeout.

    Uses one connection in both source and frozen builds. No subprocess or
    bridge-only success path may bypass weather or flap commands.
    """
    values = (alt_ft, ias_kt, lat, lon, flaps, throttle, wind_kt, temperature_c)
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("Initial conditions must be finite numbers")
    if not (-90 <= lat <= 90 and -180 <= lon <= 180):
        raise ValueError("Invalid latitude or longitude")
    if ias_kt < 0 or flaps < 0 or int(flaps) != flaps or not 0 <= throttle <= 100:
        raise ValueError("Invalid airspeed, flap index or throttle")
    if not 0 <= wind_kt <= 400 or wind_kt != int(wind_kt) or not -100 <= temperature_c <= 60:
        raise ValueError("Invalid wind speed or temperature")
    dll_path = find_simconnect_dll()
    if dll_path is None:
        raise RuntimeError("SimConnect client was not found. Run BUILD_SIMCONNECT_CLIENT.bat in the QTG Studio folder.")
    sc = ctypes.CDLL(str(dll_path))
    handle = ctypes.c_void_p()
    u32, ptr = ctypes.c_uint32, ctypes.c_void_p
    signatures = {
        "Open": [ctypes.POINTER(ptr), ctypes.c_char_p, ptr, u32, ptr, u32],
        "Close": [ptr], "AddToDataDefinition": [ptr, u32, ctypes.c_char_p, ctypes.c_char_p, u32, ctypes.c_float, u32],
        "SetDataOnSimObject": [ptr, u32, u32, u32, u32, u32, ptr],
        "MapClientEventToSimEvent": [ptr, u32, ctypes.c_char_p],
        "TransmitClientEvent": [ptr, u32, u32, u32, u32, u32],
        "RequestDataOnSimObject": [ptr, u32, u32, u32, u32, u32, u32, u32, u32],
        "CallDispatch": [ptr, ptr, ptr],
        "WeatherRequestObservationAtNearestStation": [ptr, u32, ctypes.c_float, ctypes.c_float],
        "WeatherSetModeGlobal": [ptr], "WeatherSetObservation": [ptr, u32, ctypes.c_char_p]}
    for name, signature in signatures.items():
        fn = getattr(sc, "SimConnect_" + name)
        fn.restype, fn.argtypes = ctypes.c_int32, signature
    def call(name, *args):
        hr = getattr(sc, "SimConnect_" + name)(*args)
        if hr < 0:
            raise RuntimeError(f"{name} failed (HRESULT 0x{hr & 0xffffffff:08X})")
    messages, errors = {}, []
    callback_type = ctypes.CFUNCTYPE(None, ptr, u32, ptr)
    def receive(address, size, context):
        raw = ctypes.string_at(address, size)
        if len(raw) < 12: return
        import struct
        kind = struct.unpack_from("<I", raw, 8)[0]
        if kind == 1 and len(raw) >= 24:
            errors.append(struct.unpack_from("<III", raw, 12))
        elif kind == 8 and len(raw) >= 40 + 11 * 8:
            request = struct.unpack_from("<I", raw, 12)[0]
            messages[request] = struct.unpack_from("<11d", raw, 40)
        elif kind == 10 and len(raw) > 16:
            messages[500] = raw[16:].split(b"\0", 1)[0].decode("ascii", errors="replace")
        elif kind == 3:
            errors.append(("Simulator closed",))
    callback = callback_type(receive)
    def pump_until(predicate, timeout=5):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            call("CallDispatch", handle, callback, None)
            if errors: raise RuntimeError(f"Simulator rejected initial conditions: {errors[0]}")
            if predicate(): return
            time.sleep(0.02)
        raise RuntimeError("Timed out waiting for simulator confirmation; some conditions may already have changed. " + str(messages.get("last_readback", "No readback received.")))
    def event(identifier, value):
        call("TransmitClientEvent", handle, 0, identifier, value, 1, 0x10)
    call("Open", ctypes.byref(handle), b"QTG_InitialConditions", None, 0, None, 0)
    paused_by_us = False
    try:
        state_fields = [(b"PLANE HEADING DEGREES TRUE", b"degrees"),
                        (b"AMBIENT WIND DIRECTION", b"degrees"),
                        (b"FLAPS NUM HANDLE POSITIONS", b"number"),
                        (b"PLANE LATITUDE", b"degrees"), (b"PLANE LONGITUDE", b"degrees"),
                        (b"PLANE ALTITUDE", b"feet"),
                        (b"GENERAL ENG THROTTLE LEVER POSITION:1", b"percent"),
                        (b"FLAPS HANDLE INDEX", b"number"),
                        (b"AMBIENT TEMPERATURE", b"celsius"),
                        (b"AMBIENT WIND VELOCITY", b"knots"), (b"AIRSPEED INDICATED", b"knots")]
        for name, unit in state_fields:
            call("AddToDataDefinition", handle, 100, name, unit, 4, 0.0, 0xffffffff)
        call("RequestDataOnSimObject", handle, 1, 100, 0, 1, 0, 0, 0, 0)
        call("WeatherRequestObservationAtNearestStation", handle, 500, lat, lon)
        pump_until(lambda: 1 in messages and 500 in messages)
        initial = messages[1]
        if not all(math.isfinite(value) for value in initial):
            raise RuntimeError("Simulator returned invalid initial state data")
        if flaps >= int(initial[2]):
            raise ValueError(f"This aircraft supports flap handle indices 0 through {int(initial[2])-1}")
        metar = _condition_metar(messages[500], wind_kt, temperature_c, alt_ft, initial[1])
        for identifier, name in ((200, b"PAUSE_SET"), (201, b"THROTTLE_SET"),
                                 (202, b"FLAPS_UP"), (203, b"FLAPS_INCR")):
            call("MapClientEventToSimEvent", handle, identifier, name)
        if not was_paused:
            event(200, 1)
            paused_by_us = True
        call("AddToDataDefinition", handle, 101, b"Initial Position", None, SIMCONNECT_DATATYPE_INITPOSITION, 0.0, 0xffffffff)
        position = SIMCONNECT_DATA_INITPOSITION(lat, lon, alt_ft, 0, 0, initial[0], int(on_ground),
                                               0 if on_ground else round(ias_kt))
        call("SetDataOnSimObject", handle, 101, 0, 0, 0, ctypes.sizeof(position), ctypes.byref(position))
        event(201, round(throttle * 16383 / 100))
        call("AddToDataDefinition", handle, 102, b"GENERAL ENG THROTTLE LEVER POSITION:1", b"percent", 4, 0.0, 0xffffffff)
        throttle_value = ctypes.c_double(throttle)
        call("SetDataOnSimObject", handle, 102, 0, 0, 0, ctypes.sizeof(throttle_value), ctypes.byref(throttle_value))
        event(202, 0)
        for _ in range(flaps): event(203, 0)
        call("WeatherSetModeGlobal", handle)
        call("WeatherSetObservation", handle, 0, metar)
        # Read back repeatedly: weather and controls can take time to settle.
        def confirmed():
            if 2 not in messages: return False
            state = messages.pop(2)
            messages["last_readback"] = dict(zip(["heading", "wind_direction", "flap_positions", "latitude", "longitude", "altitude_ft", "throttle_pct", "flap_index", "temperature_c", "wind_kt", "airspeed_kt"], state))
            call("RequestDataOnSimObject", handle, 2, 100, 0, 1, 0, 0, 0, 0)
            return (abs(state[3]-lat) < 0.0001 and abs(state[4]-lon) < 0.0001
                    and (on_ground or abs(state[5]-alt_ft) < 20)
                    and abs(state[6]-throttle) < 1 and round(state[7]) == flaps
                    and abs(state[8]-temperature_c) < 1.5 and abs(state[9]-wind_kt) < 1
                    and abs(state[10]-(0 if on_ground else ias_kt)) < 3)
        call("RequestDataOnSimObject", handle, 2, 100, 0, 1, 0, 0, 0, 0)
        pump_until(confirmed, timeout=8)
    finally:
        try:
            if paused_by_us: event(200, 0)
        finally:
            sc.SimConnect_Close(handle)


if __name__ == "__main__":
    if len(sys.argv) < 4:
        print("Usage: qtg_position_setter.py <alt_ft> <ias_kt> <on_ground 0|1> [heading_deg]", file=sys.stderr)
        sys.exit(4)
    try:
        alt_ft      = float(sys.argv[1])
        ias_kt      = float(sys.argv[2])
        on_ground   = sys.argv[3].strip() == "1"
        heading_deg = float(sys.argv[4]) if len(sys.argv) > 4 else 74.0
    except ValueError as e:
        print(f"Bad arguments: {e}", file=sys.stderr)
        sys.exit(4)

    sys.exit(set_aircraft_position(alt_ft, ias_kt, on_ground, heading_deg))

