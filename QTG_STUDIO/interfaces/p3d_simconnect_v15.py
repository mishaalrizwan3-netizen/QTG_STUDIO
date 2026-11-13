"""
Prepar3D SimConnect Client Module (V14.95 Final Deterministic Replay + Extended State)

Uses native 64-bit C++ bridge DLL (p3d_bridge.dll) linked against P3D SimConnect C-API.
Completely bypasses .NET/CLR version mismatch errors and COM thread apartment issues.
"""

import os
import sys
import time
import math
import ctypes
import threading
from typing import Optional, Dict, Any, Callable

# Telemetry Struct matching p3d_bridge.cpp (Pack = 1)
class TelemetryDataStruct(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("title", ctypes.c_char * 256),
        ("numberOfEngines", ctypes.c_double),
        ("airspeedIndicated", ctypes.c_double),
        ("verticalSpeed", ctypes.c_double),
        ("indicatedAltitude", ctypes.c_double),
        ("magneticHeading", ctypes.c_double),
        ("pitchAngle", ctypes.c_double),
        ("bankAngle", ctypes.c_double),
        ("trueHeading", ctypes.c_double),
        ("throttle1", ctypes.c_double),
        ("throttle2", ctypes.c_double),
        ("throttle3", ctypes.c_double),
        ("throttle4", ctypes.c_double),
        ("propRPM1", ctypes.c_double),
        ("propRPM2", ctypes.c_double),
        ("propRPM3", ctypes.c_double),
        ("propRPM4", ctypes.c_double),
        ("engRPM1", ctypes.c_double),
        ("engRPM2", ctypes.c_double),
        ("engRPM3", ctypes.c_double),
        ("engRPM4", ctypes.c_double),
        ("totalWeight", ctypes.c_double),
        ("totalFuelQuantity", ctypes.c_double),
        ("cgPercentLateral", ctypes.c_double),
        ("cgPercent", ctypes.c_double),
        ("baroPressure", ctypes.c_double),
        ("ambientTemp", ctypes.c_double),
        ("ambientWindDirection", ctypes.c_double),
        ("ambientWindVelocity", ctypes.c_double),
        ("groundSpeed", ctypes.c_double),
        ("machNumber", ctypes.c_double),
        ("pressureAlt", ctypes.c_double),
        ("radarAlt", ctypes.c_double),
        ("groundAltitude", ctypes.c_double),
        ("aoa", ctypes.c_double),
        ("sideslip", ctypes.c_double),
        ("trackAngle", ctypes.c_double),
        ("rollRate", ctypes.c_double),
        ("pitchRate", ctypes.c_double),
        ("yawRate", ctypes.c_double),
        ("accelX", ctypes.c_double),
        ("accelY", ctypes.c_double),
        ("accelZ", ctypes.c_double),
        ("rotorRPM", ctypes.c_double),
        ("fuelFlow", ctypes.c_double),
        ("torque", ctypes.c_double),
        ("egt", ctypes.c_double),
        # Control surface positions (for record & replay)
        ("elevatorPos", ctypes.c_double),   # ELEVATOR POSITION input, -1.0 to +1.0
        ("aileronPos",  ctypes.c_double),   # AILERON POSITION input,  -1.0 to +1.0
        ("rudderPos",   ctypes.c_double),   # RUDDER POSITION input,   -1.0 to +1.0
        ("elevatorTrim",ctypes.c_double),   # ELEVATOR TRIM POSITION, radians

        # V14.29 fields appended to preserve legacy layout above.
        ("velocityWorldX", ctypes.c_double),
        ("velocityWorldY", ctypes.c_double),
        ("velocityWorldZ", ctypes.c_double),
        ("velocityBodyX", ctypes.c_double),
        ("velocityBodyY", ctypes.c_double),
        ("velocityBodyZ", ctypes.c_double),
        ("rotationVelocityBodyX", ctypes.c_double),
        ("rotationVelocityBodyY", ctypes.c_double),
        ("rotationVelocityBodyZ", ctypes.c_double),
        ("elevatorDeflection", ctypes.c_double),
        ("aileronLeftDeflection", ctypes.c_double),
        ("aileronRightDeflection", ctypes.c_double),
        ("rudderDeflection", ctypes.c_double),
        ("elevatorTrimPct", ctypes.c_double),
        ("aileronTrimPct", ctypes.c_double),
        ("rudderTrimPct", ctypes.c_double),
        ("planeLatitude", ctypes.c_double),
        ("planeLongitude", ctypes.c_double),
        ("mixtureLever1", ctypes.c_double),
        ("propellerLever1", ctypes.c_double),
        ("engineCombustion1", ctypes.c_double),
        ("engineFuelValve1", ctypes.c_double),
        ("engineFuelPressure1", ctypes.c_double),
        ("manifoldPressure1", ctypes.c_double),
        ("relativeWindBodyX", ctypes.c_double),
        ("relativeWindBodyY", ctypes.c_double),
        ("relativeWindBodyZ", ctypes.c_double),
        # V14.92 causal-state coverage.
        ("flapsHandlePercent", ctypes.c_double),
        ("flapsHandleIndex", ctypes.c_double),
        ("flapsNumHandlePositions", ctypes.c_double),
        ("trailingEdgeFlapsLeftPercent", ctypes.c_double),
        ("trailingEdgeFlapsRightPercent", ctypes.c_double),
        ("trailingEdgeFlapsLeftAngle", ctypes.c_double),
        ("trailingEdgeFlapsRightAngle", ctypes.c_double),
        ("ambientDensity", ctypes.c_double),
        ("ambientPressure", ctypes.c_double),
        ("aircraftWindX", ctypes.c_double),
        ("aircraftWindY", ctypes.c_double),
        ("aircraftWindZ", ctypes.c_double),
        ("rotationAccelerationBodyX", ctypes.c_double),
        ("rotationAccelerationBodyY", ctypes.c_double),
        ("rotationAccelerationBodyZ", ctypes.c_double),
        ("fuelTankLeftMainLevel", ctypes.c_double),
        ("fuelTankRightMainLevel", ctypes.c_double),
        ("fuelTankCenterLevel", ctypes.c_double),
        ("fuelTankSelector1", ctypes.c_double),
        ("autopilotMaster", ctypes.c_double),
        ("autoCoordination", ctypes.c_double),
        ("spoilersHandlePosition", ctypes.c_double),
        ("gearHandlePosition", ctypes.c_double),
        ("brakeParkingPosition", ctypes.c_double),
    ]


V1495_MAX_PAYLOAD_STATIONS = 32
V1495_FUEL_TANK_NAMES = [
    "center", "center2", "center3", "left_main", "left_aux", "left_tip",
    "right_main", "right_aux", "right_tip", "external1", "external2",
]

class EngineSupportStateStructV1496(ctypes.Structure):
    _pack_ = 1
    _fields_ = [("values", ctypes.c_double * 8)]

ENGINE_SUPPORT_KEYS_V1496 = ('engine_oil_temperature_c_1', 'engine_cylinder_head_temperature_c_1', 'engine_carburetor_temperature_c_1', 'electrical_master_battery', 'engine_master_alternator_1', 'general_eng_fuel_pump_switch_1', 'engine_fuel_valve_1', 'general_eng_starter_1')

class ExtendedCausalStateStructV1495(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("payloadStationCount", ctypes.c_double),
        ("payloadStationWeight", ctypes.c_double * V1495_MAX_PAYLOAD_STATIONS),
        ("totalWeightPitchMoi", ctypes.c_double),
        ("totalWeightRollMoi", ctypes.c_double),
        ("totalWeightYawMoi", ctypes.c_double),
        ("totalWeightCrossCoupledMoi", ctypes.c_double),
        ("fuelTankLevel", ctypes.c_double * 11),
        ("fuelTankCapacity", ctypes.c_double * 11),
        ("fuelTankSelector1", ctypes.c_double),
        ("fuelTankSelector2", ctypes.c_double),
        ("fuelTankSelector3", ctypes.c_double),
        ("fuelTankSelector4", ctypes.c_double),
        ("recipCowlFlapPosition1", ctypes.c_double),
        ("recipPrimer1", ctypes.c_double),
        ("recipAlternateAirPosition1", ctypes.c_double),
        ("recipLeftMagneto1", ctypes.c_double),
        ("recipRightMagneto1", ctypes.c_double),
        ("recipFuelAvailable1", ctypes.c_double),
        ("recipFuelTankSelector1", ctypes.c_double),
        ("recipFuelTanksUsed1", ctypes.c_double),
        ("generalEngStarter1", ctypes.c_double),
        ("generalEngFuelPumpSwitch1", ctypes.c_double),
        ("generalEngFuelPumpOn1", ctypes.c_double),
        ("generalEngDamagePercent1", ctypes.c_double),
        ("generalEngFailed1", ctypes.c_double),
        ("generalEngAntiIcePosition1", ctypes.c_double),
        ("propDeiceSwitch1", ctypes.c_double),
        ("engOnFire1", ctypes.c_double),
        ("structuralIcePct", ctypes.c_double),
        ("pitotIcePct", ctypes.c_double),
        ("pitotHeat", ctypes.c_double),
        ("flapDamageBySpeed", ctypes.c_double),
        ("flapSpeedExceeded", ctypes.c_double),
        ("gearDamageBySpeed", ctypes.c_double),
        ("gearSpeedExceeded", ctypes.c_double),
        ("spoilersLeftPosition", ctypes.c_double),
        ("spoilersRightPosition", ctypes.c_double),
        ("gearCenterPosition", ctypes.c_double),
        ("gearLeftPosition", ctypes.c_double),
        ("gearRightPosition", ctypes.c_double),
        ("canopyOpen", ctypes.c_double),
        ("damageState", ctypes.c_double),
        ("simOnGround", ctypes.c_double),
        ("userInputEnabled", ctypes.c_double),
        ("gyroHeadingRad", ctypes.c_double),
        ("gyroDriftErrorRad", ctypes.c_double),
    ]


class InitialDynamicStateStruct(ctypes.Structure):
    """Must match InitialDynamicState in p3d_bridge_v14_30_backward_compatible.cpp."""
    _pack_ = 1
    _fields_ = [
        ("velocityWorldX", ctypes.c_double),
        ("velocityWorldY", ctypes.c_double),
        ("velocityWorldZ", ctypes.c_double),
        ("velocityBodyX", ctypes.c_double),
        ("velocityBodyY", ctypes.c_double),
        ("velocityBodyZ", ctypes.c_double),
        ("verticalSpeedFps", ctypes.c_double),
        ("rotationVelocityBodyX", ctypes.c_double),
        ("rotationVelocityBodyY", ctypes.c_double),
        ("rotationVelocityBodyZ", ctypes.c_double),
        ("elevatorTrim", ctypes.c_double),
        ("aileronTrimPct", ctypes.c_double),
        ("rudderTrimPct", ctypes.c_double),
        ("propRpm1", ctypes.c_double),
    ]

# SIMCONNECT_DATA_INITPOSITION struct for teleporting the user aircraft
class ExactPoseStateStruct(ctypes.Structure):
    """Must match ExactPoseState in p3d_bridge_v14_40_direct_pose.cpp."""
    _pack_ = 1
    _fields_ = [
        ("latitudeRad", ctypes.c_double),
        ("longitudeRad", ctypes.c_double),
        ("altitudeFt", ctypes.c_double),
        ("pitchRad", ctypes.c_double),
        ("bankRad", ctypes.c_double),
        ("headingTrueRad", ctypes.c_double),
        ("airspeedIndicatedKts", ctypes.c_double),
    ]


class ExactPoseNoAirspeedStateStruct(ctypes.Structure):
    """Must match ExactPoseNoAirspeedState in p3d_bridge_v14_66_no_ias_final_pose.cpp."""
    _pack_ = 1
    _fields_ = [
        ("latitudeRad", ctypes.c_double),
        ("longitudeRad", ctypes.c_double),
        ("altitudeFt", ctypes.c_double),
        ("pitchRad", ctypes.c_double),
        ("bankRad", ctypes.c_double),
        ("headingTrueRad", ctypes.c_double),
    ]


class InitPositionStruct(ctypes.Structure):
    _pack_ = 8
    _fields_ = [
        ("Latitude",  ctypes.c_double),  # degrees
        ("Longitude", ctypes.c_double),  # degrees
        ("Altitude",  ctypes.c_double),  # feet MSL
        ("Pitch",     ctypes.c_double),  # degrees
        ("Bank",      ctypes.c_double),  # degrees
        ("Heading",   ctypes.c_double),  # degrees true
        ("OnGround",  ctypes.c_uint),    # 1 = on ground
        ("Airspeed",  ctypes.c_uint),    # kts (ignored if OnGround=1)
    ]


class SimConnectClient:
    """
    Thread-safe Prepar3D SimConnect Client using Native C++ Bridge.
    """

    def __init__(self, app_name: str = "Piston Engine Data Logger"):
        import sys
        candidate_paths = [
            os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "p3d_bridge.dll")),
            os.path.abspath(os.path.join(getattr(sys, '_MEIPASS', ''), "p3d_bridge.dll")),
            os.path.abspath(os.path.join(os.path.dirname(sys.executable), "p3d_bridge.dll")),
            os.path.abspath(os.path.join(os.path.dirname(sys.executable), "_internal", "p3d_bridge.dll")),
        ]
        self._dll_path = None
        for p in candidate_paths:
            if p and os.path.exists(p):
                self._dll_path = p
                break
        
        if not self._dll_path:
            raise FileNotFoundError(f"Native bridge DLL not found in any candidate path: {candidate_paths}")

        self._bridge = ctypes.CDLL(self._dll_path)

        # V14.30 requires the full-layout V14.30 export while keeping legacy DLL callers safe. The telemetry
        # structure was extended.  Refuse startup rather than reading misaligned data.
        if not hasattr(self._bridge, "GetBridgeVersion"):
            raise RuntimeError(
                "V14.30 requires the rebuilt p3d_bridge.dll. "
                "Current DLL has no GetBridgeVersion export."
            )
        self._bridge.GetBridgeVersion.restype = ctypes.c_int
        self.bridge_version = int(self._bridge.GetBridgeVersion())
        if self.bridge_version < 1498:
            raise RuntimeError(
                f"V14.95 requires compatible p3d_bridge.dll version >=1498; found {self.bridge_version}. "
                "Build and use the patched p3d_bridge_v15_full_causal_state.cpp as p3d_bridge.dll."
            )

        # Setup C function signatures
        self._bridge.ConnectSim.restype = ctypes.c_int
        self._bridge.IsSimConnected.restype = ctypes.c_int
        if not hasattr(self._bridge, "GetTelemetryDataV1492"):
            raise RuntimeError("V14.95 DLL is missing GetTelemetryDataV1492 export.")
        self._bridge.GetTelemetryDataV1492.argtypes = [ctypes.POINTER(TelemetryDataStruct)]
        self._bridge.GetTelemetryDataV1492.restype = None
        for _name in ("GetExtendedCausalStateV1495", "GetExtendedCausalSequenceV1495",
                      "RequestExtendedCausalOnceV1495", "SetPayloadStationWeightsV1495",
                      "SetAllFuelTankLevelsV1495", "SetExtendedAircraftConfigurationV1495"):
            if not hasattr(self._bridge, _name):
                raise RuntimeError(f"V14.95 DLL is missing {_name} export.")
        self._bridge.GetExtendedCausalStateV1495.argtypes = [ctypes.POINTER(ExtendedCausalStateStructV1495)]
        self._bridge.GetExtendedCausalStateV1495.restype = None
        self._bridge.GetExtendedCausalSequenceV1495.restype = ctypes.c_long
        self._bridge.RequestExtendedCausalOnceV1495.restype = ctypes.c_int
        self._bridge.SetPayloadStationWeightsV1495.argtypes = [ctypes.POINTER(ctypes.c_double), ctypes.c_int]
        self._bridge.SetPayloadStationWeightsV1495.restype = ctypes.c_int
        self._bridge.SetAllFuelTankLevelsV1495.argtypes = [ctypes.POINTER(ctypes.c_double), ctypes.c_int]
        self._bridge.SetAllFuelTankLevelsV1495.restype = ctypes.c_int
        self._bridge.SetExtendedAircraftConfigurationV1495.argtypes = [ctypes.c_double] * 15
        self._bridge.SetExtendedAircraftConfigurationV1495.restype = ctypes.c_int
        if not hasattr(self._bridge, "GetTelemetrySequence") or not hasattr(self._bridge, "RequestTelemetryOnce"):
            raise RuntimeError("V14.30 DLL is missing telemetry refresh/sequence exports.")
        self._bridge.GetTelemetrySequence.restype = ctypes.c_long
        self._bridge.RequestTelemetryOnce.restype = ctypes.c_int
        self._bridge.DisconnectSim.restype = None
        self._bridge.ProcessMessages.restype = None
        self._bridge.PauseSim.restype = None
        self._bridge.UnpauseSim.restype = None
        self._bridge.SetDaytime.restype = None
        self._bridge.SetUserPosition.restype = None
        self._bridge.SetUserPosition.argtypes = [ctypes.POINTER(InitPositionStruct)]
        self._bridge.QueryAirportByICAO.restype = None
        self._bridge.QueryAirportByICAO.argtypes = [ctypes.c_char_p]
        self._bridge.GetAirportPosition.restype = ctypes.c_int
        self._bridge.GetAirportPosition.argtypes = [
            ctypes.POINTER(ctypes.c_double),
            ctypes.POINTER(ctypes.c_double),
            ctypes.POINTER(ctypes.c_double),
        ]
        # Control replay functions
        self._bridge.SetElevator.restype = None
        self._bridge.SetElevator.argtypes = [ctypes.c_double]
        self._bridge.SetAileron.restype = None
        self._bridge.SetAileron.argtypes = [ctypes.c_double]
        self._bridge.SetRudder.restype = None
        self._bridge.SetRudder.argtypes = [ctypes.c_double]
        self._bridge.SetElevatorTrim.restype = None
        self._bridge.SetElevatorTrim.argtypes = [ctypes.c_double]
        self._bridge.SetThrottle.restype = None
        self._bridge.SetThrottle.argtypes = [ctypes.c_double]
        if hasattr(self._bridge, "SetFlaps"):
            self._bridge.SetFlaps.restype = None
            self._bridge.SetFlaps.argtypes = [ctypes.c_double]
        for _name, _args in (
            ("SetEngineControls", [ctypes.c_double, ctypes.c_double]),
            ("SetFlapsHandleIndex", [ctypes.c_double]),
            ("SetFlapSurfaceState", [ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_double]),
            ("SetFuelTankLevels", [ctypes.c_double, ctypes.c_double, ctypes.c_double]),
            ("SetLinearBodyAcceleration", [ctypes.c_double, ctypes.c_double, ctypes.c_double]),
            ("SetRotationBodyAcceleration", [ctypes.c_double, ctypes.c_double, ctypes.c_double]),
            ("SetAuxConfiguration", [ctypes.c_double] * 7),
        ):
            if not hasattr(self._bridge, _name):
                raise RuntimeError(f"V14.95 DLL is missing {_name} export.")
            _fn = getattr(self._bridge, _name)
            _fn.restype = ctypes.c_int
            _fn.argtypes = _args
        if not hasattr(self._bridge, "SetGlobalWeatherMETAR"):
            raise RuntimeError("V14.95 DLL is missing SetGlobalWeatherMETAR export.")
        self._bridge.SetGlobalWeatherMETAR.restype = ctypes.c_int
        self._bridge.SetGlobalWeatherMETAR.argtypes = [ctypes.c_char_p]
        for _name in ("GetWeatherSequence", "RequestGlobalWeatherMETAR", "GetLastWeatherMETAR"):
            if not hasattr(self._bridge, _name):
                raise RuntimeError(f"V14.95 DLL is missing {_name} export.")
        self._bridge.GetWeatherSequence.restype = ctypes.c_long
        self._bridge.RequestGlobalWeatherMETAR.restype = ctypes.c_int
        self._bridge.GetLastWeatherMETAR.restype = ctypes.c_int
        self._bridge.GetLastWeatherMETAR.argtypes = [ctypes.c_void_p, ctypes.c_int]
        self._bridge.SetControlInputs.restype = None
        self._bridge.SetControlInputs.argtypes = [
            ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_double
        ]
        if not hasattr(self._bridge, "SetInitialDynamicState"):
            raise RuntimeError("V14.30 DLL is missing SetInitialDynamicState export.")
        self._bridge.SetInitialDynamicState.restype = ctypes.c_int
        self._bridge.SetInitialDynamicState.argtypes = [ctypes.POINTER(InitialDynamicStateStruct)]

        self._bridge.SetCausalFieldV1497.argtypes = [ctypes.c_int, ctypes.c_double]
        self._bridge.SetCausalFieldV1497.restype = ctypes.c_int
        self.natural_engine_mode = False
        required_engine_exports = {
            "SetWorldInitialDynamicStateV15": ([ctypes.POINTER(InitialDynamicStateStruct)], ctypes.c_int),
            "SetInitialFuelPressureV15": ([ctypes.c_double], ctypes.c_int),
            "SetInitialDynamicStateNoRpmV1496": ([ctypes.POINTER(InitialDynamicStateStruct)], ctypes.c_int),
            "SetAuxConfigurationNoCombustionV1496": ([ctypes.c_double] * 7, ctypes.c_int),
            "GetEngineSupportSequenceV1496": ([], ctypes.c_long),
            "RequestEngineSupportOnceV1496": ([], ctypes.c_int),
            "GetEngineSupportStateV1496": ([ctypes.POINTER(EngineSupportStateStructV1496)], None),
            "SetEngineSupportValueV1496": ([ctypes.c_int, ctypes.c_double], ctypes.c_int),
            "SendEnginePreparationEventV1496": ([ctypes.c_int], ctypes.c_int),
        }
        for name, (args, result) in required_engine_exports.items():
            if not hasattr(self._bridge, name):
                raise RuntimeError("Engine initialization fix requires rebuilt p3d_bridge.dll: missing " + name)
            func = getattr(self._bridge, name)
            func.argtypes, func.restype = args, result

        if not hasattr(self._bridge, "SetExactPoseState"):
            raise RuntimeError("V14.40 DLL is missing SetExactPoseState export.")
        self._bridge.SetExactPoseState.restype = ctypes.c_int
        self._bridge.SetExactPoseState.argtypes = [ctypes.POINTER(ExactPoseStateStruct)]

        if not hasattr(self._bridge, "SetExactPoseStateNoAirspeed"):
            raise RuntimeError(
                "V14.66 DLL is missing SetExactPoseStateNoAirspeed export. "
                "The old IAS-writing final-pose path is intentionally not accepted."
            )
        self._bridge.SetExactPoseStateNoAirspeed.restype = ctypes.c_int
        self._bridge.SetExactPoseStateNoAirspeed.argtypes = [
            ctypes.POINTER(ExactPoseNoAirspeedStateStruct)
        ]

        self._lock = threading.Lock()
        self._snapshot_lock = threading.RLock()
        self._stop_requested = threading.Event()
        self._connected = False
        self._latest_data: Dict[str, Any] = self._empty_telemetry()
        self.last_error: str = ""

        # FSR Playback State Tracking
        self._playback_state = "idle"
        self._playback_start_time = 0.0
        self._playback_file = ""

    def get_playback_state(self) -> str:
        """Return the current playback state ('idle', 'pending', 'playing', 'completed', 'stopped')."""
        with self._lock:
            return self._playback_state

    def reset_playback_state(self) -> bool:
        """Reset playback state back to idle if not currently playing."""
        with self._lock:
            if self._playback_state in ("pending", "playing"):
                return False
            self._playback_state = "idle"
            self._playback_start_time = 0.0
            return True

    def play_recording(self, full_path: str) -> bool:
        """Start playing an FSR recording in Prepar3D and track playback timing."""
        with self._lock:
            self._playback_file = full_path
            self._playback_state = "playing"
            self._playback_start_time = time.monotonic()

        if hasattr(self._bridge, "PlayFlightRecording"):
            try:
                filename_bytes = os.path.abspath(full_path).encode("utf-8")
                self._bridge.PlayFlightRecording(filename_bytes)
            except Exception as e:
                print(f"[SimConnect Native] PlayFlightRecording error: {e}", flush=True)

        return True

    def get_playback_elapsed_seconds(self) -> float:
        """Return total elapsed time in seconds since playback started."""
        with self._lock:
            if not self._playback_start_time or self._playback_state != "playing":
                return 0.0
            return max(0.0, time.monotonic() - self._playback_start_time)

    def pause_sim(self):
        """Send pause command to Prepar3D."""
        try:
            self._bridge.PauseSim()
            print("[SimConnect Native] Transmitted pause command to Prepar3D.", flush=True)
        except Exception as e:
            print(f"[SimConnect Native] Error sending pause: {e}", flush=True)

    def unpause_sim(self):
        """Send unpause command to Prepar3D to ensure simulation is active."""
        try:
            self._bridge.UnpauseSim()
            print("[SimConnect Native] Transmitted unpause command to Prepar3D.", flush=True)
        except Exception as e:
            print(f"[SimConnect Native] Error sending unpause: {e}", flush=True)

    def get_telemetry_sequence(self) -> int:
        try:
            return int(self._bridge.GetTelemetrySequence())
        except Exception:
            return -1

    def request_telemetry_once(self) -> bool:
        """Request one fresh SimConnect telemetry packet, including while paused."""
        try:
            return bool(self._bridge.RequestTelemetryOnce())
        except Exception as e:
            print(f"[SimConnect V14.30] RequestTelemetryOnce error: {e}", flush=True)
            return False

    def wait_for_fresh_telemetry(self, previous_sequence: int, timeout_sec: float = 1.0) -> Optional[Dict[str, Any]]:
        """Wait for a newer bridge packet, then synchronously copy that packet into Python."""
        deadline = time.monotonic() + max(0.05, float(timeout_sec))
        while time.monotonic() < deadline:
            if self.get_telemetry_sequence() > int(previous_sequence):
                snapshot = self._read_primary_snapshot()
                if snapshot is not None:
                    raw_data, sequence = snapshot
                    if sequence > int(previous_sequence):
                        self._update_latest_data(raw_data, packet_sequence=sequence)
                        result = self.get_telemetry()
                        if int(result.get("bridge_telemetry_sequence", 0)) > int(previous_sequence):
                            return result
            time.sleep(0.01)
        return None

    def queue_position_airborne(
        self,
        alt_ft: float,
        speed_kts: float,
        pitch_deg: float,
        bank_deg: float,
        hdg_deg: float,
        lat: float,
        lon: float,
        throttle: float = 80.0,
    ) -> bool:
        """Queue INITPOSITION while paused. Caller controls the unpause/commit sequence."""
        try:
            pos = InitPositionStruct(
                Latitude=float(lat), Longitude=float(lon), Altitude=float(alt_ft),
                Pitch=float(pitch_deg), Bank=float(bank_deg), Heading=float(hdg_deg),
                OnGround=0, Airspeed=int(round(float(speed_kts))),
            )
            self._bridge.SetUserPosition(ctypes.byref(pos))
            self.set_flaps(0.0)
            self.set_throttle(float(throttle))
            print(
                f"[SimConnect V14.30] INITPOSITION queued: Lat={lat:.6f}, Lon={lon:.6f}, "
                f"Alt={alt_ft:.1f}ft IAS={speed_kts:.1f}kt Pitch={pitch_deg:.2f} Bank={bank_deg:.2f} "
                f"TrueHdg={hdg_deg:.2f} Throttle={throttle:.1f}%",
                flush=True,
            )
            return True
        except Exception as e:
            print(f"[SimConnect V14.30] queue_position_airborne error: {e}", flush=True)
            return False

    def set_elevator(self, pos: float):
        """Set elevator position for replay. pos: -1.0 (full fwd) to +1.0 (full back)."""
        try:
            self._bridge.SetElevator(ctypes.c_double(max(-1.0, min(1.0, pos))))
        except Exception as e:
            print(f"[SimConnect Native] SetElevator error: {e}", flush=True)

    def set_aileron(self, pos: float):
        """Set aileron position for replay. pos: -1.0 (full left) to +1.0 (full right)."""
        try:
            self._bridge.SetAileron(ctypes.c_double(max(-1.0, min(1.0, pos))))
        except Exception as e:
            print(f"[SimConnect Native] SetAileron error: {e}", flush=True)

    def set_rudder(self, pos: float):
        """Set rudder position for replay. pos: -1.0 (full left) to +1.0 (full right)."""
        try:
            self._bridge.SetRudder(ctypes.c_double(max(-1.0, min(1.0, pos))))
        except Exception as e:
            print(f"[SimConnect Native] SetRudder error: {e}", flush=True)

    def set_elevator_trim(self, radians: float):
        """Set elevator trim position for replay. radians: typically -0.35 to +0.35."""
        try:
            self._bridge.SetElevatorTrim(ctypes.c_double(radians))
        except Exception as e:
            print(f"[SimConnect Native] SetElevatorTrim error: {e}", flush=True)

    def set_control_inputs(self, elevator: float, aileron: float, rudder: float, trim: float):
        """Set all flight control surfaces simultaneously in Prepar3D for deterministic aircraft replay."""
        try:
            self._bridge.SetControlInputs(
                ctypes.c_double(max(-1.0, min(1.0, elevator))),
                ctypes.c_double(max(-1.0, min(1.0, aileron))),
                ctypes.c_double(max(-1.0, min(1.0, rudder))),
                ctypes.c_double(trim)
            )
        except Exception as e:
            print(f"[SimConnect Native] SetControlInputs error: {e}", flush=True)

    def set_exact_pose_state(self, state: Dict[str, Any]) -> bool:
        """
        V14.40 FINAL pose correction using ordinary settable P3D SimVars.

        This does NOT call SetUserPosition / SIMCONNECT_DATA_INITPOSITION and is
        therefore intended for the final t=0 correction AFTER terrain/scenery
        has already been preloaded.
        """
        try:
            required = (
                "latitude_deg",
                "longitude_deg",
                "altitude_ft",
                "pitch_deg",
                "bank_deg",
                "true_heading_deg",
                "ias_kts",
            )
            missing = [k for k in required if state.get(k) is None]
            if missing:
                print(
                    f"[SimConnect V14.40] set_exact_pose_state missing: {missing}",
                    flush=True,
                )
                return False

            pose = ExactPoseStateStruct(
                math.radians(float(state["latitude_deg"])),
                math.radians(float(state["longitude_deg"])),
                float(state["altitude_ft"]),
                math.radians(float(state["pitch_deg"])),
                math.radians(float(state["bank_deg"])),
                math.radians(float(state["true_heading_deg"])),
                float(state["ias_kts"]),
            )

            ok = bool(self._bridge.SetExactPoseState(ctypes.byref(pose)))
            print(
                "[SimConnect V14.40] SetExactPoseState "
                f"{'OK' if ok else 'FAILED'} | "
                f"Lat={float(state['latitude_deg']):.6f} "
                f"Lon={float(state['longitude_deg']):.6f} "
                f"Alt={float(state['altitude_ft']):.1f}ft "
                f"IAS={float(state['ias_kts']):.2f}kt "
                f"Pitch={float(state['pitch_deg']):+.2f} "
                f"Bank={float(state['bank_deg']):+.2f} "
                f"Hdg={float(state['true_heading_deg']):.2f}",
                flush=True,
            )
            return ok
        except Exception as e:
            print(f"[SimConnect V14.40] set_exact_pose_state error: {e}", flush=True)
            return False

    def set_exact_pose_state_no_airspeed(self, state: Dict[str, Any]) -> bool:
        """
        V14.66 final/soft-hold pose correction WITHOUT AIRSPEED INDICATED.

        Terrain preload still supplies the initial airspeed through INITPOSITION.
        Once scenery is loaded, the exact WORLD velocity vector is authoritative;
        writing IAS again here can force a new BODY-velocity solution and create
        the first-live-frame kick seen on the simulator PC.
        """
        try:
            required = (
                "latitude_deg",
                "longitude_deg",
                "altitude_ft",
                "pitch_deg",
                "bank_deg",
                "true_heading_deg",
            )
            missing = [k for k in required if state.get(k) is None]
            if missing:
                print(
                    f"[SimConnect V14.66] no-IAS pose missing: {missing}",
                    flush=True,
                )
                return False

            pose = ExactPoseNoAirspeedStateStruct(
                math.radians(float(state["latitude_deg"])),
                math.radians(float(state["longitude_deg"])),
                float(state["altitude_ft"]),
                math.radians(float(state["pitch_deg"])),
                math.radians(float(state["bank_deg"])),
                math.radians(float(state["true_heading_deg"])),
            )
            ok = bool(self._bridge.SetExactPoseStateNoAirspeed(ctypes.byref(pose)))
            print(
                "[SimConnect V14.66] SetExactPoseStateNoAirspeed "
                f"{'OK' if ok else 'FAILED'} | "
                f"Alt={float(state['altitude_ft']):.1f}ft "
                f"Pitch={float(state['pitch_deg']):+.2f} "
                f"Bank={float(state['bank_deg']):+.2f} "
                f"Hdg={float(state['true_heading_deg']):.2f} | IAS NOT WRITTEN",
                flush=True,
            )
            return ok
        except Exception as e:
            print(
                f"[SimConnect V14.66] set_exact_pose_state_no_airspeed error: {e}",
                flush=True,
            )
            return False

    def set_initial_dynamic_state(self, state: Dict[str, Any]) -> bool:
        """
        Restore the V14.30 flight-dynamic initial state while Prepar3D is PAUSED.

        This method intentionally does NOT write elevator/aileron/rudder input.
        BRUNNER/CLS2Sim remains the sole flight-control source.
        """
        try:
            required = (
                "velocity_world_x_fps", "velocity_world_y_fps", "velocity_world_z_fps",
                "velocity_body_x_fps", "velocity_body_y_fps", "velocity_body_z_fps",
                "vertical_speed_fpm", "engine_rpm",
                "rotation_velocity_body_x_rad_s", "rotation_velocity_body_y_rad_s",
                "rotation_velocity_body_z_rad_s", "elevator_trim_rad",
                "aileron_trim_pct", "rudder_trim_pct",
            )
            missing = [k for k in required if state.get(k) is None]
            if missing:
                print(f"[SimConnect V14.30] Initial dynamic state missing: {missing}", flush=True)
                return False

            ds = InitialDynamicStateStruct(
                velocityWorldX=float(state["velocity_world_x_fps"]),
                velocityWorldY=float(state["velocity_world_y_fps"]),
                velocityWorldZ=float(state["velocity_world_z_fps"]),
                velocityBodyX=float(state["velocity_body_x_fps"]),
                velocityBodyY=float(state["velocity_body_y_fps"]),
                velocityBodyZ=float(state["velocity_body_z_fps"]),
                verticalSpeedFps=float(state["vertical_speed_fpm"]) / 60.0,
                rotationVelocityBodyX=float(state["rotation_velocity_body_x_rad_s"]),
                rotationVelocityBodyY=float(state["rotation_velocity_body_y_rad_s"]),
                rotationVelocityBodyZ=float(state["rotation_velocity_body_z_rad_s"]),
                elevatorTrim=float(state["elevator_trim_rad"]),
                aileronTrimPct=float(state["aileron_trim_pct"]),
                rudderTrimPct=float(state["rudder_trim_pct"]),
                # GENERAL ENG RPM:1 is not settable in P3D v5.
                # For Super Mushshak, seed PROP RPM:1 from the recorded Manual
                # Engine RPM; P3D telemetry shows the direct-drive pair 1:1.
                propRpm1=float(state["engine_rpm"]),
            )
            # V15.12 ROOT-CAUSE FIX:
            # Preserve natural engine operation by avoiding PROP RPM writes, but restore
            # the complete aircraft kinematic state. The bridge already provides
            # SetInitialDynamicStateNoRpmV1496, which writes WORLD XYZ + BODY XYZ +
            # VSI + body rotations + trims and stops before propRpm1.
            #
            # The previous WORLD-only path left BODY/VSI stale while WORLD XYZ was
            # replaced. P3D reconciled the inconsistent state at unpause, producing
            # the observed IAS/VSI/AoA jump before replay began.
            setter = (
                self._bridge.SetInitialDynamicStateNoRpmV1496
                if self.natural_engine_mode
                else self._bridge.SetInitialDynamicState
            )
            ok = bool(setter(ctypes.byref(ds)))
            print(
                "[SimConnect V15.12] SetInitialDynamicState "
                + ("ACK" if ok else "FAILED")
                + f" | VW=({ds.velocityWorldX:+.3f},{ds.velocityWorldY:+.3f},{ds.velocityWorldZ:+.3f})"
                + f" VB=({ds.velocityBodyX:+.3f},{ds.velocityBodyY:+.3f},{ds.velocityBodyZ:+.3f})"
                + f" VSI={ds.verticalSpeedFps * 60.0:+.1f}fpm"
                + f" ROT=({ds.rotationVelocityBodyX:+.6f},{ds.rotationVelocityBodyY:+.6f},{ds.rotationVelocityBodyZ:+.6f})"
                + f" TRIM=({ds.elevatorTrim:+.6f},{ds.aileronTrimPct:+.6f},{ds.rudderTrimPct:+.6f})"
                + (f" PROP1={ds.propRpm1:.1f}rpm (NOT WRITTEN - natural engine)"
                   if self.natural_engine_mode else f" PROP1_SEED={ds.propRpm1:.1f}rpm"),
                flush=True,
            )
            return ok
        except Exception as e:
            print(f"[SimConnect V14.30] set_initial_dynamic_state error: {e}", flush=True)
            return False

    def set_throttle(self, percent: float):
        """Set engine throttle lever position (0.0% to 100.0%)."""
        try:
            self._bridge.SetThrottle(ctypes.c_double(max(0.0, min(100.0, float(percent)))))
        except Exception as e:
            print(f"[SimConnect Native] SetThrottle error: {e}", flush=True)


    def set_engine_controls(self, mixture_percent: float, propeller_percent: float) -> bool:
        """Set mixture and propeller levers. Public API uses 0..100 percent."""
        try:
            mix = max(0.0, min(100.0, float(mixture_percent))) / 100.0
            prop = max(0.0, min(100.0, float(propeller_percent))) / 100.0
            return bool(self._bridge.SetEngineControls(ctypes.c_double(mix), ctypes.c_double(prop)))
        except Exception as e:
            print(f"[SimConnect V14.95] SetEngineControls error: {e}", flush=True)
            return False

    def set_mixture_percent(self, percent: float) -> bool:
        """Set mixture only, preserving the current measured propeller lever."""
        try:
            d = self.get_telemetry()
            prop = float(d.get("engine_propeller_lever_1", 1.0)) * 100.0
            return self.set_engine_controls(percent, prop)
        except Exception as e:
            print(f"[SimConnect V14.95] set_mixture_percent error: {e}", flush=True)
            return False

    def set_propeller_percent(self, percent: float) -> bool:
        """Set propeller lever only, preserving the current measured mixture."""
        try:
            d = self.get_telemetry()
            mix = float(d.get("engine_mixture_lever_1", 1.0)) * 100.0
            return self.set_engine_controls(mix, percent)
        except Exception as e:
            print(f"[SimConnect V14.95] set_propeller_percent error: {e}", flush=True)
            return False

    def set_flaps_handle_index(self, index: float) -> bool:
        try:
            return bool(self._bridge.SetFlapsHandleIndex(ctypes.c_double(float(index))))
        except Exception as e:
            print(f"[SimConnect V14.95] SetFlapsHandleIndex error: {e}", flush=True)
            return False

    def set_flaps_first_detent(self) -> bool:
        return self.set_flaps_handle_index(1.0)

    def set_flap_surface_state(self, left_percent: float, right_percent: float,
                               left_angle_rad: float, right_angle_rad: float) -> bool:
        """Restore actual left/right trailing-edge flap state for exact t=0."""
        try:
            lp = max(0.0, min(1.0, float(left_percent)))
            rp = max(0.0, min(1.0, float(right_percent)))
            return bool(self._bridge.SetFlapSurfaceState(
                ctypes.c_double(lp), ctypes.c_double(rp),
                ctypes.c_double(float(left_angle_rad)), ctypes.c_double(float(right_angle_rad))))
        except Exception as e:
            print(f"[SimConnect V14.95] SetFlapSurfaceState error: {e}", flush=True)
            return False

    def set_fuel_tank_levels(self, left_main: float, right_main: float, center: float = 0.0) -> bool:
        """Set tank levels as fractions 0..1."""
        try:
            vals = [max(0.0, min(1.0, float(v))) for v in (left_main, right_main, center)]
            return bool(self._bridge.SetFuelTankLevels(*(ctypes.c_double(v) for v in vals)))
        except Exception as e:
            print(f"[SimConnect V14.95] SetFuelTankLevels error: {e}", flush=True)
            return False

    def set_linear_body_acceleration(self, x_fps2: float, y_fps2: float, z_fps2: float) -> bool:
        try:
            return bool(self._bridge.SetLinearBodyAcceleration(
                ctypes.c_double(float(x_fps2)), ctypes.c_double(float(y_fps2)), ctypes.c_double(float(z_fps2))))
        except Exception as e:
            print(f"[SimConnect V14.95] SetLinearBodyAcceleration error: {e}", flush=True)
            return False

    def set_rotation_body_acceleration(self, x_rad_s2: float, y_rad_s2: float, z_rad_s2: float) -> bool:
        try:
            return bool(self._bridge.SetRotationBodyAcceleration(
                ctypes.c_double(float(x_rad_s2)), ctypes.c_double(float(y_rad_s2)), ctypes.c_double(float(z_rad_s2))))
        except Exception as e:
            print(f"[SimConnect V14.95] SetRotationBodyAcceleration error: {e}", flush=True)
            return False

    def set_aux_configuration(self, fuel_selector: float, autopilot_master: float,
                              auto_coordination: float, spoilers_handle: float,
                              gear_handle: float, parking_brake: float,
                              engine_combustion: float) -> bool:
        try:
            vals = [float(v) for v in (fuel_selector, autopilot_master, auto_coordination,
                                       spoilers_handle, gear_handle, parking_brake, engine_combustion)]
            setter = (self._bridge.SetAuxConfigurationNoCombustionV1496 if self.natural_engine_mode
                      else self._bridge.SetAuxConfiguration)
            return bool(setter(*(ctypes.c_double(v) for v in vals)))
        except Exception as e:
            print(f"[SimConnect V14.95] SetAuxConfiguration error: {e}", flush=True)
            return False

    def set_initial_fuel_pressure(self, value):
        """Seed documented settable fuel pressure while paused; never called in replay."""
        value = float(value)
        if not math.isfinite(value) or value < 0:
            raise ValueError("Invalid recorded initial fuel pressure")
        return bool(self._bridge.SetInitialFuelPressureV15(value))

    def get_engine_support_state(self, fresh=False, timeout=0.6):
        """Dedicated block; unavailable/stale data raises instead of becoming zeros."""
        seq = int(self._bridge.GetEngineSupportSequenceV1496())
        if fresh:
            if not self._bridge.RequestEngineSupportOnceV1496():
                raise RuntimeError("Engine support request rejected")
            deadline = time.monotonic() + timeout
            while int(self._bridge.GetEngineSupportSequenceV1496()) <= seq:
                if time.monotonic() >= deadline:
                    raise RuntimeError("No fresh engine support telemetry")
                time.sleep(0.005)
        elif seq == 0:
            return {}
        raw = EngineSupportStateStructV1496()
        self._bridge.GetEngineSupportStateV1496(ctypes.byref(raw))
        values = dict(zip(ENGINE_SUPPORT_KEYS_V1496, map(float, raw.values)))
        if not all(math.isfinite(v) for v in values.values()):
            raise RuntimeError("Invalid engine support telemetry")
        values["engine_support_sequence"] = int(self._bridge.GetEngineSupportSequenceV1496())
        return values

    def send_engine_preparation_event(self, action):
        return bool(self._bridge.SendEnginePreparationEventV1496(int(action)))

    def restore_engine_support_state(self, state, thermal=True):
        """Restore supported temperatures/battery; observe each toggle once and verify.

        Fuel pressure is seeded separately at paused t=0; no RPM/combustion forcing.
        """
        live = self.get_engine_support_state(fresh=True)
        for index in ((0, 1, 2, 3) if thermal else (3,)):
            key = ENGINE_SUPPORT_KEYS_V1496[index]
            value = state.get(key)
            if value is not None:
                value = float(value)
                if not math.isfinite(value) or not self._bridge.SetEngineSupportValueV1496(index, value):
                    raise RuntimeError("Engine support restore failed: " + key)
        # Toggles are serialized with a fresh acknowledgement, never retried blindly.
        for index, action in ((4, 2), (5, 1), (6, 3)):
            key = ENGINE_SUPPORT_KEYS_V1496[index]
            target = state.get(key)
            if target is None or (float(live[key]) >= 0.5) == (float(target) >= 0.5):
                continue
            if not self.send_engine_preparation_event(action):
                raise RuntimeError("Engine switch event rejected: " + key)
            deadline = time.monotonic() + 1.5
            while True:
                live = self.get_engine_support_state(fresh=True)
                if (live[key] >= 0.5) == (float(target) >= 0.5):
                    break
                if time.monotonic() >= deadline:
                    raise RuntimeError("Engine switch did not acknowledge: " + key)
                time.sleep(0.02)
        # Start is a temporary preparation action, not a replay output.
        starter = state.get("general_eng_starter_1")
        if starter is not None:
            if not self._bridge.SetEngineSupportValueV1496(7, float(starter)):
                raise RuntimeError("Starter restore rejected")
        return True

    def get_extended_causal_state(self) -> Dict[str, Any]:
        """Return V14.95 slow/causal state from the dedicated bridge block."""
        try:
            sequence = int(self._bridge.GetExtendedCausalSequenceV1495())
            if sequence <= 0:
                return {}
            raw = ExtendedCausalStateStructV1495()
            self._bridge.GetExtendedCausalStateV1495(ctypes.byref(raw))
            payload_count = max(0, min(V1495_MAX_PAYLOAD_STATIONS, int(round(float(raw.payloadStationCount))))) if math.isfinite(raw.payloadStationCount) else 0
            payload = [float(raw.payloadStationWeight[i]) for i in range(payload_count)]
            levels = [float(raw.fuelTankLevel[i]) for i in range(11)]
            caps = [float(raw.fuelTankCapacity[i]) for i in range(11)]
            out = {
                "payload_station_count": payload_count,
                "payload_station_weights_lb": payload,
                "total_weight_pitch_moi": float(raw.totalWeightPitchMoi),
                "total_weight_roll_moi": float(raw.totalWeightRollMoi),
                "total_weight_yaw_moi": float(raw.totalWeightYawMoi),
                "total_weight_cross_coupled_moi": float(raw.totalWeightCrossCoupledMoi),
                "fuel_tank_levels_all": dict(zip(V1495_FUEL_TANK_NAMES, levels)),
                "fuel_tank_capacities_gal": dict(zip(V1495_FUEL_TANK_NAMES, caps)),
                "fuel_tank_selectors": [float(raw.fuelTankSelector1), float(raw.fuelTankSelector2), float(raw.fuelTankSelector3), float(raw.fuelTankSelector4)],
                "recip_cowl_flap_position_1": float(raw.recipCowlFlapPosition1),
                "recip_primer_1": float(raw.recipPrimer1),
                "recip_alternate_air_position_1": float(raw.recipAlternateAirPosition1),
                "recip_left_magneto_1": float(raw.recipLeftMagneto1),
                "recip_right_magneto_1": float(raw.recipRightMagneto1),
                "recip_fuel_available_1": float(raw.recipFuelAvailable1),
                "recip_fuel_tank_selector_1": float(raw.recipFuelTankSelector1),
                "recip_fuel_tanks_used_1": float(raw.recipFuelTanksUsed1),
                "general_eng_starter_1": float(raw.generalEngStarter1),
                "general_eng_fuel_pump_switch_1": float(raw.generalEngFuelPumpSwitch1),
                "general_eng_fuel_pump_on_1": float(raw.generalEngFuelPumpOn1),
                "general_eng_damage_percent_1": float(raw.generalEngDamagePercent1),
                "general_eng_failed_1": float(raw.generalEngFailed1),
                "general_eng_anti_ice_position_1": float(raw.generalEngAntiIcePosition1),
                "prop_deice_switch_1": float(raw.propDeiceSwitch1),
                "eng_on_fire_1": float(raw.engOnFire1),
                "structural_ice_pct": float(raw.structuralIcePct),
                "pitot_ice_pct": float(raw.pitotIcePct),
                "pitot_heat": float(raw.pitotHeat),
                "flap_damage_by_speed": float(raw.flapDamageBySpeed),
                "flap_speed_exceeded": float(raw.flapSpeedExceeded),
                "gear_damage_by_speed": float(raw.gearDamageBySpeed),
                "gear_speed_exceeded": float(raw.gearSpeedExceeded),
                "spoilers_left_position": float(raw.spoilersLeftPosition),
                "spoilers_right_position": float(raw.spoilersRightPosition),
                "gear_center_position": float(raw.gearCenterPosition),
                "gear_left_position": float(raw.gearLeftPosition),
                "gear_right_position": float(raw.gearRightPosition),
                "canopy_open": float(raw.canopyOpen),
                "damage_state": float(raw.damageState),
                "sim_on_ground": float(raw.simOnGround),
                "user_input_enabled": float(raw.userInputEnabled),
                "gyro_heading_rad": float(raw.gyroHeadingRad),
                "gyro_drift_error_rad": float(raw.gyroDriftErrorRad),
            }
            out = {key: (value if not isinstance(value, float) or math.isfinite(value) else None)
                   for key, value in out.items()}
            out["fuel_tank_levels_all"] = {name: value for name,value in zip(V1495_FUEL_TANK_NAMES,levels)
                if math.isfinite(value) and 0 <= value <= 1}
            out["fuel_tank_capacities_gal"] = {name: value for name,value in zip(V1495_FUEL_TANK_NAMES,caps)
                if math.isfinite(value) and value >= 0}
            if not all(math.isfinite(value) and value >= 0 for value in payload):
                out["payload_station_weights_lb"] = []
            bool_keys = ("recip_primer_1", "recip_left_magneto_1", "recip_right_magneto_1",
                "recip_fuel_available_1", "general_eng_starter_1", "general_eng_fuel_pump_switch_1",
                "general_eng_fuel_pump_on_1", "general_eng_failed_1", "eng_on_fire_1",
                "flap_damage_by_speed", "gear_damage_by_speed")
            invalid_fields = []
            for key in bool_keys:
                if out.get(key) not in (None, 0.0, 1.0):
                    invalid_fields.append(key)
                    out[key] = None
            out["extended_causal_invalid_fields"] = invalid_fields
            out["extended_causal_sequence"] = sequence
            out["extended_causal_layout"] = "TAGGED_V1497"
            return out
        except Exception as e:
            print(f"[SimConnect V14.95] Extended causal read error: {e}", flush=True)
            return {}

    def request_extended_causal_once(self) -> bool:
        try:
            return bool(self._bridge.RequestExtendedCausalOnceV1495())
        except Exception:
            return False

    def set_payload_station_weights(self, weights_lb) -> bool:
        try:
            vals = [max(0.0, float(v)) for v in list(weights_lb or [])[:V1495_MAX_PAYLOAD_STATIONS]]
            arr = (ctypes.c_double * len(vals))(*vals) if vals else None
            if not vals:
                return True
            return bool(self._bridge.SetPayloadStationWeightsV1495(arr, ctypes.c_int(len(vals))))
        except Exception as e:
            print(f"[SimConnect V14.95] SetPayloadStationWeights error: {e}", flush=True)
            return False

    def set_all_fuel_tank_levels(self, levels) -> bool:
        try:
            if isinstance(levels, dict):
                if any(n not in levels or levels[n] is None for n in V1495_FUEL_TANK_NAMES):
                    raise ValueError("Incomplete all-tank fuel dictionary; no fuel write performed")
                vals = [float(levels[n]) for n in V1495_FUEL_TANK_NAMES]
            else:
                vals = list(levels or [])
            if len(vals) != 11:
                return False
            if not all(math.isfinite(float(v)) and 0 <= float(v) <= 1 for v in vals):
                raise ValueError("Invalid all-tank levels")
            arr = (ctypes.c_double * 11)(*vals)
            return bool(self._bridge.SetAllFuelTankLevelsV1495(arr, ctypes.c_int(11)))
        except Exception as e:
            print(f"[SimConnect V14.95] SetAllFuelTankLevels error: {e}", flush=True)
            return False

    def set_extended_aircraft_configuration(self, state):
        keys = ("recip_cowl_flap_position_1", "recip_primer_1", "recip_alternate_air_position_1",
            "recip_left_magneto_1", "recip_right_magneto_1", "recip_fuel_available_1",
            "recip_fuel_tank_selector_1", "recip_fuel_tanks_used_1", "general_eng_starter_1",
            "gear_center_position", "gear_left_position", "gear_right_position",
            "canopy_open", "gyro_heading_rad", "damage_state")
        for index,key in enumerate(keys):
            value = state.get(key)
            if value is None: continue
            value = float(value)
            if not math.isfinite(value) or not self._bridge.SetCausalFieldV1497(index,value):
                return False
        return True


    def set_global_weather_metar(self, metar: str) -> bool:
        """Apply a Prepar3D global-weather METAR through SimConnect."""
        try:
            if not isinstance(metar, str) or not metar.strip():
                return False
            return bool(self._bridge.SetGlobalWeatherMETAR(metar.encode("ascii", errors="strict")))
        except Exception as e:
            print(f"[SimConnect V14.95] SetGlobalWeatherMETAR error: {e}", flush=True)
            return False

    def get_global_weather_metar(self, timeout_sec: float = 1.5) -> Optional[str]:
        """Request and return P3D's current global METAR string."""
        try:
            seq0 = int(self._bridge.GetWeatherSequence())
            if not bool(self._bridge.RequestGlobalWeatherMETAR()):
                return None
            deadline = time.monotonic() + max(0.1, float(timeout_sec))
            while time.monotonic() < deadline:
                if int(self._bridge.GetWeatherSequence()) > seq0:
                    buf = ctypes.create_string_buffer(4096)
                    if bool(self._bridge.GetLastWeatherMETAR(buf, ctypes.c_int(len(buf)))):
                        value = buf.value.decode("ascii", errors="ignore").strip()
                        return value or None
                time.sleep(0.01)
            return None
        except Exception as e:
            print(f"[SimConnect V14.95] GetGlobalWeatherMETAR error: {e}", flush=True)
            return None

    def set_daytime(self):
        """Set Prepar3D local time to 12:00 PM (Daylight)."""
        try:
            self._bridge.SetDaytime()
            print("[SimConnect Native] Set P3D time to 12:00 PM (Daylight).", flush=True)
        except Exception as e:
            print(f"[SimConnect Native] Error setting daytime: {e}", flush=True)

    def set_position_rahwali(self):
        """Teleport the aircraft to exact centerline position from P3D Map screenshot (N32° 14.03', E74° 7.42', 750.23 ft, Hdg 48.4°)."""
        try:
            pos = InitPositionStruct(
                Latitude=32.23383333,
                Longitude=74.12366667,
                Altitude=750.236,
                Pitch=0.0,
                Bank=0.0,
                Heading=48.4,
                OnGround=1,
                Airspeed=0,
            )
            self._bridge.SetUserPosition(ctypes.byref(pos))
            self.pause_sim()  # Freeze sim instantly
            print("[SimConnect Native] Aircraft repositioned to Rahwali Runway 05 Start Point.", flush=True)
            self.set_daytime()
            self.pause_sim()  # Enforce paused state
        except Exception as e:
            print(f"[SimConnect Native] Error setting exact position: {e}", flush=True)

    def set_flaps(self, percent: float = 0.0):
        """Set aircraft flaps handle position (0.0% to 100.0%)."""
        if hasattr(self._bridge, "SetFlaps"):
            try:
                self._bridge.SetFlaps(ctypes.c_double(max(0.0, min(100.0, float(percent)))))
            except Exception as e:
                print(f"[SimConnect Native] SetFlaps error: {e}", flush=True)

    def set_position_airborne(
        self,
        alt_ft: float = 3000.0,
        speed_kts: float = 80.0,
        pitch_deg: float = 0.0,
        bank_deg: float = 0.0,
        hdg_deg: float = 48.4,
        lat: float = 32.23383333,
        lon: float = 74.12366667,
        throttle: float = 80.0,
    ):
        """Teleport the aircraft airborne at exact manual flight initial conditions."""
        try:
            pos = InitPositionStruct(
                Latitude=float(lat),
                Longitude=float(lon),
                Altitude=float(alt_ft),
                Pitch=float(pitch_deg),
                Bank=float(bank_deg),
                Heading=float(hdg_deg),
                OnGround=0,  # 0 = airborne
                Airspeed=int(round(float(speed_kts))),
            )
            self._bridge.SetUserPosition(ctypes.byref(pos))
            self.pause_sim()        # Freeze sim instantly upon teleporting!
            self.set_flaps(0.0)      # Explicit Flaps 0
            self.set_throttle(float(throttle))   # Match manual initial throttle
            print(f"[SimConnect Native] Aircraft placed AIRBORNE: Lat={lat:.5f}, Lon={lon:.5f}, Alt={alt_ft:.1f}ft, Speed={speed_kts:.1f}kts, Pitch={pitch_deg:.1f}°, Bank={bank_deg:.1f}°, Hdg={hdg_deg:.1f}°, Throttle={throttle:.1f}%", flush=True)
            self.set_daytime()
            self.pause_sim()        # Enforce paused state
        except Exception as e:
            print(f"[SimConnect Native] Error setting airborne position: {e}", flush=True)

    def connect_and_run(
        self,
        on_connected: Optional[Callable] = None,
        on_error: Optional[Callable[[str], None]] = None,
        place_at_rahwali: bool = False,
        place_airborne: bool = False,
        alt_ft: float = 3000.0,
        speed_kts: float = 80.0,
        initial_pitch: float = 0.0,
        initial_bank: float = 0.0,
        initial_hdg: float = 48.4,
        initial_lat: float = 32.23383333,
        initial_lon: float = 74.12366667,
        initial_throttle: float = 80.0,
    ):
        """
        Connect to Prepar3D via native bridge and poll telemetry.
        Runs continuously in a dedicated background thread until stop() is called.
        """
        self._stop_requested.clear()
        self.last_error = ""

        try:
            print("[SimConnect Native] Opening connection via p3d_bridge.dll...", flush=True)
            res = self._bridge.ConnectSim()
            if res != 1:
                raise RuntimeError("ConnectSim failed to open SimConnect session.")

            print("[SimConnect Native] Session initiated. Waiting for P3D handshake...", flush=True)
            
            start_wait = time.time()
            last_reconnect_try = start_wait
            connected_signal_sent = False

            while not self._stop_requested.is_set():
                self._bridge.ProcessMessages()
                is_conn = bool(self._bridge.IsSimConnected())

                if is_conn:
                    if not connected_signal_sent:
                        print("[SimConnect Native] Handshake complete! Connected to Prepar3D.", flush=True)
                        with self._lock:
                            self._connected = True
                        connected_signal_sent = True

                        if place_at_rahwali:
                            self.set_position_rahwali()
                            time.sleep(1.5)
                        elif place_airborne:
                            self.set_position_airborne(
                                alt_ft=alt_ft,
                                speed_kts=speed_kts,
                                pitch_deg=initial_pitch,
                                bank_deg=initial_bank,
                                hdg_deg=initial_hdg,
                                lat=initial_lat,
                                lon=initial_lon,
                                throttle=initial_throttle,
                            )
                            time.sleep(1.5)

                        if on_connected:
                            on_connected()

                    # Fetch telemetry
                    snapshot = self._read_primary_snapshot()
                    if snapshot is not None:
                        raw_data, sequence = snapshot
                        self._update_latest_data(raw_data, packet_sequence=sequence)
                else:
                    with self._lock:
                        self._connected = False
                    if not connected_signal_sent:
                        now = time.time()
                        if now - last_reconnect_try > 2.0:
                            last_reconnect_try = now
                            print("[SimConnect Native] Waiting for Prepar3D flight session... retrying handshake...", flush=True)
                            try:
                                self._bridge.ConnectSim()
                            except Exception:
                                pass

                    if time.time() - start_wait > 30.0 and not connected_signal_sent:
                        raise TimeoutError("Timed out waiting for Prepar3D connection handshake. Please ensure Prepar3D is in an active flight scenario.")

                time.sleep(0.04)  # ~25 Hz poll loop

        except Exception as exc:
            self.last_error = str(exc)
            print(f"[SimConnect Native] ERROR: {exc}", flush=True)
            with self._lock:
                self._connected = False
            if on_error:
                on_error(str(exc))
        finally:
            self._close_connection()
            print("[SimConnect Native] Poll thread exiting.", flush=True)

    def stop(self):
        """Signal poll loop to exit."""
        self._stop_requested.set()

    def is_connected(self) -> bool:
        with self._lock:
            return self._connected

    def get_telemetry(self) -> Dict[str, Any]:
        with self._lock:
            return dict(self._latest_data)

    def _close_connection(self):
        with self._lock:
            self._connected = False
        try:
            self._bridge.DisconnectSim()
        except Exception:
            pass

    def _read_primary_snapshot(self):
        # Bracket the copy with sequence reads. This detects a changing
        # sequence during the copy, but cannot prove the DLL writes atomically.
        with self._snapshot_lock:
            for _ in range(3):
                before = self.get_telemetry_sequence()
                if before <= 0:
                    return None
                raw = TelemetryDataStruct()
                self._bridge.GetTelemetryDataV1492(ctypes.byref(raw))
                after = self.get_telemetry_sequence()
                if before == after:
                    return raw, after
        return None

    def _update_latest_data(self, raw: TelemetryDataStruct, packet_sequence=None):
        sequence = self.get_telemetry_sequence() if packet_sequence is None else int(packet_sequence)
        if sequence <= 0:
            return False
        num_eng = max(1, min(4, int(round(float(raw.numberOfEngines)))))
        throttles = [
            max(0.0, float(raw.throttle1)),
            max(0.0, float(raw.throttle2)),
            max(0.0, float(raw.throttle3)),
            max(0.0, float(raw.throttle4)),
        ]
        max_t = max(throttles[:num_eng])

        title_str = raw.title.decode("utf-8", errors="ignore").strip()

        with self._lock:
            # A cached DLL read is not a newly received packet. Preserve its
            # prior update timestamp instead of making stale telemetry look fresh.
            previous = int(self._latest_data.get("bridge_telemetry_sequence", 0) or 0)
            if sequence <= previous:
                return False
            self._latest_data = {
                "title": title_str or "Piston Engine Aircraft",
                "number_of_engines": num_eng,
                "airspeed_indicated": max(0.0, float(raw.airspeedIndicated)),
                "vertical_speed": float(raw.verticalSpeed),
                "indicated_altitude": float(raw.indicatedAltitude),
                "magnetic_heading": float(raw.magneticHeading) % 360.0,
                "pitch_angle": float(raw.pitchAngle),
                "bank_angle": -float(raw.bankAngle),   # Right bank is (+), Left bank is (-)
                "true_heading": float(raw.trueHeading) % 360.0,
                "yaw_angle": float(raw.trueHeading) % 360.0,
                "throttles": throttles,
                "max_throttle": max_t,
                "prop_rpms": [
                    float(raw.propRPM1),
                    float(raw.propRPM2),
                    float(raw.propRPM3),
                    float(raw.propRPM4),
                ],
                "eng_rpms": [
                    float(raw.engRPM1),
                    float(raw.engRPM2),
                    float(raw.engRPM3),
                    float(raw.engRPM4),
                ],
                "engine_mixture_lever_1": float(raw.mixtureLever1),
                "engine_propeller_lever_1": float(raw.propellerLever1),
                "engine_combustion_1": float(raw.engineCombustion1),
                "engine_fuel_valve_1": float(raw.engineFuelValve1),
                "engine_fuel_pressure_psi_1": float(raw.engineFuelPressure1),
                "engine_manifold_pressure_psi_1": float(raw.manifoldPressure1),
                "relative_wind_body_x_fps": float(raw.relativeWindBodyX),
                "relative_wind_body_y_fps": float(raw.relativeWindBodyY),
                "relative_wind_body_z_fps": float(raw.relativeWindBodyZ),
                "flaps_handle_percent": float(raw.flapsHandlePercent),
                "flaps_handle_index": float(raw.flapsHandleIndex),
                "flaps_num_handle_positions": float(raw.flapsNumHandlePositions),
                "trailing_edge_flaps_left_percent": float(raw.trailingEdgeFlapsLeftPercent),
                "trailing_edge_flaps_right_percent": float(raw.trailingEdgeFlapsRightPercent),
                "trailing_edge_flaps_left_angle_rad": float(raw.trailingEdgeFlapsLeftAngle),
                "trailing_edge_flaps_right_angle_rad": float(raw.trailingEdgeFlapsRightAngle),
                "ambient_density_slug_ft3": float(raw.ambientDensity),
                "ambient_pressure_mb": float(raw.ambientPressure),
                "aircraft_wind_x_kt": float(raw.aircraftWindX),
                "aircraft_wind_y_kt": float(raw.aircraftWindY),
                "aircraft_wind_z_kt": float(raw.aircraftWindZ),
                "rotation_acceleration_body_x_rad_s2": float(raw.rotationAccelerationBodyX),
                "rotation_acceleration_body_y_rad_s2": float(raw.rotationAccelerationBodyY),
                "rotation_acceleration_body_z_rad_s2": float(raw.rotationAccelerationBodyZ),
                "fuel_tank_left_main_level": float(raw.fuelTankLeftMainLevel),
                "fuel_tank_right_main_level": float(raw.fuelTankRightMainLevel),
                "fuel_tank_center_level": float(raw.fuelTankCenterLevel),
                "fuel_tank_selector_1": float(raw.fuelTankSelector1),
                "autopilot_master": float(raw.autopilotMaster),
                "auto_coordination": float(raw.autoCoordination),
                "spoilers_handle_position": float(raw.spoilersHandlePosition),
                "gear_handle_position": float(raw.gearHandlePosition),
                "brake_parking_position": float(raw.brakeParkingPosition),
                "total_weight": float(raw.totalWeight),
                "total_fuel_quantity": float(raw.totalFuelQuantity),
                "cg_percent_lateral": float(raw.cgPercentLateral),
                "cg_percent": float(raw.cgPercent),
                "baro_pressure": float(raw.baroPressure),
                "ambient_temp": float(raw.ambientTemp),
                "ambient_wind_dir": float(raw.ambientWindDirection),
                "ambient_wind_spd": float(raw.ambientWindVelocity),
                "ground_speed": float(raw.groundSpeed),
                "mach_number": float(raw.machNumber),
                "pressure_alt": float(raw.pressureAlt),
                "radar_alt": float(raw.radarAlt),
                "ground_alt": float(raw.groundAltitude),
                "aoa": float(raw.aoa),
                "sideslip": float(raw.sideslip),
                "track_angle": float(raw.trackAngle),
                "roll_rate": float(raw.rollRate),
                "pitch_rate": float(raw.pitchRate),
                "yaw_rate": float(raw.yawRate),
                "accelX": float(raw.accelX),
                "accelY": float(raw.accelY),
                "accelZ": float(raw.accelZ),
                "rotorRPM": float(raw.rotorRPM),
                "fuelFlow": float(raw.fuelFlow),
                "torque": float(raw.torque),
                "egt": float(raw.egt),
                "elevator_pos": float(raw.elevatorPos),
                "aileron_pos": float(raw.aileronPos),
                "rudder_pos": float(raw.rudderPos),
                "elevator_trim": float(raw.elevatorTrim),

                # V14.29 exact raw flight-dynamic state / actual surface telemetry.
                "velocity_world_x": float(raw.velocityWorldX),
                "velocity_world_y": float(raw.velocityWorldY),
                "velocity_world_z": float(raw.velocityWorldZ),
                "velocity_body_x": float(raw.velocityBodyX),
                "velocity_body_y": float(raw.velocityBodyY),
                "velocity_body_z": float(raw.velocityBodyZ),
                "rotation_velocity_body_x": float(raw.rotationVelocityBodyX),
                "rotation_velocity_body_y": float(raw.rotationVelocityBodyY),
                "rotation_velocity_body_z": float(raw.rotationVelocityBodyZ),
                "elevator_deflection": float(raw.elevatorDeflection),
                "aileron_left_deflection": float(raw.aileronLeftDeflection),
                "aileron_right_deflection": float(raw.aileronRightDeflection),
                "rudder_deflection": float(raw.rudderDeflection),
                "elevator_trim_pct": float(raw.elevatorTrimPct),
                "aileron_trim_pct": float(raw.aileronTrimPct),
                "rudder_trim_pct": float(raw.rudderTrimPct),
                "plane_latitude": float(raw.planeLatitude),
                "plane_longitude": float(raw.planeLongitude),
                "bridge_version": int(self.bridge_version),
                "bridge_telemetry_sequence": sequence,

                "yoke_y_percent": float(raw.elevatorPos) * 100.0,
                "yoke_x_percent": float(raw.aileronPos) * 100.0,
                "rudder_pedal_percent": float(raw.rudderPos) * 100.0,
                "last_updated": time.time(),
            }
            # V14.95 merges the slower causal/inertia block into the normal
            # telemetry dictionary, so all existing logger/audit paths capture it.
            try:
                self._latest_data.update(self.get_extended_causal_state())
                self._latest_data.update(self.get_engine_support_state())
            except Exception:
                pass

    @staticmethod
    def _empty_telemetry() -> Dict[str, Any]:
        return {
            "title": "Super Mushshak Aircraft",
            "number_of_engines": 1,
            "airspeed_indicated": 0.0,
            "vertical_speed": 0.0,
            "indicated_altitude": 0.0,
            "magnetic_heading": 0.0,
            "pitch_angle": 0.0,
            "bank_angle": 0.0,
            "true_heading": 0.0,
            "yaw_angle": 0.0,
            "throttles": [0.0, 0.0, 0.0, 0.0],
            "max_throttle": 0.0,
            "prop_rpms": [0.0, 0.0, 0.0, 0.0],
            "eng_rpms": [0.0, 0.0, 0.0, 0.0],
            "engine_mixture_lever_1": 0.0,
            "engine_propeller_lever_1": 0.0,
            "engine_combustion_1": 0.0,
            "engine_fuel_valve_1": 0.0,
            "engine_fuel_pressure_psi_1": 0.0,
            "engine_manifold_pressure_psi_1": 0.0,
            "relative_wind_body_x_fps": 0.0,
            "relative_wind_body_y_fps": 0.0,
            "relative_wind_body_z_fps": 0.0,
            "flaps_handle_percent": 0.0,
            "flaps_handle_index": 0.0,
            "flaps_num_handle_positions": 0.0,
            "trailing_edge_flaps_left_percent": 0.0,
            "trailing_edge_flaps_right_percent": 0.0,
            "trailing_edge_flaps_left_angle_rad": 0.0,
            "trailing_edge_flaps_right_angle_rad": 0.0,
            "ambient_density_slug_ft3": 0.0,
            "ambient_pressure_mb": 0.0,
            "aircraft_wind_x_kt": 0.0,
            "aircraft_wind_y_kt": 0.0,
            "aircraft_wind_z_kt": 0.0,
            "rotation_acceleration_body_x_rad_s2": 0.0,
            "rotation_acceleration_body_y_rad_s2": 0.0,
            "rotation_acceleration_body_z_rad_s2": 0.0,
            "fuel_tank_left_main_level": 0.0,
            "fuel_tank_right_main_level": 0.0,
            "fuel_tank_center_level": 0.0,
            "fuel_tank_selector_1": 0.0,
            "autopilot_master": 0.0,
            "auto_coordination": 0.0,
            "spoilers_handle_position": 0.0,
            "gear_handle_position": 0.0,
            "brake_parking_position": 0.0,
            "total_weight": 0.0,
            "total_fuel_quantity": 0.0,
            "cg_percent_lateral": 0.0,
            "cg_percent": 0.0,
            "baro_pressure": 0.0,
            "ambient_temp": 0.0,
            "ambient_wind_dir": 0.0,
            "ambient_wind_spd": 0.0,
            "ground_speed": 0.0,
            "mach_number": 0.0,
            "pressure_alt": 0.0,
            "radar_alt": 0.0,
            "ground_alt": 0.0,
            "aoa": 0.0,
            "sideslip": 0.0,
            "track_angle": 0.0,
            "roll_rate": 0.0,
            "pitch_rate": 0.0,
            "yaw_rate": 0.0,
            "accelX": 0.0,
            "accelY": 0.0,
            "accelZ": 0.0,
            "rotorRPM": 0.0,
            "fuelFlow": 0.0,
            "torque": 0.0,
            "egt": 0.0,
            "elevator_pos": 0.0,
            "aileron_pos": 0.0,
            "rudder_pos": 0.0,
            "elevator_trim": 0.0,
            "velocity_world_x": 0.0,
            "velocity_world_y": 0.0,
            "velocity_world_z": 0.0,
            "velocity_body_x": 0.0,
            "velocity_body_y": 0.0,
            "velocity_body_z": 0.0,
            "rotation_velocity_body_x": 0.0,
            "rotation_velocity_body_y": 0.0,
            "rotation_velocity_body_z": 0.0,
            "elevator_deflection": 0.0,
            "aileron_left_deflection": 0.0,
            "aileron_right_deflection": 0.0,
            "rudder_deflection": 0.0,
            "elevator_trim_pct": 0.0,
            "aileron_trim_pct": 0.0,
            "rudder_trim_pct": 0.0,
            "plane_latitude": 0.0,
            "plane_longitude": 0.0,
            "bridge_version": 1470,
            "bridge_telemetry_sequence": 0,
            "yoke_y_percent": 0.0,
            "yoke_x_percent": 0.0,
            "rudder_pedal_percent": 0.0,
            "last_updated": 0.0,
        }
