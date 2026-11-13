#include <windows.h>
#include <SimConnect.h>
#include <stdio.h>
#include <string.h>
#include <stddef.h>
#include <limits>
#include <cmath>
#include <map>
#include <string>

// Associate asynchronous SDK exceptions with the exact registration that caused them.
static std::map<DWORD, std::string> g_PacketDescriptions;
static HRESULT TrackedAddToDataDefinition(HANDLE connection, SIMCONNECT_DATA_DEFINITION_ID definition,
    const char* name, const char* units, SIMCONNECT_DATATYPE type, float epsilon, DWORD datum) {
    HRESULT result = SimConnect_AddToDataDefinition(connection, definition, name, units, type, epsilon, datum);
    DWORD packet = 0;
    if (SUCCEEDED(SimConnect_GetLastSentPacketID(connection, &packet))) {
        g_PacketDescriptions[packet] = std::string("definition=") + std::to_string(definition)
            + " variable=" + (name ? name : "") + " units=" + (units ? units : "");
    }
    return result;
}

#pragma pack(push, 1)
typedef struct {
    char title[256];
    double numberOfEngines;
    double airspeedIndicated;
    double verticalSpeed;
    double indicatedAltitude;
    double magneticHeading;
    double pitchAngle;
    double bankAngle;
    double trueHeading;
    double throttle1;
    double throttle2;
    double throttle3;
    double throttle4;
    double propRPM1;
    double propRPM2;
    double propRPM3;
    double propRPM4;
    double engRPM1;
    double engRPM2;
    double engRPM3;
    double engRPM4;
    // New variables added for Weight and Balance / Environment
    double totalWeight;
    double totalFuelQuantity;
    double cgPercentLateral;
    double cgPercent;
    double baroPressure;
    double ambientTemp;
    double ambientWindDirection;
    double ambientWindVelocity;
    // New Flight Parameters
    double groundSpeed;
    double machNumber;
    double pressureAlt;
    double radarAlt;
    double groundAltitude;
    double aoa;
    double sideslip;
    double trackAngle;
    double rollRate;
    double pitchRate;
    double yawRate;
    double accelX;
    double accelY;
    double accelZ;
    double rotorRPM;
    double fuelFlow;
    double torque;
    double egt;
    // Control surface positions (for record & replay)
    double elevatorPos;
    double aileronPos;
    double rudderPos;
    double elevatorTrim;

    // V14.30 deterministic initial-state / control-path telemetry.
    // Appended to preserve the legacy field order above.
    double velocityWorldX;              // ft/s, East/West
    double velocityWorldY;              // ft/s, vertical
    double velocityWorldZ;              // ft/s, North/South
    double velocityBodyX;               // ft/s, body-axis lateral per P3D SimVar
    double velocityBodyY;               // ft/s, body-axis vertical per P3D SimVar
    double velocityBodyZ;               // ft/s, body-axis longitudinal per P3D SimVar
    double rotationVelocityBodyX;       // rad/s, exact P3D BODY X SimVar
    double rotationVelocityBodyY;       // rad/s, exact P3D BODY Y SimVar
    double rotationVelocityBodyZ;       // rad/s, exact P3D BODY Z SimVar
    double elevatorDeflection;          // rad, actual surface deflection
    double aileronLeftDeflection;       // rad, actual left aileron deflection
    double aileronRightDeflection;      // rad, actual right aileron deflection
    double rudderDeflection;            // rad, actual surface deflection
    double elevatorTrimPct;             // percent over 100 (read-only indicator)
    double aileronTrimPct;              // percent over 100, settable
    double rudderTrimPct;               // percent over 100, settable
    double planeLatitude;               // degrees
    double planeLongitude;              // degrees
    // V14.87 read-only cause diagnostics; keep existing V14.70 prefix intact.
    double mixtureLever1;               // percent over 100
    double propellerLever1;             // percent over 100
    double engineCombustion1;           // bool
    double engineFuelValve1;            // bool
    double engineFuelPressure1;         // psi
    double manifoldPressure1;           // psi
    double relativeWindBodyX;           // ft/s lateral air-relative flow
    double relativeWindBodyY;           // ft/s vertical air-relative flow
    double relativeWindBodyZ;           // ft/s longitudinal air-relative flow

    // V14.92 deterministic causal-state coverage. Appended only; never alter
    // any existing V14.87 field position.
    double flapsHandlePercent;
    double flapsHandleIndex;
    double flapsNumHandlePositions;
    double trailingEdgeFlapsLeftPercent;
    double trailingEdgeFlapsRightPercent;
    double trailingEdgeFlapsLeftAngle;
    double trailingEdgeFlapsRightAngle;
    double ambientDensity;
    double ambientPressure;
    double aircraftWindX;
    double aircraftWindY;
    double aircraftWindZ;
    double rotationAccelerationBodyX;
    double rotationAccelerationBodyY;
    double rotationAccelerationBodyZ;
    double fuelTankLeftMainLevel;
    double fuelTankRightMainLevel;
    double fuelTankCenterLevel;
    double fuelTankSelector1;
    double autopilotMaster;
    double autoCoordination;
    double spoilersHandlePosition;
    double gearHandlePosition;
    double brakeParkingPosition;
    double propBetaDiagnostic;
    double engineTorqueDiagnostic;
    double fuelFlowDiagnostic;
    double egtDiagnostic;
    double mixtureRatioDiagnostic;
} TelemetryData;

// The legacy Python wrapper allocates exactly 656 bytes. Keep the original
// telemetry prefix byte-for-byte compatible so V14.21 and other older builds
// can safely use this DLL through GetTelemetryData().
static_assert(offsetof(TelemetryData, velocityWorldX) == 656, "Legacy telemetry prefix changed");

// V14.30: values that are explicitly settable through Prepar3D SimConnect.
// Keep all entries as doubles and in EXACTLY the same order as DEFINITION_DYNAMICS.
typedef struct {
    double velocityWorldX;
    double velocityWorldY;
    double velocityWorldZ;

    // V14.68: exact BODY linear velocity.
    double velocityBodyX;
    double velocityBodyY;
    double velocityBodyZ;

    // V14.70: P3D exposes VERTICAL SPEED as a separately settable
    // flight-instrument state.  This is intentionally restored in addition
    // to WORLD/BODY velocity because the recorded Manual VSI can have filter/
    // instrument memory that is not identical to instantaneous WORLD Y.
    double verticalSpeedFps;

    double rotationVelocityBodyX;
    double rotationVelocityBodyY;
    double rotationVelocityBodyZ;
    double elevatorTrim;
    double aileronTrimPct;
    double rudderTrimPct;

    // V14.70: GENERAL ENG RPM is read-only in the P3D v5 SDK, but PROP RPM:1
    // is settable.  Super Mushshak is a piston/propeller aircraft and the
    // recorded P3D telemetry shows Engine RPM and Prop RPM coupled 1:1, so
    // the Manual Engine-RPM value is used as the t=0 prop/shaft RPM seed.
    double propRpm1;
} InitialDynamicState;

// V14.40 FINAL pose setter.
// Unlike SIMCONNECT_DATA_INITPOSITION, these are ordinary settable P3D SimVars.
// Angles are radians to match the native P3D SDK units.
typedef struct {
    double latitudeRad;
    double longitudeRad;
    double altitudeFt;
    double pitchRad;
    double bankRad;
    double headingTrueRad;
    double airspeedIndicatedKts;
} ExactPoseState;

// V14.66 FINAL pose setter used after the terrain preload.
// IMPORTANT: IAS is deliberately excluded.  The old V14.40 exact-pose setter
// wrote AIRSPEED INDICATED at the same time as the WORLD velocity vector.
// On P3D v5 that can recreate a mostly-forward BODY velocity and leave WORLD
// and BODY velocities mutually inconsistent until the first live physics frame.
// V14.66 therefore restores only geometric pose here; the existing
// DEFINITION_DYNAMICS call supplies the authoritative WORLD velocity vector.
typedef struct {
    double latitudeRad;
    double longitudeRad;
    double altitudeFt;
    double pitchRad;
    double bankRad;
    double headingTrueRad;
} ExactPoseNoAirspeedState;

// V14.92 setters for causal state that older September-17 Manual bundles did
// not record. These definitions do NOT include primary flight-control axes.
typedef struct { double mixture1; double propeller1; } EngineControlState;
typedef struct { double flapsHandleIndex; } FlapsHandleState;
typedef struct {
    double leftPercent;
    double rightPercent;
    double leftAngle;
    double rightAngle;
} FlapSurfaceState;
typedef struct { double leftMain; double rightMain; double center; } FuelTankLevelsState;
typedef struct { double x; double y; double z; } LinearBodyAccelerationState;
typedef struct { double x; double y; double z; } RotationBodyAccelerationState;
typedef struct {
    double fuelTankSelector1;
    double autopilotMaster;
    double autoCoordination;
    double spoilersHandlePosition;
    double gearHandlePosition;
    double brakeParkingPosition;
    double engineCombustion1;
} AuxConfigurationState;

// V14.95 extended causal-state audit. Kept separate from the legacy telemetry
// block so older callers remain byte-for-byte compatible.
#define V1495_MAX_PAYLOAD_STATIONS 32
#define V1495_FUEL_TANK_COUNT 11
typedef struct {
    double payloadStationCount;
    double payloadStationWeight[V1495_MAX_PAYLOAD_STATIONS];
    double totalWeightPitchMoi;
    double totalWeightRollMoi;
    double totalWeightYawMoi;
    double totalWeightCrossCoupledMoi;

    // Fuel levels: center, center2, center3, left main, left aux, left tip,
    // right main, right aux, right tip, external1, external2.
    double fuelTankLevel[V1495_FUEL_TANK_COUNT];
    double fuelTankCapacity[V1495_FUEL_TANK_COUNT];
    double fuelTankSelector1;
    double fuelTankSelector2;
    double fuelTankSelector3;
    double fuelTankSelector4;

    // Piston-engine causal controls / audit channels.
    double recipCowlFlapPosition1;
    double recipPrimer1;
    double recipAlternateAirPosition1;
    double recipLeftMagneto1;
    double recipRightMagneto1;
    double recipFuelAvailable1;
    double recipFuelTankSelector1;
    double recipFuelTanksUsed1;
    double generalEngStarter1;
    double generalEngFuelPumpSwitch1;
    double generalEngFuelPumpOn1;
    double generalEngDamagePercent1;
    double generalEngFailed1;
    double generalEngAntiIcePosition1;
    double propDeiceSwitch1;
    double engOnFire1;

    // Airframe/configuration/failure audit.
    double structuralIcePct;
    double pitotIcePct;
    double pitotHeat;
    double flapDamageBySpeed;
    double flapSpeedExceeded;
    double gearDamageBySpeed;
    double gearSpeedExceeded;
    double spoilersLeftPosition;
    double spoilersRightPosition;
    double gearCenterPosition;
    double gearLeftPosition;
    double gearRightPosition;
    double canopyOpen;
    double damageState;
    double simOnGround;
    double userInputEnabled;

    // Instrument state: does not drive trajectory, but removes the old D-key
    // heading-indicator ambiguity and makes the initial cockpit state auditable.
    double gyroHeadingRad;
    double gyroDriftErrorRad;
} ExtendedCausalStateV1495;

typedef struct {
    double cowlFlapPosition1;
    double primer1;
    double alternateAirPosition1;
    double leftMagneto1;
    double rightMagneto1;
    double fuelAvailable1;
    double recipFuelTankSelector1;
    double recipFuelTanksUsed1;
    double starter1;
    double gearCenterPosition;
    double gearLeftPosition;
    double gearRightPosition;
    double canopyOpen;
    double gyroHeadingRad;
    double damageState;
} ExtendedWritableStateV1495;

typedef struct { double values[8]; } EngineSupportStateV1496;
#pragma pack(pop)

static HANDLE hSimConnect = NULL;
static TelemetryData g_Telemetry = {0};
static ExtendedCausalStateV1495 g_ExtendedCausal = {0};
static volatile LONG g_ExtendedCausalSequence = 0;
static CRITICAL_SECTION g_CS;
static EngineSupportStateV1496 g_EngineSupport = {0};
static volatile LONG g_EngineSupportSequence = 0;
static volatile BOOL g_Connected = FALSE;
static volatile LONG g_TelemetrySequence = 0;
static volatile LONG g_WeatherSequence = 0;
static char g_GlobalWeatherMETAR[4096] = {0};
static BOOL g_CSInitialized = FALSE;

// Airport facility query state
static SIMCONNECT_DATA_FACILITY_AIRPORT g_FoundAirport = {0};
static volatile BOOL g_AirportFound = FALSE;
static char g_QueryICAO[16] = {0};

enum DATA_DEFINE_ID {
    DEFINITION_TELEMETRY = 1,
    DEFINITION_POSITION = 2,
    DEFINITION_DYNAMICS = 3,
    DEFINITION_EXACT_POSE = 4,
    DEFINITION_EXACT_POSE_NO_IAS = 5,
    DEFINITION_ENGINE_CONTROLS = 6,
    DEFINITION_FLAPS_HANDLE = 7,
    DEFINITION_FUEL_TANK_LEVELS = 8,
    DEFINITION_LINEAR_BODY_ACCEL = 9,
    DEFINITION_ROTATION_BODY_ACCEL = 10,
    DEFINITION_AUX_CONFIGURATION = 11,
    DEFINITION_FLAP_SURFACES = 12,
    DEFINITION_EXTENDED_CAUSAL = 13,
    DEFINITION_ALL_FUEL_TANKS = 14,
    DEFINITION_EXTENDED_WRITABLE = 15,
    DEFINITION_DYNAMICS_NO_RPM = 16,
    DEFINITION_AUX_NO_COMBUSTION = 17,
    DEFINITION_ENGINE_SUPPORT = 18,
    DEFINITION_INITIAL_FUEL_PRESSURE_V15 = 19,
    DEFINITION_WORLD_DYNAMICS_V15 = 20,
    DEFINITION_KINEMATICS_V1502 = 21,
    DEFINITION_FULL_KINEMATICS_V1505 = 22,
    DEFINITION_INITIAL_INSTRUMENTS_V1506 = 23,
    DEFINITION_INITIAL_MIXTURE_V1515 = 24,
    DEFINITION_INITIAL_PROPELLER_V1515 = 25,
    DEFINITION_INITIAL_TRIM_V1516 = 26,
    DEFINITION_ENGINE_SUPPORT_SET_BASE = 30,
    DEFINITION_CAUSAL_FIELD_BASE = 200,
    DEFINITION_PAYLOAD_BASE = 100
};
enum DATA_REQUEST_ID { REQUEST_TELEMETRY = 1, REQUEST_AIRPORT_LIST = 2, REQUEST_TELEMETRY_ONCE = 3, REQUEST_WEATHER_GLOB = 4, REQUEST_EXTENDED_CAUSAL = 5, REQUEST_EXTENDED_CAUSAL_ONCE = 6, REQUEST_ENGINE_SUPPORT = 7, REQUEST_ENGINE_SUPPORT_STREAM = 8 };
enum EVENT_ID {
    EVENT_PAUSE_ON = 100,
    EVENT_PAUSE_OFF = 101,
    EVENT_CLOCK_HOURS = 102,
    EVENT_CLOCK_MINUTES = 103,
    EVENT_ELEVATOR_SET = 301,
    EVENT_AILERON_SET = 302,
    EVENT_RUDDER_SET = 303,
    EVENT_ELEVATOR_TRIM_SET = 304,
    EVENT_THROTTLE_SET = 305,
    EVENT_ENGINE_AUTO_START = 401,
    EVENT_ENGINE_PUMP_TOGGLE = 402,
    EVENT_ENGINE_ALTERNATOR_TOGGLE = 403,
    EVENT_ENGINE_VALVE_TOGGLE = 404
};


static void DecodeTaggedScalars(const unsigned char* data, size_t bytes, DWORD datums,
    double* values, size_t count) {
    for (size_t i = 0; i < count; ++i) values[i] = std::numeric_limits<double>::quiet_NaN();
    size_t offset = 0;
    for (DWORD i = 0; i < datums; ++i) {
        if (bytes - offset < sizeof(DWORD) + sizeof(double)) break;
        DWORD id; double value;
        memcpy(&id, data + offset, sizeof(id)); offset += sizeof(id);
        memcpy(&value, data + offset, sizeof(value)); offset += sizeof(value);
        if (id < count && std::isfinite(value)) values[id] = value;
    }
}

static volatile LONG g_SimulationRunningV1509 = -1;
static bool g_PauseSubscribedV1509 = false;

#include "verification_snapshot_v1.h"

void CALLBACK MyDispatchProc(SIMCONNECT_RECV* pData, DWORD cbData, void* pContext) {
    if (!pData) return;
    
    switch (pData->dwID) {
    case SIMCONNECT_RECV_ID_OPEN: {
        SIMCONNECT_RECV_OPEN* pOpen = (SIMCONNECT_RECV_OPEN*)pData;
        printf("[CPP BRIDGE] ON RECV OPEN FIRED! AppName: %s\n", pOpen->szApplicationName);
        fflush(stdout);
        g_Connected = TRUE;
        g_PauseSubscribedV1509 = false;
        HRESULT hr = SimConnect_RequestDataOnSimObject(hSimConnect, REQUEST_TELEMETRY, DEFINITION_TELEMETRY,
            SIMCONNECT_OBJECT_ID_USER, SIMCONNECT_PERIOD_SIM_FRAME, SIMCONNECT_DATA_REQUEST_FLAG_DEFAULT, 0, 0, 0);
        SimConnect_RequestDataOnSimObject(hSimConnect, REQUEST_ENGINE_SUPPORT_STREAM, DEFINITION_ENGINE_SUPPORT,
            SIMCONNECT_OBJECT_ID_USER, SIMCONNECT_PERIOD_SIM_FRAME, SIMCONNECT_DATA_REQUEST_FLAG_DEFAULT, 0, 4, 0);
        HRESULT hrExt = SimConnect_RequestDataOnSimObject(hSimConnect, REQUEST_EXTENDED_CAUSAL, DEFINITION_EXTENDED_CAUSAL,
            SIMCONNECT_OBJECT_ID_USER, SIMCONNECT_PERIOD_SIM_FRAME, SIMCONNECT_DATA_REQUEST_FLAG_TAGGED, 0, 4, 0);
        printf("[CPP BRIDGE] RequestDataOnSimObject HRESULT: 0x%08X EXT=0x%08X\n", (unsigned int)hr, (unsigned int)hrExt);
        fflush(stdout);
        break;
    }
    case SIMCONNECT_RECV_ID_EVENT: {
        auto* event = reinterpret_cast<SIMCONNECT_RECV_EVENT*>(pData);
        if (event->uEventID == 1509) InterlockedExchange(&g_SimulationRunningV1509, event->dwData ? 0 : 1);
        break;
    }
    case SIMCONNECT_RECV_ID_QUIT:
        printf("[CPP BRIDGE] ON RECV QUIT FIRED!\n");
        fflush(stdout);
        g_Connected = FALSE;
        InvalidateVerification();
        break;
    case SIMCONNECT_RECV_ID_EXCEPTION: {
        SIMCONNECT_RECV_EXCEPTION* pExc = (SIMCONNECT_RECV_EXCEPTION*)pData;
        if (g_PacketDescriptions.count(pExc->dwSendID) &&
            g_PacketDescriptions[pExc->dwSendID].find("definition=19000 ") == 0) {
            EnterCriticalSection(&g_CS);
            g_VerificationRegistered = false;
            LeaveCriticalSection(&g_CS);
        }
        printf("[CPP BRIDGE] ON RECV EXCEPTION: %lu index: %lu send: %lu %s\n",
            pExc->dwException, pExc->dwIndex, pExc->dwSendID,
            g_PacketDescriptions.count(pExc->dwSendID) ? g_PacketDescriptions[pExc->dwSendID].c_str() : "");
        fflush(stdout);
        break;
    }
    case SIMCONNECT_RECV_ID_SIMOBJECT_DATA: {
        SIMCONNECT_RECV_SIMOBJECT_DATA* pObjData = (SIMCONNECT_RECV_SIMOBJECT_DATA*)pData;
        if (pObjData->dwDefineID == VERIFICATION_DEFINITION) {
            ReceiveVerification(pObjData, cbData);
            break;
        }
        if (pObjData->dwRequestID == REQUEST_TELEMETRY || pObjData->dwRequestID == REQUEST_TELEMETRY_ONCE) {
            TelemetryData* pDataStruct = (TelemetryData*)&pObjData->dwData;
            if (g_CSInitialized) {
                EnterCriticalSection(&g_CS);
                g_Telemetry = *pDataStruct;
                InterlockedIncrement(&g_TelemetrySequence);
                LeaveCriticalSection(&g_CS);
            }
        } else if (pObjData->dwRequestID == REQUEST_ENGINE_SUPPORT || pObjData->dwRequestID == REQUEST_ENGINE_SUPPORT_STREAM) {
            if (g_CSInitialized && cbData >= offsetof(SIMCONNECT_RECV_SIMOBJECT_DATA, dwData) + sizeof(EngineSupportStateV1496)) {
                EnterCriticalSection(&g_CS);
                memcpy(&g_EngineSupport, &pObjData->dwData, sizeof(g_EngineSupport));
                InterlockedIncrement(&g_EngineSupportSequence);
                LeaveCriticalSection(&g_CS);
            }
        } else if (pObjData->dwRequestID == REQUEST_EXTENDED_CAUSAL || pObjData->dwRequestID == REQUEST_EXTENDED_CAUSAL_ONCE) {
            if (g_CSInitialized && (pObjData->dwFlags & SIMCONNECT_DATA_REQUEST_FLAG_TAGGED)) {
                ExtendedCausalStateV1495 next;
                const size_t headerBytes = offsetof(SIMCONNECT_RECV_SIMOBJECT_DATA, dwData);
                if (cbData < headerBytes) break;
                DecodeTaggedScalars(reinterpret_cast<const unsigned char*>(&pObjData->dwData),
                    cbData - headerBytes, pObjData->dwDefineCount,
                    reinterpret_cast<double*>(&next), sizeof(next) / sizeof(double));
                EnterCriticalSection(&g_CS);
                g_ExtendedCausal = next;
                InterlockedIncrement(&g_ExtendedCausalSequence);
                LeaveCriticalSection(&g_CS);
            }
        }
        break;
    }
    case SIMCONNECT_RECV_ID_WEATHER_OBSERVATION: {
        SIMCONNECT_RECV_WEATHER_OBSERVATION* pWx = (SIMCONNECT_RECV_WEATHER_OBSERVATION*)pData;
        if (pWx->dwRequestID == REQUEST_WEATHER_GLOB && g_CSInitialized) {
            EnterCriticalSection(&g_CS);
            strncpy_s(g_GlobalWeatherMETAR, sizeof(g_GlobalWeatherMETAR), pWx->szMetar, _TRUNCATE);
            InterlockedIncrement(&g_WeatherSequence);
            LeaveCriticalSection(&g_CS);
        }
        break;
    }
    case SIMCONNECT_RECV_ID_AIRPORT_LIST: {
        SIMCONNECT_RECV_AIRPORT_LIST* pList = (SIMCONNECT_RECV_AIRPORT_LIST*)pData;
        for (DWORD i = 0; i < pList->dwArraySize; i++) {
            SIMCONNECT_DATA_FACILITY_AIRPORT* ap = &pList->rgData[i];
            // Match by ICAO ident (case-insensitive)
            if (_stricmp(ap->Icao, g_QueryICAO) == 0) {
                if (g_CSInitialized) {
                    EnterCriticalSection(&g_CS);
                    g_FoundAirport = *ap;
                    g_AirportFound = TRUE;
                    LeaveCriticalSection(&g_CS);
                    // Altitude from P3D facility list is in meters; convert to feet for display
                    printf("[CPP BRIDGE] Airport found: %s  Lat=%.6f  Lon=%.6f  Alt=%.1f m (%.1f ft)\n",
                        ap->Icao, ap->Latitude, ap->Longitude, ap->Altitude, ap->Altitude * 3.28084);
                    fflush(stdout);
                }
                break;
            }
        }
        break;
    }
    }
}

extern "C" {
    // Used by the Python wrapper to prevent loading an older DLL with the
    // expanded V14.30 TelemetryData layout.
    __declspec(dllexport) int GetBridgeVersion() {
        return 1517;
    }

    __declspec(dllexport) int ConnectSim() {
        if (!g_CSInitialized) {
            InitializeCriticalSection(&g_CS);
            g_CSInitialized = TRUE;
        }
        
        HRESULT hr = SimConnect_Open(&hSimConnect, "Piston Engine Logger Bridge", NULL, 0, 0, SIMCONNECT_OPEN_CONFIGINDEX_LOCAL);
        if (FAILED(hr)) {
            hr = SimConnect_Open(&hSimConnect, "Piston Engine Logger Bridge", NULL, 0, 0, 0);
        }

        printf("[CPP BRIDGE] SimConnect_Open result: 0x%08X (hSimConnect: %p)\n", (unsigned int)hr, hSimConnect);
        fflush(stdout);

        if (SUCCEEDED(hr) && hSimConnect) {
            RegisterVerification();
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "TITLE", NULL, SIMCONNECT_DATATYPE_STRING256, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "NUMBER OF ENGINES", "number", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "AIRSPEED INDICATED", "knots", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "VERTICAL SPEED", "feet per minute", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "INDICATED ALTITUDE", "feet", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "PLANE HEADING DEGREES MAGNETIC", "degrees", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "PLANE PITCH DEGREES", "degrees", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "PLANE BANK DEGREES", "degrees", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "PLANE HEADING DEGREES TRUE", "degrees", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            
            // Throttle is explicitly allowed as read-only engine instrumentation.
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "GENERAL ENG THROTTLE LEVER POSITION:1", "percent", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "GENERAL ENG THROTTLE LEVER POSITION:2", "percent", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "GENERAL ENG THROTTLE LEVER POSITION:3", "percent", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "GENERAL ENG THROTTLE LEVER POSITION:4", "percent", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);

            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "PROP RPM:1", "rpm", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "PROP RPM:2", "rpm", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "PROP RPM:3", "rpm", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "PROP RPM:4", "rpm", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);

            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "GENERAL ENG RPM:1", "rpm", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "GENERAL ENG RPM:2", "rpm", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "GENERAL ENG RPM:3", "rpm", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "GENERAL ENG RPM:4", "rpm", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);

            // New data definitions for Weight & Environment
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "TOTAL WEIGHT", "kilograms", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "FUEL TOTAL QUANTITY WEIGHT", "kilograms", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "CG LATERAL", "feet", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "CG PERCENT", "percent", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "KOHLSMAN SETTING MB", "millibars", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "AMBIENT TEMPERATURE", "celsius", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "AMBIENT WIND DIRECTION", "degrees", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "AMBIENT WIND VELOCITY", "knots", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);

            // New Flight Parameters
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "GROUND VELOCITY", "knots", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "AIRSPEED MACH", "mach", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "PLANE ALTITUDE", "feet", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "PLANE ALT ABOVE GROUND", "feet", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "GROUND ALTITUDE", "feet", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "INCIDENCE ALPHA", "degrees", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "INCIDENCE BETA", "degrees", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "GPS GROUND TRUE TRACK", "degrees", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "ROTATION VELOCITY BODY Z", "Degrees per second", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "ROTATION VELOCITY BODY X", "Degrees per second", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "ROTATION VELOCITY BODY Y", "Degrees per second", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "ACCELERATION BODY X", "feet per second squared", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "ACCELERATION BODY Y", "feet per second squared", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "ACCELERATION BODY Z", "feet per second squared", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "ROTOR RPM:1", "percent", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "GENERAL ENG FUEL FLOW PPH:1", "pounds per hour", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "ENG TORQUE:1", "percent", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "GENERAL ENG EXHAUST GAS TEMPERATURE:1", "celsius", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);

            // Control surface positions
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "SIM ON GROUND", "bool", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED); // Reserved ABI slot: no controller query.
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "SIM ON GROUND", "bool", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED); // Reserved ABI slot: no controller query.
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "SIM ON GROUND", "bool", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED); // Reserved ABI slot: no controller query.
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "ELEVATOR TRIM POSITION", "radians", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED); // Authorized read-only trim diagnostic.


            // V14.30 exact dynamic-state and control-path telemetry.
            // P3D v5 SDK marks WORLD velocity, BODY rotation velocity, control deflections,
            // actual control-surface deflections, ELEVATOR TRIM POSITION, AILERON TRIM PCT
            // and RUDDER TRIM PCT as SimConnect-accessible variables.
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "VELOCITY WORLD X", "feet per second", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "VELOCITY WORLD Y", "feet per second", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "VELOCITY WORLD Z", "feet per second", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "VELOCITY BODY X", "feet per second", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "VELOCITY BODY Y", "feet per second", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "VELOCITY BODY Z", "feet per second", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "ROTATION VELOCITY BODY X", "radians per second", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "ROTATION VELOCITY BODY Y", "radians per second", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "ROTATION VELOCITY BODY Z", "radians per second", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "ELEVATOR DEFLECTION", "radians", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED); // Authorized read-only aerodynamic surface diagnostic.
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "AILERON LEFT DEFLECTION", "radians", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED); // Authorized read-only aerodynamic surface diagnostic.
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "AILERON RIGHT DEFLECTION", "radians", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED); // Authorized read-only aerodynamic surface diagnostic.
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "RUDDER DEFLECTION", "radians", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED); // Authorized read-only aerodynamic surface diagnostic.
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "ELEVATOR TRIM PCT", "percent over 100", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED); // Authorized read-only trim diagnostic.
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "AILERON TRIM PCT", "percent over 100", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED); // Authorized read-only trim diagnostic.
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "RUDDER TRIM PCT", "percent over 100", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED); // Authorized read-only trim diagnostic.
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "PLANE LATITUDE", "degrees", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "PLANE LONGITUDE", "degrees", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            // V14.87: first-live Manual/Automatic engine and air-relative evidence.
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "GENERAL ENG MIXTURE LEVER POSITION:1", "percent over 100", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED); // Authorized read-only engine diagnostic.
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "GENERAL ENG PROPELLER LEVER POSITION:1", "percent over 100", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED); // Authorized read-only engine diagnostic.
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "GENERAL ENG COMBUSTION:1", "bool", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "GENERAL ENG FUEL VALVE:1", "bool", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "GENERAL ENG FUEL PRESSURE:1", "psi", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "RECIP ENG MANIFOLD PRESSURE:1", "psi", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "RELATIVE WIND VELOCITY BODY X", "feet per second", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "RELATIVE WIND VELOCITY BODY Y", "feet per second", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "RELATIVE WIND VELOCITY BODY Z", "feet per second", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);

            // V14.92: configuration, environment, rotational-acceleration and
            // mass-distribution evidence. All are appended after the V14.87
            // telemetry prefix to keep older wrappers ABI-safe.
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "SIM ON GROUND", "bool", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED); // Reserved ABI slot: no controller query.
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "SIM ON GROUND", "bool", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED); // Reserved ABI slot: no controller query.
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "FLAPS NUM HANDLE POSITIONS", "number", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "TRAILING EDGE FLAPS LEFT PERCENT", "percent over 100", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "TRAILING EDGE FLAPS RIGHT PERCENT", "percent over 100", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "TRAILING EDGE FLAPS LEFT ANGLE", "radians", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "TRAILING EDGE FLAPS RIGHT ANGLE", "radians", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "AMBIENT DENSITY", "slugs per cubic feet", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "AMBIENT PRESSURE", "millibars", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "AIRCRAFT WIND X", "knots", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "AIRCRAFT WIND Y", "knots", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "AIRCRAFT WIND Z", "knots", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "ROTATION ACCELERATION BODY X", "radians per second squared", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "ROTATION ACCELERATION BODY Y", "radians per second squared", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "ROTATION ACCELERATION BODY Z", "radians per second squared", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "FUEL TANK LEFT MAIN LEVEL", "percent over 100", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "FUEL TANK RIGHT MAIN LEVEL", "percent over 100", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "FUEL TANK CENTER LEVEL", "percent over 100", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "FUEL TANK SELECTOR:1", "enum", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "SIM ON GROUND", "bool", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED); // Reserved ABI slot: no controller query.
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "SIM ON GROUND", "bool", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED); // Reserved ABI slot: no controller query.
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "SIM ON GROUND", "bool", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED); // Reserved ABI slot: no controller query.
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "SIM ON GROUND", "bool", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED); // Reserved ABI slot: no controller query.
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "SIM ON GROUND", "bool", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED); // Reserved ABI slot: no controller query.

            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "PROP BETA:1", "radians", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "ENG TORQUE:1", "foot pounds", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "ENG FUEL FLOW PPH:1", "pounds per hour", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "ENG EXHAUST GAS TEMPERATURE:1", "celsius", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_TELEMETRY, "RECIP MIXTURE RATIO:1", "ratio", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            // Separate set-definition used ONLY while P3D is paused immediately before
            // Automatic replay.  It never writes elevator/aileron/rudder position.
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_DYNAMICS, "VELOCITY WORLD X", "feet per second", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_DYNAMICS, "VELOCITY WORLD Y", "feet per second", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_DYNAMICS, "VELOCITY WORLD Z", "feet per second", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);

            // V14.68: BODY X/Y/Z are settable in the P3D v5 SDK.
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_DYNAMICS, "VELOCITY BODY X", "feet per second", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_DYNAMICS, "VELOCITY BODY Y", "feet per second", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_DYNAMICS, "VELOCITY BODY Z", "feet per second", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);

            // V14.70: direct restoration of the recorded VSI/instrument state.
            // P3D v5 SDK units for VERTICAL SPEED are feet per second.
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_DYNAMICS, "VERTICAL SPEED", "feet per second", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);

            TrackedAddToDataDefinition(hSimConnect, DEFINITION_DYNAMICS, "ROTATION VELOCITY BODY X", "radians per second", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_DYNAMICS, "ROTATION VELOCITY BODY Y", "radians per second", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_DYNAMICS, "ROTATION VELOCITY BODY Z", "radians per second", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_DYNAMICS, "ELEVATOR TRIM POSITION", "radians", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_DYNAMICS, "AILERON TRIM PCT", "percent over 100", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_DYNAMICS, "RUDDER TRIM PCT", "percent over 100", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);

            // V14.70: PROP RPM:1 is settable in the P3D v5 SDK.
            // GENERAL ENG RPM:1 is read-only, so this is the legitimate
            // simulator-state path for seeding the coupled piston/prop shaft.
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_DYNAMICS, "PROP RPM:1", "rpm", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);

            // V14.92 causal state setters. The September-17 legacy-profile
            // system uses only these non-primary-control definitions.
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_ENGINE_CONTROLS, "GENERAL ENG MIXTURE LEVER POSITION:1", "percent over 100", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_ENGINE_CONTROLS, "GENERAL ENG PROPELLER LEVER POSITION:1", "percent over 100", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_INITIAL_MIXTURE_V1515, "GENERAL ENG MIXTURE LEVER POSITION:1", "percent over 100", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_INITIAL_PROPELLER_V1515, "GENERAL ENG PROPELLER LEVER POSITION:1", "percent over 100", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_FLAPS_HANDLE, "FLAPS HANDLE INDEX", "number", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_FLAP_SURFACES, "TRAILING EDGE FLAPS LEFT PERCENT", "percent over 100", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_FLAP_SURFACES, "TRAILING EDGE FLAPS RIGHT PERCENT", "percent over 100", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_FLAP_SURFACES, "TRAILING EDGE FLAPS LEFT ANGLE", "radians", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_FLAP_SURFACES, "TRAILING EDGE FLAPS RIGHT ANGLE", "radians", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_FUEL_TANK_LEVELS, "FUEL TANK LEFT MAIN LEVEL", "percent over 100", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_FUEL_TANK_LEVELS, "FUEL TANK RIGHT MAIN LEVEL", "percent over 100", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_FUEL_TANK_LEVELS, "FUEL TANK CENTER LEVEL", "percent over 100", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_LINEAR_BODY_ACCEL, "ACCELERATION BODY X", "feet per second squared", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_LINEAR_BODY_ACCEL, "ACCELERATION BODY Y", "feet per second squared", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_LINEAR_BODY_ACCEL, "ACCELERATION BODY Z", "feet per second squared", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_ROTATION_BODY_ACCEL, "ROTATION ACCELERATION BODY X", "radians per second squared", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_ROTATION_BODY_ACCEL, "ROTATION ACCELERATION BODY Y", "radians per second squared", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_ROTATION_BODY_ACCEL, "ROTATION ACCELERATION BODY Z", "radians per second squared", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_AUX_CONFIGURATION, "FUEL TANK SELECTOR:1", "enum", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_AUX_CONFIGURATION, "AUTOPILOT MASTER", "bool", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_AUX_CONFIGURATION, "AUTO COORDINATION", "bool", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_AUX_CONFIGURATION, "SPOILERS HANDLE POSITION", "position", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_AUX_CONFIGURATION, "GEAR HANDLE POSITION", "bool", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_AUX_CONFIGURATION, "BRAKE PARKING POSITION", "bool", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_AUX_CONFIGURATION, "GENERAL ENG COMBUSTION:1", "bool", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);

            // V14.95 extended causal-state / inertia / failure audit.
            DWORD extendedDatumId = 0;
            auto addExtendedField = [&](const char* name, const char* units) {
                DWORD id = extendedDatumId++;
                HRESULT hr = TrackedAddToDataDefinition(hSimConnect, DEFINITION_EXTENDED_CAUSAL,
                    name, units, SIMCONNECT_DATATYPE_FLOAT64, 0.0f, id);
                if (FAILED(hr)) printf("[EXTENDED FIELD UNAVAILABLE] %s HRESULT=0x%08X\n", name, (unsigned int)hr);
            };
            addExtendedField("PAYLOAD STATION COUNT", "number");
            for (int i = 1; i <= V1495_MAX_PAYLOAD_STATIONS; ++i) { char n[64]; sprintf_s(n, "PAYLOAD STATION WEIGHT:%d", i); addExtendedField(n, "pounds"); }
            ++extendedDatumId; // TOTAL WEIGHT PITCH MOI: unavailable on this server; preserve tagged ID.
            ++extendedDatumId; // TOTAL WEIGHT ROLL MOI: unavailable on this server; preserve tagged ID.
            ++extendedDatumId; // TOTAL WEIGHT YAW MOI: unavailable on this server; preserve tagged ID.
            ++extendedDatumId; // TOTAL WEIGHT CROSS COUPLED MOI: unavailable on this server; preserve tagged ID.
            addExtendedField("FUEL TANK CENTER LEVEL", "percent over 100");
            addExtendedField("FUEL TANK CENTER2 LEVEL", "percent over 100");
            addExtendedField("FUEL TANK CENTER3 LEVEL", "percent over 100");
            addExtendedField("FUEL TANK LEFT MAIN LEVEL", "percent over 100");
            addExtendedField("FUEL TANK LEFT AUX LEVEL", "percent over 100");
            addExtendedField("FUEL TANK LEFT TIP LEVEL", "percent over 100");
            addExtendedField("FUEL TANK RIGHT MAIN LEVEL", "percent over 100");
            addExtendedField("FUEL TANK RIGHT AUX LEVEL", "percent over 100");
            addExtendedField("FUEL TANK RIGHT TIP LEVEL", "percent over 100");
            addExtendedField("FUEL TANK EXTERNAL1 LEVEL", "percent over 100");
            addExtendedField("FUEL TANK EXTERNAL2 LEVEL", "percent over 100");
            addExtendedField("FUEL TANK CENTER CAPACITY", "gallons");
            addExtendedField("FUEL TANK CENTER2 CAPACITY", "gallons");
            addExtendedField("FUEL TANK CENTER3 CAPACITY", "gallons");
            addExtendedField("FUEL TANK LEFT MAIN CAPACITY", "gallons");
            addExtendedField("FUEL TANK LEFT AUX CAPACITY", "gallons");
            addExtendedField("FUEL TANK LEFT TIP CAPACITY", "gallons");
            addExtendedField("FUEL TANK RIGHT MAIN CAPACITY", "gallons");
            addExtendedField("FUEL TANK RIGHT AUX CAPACITY", "gallons");
            addExtendedField("FUEL TANK RIGHT TIP CAPACITY", "gallons");
            addExtendedField("FUEL TANK EXTERNAL1 CAPACITY", "gallons");
            addExtendedField("FUEL TANK EXTERNAL2 CAPACITY", "gallons");
            addExtendedField("FUEL TANK SELECTOR:1", "enum");
            addExtendedField("FUEL TANK SELECTOR:2", "enum");
            addExtendedField("FUEL TANK SELECTOR:3", "enum");
            addExtendedField("FUEL TANK SELECTOR:4", "enum");
            addExtendedField("RECIP ENG COWL FLAP POSITION:1", "percent");
            addExtendedField("RECIP ENG PRIMER:1", "bool");
            addExtendedField("RECIP ENG ALTERNATE AIR POSITION:1", "position");
            addExtendedField("RECIP ENG LEFT MAGNETO:1", "bool");
            addExtendedField("RECIP ENG RIGHT MAGNETO:1", "bool");
            addExtendedField("RECIP ENG FUEL AVAILABLE:1", "bool");
            addExtendedField("RECIP ENG FUEL TANK SELECTOR:1", "enum");
            addExtendedField("RECIP ENG FUEL TANKS USED:1", "mask");
            addExtendedField("GENERAL ENG STARTER:1", "bool");
            addExtendedField("GENERAL ENG FUEL PUMP SWITCH:1", "bool");
            addExtendedField("GENERAL ENG FUEL PUMP ON:1", "bool");
            addExtendedField("GENERAL ENG DAMAGE PERCENT:1", "percent");
            addExtendedField("GENERAL ENG FAILED:1", "bool");
            addExtendedField("GENERAL ENG ANTI ICE POSITION:1", "bool");
            addExtendedField("PROP DEICE SWITCH:1", "bool");
            addExtendedField("ENG ON FIRE:1", "bool");
            addExtendedField("STRUCTURAL ICE PCT", "percent over 100");
            addExtendedField("PITOT ICE PCT", "percent over 100");
            addExtendedField("PITOT HEAT", "bool");
            addExtendedField("FLAP DAMAGE BY SPEED", "bool");
            addExtendedField("FLAP SPEED EXCEEDED", "bool");
            addExtendedField("GEAR DAMAGE BY SPEED", "bool");
            addExtendedField("GEAR SPEED EXCEEDED", "bool");
            addExtendedField("SPOILERS LEFT POSITION", "position");
            addExtendedField("SPOILERS RIGHT POSITION", "position");
            addExtendedField("GEAR CENTER POSITION", "percent over 100");
            addExtendedField("GEAR LEFT POSITION", "percent over 100");
            addExtendedField("GEAR RIGHT POSITION", "percent over 100");
            addExtendedField("CANOPY OPEN", "percent over 100");
            addExtendedField("DAMAGE STATE", "enum");
            addExtendedField("SIM ON GROUND", "bool");
            addExtendedField("USER INPUT ENABLED", "bool");
            addExtendedField("PLANE HEADING DEGREES GYRO", "radians");
            addExtendedField("GYRO DRIFT ERROR", "radians");
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_ALL_FUEL_TANKS, "FUEL TANK CENTER LEVEL", "percent over 100", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_ALL_FUEL_TANKS, "FUEL TANK CENTER2 LEVEL", "percent over 100", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_ALL_FUEL_TANKS, "FUEL TANK CENTER3 LEVEL", "percent over 100", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_ALL_FUEL_TANKS, "FUEL TANK LEFT MAIN LEVEL", "percent over 100", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_ALL_FUEL_TANKS, "FUEL TANK LEFT AUX LEVEL", "percent over 100", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_ALL_FUEL_TANKS, "FUEL TANK LEFT TIP LEVEL", "percent over 100", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_ALL_FUEL_TANKS, "FUEL TANK RIGHT MAIN LEVEL", "percent over 100", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_ALL_FUEL_TANKS, "FUEL TANK RIGHT AUX LEVEL", "percent over 100", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_ALL_FUEL_TANKS, "FUEL TANK RIGHT TIP LEVEL", "percent over 100", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_ALL_FUEL_TANKS, "FUEL TANK EXTERNAL1 LEVEL", "percent over 100", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_ALL_FUEL_TANKS, "FUEL TANK EXTERNAL2 LEVEL", "percent over 100", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_EXTENDED_WRITABLE, "RECIP ENG COWL FLAP POSITION:1", "percent", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_EXTENDED_WRITABLE, "RECIP ENG PRIMER:1", "bool", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_EXTENDED_WRITABLE, "RECIP ENG ALTERNATE AIR POSITION:1", "position", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_EXTENDED_WRITABLE, "RECIP ENG LEFT MAGNETO:1", "bool", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_EXTENDED_WRITABLE, "RECIP ENG RIGHT MAGNETO:1", "bool", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_EXTENDED_WRITABLE, "RECIP ENG FUEL AVAILABLE:1", "bool", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_EXTENDED_WRITABLE, "RECIP ENG FUEL TANK SELECTOR:1", "enum", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_EXTENDED_WRITABLE, "RECIP ENG FUEL TANKS USED:1", "mask", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_EXTENDED_WRITABLE, "GENERAL ENG STARTER:1", "bool", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_EXTENDED_WRITABLE, "GEAR CENTER POSITION", "percent over 100", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_EXTENDED_WRITABLE, "GEAR LEFT POSITION", "percent over 100", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_EXTENDED_WRITABLE, "GEAR RIGHT POSITION", "percent over 100", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_EXTENDED_WRITABLE, "CANOPY OPEN", "percent over 100", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_EXTENDED_WRITABLE, "PLANE HEADING DEGREES GYRO", "radians", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_EXTENDED_WRITABLE, "DAMAGE STATE", "enum", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            for (int i = 1; i <= V1495_MAX_PAYLOAD_STATIONS; ++i) { char n[64]; sprintf_s(n, "PAYLOAD STATION WEIGHT:%d", i); TrackedAddToDataDefinition(hSimConnect, (SIMCONNECT_DATA_DEFINITION_ID)(DEFINITION_PAYLOAD_BASE + i - 1), n, "pounds", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED); }

            TrackedAddToDataDefinition(hSimConnect, DEFINITION_DYNAMICS_NO_RPM, "VELOCITY WORLD X", "feet per second", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_DYNAMICS_NO_RPM, "VELOCITY WORLD Y", "feet per second", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_DYNAMICS_NO_RPM, "VELOCITY WORLD Z", "feet per second", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_DYNAMICS_NO_RPM, "VELOCITY BODY X", "feet per second", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_DYNAMICS_NO_RPM, "VELOCITY BODY Y", "feet per second", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_DYNAMICS_NO_RPM, "VELOCITY BODY Z", "feet per second", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_DYNAMICS_NO_RPM, "VERTICAL SPEED", "feet per second", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_DYNAMICS_NO_RPM, "ROTATION VELOCITY BODY X", "radians per second", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_DYNAMICS_NO_RPM, "ROTATION VELOCITY BODY Y", "radians per second", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_DYNAMICS_NO_RPM, "ROTATION VELOCITY BODY Z", "radians per second", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_DYNAMICS_NO_RPM, "ELEVATOR TRIM POSITION", "radians", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_DYNAMICS_NO_RPM, "AILERON TRIM PCT", "percent over 100", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_DYNAMICS_NO_RPM, "RUDDER TRIM PCT", "percent over 100", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_AUX_NO_COMBUSTION, "FUEL TANK SELECTOR:1", "enum", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_AUX_NO_COMBUSTION, "AUTOPILOT MASTER", "bool", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_AUX_NO_COMBUSTION, "AUTO COORDINATION", "bool", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_AUX_NO_COMBUSTION, "SPOILERS HANDLE POSITION", "position", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_AUX_NO_COMBUSTION, "GEAR HANDLE POSITION", "bool", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_AUX_NO_COMBUSTION, "BRAKE PARKING POSITION", "bool", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_WORLD_DYNAMICS_V15, "VELOCITY WORLD X", "feet per second", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_WORLD_DYNAMICS_V15, "VELOCITY WORLD Y", "feet per second", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_WORLD_DYNAMICS_V15, "VELOCITY WORLD Z", "feet per second", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_WORLD_DYNAMICS_V15, "ROTATION VELOCITY BODY X", "radians per second", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_WORLD_DYNAMICS_V15, "ROTATION VELOCITY BODY Y", "radians per second", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_WORLD_DYNAMICS_V15, "ROTATION VELOCITY BODY Z", "radians per second", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_WORLD_DYNAMICS_V15, "ELEVATOR TRIM POSITION", "radians", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_WORLD_DYNAMICS_V15, "AILERON TRIM PCT", "percent over 100", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_WORLD_DYNAMICS_V15, "RUDDER TRIM PCT", "percent over 100", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_INITIAL_FUEL_PRESSURE_V15, "GENERAL ENG FUEL PRESSURE:1", "psi", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_ENGINE_SUPPORT, "GENERAL ENG OIL TEMPERATURE:1", "celsius", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, (SIMCONNECT_DATA_DEFINITION_ID)(DEFINITION_ENGINE_SUPPORT_SET_BASE + 0), "GENERAL ENG OIL TEMPERATURE:1", "celsius", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_ENGINE_SUPPORT, "RECIP ENG CYLINDER HEAD TEMPERATURE:1", "celsius", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, (SIMCONNECT_DATA_DEFINITION_ID)(DEFINITION_ENGINE_SUPPORT_SET_BASE + 1), "RECIP ENG CYLINDER HEAD TEMPERATURE:1", "celsius", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_ENGINE_SUPPORT, "RECIP CARBURETOR TEMPERATURE:1", "celsius", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, (SIMCONNECT_DATA_DEFINITION_ID)(DEFINITION_ENGINE_SUPPORT_SET_BASE + 2), "RECIP CARBURETOR TEMPERATURE:1", "celsius", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_ENGINE_SUPPORT, "ELECTRICAL MASTER BATTERY", "bool", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, (SIMCONNECT_DATA_DEFINITION_ID)(DEFINITION_ENGINE_SUPPORT_SET_BASE + 3), "ELECTRICAL MASTER BATTERY", "bool", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_ENGINE_SUPPORT, "GENERAL ENG MASTER ALTERNATOR:1", "bool", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_ENGINE_SUPPORT, "GENERAL ENG FUEL PUMP SWITCH:1", "bool", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_ENGINE_SUPPORT, "GENERAL ENG FUEL VALVE:1", "bool", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_ENGINE_SUPPORT, "GENERAL ENG STARTER:1", "bool", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, (SIMCONNECT_DATA_DEFINITION_ID)(DEFINITION_ENGINE_SUPPORT_SET_BASE + 7), "GENERAL ENG STARTER:1", "bool", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_ENGINE_AUTO_START, "ENGINE_AUTO_START");
            SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_ENGINE_PUMP_TOGGLE, "TOGGLE_ELECT_FUEL_PUMP1");
            SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_ENGINE_ALTERNATOR_TOGGLE, "TOGGLE_ALTERNATOR1");
            SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_ENGINE_VALVE_TOGGLE, "TOGGLE_FUEL_VALVE_ENG1");

            TrackedAddToDataDefinition(hSimConnect, (SIMCONNECT_DATA_DEFINITION_ID)(DEFINITION_CAUSAL_FIELD_BASE + 0), "RECIP ENG COWL FLAP POSITION:1", "percent", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, (SIMCONNECT_DATA_DEFINITION_ID)(DEFINITION_CAUSAL_FIELD_BASE + 1), "RECIP ENG PRIMER:1", "bool", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, (SIMCONNECT_DATA_DEFINITION_ID)(DEFINITION_CAUSAL_FIELD_BASE + 2), "RECIP ENG ALTERNATE AIR POSITION:1", "position", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, (SIMCONNECT_DATA_DEFINITION_ID)(DEFINITION_CAUSAL_FIELD_BASE + 3), "RECIP ENG LEFT MAGNETO:1", "bool", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, (SIMCONNECT_DATA_DEFINITION_ID)(DEFINITION_CAUSAL_FIELD_BASE + 4), "RECIP ENG RIGHT MAGNETO:1", "bool", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, (SIMCONNECT_DATA_DEFINITION_ID)(DEFINITION_CAUSAL_FIELD_BASE + 5), "RECIP ENG FUEL AVAILABLE:1", "bool", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, (SIMCONNECT_DATA_DEFINITION_ID)(DEFINITION_CAUSAL_FIELD_BASE + 6), "RECIP ENG FUEL TANK SELECTOR:1", "enum", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, (SIMCONNECT_DATA_DEFINITION_ID)(DEFINITION_CAUSAL_FIELD_BASE + 7), "RECIP ENG FUEL TANKS USED:1", "mask", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, (SIMCONNECT_DATA_DEFINITION_ID)(DEFINITION_CAUSAL_FIELD_BASE + 8), "GENERAL ENG STARTER:1", "bool", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, (SIMCONNECT_DATA_DEFINITION_ID)(DEFINITION_CAUSAL_FIELD_BASE + 9), "GEAR CENTER POSITION", "percent over 100", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, (SIMCONNECT_DATA_DEFINITION_ID)(DEFINITION_CAUSAL_FIELD_BASE + 10), "GEAR LEFT POSITION", "percent over 100", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, (SIMCONNECT_DATA_DEFINITION_ID)(DEFINITION_CAUSAL_FIELD_BASE + 11), "GEAR RIGHT POSITION", "percent over 100", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, (SIMCONNECT_DATA_DEFINITION_ID)(DEFINITION_CAUSAL_FIELD_BASE + 12), "CANOPY OPEN", "percent over 100", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, (SIMCONNECT_DATA_DEFINITION_ID)(DEFINITION_CAUSAL_FIELD_BASE + 13), "PLANE HEADING DEGREES GYRO", "radians", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, (SIMCONNECT_DATA_DEFINITION_ID)(DEFINITION_CAUSAL_FIELD_BASE + 14), "DAMAGE STATE", "enum", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            // Register INITPOSITION definition for TERRAIN PRELOAD only.
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_POSITION, "INITIAL POSITION", NULL, SIMCONNECT_DATATYPE_INITPOSITION, 0.0f, SIMCONNECT_UNUSED);

            // V14.40 FINAL exact pose definition.
            // These variables are settable P3D SimVars.  This path is used only
            // AFTER terrain/scenery is loaded, so the final QTG t=0 correction
            // does not use SIMCONNECT_DATA_INITPOSITION again.
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_EXACT_POSE, "PLANE LATITUDE", "radians", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_EXACT_POSE, "PLANE LONGITUDE", "radians", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_EXACT_POSE, "PLANE ALTITUDE", "feet", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_EXACT_POSE, "PLANE PITCH DEGREES", "radians", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_EXACT_POSE, "PLANE BANK DEGREES", "radians", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_EXACT_POSE, "PLANE HEADING DEGREES TRUE", "radians", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_EXACT_POSE, "AIRSPEED INDICATED", "knots", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);

            // V14.66 FINAL exact-pose definition WITHOUT AIRSPEED INDICATED.
            // Terrain preload still uses SIMCONNECT_DATA_INITPOSITION with airspeed.
            // After scenery is loaded, geometric pose and WORLD dynamics are restored
            // independently so P3D does not overwrite the Manual BODY-velocity state.
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_EXACT_POSE_NO_IAS, "PLANE LATITUDE", "radians", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_EXACT_POSE_NO_IAS, "PLANE LONGITUDE", "radians", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_EXACT_POSE_NO_IAS, "PLANE ALTITUDE", "feet", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_EXACT_POSE_NO_IAS, "PLANE PITCH DEGREES", "radians", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_EXACT_POSE_NO_IAS, "PLANE BANK DEGREES", "radians", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_EXACT_POSE_NO_IAS, "PLANE HEADING DEGREES TRUE", "radians", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);

            TrackedAddToDataDefinition(hSimConnect, DEFINITION_KINEMATICS_V1502, "VELOCITY WORLD X", "feet per second", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_KINEMATICS_V1502, "VELOCITY WORLD Y", "feet per second", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_KINEMATICS_V1502, "VELOCITY WORLD Z", "feet per second", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_KINEMATICS_V1502, "ROTATION VELOCITY BODY X", "radians per second", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_KINEMATICS_V1502, "ROTATION VELOCITY BODY Y", "radians per second", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_KINEMATICS_V1502, "ROTATION VELOCITY BODY Z", "radians per second", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);

            TrackedAddToDataDefinition(hSimConnect, DEFINITION_FULL_KINEMATICS_V1505, "VELOCITY WORLD X", "feet per second", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_FULL_KINEMATICS_V1505, "VELOCITY WORLD Y", "feet per second", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_FULL_KINEMATICS_V1505, "VELOCITY WORLD Z", "feet per second", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_FULL_KINEMATICS_V1505, "VELOCITY BODY X", "feet per second", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_FULL_KINEMATICS_V1505, "VELOCITY BODY Y", "feet per second", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_FULL_KINEMATICS_V1505, "VELOCITY BODY Z", "feet per second", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_FULL_KINEMATICS_V1505, "ROTATION VELOCITY BODY X", "radians per second", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_FULL_KINEMATICS_V1505, "ROTATION VELOCITY BODY Y", "radians per second", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_FULL_KINEMATICS_V1505, "ROTATION VELOCITY BODY Z", "radians per second", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);

            // Map pause/control/throttle events ONCE on connect.
            TrackedAddToDataDefinition(hSimConnect, DEFINITION_INITIAL_INSTRUMENTS_V1506, "VERTICAL SPEED", "feet per second", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, (SIMCONNECT_DATA_DEFINITION_ID)DEFINITION_INITIAL_TRIM_V1516, "ELEVATOR TRIM POSITION", "radians", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, (SIMCONNECT_DATA_DEFINITION_ID)DEFINITION_INITIAL_TRIM_V1516, "AILERON TRIM PCT", "percent over 100", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            TrackedAddToDataDefinition(hSimConnect, (SIMCONNECT_DATA_DEFINITION_ID)DEFINITION_INITIAL_TRIM_V1516, "RUDDER TRIM PCT", "percent over 100", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED);
            // Re-mapping PAUSE_ON / PAUSE_OFF on every call can generate
            // SIMCONNECT_EXCEPTION_EVENT_ID_DUPLICATE.
            SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_PAUSE_ON, "PAUSE_ON");
            SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_PAUSE_OFF, "PAUSE_OFF");

            // Map Control & Throttle Events once on connect
            SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_ELEVATOR_SET, "ELEVATOR_SET");
            SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_AILERON_SET, "AILERON_SET");
            SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_RUDDER_SET, "RUDDER_SET");
            SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_ELEVATOR_TRIM_SET, "ELEVATOR_TRIM_SET");
            SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_THROTTLE_SET, "THROTTLE_SET");

            // Daytime IDs must also be mapped once per connection.
            SimConnect_MapClientEventToSimEvent(hSimConnect, 203, "ZULU_HOURS_SET");
            SimConnect_MapClientEventToSimEvent(hSimConnect, 204, "CLOCK_HOURS_SET");

            return 1;
        }
        return 0;
    }

    __declspec(dllexport) void ProcessMessages() {
        if (hSimConnect) {
            SimConnect_CallDispatch(hSimConnect, MyDispatchProc, NULL);
        }
    }

    __declspec(dllexport) int IsSimConnected() {
        return g_Connected ? 1 : 0;
    }

    // Backward-compatible legacy export.  Older Python wrappers allocate only
    // the original telemetry prefix, so NEVER copy the V14.30 extension here.
    __declspec(dllexport) void GetTelemetryData(void* pOut) {
        if (pOut && g_CSInitialized) {
            EnterCriticalSection(&g_CS);
            memcpy(pOut, &g_Telemetry, offsetof(TelemetryData, velocityWorldX));
            LeaveCriticalSection(&g_CS);
        }
    }

    // V14.30 full-layout export used only by p3d_simconnect_v14_30.py.
    __declspec(dllexport) void GetTelemetryDataV1430(TelemetryData* pOut) {
        if (pOut && g_CSInitialized) {
            EnterCriticalSection(&g_CS);
            // Old wrappers allocate only the V14.70 prefix.
            memcpy(pOut, &g_Telemetry, offsetof(TelemetryData, mixtureLever1));
            LeaveCriticalSection(&g_CS);
        }
    }

    __declspec(dllexport) void GetTelemetryDataV1487(TelemetryData* pOut) {
        if (pOut && g_CSInitialized) {
            EnterCriticalSection(&g_CS);
            // A V14.87 caller allocates only through relativeWindBodyZ.
            memcpy(pOut, &g_Telemetry, offsetof(TelemetryData, flapsHandlePercent));
            LeaveCriticalSection(&g_CS);
        }
    }

    __declspec(dllexport) void GetTelemetryDataV1492(TelemetryData* pOut) {
        if (pOut && g_CSInitialized) {
            EnterCriticalSection(&g_CS);
            memcpy(pOut, &g_Telemetry, offsetof(TelemetryData, propBetaDiagnostic));
            LeaveCriticalSection(&g_CS);
        }
    }
    __declspec(dllexport) void GetTelemetryDataV1507(TelemetryData* pOut) {
        if (pOut && g_CSInitialized) {
            EnterCriticalSection(&g_CS);
            *pOut = g_Telemetry;
            LeaveCriticalSection(&g_CS);
        }
    }

    __declspec(dllexport) LONG GetTelemetrySequence() {
        return InterlockedCompareExchange(&g_TelemetrySequence, 0, 0);
    }

    __declspec(dllexport) int RequestTelemetryOnce() {
        if (!hSimConnect) return 0;
        HRESULT hr = SimConnect_RequestDataOnSimObject(
            hSimConnect, REQUEST_TELEMETRY_ONCE, DEFINITION_TELEMETRY,
            SIMCONNECT_OBJECT_ID_USER, SIMCONNECT_PERIOD_ONCE,
            SIMCONNECT_DATA_REQUEST_FLAG_DEFAULT, 0, 0, 0
        );
        return SUCCEEDED(hr) ? 1 : 0;
    }

    __declspec(dllexport) int RequestSimulationRunningV1509() {
        if (!hSimConnect) return 0;
        if (g_PauseSubscribedV1509) SimConnect_UnsubscribeFromSystemEvent(hSimConnect, 1509);
        InterlockedExchange(&g_SimulationRunningV1509, -1);
        HRESULT hr=SimConnect_SubscribeToSystemEvent(hSimConnect, 1509, "Pause");
        g_PauseSubscribedV1509=SUCCEEDED(hr);
        return SUCCEEDED(hr) ? 1 : 0;
    }
    __declspec(dllexport) int GetSimulationRunningV1509() {
        return static_cast<int>(InterlockedCompareExchange(&g_SimulationRunningV1509, -1, -1));
    }

    __declspec(dllexport) void PauseSim() {
        if (hSimConnect) {
            HRESULT hr = SimConnect_TransmitClientEvent(
                hSimConnect,
                SIMCONNECT_OBJECT_ID_USER,
                EVENT_PAUSE_ON,
                1,
                SIMCONNECT_GROUP_PRIORITY_HIGHEST,
                SIMCONNECT_EVENT_FLAG_GROUPID_IS_PRIORITY
            );
            printf("[CPP BRIDGE V14.30.2] PauseSim transmit HRESULT: 0x%08X\n", (unsigned int)hr);
            fflush(stdout);
        }
    }

    __declspec(dllexport) void UnpauseSim() {
        if (hSimConnect) {
            HRESULT hr = SimConnect_TransmitClientEvent(
                hSimConnect,
                SIMCONNECT_OBJECT_ID_USER,
                EVENT_PAUSE_OFF,
                1,
                SIMCONNECT_GROUP_PRIORITY_HIGHEST,
                SIMCONNECT_EVENT_FLAG_GROUPID_IS_PRIORITY
            );
            printf("[CPP BRIDGE V14.30.2] UnpauseSim transmit HRESULT: 0x%08X\n", (unsigned int)hr);
            fflush(stdout);
        }
    }

    __declspec(dllexport) void SetDaytime() {
        if (hSimConnect) {
            // 1. Kneeboard / Environment action for Day

            // 2. Set Time Day key event

            // 3. Set Zulu Hours to 07:00 UTC (12:00 PM local in Pakistan UTC+5)
            SimConnect_TransmitClientEvent(hSimConnect, SIMCONNECT_OBJECT_ID_USER, 203, 7, SIMCONNECT_GROUP_PRIORITY_HIGHEST, SIMCONNECT_EVENT_FLAG_GROUPID_IS_PRIORITY);

            // 4. Set Clock Hours to 12
            SimConnect_TransmitClientEvent(hSimConnect, SIMCONNECT_OBJECT_ID_USER, 204, 12, SIMCONNECT_GROUP_PRIORITY_HIGHEST, SIMCONNECT_EVENT_FLAG_GROUPID_IS_PRIORITY);

            printf("[CPP BRIDGE] SetDaytime: Transmitted supported ZULU 07:00 and local 12:00 clock events.\n");
            fflush(stdout);
        }
    }

    __declspec(dllexport) void SetUserPosition(SIMCONNECT_DATA_INITPOSITION* pPos) {
        if (hSimConnect && pPos) {
            HRESULT hr = SimConnect_SetDataOnSimObject(
                hSimConnect,
                DEFINITION_POSITION,
                SIMCONNECT_OBJECT_ID_USER,
                0,
                0,
                sizeof(SIMCONNECT_DATA_INITPOSITION),
                pPos
            );
            printf("[CPP BRIDGE] SetUserPosition HRESULT: 0x%08X (Lat=%.4f Lon=%.4f Alt=%.0f Hdg=%.1f)\n",
                (unsigned int)hr, pPos->Latitude, pPos->Longitude, pPos->Altitude, pPos->Heading);
            fflush(stdout);
        }
    }

    __declspec(dllexport) void QueryAirportByICAO(const char* icao) {
        if (!icao) return;
        strncpy_s(g_QueryICAO, sizeof(g_QueryICAO), icao, _TRUNCATE);
        g_AirportFound = FALSE;
        if (hSimConnect) {
            HRESULT hr = SimConnect_RequestFacilitiesList(hSimConnect, SIMCONNECT_FACILITY_LIST_TYPE_AIRPORT, REQUEST_AIRPORT_LIST);
            printf("[CPP BRIDGE] QueryAirportByICAO('%s') - RequestFacilitiesList HRESULT: 0x%08X\n",
                g_QueryICAO, (unsigned int)hr);
            fflush(stdout);
        }
    }

    __declspec(dllexport) int GetAirportPosition(double* lat, double* lon, double* alt) {
        if (!lat || !lon || !alt) return 0;
        if (g_CSInitialized) {
            EnterCriticalSection(&g_CS);
            BOOL found = g_AirportFound;
            if (found) {
                *lat = g_FoundAirport.Latitude;
                *lon = g_FoundAirport.Longitude;
                *alt = g_FoundAirport.Altitude;
            }
            LeaveCriticalSection(&g_CS);
            return found ? 1 : 0;
        }
        return 0;
    }

    __declspec(dllexport) void SetElevator(double pos) {
        // Disabled: all controller commands must go through Brunner / CLS2Sim.
    }

    __declspec(dllexport) void SetAileron(double pos) {
        // Disabled: all controller commands must go through Brunner / CLS2Sim.
    }

    __declspec(dllexport) void SetRudder(double pos) {
        // Disabled: all controller commands must go through Brunner / CLS2Sim.
    }

    __declspec(dllexport) void SetElevatorTrim(double radians) {
        // Disabled: all controller commands must go through Brunner / CLS2Sim.
    }

    // Explicit preparation-only throttle path. General control exports stay disabled.
    __declspec(dllexport) int SetInitialThrottleV1504(double percent) {
        if (!hSimConnect || !std::isfinite(percent) || percent < 0.0 || percent > 100.0) return 0;
        DWORD value = (DWORD)std::lround(percent / 100.0 * 16383.0);
        return SUCCEEDED(SimConnect_TransmitClientEvent(hSimConnect, SIMCONNECT_OBJECT_ID_USER,
            EVENT_THROTTLE_SET, value, SIMCONNECT_GROUP_PRIORITY_HIGHEST,
            SIMCONNECT_EVENT_FLAG_GROUPID_IS_PRIORITY)) ? 1 : 0;
    }

    // Explicitly authorized throttle replay; no flight-axis or RPM writes.
    __declspec(dllexport) int SetReplayThrottleV1514(double percent) {
        return SetInitialThrottleV1504(percent);
    }

    __declspec(dllexport) void SetThrottle(double percent) {
        // Disabled: all controller commands must go through Brunner / CLS2Sim.
    }

    __declspec(dllexport) void SetControlInputs(double elevator, double aileron, double rudder, double trim) {
        // Disabled: all controller commands must go through Brunner / CLS2Sim.
    }

    // V14.40 FINAL exact-pose operation. P3D must be PAUSED when called.
    //
    // IMPORTANT: this does NOT use SIMCONNECT_DATA_INITPOSITION.
    // It writes ordinary settable aircraft SimVars after scenery is already
    // loaded, avoiding a second terrain-loader cycle at final QTG t=0.
    __declspec(dllexport) int SetExactPoseState(ExactPoseState* pState) {
        if (!hSimConnect || !pState) return 0;

        HRESULT hr = SimConnect_SetDataOnSimObject(
            hSimConnect,
            DEFINITION_EXACT_POSE,
            SIMCONNECT_OBJECT_ID_USER,
            0,
            0,
            sizeof(ExactPoseState),
            pState
        );

        printf(
            "[CPP BRIDGE V14.40] SetExactPoseState HRESULT: 0x%08X "
            "LAT=%.8f rad LON=%.8f rad ALT=%.2f ft "
            "PITCH=%.6f rad BANK=%.6f rad HDG=%.6f rad IAS=%.2f kt\n",
            (unsigned int)hr,
            pState->latitudeRad, pState->longitudeRad, pState->altitudeFt,
            pState->pitchRad, pState->bankRad, pState->headingTrueRad,
            pState->airspeedIndicatedKts
        );
        fflush(stdout);
        return SUCCEEDED(hr) ? 1 : 0;
    }

    // V14.66 exact geometric pose operation with NO IAS write.
    // This is the required final/soft-hold pose path.  It prevents AIRSPEED
    // INDICATED from fighting the separately restored WORLD velocity vector.
    __declspec(dllexport) int SetExactPoseStateNoAirspeed(ExactPoseNoAirspeedState* pState) {
        if (!hSimConnect || !pState) return 0;

        HRESULT hr = SimConnect_SetDataOnSimObject(
            hSimConnect,
            DEFINITION_EXACT_POSE_NO_IAS,
            SIMCONNECT_OBJECT_ID_USER,
            0,
            0,
            sizeof(ExactPoseNoAirspeedState),
            pState
        );

        printf(
            "[CPP BRIDGE V14.66] SetExactPoseStateNoAirspeed HRESULT: 0x%08X "
            "LAT=%.8f rad LON=%.8f rad ALT=%.2f ft "
            "PITCH=%.6f rad BANK=%.6f rad HDG=%.6f rad\n",
            (unsigned int)hr,
            pState->latitudeRad, pState->longitudeRad, pState->altitudeFt,
            pState->pitchRad, pState->bankRad, pState->headingTrueRad
        );
        fflush(stdout);
        return SUCCEEDED(hr) ? 1 : 0;
    }

    // V14.30 initial-condition operation. P3D must be PAUSED when called.
    // This intentionally does NOT write ELEVATOR POSITION / AILERON POSITION /
    // RUDDER POSITION so BRUNNER/CLS2Sim remains the sole flight-control source.
    __declspec(dllexport) int SetInitialDynamicState(InitialDynamicState* pState) {
        // Disabled: all controller commands must go through Brunner / CLS2Sim.
        return 0;
    }


    // No shaft RPM or combustion flag is written by these operations.
    // One consistent WORLD velocity; BODY velocity and VSI are derived observations.
    __declspec(dllexport) int SetWorldInitialDynamicStateV15(InitialDynamicState* pState) {
        // Disabled: all controller commands must go through Brunner / CLS2Sim.
        return 0;
    }
    // Initial-condition seed only. Python calls while paused, never during replay.
    __declspec(dllexport) int SetKinematicStateV1502(const double* values) {
        if (!hSimConnect || !values) return 0;
        for (int i=0; i<6; ++i) if (!std::isfinite(values[i])) return 0;
        return SUCCEEDED(SimConnect_SetDataOnSimObject(hSimConnect, DEFINITION_KINEMATICS_V1502,
            SIMCONNECT_OBJECT_ID_USER, 0, 0, 6*sizeof(double), const_cast<double*>(values))) ? 1 : 0;
    }

    // Paused initialization only: instrument memories, independent of physical velocity.
    __declspec(dllexport) int SetInitialInstrumentsV1506(double vsiFps, double altitudeFt) {
        if (!hSimConnect || !std::isfinite(vsiFps) || !std::isfinite(altitudeFt)) return 0;
        // INDICATED ALTITUDE writes move the aircraft in P3D. Preserve physical altitude.
        double values[1] = {vsiFps};
        return SUCCEEDED(SimConnect_SetDataOnSimObject(hSimConnect, DEFINITION_INITIAL_INSTRUMENTS_V1506,
            SIMCONNECT_OBJECT_ID_USER, 0, 0, sizeof(values), values)) ? 1 : 0;
    }

    __declspec(dllexport) int SetFullKinematicStateV1505(const double* values) {
        if (!hSimConnect || !values) return 0;
        for (int i=0; i<9; ++i) if (!std::isfinite(values[i])) return 0;
        return SUCCEEDED(SimConnect_SetDataOnSimObject(hSimConnect, DEFINITION_FULL_KINEMATICS_V1505,
            SIMCONNECT_OBJECT_ID_USER, 0, 0, 9*sizeof(double), const_cast<double*>(values))) ? 1 : 0;
    }

    __declspec(dllexport) int SetInitialFuelPressureV15(double value) {
        if (!hSimConnect || !std::isfinite(value) || value < 0.0) return 0;
        return SUCCEEDED(SimConnect_SetDataOnSimObject(hSimConnect, DEFINITION_INITIAL_FUEL_PRESSURE_V15,
            SIMCONNECT_OBJECT_ID_USER, 0, 0, sizeof(value), &value)) ? 1 : 0;
    }
    __declspec(dllexport) int SetInitialDynamicStateNoRpmV1496(InitialDynamicState* pState) {
        // Disabled: all controller commands must go through Brunner / CLS2Sim.
        return 0;
    }
    __declspec(dllexport) int SetAuxConfigurationNoCombustionV1496(double fuelSelector, double autopilotMaster,
        double autoCoordination, double spoilersHandle, double gearHandle, double parkingBrake, double unusedCombustion) {
        // Disabled: all controller commands must go through Brunner / CLS2Sim.
        return 0;
    }
    __declspec(dllexport) LONG GetEngineSupportSequenceV1496() {
        return InterlockedCompareExchange(&g_EngineSupportSequence, 0, 0);
    }
    __declspec(dllexport) int RequestEngineSupportOnceV1496() {
        if (!hSimConnect) return 0;
        return SUCCEEDED(SimConnect_RequestDataOnSimObject(hSimConnect, REQUEST_ENGINE_SUPPORT,
            DEFINITION_ENGINE_SUPPORT, SIMCONNECT_OBJECT_ID_USER, SIMCONNECT_PERIOD_ONCE)) ? 1 : 0;
    }
    __declspec(dllexport) void GetEngineSupportStateV1496(EngineSupportStateV1496* outData) {
        if (!outData || !g_CSInitialized) return;
        EnterCriticalSection(&g_CS); *outData = g_EngineSupport; LeaveCriticalSection(&g_CS);
    }
    __declspec(dllexport) int SetEngineSupportValueV1496(int index, double value) {
        if (!hSimConnect || !(index == 0 || index == 1 || index == 2 || index == 3 || index == 7)) return 0;
        return SUCCEEDED(SimConnect_SetDataOnSimObject(hSimConnect,
            (SIMCONNECT_DATA_DEFINITION_ID)(DEFINITION_ENGINE_SUPPORT_SET_BASE + index),
            SIMCONNECT_OBJECT_ID_USER, 0, 0, sizeof(value), &value)) ? 1 : 0;
    }
    __declspec(dllexport) int SendEnginePreparationEventV1496(int action) {
        if (!hSimConnect) return 0;
        EVENT_ID eventId;
        switch (action) {
        case 0: eventId = EVENT_ENGINE_AUTO_START; break;
        case 1: eventId = EVENT_ENGINE_PUMP_TOGGLE; break;
        case 2: eventId = EVENT_ENGINE_ALTERNATOR_TOGGLE; break;
        case 3: eventId = EVENT_ENGINE_VALVE_TOGGLE; break;
        default: return 0;
        }
        return SUCCEEDED(SimConnect_TransmitClientEvent(hSimConnect, SIMCONNECT_OBJECT_ID_USER,
            eventId, 0, SIMCONNECT_GROUP_PRIORITY_HIGHEST, SIMCONNECT_EVENT_FLAG_GROUPID_IS_PRIORITY)) ? 1 : 0;
    }

    __declspec(dllexport) int SetAuxConfiguration(double fuelSelector, double autopilotMaster,
        double autoCoordination, double spoilersHandle, double gearHandle,
        double parkingBrake, double engineCombustion) {
        // Disabled: all controller commands must go through Brunner / CLS2Sim.
        return 0;
    }

    __declspec(dllexport) LONG GetWeatherSequence() {
        return InterlockedCompareExchange(&g_WeatherSequence, 0, 0);
    }

    __declspec(dllexport) int RequestGlobalWeatherMETAR() {
        if (!hSimConnect) return 0;
        HRESULT hr = SimConnect_WeatherRequestObservationAtStation(
            hSimConnect, REQUEST_WEATHER_GLOB, "GLOB");
        return SUCCEEDED(hr) ? 1 : 0;
    }

    __declspec(dllexport) int GetLastWeatherMETAR(char* outBuf, int outSize) {
        if (!outBuf || outSize <= 1 || !g_CSInitialized) return 0;
        EnterCriticalSection(&g_CS);
        strncpy_s(outBuf, (size_t)outSize, g_GlobalWeatherMETAR, _TRUNCATE);
        int ok = g_GlobalWeatherMETAR[0] ? 1 : 0;
        LeaveCriticalSection(&g_CS);
        return ok;
    }

    // V14.92: set non-primary-control causal state. These are intentionally
    // separate from SetInitialDynamicState so legacy-profile assumptions are
    // explicit and can be audited independently of the recorded rigid-body state.
    // Explicitly authorized initial engine levers only; -1 leaves an unrecorded lever alone.
    // Require a confirmed pause. Legacy controller exports stay disabled.
    __declspec(dllexport) int SetInitialTrimV1516(double elevatorTrimRad, double aileronTrimPct, double rudderTrimPct) {
        if (!hSimConnect) return 0;
        if (!std::isfinite(elevatorTrimRad) || !std::isfinite(aileronTrimPct) || !std::isfinite(rudderTrimPct)) return 0;
        double values[3] = {elevatorTrimRad, aileronTrimPct, rudderTrimPct};
        return SUCCEEDED(SimConnect_SetDataOnSimObject(hSimConnect, (SIMCONNECT_DATA_DEFINITION_ID)DEFINITION_INITIAL_TRIM_V1516,
            SIMCONNECT_OBJECT_ID_USER, 0, 0, 3*sizeof(double), values)) ? 1 : 0;
    }

    __declspec(dllexport) int SetInitialEngineLeversV1515(double mixturePercent, double propellerPercent) {
        if (!hSimConnect || InterlockedCompareExchange(&g_SimulationRunningV1509, -1, -1) != 0) return 0;
        const double values[2] = { mixturePercent, propellerPercent };
        for (int i=0; i<2; ++i) {
            if (!std::isfinite(values[i]) || (values[i] != -1.0 && (values[i]<0.0 || values[i]>100.0))) return 0;
        }
        for (int i=0; i<2; ++i) {
            if (values[i] == -1.0) continue;
            double normalized = values[i]/100.0;
            HRESULT hr = SimConnect_SetDataOnSimObject(hSimConnect,
                i == 0 ? DEFINITION_INITIAL_MIXTURE_V1515 : DEFINITION_INITIAL_PROPELLER_V1515,
                SIMCONNECT_OBJECT_ID_USER, 0, 0, sizeof(normalized), &normalized);
            if (FAILED(hr)) return 0;
        }
        return 1;
    }

    __declspec(dllexport) int SetEngineControls(double mixture1, double propeller1) {
        // Disabled: all controller commands must go through Brunner / CLS2Sim.
        return 0;
    }

    // One-time paused initial configuration; runtime control export stays disabled.
    __declspec(dllexport) int SetInitialFlapsV1517(double index) {
        if (!hSimConnect || InterlockedCompareExchange(&g_SimulationRunningV1509, -1, -1) != 0) return 0;
        if (!std::isfinite(index) || index < 0.0 || index > 10.0 || std::floor(index) != index) return 0;
        FlapsHandleState state = {index};
        return SUCCEEDED(SimConnect_SetDataOnSimObject(hSimConnect, DEFINITION_FLAPS_HANDLE,
            SIMCONNECT_OBJECT_ID_USER, 0, 0, sizeof(state), &state)) ? 1 : 0;
    }

    __declspec(dllexport) int SetFlapsHandleIndex(double index) {
        // Disabled: all controller commands must go through Brunner / CLS2Sim.
        return 0;
    }

    __declspec(dllexport) int SetFlapSurfaceState(double leftPercent, double rightPercent,
        double leftAngle, double rightAngle) {
        // Disabled: all controller commands must go through Brunner / CLS2Sim.
        return 0;
    }

    __declspec(dllexport) int SetFuelTankLevels(double leftMain, double rightMain, double center) {
        if (!hSimConnect) return 0;
        FuelTankLevelsState v = {
            max(0.0, min(1.0, leftMain)),
            max(0.0, min(1.0, rightMain)),
            max(0.0, min(1.0, center))
        };
        HRESULT hr = SimConnect_SetDataOnSimObject(hSimConnect, DEFINITION_FUEL_TANK_LEVELS,
            SIMCONNECT_OBJECT_ID_USER, 0, 0, sizeof(v), &v);
        return SUCCEEDED(hr) ? 1 : 0;
    }

    __declspec(dllexport) int SetLinearBodyAcceleration(double x, double y, double z) {
        if (!hSimConnect) return 0;
        LinearBodyAccelerationState v = { x, y, z };
        HRESULT hr = SimConnect_SetDataOnSimObject(hSimConnect, DEFINITION_LINEAR_BODY_ACCEL,
            SIMCONNECT_OBJECT_ID_USER, 0, 0, sizeof(v), &v);
        return SUCCEEDED(hr) ? 1 : 0;
    }

    __declspec(dllexport) int SetRotationBodyAcceleration(double x, double y, double z) {
        if (!hSimConnect) return 0;
        RotationBodyAccelerationState v = { x, y, z };
        HRESULT hr = SimConnect_SetDataOnSimObject(hSimConnect, DEFINITION_ROTATION_BODY_ACCEL,
            SIMCONNECT_OBJECT_ID_USER, 0, 0, sizeof(v), &v);
        return SUCCEEDED(hr) ? 1 : 0;
    }

    __declspec(dllexport) LONG GetExtendedCausalSequenceV1495() {
        return InterlockedCompareExchange(&g_ExtendedCausalSequence, 0, 0);
    }

    __declspec(dllexport) int RequestExtendedCausalOnceV1495() {
        if (!hSimConnect) return 0;
        HRESULT hr = SimConnect_RequestDataOnSimObject(hSimConnect, REQUEST_EXTENDED_CAUSAL_ONCE,
            DEFINITION_EXTENDED_CAUSAL, SIMCONNECT_OBJECT_ID_USER, SIMCONNECT_PERIOD_ONCE,
            SIMCONNECT_DATA_REQUEST_FLAG_TAGGED, 0, 0, 0);
        return SUCCEEDED(hr) ? 1 : 0;
    }

    __declspec(dllexport) void GetExtendedCausalStateV1495(ExtendedCausalStateV1495* outData) {
        if (!outData || !g_CSInitialized) return;
        EnterCriticalSection(&g_CS);
        *outData = g_ExtendedCausal;
        LeaveCriticalSection(&g_CS);
    }

    __declspec(dllexport) int SetPayloadStationWeightsV1495(const double* weightsLb, int count) {
        if (!hSimConnect || !weightsLb || count < 0) return 0;
        int n = min(count, V1495_MAX_PAYLOAD_STATIONS);
        for (int i = 0; i < n; ++i) {
            double v = max(0.0, weightsLb[i]);
            HRESULT hr = SimConnect_SetDataOnSimObject(hSimConnect,
                (SIMCONNECT_DATA_DEFINITION_ID)(DEFINITION_PAYLOAD_BASE + i),
                SIMCONNECT_OBJECT_ID_USER, 0, 0, sizeof(double), &v);
            if (FAILED(hr)) return 0;
        }
        return 1;
    }

    __declspec(dllexport) int SetAllFuelTankLevelsV1495(const double* levels, int count) {
        if (!hSimConnect || !levels || count < V1495_FUEL_TANK_COUNT) return 0;
        double v[V1495_FUEL_TANK_COUNT];
        for (int i = 0; i < V1495_FUEL_TANK_COUNT; ++i) v[i] = max(0.0, min(1.0, levels[i]));
        HRESULT hr = SimConnect_SetDataOnSimObject(hSimConnect, DEFINITION_ALL_FUEL_TANKS,
            SIMCONNECT_OBJECT_ID_USER, 0, 0, sizeof(v), &v);
        return SUCCEEDED(hr) ? 1 : 0;
    }

    __declspec(dllexport) int SetCausalFieldV1497(int index, double value) {
        // Disabled: all controller commands must go through Brunner / CLS2Sim.
        return 0;
    }

    __declspec(dllexport) int SetExtendedAircraftConfigurationV1495(
        double cowlFlap, double primer, double alternateAir, double leftMag, double rightMag,
        double fuelAvailable, double recipFuelSelector, double recipFuelTanksUsed,
        double starter, double gearCenter, double gearLeft, double gearRight,
        double canopyOpen, double gyroHeadingRad, double damageState) {
        // Disabled: all controller commands must go through Brunner / CLS2Sim.
        return 0;
    }

    __declspec(dllexport) int SetGlobalWeatherMETAR(const char* metar) {
        if (!hSimConnect || !metar || !metar[0]) return 0;
        // V14.95 deterministic weather injection.
        // P3D SDK recommends setting the observation and then selecting the
        // global/custom mode so the Weather UI/state is actually changed. The
        // extra observation/global pass is intentional: it removes a stale
        // Weather Theme (e.g. Fair Weather) and makes the GLOB observation the
        // authoritative air mass before the QTG release.
        HRESULT hr0 = SimConnect_WeatherSetModeCustom(hSimConnect);
        HRESULT hr1 = SimConnect_WeatherSetDynamicUpdateRate(hSimConnect, 0);
        HRESULT hr2 = SimConnect_WeatherSetObservation(hSimConnect, 0, metar);
        HRESULT hr3 = SimConnect_WeatherSetModeGlobal(hSimConnect);
        HRESULT hr4 = SimConnect_WeatherSetObservation(hSimConnect, 0, metar);
        HRESULT hr5 = SimConnect_WeatherSetModeGlobal(hSimConnect);
        printf("[CPP BRIDGE V14.95] Weather METAR='%s' HRESULTS=%08X/%08X/%08X/%08X/%08X/%08X\n",
            metar, (unsigned int)hr0, (unsigned int)hr1, (unsigned int)hr2,
            (unsigned int)hr3, (unsigned int)hr4, (unsigned int)hr5);
        fflush(stdout);
        return (SUCCEEDED(hr0) && SUCCEEDED(hr1) && SUCCEEDED(hr2) &&
                SUCCEEDED(hr3) && SUCCEEDED(hr4) && SUCCEEDED(hr5)) ? 1 : 0;
    }

    __declspec(dllexport) void DisconnectSim() {
        if (g_CSInitialized) EnterCriticalSection(&g_CS);
        InvalidateVerification();
        if (hSimConnect) {
            SimConnect_Close(hSimConnect);
            hSimConnect = NULL;
        }
        g_Connected = FALSE;
        if (g_CSInitialized) LeaveCriticalSection(&g_CS);
    }
}
