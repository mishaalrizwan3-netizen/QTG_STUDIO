// P3D v5 SimConnect x64: BRUNNER-only V4.1 telemetry and throttle (bank hold removed).
// IMPORTANT ABI: Must be paired with SMK_IAS_PID_BRUNNER_ONLY_V4_1.py.
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <Windows.h>
#include <SimConnect.h>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstddef>
#include <cstring>
#include <mutex>
#include <thread>
#include <chrono>
#include <algorithm>

#if defined(_MSC_VER)
#define API extern "C" __declspec(dllexport)
#else
#define API extern "C"
#endif

enum DefinitionID : DWORD { 
    DATA_FLIGHT = 1, 
    DATA_ELEVATOR = 2, 
    DATA_YOKE_Y = 3,
    DATA_THROTTLE = 5
};

enum EventID : DWORD {
    EVENT_PAUSE = 100,
    EVENT_SIM_START = 101,
    EVENT_SIM_STOP = 102
};

enum RequestID : DWORD { 
    REQUEST_FLIGHT = 1,
    REQUEST_SIM_ACTIVE = 2
};

struct FlightData {
    double pitchDeg;
    double elevatorInput;
    double yokeYPosition;
    double iasKt;
    double altitudeFt;
    double throttlePercent;
    double totalWeightKg;
    double ambientTempC;
};

struct PitchBridgeState {
    double pitchDeg;
    double elevatorPosition;
    double yokeYPosition;
    double iasKt;
    double altitudeFt;
    double throttlePercent;
    double sentElevator;
    double sentYokeY;
    double sentAxis;
    unsigned long long telemetryAgeMs;
    int connected;
    int telemetryValid;
    int writingEnabled;
    int controlMode; // fixed at BRUNNER-only (3)
    int lastWriteOk;
    int lastError;
    double totalWeightKg;
    double ambientTempC;
    int isPaused;
};

static std::atomic<bool> g_stop{false};
static std::atomic<bool> g_started{false};
static std::atomic<bool> g_enabled{false};
static std::atomic<int> g_controlMode{3}; // fixed BRUNNER-only
static std::atomic<double> g_commandValue{0.0}; // legacy ABI only; NEVER applied to P3D pitch
static std::atomic<unsigned long long> g_lastCommandMs{0};
static std::atomic<bool> g_isPaused{false};
static std::atomic<bool> g_simActive{true};

static std::mutex g_stateMutex;
static FlightData g_flight{};
static unsigned long long g_lastFrameMs = 0;
static bool g_connected = false;
static bool g_telemetryValid = false;
static int g_lastWriteOk = 0;
static int g_lastError = 0;

static double g_lastSentElevator = 0.0;
static double g_lastSentYokeY = 0.0;
static double g_lastSentAxis = 0.0;

static_assert(sizeof(PitchBridgeState) == 128, "V4.1 Python/C++ PitchBridgeState ABI mismatch");
static std::thread g_worker;
static HANDLE hSimForCallback = nullptr;

static unsigned long long nowMs() { return GetTickCount64(); }

static void updateConnection(bool connected, int error = 0) {
    std::lock_guard<std::mutex> lock(g_stateMutex);
    g_connected = connected;
    g_telemetryValid = false;
    g_lastFrameMs = 0;
    g_lastWriteOk = 0;
    g_lastError = error;
}

static void CALLBACK receiveCallback(SIMCONNECT_RECV* msg, DWORD bytes, void*) {
    if (!msg || bytes < sizeof(SIMCONNECT_RECV)) return;
    switch (msg->dwID) {
        case SIMCONNECT_RECV_ID_SIMOBJECT_DATA: {
            if (bytes < sizeof(SIMCONNECT_RECV_SIMOBJECT_DATA)) return;
            auto* data = reinterpret_cast<SIMCONNECT_RECV_SIMOBJECT_DATA*>(msg);
            if (data->dwRequestID != REQUEST_FLIGHT) return;
            if (bytes < offsetof(SIMCONNECT_RECV_SIMOBJECT_DATA, dwData) + sizeof(FlightData)) return;
            
            FlightData frame{};
            memcpy(&frame, &data->dwData, sizeof(frame));
            if (!std::isfinite(frame.pitchDeg) || !std::isfinite(frame.elevatorInput) ||
                !std::isfinite(frame.yokeYPosition)) return;

            std::lock_guard<std::mutex> lock(g_stateMutex);
            g_flight = frame;
            g_lastFrameMs = nowMs();
            g_telemetryValid = true;

            break;
        }
        case SIMCONNECT_RECV_ID_EVENT: {
            if (bytes < sizeof(SIMCONNECT_RECV_EVENT)) return;
            const auto* evt = reinterpret_cast<const SIMCONNECT_RECV_EVENT*>(msg);
            if (evt->uEventID == EVENT_PAUSE) g_isPaused.store(evt->dwData != 0);
            if (evt->uEventID == EVENT_SIM_START) g_simActive.store(true);
            if (evt->uEventID == EVENT_SIM_STOP) g_simActive.store(false);
            break;
        }
        case SIMCONNECT_RECV_ID_SYSTEM_STATE: {
            if (bytes < sizeof(SIMCONNECT_RECV_SYSTEM_STATE)) return;
            const auto* state = reinterpret_cast<const SIMCONNECT_RECV_SYSTEM_STATE*>(msg);
            if (state->dwRequestID == REQUEST_SIM_ACTIVE) g_simActive.store(state->dwInteger != 0);
            break;
        }
        case SIMCONNECT_RECV_ID_QUIT:
            updateConnection(false, -3);
            g_enabled.store(false);
            break;
        case SIMCONNECT_RECV_ID_EXCEPTION: {
            auto* e = reinterpret_cast<SIMCONNECT_RECV_EXCEPTION*>(msg);
            std::lock_guard<std::mutex> lock(g_stateMutex);
            g_lastError = static_cast<int>(e->dwException);
            break;
        }
        default: break;
    }
}

static HRESULT registerDefinitions(HANDLE sim) {
    HRESULT hr = S_OK;

    // Flight telemetry definitions
    hr = SimConnect_AddToDataDefinition(sim, DATA_FLIGHT, "PLANE PITCH DEGREES", "degrees");
    if (FAILED(hr)) return hr;
    hr = SimConnect_AddToDataDefinition(sim, DATA_FLIGHT, "ELEVATOR POSITION", "position");
    if (FAILED(hr)) return hr;
    hr = SimConnect_AddToDataDefinition(sim, DATA_FLIGHT, "YOKE Y POSITION", "position");
    if (FAILED(hr)) return hr;
    hr = SimConnect_AddToDataDefinition(sim, DATA_FLIGHT, "AIRSPEED INDICATED", "knots");
    if (FAILED(hr)) return hr;
    hr = SimConnect_AddToDataDefinition(sim, DATA_FLIGHT, "INDICATED ALTITUDE", "feet");
    if (FAILED(hr)) return hr;
    hr = SimConnect_AddToDataDefinition(sim, DATA_FLIGHT, "GENERAL ENG THROTTLE LEVER POSITION:1", "percent");
    if (FAILED(hr)) return hr;
    hr = SimConnect_AddToDataDefinition(sim, DATA_FLIGHT, "TOTAL WEIGHT", "kilograms");
    if (FAILED(hr)) return hr;
    hr = SimConnect_AddToDataDefinition(sim, DATA_FLIGHT, "AMBIENT TEMPERATURE", "celsius");
    if (FAILED(hr)) return hr;

    // No bank/elevator/yoke/axis writes. Only throttle is settable.
    hr = SimConnect_AddToDataDefinition(sim, DATA_THROTTLE,
        "GENERAL ENG THROTTLE LEVER POSITION:1", "percent");
    if (FAILED(hr)) return hr;
    hr = SimConnect_SubscribeToSystemEvent(sim, EVENT_PAUSE, "Pause");
    if (FAILED(hr)) return hr;
    hr = SimConnect_SubscribeToSystemEvent(sim, EVENT_SIM_START, "SimStart");
    if (FAILED(hr)) return hr;
    hr = SimConnect_SubscribeToSystemEvent(sim, EVENT_SIM_STOP, "SimStop");
    if (FAILED(hr)) return hr;
    hr = SimConnect_RequestSystemState(sim, REQUEST_SIM_ACTIVE, "Sim");
    if (FAILED(hr)) return hr;

    // Request continuous data per simulation frame
    return SimConnect_RequestDataOnSimObject(
        sim, REQUEST_FLIGHT, DATA_FLIGHT, SIMCONNECT_OBJECT_ID_USER,
        SIMCONNECT_PERIOD_SIM_FRAME, 0, 0, 0, 0);
}

static void workerLoop() {
    while (!g_stop.load()) {
        HANDLE sim = nullptr;
        HRESULT hr = SimConnect_Open(&sim, "SMK_Pitch_PID", nullptr, 0, nullptr, 0);
        if (FAILED(hr)) {
            updateConnection(false, static_cast<int>(hr));
            for (int i = 0; i < 15 && !g_stop.load(); ++i)
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        hr = registerDefinitions(sim);
        if (FAILED(hr)) {
            updateConnection(false, static_cast<int>(hr));
            SimConnect_Close(sim);
            std::this_thread::sleep_for(std::chrono::milliseconds(600));
            continue;
        }

        g_isPaused.store(false);
        updateConnection(true);
        hSimForCallback = sim;
        auto lastThrottleWriteMs = nowMs();

        while (!g_stop.load()) {
            hr = SimConnect_CallDispatch(sim, receiveCallback, nullptr);
            if (FAILED(hr)) {
                updateConnection(false, static_cast<int>(hr));
                g_enabled.store(false);
                break;
            }

            const auto tick = nowMs();
            bool safeToWrite = false;
            {
                std::lock_guard<std::mutex> lock(g_stateMutex);
                safeToWrite = g_connected && g_telemetryValid && (tick - g_lastFrameMs < 1200);
            }

            // BRUNNER is the sole pitch actuator; bank evolves freely in P3D.
            // Preserve the existing throttle-only write behavior.
            if (g_enabled.load() && safeToWrite && g_simActive.load() &&
                !g_isPaused.load() && (tick - lastThrottleWriteMs) >= 500) {
                double fullThrottle = 100.0;
                HRESULT throttleHr = SimConnect_SetDataOnSimObject(sim, DATA_THROTTLE,
                    SIMCONNECT_OBJECT_ID_USER, 0, 0, sizeof(fullThrottle), &fullThrottle);
                {
                    std::lock_guard<std::mutex> lock(g_stateMutex);
                    g_lastWriteOk = SUCCEEDED(throttleHr) ? 1 : 0;
                    if (FAILED(throttleHr)) g_lastError = static_cast<int>(throttleHr);
                }
                lastThrottleWriteMs = tick;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        g_enabled.store(false);
        hSimForCallback = nullptr;
        SimConnect_Close(sim);
        updateConnection(false);
    }
    g_started.store(false);
}

API int SC_Start() {
    if (g_started.exchange(true)) return 1;
    g_stop.store(false);
    g_enabled.store(false);
    try { g_worker = std::thread(workerLoop); }
    catch (...) { g_started.store(false); return 0; }
    return 1;
}

API void SC_Enable(int enabled) {
    if (!enabled) {
        g_enabled.store(false);
        return;
    }
    g_lastCommandMs.store(nowMs());
    g_enabled.store(true);
}

API void SC_SetControlMode(int) {
    g_controlMode.store(3); // Always BRUNNER-only; cannot select elevator modes.
}

API void SC_Command(double value) {
    if (!std::isfinite(value)) return;
    g_commandValue.store(std::clamp(value, -1.0, 1.0));
    g_lastCommandMs.store(nowMs());
}

API int SC_GetAbiVersion() { return 41; }
API int SC_IsPaused() { return g_isPaused.load() ? 1 : 0; }

API int SC_GetState(PitchBridgeState* out) {
    if (!out) return 0;
    std::lock_guard<std::mutex> lock(g_stateMutex);
    out->pitchDeg = g_flight.pitchDeg;
    out->elevatorPosition = g_flight.elevatorInput;
    out->yokeYPosition = g_flight.yokeYPosition;
    out->iasKt = g_flight.iasKt;
    out->altitudeFt = g_flight.altitudeFt;
    out->throttlePercent = g_flight.throttlePercent;
    out->sentElevator = g_lastSentElevator;
    out->sentYokeY = g_lastSentYokeY;
    out->sentAxis = g_lastSentAxis;
    out->telemetryAgeMs = g_lastFrameMs ? nowMs() - g_lastFrameMs : 999999;
    out->connected = g_connected ? 1 : 0;
    out->telemetryValid = g_telemetryValid ? 1 : 0;
    out->writingEnabled = g_enabled.load() ? 1 : 0;
    out->controlMode = g_controlMode.load();
    out->lastWriteOk = g_lastWriteOk;
    out->lastError = g_lastError;
    out->totalWeightKg = g_flight.totalWeightKg;
    out->ambientTempC = g_flight.ambientTempC;
    out->isPaused = g_isPaused.load() ? 1 : 0;
    return 1;
}

API void SC_Stop() {
    g_enabled.store(false);
    g_stop.store(true);
    if (g_worker.joinable()) g_worker.join();
    g_started.store(false);
}
