// P3D SimConnect 64-bit native bridge for Pitch Angle PID Loop & Hardcoded Bank=0 Hold
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
    DATA_BANK = 4
};

enum EventID : DWORD { 
    EVENT_AXIS_ELEVATOR_SET = 1,
    EVENT_PAUSE = 2,
    EVENT_PAUSED = 3,
    EVENT_UNPAUSED = 4
};

enum RequestID : DWORD { 
    REQUEST_FLIGHT = 1,
    REQUEST_SIM_STATE = 2
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
    int controlMode;
    int lastWriteOk;
    int lastError;
    double totalWeightKg;
    double ambientTempC;
    int isPaused;
};

static std::atomic<bool> g_stop{false};
static std::atomic<bool> g_started{false};
static std::atomic<bool> g_enabled{false};
static std::atomic<int> g_controlMode{3};
static std::atomic<double> g_commandValue{0.0};
static std::atomic<unsigned long long> g_lastCommandMs{0};
static std::atomic<bool> g_isPaused{false};

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
    if (!connected) {
        g_isPaused.store(false);
    }
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
            auto* evt = reinterpret_cast<SIMCONNECT_RECV_EVENT*>(msg);
            if (evt->uEventID == EVENT_PAUSE) {
                g_isPaused.store(evt->dwData != 0);
            } else if (evt->uEventID == EVENT_PAUSED) {
                g_isPaused.store(true);
            } else if (evt->uEventID == EVENT_UNPAUSED) {
                g_isPaused.store(false);
            }
            break;
        }
        case SIMCONNECT_RECV_ID_SYSTEM_STATE: {
            if (bytes < sizeof(SIMCONNECT_RECV_SYSTEM_STATE)) return;
            auto* st = reinterpret_cast<SIMCONNECT_RECV_SYSTEM_STATE*>(msg);
            if (st->dwRequestID == REQUEST_SIM_STATE) {
                // "Sim" returns 1 if running, 0 if paused
                g_isPaused.store(st->dwInteger == 0);
            }
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

    // Hardcoded wings-level bank angle (0.0 deg) definition
    hr = SimConnect_AddToDataDefinition(sim, DATA_BANK, "PLANE BANK DEGREES", "degrees");
    if (FAILED(hr)) return hr;

    // Subscribe to pause events and query current simulator state
    SimConnect_SubscribeToSystemEvent(sim, EVENT_PAUSE, "Pause");
    SimConnect_SubscribeToSystemEvent(sim, EVENT_PAUSED, "Paused");
    SimConnect_SubscribeToSystemEvent(sim, EVENT_UNPAUSED, "Unpaused");
    SimConnect_RequestSystemState(sim, REQUEST_SIM_STATE, "Sim");

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

        updateConnection(true);
        hSimForCallback = sim;
        auto lastWriteMs = nowMs();
        auto lastStateCheckMs = nowMs();

        while (!g_stop.load()) {
            hr = SimConnect_CallDispatch(sim, receiveCallback, nullptr);
            if (FAILED(hr)) {
                updateConnection(false, static_cast<int>(hr));
                g_enabled.store(false);
                break;
            }

            const auto tick = nowMs();

            // Actively refresh pause state every 500ms
            if ((tick - lastStateCheckMs) >= 500) {
                SimConnect_RequestSystemState(sim, REQUEST_SIM_STATE, "Sim");
                lastStateCheckMs = tick;
            }

            bool safeToWrite = false;
            {
                std::lock_guard<std::mutex> lock(g_stateMutex);
                safeToWrite = g_connected && g_telemetryValid && (tick - g_lastFrameMs < 1200);
            }

            // Hardcoded wings-level bank angle (0.0 deg) enforced at 50 Hz (20ms interval)
            // Paused: Do NOT write bank angles while simulation is paused
            if (g_enabled.load() && safeToWrite && !g_isPaused.load() && (tick - lastWriteMs) >= 20) {
                double zeroBank = 0.0;
                HRESULT writeHr = SimConnect_SetDataOnSimObject(sim, DATA_BANK,
                    SIMCONNECT_OBJECT_ID_USER, 0, 0, sizeof(zeroBank), &zeroBank);
                {
                    std::lock_guard<std::mutex> lock(g_stateMutex);
                    g_lastWriteOk = SUCCEEDED(writeHr) ? 1 : 0;
                    if (FAILED(writeHr)) g_lastError = static_cast<int>(writeHr);
                }
                lastWriteMs = tick;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(2));
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

API void SC_Stop() {
    g_stop.store(true);
    g_enabled.store(false);
    if (g_worker.joinable()) {
        try { g_worker.join(); } catch (...) {}
    }
    g_started.store(false);
}

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
    out->telemetryAgeMs = (g_lastFrameMs > 0) ? (nowMs() - g_lastFrameMs) : 9999;
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

API int SC_IsPaused() {
    return g_isPaused.load() ? 1 : 0;
}

API void SC_SetControlMode(int mode) {
    g_controlMode.store(mode);
}

API void SC_Command(double value) {
    g_commandValue.store(value);
    g_lastCommandMs.store(nowMs());
}

API void SC_Enable(int enabled) {
    g_enabled.store(enabled != 0);
}
