// QTG Studio - P3D SimConnect x64 read-only telemetry bridge.
// Compiles with MSVC and the user-supplied SimConnect.h + SimConnect.lib.
// NO SimConnect_SetDataOnSimObject or axis override code exists here.
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <Windows.h>
#include <SimConnect.h>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <mutex>
#include <thread>
#include "qtg_p3d_bridge.h"

static_assert(sizeof(QTG_P3D_State) == 136, "Unexpected QTG/Python ABI layout");

enum : DWORD { DEF_FLIGHT = 1 };
enum : DWORD { REQ_FLIGHT = 1 };
enum : DWORD { EVT_PAUSE = 1, EVT_START = 2, EVT_STOP = 3 };

// Must remain in exactly the same order as SimConnect_AddToDataDefinition below.
struct FlightFrame {
    double pitch_deg;
    double bank_deg;
    double ias_kt;
    double altitude_ft;
    double throttle_percent;
    double engine_rpm;
    double prop_rpm;
    double weight_kg;
    double temperature_c;
    double elevator_position;
    double aileron_position;
    double rudder_position;
};
static_assert(sizeof(FlightFrame) == 96, "Flight frame must contain 12 doubles");

static std::atomic<bool> g_stop{false};
static std::atomic<bool> g_started{false};
static std::thread g_thread;
static std::mutex g_lock;
static QTG_P3D_State g_state{};
static ULONGLONG g_last_frame = 0;

static void mark_disconnected(int err) {
    std::lock_guard<std::mutex> lock(g_lock);
    g_state.connected = 0;
    g_state.telemetry_valid = 0;
    g_last_frame = 0;
    g_state.last_error = err;
}

static HRESULT register_data(HANDLE sim) {
    struct Def { const char* name; const char* unit; };
    const Def defs[] = {
        {"PLANE PITCH DEGREES", "degrees"},
        {"PLANE BANK DEGREES", "degrees"},
        {"AIRSPEED INDICATED", "knots"},
        {"INDICATED ALTITUDE", "feet"},
        {"GENERAL ENG THROTTLE LEVER POSITION:1", "percent"},
        {"GENERAL ENG RPM:1", "rpm"},
        {"PROP RPM:1", "rpm"},
        {"TOTAL WEIGHT", "kilograms"},
        {"AMBIENT TEMPERATURE", "celsius"},
        {"ELEVATOR POSITION", "position"},
        {"AILERON POSITION", "position"},
        {"RUDDER POSITION", "position"},
    };
    for (const auto& def : defs) {
        HRESULT hr = SimConnect_AddToDataDefinition(sim, DEF_FLIGHT, def.name, def.unit);
        if (FAILED(hr)) return hr;
    }
    HRESULT hr = SimConnect_SubscribeToSystemEvent(sim, EVT_PAUSE, "Pause");
    if (FAILED(hr)) return hr;
    hr = SimConnect_SubscribeToSystemEvent(sim, EVT_START, "SimStart");
    if (FAILED(hr)) return hr;
    hr = SimConnect_SubscribeToSystemEvent(sim, EVT_STOP, "SimStop");
    if (FAILED(hr)) return hr;
    // Read once per simulation frame. Python samples the latest snapshot without blocking.
    return SimConnect_RequestDataOnSimObject(sim, REQ_FLIGHT, DEF_FLIGHT,
        SIMCONNECT_OBJECT_ID_USER, SIMCONNECT_PERIOD_SIM_FRAME, 0, 0, 0, 0);
}

static void CALLBACK dispatch(SIMCONNECT_RECV* msg, DWORD length, void*) {
    if (!msg || length < sizeof(SIMCONNECT_RECV)) return;
    switch (msg->dwID) {
    case SIMCONNECT_RECV_ID_SIMOBJECT_DATA: {
        if (length < offsetof(SIMCONNECT_RECV_SIMOBJECT_DATA, dwData) + sizeof(FlightFrame)) return;
        const auto* in = reinterpret_cast<const SIMCONNECT_RECV_SIMOBJECT_DATA*>(msg);
        if (in->dwRequestID != REQ_FLIGHT) return;
        FlightFrame f{};
        std::memcpy(&f, &in->dwData, sizeof(f));
        if (!std::isfinite(f.ias_kt) || !std::isfinite(f.pitch_deg) || !std::isfinite(f.bank_deg)) return;
        std::lock_guard<std::mutex> lock(g_lock);
        std::memcpy(&g_state.pitch_deg, &f, sizeof(f));
        g_last_frame = GetTickCount64();
        g_state.telemetry_valid = 1;
        break;
    }
    case SIMCONNECT_RECV_ID_EVENT: {
        if (length < sizeof(SIMCONNECT_RECV_EVENT)) return;
        const auto* ev = reinterpret_cast<const SIMCONNECT_RECV_EVENT*>(msg);
        std::lock_guard<std::mutex> lock(g_lock);
        if (ev->uEventID == EVT_PAUSE) g_state.paused = ev->dwData ? 1 : 0;
        if (ev->uEventID == EVT_START) g_state.sim_running = 1;
        if (ev->uEventID == EVT_STOP) g_state.sim_running = 0;
        break;
    }
    case SIMCONNECT_RECV_ID_EXCEPTION: {
        if (length < sizeof(SIMCONNECT_RECV_EXCEPTION)) return;
        const auto* ex = reinterpret_cast<const SIMCONNECT_RECV_EXCEPTION*>(msg);
        std::lock_guard<std::mutex> lock(g_lock);
        g_state.last_error = static_cast<int>(ex->dwException);
        break;
    }
    case SIMCONNECT_RECV_ID_QUIT:
        mark_disconnected(-3);
        break;
    default: break;
    }
}

static void worker() {
    while (!g_stop.load()) {
        HANDLE sim = nullptr;
        HRESULT hr = SimConnect_Open(&sim, "QTG_Studio_ReadOnly", nullptr, 0, nullptr, 0);
        if (FAILED(hr)) {
            mark_disconnected(static_cast<int>(hr));
            for (int n = 0; n < 20 && !g_stop.load(); ++n)
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        hr = register_data(sim);
        if (FAILED(hr)) {
            mark_disconnected(static_cast<int>(hr));
            SimConnect_Close(sim);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            continue;
        }
        {
            std::lock_guard<std::mutex> lock(g_lock);
            g_state.connected = 1;
            g_state.sim_running = 1;
            g_state.paused = 0;
            g_state.last_error = 0;
        }
        while (!g_stop.load()) {
            hr = SimConnect_CallDispatch(sim, dispatch, nullptr);
            if (FAILED(hr)) {
                mark_disconnected(static_cast<int>(hr));
                break;
            }
            {
                std::lock_guard<std::mutex> lock(g_lock);
                if (!g_state.connected) break; // SimConnect sent QUIT
                if (g_last_frame && GetTickCount64() - g_last_frame > 1500)
                    g_state.telemetry_valid = 0;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        SimConnect_Close(sim);
        mark_disconnected(0);
    }
    g_started = false;
}

QTG_API int QTG_BridgeVersion(void) { return 1; }
QTG_API int QTG_BridgeStart(void) {
    if (g_started.exchange(true)) return 1;
    g_stop = false;
    try { g_thread = std::thread(worker); }
    catch (...) { g_started = false; return 0; }
    return 1;
}
QTG_API int QTG_GetState(QTG_P3D_State* out) {
    if (!out || out->size != sizeof(QTG_P3D_State)) return 0;
    std::lock_guard<std::mutex> lock(g_lock);
    QTG_P3D_State snap = g_state;
    snap.size = sizeof(QTG_P3D_State);
    snap.abi_version = 1;
    snap.telemetry_age_ms = g_last_frame ? GetTickCount64() - g_last_frame : UINT64_MAX;
    *out = snap;
    return 1;
}
QTG_API void QTG_BridgeStop(void) {
    g_stop = true;
    if (g_thread.joinable()) g_thread.join();
    g_started = false;
}
