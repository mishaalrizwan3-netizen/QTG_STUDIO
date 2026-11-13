#pragma once
// Public ABI of the QTG Studio read-only SimConnect interface.
// The bridge does not write flight controls, throttle, pauses, or simulation state.
#include <stdint.h>
#ifdef _WIN32
  #define QTG_API extern "C" __declspec(dllexport)
#else
  #define QTG_API extern "C"
#endif

#pragma pack(push, 8)
typedef struct QTG_P3D_State {
    uint32_t size;          // sizeof(QTG_P3D_State) = 136
    uint32_t abi_version;   // 1
    int32_t connected;      // 1 after SimConnect_Open succeeds
    int32_t telemetry_valid; // 1 when at least one valid sample received
    int32_t paused;         // 1 when Pause event reports paused
    int32_t sim_running;    // 0 after SimStop event
    uint64_t telemetry_age_ms;
    int32_t last_error;     // HRESULT or SimConnect exception ID
    int32_t reserved;
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
} QTG_P3D_State;
#pragma pack(pop)

QTG_API int QTG_BridgeVersion(void);          // Returns 1
QTG_API int QTG_BridgeStart(void);            // Starts worker, attempts connection
QTG_API int QTG_GetState(QTG_P3D_State* out); // Thread-safe snapshot
QTG_API void QTG_BridgeStop(void);           // Stops worker and closes SimConnect
