// Additive read-only ABI. Included after legacy globals; no legacy layout changes.
static const DWORD VERIFICATION_DEFINITION = 19000;
static DWORD g_VerificationNextRequest = 0x40000000;
static DWORD g_VerificationPending = 0;
static bool g_VerificationRegistered = false;
#pragma pack(push, 1)
struct VerificationSnapshotV1 {
    DWORD size, version, connected, requestId;
    unsigned long long generation, sequence, validMask, receiptTicks, copiedTicks, frequency;
    LONG running; DWORD reserved;
    double values[23];
};
#pragma pack(pop)
static VerificationSnapshotV1 g_Verification = {};
static_assert(sizeof(VerificationSnapshotV1) == 72 + 23*8, "Verification ABI layout");
static void InvalidateVerification() {
    if (!g_CSInitialized) return;
    EnterCriticalSection(&g_CS);
    g_VerificationPending = 0;
    g_Verification.connected = 0;
    g_Verification.requestId = 0;
    g_Verification.validMask = 0;
    LeaveCriticalSection(&g_CS);
}
static void RegisterVerification() {
    EnterCriticalSection(&g_CS);
    ++g_Verification.generation;
    g_Verification.sequence = 0;
    g_Verification.requestId = g_VerificationPending = 0;
    g_Verification.validMask = 0;
    g_VerificationRegistered = true;
    if (FAILED(TrackedAddToDataDefinition(hSimConnect, VERIFICATION_DEFINITION, "SIM ON GROUND", "bool", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED))) g_VerificationRegistered = false;
    if (FAILED(TrackedAddToDataDefinition(hSimConnect, VERIFICATION_DEFINITION, "FLAPS HANDLE INDEX", "number", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED))) g_VerificationRegistered = false;
    if (FAILED(TrackedAddToDataDefinition(hSimConnect, VERIFICATION_DEFINITION, "PLANE ALTITUDE", "feet", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED))) g_VerificationRegistered = false;
    if (FAILED(TrackedAddToDataDefinition(hSimConnect, VERIFICATION_DEFINITION, "AIRSPEED INDICATED", "knots", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED))) g_VerificationRegistered = false;
    if (FAILED(TrackedAddToDataDefinition(hSimConnect, VERIFICATION_DEFINITION, "GROUND VELOCITY", "knots", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED))) g_VerificationRegistered = false;
    if (FAILED(TrackedAddToDataDefinition(hSimConnect, VERIFICATION_DEFINITION, "PLANE LATITUDE", "degrees", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED))) g_VerificationRegistered = false;
    if (FAILED(TrackedAddToDataDefinition(hSimConnect, VERIFICATION_DEFINITION, "PLANE LONGITUDE", "degrees", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED))) g_VerificationRegistered = false;
    if (FAILED(TrackedAddToDataDefinition(hSimConnect, VERIFICATION_DEFINITION, "PLANE HEADING DEGREES TRUE", "degrees", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED))) g_VerificationRegistered = false;
    if (FAILED(TrackedAddToDataDefinition(hSimConnect, VERIFICATION_DEFINITION, "PLANE PITCH DEGREES", "degrees", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED))) g_VerificationRegistered = false;
    if (FAILED(TrackedAddToDataDefinition(hSimConnect, VERIFICATION_DEFINITION, "PLANE BANK DEGREES", "degrees", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED))) g_VerificationRegistered = false;
    if (FAILED(TrackedAddToDataDefinition(hSimConnect, VERIFICATION_DEFINITION, "GENERAL ENG THROTTLE LEVER POSITION:1", "percent", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED))) g_VerificationRegistered = false;
    if (FAILED(TrackedAddToDataDefinition(hSimConnect, VERIFICATION_DEFINITION, "GENERAL ENG RPM:1", "rpm", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED))) g_VerificationRegistered = false;
    if (FAILED(TrackedAddToDataDefinition(hSimConnect, VERIFICATION_DEFINITION, "PROP RPM:1", "rpm", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED))) g_VerificationRegistered = false;
    if (FAILED(TrackedAddToDataDefinition(hSimConnect, VERIFICATION_DEFINITION, "AMBIENT TEMPERATURE", "celsius", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED))) g_VerificationRegistered = false;
    if (FAILED(TrackedAddToDataDefinition(hSimConnect, VERIFICATION_DEFINITION, "AMBIENT WIND VELOCITY", "knots", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED))) g_VerificationRegistered = false;
    if (FAILED(TrackedAddToDataDefinition(hSimConnect, VERIFICATION_DEFINITION, "AMBIENT WIND DIRECTION", "degrees", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED))) g_VerificationRegistered = false;
    if (FAILED(TrackedAddToDataDefinition(hSimConnect, VERIFICATION_DEFINITION, "KOHLSMAN SETTING MB", "millibars", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED))) g_VerificationRegistered = false;
    if (FAILED(TrackedAddToDataDefinition(hSimConnect, VERIFICATION_DEFINITION, "AMBIENT PRESSURE", "millibars", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED))) g_VerificationRegistered = false;
    if (FAILED(TrackedAddToDataDefinition(hSimConnect, VERIFICATION_DEFINITION, "SIM TIME", "seconds", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED))) g_VerificationRegistered = false;
    if (FAILED(TrackedAddToDataDefinition(hSimConnect, VERIFICATION_DEFINITION, "INDICATED ALTITUDE", "feet", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED))) g_VerificationRegistered = false;
    if (FAILED(TrackedAddToDataDefinition(hSimConnect, VERIFICATION_DEFINITION, "ENG MANIFOLD PRESSURE:1", "psi", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED))) g_VerificationRegistered = false;
    if (FAILED(TrackedAddToDataDefinition(hSimConnect, VERIFICATION_DEFINITION, "GENERAL ENG MIXTURE LEVER POSITION:1", "percent", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED))) g_VerificationRegistered = false;
    if (FAILED(TrackedAddToDataDefinition(hSimConnect, VERIFICATION_DEFINITION, "GENERAL ENG PROPELLER LEVER POSITION:1", "percent", SIMCONNECT_DATATYPE_FLOAT64, 0.0f, SIMCONNECT_UNUSED))) g_VerificationRegistered = false;
    LeaveCriticalSection(&g_CS);
}
static void ReceiveVerification(SIMCONNECT_RECV_SIMOBJECT_DATA* packet, DWORD bytes) {
    EnterCriticalSection(&g_CS);
    if (g_Connected && g_VerificationRegistered && g_VerificationPending &&
        packet->dwRequestID == g_VerificationPending &&
        packet->dwDefineCount == 23 && packet->dwFlags == SIMCONNECT_DATA_REQUEST_FLAG_DEFAULT &&
        bytes >= offsetof(SIMCONNECT_RECV_SIMOBJECT_DATA, dwData) + sizeof(g_Verification.values)) {
        memcpy(g_Verification.values, &packet->dwData, sizeof(g_Verification.values));
        g_Verification.validMask = 0;
        for (int i=0; i<23; ++i)
            if (std::isfinite(g_Verification.values[i])) g_Verification.validMask |= (1ULL << i);
        LARGE_INTEGER ticks, frequency;
        QueryPerformanceCounter(&ticks); QueryPerformanceFrequency(&frequency);
        g_Verification.running = InterlockedCompareExchange(&g_SimulationRunningV1509, -1, -1);
        g_Verification.receiptTicks = ticks.QuadPart;
        g_Verification.frequency = frequency.QuadPart;
        g_Verification.requestId = g_VerificationPending;
        ++g_Verification.sequence;
        g_VerificationPending = 0; // Duplicated callbacks cannot count as samples.
    }
    LeaveCriticalSection(&g_CS);
}
extern "C" {
__declspec(dllexport) DWORD RequestVerificationV1() {
    if (!g_CSInitialized) return 0;
    EnterCriticalSection(&g_CS);
    DWORD token = 0;
    // Never recycle IDs, including reconnects; exhaustion fails closed.
    if (hSimConnect && g_Connected && g_VerificationRegistered && g_VerificationNextRequest < 0x7fffffff) {
        token = ++g_VerificationNextRequest;
        g_VerificationPending = token;
        g_Verification.requestId = 0;
        if (FAILED(SimConnect_RequestDataOnSimObject(hSimConnect, token, VERIFICATION_DEFINITION,
                SIMCONNECT_OBJECT_ID_USER, SIMCONNECT_PERIOD_ONCE))) {
            token = 0; g_VerificationPending = 0;
        }
    }
    LeaveCriticalSection(&g_CS);
    return token;
}
__declspec(dllexport) int GetVerificationV1(VerificationSnapshotV1* out, DWORD size) {
    if (!out || size != sizeof(*out) || !g_CSInitialized) return 0;
    EnterCriticalSection(&g_CS);
    *out = g_Verification;
    out->size = sizeof(*out); out->version = 1;
    out->connected = g_Connected && hSimConnect && g_VerificationRegistered;
    LARGE_INTEGER ticks; QueryPerformanceCounter(&ticks); out->copiedTicks = ticks.QuadPart;
    LeaveCriticalSection(&g_CS);
    return 1;
}
}
