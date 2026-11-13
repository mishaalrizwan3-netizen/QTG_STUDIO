// Offline executable: the real snapshot implementation with mocked SimConnect.
#include <windows.h>
#include <stddef.h>
#include <cstring>
#include <cmath>
#include <cassert>
#include <thread>
static CRITICAL_SECTION g_CS;
static BOOL g_CSInitialized = TRUE, g_Connected = TRUE;
static HANDLE hSimConnect = reinterpret_cast<HANDLE>(1);
static volatile LONG g_SimulationRunningV1509 = 0;
static const int SIMCONNECT_DATATYPE_FLOAT64=0, SIMCONNECT_UNUSED=0,
    SIMCONNECT_OBJECT_ID_USER=0, SIMCONNECT_PERIOD_ONCE=0,
    SIMCONNECT_DATA_REQUEST_FLAG_DEFAULT=0;
struct SIMCONNECT_RECV_SIMOBJECT_DATA {
    DWORD dwRequestID, dwObjectID, dwDefineCount, dwFlags;
    double dwData[23];
};
static HRESULT TrackedAddToDataDefinition(HANDLE, DWORD, const char*, const char*, int, float, int) { return S_OK; }
static HRESULT SimConnect_RequestDataOnSimObject(HANDLE, DWORD, DWORD, int, int) { return S_OK; }
#include "../interfaces/brunner_base/native/src/verification_snapshot_v1.h"

int main() {
    InitializeCriticalSection(&g_CS);
    RegisterVerification();
    SIMCONNECT_RECV_SIMOBJECT_DATA packet = {};
    packet.dwDefineCount = 23;
    auto deliver = [&]() { ReceiveVerification(&packet, sizeof(packet)); };
    DWORD first = RequestVerificationV1();
    packet.dwRequestID = first;
    packet.dwData[0]=1.; packet.dwData[1]=0.;
    deliver();
    VerificationSnapshotV1 out;
    assert(GetVerificationV1(&out, sizeof(out)));
    assert(out.requestId==first && out.sequence==1 && out.values[0]==1. && out.values[1]==0.);
    assert(out.receiptTicks && out.frequency && out.copiedTicks>=out.receiptTicks);
    deliver(); assert(g_Verification.sequence==1); // duplicate callback
    DWORD second = RequestVerificationV1();
    deliver(); assert(g_Verification.sequence==1); // superseded callback
    packet.dwRequestID=second;
    packet.dwDefineCount=22; deliver(); assert(g_Verification.sequence==1);
    packet.dwDefineCount=23;
    ReceiveVerification(&packet, sizeof(packet)-8); assert(g_Verification.sequence==1);
    deliver(); assert(g_Verification.sequence==2); // paused, identical values
    const auto generation=g_Verification.generation;
    InvalidateVerification();
    assert(GetVerificationV1(&out, sizeof(out)) && out.requestId==0);
    RegisterVerification();
    assert(g_Verification.generation>generation);
    RequestVerificationV1(); deliver(); assert(g_Verification.sequence==0); // old connection response
    // Concurrent atomic payload/metadata copies: encoded request must always agree.
    std::thread producer([&]() {
        for(int i=0;i<10000;++i) {
            packet.dwRequestID=RequestVerificationV1();
            packet.dwData[2]=packet.dwRequestID;
            deliver();
        }
    });
    for(int i=0;i<10000;++i) {
        GetVerificationV1(&out,sizeof(out));
        if(out.requestId) assert(out.values[2]==out.requestId);
    }
    producer.join();
    packet.dwRequestID=RequestVerificationV1();
    packet.dwData[1]=std::nan(""); deliver();
    assert(!(g_Verification.validMask & 2));
    g_VerificationRegistered=false;
    assert(RequestVerificationV1()==0);
    g_Connected=false;
    GetVerificationV1(&out,sizeof(out)); assert(!out.connected);
    assert(!GetVerificationV1(&out,sizeof(out)-1));
    DeleteCriticalSection(&g_CS);
}
