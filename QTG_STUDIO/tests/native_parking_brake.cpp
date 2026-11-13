// Invoke the actual diagnostic callback with fake packets; never call main/Open.
#define main diagnostic_main_not_invoked
#include "../tools/parking_brake_diagnostic.cpp"
#undef main
#include <cassert>
#include <vector>
int main(int argc, char** argv) {
    assert(argc == 2);
    Observer observer;
    observer.csv.open(std::filesystem::path(argv[1])/"mock.csv");
    observer.events.open(std::filesystem::path(argv[1])/"events.jsonl");
    observer.opened = observer.connected = true;
    const auto header = offsetof(SIMCONNECT_RECV_SIMOBJECT_DATA,dwData);
    std::vector<unsigned char> storage(header+sizeof(double));
    auto* packet = reinterpret_cast<SIMCONNECT_RECV_SIMOBJECT_DATA*>(storage.data());
    packet->dwID = SIMCONNECT_RECV_ID_SIMOBJECT_DATA;
    // USER=0 is a request proxy, not the server-assigned ID in a response.
    packet->dwObjectID = 1;
    packet->dwDefineCount = 1;
    packet->dwFlags = SIMCONNECT_DATA_REQUEST_FLAG_DEFAULT;
    auto sample = [&](int channel,double value) {
        packet->dwDefineID = channel+1;
        packet->dwRequestID = observer.state.request(channel);
        observer.sent[channel] = Clock::now();
        memcpy(&packet->dwData,&value,sizeof(value));
        receive(reinterpret_cast<SIMCONNECT_RECV*>(packet),static_cast<DWORD>(storage.size()),&observer);
    };
    sample(0,0.); assert(observer.received[0]==1 && observer.last[0]=="0");
    assert(observer.object_known && observer.object_id==1);
    sample(0,1.); assert(observer.received[0]==2 && observer.last[0]=="1");
    observer.running = 0;
    sample(0,1.); assert(observer.received[0]==3); // paused/unchanged NEW response
    receive(reinterpret_cast<SIMCONNECT_RECV*>(packet),static_cast<DWORD>(storage.size()),&observer);
    assert(observer.received[0]==3 && observer.ignored==1); // duplicate cache
    const auto old = packet->dwRequestID;
    observer.state.request(0);
    packet->dwRequestID=old;
    receive(reinterpret_cast<SIMCONNECT_RECV*>(packet),static_cast<DWORD>(storage.size()),&observer);
    assert(observer.received[0]==3 && observer.ignored==2); // superseded request
    sample(1,.5); sample(2,.75);
    assert(observer.last[1]=="0.5" && observer.last[2]=="0.75");
    sample(0,2.); sample(1,std::nan("")); sample(2,-.1);
    assert(observer.invalid[0]==1 && observer.invalid[1]==1 && observer.invalid[2]==1);
    packet->dwDefineID=1; packet->dwRequestID=observer.state.request(0);
    auto sequence=observer.state.sequence[0];
    receive(reinterpret_cast<SIMCONNECT_RECV*>(packet),static_cast<DWORD>(storage.size()-1),&observer);
    assert(observer.state.sequence[0]==sequence); // truncated
    packet->dwDefineCount=0;
    receive(reinterpret_cast<SIMCONNECT_RECV*>(packet),static_cast<DWORD>(storage.size()),&observer);
    assert(observer.state.sequence[0]==sequence); // incomplete
    packet->dwDefineCount=1;
    packet->dwFlags=SIMCONNECT_DATA_REQUEST_FLAG_TAGGED;
    receive(reinterpret_cast<SIMCONNECT_RECV*>(packet),static_cast<DWORD>(storage.size()),&observer);
    assert(observer.state.sequence[0]==sequence); // wrong layout flags
    packet->dwFlags=SIMCONNECT_DATA_REQUEST_FLAG_DEFAULT;
    observer.sent[0]=Clock::now()-std::chrono::seconds(3);
    receive(reinterpret_cast<SIMCONNECT_RECV*>(packet),static_cast<DWORD>(storage.size()),&observer);
    assert(observer.missing[0]==1 && observer.received[0]==3); // late
    sample(3,42.); sample(3,42.); assert(observer.received[3]==2); // no physics assumption
    // Fully correlated data may NOT silently switch aircraft within this session.
    packet->dwObjectID=99;
    sample(0,0.);
    assert(observer.failed && observer.received[0]==3);
    assert(observer.reason.find("object changed")!=std::string::npos);
    observer.failed=false; stopping=false; packet->dwObjectID=1;
    observer.state.pending[0]=123;
    SIMCONNECT_RECV quit={}; quit.dwID=SIMCONNECT_RECV_ID_QUIT;
    receive(&quit,sizeof(quit),&observer);
    assert(observer.failed && !observer.connected && !observer.state.pending[0] && stopping);
    stopping=false;
    assert(cancel(CTRL_C_EVENT) && stopping); // cancellation signals only; no control calls
    assert(brake_diag::display(0,0.)=="OFF" && brake_diag::display(0,1.)=="ON");
    assert(brake_diag::display(0,std::nan(""))=="UNAVAILABLE");
    assert(!brake_diag::valid(1,1.1) && brake_diag::valid(1,0.) && brake_diag::valid(1,1.));
    observer.state.next=UINT32_MAX; assert(!observer.state.request(0));
    Observer alternate;
    alternate.csv.open(std::filesystem::path(argv[1])/"alternate.csv");
    alternate.connected=true;
    packet->dwObjectID=37; packet->dwDefineID=1; packet->dwRequestID=alternate.state.request(0);
    alternate.sent[0]=Clock::now();
    double off=0.; memcpy(&packet->dwData,&off,sizeof(off));
    receive(reinterpret_cast<SIMCONNECT_RECV*>(packet),static_cast<DWORD>(storage.size()),&alternate);
    assert(alternate.received[0]==1 && alternate.object_id==37); // no hard-coded actual ID
    Observer paired;
    paired.parking_only=true; paired.connected=true;
    paired.csv.open(std::filesystem::path(argv[1])/"pair.csv");
    paired.csv << "observed_utc,pid,connection_generation,request_id,channel_sequence,receipt_qpc_ticks,qpc_frequency,session_elapsed_s,request_to_receipt_s,running,simvar,unit,value,status,timestamp_provenance\n";
    std::vector<unsigned char> pair_storage(header+2*sizeof(double));
    auto* pair_packet=reinterpret_cast<SIMCONNECT_RECV_SIMOBJECT_DATA*>(pair_storage.data());
    pair_packet->dwID=SIMCONNECT_RECV_ID_SIMOBJECT_DATA;
    pair_packet->dwObjectID=37;pair_packet->dwDefineID=1;pair_packet->dwDefineCount=2;
    double pair_values[2]={0.,42.};
    pair_packet->dwRequestID=paired.state.request(0);paired.sent[0]=Clock::now();
    memcpy(&pair_packet->dwData,pair_values,sizeof(pair_values));
    receive(pair_packet,static_cast<DWORD>(pair_storage.size()),&paired);
    assert(paired.received[0]==1 && paired.received[3]==1);
    assert(paired.received[1]==0 && paired.received[2]==0);
    pair_packet->dwRequestID=paired.state.request(0);paired.sent[0]=Clock::now();
    pair_packet->dwDefineCount=1;
    receive(pair_packet,static_cast<DWORD>(pair_storage.size()),&paired);
    assert(paired.received[0]==1); // legacy single-field payload cannot populate the pair
    pair_packet->dwDefineCount=2;
    pair_values[0]=1.;pair_values[1]=43.;memcpy(&pair_packet->dwData,pair_values,sizeof(pair_values));
    receive(pair_packet,static_cast<DWORD>(pair_storage.size()),&paired);
    assert(paired.received[0]==2 && paired.last[0]=="1" && paired.last[3]=="43");
    Observer exceptional;
    exceptional.events.open(std::filesystem::path(argv[1])/"exception.jsonl");
    exceptional.sends[77]={"RequestDataOnSimObject",101,1};
    SIMCONNECT_RECV_EXCEPTION error={};
    error.dwID=SIMCONNECT_RECV_ID_EXCEPTION;
    error.dwException=SIMCONNECT_EXCEPTION_NAME_UNRECOGNIZED;
    error.dwSendID=77; error.dwIndex=1;
    receive(&error,sizeof(error),&exceptional);
    assert(exceptional.failed && exceptional.exceptions==1 && stopping);
    assert(exceptional.reason.find("_send_77_index_1")!=std::string::npos);
    assert(std::string(observer.state.rejection(55,1,1,true,true))=="unknown_definition");
    return 0;
}
