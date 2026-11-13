// Standalone SDK observer. Never loads or changes the QTG bridge DLL.
#include <windows.h>
#include <SimConnect.h>
#include <atomic>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <stdexcept>
#include <thread>
#include "parking_brake_core.h"

using Clock = std::chrono::steady_clock;
static std::atomic<bool> stopping(false);
static BOOL WINAPI cancel(DWORD event) {
    if (event == CTRL_C_EVENT || event == CTRL_BREAK_EVENT) { stopping = true; return TRUE; }
    return FALSE;
}
static std::string utc() {
    SYSTEMTIME t; GetSystemTime(&t);
    char out[40]; sprintf_s(out, "%04u-%02u-%02uT%02u:%02u:%02u.%03uZ",
        t.wYear,t.wMonth,t.wDay,t.wHour,t.wMinute,t.wSecond,t.wMilliseconds);
    return out;
}
static std::string json_string(const std::string& value) {
    std::ostringstream out; out << '"';
    for (unsigned char c:value) {
        if (c=='"' || c=='\\') out << '\\' << c;
        else if (c<32) out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << int(c) << std::dec;
        else out << c;
    }
    out << '"'; return out.str();
}
struct Observer {
    HANDLE connection = nullptr;
    brake_diag::State state;
    bool connected = false, failed = false, opened = false;
    int running = -1;
    bool parking_only = false;
    HANDLE owner = nullptr;
    std::string reason = "duration_complete";
    std::ofstream csv, events;
    bool object_known = false;
    DWORD object_id = 0;
    struct Sent { std::string operation; DWORD request, definition; };
    std::map<DWORD,Sent> sends;
    uint64_t callbacks = 0, exceptions = 0;
    Clock::time_point sent[brake_diag::COUNT], began = Clock::now();
    uint64_t received[brake_diag::COUNT] = {}, invalid[brake_diag::COUNT] = {},
             missing[brake_diag::COUNT] = {}, ignored = 0;
    std::string first[brake_diag::COUNT], last[brake_diag::COUNT];
    void event(const char* kind, const std::string& fields = "") {
        if (events.is_open()) {
            events << "{\"observed_utc\":" << json_string(utc()) << ",\"event\":" << json_string(kind)
                << ",\"running\":" << running << fields << "}\n";
            events.flush();
            if (!events) throw std::runtime_error("Callback diagnostic log write failed");
        }
    }
    HRESULT track(const char* operation, HRESULT result, DWORD request=0, DWORD definition=0) {
        DWORD send = 0;
        const bool tracked = SUCCEEDED(result) && connection &&
            SUCCEEDED(SimConnect_GetLastSentPacketID(connection,&send));
        if (tracked) sends[send] = {operation,request,definition};
        event("SDK_CALL",",\"operation\":"+json_string(operation)+",\"hresult\":"+
            std::to_string(static_cast<unsigned long>(result))+",\"request_id\":"+std::to_string(request)+
            ",\"definition_id\":"+std::to_string(definition)+",\"request_object_proxy\":0,\"send_id\":"+
            (tracked ? std::to_string(send) : "null"));
        return result;
    }
    void row(int channel, const char* status, const std::string& value = "",
             uint32_t request = 0, double request_age = 0., LONGLONG receipt = 0) {
        LARGE_INTEGER ticks, frequency;
        QueryPerformanceCounter(&ticks); QueryPerformanceFrequency(&frequency);
        if (receipt) ticks.QuadPart = receipt; // One callback stamp for the paired payload.
        const std::string observed = utc();
        const bool callback = std::string(status) == "NEW_CORRELATED_RESPONSE" ||
            std::string(status) == "INVALID" || std::string(status) == "LATE_REJECTED";
        csv << observed << ',' << GetCurrentProcessId() << ",1," << request << ','
            << state.sequence[channel] << ',';
        if (callback) csv << ticks.QuadPart;
        csv << ',' << frequency.QuadPart << ','
            << brake_diag::number(std::chrono::duration<double>(Clock::now()-began).count()) << ','
            << brake_diag::number(request_age) << ',' << running << ',' << brake_diag::names[channel]
            << ',' << brake_diag::units[channel] << ',' << value << ',' << status
            << ',' << (callback ? "native_callback_receipt_not_internal_acquisition" : "host_timeout_observation") << '\n';
        csv.flush();
        if (!csv) throw std::runtime_error("CSV write failed");
        std::cout << observed << " | " << brake_diag::names[channel] << " | "
                  << (value.empty() ? "Not available" : value) << " | " << status << std::endl;
    }
    ~Observer() { if (connection) SimConnect_Close(connection); if (owner) CloseHandle(owner); }
};
static void CALLBACK receive(SIMCONNECT_RECV* packet, DWORD bytes, void* context) {
    auto& o = *static_cast<Observer*>(context);
    try {
        ++o.callbacks;
        if (!packet || bytes < sizeof(SIMCONNECT_RECV)) {
            ++o.ignored; o.event("REJECTED_CALLBACK",",\"reason\":\"short_callback_header\""); return;
        }
        o.event("CALLBACK",",\"callback_id\":"+std::to_string(packet->dwID)+
            ",\"callback_bytes\":"+std::to_string(bytes)+",\"packet_size\":"+std::to_string(packet->dwSize));
        if (packet->dwID == SIMCONNECT_RECV_ID_OPEN) {
            if (o.opened) throw std::runtime_error("Unexpected second OPEN; reconnect rejected");
            o.opened = o.connected = true;
            std::cout << "CONNECTED: owned read-only diagnostic transport" << std::endl;
        } else if (packet->dwID == SIMCONNECT_RECV_ID_QUIT) {
            o.connected = false; o.failed = true; o.reason = "simulator_disconnected";
            o.state.disconnect(); stopping = true;
        } else if (packet->dwID == SIMCONNECT_RECV_ID_EXCEPTION) {
            if (bytes < sizeof(SIMCONNECT_RECV_EXCEPTION)) throw std::runtime_error("Truncated SDK exception callback");
            auto* error = reinterpret_cast<SIMCONNECT_RECV_EXCEPTION*>(packet);
            ++o.exceptions;
            const auto found = o.sends.find(error->dwSendID);
            const auto detail = found == o.sends.end() ? "unknown_send" : found->second.operation;
            o.event("SDK_EXCEPTION",",\"exception_code\":"+std::to_string(error->dwException)+
                ",\"send_id\":"+std::to_string(error->dwSendID)+",\"index\":"+std::to_string(error->dwIndex)+
                ",\"operation\":"+json_string(detail)+",\"request_id\":"+
                (found==o.sends.end()?"null":std::to_string(found->second.request))+
                ",\"definition_id\":"+(found==o.sends.end()?"null":std::to_string(found->second.definition)));
            o.failed = true;
            o.reason = "sdk_exception_"+std::to_string(error->dwException)+
                "_send_"+std::to_string(error->dwSendID)+"_index_"+std::to_string(error->dwIndex);
            std::cerr << "SDK EXCEPTION: " << o.reason << " operation=" << detail << std::endl;
            stopping = true; // Unsupported definitions are never replaced by other channels.
        } else if (packet->dwID == SIMCONNECT_RECV_ID_EVENT) {
            if (bytes < sizeof(SIMCONNECT_RECV_EVENT)) throw std::runtime_error("Truncated event callback");
            auto* event = reinterpret_cast<SIMCONNECT_RECV_EVENT*>(packet);
            if (event->uEventID == 20) o.running = event->dwData ? 0 : 1;
        } else if (packet->dwID == SIMCONNECT_RECV_ID_SIMOBJECT_DATA) {
            const auto header = offsetof(SIMCONNECT_RECV_SIMOBJECT_DATA, dwData);
            if (bytes < header) {
                ++o.ignored; o.event("REJECTED_CALLBACK",",\"reason\":\"short_simobject_header\""); return;
            }
            auto* data = reinterpret_cast<SIMCONNECT_RECV_SIMOBJECT_DATA*>(packet);
            const int channel = static_cast<int>(data->dwDefineID)-1;
            const unsigned expected_count = o.parking_only ? 2 : 1;
            const size_t expected_bytes = expected_count*sizeof(double);
            std::string details = ",\"request_id\":"+std::to_string(data->dwRequestID)+
                ",\"definition_id\":"+std::to_string(data->dwDefineID)+",\"returned_object_id\":"+
                std::to_string(data->dwObjectID)+",\"datum_count\":"+std::to_string(data->dwDefineCount)+
                ",\"flags\":"+std::to_string(data->dwFlags)+",\"callback_bytes\":"+std::to_string(bytes);
            const char* rejected = o.connected ? o.state.rejection(channel,data->dwRequestID,
                data->dwDefineCount == expected_count ? 1 : 0,
                data->dwFlags == SIMCONNECT_DATA_REQUEST_FLAG_DEFAULT,bytes >= header+expected_bytes) : "not_connected";
            if (o.parking_only && channel != 0) rejected = "unexpected_parking_pair_definition";
            if (rejected) {
                ++o.ignored; o.event("REJECTED_DATA",details+",\"reason\":"+json_string(rejected));
                std::cerr << "REJECTED DATA: " << rejected << details << std::endl;
                return;
            }
            // USER=0 selects the user vehicle in a REQUEST, not the server ID
            // returned in DATA. These private outstanding IDs target only USER.
            // Bind the actual ID from the first fully correlated reply; never
            // hard-code 0 or 1, and reject a vehicle change within this session.
            if (o.object_known && o.object_id != data->dwObjectID) {
                o.event("REJECTED_DATA",details+",\"reason\":\"user_object_changed\"");
                throw std::runtime_error("User-aircraft response object changed; start a new diagnostic session");
            }
            o.object_known = true; o.object_id = data->dwObjectID;
            o.state.accept(channel,data->dwRequestID,data->dwDefineCount,true,true);
            o.event("MATCHED_DATA",details);
            double value; memcpy(&value,&data->dwData,sizeof(value));
            const double age = std::chrono::duration<double>(Clock::now()-o.sent[channel]).count();
            if (age > 2.) { ++o.missing[channel]; o.row(channel,"LATE_REJECTED","",data->dwRequestID,age); return; }
            if (o.parking_only) {
                double pair[2]; memcpy(pair,&data->dwData,sizeof(pair));
                LARGE_INTEGER stamp; QueryPerformanceCounter(&stamp);
                o.state.sequence[3] = o.state.sequence[0];
                for (int i=0;i<2;++i) {
                    const int key = i == 0 ? 0 : 3;
                    if (!brake_diag::valid(key,pair[i])) {
                        ++o.invalid[key];
                        o.row(key,"INVALID",brake_diag::number(pair[i]),data->dwRequestID,age,stamp.QuadPart);
                    } else {
                        ++o.received[key];
                        if (o.first[key].empty()) o.first[key]=brake_diag::number(pair[i]);
                        o.last[key]=brake_diag::number(pair[i]);
                        o.row(key,"NEW_CORRELATED_RESPONSE",brake_diag::number(pair[i]),data->dwRequestID,age,stamp.QuadPart);
                    }
                }
                return; // Never decode or request either wheel-brake channel in this mode.
            }
            if (!brake_diag::valid(channel,value)) {
                ++o.invalid[channel];
                o.row(channel,"INVALID",brake_diag::number(value),data->dwRequestID,age);
                return;
            }
            ++o.received[channel];
            if (o.first[channel].empty()) o.first[channel] = brake_diag::number(value);
            o.last[channel] = brake_diag::number(value);
            o.row(channel,"NEW_CORRELATED_RESPONSE",brake_diag::number(value),data->dwRequestID,age);
            if (channel == 0) std::cout << "Parking brake: " << brake_diag::display(channel,value) << std::endl;
        } else o.event("UNHANDLED_CALLBACK",",\"callback_id\":"+std::to_string(packet->dwID));
    } catch (const std::exception& error) {
        o.failed = true; o.reason = error.what(); stopping = true;
    }
}
int main(int argc, char** argv) {
    // No connection without this exact opt-in; --help is safe offline.
    if (argc < 5 || std::string(argv[1]) != "--connect-read-only") {
        std::cout << "Usage: parking_brake_diagnostic.exe --connect-read-only OUTPUT_PARENT --duration-s SECONDS\n"
            "Optional: --parking-only --stop-file PATH --owner-pid PID\n"
            "Own SDK observer connection. No brake or aircraft commands.\n";
        return argc == 2 && std::string(argv[1]) == "--help" ? 0 : 2;
    }
    Observer o;
    std::filesystem::path folder;
    std::filesystem::path stop_file;
    try {
        for (int i=5;i<argc;++i) {
            const std::string option=argv[i];
            if (option=="--parking-only") o.parking_only=true;
            else if (option=="--stop-file" && i+1<argc) stop_file=argv[++i];
            else if (option=="--owner-pid" && i+1<argc) {
                const std::string text=argv[++i]; size_t used=0;
                const auto pid=std::stoul(text,&used);
                if (used!=text.size() || !pid || pid>MAXDWORD) throw std::runtime_error("Invalid owner PID");
                o.owner=OpenProcess(SYNCHRONIZE,FALSE,static_cast<DWORD>(pid));
                if (!o.owner) throw std::runtime_error("Cannot observe owner process; refusing connection");
            } else throw std::runtime_error("Unknown/incomplete diagnostic option: "+option);
        }
        size_t parsed = 0;
        const double duration = std::stod(argv[4], &parsed);
        if (std::string(argv[3]) != "--duration-s" || parsed != strlen(argv[4]) ||
            !std::isfinite(duration) || duration <= 0 || duration > 600)
            throw std::runtime_error("Duration must be finite and between 0 and 600 seconds");
        for (const char* key : {"QTG_UNIVERSAL_ENGINE_ACCELERATION","QTG_UNIVERSAL_ENGINE_DECELERATION"}) {
            char value[8] = {};
            if (GetEnvironmentVariableA(key,value,sizeof(value)) && std::string(value) != "0")
                throw std::runtime_error("Keep Universal Initialization disabled");
        }
        folder = std::filesystem::path(argv[2]) /
            ("brakes_"+std::to_string(GetCurrentProcessId())+"_"+std::to_string(GetTickCount64()));
        if (!std::filesystem::create_directories(folder)) throw std::runtime_error("Session folder already exists");
        o.csv.open(folder/"brake_samples.csv",std::ios::out);
        if (!o.csv) throw std::runtime_error("Cannot create diagnostic CSV");
        o.events.open(folder/"callback_events.jsonl",std::ios::out);
        if (!o.events) throw std::runtime_error("Cannot create callback diagnostic log");
        o.csv << "observed_utc,pid,connection_generation,request_id,channel_sequence,receipt_qpc_ticks,qpc_frequency,session_elapsed_s,request_to_receipt_s,running,simvar,unit,value,status,timestamp_provenance\n";
        std::cout << "Evidence: " << std::filesystem::absolute(folder).string() << std::endl;
        SetConsoleCtrlHandler(cancel,TRUE);
        if (FAILED(o.track("Open",SimConnect_Open(&o.connection,"QTG parking-brake read-only diagnostic",nullptr,0,nullptr,0))))
            throw std::runtime_error("CONNECTION_FAILURE: SimConnect_Open failed; simulator not started by this tool");
        for (int i=0; i<brake_diag::COUNT; ++i) {
            if (o.parking_only && i!=0 && i!=3) continue;
            const DWORD definition=o.parking_only ? 1 : i+1;
            if (FAILED(o.track("AddToDataDefinition",SimConnect_AddToDataDefinition(o.connection,definition,brake_diag::names[i],
                    brake_diag::units[i],SIMCONNECT_DATATYPE_FLOAT64,0.f,SIMCONNECT_UNUSED),0,definition)))
                throw std::runtime_error("DEFINITION_FAILURE: read definition rejected");
        }
        if (FAILED(o.track("SubscribePause",SimConnect_SubscribeToSystemEvent(o.connection,20,"Pause"))))
            std::cout << "Pause observation unavailable; running state stays UNKNOWN" << std::endl;
        auto next = Clock::now();
        while (!stopping) {
            if ((!stop_file.empty() && std::filesystem::exists(stop_file)) ||
                (o.owner && WaitForSingleObject(o.owner,0)!=WAIT_TIMEOUT)) {
                o.reason="capture_owner_stopped"; break;
            }
            if (FAILED(SimConnect_CallDispatch(o.connection,receive,&o)))
                throw std::runtime_error("SimConnect dispatch failed/disconnected");
            if (stopping) break; // Do not issue another read after a terminal callback.
            const auto now = Clock::now();
            const double elapsed = std::chrono::duration<double>(now-o.began).count();
            if (!o.opened && elapsed >= 15.) throw std::runtime_error("Connection handshake timeout");
            if (elapsed >= duration) break;
            if (o.connected) {
                for (int i=0;i<brake_diag::COUNT;++i) {
                    if (o.parking_only && i!=0) continue;
                    if (o.state.pending[i] && std::chrono::duration<double>(now-o.sent[i]).count() >= 2.) {
                        ++o.missing[i];
                        const double age = std::chrono::duration<double>(now-o.sent[i]).count();
                        o.row(i,"RESPONSE_TIMEOUT","",o.state.pending[i],age);
                        o.event("RESPONSE_TIMEOUT",",\"request_id\":"+std::to_string(o.state.pending[i])+
                            ",\"definition_id\":"+std::to_string(i+1)+",\"ignored_callbacks\":"+std::to_string(o.ignored));
                        o.state.pending[i]=0;
                        if (i == 0) throw std::runtime_error("RESPONSE_TIMEOUT: Parking-brake responses unavailable; inspect callback_events.jsonl (not proof of unsupported variable)");
                    }
                }
                if (now >= next) {
                    for (int i=0;i<brake_diag::COUNT;++i) if (!o.state.pending[i]) {
                        if (o.parking_only && i!=0) continue;
                        const auto token = o.state.request(i);
                        if (!token) throw std::runtime_error("Request IDs exhausted");
                        o.sent[i] = now;
                        if (FAILED(o.track("RequestDataOnSimObject",SimConnect_RequestDataOnSimObject(o.connection,token,i+1,
                                SIMCONNECT_OBJECT_ID_USER,SIMCONNECT_PERIOD_ONCE),token,i+1)))
                            throw std::runtime_error("REQUEST_FAILURE: read request rejected");
                    }
                    // Deceleration capture needs observations during a subsecond event.
                    // One paired request at a time; do not accumulate SDK requests.
                    next = now+std::chrono::milliseconds(o.parking_only ? 20 : 250);
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        if (stopping && !o.failed) o.reason = "operator_cancelled";
        if (!o.opened && !o.failed) throw std::runtime_error("CONNECTION_FAILURE: No completed handshake");
        if (!o.received[0] && !o.failed) throw std::runtime_error("MISSING_VALID_VALUE: No valid parking-brake measurements");
    } catch (const std::exception& error) { o.failed=true; o.reason=error.what(); }
    // Close only this observer's handle; no worker, brake, pause or BRUNNER command.
    if (o.connection) {
        if (FAILED(SimConnect_Close(o.connection))) { o.failed=true; o.reason="SDK close failed"; }
        o.connection=nullptr;
    }
    o.state.disconnect();
    o.csv.close();
    try { o.event("SESSION_END",",\"reason\":"+json_string(o.reason)); }
    catch (const std::exception& error) { o.failed=true; o.reason=error.what(); }
    o.events.close();
    if (!folder.empty()) {
        std::ofstream summary(folder/"brake_summary.json");
        summary << "{\"state\":\"" << (o.failed?"FAILED":"CLOSED") << "\",\"reason\":\"";
        // Escape exception messages before JSON publication.
        for (char c:o.reason) { if(c=='"'||c=='\\') summary << '\\'; if(c>=32) summary << c; }
        summary << "\",\"closed_utc\":\"" << utc() << "\",\"opened\":" << (o.opened?"true":"false")
            << ",\"ignored_callbacks\":" << o.ignored
            << ",\"callbacks\":" << o.callbacks << ",\"sdk_exceptions\":" << o.exceptions
            << ",\"response_object_id\":" << (o.object_known?std::to_string(o.object_id):"null")
            << ",\"source_acquisition_timestamp\":null,\"simultaneous_channels\":" << (o.parking_only?"true":"false")
            << ",\"parking_only\":" << (o.parking_only?"true":"false") << ",\"channels\":[";
        for(int i=0;i<brake_diag::COUNT;++i) {
            if(i) summary << ',';
            summary << "{\"simvar\":\"" << brake_diag::names[i] << "\",\"unit\":\"" << brake_diag::units[i]
                << "\",\"valid\":" << o.received[i] << ",\"invalid\":" << o.invalid[i]
                << ",\"missing\":" << o.missing[i] << ",\"first\":" << (o.first[i].empty()?"null":o.first[i])
                << ",\"last\":" << (o.last[i].empty()?"null":o.last[i]) << '}';
        }
        summary << "]}\n"; summary.flush();
        if (!summary) { std::cerr << "Summary save failed\n"; return 1; }
    }
    std::cout << (o.failed?"FAILED: ":"CLOSED: ") << o.reason << std::endl;
    return o.failed?1:0;
}
