// Read-only diagnostic semantics, independent of all legacy telemetry layouts.
#pragma once
#include <cmath>
#include <cstdint>
#include <string>
#include <sstream>
#include <iomanip>
namespace brake_diag {
constexpr int COUNT = 4;
inline const char* names[COUNT] = {"BRAKE PARKING POSITION", "BRAKE LEFT POSITION",
    "BRAKE RIGHT POSITION", "SIM TIME"};
inline const char* units[COUNT] = {"bool", "position", "position", "seconds"};
inline bool valid(int channel, double value) {
    if (channel < 0 || channel >= COUNT || !std::isfinite(value)) return false;
    if (channel == 0) return value == 0. || value == 1.;
    return channel == 3 ? value >= 0. : value >= 0. && value <= 1.;
}
struct State {
    uint32_t next = 100, pending[COUNT] = {};
    uint64_t sequence[COUNT] = {};
    uint32_t request(int channel) {
        if (next == UINT32_MAX) return 0;
        return pending[channel] = ++next;
    }
    const char* rejection(int channel, uint32_t request, unsigned count, bool flags_default,
                          bool payload_complete) const {
        if (channel < 0 || channel >= COUNT) return "unknown_definition";
        if (!pending[channel]) return "no_outstanding_request_duplicate_or_cached";
        if (pending[channel] != request) return "request_id_mismatch_or_superseded";
        if (count != 1) return "unexpected_datum_count";
        if (!flags_default) return "unexpected_request_flags";
        if (!payload_complete) return "incomplete_double_payload";
        return nullptr;
    }
    bool accept(int channel, uint32_t request, unsigned count, bool flags_default,
                bool payload_complete) {
        if (rejection(channel,request,count,flags_default,payload_complete)) return false;
        pending[channel] = 0;
        ++sequence[channel];
        return true;
    }
    void disconnect() { for (auto& id : pending) id = 0; }
};
inline std::string number(double value) {
    std::ostringstream out;
    out << std::setprecision(17) << value;
    return out.str();
}
inline std::string display(int channel, double value) {
    if (!valid(channel, value)) return "UNAVAILABLE";
    return channel == 0 ? (value == 1. ? "ON" : "OFF") : number(value);
}
}
