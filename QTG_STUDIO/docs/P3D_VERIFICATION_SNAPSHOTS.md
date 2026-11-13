# P3D verification snapshots V1 — Stage 5B

Both engine Universal Initialization switches remain disabled by default. This
change supplies read-only verification data; it is not approval of a live plan.

## ABI and provenance

The existing connection registers definition 19000. `RequestVerificationV1()`
returns a unique DWORD request ID or zero. IDs are never recycled, including
reconnects; exhaustion fails closed. Only one verification request is pending.
Ordinary stream and one-shot IDs are untouched.

`GetVerificationV1(out, size)` atomically copies a packed, versioned snapshot:
size/version, connected capability, response request ID, connection generation,
native response sequence, per-channel finite-value validity mask, callback QPC
receipt ticks, copy QPC ticks, QPC frequency, observed Pause-event running state
(unknown/paused/running), and 23 doubles. V1 size is 256 bytes (72 + 23*8).

The channel order, native names and units are explicit in
`interfaces/brunner_base/src/verification_snapshot.py::CHANNELS`, matching the
new native header. Channels cover ground contact, flap handle index, MSL and
indicated altitude, IAS, ground speed, position, attitude, heading, throttle,
independent engine/propeller RPM, OAT, ambient wind, altimeter setting, ambient
pressure, simulator elapsed time, manifold pressure, mixture and propeller lever.
Ambient pressure and altimeter setting remain separate quantities.

Native callbacks must match the current pending request, definition, complete
payload/count and untagged format. A consumed request cannot be counted twice.
Registration errors disable the capability. Disconnect/quit invalidate pending
requests; reconnect advances generation. Payload and associated metadata share
the existing critical section. This adds no simulator writes or connections.

Python serializes verification reads per client, checks version/size/generation
and request identity, then converts native QPC **age** into a conservative Python
monotonic receipt estimate. It does not compare QPC absolute ticks with Python
monotonic time or Unix time. Legacy cache timestamps are not used or modified.
The verifier rejects stale, repeated, reordered, nonfinite and reset-clock data.
The engine adapter rejects any mandatory field absent from the valid snapshot.

Receipt timing proves a newly delivered, correlated native response, not the
internal acquisition instant of every P3D subsystem. That timestamp remains
unavailable and is explicitly null. Advancing simulator elapsed time is separate
evidence, not proof that every physics component has updated. Unknown pause state
remains unknown. Freshly delivered paused values may legitimately be unchanged.
No new physics-progress gate or engineering threshold is introduced here.

Ground contact comes from `SIM ON GROUND`; flap handle comes independently from
`FLAPS HANDLE INDEX`. The misleading legacy flap slot, historical references,
CSV columns, diagnostic cache path and legacy stream layouts remain unchanged.
Old flap sidecar fields are **not retrospectively certified** by this change.

## Offline validation and native deployment

Tests cover the Python ABI/reader and the actual native snapshot header using
mocked SimConnect calls, including 10,000 concurrent atomic copies. A separate
MSVC `/Zs` test checks the complete production bridge against the installed SDK.
Neither test loads the production DLL or connects to P3D/BRUNNER.

Final offline results: root suite **286 passed**; legacy bridge suite **94 passed,
1 failed, 1 error** (96 executed). Total: **380 passed, 1 failure, 1 error**.
The 21 newly added tests, including the native harness and SDK compile-only
check, passed. No tests were skipped. The two legacy RPM cases also reproduce
in isolation: `test_engine_acceleration_restores_transient_without_live_settling`
expects a direct RPM setter call absent from the existing natural pre-roll path;
`test_engine_transient_wrong_readback_blocks` omits `throttle` from its reference
fixture and raises `KeyError`. Neither the tests nor that production path were
changed in Stage 5B.

The production DLL has not been rebuilt or replaced. Before supervised testing,
close QTG Studio, preserve the installed DLL outside the source tree, and use the
existing `interfaces/brunner_base/tools/build_bridge.cmd` in the normal build
environment. Verify x64 ABI, old exports, new exports and the deployed filename
`p3d_bridge_engine_init_v1517.dll`. First verify the existing manager-disabled
Manual/Replay paths, then observe verification-only paused/running/disconnected
and reconnected reads. Validate installed-SDK SimVar support, genuine ground/flap
values, elapsed-time/pause behavior and request latency. Unsupported older DLLs
remain usable on legacy paths; manager verification fails explicitly.

## Before supervised Deceleration activation

Approve separate Manual and Recorded Replay plans with named engineering review,
per-field target evidence, units and tolerances, explicit exclusions, freshness
deadline and at least three samples. CONFIGURED baseline is insufficient.
Review ground IAS/ground speed criteria, weather, attitude/position and optional
weight/CG/trim requirements without borrowing airborne tolerances. Existing RPM
tolerances remain unchanged. Replay must bind the exact selected CSV/sidecar and
recording-start state; independently establish genuine flap-target provenance
instead of approving the historical ground-status alias as a flap measurement.
Unsupported fields must be explicitly excluded with review or implemented and
validated before they can be required.

Keep Acceleration disabled. First validate snapshots with both switches off,
including frozen caches, delayed callbacks, disconnect/reconnect and cancellation.
Only after plan review conduct a supervised Deceleration-only experiment; verify
recording remains blocked on failure and legacy cleanup/control ownership is
preserved. No live simulator result is claimed by Stage 5B.
