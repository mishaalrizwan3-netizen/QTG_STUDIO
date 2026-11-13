# Stage 5G: read-only parking-brake verification

This is a separate opt-in SDK observer, not a QTG test mode. No test worker, GUI,
BRUNNER connection, Universal Initialization Manager or installed bridge DLL is
loaded. No aircraft initialization, brake event, pause/unpause, actuator write,
PID or recording command exists in the executable. It does not launch P3D.

## Genuine sources and units

The [Prepar3D v5 SDK simulation-variable table](https://www.prepar3d.com/SDKv5/sdk/references/variables/simulation_variables.html)
and [v4 SDK table](https://www.prepar3d.com/SDKv4/sdk/references/variables/simulation_variables.html)
document:

| SimVar | Requested unit | Interpretation |
|---|---|---|
| BRAKE PARKING POSITION | bool | 0 OFF, 1 ON |
| BRAKE LEFT POSITION | position | Normalized 0 released to 1 full simulated left braking |
| BRAKE RIGHT POSITION | position | Normalized 0 released to 1 full simulated right braking |
| SIM TIME | seconds | Separate simulator elapsed-time observation |

The SDK describes the wheel-brake channels as brake *effect*. They are not
authoritative physical pedal positions or measured brake force. Parking brakes
may also influence these readings. Differential pedal tests with parking OFF
are needed before interpreting them as evidence of pedal braking on this aircraft.
Older v2 documentation uses `position 32k` for wheel brakes; this tool explicitly
requests v4/v5 `position`, and never divides a value by 32767 or invents a percent.
Values outside documented domains are logged INVALID with their raw value.

No legacy `brakeParkingPosition` field is used. In particular, reserved slots
populated with SIM ON GROUND are not brake measurements. Historical CSVs and
sidecars remain untouched; the missing historical brake state stays unknown.

## Why a separate executable

The installed bridge exposes no usable genuine brake read API and does not expose
its SimConnect handle for adding diagnostic definitions. Its auxiliary setters
are disabled and are not a read interface. Reusing a named legacy field would
produce false provenance. Rather than alter/redeploy that working DLL or expand
the 23-measurement ABI, this executable uses the same established v4.5 SDK/client
library in its own process, with exactly one owned read-only SimConnect connection.
It is a narrow diagnostic, not another QTG telemetry/cache architecture.

Initial validation requires QTG Studio, its workers and diagnostic-only connection
owners to be closed. This tool neither discovers nor borrows their handles. It
closes only its own connection. Concurrent operation alongside a QTG worker is
not yet validated or authorized by this stage's stationary checklist.

No changes to native bridge source, headers, exports, installed binary or Python
worker interfaces were necessary. No DLL deployment is required.

## Build and offline evidence

From the application directory, an optional rebuild command is:

```powershell
python -B tools/build_parking_brake_diagnostic.py
```

It builds to a new unique `build/stage5g_*` folder and runs only `--help`. It
never copies to native/bin. The manifest records executable/source SHA-256 and
the installed DLL hash before and after the build.

Prepared executable:

`build/stage5g_20261111T052556Z_c30e64a3/parking_brake_diagnostic.exe`

This is the corrected RESPONSE_TIMEOUT build. The earlier executable and its
evidence are retained; do not use the earlier `051505Z_6321822d` build for retesting.

Installed worker DLL remained:

`d6ffc5f25d3931558572f81c6184678f2d48cca6e7e605a2e4cf160de238fc7c`

New offline tests compile and invoke the actual native callback with fake packets.
They cover OFF/ON, independent wheel channels, paused unchanged new responses,
duplicate/superseded callbacks, short/incomplete payloads, nonfinite/out-of-domain
values, late responses, disconnection, cancellation and ID exhaustion. A source
audit limits SDK calls to Open/Close, AddToDataDefinition, SubscribeToSystemEvent,
CallDispatch and RequestDataOnSimObject. No live connection is opened by tests.

## Supervised stationary test — operator actions only

1. Keep both Universal switches disabled. Close QTG Studio, workers and other QTG
   diagnostics. Have the operator establish a safe stationary aircraft, preferably
   engine OFF, on level ground. No full-power run is required. The tool will not
   secure the aircraft or change pause/brakes for you.
2. In PowerShell:

   ```powershell
   Set-Location -LiteralPath '<PROJECT_ROOT>'
   $env:QTG_UNIVERSAL_ENGINE_ACCELERATION = '0'
   $env:QTG_UNIVERSAL_ENGINE_DECELERATION = '0'
   & '.\build\stage5g_20261111T052556Z_c30e64a3\parking_brake_diagnostic.exe' --connect-read-only 'D:\QTG_Test_Results\brake_diagnostics' --duration-s 60
   ```

   This is an explicit live connection command for operator use, not executed by
   Codex. The terminal prints the unique evidence folder and CONNECTED after OPEN.
3. Observe the cockpit parking-brake state. Under safe conditions the operator
   manually performs OFF → ON → OFF → ON, holding each state for several samples.
   Confirm the raw parking value changes 0 → 1 → 0 → 1 and console OFF/ON agrees.
   Note UTC/operator action times independently. A connection message alone is
   not a valid brake measurement.
4. If safe, parking OFF and engine OFF, manually apply/release left and right
   toe brakes separately. Check independent normalized wheel-brake changes.
   Record unsupported, coupled or unexpected aircraft behavior rather than
   inferring physical pedal values. Do not change BRUNNER settings or assignments.
5. Observe while paused and, only if safe, operator-unpaused. New correlated
   responses may legitimately be unchanged when paused. Pause state starts -1
   UNKNOWN until the Pause event arrives; it is never inferred from packet traffic.
   SIM TIME is separately timestamped; no single frame proves physics progression.
   If paused ONCE requests are unsupported by this installation, expect timeout,
   not fabricated data. Diagnose that outcome before further integration.
6. Stop with Ctrl+C, or let the bounded 60-second session finish. Confirm CLOSED
   and inspect both files. On FAILED, retain evidence and stop. Disconnect/fault
   injection needs separate supervision; the observer never reconnects itself.

The connection handshake has a 15-second deadline, individual responses have a
2-second diagnostic deadline, and requested duration is capped at 600 seconds.
These diagnostic limits are not QTG acceptance tolerances. Only explicit
`--connect-read-only` connects; `--help` and incorrect arguments do not.

## Logs and timing limitations

The session folder contains `brake_samples.csv`, `brake_summary.json` and
`callback_events.jsonl`. The latter records each callback ID/size, matching data's
request/definition/server-object IDs, datum count/flags and explicit rejection
reasons. SDK calls record HRESULT and send ID; SDK exceptions record numeric
code, send ID, index and the correlated operation/request/definition when known.
No received exception is silently converted into a response timeout. CSV rows
include UTC observation, process-scoped generation, unique request ID, per-channel
sequence, callback QPC ticks/frequency, session elapsed time, request-to-receipt
duration, running state, exact SimVar/unit, raw value and status. New responses
are requested every 250 ms when no prior request is pending. Duplicated/cached,
superseded, wrong-object and incomplete responses are rejected, not reused.
Counters identify ignored callbacks. No automatic reconnect means generation 1
is meaningful only together with this process/session, never across runs.

NEW_CORRELATED_RESPONSE proves a newly delivered response to this observer's read
request. It does not prove an internal sensor acquisition time, aircraft physical
correctness or simulation progression. Timeout rows have no receipt timestamp and
are explicitly host observations. Channels are independently requested and are
not an atomic multi-channel snapshot. This includes SIM TIME. The summary retains
valid/invalid/missing counts and first/last valid raw values, not just the final
sample. Native SDK exceptions fail visibly, including unsupported definitions;
no alternate channel or zero fills the gap.

## Proposed recording capture — not implemented or activated

1. Validate the stationary observer first. Then separately validate simultaneous
   SDK observer/worker sessions, unchanged legacy initialization/replay, disconnect
   behavior and safe ownership cleanup before using it alongside a QTG run.
   Alternatively add a separately versioned optional brake block on the existing
   bridge connection in a future reviewed stage; do not expand the 23-channel ABI.
2. Preserve raw reference files. Create a new brake-evidence sidecar bound to the
   test/mode, recording UUID, exact CSV and initialization-sidecar hashes, observer
   session/process and documented source/units. Never infer historical brake state.
3. Establish a measured time anchor between the recording timer and host QPC/UTC
   at recording start. Validate clock-domain conversion rather than aligning
   recordings by filenames or file modification times. A clock change invalidates
   alignment. Record pre-start, recording-start and continuous pre-throttle samples.
4. Determine throttle onset using the existing measured-throttle detector after
   the run. Select the last valid brake observation strictly before onset, retaining
   its time separation and any gaps. The current 250-ms request schedule cannot
   establish exact state at a sub-frame event. Set an explicitly reviewed capture
   cadence/gap criterion before claiming an immediate pre-event measurement;
   otherwise report it UNVERIFIED. Never interpolate discrete parking-brake state.
5. Include left/right simulated brake application alongside parking state, not as
   substitutes. Log unknown/invalid/missing values and transitions throughout.
   This sidecar remains informational: no recording block, brake restoration,
   throttle/replay change or manager authorization is introduced by Stage 5G.

## Initial Stage 5G offline results, 11 November 2026

- New brake tests: 4 passed, including the compiled native callback harness.
- Focused combined brake, diagnostic-only, snapshot diagnostic, Python/native
  snapshot and initialization-compatibility regressions: 108 passed, 0 failures,
  0 errors, 0 skips. Log: `build/stage5g_20261111T051505Z_6321822d/offline_regression.log`.
- Standalone executable build and offline `--help` check passed. Executable SHA-256:
  `703d997417f180526ce7e804c688259f5d21923458dec7f10c8f16ba3696c6fe`.
- No live P3D test was performed. SimConnect runtime loading for `--help` is not
  evidence that a live aircraft's brake channels work; complete the supervised check.

## RESPONSE_TIMEOUT investigation and correction

Supplied evidence folder: `D:\QTG_Test_Results\brake_diagnostics\brakes_7904_5224218`.
The summary records four ignored callbacks, zero accepted values on every channel
and running state 1 in the timeout CSV row. Requests therefore cannot be described
as having received no callbacks. The first parking timeout terminated the session;
zero timeout counts on the other channels do not establish that they were valid.

Confirmed defect: the callback compared `dwObjectID` against
`SIMCONNECT_OBJECT_ID_USER` (0). The installed SDK header explicitly calls this
a **proxy** for user-vehicle requests. The SDK describes response `dwObjectID`
as the **server-defined** ID. A fully correlated response carrying ID 1 was
rejected offline by the old implementation; the old mock had incorrectly assumed
that the response echoed the proxy. The [SDK's request/response example](https://www.prepar3d.com/SDKv5/sdk/simconnect_api/references/general_functions.html)
matches the request ID rather than comparing the response ID to that proxy.

The corrected callback matches only outstanding private requests sent to USER,
checks definition/count/flags/payload, and binds the returned actual object ID
for consistency within this connection. It does not hard-code either 0 or 1.
An aircraft/object change fails the session instead of silently rebinding.
Regressions cover returned IDs 1 and 37, duplicate/superseded/malformed packets,
changed aircraft and exception-to-send correlation. The old filter fails the new
ID-1 reproducer; the corrected filter passes it.

The original log did not retain object IDs or rejection reasons, so the exact
four discarded packets cannot be reconstructed. Their rejection pattern is
consistent with the confirmed filter defect, not proof that the brake variable
was unsupported. No timeout, measurement domain or acceptance threshold was
relaxed. The next supervised run must establish actual valid OFF and ON values
before claiming live success. Preserve all three new evidence files on failure.

The updated executable SHA-256 is:
`1328e80fd37577a0f0414c317aad2eb4cc64a24fded62e75fea81ad185dc9953`.
Connection failure, definition failure, request failure, invalid/missing values,
SDK exception and response timeout now have distinct messages/log records.

Correction validation: 108 focused offline tests passed, 0 failures, 0 errors,
0 skips. Build and `--help` passed. Updated log:
`build/stage5g_20261111T052556Z_c30e64a3/offline_regression.log`.
The installed worker DLL and original 23-channel verification-header SHA-256
values were rechecked unchanged. No live retest was performed by Codex.
