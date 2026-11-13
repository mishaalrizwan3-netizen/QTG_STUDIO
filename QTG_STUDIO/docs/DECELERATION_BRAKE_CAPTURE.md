# Stage 5H: optional Deceleration parking-brake evidence

Disabled by default. This adds informational sidecars, not a brake setting,
readiness requirement, replay channel or replacement QTG recording format.
No live coexistence or engine test was performed by Codex.

## Ownership and integration review

The installed bridge hides its connection handle and has no genuine brake read
export. Borrowing a legacy struct slot is invalid. Its 23-channel verification
ABI and installed DLL remain unchanged. The separately built SDK observer owns
one connection in its own process; it does not load or call the worker DLL.
Request/definition IDs and dispatch state belong to that connection, independently
of the worker. It only opens/closes, registers read definitions, requests data,
pumps messages, observes Pause and reads send IDs. There are no brake, weather,
position, engine, pause/unpause or BRUNNER commands.

The [Prepar3D SDK SimConnect_Open documentation](https://www.prepar3d.com/SDKv5/sdk/simconnect_api/references/general_functions.html)
describes separate client connections and their corresponding owned handles.
This supports the architecture; it does not establish that extra scheduling/SDK
load has no effect on this simulator. That remains a supervised validation gate.

The companion caller uses a named ownership mutex per worker context. It never
sends a worker command or writes its status file. A Windows regression experiment
found that atomic replacement still failed while a reader held the destination
open, despite requesting delete sharing. The final design therefore reads status
only once, before recording, and **does not reopen it during active capture**.
The one short preflight read can coincide with an ordinary status retry; it is
not held open. A regression verifies no further status reads occur. Its own
console/logs/stop file are isolated in a unique capture folder. Native shutdown
closes only its own handle. A stop file, caller-process lifetime observation and
bounded lifetime prevent a detached observer. The caller can terminate only the
child process it created if graceful shutdown fails. No reconnect is attempted.

The only hooks in the existing recording loops copy their existing `t0` into
memory after its assignment. The marker is preloaded and QPC mapping checked
before hardware startup. It does no disk I/O, SDK read, sleep, acquisition or
control operation in the recording loop. With the flag off the marker is absent.
With it on, only 1.f.(2) Manual/Recorded Replay is eligible; PID and other tests
are excluded. Metadata errors cannot raise into recording. A new initialization
clears old marker evidence. The ordinary timing, preparation, unpause, recording,
throttle/replay commands and hardware ownership remain in their existing code.

The exact same marker is included in new QTG context/`.qtg_test.json` metadata.
The observer buffers first, then recovers the marker from completed-run metadata
after recording/reporting. It watches only new metadata in that test's result
directory, not historical references or the entire results tree. Unique marker,
mode, worker-command-directory and capture-time checks prevent attaching an old
or unrelated run. Missing/failed report metadata cannot yield invented alignment.
The primary recording CSV, initialization sidecar format, historical references,
analysis/timing calculations, RPM limits and readiness logic are unchanged.

## Native payload, timing and validity

The new observer mode is `--parking-only`. It registers exactly these two fields
in one FLOAT64 data definition and copies one validated callback payload:

- BRAKE PARKING POSITION, bool, exactly 0 OFF / 1 ON.
- SIM TIME, seconds, finite and nonnegative.

It **does not register, request or decode left/right wheel-brake channels**.
Those remain excluded until their separate decoding issue is resolved.

Both values receive the same callback QPC timestamp, request ID and sequence.
Responses must match the pending private request, definition, count, flags,
payload size and stable returned object ID. The native two-second response
deadline is unchanged. Identical paused values may be newly delivered responses;
they do not establish physics progression. Paused/running/unknown is recorded.

The parking-only mode schedules one request at a time with a nominal 20-ms cadence.
Actual SDK response intervals, request-to-receipt delay and gaps are evidence,
not assumed performance. The generic older diagnostic mode remains at 250 ms.

The existing QTG timer uses Python perf_counter. On Windows the marker verifies
the relationship to native QPC by a bracketed host-clock read and records its
frequency. No Unix-wall-clock-to-monotonic subtraction is used for alignment.
UTC remains useful for operators; alignment depends on QPC, not filenames/UTC.
Failed domain/frequency checks, backward simulator time, connection/sequence
changes and invalid data produce unavailable alignment, never inferred OFF.

Callback receipt/request correlation is available; **internal simulator acquisition
time remains unavailable**. Fresh response delivery is not proof of physical brake
effect. Source timestamps/units/raw values remain in the native CSV.

The aligned CSV shows the last prior brake observation and its timestamp/gap,
alongside copied QTG throttle and ground-speed columns. It never interpolates
parking-brake state or claims a value was sampled exactly at the QTG row time.
The diagnostic age budget includes both the prior-observation gap and native
request-to-receipt duration. Exceeding the operator's diagnostic age criterion,
an invalid observation or capture shutdown produces UNAVAILABLE with a blank value.
Zero is a valid OFF measurement. The criterion is not a QTG acceptance tolerance.

The existing `manual_reference_preflight.analyze_reference()` determines measured
throttle onset without changing the recorded timeline. The summary includes
recording-start and last-strictly-before-onset observations with explicit gaps.
Raw paired samples and aligned rows cover the complete recorded manoeuvre,
including the measured event. Exact instantaneous state between samples is unknown.

## Supervised validation — no automatic QTG test

Use the source application for this experiment; an old packaged executable does
not contain the new clock marker. Keep both Universal switches off throughout.

1. Before launch, in PowerShell:

   ```powershell
   Set-Location -LiteralPath '<PROJECT_ROOT>'
   $env:QTG_UNIVERSAL_ENGINE_ACCELERATION = '0'
   $env:QTG_UNIVERSAL_ENGINE_DECELERATION = '0'
   $env:QTG_DECELERATION_BRAKE_CAPTURE = '1'
   python -B main.py
   ```

2. Under normal operator supervision, prepare the Deceleration panel using its
   existing procedure. Do not change brakes or throttle to make the diagnostic
   pass. First validate coexistence with an already-prepared **paused, idle worker**
   (idle means not recording, not engine idle). Opening a test panel can initialize
   aircraft conditions; do that only as part of the approved normal setup.
   Do not click Record or unpause for the initial coexistence check.

3. Obtain the exact active worker context from its `--brunner-test` process command:

   ```powershell
   Get-CimInstance Win32_Process |
       Where-Object { $_.CommandLine -match '--brunner-test' } |
       Select-Object ProcessId, CommandLine
   ```

   Select the intended Deceleration worker's `D:\QTG_Test_Results\sessions\..._context.json`.
   Do not select a Stage 5E diagnostic-only context or guess from file dates.

4. In a second terminal, set the same three environment variables as step 1 and:

   ```powershell
   Set-Location -LiteralPath '<PROJECT_ROOT>'
   $context = '<exact active Deceleration worker context.json path>'
   $observer = '.\build\stage5h_20261111T054732Z_3f925a6c\parking_brake_diagnostic.exe'
   python -B tools/deceleration_brake_capture.py start --context "$context" --executable "$observer" --output 'D:\QTG_Test_Results\brake_capture' --duration-s 30 --max-age-s 0.1
   ```

   The initial check ends without recording or alignment; exit code 1 for absent
   alignment is expected in this no-recording check. Inspect the private native
   CSV/summary/callback log, require valid paired brake/time responses, and confirm
   the normal worker remains connected, responsive and paused. If separately safe
   and approved, the operator can toggle parking brake while paused to validate
   ON/OFF with both clients present. No full-power manoeuvre is needed for this check.
   Stop on unexpected pauses, disconnects, SDK exceptions or delayed worker updates.

5. Only after that passes and an actual test is separately authorized, run the
   same companion command with `--duration-s 180` **before** clicking Record. Wait
   for `Native parking response observed`. The tool does not click Record, alter
   its availability or block an otherwise permitted run. Perform Manual exactly
   as before: initial conditions, recording, operator throttle reduction.
   Never toggle brakes merely to make the two datasets agree.

6. The caller buffers through recording and report generation. Once the saved
   CSV/metadata become available, it recovers the recording marker and stops its
   own observer. If necessary, stop it with Ctrl+C after QTG finishes and align
   offline later. It never monitors the live status file while recording. It binds that
   CSV by the unique recording ID in its metadata, then saves separate alignment
   files. For Automatic Replay repeat with a new companion capture, correct worker
   context and the exact selected Manual reference. No observer or brake state
   is replayed into P3D. Close/restart the application between experiments if that
   is the established supervised workflow.

7. Review `parking_brake_alignment.json`: recording-start status, pre-onset value,
   sample gap/age, native valid-pair count, issues and source/CSV hashes. Review the
   full aligned CSV during the event. Confirm both OFF and ON cases against cockpit
   evidence where safely possible, and compare native/worker sampling gaps against
   the baseline. The `0.1`-second criterion above is an operator-selected diagnostic
   quality check, not an engineering-approved instantaneous-state tolerance.
   Do not draw left/right pedal conclusions or qualify a run from matching graphs.

8. Test diagnostic-only cancellation while the worker is idle: Ctrl+C in the
   companion terminal must stop/close only its observer; worker state must persist.
   Simulator disconnect/fault injection requires separate supervision and must
   leave brake values unavailable. If a QTG run needs stopping, use QTG's existing
   operator Stop; the diagnostic will never send that command on your behalf.

## Outputs and recovery

Each new capture folder contains `capture.json`, `observer_console.log`, a native
session subfolder with raw CSV/JSON/callback evidence, and, when alignment succeeds:

- `parking_brake_aligned.csv`: copied QTG time/throttle/ground speed plus explicitly
  aged native brake observations/status; does not replace the QTG CSV.
- `parking_brake_alignment.json`: onset/start diagnostics, clock marker, hashes,
  exclusions and quality gaps.

The automatic path waits only within the requested lifetime (maximum 600 s).
Observer failure or missing clock marker leaves alignment unavailable without
touching QTG execution. A stale heartbeat is rejected at attachment; it is not
polled during recording. A failed clock mapping cannot authorize alignment.
The native child watches its caller process and private stop file. A duplicate
capture for the same worker context is refused before launching a second observer.

If report generation finishes after the caller's deadline, align offline later:

```powershell
python -B tools/deceleration_brake_capture.py align --capture '<capture folder>\capture.json' --csv '<exact newly saved run.csv>'
```

The CSV must carry the matching recording marker; historical/unrelated CSVs are
rejected. Offline alignment writes a new unique subfolder and never connects to
P3D. Existing evidence is not overwritten. Missing marker/data remains unavailable.

To disable: close the companion and application, remove
`Env:QTG_DECELERATION_BRAKE_CAPTURE`, and relaunch normally. Leave both Universal
switches disabled. No DLL deployment or rollback is necessary.

## Files and offline validation

New: `backend/brake_recording.py`, `tools/deceleration_brake_capture.py`,
`tests/test_brake_recording.py`, this guide.

Modified: `qtg_common_engine.py`
(optional preload and old-marker cleanup),
`interfaces/brunner_base/Super_Mushak_Logger_GUI.py` (two metadata hooks and helper),
`tools/parking_brake_diagnostic.cpp` (paired parking-only mode/owned shutdown),
`tools/build_parking_brake_diagnostic.py` (new staged executable/protocol manifest),
`tests/native_parking_brake.cpp`, `tests/test_parking_brake_diagnostic.py`,
`tests/test_airborne_preparation.py` (load the helper in the isolated AST fixture).

The paired executable built successfully; only its offline `--help` was executed.
Its SHA-256 is `38f341c23da72ded12c9e925b3f0cd291ba02fb64b4b680975059977371a9e88`.
Installed worker DLL SHA-256 remains
`d6ffc5f25d3931558572f81c6184678f2d48cca6e7e605a2e4cf160de238fc7c`.
Native bridge sources, the 23-channel ABI and dashboard DLLs were not changed.

Live coexistence, true sampling gaps during the subsecond event, worker timing
impact and disconnect behavior still require the supervised steps above. No
readiness, recording-authorization or qualification claim is made by this feature.

Final offline results: **375 root tests passed, 0 failures, 0 errors, 0 skips**,
including 26 sidecar tests and the native paired-payload harness. Log:
`build/stage5h_20261111T054732Z_3f925a6c/post_review_root_regression.log`.

Separate legacy bridge discovery: 96 run, 94 passed, one failure and one error.
Both match the previously confirmed RPM issues:
`test_engine_acceleration_restores_transient_without_live_settling` expects the
old direct RPM setter call, and `test_engine_transient_wrong_readback_blocks`
has a missing `throttle` fixture field. Neither production RPM behavior nor those
legacy tests were changed. Log: `build/stage5h_20261111T054732Z_3f925a6c/legacy_regression.log`.

The Windows sharing experiment failed as described above; that design was removed,
and the passing regression now proves the final observer reads live worker status
only at preflight. This is not a claim that Windows permits replacing an open status
file. Installed worker, dashboard DLLs and the original verification-header hashes
were checked unchanged. No live P3D/BRUNNER operation or Git command was performed.
