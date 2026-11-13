# Stage 5D — supervised read-only native verification

Prepared 10 November 2026. **Installation and live testing have not been run.**
Universal Initialization stays disabled for both engine tests.

Stage 5E adds a separate [diagnostic-only connection startup](P3D_DIAGNOSTIC_ONLY.md).
Use that foreground mode for cold-start read-only verification while normal QTG
Studio/workers remain closed. It supplies a context for this same diagnostic
caller without test initialization or BRUNNER startup. The existing-worker route
below still requires an already-connected idle worker.

Offline results: **32 new caller tests passed**; full root suite **318 passed**;
legacy bridge suite **94 passed, 1 failure, 1 error** (the two previously confirmed
RPM fixture/expectation issues). No tests skipped. The caller tests are included
in the root count. They cover the command route, 23-channel display, borrowed
connection ownership, cache/supersession probes, unknown pause state, disconnection,
reconnect identity, cancellation, malformed data, logging and safe worker shutdown.
The final focused 32-test rerun also passed after explicitly labelling stored and
stopped readings as historical. Stage 5D logs/hash evidence are in
`build/stage5d_20261110_175102/`.

## Files and connection ownership

Console: `tools/p3d_verification_diagnostic.py`.
Worker caller: `backend/p3d_verification_diagnostic.py`.
The worker command handler accepts only two additional diagnostic actions. Normal
initialization, recording, replay and PID logic is unchanged. Starting an ordinary
QTG action stops this monitor's event; it does not wait for diagnostic I/O or block
the ordinary action. Worker shutdown waits for the reader before closing the
existing connection.

The console sends file IPC to an **existing** worker. The background reader uses
that worker's `p3d.verification_snapshot()`, which calls RequestVerificationV1 and
GetVerificationV1 on its existing DLL/connection. Neither process creates another
SimConnect client or touches BRUNNER. No connect, disconnect, positioning, weather,
throttle, pause/unpause, PID, recording or hardware-release command is provided.

Prerequisites: use the current **source application**, an already-connected idle
worker with the candidate DLL, no active preparation/recording/replay/PID and no
Universal verification sampler. Both engine environment switches must be off.
An old packaged EXE will not contain these new worker commands.

**Do not open a test panel solely to obtain a connection:** the existing worker
startup can automatically initialize the test. The caller does not change that
startup behaviour. If a suitable idle worker is unavailable after deployment,
stop; obtaining one through the normal workflow requires separate operator
authorization for its existing initialization/hardware side effects. This tool
does not supply a cold-start, zero-initialization worker or a connection fallback
through its existing-worker route. Use the explicit Stage 5E mode linked above.

## Preserved DLL and fresh independent backup

Production, unchanged:
`<PROJECT_ROOT>\interfaces\brunner_base\native\bin\p3d_bridge_engine_init_v1517.dll`

Original SHA-256:
`5797ad0efb2d8be2ebbf11882b42aef74b38d367a5d8a491b7d98e7ae2442c80`

Fresh independently verified backup, created without overwriting an older backup:
`D:\QTG_Bridge_Backups\Stage5D_20261110_174237_cd3910b9\p3d_bridge_engine_init_v1517.dll`

Its adjacent `backup_manifest.json` records the original path, hash and time
(`2026-11-10T17:42:37+05:00`). The backup is 81,408 bytes and matches the original.

Staged candidate:
`<PROJECT_ROOT>\build\stage5c_20261110_170331\p3d_bridge_engine_init_v1517.dll`

Candidate SHA-256:
`d6ffc5f25d3931558572f81c6184678f2d48cca6e7e605a2e4cf160de238fc7c`

All 64 legacy export names/ordinals are preserved; verification exports are 65/66.
Do not run the ordinary build script to deploy: it overwrites production and does
not use the staged ordinal-preserving DEF file.

## Future installation checklist — requires explicit operator authorization

1. Obtain authorization for this exact candidate, a stationary supervised P3D
   check and the normal setup needed to provide an idle worker. This document
   and the diagnostic program are not permission to deploy or start a test.
2. Finish any test safely using the normal operator procedure; close QTG Studio
   and **all workers**. Verify the production DLL is no longer loaded. Do not
   force-close a running test or replace a loaded DLL.
3. Verify the original, fresh backup and candidate hashes above. If production has
   changed, stop: preserve that version independently and review before replacing.
   Keep the backup folder outside the repository intact.
4. Set both engine switches to zero in the source application's launch shell:

   ```powershell
   $env:QTG_UNIVERSAL_ENGINE_ACCELERATION = '0'
   $env:QTG_UNIVERSAL_ENGINE_DECELERATION = '0'
   ```

5. Only after authorization and shutdown, copy the single candidate DLL to the
   exact production worker path. Do not replace `QTG_STUDIO\p3d_bridge.dll`,
   `QTG_STUDIO\bridge\bin\qtg_p3d_bridge.dll` or any Python/reference files.
   Verify the installed file's SHA-256 equals the candidate hash before launch.
6. For read-only verification, use the explicit Stage 5E diagnostic-only mode
   linked above; do not launch a test panel to obtain a connection. For separately
   authorized normal legacy-operation checks, launch the source application and
   establish the separately authorized normal
   connection/setup, then stop at an idle state with no QTG recording or replay.
   Confirm the actual worker's loaded DLL path and candidate on-disk hash. If no
   idle worker exists without further initialization, stop for that authorization.

No step above has been executed by Stage 5D, except creating/verifying the backup.

## Select the live worker explicitly

Use the running worker's `--brunner-test ..._context.json` command-line argument,
not the newest file guessed from the sessions folder. Windows Task Manager can
show the Command line column, or inspect it read-only:

```powershell
Get-CimInstance Win32_Process |
  Where-Object { $_.Name -eq 'python.exe' -and $_.CommandLine -match '--brunner-test' } |
  Select-Object ProcessId, CommandLine | Format-List
```

In the commands below, set `$context` to that **current worker's exact context
path**. A historical session ID is deliberately not hard-coded. If there are
multiple workers, verify the selected one is the intended idle Deceleration
worker. The console requires a matching worker status with a heartbeat no older
than five seconds; this transport check is not a telemetry freshness criterion.

```powershell
Set-Location -LiteralPath '<PROJECT_ROOT>'
$context = 'D:\QTG_Test_Results\sessions\<current-worker-session-id>_context.json'
```

Replace the placeholder before running. New diagnostic commands expire after
five seconds so a dead session cannot leave a delayed diagnostic start queued.

## Supervised live checks — not automatic

Start with a stationary aircraft, secured as required by the operator, no active
test, and P3D intentionally paused. The supervisor owns all changes to P3D state.
This caller never changes pause state, aircraft controls or connection ownership.

1. **Normal read / intentionally paused:**

   ```powershell
   python -B tools/p3d_verification_diagnostic.py --context "$context" start --max-age-s 1 --duration-s 30
   ```

   `1` second is an explicitly chosen **diagnostic display criterion**, not an
   engineering readiness limit. Every accepted response should have a new request
   ID, native sequence and receipt timing. Values may legitimately be identical
   while paused; unchanged simulator time does not invalidate a new response.
   Running state may be Unknown if the existing Pause observer has not received
   an event. Mark that check UNVERIFIED; the diagnostic does not force a pause
   transition or install/reassert aircraft-control commands to make it pass.

2. **Check native values:** compare all 23 displayed measurements with independent
   cockpit/instrument/reference observations as available. Specifically compare
   ground contact, genuine flap handle, IAS, MSL and indicated altitude, independent
   engine/propeller RPM, throttle, OAT, ambient pressure and altimeter setting.
   Ambient pressure and the altimeter setting are different measurements; do not
   expect them to be numerically identical at nonzero altitude. Missing values
   must remain Not available, never zero-filled or substituted.

3. **Flap/ground separation:** while stationary and only if separately approved,
   the operator may select two flap detents through normal cockpit controls.
   Observe genuine FLAPS HANDLE INDEX change while SIM ON GROUND stays true.
   The diagnostic issues no flap/BRUNNER commands. Never infer flap accuracy just
   because a validity bit is set or the aircraft is on the ground.

4. **Running/paused:** only if the supervisor approves safe stationary running,
   manually unpause P3D and observe simulator elapsed time advance, then manually
   pause again. Observe running-state changes and fresh responses after both
   transitions. Stop if aircraft behaviour is unexpected. A new native packet
   does not prove every physics component is updating; a frozen time observation
   with Running true is shown explicitly. Unknown state remains UNVERIFIED.

5. **Cache-copy/staleness probe:** after the previous monitor stops:

   ```powershell
   python -B tools/p3d_verification_diagnostic.py --context "$context" start --max-age-s 1 --duration-s 5 --probe cache
   ```

   This sends one seed request, then only calls GetVerificationV1. With no other
   verification reader, native request ID, sequence and receipt time stay fixed
   while receipt age grows. Copies show CACHED_COPY_NOT_REQUESTED and eventually
   exceed the chosen age criterion. Copying/exporting data must never become
   another correlated sample. If another reader updates the snapshot, the cache
   probe still does not claim correlation for that reader's request.

6. **Superseded requests:** after that monitor stops:

   ```powershell
   python -B tools/p3d_verification_diagnostic.py --context "$context" start --max-age-s 1 --duration-s 5 --probe superseded
   ```

   The caller sends two back-to-back read requests and accepts only the newest
   token on the same connection generation. Old export copies are ignored and
   counted. This does **not** prove an obsolete server callback arrived late:
   the bridge does not expose a rejected-callback counter. The result explicitly
   says that delivery is not observable. Offline tests cover actual old-response
   rejection. Do not label late-callback delivery VERIFIED from token ordering.

7. **Connection loss/reconnect:** a separate supervised fault-injection action
   must be approved; the caller never disconnects/reconnects or launches a client.
   Observe DISCONNECTED/UNAVAILABLE with no fresh sample during loss. If the
   existing owner restores the same process's connection, require a new generation
   and reject old identities. This legacy client does not automatically reconnect
   after its established handshake; reconnect may require a separately approved
   owner lifecycle. Do not force it by starting a QTG initialization. If no such
   safe owner operation is available, leave live reconnect UNVERIFIED. A new
   process can restart generation at 1; do not compare it across different PIDs.

8. **Stop/cancel/repeat:** Ctrl+C requests only diagnostic stop. Alternatively,
   from another shell with the same `$context`:

   ```powershell
   python -B tools/p3d_verification_diagnostic.py --context "$context" stop
   python -B tools/p3d_verification_diagnostic.py --context "$context" status
   ```

   Confirm monitoring ends, the existing connection stays owned by its worker,
   QTG recording remains untouched and a second diagnostic produces a separate
   log folder. Starting any normal QTG action also stops diagnostic monitoring;
   do not start an actual test solely to exercise that offline-tested guard.

## Evidence and interpretation

The console displays all 23 channels with source SimVars, units and per-channel
validity, plus request ID, native sequence, generation, receipt age, running state,
simulation-time progression and client-selected DLL path/on-disk hash. A validity
bit plus finite value is not a measurement-accuracy certification. No diagnostic
result authorizes recording or changes manager readiness.

Private worker status: `<worker-status-stem>.verification.json`. It never writes
ordinary worker status or the existing Telemetry Diagnostic status. Each run saves
`snapshots.jsonl`, `measurements.csv` and `summary.json` under a separate UUID folder
in `sessions\p3d_verification`. The summary contains whole-session state counts,
channel availability/first-last evidence and connection transitions, not just the
last missing frame. File paths are printed with FILE EXISTS only after checking
them; logging/publication failures are exposed. Exit code 0 means caller completion,
not qualification or all checks passing. Mark each supervised criterion MATCHED,
NOT MATCHED or UNVERIFIED with its evidence.
The `status` command reads a stored status file; it does not acquire a new native
sample. The console shows observation UTC and receipt age **at that observation**;
stopped measurements are explicitly historical.

## Rollback — after any unexpected behaviour

1. Stop the diagnostic, safely stop any test through normal operator controls and
   close QTG Studio/all workers. Do not replace a loaded DLL or force-stop hardware.
2. Verify the fresh backup SHA-256 equals the original hash above.
3. Copy that backup DLL to the exact original production worker path, replacing
   only that file. Verify production's hash is restored to the original hash.
4. Keep both Universal switches disabled. Restart only under supervision and
   confirm the original loaded path/legacy operation. Retain candidate, logs and
   backup for review. No source rollback, reference rewrite or Git operation is
   required: the optional caller safely reports unsupported API on the old DLL.
