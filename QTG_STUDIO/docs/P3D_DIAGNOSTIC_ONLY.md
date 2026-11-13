# Stage 5E — diagnostic-only P3D connection

This mode is explicitly selected. It does not launch P3D or the normal QTG GUI,
construct a test worker, initialize aircraft conditions, connect to BRUNNER or
start a test. Implementation/testing was offline. Live behaviour still requires
operator supervision; do not treat the offline results as simulator verification.

## Exact startup command — supervised use only

First close normal QTG Studio, its workers and other QTG diagnostic transports.
Do not run this mode alongside a test or replay. Manually establish a safe,
stationary P3D scenario under supervision, initially paused; the tool will not
pause or reposition it. P3D must already be running when the operator elects to
connect. No connection is created by `--help` or by the ordinary console commands.

From PowerShell in the source application directory:

```powershell
Set-Location -LiteralPath '<PROJECT_ROOT>'
$env:QTG_UNIVERSAL_ENGINE_ACCELERATION = '0'
$env:QTG_UNIVERSAL_ENGINE_DECELERATION = '0'
python -B tools/p3d_verification_diagnostic.py diagnostic-only --session-root 'D:\QTG_Test_Results\verification_sessions' --connection-timeout-s 15 --session-timeout-s 300
```

Keep this foreground terminal open. It prints the exact newly generated
`...\context.json` path, then `IDLE_CONNECTED` after handshake. If the DLL hash,
connection, runtime guard or handshake check fails, it stops with FAILED instead
of invoking a different startup/initialization path. It never starts Prepar3D.exe.

There is one owned SimConnect client in this process, using the existing
`SimConnectClient`/native bridge/poller. No transport architecture is duplicated.
An OS mutex prevents a second **diagnostic-only** owner for this project, even
with a different session-root folder. It does not control unrelated simulator
clients: closing existing QTG transports first remains an operator prerequisite.

## Reuse the existing verification caller

In a second PowerShell terminal, set `$context` to the exact path printed above;
do not select an old context or guess the newest normal QTG session.

```powershell
Set-Location -LiteralPath '<PROJECT_ROOT>'
$context = '<paste the printed absolute context.json path>'
python -B tools/p3d_verification_diagnostic.py --context "$context" start --max-age-s 1 --duration-s 30
```

This uses the same Stage 5D caller and snapshot analysis, with diagnostic file IPC
to this session rather than the ordinary QTG worker. It reads all 23 native
measurements plus validity, request ID, generation, sequence, receipt age and
running state. The one-second age is an operator-selected diagnostic criterion,
not an approved initialization tolerance. There is no READY state and no ability
to authorize recording. Status always has `can_record=false`, `recording=false`
and `controlling=false`.

The generated context includes `session_kind=p3d_verification_only`, owner PID,
command directory, status path and the pinned DLL hash. Context/status files and
CSV/JSON diagnostic evidence live only in the new unique session folder. No
historical recording/reference or existing QTG status file is overwritten.

## Audited startup side effects and bypasses

| Existing path | Side effect | Diagnostic-only treatment |
|---|---|---|
| `SuperMushakLoggerApp.__init__()` / `_start_hardware_connections()` | Starts GUI timer, BRUNNER UDP connection and P3D connection | Base class and GUI are never imported or constructed |
| `qtg_common_engine.run()` initial timer tick | Automatically calls `initialize_mode()` once P3D/BRUNNER connect | No QTG engine import or timer; no initialization action accepted |
| `backend.initializer.apply_manual_conditions()` | Pause, weather restoration, INITPOSITION, throttle/flaps and checks | Never imported/called |
| Automatic engine procedures | Reference restoration, airflow recovery, RPM pre-roll and BRUNNER preparation | Never imported/called |
| `SimConnectClient.connect_and_run()` placement branches | Optional ground/airborne placement, daytime and pause via placement helpers | Both placement flags explicitly false; both callbacks explicitly None |
| Native `ConnectSim()` | Opens transport; registers definitions and maps event IDs | Reused unchanged; registrations/maps do not write variables or transmit events |
| Native OPEN callback | Starts existing telemetry/engine-support/extended read streams | Reused unchanged on the same connection |
| `simulation_running()` | Reads/subscribes to the native Pause event | Read-only query allowed; unknown is reported UNVERIFIED; never sends PAUSE_ON/OFF |
| Normal test shutdown | May pause simulator/release BRUNNER | Never used; closes only this owned transport |

Before constructing the client, this mode verifies the installed DLL's SHA-256:

`d6ffc5f25d3931558572f81c6184678f2d48cca6e7e605a2e4cf160de238fc7c`

It rejects a different hash or a client-selected path other than
`interfaces/brunner_base/native/bin/p3d_bridge_engine_init_v1517.dll`. No binary is
rebuilt or replaced. The existing Python constructor only binds function signatures
and reads the bridge version; it does not connect or send aircraft controls.

After construction a native export gate exposes only connection lifecycle,
message pumping, telemetry getters, verification request/copy and read-only Pause
observation exports. Position, weather setters, engine/actuator setters, pause,
unpause and preparation-event exports are rejected **before native invocation**.
Any unexpected forbidden access fails the session. The legacy poller's initial
handshake retry is suppressed after its first ConnectSim result, preventing another
native open/handle; it simply waits within the session's connection deadline.

Offline source audits check the pinned native ConnectSim, dispatch, registration
helper, disconnect and Pause observation call paths for their expected SDK calls.
They contain no SimConnect_SetDataOnSimObject, TransmitClientEvent or WeatherSet
on the connection path. Defining a settable SimVar or mapping an event ID is not
issuing a write/event. Simulator physics or external weather software can still
change the aircraft independently; this tool does not freeze or stabilize it.

Only verification start, verification stop and diagnostic-session close commands
are accepted. Initialize, record, replay, PID, ordinary stop and ordinary shutdown
commands cannot be forwarded to a test handler. No BRUNNER object or control
authority is acquired. Both Universal Initialization switches must be disabled;
the new process explicitly sets its own values to zero and constructs no manager.

## Safe shutdown, cancellation and faults

To stop just monitoring and retain the owned idle connection:

```powershell
python -B tools/p3d_verification_diagnostic.py --context "$context" stop
```

To stop monitoring **and close the diagnostic-only connection**:

```powershell
python -B tools/p3d_verification_diagnostic.py --context "$context" close
```

`close` refuses a normal QTG worker context and waits for CLOSED/FAILED status.
Alternatively press Ctrl+C in the foreground diagnostic-only terminal. Ctrl+C
in a monitoring terminal requests monitor stop only; close the owner separately.
The foreground owner closes itself when its 300-second session deadline expires.
There are no spawned child workers or background GUI processes to leave behind.

Shutdown signals/joins the verification reader first, then signals the existing
client's stop event and joins its poller. The existing client closes its owned
native handle in `finally`; no pause/unpause or BRUNNER release is issued. Read
and connection joins are bounded at three seconds each. If native/SDK I/O does
not finish, status is FAILED with the shutdown-deadline reason; the foreground
process exits, rather than leaving a detached worker. Its threads are daemon
threads, and the OS releases its process-owned handles/mutex. Do not claim a
normal SDK disconnect was confirmed in that failure case.

Connection loss ends the session with FAILED and closes its owned transport.
There is no automatic reconnect, reload, reinitialization or fallback. An operator
may deliberately start a new supervised session. Its new PID and generation may
start from 1; never compare native generation numbers across different processes.
The last status and evidence remain on disk and are explicitly historical.

## Supervised P3D verification procedure

Only after separate approval for live checks:

1. Keep both Universal switches off; close QTG/other diagnostic transports. Start
   the command above against an operator-prepared stationary, paused P3D scenario.
   Wait for IDLE_CONNECTED and confirm the installed DLL path/hash in diagnostic
   output. Failure means stop, not opening a test panel as a workaround.
2. Run the 30-second normal snapshot command. Compare genuine ground/flap handle,
   IAS, MSL/indicated altitude, independent engine/prop RPM, throttle, OAT, ambient
   pressure and altimeter setting with cockpit/reference evidence. Check sources
   and units. Never substitute unavailable measurements.
3. If separately approved, let the operator change a flap detent while stationary
   to verify flap handle changes independently of ground contact. The diagnostic
   issues no cockpit/BRUNNER commands.
4. If safe and approved, the operator manually unpauses/pauses P3D. Observe fresh
   correlated responses during both states and separate simulator-time progression.
   Unchanged paused values can be legitimate; unknown pause state stays unverified.
5. After normal monitoring finishes, run read-only probes one at a time:

   ```powershell
   python -B tools/p3d_verification_diagnostic.py --context "$context" start --max-age-s 1 --duration-s 5 --probe cache
   python -B tools/p3d_verification_diagnostic.py --context "$context" start --max-age-s 1 --duration-s 5 --probe superseded
   ```

   The cache probe requests one seed sample then copies without requesting another;
   it must not label copies as new responses. The supersession probe accepts only
   the newest token. It cannot prove an obsolete server callback actually arrived
   late; that limitation remains explicit and is tested offline.
6. Connection-loss fault injection requires separate supervision/approval. Expect
   FAILED/DISCONNECTED without a new READY or reconnect. Close the old process,
   then explicitly restart a new diagnostic-only session if approved.
7. Stop and close using the commands above. Confirm the foreground owner exits
   and status says CLOSED. Review saved CSV/JSON evidence and mark each criterion
   MATCHED, NOT MATCHED or UNVERIFIED. Do not start a QTG test or enable a manager
   merely because connection or native receipt freshness succeeded.

Internal per-measurement acquisition time remains unavailable. Native callback
receipt, correlated responses and simulation-time progression are separate facts.
Live verification and normal legacy workflow behaviour were not tested in this
offline implementation stage.

## Offline validation — 11 November 2026

- New diagnostic-only tests: 27 passed, no failures/errors/skips.
- Full root discovery (including those 27 and the existing snapshot diagnostic
  tests): 345 passed, no failures/errors/skips; unittest reports `OK`.
- Legacy bridge discovery: 96 run, 94 passed, one failure and one error. The
  previously established RPM issues remain unchanged:
  `test_engine_acceleration_restores_transient_without_live_settling` expects a
  direct RPM setter call; `test_engine_transient_wrong_readback_blocks` has a
  missing `throttle` fixture field. No production RPM changes were made.
- Logs: application-root `stage5e_root_tests.log` and
  `interfaces/brunner_base/stage5e_legacy_tests.log`.
  Windows PowerShell wrapped unittest's stderr progress as `NativeCommandError`
  and returned shell status 1 for root discovery despite its final `OK` result;
  the log records all 345 test results with no unittest failures/errors.
- Installed/candidate DLL hashes still match the pinned hash above; independent
  original backup and dashboard DLL hashes were rechecked and remain unchanged.
- Tests use fake native exports and isolated files; the Windows ownership-mutex
  test does not open SimConnect. No live connection or DLL replacement occurred.
