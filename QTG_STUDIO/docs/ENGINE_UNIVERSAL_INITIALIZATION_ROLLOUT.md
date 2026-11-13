# Engine Universal Initialization: controlled rollout

Both engine opt-ins are OFF by default. No plan in this change is approved for
live recording. The other 28 tests are not selected by these switches. Do not
change the existing Acceleration Manual airflow option to imply nonzero-wind validation.

## Behavior and existing routines

The existing manager drives VALIDATING / PREPARING / VERIFYING / READY /
RECORDING / COMPLETE, with FAILED and CANCELLED terminal outcomes. Attempt
metadata identifies the initial entry as INITIALIZING. There is
one attempt ID throughout, including generation-guarded preparation callbacks.

The engine adapter invokes `qtg_common_engine._initialize_mode` exactly once.
This retains `start_manual_test` / `apply_manual_conditions`, Acceleration
`start_auto_test_only`, and Deceleration's shared `run_initialization` adapter.
Weather, positioning, RPM preparation and BRUNNER ownership stay in those routines.

Only after their READY callback does a verification reader request fresh packets
through the existing P3D client and read physical BRUNNER axes through the existing
serialized interface. It sends no actuator or simulator state commands. The
manager requires the reviewed plan's checks to pass on at least three new frames,
and another new frame to authorize recording. The reader must stop before handoff.
Initialization target monitoring then ends. Natural IAS, altitude and attitude
changes during the recorded manoeuvre remain permitted; existing safety monitoring
and measured throttle-onset analysis remain unchanged.

Cleanup cancels only the adapter reader. The existing runtime owns physical pause
and release. Failed cleanup prevents the next attempt or recording handoff.

## Review prerequisites

- Stage 5B added authoritative V1 snapshots on the existing bridge connection.
  The operator reports supervised validation of correlated native responses,
  genuine ground contact/flap index, pause state and simulator time. Python
  cache-copy timestamps are not used to authorize READY. Internal simulator
  sensor acquisition time is unavailable; native callback receipt is the evidence.
- Acknowledged BRUNNER physical reads are separate from simulator controls.
  Automatic additionally retains the existing 0.005 normalized alignment limit.
  The adapter never engages motors to correct a mismatch.
- Registry CONFIGURED means baseline evidence only. Many additional tolerances,
  including weather readiness tolerances, need explicit review. Existing weather
  diagnostic thresholds are not automatically promoted to readiness criteria.

The remaining blocker is a reviewed test/mode plan, not the former cache timing
gap. No approved live plan is supplied here. Mocked READY is not live validation.
Weight, CG, fuel, trim and body/world velocity are not authoritative fields in
the 23-channel snapshot. Require approved exclusions or a separately reviewed
telemetry extension; never promote legacy cache values. Pressure setting
(`barometer`, KOHLSMAN SETTING MB) is distinct from measured ambient pressure.
The latter is available in V1 but not currently a registry plan check. Do not
claim ambient-pressure readiness from an altimeter-setting match.

Final engine verification now requires `running is False` in metadata correlated
with every delivered native frame. Unchanged paused values may pass on at least
three fresh responses; sequence progression alone never proves physics progression.
Unpause/unknown pause state revokes readiness. Reader failure during handoff blocks
recording. No additional simulator commands implement these safeguards.

## Reviewed plan file contract

Each JSON plan is specific to `test_reference` and `mode` (`manual` or
`recorded_replay`). It requires a nonempty `review_source`, explicitly reviewed
positive finite `timeout_s` and `max_age_s`, `stable_samples` (at least 3), `checks`, and `exclusions`.
Every registry field needs a check or a documented reviewed exclusion.
The overall lifecycle deadline must cover the selected existing procedure; it
does not replace or extend any of that procedure's internal deadlines, including
the optional Manual airflow recovery's 60-second bound.

Each check supplies `tolerance` and `tolerance_source`. Units and telemetry names
come from the existing requirements registry. Existing registry tolerances must
be preserved; Acceleration IAS cannot exceed 2 kt. No generic tolerances are added.

Mandatory checks: `on_ground`, `alt_ft`, `ias_kt`, `lat`, `lon`, `heading_deg`,
`pitch_deg`, `bank_deg`, `throttle`, `flaps`, `temperature_c`, `wind_speed`,
`wind_direction`, `barometer`; Replay also requires `engine_rpm` and `prop_rpm`.

Manual targets use selected operator conditions where present. Additional targets
require explicit `value` and `value_source` entries from reviewed evidence, including weather. Replay
targets come exclusively from the selected CSV's matching `_init_conditions.json`
Manual snapshot; its first CSV IAS/altitude/throttle must agree to the CSV's
recorded precision. Its plan must bind the exact absolute `reference_csv` path.
Recording-start targets are never replaced by pre-event values.

The existing Acceleration 350 RPM normal / 400 RPM fallback and Deceleration 60 RPM
checks remain in their existing routines. The manager preserves the registry's
existing final engine-RPM limits; propeller-RPM criteria require their own reviewed
source. The adapter does not reinterpret a pre-roll tolerance as a different
final-readiness tolerance or change either routine's fallback behavior.

## Supervised initialization-only sequence and reversible switches

First test with both variables unset: legacy initialization and replay must remain
unchanged. Close QTG Studio and its worker between these experiments. Start from
the source application or a newly built executable containing this change.

After engineering approval of the Manual plan and separate authorization to run
the supervised experiment, begin with Deceleration Manual only. This is not a
read-only transport experiment: the unchanged legacy preparation intentionally
applies aircraft conditions and prepares Manual BRUNNER authority. It does not
perform the throttle-reduction manoeuvre. Obtain operator clearance first.

```powershell
Set-Location '<PROJECT_ROOT>'
$env:QTG_UNIVERSAL_ENGINE_ACCELERATION = '0'
$env:QTG_UNIVERSAL_ENGINE_DECELERATION = '1'
$env:QTG_DECELERATION_BRAKE_CAPTURE = '0'
$env:QTG_UNIVERSAL_INITIALIZATION_ONLY = '1'
$env:QTG_UNIVERSAL_ENGINE_DECELERATION_PLAN = 'C:\reviewed-plans\deceleration-manual.json'
python main.py
```

1. Close all earlier QTG workers and diagnostics; leave the validated production
   DLL installed. Start the source application using the commands above; choose
   **1.f.(2) Engine Deceleration, Manual**. Review the ground settings before
   opening its worker panel (opening can start initialization).
2. Observe the existing preparation, followed by Universal verification. Never
   click Record or select Automatic. The initialization-only interlock additionally
   disables the application Record action and rejects worker Record/PID commands.
   Removing its environment variable cannot unlock this worker; close it first.
3. Inspect the current session status JSON in `D:\QTG_Test_Results\sessions`.
   `initialization_attempt` must show `selected_system: Universal Initialization Manager`,
   a new `attempt_id`, `initialization_only: true`, the reviewed source,
   `preparation_dispatch_started/returned`, native generation/request/sequence,
   `running: false`, observed and verified measurements, and status READY.
   `initialization_source` separately identifies the existing Manual entry point.
   Verify `recording: false`, `samples: 0`, `can_record: false`. Save a copy of
   this status outside the session before stopping; the status file is updated.
4. A returned dispatcher or successfully issued command is not independent P3D
   command acknowledgement. The metadata labels that limitation explicitly.
   Genuine measured convergence, units, sources and fresh native responses are
   the readiness evidence. Capture the operator console as additional evidence.
5. On failure inspect target/actual/tolerance and reason. Do not relax a limit or
   substitute unavailable values. No legacy fallback is allowed. Stop after any
   unexpected aircraft movement, hardware activity or connection behavior.
6. Press Stop while preparing, then repeat in a fresh worker for cancellation
   validation. Verify CANCELLED, no recording, reader cancellation and actual P3D
   pause/hardware release. Cleanup returning does not independently prove physical
   release. Close the worker; do not use forced process termination as a normal stop.
7. Disconnect/reconnect is a separate supervised negative test: the old attempt
   must fail, never resume READY. New initialization needs a new attempt and reader.
   Do not perform it automatically or during a manoeuvre.

The live trial is **blocked pending plan approval**. In particular, approve
weather/IAS/heading/attitude/position tolerances, lifetime/freshness budgets and
all exclusions. Existing Deceleration altitude 25 ft, throttle 1 percentage point,
flaps 0.1 index and Replay engine RPM 60 remain unchanged. These are existing
implementation limits, not new engineering approval. Ground IAS's legacy 35 kt
special case is not approval of a stationary-ground-speed criterion.

Manual targets established by `test_suites/engine_deceleration/initialization.json`:
ground, 755 ft MSL, IAS 0 kt, latitude 32.23366667, longitude 74.1235,
true heading 48.4 degrees, pitch/bank 0 degrees, flap handle 0, throttle 100%,
OAT 15 C. Operator overrides take runtime precedence. Wind and barometer require
explicit reviewed targets consistent with the existing weather procedure; no
new weather commands are added to make a plan pass. Manual engine/propeller RPM,
ground speed and unavailable configuration fields require reviewed checks or
justified exclusions. Do not force a 2700 RPM recording-start target.

Evidence reviewed: Manual 2026-11-10 13:52:11 versus Automatic 2026-11-11 10:50:13
(the latter metadata selects that Manual and `manager_enabled: false`). First
CSV engine/prop RPM: 989/989 versus 987/987; last sample before measured throttle
onset: 2680/2680 versus 2681/2681. Ground speed: 0.01 versus 0.01 kt initially,
3.14 versus 3.11 kt immediately before onset. IAS: 0.01 versus 0.68 initially,
3.47 versus 3.75 kt pre-event. Onsets 5.472411 versus 5.403673 s. OAT 15 C in
both CSVs. These rounded recorded values support preserving the working procedure;
they neither establish brake state nor approve readiness tolerances or prove
which individual change caused the improved ground-speed match.

Replay remains a later stage. Historical `flaps` or a field name alone is not
proof of genuine FLAPS HANDLE INDEX provenance. Its plan needs an independently
reviewed reference provenance solution and immutable CSV/sidecar binding before
live authorization; the current loader binds paths and checks three starting
columns, not file hashes. Do not activate Replay with historical aliases.

Only after a separately reviewed successful initialization-only trial, complete
Deceleration Manual/Replay validation and legacy comparison may the following
Acceleration stage be considered. It is not authorized by this checklist:

```powershell
Remove-Item Env:QTG_UNIVERSAL_ENGINE_DECELERATION -ErrorAction SilentlyContinue
Remove-Item Env:QTG_UNIVERSAL_INITIALIZATION_ONLY -ErrorAction SilentlyContinue
$env:QTG_UNIVERSAL_ENGINE_ACCELERATION = '1'
$env:QTG_UNIVERSAL_ENGINE_ACCELERATION_PLAN = 'C:\reviewed-plans\acceleration-manual-zero-wind.json'
python main.py
```

The separate `QTG_ACCELERATION_MANUAL_AIRFLOW_PREPARATION=1` opt-in may be used for
that supervised Manual session; it does not activate any PID. For Replay, restart
with a separately reviewed Replay plan bound to the zero-wind Manual recording.
Do not generalize success to nonzero wind. Do not enforce 80 kt / 2000 ft during
the manoeuvre: those are recording-start preparation conditions in this procedure.

To revert, close the app and worker, remove both engine switches and plan paths,
and relaunch normally. Remove the optional airflow switch too if it was set:

```powershell
Remove-Item Env:QTG_UNIVERSAL_ENGINE_DECELERATION, Env:QTG_UNIVERSAL_ENGINE_DECELERATION_PLAN, Env:QTG_UNIVERSAL_ENGINE_ACCELERATION, Env:QTG_UNIVERSAL_ENGINE_ACCELERATION_PLAN, Env:QTG_UNIVERSAL_INITIALIZATION_ONLY, Env:QTG_ACCELERATION_MANUAL_AIRFLOW_PREPARATION -ErrorAction SilentlyContinue
$env:QTG_DECELERATION_BRAKE_CAPTURE = '0'
python main.py
```

These are process-local variables; do not use `setx`. No simulator tests have
been performed by this implementation. Do not perform unattended hardware tests.
