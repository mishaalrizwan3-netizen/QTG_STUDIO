# Stage 5J: enforced plan approval and configuration integrity

## Stage 5N: shared Manual integration (default OFF)

The same `InitializationManager`, `Plan`, approval verifier, authoritative V1
reader, common Manual dispatcher and legacy cleanup owner now serve different
Manual test references. No second initialization engine is introduced. The demo
EXE/ZIP is an immutable earlier build and was not rebuilt for Stage 5N.

`backend.initialization_configuration.resolve_configuration(context)` resolves
the existing GUI/session conditions by test reference. Deceleration delegates
to its existing file-and-overrides resolver; other tests use the submitted GUI
conditions. `ResolvedConfiguration` freezes scalar conditions, units, provenance
and identity. The existing `configuration_snapshot(context)` API delegates to
this interface. Both the preparation dispatcher and plan compiler take copies
from the same bound snapshot; subsequent context/file changes revoke the attempt.

All enabled Manual plans now use `target_source: resolved_initial_conditions`
and must omit duplicate `value` / `value_source` targets, including Acceleration.
Older value-based Acceleration Manual plans require migration and new approval.
Replay target resolution, CSV interpretation, tolerances and reference binding
remain separate and unchanged. No missing Manual RPM or weather target is inferred.

Explicit generic activation is a process-local JSON mapping in
`QTG_UNIVERSAL_MANUAL_PLANS`: known test reference -> reviewed plan file path.
Unset or `{}` means no generic activation. It selects Manual/replay-control
initialization only; it does not select Automatic Recorded Replay or Climb PID.
The existing engine switches remain supported, off by default. A malformed
mapping, missing plan or failed approval blocks instead of falling back.

Non-engine plans require an explicitly reviewed `manual_policy`, covered by the
existing whole-document approval hash. Its fields are:

* `status`: `CONFIGURED` (does not approve the document by itself).
* `preparation`: `existing_manual`, selecting `_initialize_mode` and its existing
  reference-specific extensions. It introduces no alternate positioning, throttle,
  weather or actuator commands. Ground/airborne commands use the reviewed GUI
  `on_ground` setting; static tests do not receive engine RPM pre-roll.
* `mandatory_checks`: explicit nonempty list of registry parameters that must be
  checked. All other registry fields still require checks or approved exclusions.
  Engine aircraft/weather checks remain mandatory. Engine and propeller RPM are
  phase-specific: Manual paused READY requires them only when the approved plan
  explicitly checks them against configured targets. Otherwise each shaft needs
  an explicit reviewed exclusion, and RPM remains diagnostic telemetry. Replay
  retains both recording-start RPM checks and its existing tolerances.
* `physical_controls`: `read_only` or explicitly reviewed `not_required`. The latter
  omits manager physical-position reads; the existing worker connection/preparation
  requirements are unchanged. Engine plans retain physical readback.
* `stability`: explicit scope and engineering `source`. `paused_state` verifies
  only fresh matching paused conditions, not stabilized airborne flight.
  `running_window` additionally requires reviewed `limits` (per-channel range,
  in registry units), `duration_s`, `min_samples` (at least 3), and `max_gap_s`.
  Its channels must also have reviewed target checks. No numbers are defaulted.

The generic loader creates an attempt-local registry view only after document
approval and policy validation. It never changes the global 2 CONFIGURED / 28
UNCONFIGURED evidence inventory or automatically enables other references.
1.c.(2) remains unavailable in the current GUI. Catalog membership or generic
GUI defaults are not an engineering approval of applicability or targets.

For a reviewed running window, the existing adapter starts read-only sampling
during preparation and reuses `StabilityWindow`. Only authoritative responses
with running state true, advancing simulator time, fresh request identity and
in-band measurements accumulate evidence. Paused frames never establish the
window. Final verification must confirm pause shortly after the accepted running
window, within the reviewed maximum gap. Subsequent paused verification preserves
that evidence while the manager continues checking fresh in-band target values.
The metadata distinguishes `stability_scope`, `running_stability_verified` and
`stability_evidence`. Native callback receipt is not an internal sensor timestamp.

This observer does not unpause or stabilize the aircraft. If the existing
preparation never supplies the reviewed running window, readiness fails with
an explicit requirement for a validated preparation method. This is a remaining
airborne engineering/validation blocker, not a reason to widen limits or add PID.

The GUI assigns a configuration revision to each enabled Manual request. Target
or selected-reference edits invalidate the GUI binding, immediately disable
Record and send revision-correlated cancellation. Returning a field to its old
value does not revive the attempt. The worker ignores old revisions, processes
queued invalidations before queued Record requests, and requires the current
GUI revision on recording commands. Successful handoff consumes authorization
and stops initialization sampling; later GUI edits cannot interrupt a manoeuvre.
Disabled legacy workflows retain their existing routing. The initialization-only
lock remains latched and cannot be cleared by an environment edit.

### Shared Automatic integration (default OFF)

`QTG_UNIVERSAL_AUTOMATIC_PLANS` selects reviewed plans independently by test and
execution method, for example the **inactive template**:

```json
{"1.f.(2)":{"recorded_replay":"C:/reviewed-plans/deceleration-replay.json"},
 "1.c.(1)":{"pid":"C:/reviewed-plans/climb-pid.json"}}
```

Unset or `{}` enables nothing. The existing engine switches remain supported.
No real plan has been approved or activated by this implementation. Automatic
plans use `mode: recorded_replay` or `mode: pid`, the exact `reference_csv` path,
and `reference_identity` with `csv_sha256` and `sidecar_sha256`. New checks use
`target_source: manual_recording_t0`, with separately approved tolerances/sources;
they do not define replacement targets. Legacy engine plans with literal values
are accepted only if those values equal the validated selected reference.

`ReferenceSnapshot` freezes the full-precision first-sample sidecar and checks
every available mapped CSV measurement at its recorded precision. Required
values must exist and be finite; ground contact and flap handle use independent
recorded channels. Legacy flap aliases are never reinterpreted. Missing required
metadata, contradictions and changed files block. Recording-start sample index
and CSV time are logged separately from measured throttle-event alignment.
Leading invalid samples block rather than silently trimming the working Replay
timeline or associating a later sample with the first-sample sidecar. A validated
new metadata/alignment contract would be needed to support that case.

Non-engine Automatic plans also require an explicitly approved `automatic_policy`:
`status: CONFIGURED`, `preparation: existing_recorded_replay` or
`existing_climb_pid`, `stability_scope: paused_state`, an engineering `source`,
and explicit `mandatory_checks` including the common aircraft/weather checks.
Other registry entries require checks or individually approved exclusions. This
creates an attempt-local registry view; it does not approve or enable the catalog.
The existing Manager and authoritative V1 reader handle both modes.

Climb uses the existing Manual-reference selector when Universal PID is explicitly
selected. Its separate validation dataset is not an initialization reference.
The adapter applies the reference through the existing condition, weather and
state-restoration interfaces, then the preparation thread waits while P3D is
paused. After consecutive authoritative matches and another fresh authorization
frame, the shared Manager enters `EXECUTING`, stops initialization sampling and
releases the existing PID workflow. This is not QTG recording. Existing PID gains,
control calculations, 80 kt stabilization and response checks remain unchanged.
The internal recording transition must consume the same attempt authorization;
cancellation, lost connections, GUI reference changes or modified reference files
prevent it. Initialization-only mode never permits the PID handoff. Final t=0
checks cease before PID flight; they are not imposed during the manoeuvre.

GUI selection changes revoke either mode's pending attempt. Activation is latched
for an attempt: removing environment opt-in cannot expose legacy recording.
Default-off Manual, Replay and PID paths retain their existing procedures.

After engineering approval and supervised source validation, a **separate** frozen
candidate must include these backend modules, dynamic test/BRUNNER/PID resources,
configuration JSONs and the unchanged verified production DLL. Check frozen worker
startup, plan/reference loading, default-off behavior and operation with no Python
or internet on the target PC. Do not overwrite/rebuild the preserved demonstration
EXE or ZIP. No candidate build or live validation is authorized by this document.

### Supervised first test: Deceleration Manual, only after approval

1. Finish engineering review of the actual resolved GUI/file settings, all
   mandatory limits, exclusions, freshness/deadline budgets and paused-state
   scope. Retain separate recording-start and pre-throttle RPM phases. Create
   the complete existing approval record and content/configuration identities.
   No test fixture or draft example is a live approval.
2. Close prior workers/diagnostics. Start P3D/CLS2Sim with the proven profile.
   Use the source application; the preserved demo EXE predates this integration.
3. In a fresh PowerShell session, after explicit live-test authorization:

```powershell
Set-Location '<PROJECT_ROOT>'
$env:QTG_UNIVERSAL_ENGINE_ACCELERATION = '0'
$env:QTG_UNIVERSAL_ENGINE_DECELERATION = '0'
$env:QTG_DECELERATION_BRAKE_CAPTURE = '0'
$env:QTG_ACCELERATION_MANUAL_AIRFLOW_PREPARATION = '0'
$env:QTG_UNIVERSAL_INITIALIZATION_ONLY = '1'
$env:QTG_UNIVERSAL_MANUAL_PLANS = '{"1.f.(2)":"C:/reviewed-plans/deceleration-manual.json"}'
python main.py
```

4. Select Deceleration and inspect the exact approved settings before Manual.
   Clicking Manual can apply aircraft conditions. Do not move the throttle for
   a manoeuvre. Inspect fresh native generation/request/sequence, paused state,
   target/actual rows, approved plan/configuration identities and one attempt ID.
   READY must retain `can_record: false`, `recording: false`, and zero samples
   under the initialization-only lock. No automatic recording is added.
5. After capturing evidence, Stop and verify actual P3D pause/control release.
   Validate cancellation, GUI-edit invalidation, and disconnect/reconnect rejection
   in separately supervised negative trials. Each retry requires a new attempt.
   Never bypass an unresolved criterion or continue after unexpected behavior.
6. Roll back by Stop, verifying pause/release, closing workers/application, then
   removing `QTG_UNIVERSAL_MANUAL_PLANS` and `QTG_UNIVERSAL_INITIALIZATION_ONLY`
   from the launching shell and keeping both engine flags at 0. Relaunch normally.
   No DLL or source replacement is required. Removing a flag cannot unlock an
   existing initialization-only worker; close it first.

Weight, CG, trim, fuel, local wind and other V1 gaps remain unresolved wherever
mandatory. No additional measurement or stability tolerance is approved here.
Proceed to Acceleration Manual and Climb Manual only after their own review and
separate supervised approval. Climb Automatic PID and engine Replay are not
activated through the generic Manual mapping.

Stage 5L adds the Deceleration Manual target-source contract below. Existing
approval, identity, lifecycle and hardware protections remain in force.

Both engine flags remain disabled by default. The Stage 5I candidate remains an
inactive historical review artifact. No engineering criterion is approved here.
This contract supersedes the Stage 5I descriptions of unenforced approval and
pre-reload plan construction; it does not approve its proposed tolerances.

## Approval contract

An executable engine plan now requires an `approval` object:

```json
{
  "status": "APPROVED",
  "reviewer": "named authorized engineering reviewer",
  "reviewed_at": "ISO-8601 timestamp including timezone",
  "record": "review record identifying the approved targets, limits and exclusions",
  "approved_checks": ["every checks key, each exactly once"],
  "approved_exclusions": ["every exclusions key, each exactly once"],
  "payload_sha256": "SHA-256 of the canonical complete plan excluding approval"
}
```

This example is not executable. If a top-level `approval_status` is present it
must also be `APPROVED`. `review_source` remains mandatory. Each target and
tolerance must be finite, numeric, explicitly sourced and mode-specific. Every
registry requirement needs a check or explicitly approved exclusion. Engine
and propeller RPM checks are mandatory for the enabled engine Manual path as
well as Replay; unresolved Manual criteria cannot silently become exclusions.
No target or tolerance is automatically generated.

`backend.initialization_plan_safety.document_identity(document)` computes the
payload hash using sorted JSON keys, compact separators and no nonfinite numbers,
UTF-8 encoded. The hash includes configuration identities, all criteria, exclusions,
review source, deadlines and all other non-approval fields. Approval lists must
exactly cover checks and exclusions. Hash/list changes require another review.
Do not use the synthetic test approval helpers to approve a live plan.

The loader validates this record before constructing a plan. The compiled plan
also carries a frozen `PlanApproval` bound to `plan_identity(plan)`, including
resolved targets, units, channels, tolerances, scope, exclusions and input binding.
The core independently rejects missing approval or a changed compiled plan.
The manager copies checks/exclusions into immutable mappings before execution.

This is an explicit review attestation and content-integrity mechanism, **not a
digital signature or authentication service**. Reviewer authority and custody of
approved files/review records remain operator responsibilities. Merely writing
APPROVED into a draft without resolving and reviewing its inputs is not approval.

## Final Manual configuration binding

For 1.f.(2) Manual, an approved plan must include:

```json
"configuration_identity": {
  "source_sha256": "SHA-256 of exact initialization.json bytes",
  "resolved_sha256": "canonical identity of validated final conditions",
  "overrides_sha256": "canonical identity of explicit operator overrides, or {}",
  "weather_overrides_sha256": "canonical identity of weather override metadata, or {}"
}
```

`engine_initialization_adapter.configuration_snapshot(context)` obtains this
identity read-only. It uses the existing Deceleration resolver and rejects a
configuration-file edit during resolution. The final file and overrides are
resolved **before** loading/validating the plan. Deceleration Manual checks now
reference that resolved snapshot; no separate target values are accepted in the
criteria file. Approval binds the actual resolved settings through configuration
identity, rather than a second copy of editable target numbers.

```json
"checks": {
  "alt_ft": {
    "target_source": "resolved_initial_conditions",
    "tolerance": 25,
    "tolerance_source": "existing backend/initializer.py:apply_manual_conditions; review record must confirm use"
  }
}
```

This is a partial schema example, not an approved plan. Every Deceleration Manual
check must use `target_source: resolved_initial_conditions` and omit `value` and
`value_source`. Old value-based criteria require migration and a new review; they
are not automatically approved. Tolerances/exclusions/deadlines remain engineering
criteria; a GUI target is not itself a tolerance or approval.

The GUI flow is `frontend/main.py:_ensure_test_settings` / `_build_settings_panel`
(Tk variables in `_test_settings`), `_apply_test_settings`,
`deceleration_config.display_overrides`, then `qtg_common_engine.launch` or the
worker `initialize` command. Edited GUI fields become `initial_condition_overrides`.
The worker resolves defaults from `test_suites/engine_deceleration/initialization.json`
plus those overrides. GUI edits are held in application memory; launch also saves
resolved conditions and overrides in session context JSON. The settings panel
does not write them back into initialization.json.

The current panel exposes altitude, IAS, flap handle, position, heading, pitch,
bank, throttle, OAT and ground/flight condition. It does not expose paused engine
or propeller RPM, ambient wind, KOHLSMAN or a readiness-tolerance editor. The
enabled snapshot can retain explicitly supplied optional registry conditions
from the **same** initialization.json or overrides, with no defaults for them.
Optional conditions are verification targets, not new aircraft commands. An
explicit Manual `engine_rpm` / `prop_rpm` condition refers to paused initialization
and still needs its own reviewed tolerance. Historical recording-start/pre-event
values and proposals are never fallback targets.

Unsupported `weather_overrides` metadata blocks the enabled path instead of
being ignored: the existing Manual command routine has no such consumer. GUI
OAT changes already travel through ordinary initial conditions. Ambient wind and
setting checks need explicit configured expectations; existing weather-command
behavior is unchanged. File contents, targets, overrides, reference or criteria
changes invalidate the previous approval/attempt.

The enabled Manual dispatcher uses a copy of these bound conditions instead of
reloading different command values later. The legacy flag-off branch continues
to call its original resolver and preparation functions. No positioning, weather,
throttle, RPM, flap or BRUNNER commands are added or changed.

The adapter checks plan bytes, selected test/mode/reference, source-file identity,
resolved conditions and operator overrides before preparation, during verification
and at recording handoff. Any change or unavailable input invalidates the attempt;
even a whitespace-only edit requires a fresh review/attempt. During preparation,
any command values already handed to the legacy routine remain the reviewed copy;
the reader checks integrity before allowing READY. Stop and review before retrying.

Recorded Replay retains its existing measured recording-start contract and 60 RPM
Deceleration check. Its approved plan additionally binds CSV and sidecar hashes
in `reference_identity` (`csv_sha256`, `sidecar_sha256`), and each reviewed target
must equal the selected measured target. Reference edits cannot silently inherit
approval. This does not resolve historical flap provenance or approve Replay.

Metadata exposes `approved_plan_identity`, `reviewed_input_identity` and
`configuration_identity` alongside the existing attempt and native evidence.
Freshness, final pause, cancellation and handoff requirements remain enforced.

## Manual RPM scope and BLOCKED results

The registry's 60 RPM Deceleration value remains unchanged as Replay evidence.
The core applies its preservation constraint only to `recorded_replay`, not to
Manual. An explicitly approved Manual band can differ; a missing Manual target
or band is BLOCKED. No target near 989 or 2700 RPM is supplied by this change.
Historical recording-start approximately 989 RPM and pre-throttle approximately
2680 RPM remain different observations. No preparation sequence or RPM command
changes are introduced.

`State.BLOCKED` is terminal for approval, schema and integrity rejection. It cannot
be revived by READY, cancellation or failure callbacks. Unknown/missing targets,
unresolved limits, unapproved exclusions and changed identities cannot dispatch
preparation or authorize recording. Runtime telemetry/preparation failures remain
FAILED; operator cancellation remains CANCELLED. A changed input detected during
handoff blocks recording even if the runtime reports that handoff as FAILED.

Pre-dispatch rejection requests no hardware cleanup because no preparation was
acquired. If an attempt already started, the existing legacy cleanup owner handles
pause/release, and errors remain visible. The adapter adds no cleanup commands.
The initialization-only lock still prevents recording/replay and PID, including
after a mocked successful READY. There is no fallback to legacy readiness.

## Readiness diagnostics

The current worker status JSON contains
`initialization_attempt.readiness_diagnostics` and `readiness_diagnostic_text`.
The console prints the same parameter table when result categories change.
Rows show parameter, configured target, actual P3D value, unit, tolerance,
PASS / FAIL / UNAVAILABLE, native receipt age and freshness status. Missing values
display as Not available. Rows update while awaiting a new frame, so an expired
last snapshot becomes UNAVAILABLE / STALE_NATIVE_RECEIPT.

PASS is a per-channel comparison, not recording authorization. Approval, current
identity, consecutive samples, pause, hardware and lifecycle checks still govern
READY. Blocked configuration/criteria show no passing measurements. Native receipt
freshness is not internal sensor acquisition timing or physics progression.

## Ambient pressure and unavailable configuration

The existing V1 channel `AMBIENT PRESSURE` / millibars already supplies
`ambient_pressure_mb` through authoritative feedback, separately from
`KOHLSMAN SETTING MB` / `barometer`. No ABI or DLL change is needed to read it.
The registry still does not expose ambient pressure as a plan requirement.
No ambient-pressure target, tolerance or mandatory readiness gate is added here.
A later approved registry extension could consume the existing measurement.
Until then, do not claim ambient pressure was checked through altimeter setting.

Weight, fuel, CG, trim, density and other unsupported authoritative measurements
have no new validated source. They remain unresolved unless the permitted plan
schema has an explicitly approved exclusion. The engine loader rejects checks
outside the existing V1 channel inventory before preparation; an approval record
cannot promote a legacy-only value to authoritative telemetry. Fuel and other non-schema deferrals
must remain visible in the engineering review record; JSON extra fields do not
create executable checks. Historical suspect CG and flap aliases are not targets.

## Before any supervised initialization-only trial

Resolve the Stage 5I engineering decisions, including paused Manual engine and
propeller RPM criteria, weather/heading/attitude/IAS/ground-speed criteria, loading
and configuration exclusions, ambient-pressure scope and lifetime/freshness budgets.
Obtain the complete approval record and compute identities from the final files
and operator settings. The existing Stage 5I candidate is rejected as a draft.

Only after separate operator authorization use the existing supervised checklist
with Acceleration=0, Deceleration=1, brake capture=0,
`QTG_UNIVERSAL_INITIALIZATION_ONLY=1`, and a separate approved plan path. Preserve
the reviewed settings. Opening the Manual worker can initialize the aircraft;
this is not a passive telemetry experiment. Capture the attempt IDs, approved
input identities, native measurements, READY/BLOCKED/FAILED status and zero
recording samples. Stop and close the worker; do not perform throttle reduction.

Rollback is unchanged: Stop, inspect actual pause/release, close workers/application,
set both engine switches and brake capture to 0, remove the plan path and
initialization-only environment variable, then relaunch normally. No DLL, source
or historical reference replacement is required. No live test is authorized here.
