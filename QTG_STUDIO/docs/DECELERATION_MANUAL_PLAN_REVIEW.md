# Stage 5I: Deceleration Manual plan review

Status: **DRAFT / NOT APPROVED / DO NOT ACTIVATE**. Scope is 1.f.(2) Manual,
initialization-only. No production code, flag default, procedure or DLL is changed.
Candidate: [deceleration_manual_plan_candidate.json](deceleration_manual_plan_candidate.json).
It deliberately contains unresolved null values and an empty `review_source`.
It is a review template, not an executable plan. `approval_status` is documentary;
the existing loader does not enforce that extra field. Do not set a plan variable
to this artifact. Create a separate approved copy only after the decisions below.

## Evidence and phase contract

- **C**: `test_suites/engine_deceleration/initialization.json`, current Manual
  configuration. `backend/deceleration_config.py:resolve_initial_conditions`
  reloads it and applies explicit operator overrides. These are command targets,
  not proof that measurements match. No overrides are assumed by this candidate.
- **L**: `backend/initializer.py:apply_manual_conditions`, existing paused Manual
  preparation: pause confirmation, weather temperature, ground INITPOSITION,
  altitude readback, throttle and flap setting, condition readback. No new RPM,
  wind, pressure, trim or flight-control commands are proposed.
- **R**: `D:/QTG_Test_Results/1_f_2_Engine_deceleration/` Manual
  `1_f_2_Engine_deceleration_manual_2026-11-10_13-52-11.csv` and matching
  `_init_conditions.json`. SHA-256 hashes are preserved in the candidate. This
  is the Manual selected by the successful 11 November session already reviewed.
  Historical CSV/sidecar values are recorded observations, not authoritative
  V1 acquisitions or approved acceptance criteria.
- **V**: `interfaces/brunner_base/src/verification_snapshot.py:CHANNELS` and
  `backend/telemetry_verification.py:_authoritative_feedback`: correlated V1
  native responses, same worker connection, genuine sources listed below.
  User-reported live validation supports this API, not every historical channel.
- **D**: `backend/weather_verification.py:CHANNELS`: diagnostic comparison limits
  only. Promoting them to readiness requires engineering approval.

| Phase | Engine/prop RPM | Throttle | Ground speed | Interpretation |
|---|---:|---:|---:|---|
| Paused Universal READY | No approved fixed Manual target yet | C: 100% | Proposed approximately stationary, limit unresolved | Must be independently measured before recording; do not substitute a later state |
| Historical Manual recording start | CSV 989/989; snapshot engine 989.1339446 | Snapshot 99.9938965% | Snapshot 0.0084356 kt | First CSV sample is at 0.0896963 s; not the exact timer origin or necessarily the earlier READY instant |
| Immediately before measured throttle reduction | CSV 2680/2680 | 100% | 3.14 kt | Last pre-onset sample at 5.4720133 s; detected onset 5.4724110 s |

The aircraft spools up and can move during the existing full-throttle baseline.
Do not command 2700 RPM at recording start, require pre-event ground speed while
paused, alter the five-second baseline, or monitor READY targets during the
manoeuvre. This stage does not start recording. Historical 989 RPM supports a
recording-start observation, not an approved invariant across new Manual starts.

## Parameter inventory

`B` = commanded by existing Manual preparation and measured; `M` = measured only
in this path. `V1` = authoritative native read available. `Legacy` = cache source
exists but no validated alternative with current request-correlated provenance
was established. All proposals and exclusions below require approval. Existing
implementation limits are preserved, but no separate formal engineering approval
record was found in the inspected evidence. Units are request/plan units, not
guessed from a display label.

| Parameter / SimConnect source | READY target / unit | Criterion and provenance | Command/read | Verification / approval |
|---|---|---|---|---|
| Ground / `SIM ON GROUND` | C: true / bool | Proposed exact true; V rejects values outside 0/1 | B | V1; exact check approval needed |
| Altitude / `PLANE ALTITUDE` | C: 755 ft MSL | Existing ±25 ft, L and `_wait_for_initial_altitude`; R measured 749.488844 ft | B | V1; retain limit, confirm review |
| IAS / `AIRSPEED INDICATED` | C: 0 kt | No approved Manual ground IAS tolerance found | B | V1; approval blocks READY; legacy 35 kt ground Replay special case is not an approved Manual limit |
| Ground speed / `GROUND VELOCITY` | Proposed 0 kt; R 0.0084356 kt | No approved limit; single stationary observation cannot establish one | M | V1; target/limit or explicit trial exclusion needs approval |
| Latitude / `PLANE LATITUDE` | C: 32.23366667 deg | Existing ±0.001 deg readback, L; not a surveyed-location qualification limit | B | V1; proposed unchanged trial criterion needs review |
| Longitude / `PLANE LONGITUDE` | C: 74.1235 deg | Existing ±0.001 deg readback, L | B | V1; proposed unchanged trial criterion needs review |
| Heading / `PLANE HEADING DEGREES TRUE` | C: 48.4 deg true | Unresolved circular tolerance; R 48.40000298 deg | B | V1; approval needed |
| Pitch / `PLANE PITCH DEGREES` | C: 0 deg | Unresolved; R 0.11342390 deg; account for ground/gear settling and native sign | B | V1; approval needed; no attitude controller |
| Bank / `PLANE BANK DEGREES` | C: 0 deg | Unresolved; R 0.00701980 deg; do not require mathematical zero from one frame | B | V1; approval needed |
| Flaps / `FLAPS HANDLE INDEX` | C: 0 / handle index | Existing registry ±0.1 index; only discrete index 0 passes. L historically skips UP readback | B | V1 genuine; approval of added mandatory UP verification needed; reject R legacy alias 1 |
| Throttle / `GENERAL ENG THROTTLE LEVER POSITION:1` | C: 100% | Existing ±1 percentage point, L; this is lever position, not shaft power | B | V1; retain limit, confirm review |
| Engine / `GENERAL ENG RPM:1` | Unresolved paused target; R 989.1339446 rpm at recording start | No approved Manual tolerance. Registry forces 60 if checked, despite Replay-only provenance | M | V1; Manual target/criterion approval or separate schema scope correction needed |
| Propeller / `PROP RPM:1` | Unresolved paused target; R approximately 989 rpm | Independent measurement; no approved Manual tolerance | M | V1; approval needed; do not substitute engine RPM |
| Mixture / `GENERAL ENG MIXTURE LEVER POSITION:1` | Proposed R: 100% | No approved lever tolerance | M | V1; target and criterion approval needed; no new lever command |
| Propeller lever / `GENERAL ENG PROPELLER LEVER POSITION:1` | Proposed R: 100% | No approved lever tolerance | M | V1; target and criterion approval needed |
| Manifold pressure / `ENG MANIFOLD PRESSURE:1` | No fixed READY target; R 6.61162015 psi | Proposed exclusion from fixed-target trial checks: transient engine state, no justified acceptance band | M | V1 available/logged; exclusion needs approval, not a telemetry limitation |
| OAT / `AMBIENT TEMPERATURE` | C: 15 C; R 15.01291561 C | Proposed ±1 C from D; diagnostic limit, not approved readiness | B | V1; approval needed; verify aircraft OAT, not commanded METAR alone |
| Ambient wind / `AMBIENT WIND VELOCITY` | Proposed R: 5.984132952 kt | Proposed ±1 kt from D; no configured Manual wind target | M | V1; target/criterion approval needed |
| Wind direction / `AMBIENT WIND DIRECTION` | Proposed R: 325 deg, native convention | Proposed ±5 deg circular difference from D | M | V1; approval needed; do not confuse ambient wind with aircraft-relative components |
| Altimeter setting / `KOHLSMAN SETTING MB` | Proposed R: 1013 hPa | Proposed ±1 hPa from D | M | V1; approval needed; not atmospheric pressure |
| Ambient pressure / `AMBIENT PRESSURE` | R: 986.0013428 mbar; READY target not approved | D has ±1 mbar diagnostic only. V1 reads it but registry cannot check it | M | V1; explicit trial deferral or reviewed schema extension needed |
| Indicated altitude / `INDICATED ALTITUDE` | R: 749.3333740 ft indicated; no READY target | Not MSL or pressure altitude; no ground altimeter-offset criterion established | M | V1; proposed diagnostic-only trial scope needs review |
| Weight / `TOTAL WEIGHT` | R: 834.1312419 kg; no approved target | No approved Manual criterion; proposed trial-only exclusion | M | Legacy; no validated authoritative alternative established; approve loading documentation/exclusion or defer trial |
| Fuel mass / `FUEL TOTAL QUANTITY WEIGHT` | R: `total_fuel_quantity` 95.00076199 kg per native definition; not an approved target | No approved criterion; not a registry field; proposed explicit trial deferral | M | Legacy; do not infer quantity volume or trust historical export label alone |
| CG / `CG PERCENT` | Unresolved; R −89.9840057 must not be used as target | Legacy meaning/decoding is suspect; proposed trial-only exclusion | M | Legacy; requires independent configuration evidence and approved exclusion or source validation |
| Elevator trim / `ELEVATOR TRIM POSITION` | R: 0 rad; no approved READY target | Proposed trial-only exclusion; no tolerance established | M | Legacy; not V1; no validated alternative established |
| Aileron/rudder trim / `AILERON TRIM PCT`, `RUDDER TRIM PCT` | R: 0/0%; native request `percent over 100`, Python explicitly multiplies by 100 | Conversion is documented in `p3d_simconnect_v15.py:get_telemetry`; live decoding/freshness still needs validation before a check. Proposed exclusions | M | Legacy; approval needed |
| Density / `AMBIENT DENSITY`, aircraft-relative wind / legacy wind components | R density 0.00231352818 slug/ft^3; no approved READY target for either | No V1 readiness checks; proposed diagnostic-only ground trial scope | M | Legacy; do not invent zero wind/density or require airborne recovery on ground |
| Physical BRUNNER pitch/roll/rudder / direct `read_axis` | No approved Manual physical target; R normalized 0.14413142/−0.03286386/0 | Existing compulsory finite replies on all three axes under strict serialized I/O; ownership conflicts fail | M here; existing preparation retains authority | Separate acknowledged hardware reads, not V1 or simulator surfaces; no native acquisition timestamp or approved Manual positional band |

The six registry exclusions in the candidate (manifold pressure, weight, CG and
three trims) are **proposed**, not approved. Fuel, ambient pressure, indicated
altitude and airflow/density deferrals are separately listed because they are
not enforceable plan fields. The candidate conservatively retains unresolved
engine/prop RPM, ground speed and lever checks instead of silently excluding them.
Engineering may approve a narrower trial scope, but this document does not.

## READY and lifecycle conditions that are not scalar targets

Before READY: correct 1.f.(2)/Manual plan and reviewed source; every registry
field checked or explicitly excluded; valid units/channels/finite targets and
limits; no active recording/PID/external controller; legacy preparation completion
for the current attempt; valid direct BRUNNER replies; at least **three** new
matching native responses; current stable connection generation; increasing
request ID/sequence/receipt time; finite native simulator time, no time
reversal; finite mandatory measurements; native freshness within an approved
`max_age_s`; lifetime within an approved `timeout_s`; final correlated
`running=false`; no cancellation, producer error or cleanup failure.

`stable_samples=3` is existing engine enforcement. The reader waits 0.25 s between
collections; this is not an approved physics-stabilization duration. Paused fresh
responses can contain unchanged values and unchanged simulator time. Never require
running physics progression for paused final verification, or infer it from packets.
The manager fails on the first mismatching verification frame; it does not wait
through a new convergence phase. READY is monitored until stopped in this trial.

Budget evidence, not approval: native acquire has a 1.5 s response deadline;
handoff joins the reader for 1.5 s; L confirms initial pause within 3 s, altitude
within 5 s, condition readback within 3 s, with paused telemetry reads potentially
blocking up to 2 s plus engine-support queries. These nested waits and real host
scheduling must inform lifetime/freshness approval. Do not sum them into an assumed
approved deadline, inherit mock budgets, increase legacy timeouts, or equate
callback receipt time with internal sensor acquisition time.

## Blocking decisions and technical limitations

- [ ] Named reviewer/date/evidence record; confirm C and all operator overrides.
  Existing implementation and successful graphs do not constitute approval of
  new mandatory gates. No separate signed approval record was found.
- [ ] Approve IAS/ground-speed target bands, heading/pitch/bank bands and geographic
  limits; preserve ±25 ft altitude, ±1 percentage point throttle and ±0.1 flap index.
- [ ] Define paused Manual engine/prop RPM acceptance separately from recording
  start and pre-throttle RPM. **Schema limitation:** `_validate` applies the
  registry's 60 RPM equality constraint to a Manual engine check although its
  evidence is Replay-only. Either approve that numerical band specifically for
  the chosen Manual target or review a minimal scope correction separately.
  Do not silently transfer approval or command a different engine state.
- [ ] Approve mixture/propeller-lever target bands or justified trial exclusions;
  decide whether transient manifold pressure needs a fixed target at all.
- [ ] Approve proposed weather targets and diagnostic-derived bands, or provide
  justified alternatives. L supplies OAT/altitude only to `reference_weather_metar`;
  current METAR wind and pressure are retained. Historical wind/1013 setting are
  not automatically applied by Manual initialization. Operator-established weather
  must be documented; mismatch must fail, not cause extra restoration commands.
- [ ] Decide whether ambient-pressure verification is mandatory for this trial.
  It is available in V1 but absent from registry checks. It cannot be enabled by
  putting an unsupported key in `checks`. Approve a scoped deferral or request a
  separately reviewed minimal extension. Do not claim it checked via KOHLSMAN.
- [ ] Approve each of six exclusions and each non-schema deferral individually.
  Document independent simulator loading/configuration evidence. No validated
  acquisition-fresh alternate source was established for weight/fuel/CG/trim.
  Screenshots/operator inspection can support a supervised exclusion, not replace
  machine-authoritative telemetry or establish numerical readiness automatically.
- [ ] Approve `timeout_s`/`max_age_s`, paused sampling interpretation and three-frame
  rule. No deadline or freshness number is invented in this candidate.
- [ ] **Configuration binding limitation:** `load_plan` uses session conditions
  before `_initialize_mode` reloads C and overrides. A changed file/session can
  produce different planned and commanded targets. For a trial, freeze reviewed
  files/settings, launch a new worker and compare `planned_checks` against the
  published `initial_conditions`. Any discrepancy blocks the experiment. If this
  procedural control cannot be guaranteed, request a minimal pre-dispatch binding
  correction before live testing. No such production correction is made here.
- [ ] **Approval metadata limitation:** the loader does not enforce draft/approved
  flags, reviewer signatures or file hashes. The candidate remains unexecutable
  through missing required targets/limits and empty `review_source`; changing its
  documentary status alone is not approval. Control the approved copy explicitly.
- [ ] Approve operator handling of no-recording cancellation and actual pause /
  hardware release inspection. Legacy cleanup returning is not independent release
  measurement. Live disconnect/reconnect rejection still needs supervised evidence.

**These unresolved items prevent a READY-seeking supervised initialization-only
trial today.** An explicitly authorized blocked-path diagnostic could stop at
validation, but would not validate initialization and is not launched by this stage.
No DLL, brake, replay or flight-control change is needed merely to review the plan.

## Exact supervised procedure, only after the decisions are approved

1. Save a separate complete approved JSON, e.g.
   `C:\reviewed-plans\deceleration-manual-approved.json`. Resolve every null,
   replace `review_source` with the approval record, retain supported schema and
   existing limits, attach the signed exclusions/deferrals, and record its SHA-256.
   Confirm approved targets match unchanged C and reviewed operator settings.
   Do not activate this draft. Close QTG Studio, workers and other diagnostics.
2. With the operator supervising the stationary aircraft, confirm simulator model,
   loading/configuration and documented weather. The existing preparation WILL
   apply ground position, OAT, throttle/flaps and Manual BRUNNER preparation.
   Clear the area/controls for that established procedure; do not manually perform
   the engine-response manoeuvre or start an unrelated monitor.
3. In a new PowerShell window, run only after explicit live-test authorization:

```powershell
Set-Location '<PROJECT_ROOT>'
$env:QTG_UNIVERSAL_ENGINE_ACCELERATION = '0'
$env:QTG_UNIVERSAL_ENGINE_DECELERATION = '1'
$env:QTG_DECELERATION_BRAKE_CAPTURE = '0'
$env:QTG_UNIVERSAL_INITIALIZATION_ONLY = '1'
$env:QTG_UNIVERSAL_ENGINE_DECELERATION_PLAN = 'C:\reviewed-plans\deceleration-manual-approved.json'
Get-FileHash -Algorithm SHA256 -LiteralPath $env:QTG_UNIVERSAL_ENGINE_DECELERATION_PLAN
python main.py
```

4. Select **1.f.(2) Engine Deceleration -> Manual**, review settings, then open
   its worker. Opening can initialize immediately. Do not choose Automatic, Record
   or PID. Watch preparation and verification. An issued/returned command is not
   measured convergence or independent native command acknowledgement.
5. In the current `D:\QTG_Test_Results\sessions\<session>_status.json`, save evidence
   of a new attempt ID, `manager_enabled=true`, `initialization_only=true`, selected
   system `Universal Initialization Manager`, Manual dispatcher, reviewed source,
   planned/observed/verified checks, current generation/request/sequence/receipt
   age, `running=false`, direct physical readbacks and final READY or precise
   FAILED reason. Check planned targets against published initial_conditions.
   Confirm `recording=false`, `samples=0`, `can_record=false` and no test CSV was
   started. Capture console/status outside the live status path before stopping.
6. On mismatch/timeout/disconnection or unexpected activity: Stop; do not widen a
   limit or retry automatically. Inspect actual P3D pause and physical release.
   Save failure/cleanup evidence. In a separate fresh worker, supervised cancellation
   should produce CANCELLED and no recording; late callbacks must not revive READY.
   A separately authorized reconnect negative test must fail the old attempt and
   require a new worker/attempt. Do not undertake it automatically.
7. Stop and close the worker/application even after successful READY. The lock is
   latched for the worker lifetime. No recording, throttle reduction or later test
   workflow is authorized by this trial. Report results before further activation.

## Exact rollback

Stop initialization, confirm actual pause/release, save evidence, close the worker
and QTG Studio, and confirm they exited. In the launching PowerShell:

```powershell
$env:QTG_UNIVERSAL_ENGINE_DECELERATION = '0'
$env:QTG_UNIVERSAL_ENGINE_ACCELERATION = '0'
$env:QTG_DECELERATION_BRAKE_CAPTURE = '0'
Remove-Item Env:QTG_UNIVERSAL_ENGINE_DECELERATION_PLAN -ErrorAction SilentlyContinue
Remove-Item Env:QTG_UNIVERSAL_INITIALIZATION_ONLY -ErrorAction SilentlyContinue
python main.py
```

Relaunching normally restores legacy selection; do not start another test without
operator authorization. No files, references, DLLs, weather procedures or source
changes need reverting. Do not use `setx`, Git, forced process termination as a
normal cleanup method, or unlock a still-running initialization-only worker.
