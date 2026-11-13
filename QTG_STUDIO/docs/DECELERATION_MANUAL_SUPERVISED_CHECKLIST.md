# Deceleration Manual Supervised Initialization Checklist

**Prepared only. No live authorization or automatic activation.** Use after the
[Decision Sheet](DECELERATION_MANUAL_DECISIONS.md) is signed and a separate complete
approved plan is issued. The [proposal](deceleration_manual_plan_proposed.json)
must not be activated. This trial ends at paused READY or a blocking/failure result;
it does not record, reduce throttle or run Automatic Replay.

## Before launch

- [ ] All mandatory numerical targets/bands, freshness/deadline budgets and
  exclusions/deferrals approved. Paused engine/prop RPM are resolved separately
  from recording-start and pre-event RPM. No unsupported mandatory measurement.
- [ ] Complete approved JSON at `docs/deceleration_manual_plan_approved.json` (issued only after signoff).
  Stage 5J approval record, check/exclusion key coverage, payload and configuration
  identities match final files and operator overrides. Do not modify active inputs.
- [ ] Identify earlier QTG workers/diagnostics and arrange their normal operator-supervised
  stop before this trial; do not terminate an active demonstration remotely. Leave the
  installed DLL unchanged. Confirm the source application and reviewed configuration match.
- [ ] Operator verifies intended stationary ground location, loading/configuration
  evidence and approved weather setup. Opening the worker can perform the existing
  aircraft initialization and BRUNNER Manual preparation; this is not passive monitoring.

Only after separate supervised-live authorization, launch from a new PowerShell:

```powershell
Set-Location '<PROJECT_ROOT>'
$trialPlan = (Resolve-Path -LiteralPath 'docs/deceleration_manual_plan_approved.json').Path
# Offline preflight only: rejects draft, incomplete, changed or unapproved inputs.
python -B -c "import sys; from backend.engine_initialization_adapter import load_plan; p=load_plan({'test_reference':'1.f.(2)','mode':'manual'},sys.argv[1]); print('Approved plan preflight PASS',p.approval.identity)" $trialPlan
if ($LASTEXITCODE -ne 0) { throw 'Plan preflight failed; do not launch the trial.' }
Get-FileHash -Algorithm SHA256 -LiteralPath $trialPlan
$env:QTG_UNIVERSAL_ENGINE_ACCELERATION = '0'
$env:QTG_UNIVERSAL_ENGINE_DECELERATION = '0'
$env:QTG_UNIVERSAL_AUTOMATIC_PLANS = '{}'
$env:QTG_DECELERATION_BRAKE_CAPTURE = '0'
$env:QTG_ACCELERATION_MANUAL_AIRFLOW_PREPARATION = '0'
$env:QTG_UNIVERSAL_INITIALIZATION_ONLY = '1'
$env:QTG_UNIVERSAL_MANUAL_PLANS = ConvertTo-Json -Compress -InputObject @{'1.f.(2)' = $trialPlan}
python -B main.py
```

This uses the shared generic Manual mapping with both older engine switches OFF.
Do not execute until the plan is explicitly approved and separate live authorization
is given. Opening Manual invokes the unchanged preparation, including configured
100% throttle and existing BRUNNER Manual-ownership commands. The operator must be
present, stationary on the ground, with hardware stop/release accessible. No Record,
Space, throttle reduction, PID, Replay or competing diagnostic is part of this trial.

Final approval record checklist:

- [ ] Sign the existing-versus-proposed numerical criteria in the decision sheet.
- [ ] Confirm configured stationary ground speed 0 kt, wind 0 kt, QNH 1013 hPa,
  OAT 15 C and all other unchanged GUI targets; wind direction is conditionally N/A.
- [ ] Approve **each** of the 11 exclusions, with independent loading, CG, trim and
  cockpit-lever evidence; explicitly disposition fuel and other non-schema deferrals.
- [ ] Approve 45 s overall trial deadline, 1 s native receipt age and three new matching
  responses; preserve earlier local failures and final paused-state/ownership checks.
- [ ] Record reviewer, timezone-bearing timestamp and evidence; finalize plan/check/
  exclusion approval lists and payload/configuration identities. Do not use synthetic
  test approvals. Keep proposed JSON inactive and issue the approved copy separately.
- [ ] Confirm no unsupported measurement is still mandatory. If one is, STOP for a
  validated source or separately reviewed software change.
- [ ] Record exact source/configuration/approved-plan hashes and 329-test result.


## Observe and retain evidence

1. Select **1.f.(2) Engine Deceleration -> Manual**. Review exact settings before
   opening its worker. Do not select Automatic, Record or PID; do not perform
   throttle reduction or launch a competing verification monitor.
2. Observe unchanged preparation followed by Universal verification. Preserve the
   console and current worker's exact session status JSON in
   `D:\QTG_Test_Results\sessions`; identify it from the worker context, not an old file.
3. Confirm new attempt ID, Manual mode, `manager_enabled=true`,
   `initialization_only=true`, selected Universal system, dispatcher provenance,
   `approved_plan_identity`, `reviewed_input_identity`, `configuration_identity`,
   approved source, planned checks/exclusions and matching published conditions.
4. Confirm fresh native generation/request/sequence and receipt age, genuine
   ground/flap fields, independent engine/prop RPM, weather/instrument-setting
   values, correlated `running=false`, and separate physical-control readbacks.
   Three fresh matching paused responses may be unchanged; this does not prove
   running physics advancement or internal sensor acquisition timing.
5. At READY, confirm `recording=false`, `samples=0`, `can_record=false` and no QTG
   recording CSV. Save a copy of status evidence before Stop overwrites it.
   A returned command is not independent acknowledgement or measured convergence.
   Ambient pressure and approved excluded fields must not be reported as verified
   target checks if they were only observed/deferred.
6. On BLOCKED, FAILED, timeout, disconnect or unexpected aircraft/hardware activity,
   Stop and save the exact reason plus cleanup results. Do not change limits,
   repeatedly reinitialize or silently return to legacy readiness in that session.
7. Stop and close after inspection, including successful READY. In a separately
   supervised fresh attempt, exercise cancellation and verify no late READY/no
   recording, actual simulator pause and actual physical release. Perform a live
   disconnect/reconnect negative check only with separate authorization; the old
   attempt must not recover READY automatically.

Neither environment flags nor a green GUI alone prove execution. The provenance,
approved identities and genuine matched measurements are required evidence.
No full Manual run or comparison is authorized by this initialization-only result.

## Stop and rollback

Use normal Stop, inspect actual pause/release, save evidence and close the worker
and application. The initialization-only lock is latched until worker exit.
In the launching shell:

```powershell
$env:QTG_UNIVERSAL_ENGINE_DECELERATION = '0'
$env:QTG_UNIVERSAL_ENGINE_ACCELERATION = '0'
$env:QTG_DECELERATION_BRAKE_CAPTURE = '0'
$env:QTG_UNIVERSAL_MANUAL_PLANS = '{}'
$env:QTG_UNIVERSAL_AUTOMATIC_PLANS = '{}'
Remove-Item Env:QTG_UNIVERSAL_ENGINE_DECELERATION_PLAN -ErrorAction SilentlyContinue
Remove-Item Env:QTG_UNIVERSAL_ENGINE_ACCELERATION_PLAN -ErrorAction SilentlyContinue
Remove-Item Env:QTG_UNIVERSAL_INITIALIZATION_ONLY -ErrorAction SilentlyContinue
# Relaunch only if separately authorized; no automatic restart.
```

A subsequent authorized source launch uses legacy selection; do not begin another test without operator
authorization. No DLL, references or source files require replacement. Do not use
Git, `setx` or forced termination as normal cleanup. Report live findings and
unresolved measurements before seeking any further activation.
