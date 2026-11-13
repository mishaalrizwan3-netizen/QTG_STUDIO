# Deceleration Manual Engineering Decision Sheet

**APPROVED FOR LIMITED TRIAL; LIVE EXECUTION NOT AUTHORIZED.** On 2026-11-12T10:53:22+05:00,
mishaal explicitly approved the 14 checks, 11 exclusions and four diagnostic
deferrals in the existing plan. Approval applies to the exact bound configuration
and plan identity; the historical review sections below predate this decision. Scope: 1.f.(2) Manual,
final paused preparation state, before recording capture/timer and unpause.
No manoeuvre or Automatic activation is covered.

The [Approval Matrix](DECELERATION_MANUAL_APPROVAL_MATRIX.csv) has one row per
parameter or invariant, including sources, units, mandatory status, consequences
of tight/loose limits and additional validation. The [proposed plan](deceleration_manual_plan_proposed.json)
has complete configured check targets and now contains the recorded approval. The [live checklist](DECELERATION_MANUAL_SUPERVISED_CHECKLIST.md)
applies only after approval and separate operator authorization.

## Latest operator-selected trial limits ? supersedes earlier numerical proposals

**HISTORICAL REVIEW BEFORE APPROVAL.** The latest operator request specifies limited-trial
limits, not EASA qualification limits. Targets remain the resolved GUI/file conditions.

| Parameter | Latest trial proposal |
|---|---|
| MSL altitude | +/-25 ft (existing supported limit, operator selected) |
| Heading | +/-5 degrees, circular |
| Pitch / bank | +/-2 degrees each |
| Throttle | +/-1 percentage point (existing supported limit, operator selected) |
| OAT | +/-5 C |
| IAS / ground speed | +/-0.5 kt each |
| Wind | Maximum 1 kt; direction conditionally excluded only for configured calm |
| QNH | +/-1 hPa |
| Latitude / longitude | +/-0.001 degree each |
| Genuine flaps / ground | +/-0.1 genuine handle index / TRUE |
| Freshness / responses / deadline | 1 s native receipt age / at least 3 matching fresh responses / 45 s |

**Tolerance compatibility:** The operator selected the supported registry limits:
altitude +/-25 ft, throttle +/-1 percentage point and genuine flap handle +/-0.1.
The previously identified exact-tolerance conflict is resolved in the draft. These
limits and all other trial criteria still require formal engineering approval.
The existing Manual preparation and its independent checks are unchanged.

**Available setup evidence:** the operator stated, "I have checked the aircraft
weight, CG, fuel and trim settings and consider the setup satisfactory." Record
this as operator attestation, not missing evidence and not native verification.
Numerical values, time and supporting files were not supplied. Engineering must
accept the attestation for the limited trial or specify the necessary corroboration.
Mixture and propeller-lever setting confirmation are still absent.

Each of the existing 11 exclusions still needs individual approval. Engine and
propeller RPM remain separate paused diagnostic observations; no later RPM target
is imported. Mixture and propeller lever need independent cockpit confirmation;
manifold pressure needs an explicit paused-phase scope exclusion. Loading/CG and
each trim exclusion require explicit acceptance of the operator setup evidence.
Calm direction exclusion must be confirmed alongside the actual-speed allowance
up to 1 kt. Separately approve fuel, density/local airflow, ambient pressure and
indicated-altitude deferrals; no unavailable measurement is silently passed.

**Formal approval missing:** explicit checks/policy/deadline and exclusion
signoff, reviewer name, timestamped approval record and accepted evidence references.
The current request asks for engineering review and does not supply those records.
The plan remains unapproved. Earlier 329 tests are evidence for existing software,
not approval of these new trial bands. No live commands are authorized by this document.

## Stage 2 review using the 12 November Stage 1 observation

Evidence: `D:/QTG_Test_Results/sessions/p3d_verification/434b7e866dbc4aba8624ab3a941ff64a`.
The CSV contains 437 channel rows: 19 fresh responses for all 23 channels. Native
generation remained 1, request/sequence identifiers advanced, receipt age was
0.237-11.2181 ms, and running=false throughout. These are repeated observations of
one intentionally paused preparation, not 19 independent preparations or evidence
of running stabilization. Internal simulator acquisition timestamps remain unavailable.

The Stage 1 archive predates the operator-selected weather keys now present in configuration.
The proposal now uses `target_source: resolved_initial_conditions` for every check;
all former literal plan targets have been removed. Its configuration identity is
current but is not approval. The draft has 14 proposed checks: the existing mandatory aircraft/weather checks
with calm direction explicitly conditional, plus the proposed ground-speed gate.
Engine/propeller RPM, mixture and propeller lever are proposed diagnostic-only
exclusions for this limited trial; each still needs explicit approval.

The mandatory software parameters are `on_ground`, `alt_ft`, `ias_kt`, `lat`,
`lon`, `heading_deg`, `pitch_deg`, `bank_deg`, `throttle`, `flaps`, `temperature_c`,
`wind_speed`, `wind_direction`, and `barometer`. Final pause, fresh correlated
feedback, at least three matching responses, approval/configuration identity,
bounded deadlines, cancellation and BRUNNER ownership/direct-read checks are
additional runtime safeguards. They are not replaced by numeric parameter matches.

**Current status: BLOCKED pending engineering approval and deadline.** The operator
selected `wind_speed=0` kt and `barometer=1013` hPa in the existing initialization.json;
`temperature_c=15` C is unchanged. These are operator selections, not copied observations.
The existing resolver supplies them to verification. The operator also selected `ground_speed=0` kt in the same configuration;
its proposed +/-0.1 kt gate remains subject to approval. No target literals are duplicated in the plan.
The configuration identity has been refreshed for review; no approval has been issued.

The draft now selects an explicit conditional Manual policy: wind direction is not
applicable only when the immutable configured wind-speed target is exactly zero.
The exclusion still requires individual engineering approval, and the wind-speed
check remains mandatory. Nonzero speed requires direction; changed configuration
or policy invalidates the previous identity. Automatic Replay is unchanged.

Existing Manual preparation commands temperature but preserves current wind and
pressure. Configuration targets do not add weather commands: the operator must
establish the selected calm wind and QNH before preparation. No weather behavior
has been changed. If automatic application or GUI weather editors are required,
that is separate work, not part of this limited initialization-only trial.

Proposed new numerical bands below are review options, not approved limits:
IAS/ground speed +/-0.1 kt (about 0.0514 m/s), true heading +/-0.1 degree circular,
pitch +/-0.5 degree, bank +/-0.1 degree. They select a narrow stationary/ground-pose
screening envelope. Stage 1 supports considering them (IAS/ground speed 0, heading
error about 1.5e-9 degree, pitch +0.2 degree, bank 0); it does not validate these
particular widths. Earlier ground pitch +0.1134 degree supports investigating
gear/terrain settling, not declaring it the cause. Review terrain, sign, allowed
orientation error and independent preparation repeatability before approval.
Too tight rejects numerical/ground settling; too loose accepts incorrect motion
or orientation. Do not alter GUI attitude targets to equal the observations.

Retain existing software limits for review: altitude +/-25 ft, coordinates
+/-0.001 degree, throttle +/-1 percentage point and genuine flap index +/-0.1.
Stage 1 errors were -5.49999 ft, negligible coordinate error, -0.006104 percentage
point and zero genuine-flap error. These implementations still need trial signoff;
the coordinate envelope is broad and needs location/scenery confirmation.

The proposed 1-second receipt-age budget retains the Stage 1 diagnostic setting
and allows host/sampler/handoff delay relative to observed response age, subject
to engineering approval. It is not sensor acquisition age. Full initialization
duration was not captured. The final draft proposes `timeout_s=45` seconds from
existing operation wait budgets, with engineering approval still required. The three-response requirement remains unchanged.

Stage 1 verified native flap index **0**. The legacy cache's **1** is inconsistent
and is not an acceptance source. Both native shaft RPMs were **0** while paused;
this does not establish a target or prove the engine's running/off condition.
BRUNNER's socket flag and `controlling=false` are evidence of worker state only;
strict direct axis reads and competing-owner rejection remain required at runtime.

The code's initialization-only flag is latched, publishes `can_record=false` and
rejects recording/PID commands; the inherited Record entry is guarded too. The
319-test offline result supports those safeguards. Stage 1 itself used legacy
mode with `can_record=true`; its zero samples do not demonstrate the live lock.
The future approved trial must independently confirm `recording=false`, `samples=0`
and `can_record=false` even after READY. No flag is activated by this review.

## Decisions requiring a recorded engineering disposition

| Decision | Matrix rows | Required disposition |
|---|---|---|
| D01 Existing configuration and retained limits | M01-M04, M10-M11 | Confirm intended runway/scenery and no-override defaults: latitude 32.23366667, longitude 74.1235, 755 ft MSL, ground true, flap index 0, throttle 100%. Retain existing +/-0.001 degree coordinate, +/-25 ft MSL, +/-0.1 flap index and +/-1 percentage-point throttle limits, or explicitly request separate review before changing existing limits. Approve exact ground equality and the genuine UP readback gate. Existing implementation is evidence, not a newly issued approval. |
| D02 Stationary air/ground state | M05-M06 | Review proposed +/-0.1 kt IAS and ground-speed bands; Stage 1 measured both as 0 throughout one paused capture. IAS target stays GUI-defined. The operator has explicitly selected ground_speed=0 kt in the authoritative configuration; approve the additional stationary gate and its band. No legacy 35 kt or pre-event 3.14 kt substitution. |
| D03 Heading and ground attitude | M07-M09 | Review proposed circular heading +/-0.1 degree, pitch +/-0.5 degree and bank +/-0.1 degree as provisional screening options. Stage 1 errors were approximately 0/+0.2/0. Widths require engineering justification and independent repeatability; 19 paused frames alone do not validate them. Keep configured targets unchanged. |
| D04 Phase-specific engine and propeller RPM | M12-M13 | Approve proposed diagnostic-only exclusions for paused Manual, unless a reviewed phase specification explicitly requires configured RPM targets and bands. The current software permits these exclusions after explicit review. Stage 1 read 0/0 while paused; historical 989/989 recording start and 2680/2680 pre-throttle are separate phases. Do not configure 0, 989 or 2700 merely to obtain READY. Replay RPM checks and tolerances are unchanged. |
| D05 Engine configuration | M29-M31 | Approve proposed trial-only mixture, propeller-lever and manifold-pressure exclusions individually, or establish explicit configuration targets and justified bands if mandatory. Stage 1 100% levers and 14.6958 psi are observations only. Existing Manual commands do not set these two levers. |
| D06 Weather and instrument setting | M14-M17 | Retain GUI OAT target and review diagnostic-derived +/-1 C. Operator-selected wind speed 0 kt and QNH 1013 hPa now resolve from the existing source. Approve the calm-only direction exclusion and proposed +/-1 kt / +/-1 hPa bands; no direction tolerance is needed for this calm-only plan. These are diagnostic-derived proposals, not approved readiness limits. Nonzero wind requires a direction target and reviewed band. Preserve existing weather behavior. |
| D07 Ambient pressure and indicated altitude | M18, M33 | Recommend explicit observed-only deferral for this limited trial. Ambient pressure is available in V1, but neither it nor indicated altitude has a registry plan gate. If either is mandatory for READY, target/band approval alone is insufficient: supported schema work must be separately reviewed before activation. Never approve atmospheric-pressure equivalence from an altimeter-setting match. |
| D08 Loading and trim exclusions | M19, M21-M24 | Decide each proposed registry exclusion separately: `weight`, `cg`, `elevator_trim`, `aileron_trim`, `rudder_trim`. Document independent loading/configuration evidence and why limited initialization-only scope permits the exclusion. No validated acquisition-fresh alternate is established. Suspect historical CG -89.984% must not become a target. If any is mandatory, block until a validated source exists. |
| D09 Fuel and atmospheric diagnostic scope | M20, M34 | Approve or reject explicit trial-only deferrals for fuel mass, density and local aircraft airflow. These are not implemented registry gates; the review record must not imply machine enforcement. Do not import airborne recovery requirements or infer parking-brake OFF from missing telemetry. Stage 5H capture remains off. |
| D10 Freshness and lifetime | M26-M28 | Review proposed `max_age_s=1.0` against Stage 1 receipt ages and existing host/sampler/handoff latency. Retain at least three matching new responses. Review the proposed 45-second full-attempt budget described below; capture actual preparation/verification elapsed times during the trial. Local timeouts remain unchanged. Preserve internal deadlines and distinguish paused response freshness from physics progression. |
| D11 Pause, authority and handover | M25, M32 | Retain final correlated `running=false`, finite acknowledged direct BRUNNER axis reads and competing-controller rejection. Approve the existing Manual hands-off authority interpretation without introducing historic physical-position targets, PID, replay or new actuator commands. Define operator checks of actual pause and physical release during cancellation. |
| D12 Formal approval and file custody | M35 | Named authorized reviewer, timestamp with timezone, evidence record, all check/exclusion decisions and explicit non-schema deferrals. Finalize exact settings/overrides, all targets/bands and budgets before computing identities. Issue a separate approved copy with the Stage 5J approval object and complete key lists. Do not edit active inputs or use synthetic test approvals. Content hashes are not reviewer authentication. |

Numeric values are intentionally withheld where one historical trace cannot justify
a range. Complete those decisions with independent engineering bounds and paused
repeatability data, not wider limits selected to obtain READY.

## Individual exclusion signoff

Each entry requires a named reviewer, evidence and the limitation of
**initialization-only, no recording, no qualification claim**:

- [ ] `wind_direction`: conditional exclusion only for configured wind speed exactly zero; wind-speed verification remains mandatory.
- [ ] `manifold_pressure`: available V1 observation, but no approved paused fixed target.
- [ ] `engine_rpm`: phase-specific Manual diagnostic observation; Replay unchanged.
- [ ] `prop_rpm`: independent phase-specific Manual diagnostic observation; Replay unchanged.
- [ ] `mixture`: no configured target; observed 100% is not approval.
- [ ] `propeller_lever`: no configured target; observed 100% is not approval.
- [ ] `weight`: legacy value only; independent simulator loading evidence required.
- [ ] `cg`: suspect historical decoding; do not substitute a plausible CG.
- [ ] `elevator_trim`: no validated authoritative acquisition for this field.
- [ ] `aileron_trim`: legacy scaling documented, live provenance not validated.
- [ ] `rudder_trim`: legacy scaling documented, live provenance not validated.

Separate written deferrals, if accepted: ambient pressure, indicated-altitude
offset, fuel, density/local airflow. These are scope decisions, not enforceable
JSON exclusions. Ground speed remains a proposed additional check requiring a
configured target. Mixture and propeller lever are now proposed exclusions;
neither disposition is approved automatically.

## Prerequisites to approval

Use the [Stage 5J safety contract](INITIALIZATION_PLAN_SAFETY.md), which supersedes
older claims about unenforced approvals, pre-reload plan construction and Manual
inheritance of Replay's RPM limit. Phase observations, hashes and source paths are
preserved in the proposed plan; they are supporting evidence, not approved targets.

Stage 1 now supplies paused-state observations. Any additional independent
preparation/repeatability or duration evidence requires separate authorization
through the existing legacy setup and validated read-only verification route.
Keep both Universal switches off for that evidence collection; do not use a
fabricated temporary RPM plan to enter preparation. The existing
[diagnostic-only instructions](P3D_DIAGNOSTIC_ONLY.md) and supervised caller
instructions govern connection ownership. No new live session is authorized here.

After all decisions are resolved, the trial may be considered for authorization
**only if its chosen mandatory requirements are executable using the existing
validated sources and schema**. If a deferred unsupported measurement is judged
mandatory, technical validation work remains necessary before authorization.


## Final preparation: deadline recommendation and exclusions

**Proposed overall deadline: 45 seconds, NOT APPROVED.** Current code waits:
3 s for pause; 1.5 s weather read; 5 s altitude loop plus up to 3 s for its last
telemetry/support read; 3 s flap pause; 3 s final configuration loop plus up to
3 s for its last read. Subtotal: 21.5 s. Allow roughly 7 s for initial ownership
acknowledgements, three native requests (1.5 s each), three-axis physical reads
(nine 0.05 s socket waits per frame), sampling and UI delivery. Round to 30 s
plus 15 s contingency. Existing local failures stop earlier; no physical band changes.
Source: backend/initializer.py; Super_Mushak_Logger_GUI.py:_wait_for_initial_altitude,
_refresh_paused_telemetry; p3d_simconnect_v15.py:verification_snapshot;
brunner_interface.py:prepare_manual_mode/read_axis.

Stage 1 archived status and its empty worker log do not establish full preparation
elapsed time. This recommendation is a conservative trial budget, not an empirical
worst-case or a guarantee that blocked native/lock/OS calls are interruptible.
Supervision and physical stop controls remain required. Capture attempt start,
preparation return, first native response, final READY/failure and cleanup timings.

Review **each** registry exclusion separately:

| Exclusion | Trial justification / independent evidence required |
|---|---|
| engine_rpm | Diagnostic paused shaft speed; no approved paused target. Preserve later engine preparation and Replay RPM checks. |
| prop_rpm | Independent diagnostic shaft speed; no approved paused target. Same phase separation. |
| mixture | No configured target; operator verifies intended cockpit lever and engine configuration before trial. |
| propeller_lever | No configured target; independently verify cockpit lever configuration. |
| manifold_pressure | Observe native measurement; not a fixed engine-operating requirement while paused. |
| wind_direction | Exact configured calm only; wind-speed verification remains mandatory. |
| weight | No authoritative V1 measurement; dated simulator loading-screen evidence required. |
| cg | No validated authoritative measurement; independent loading/CG evidence required; reject suspect historical decoding. |
| elevator_trim | No validated V1 source; independent cockpit/trim-setting evidence required. |
| aileron_trim | No validated V1 source; independent cockpit/trim-setting evidence required. |
| rudder_trim | No validated V1 source; independent cockpit/trim-setting evidence required. |

Separately record trial-only diagnostic deferrals for fuel (independent loading/fuel
screen), density/local airflow, ambient pressure and indicated altitude. The latter
two are native observations but not enforced registry gates. If engineering makes
any unavailable or non-schema measurement mandatory, activation stays blocked until
a validated source/gate is implemented; approval cannot make an unavailable value pass.

Final signoff must name the reviewer, timestamp and evidence record, cover every
check/exclusion and non-schema deferral, and bind the final configuration and plan
identities. This draft is not approved by editing values or computing hashes.
