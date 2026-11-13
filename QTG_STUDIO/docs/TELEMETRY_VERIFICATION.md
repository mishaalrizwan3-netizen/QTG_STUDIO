# Stage 3D source audit

No connection, packet request, control command, runtime wiring or feature activation
is performed by `backend/telemetry_verification.py`. Supply the existing running
SimConnectClient. BRUNNER input is an optional externally maintained stamped cache;
the adapter does not call `read_axis`, which sends UDP queries and may return zeros
on errors. BRUNNER availability is separate from the P3D Feedback.connected flag.
Consumers requiring hardware feedback must check the separate snapshot explicitly.

Evidence: `interfaces/brunner_base/src/p3d_simconnect_v15.py::_update_latest_data`,
`get_telemetry`, `get_telemetry_sequence`; native definitions in
`interfaces/brunner_base/native/src/p3d_bridge_v15_full_causal_state.cpp`;
`src/brunner_interface.py::read_axis`; weekend `_shared/engine_initialization.py`;
`qtg_common_engine.py::load_base` telemetry patches.

| Measurement | Native source / Python channel | Units and transformation |
|---|---|---|
| Engine / prop RPM | GENERAL ENG RPM:1 / PROP RPM:1; eng_rpms[0] / prop_rpms[0] | rpm; use independent native arrays, not engine_rpm fallback alias |
| Throttle | GENERAL ENG THROTTLE LEVER POSITION:1 / engine_throttle_percent | percent; Python clamps 0–100 |
| Mixture / prop lever | GENERAL ENG MIXTURE / PROPELLER LEVER POSITION:1 | native fraction multiplied by 100 in Python |
| Manifold pressure | RECIP ENG MANIFOLD PRESSURE:1 / engine_manifold_pressure_psi_1 | psi; no inHg conversion in adapter |
| IAS / ground speed | AIRSPEED INDICATED / GROUND VELOCITY | knots; IAS is clamped nonnegative in existing driver |
| MSL altitude | PLANE ALTITUDE / pressure_alt | ft MSL despite misleading Python name |
| Indicated altitude | INDICATED ALTITUDE / indicated_altitude | feet indicated; not MSL or pressure altitude |
| Pressure altitude | no dedicated mapped native channel established | unavailable; do not relabel MSL |
| True / magnetic heading | PLANE HEADING DEGREES TRUE / MAGNETIC | degrees; modulo 360 in Python; registry uses true |
| Pitch / bank | PLANE PITCH / BANK DEGREES | degrees; bank sign reversed by existing Python driver |
| Vertical speed | VERTICAL SPEED / vertical_speed | ft/min; separate from velocity_world_y (ft/s) |
| Position | PLANE LATITUDE / LONGITUDE | degrees |
| Flaps | measured_flaps_handle_index | handle index; native main ABI slots are SIM ON GROUND placeholders; excluded until bridge mapping is validated |
| Trim | ELEVATOR TRIM POSITION; trim_elevator_angle_rad | radians; aileron/rudder trim fractions converted to percent |
| Weight / fuel | TOTAL WEIGHT / FUEL TOTAL QUANTITY WEIGHT | kg |
| CG | CG PERCENT / cg_percent | percent MAC; lateral cg_percent_lateral is actually CG LATERAL in feet |
| OAT / ambient pressure | AMBIENT TEMPERATURE / PRESSURE | Celsius / millibars |
| Barometer | KOHLSMAN SETTING MB / baro_pressure | mbar, numerically hPa; altimeter setting, not ambient pressure |
| Wind | AMBIENT WIND DIRECTION / VELOCITY | degrees / knots; reference convention requires simulator confirmation |
| Ground contact | sim_on_ground from extended causal block | bool; separately sequenced merged block, excluded from adapter until independently stamped |
| Pause | existing bridge Pause system-event state and pause confirmation methods | independently updated; no timestamped sample exported to this adapter; unavailable |
| Physical BRUNNER axes | CLS2Sim read_axis position 0x11 | normalized -1 to +1, not P3D control surfaces |
| BRUNNER forces | applied 0x21 / virtual 0x51 (and runtime 0x50 fallback) | scaling/provenance varies; no force-unit normalization or substitution added here |

Main bridge telemetry requests SIMCONNECT_PERIOD_SIM_FRAME. Extended causal and
engine-support requests specify interval 4 (frame-dependent, not a guaranteed Hz).
Python sampling/recording around 50 Hz does not prove native acquisition at 50 Hz.
BRUNNER queries are caller-driven; no guaranteed cached update rate established.

## Freshness and consistency limits

The adapter brackets the cache read with bridge-sequence reads and rejects differing
sequences, repeated sequence/time, nonfinite/missing timestamps, old/future times,
and wall-clock jumps. It converts Unix cache time to host monotonic time with an
explicit anchored offset; simulation-time clocks are never guessed or accepted.
This is a conservative best-effort cache check, NOT an atomic native snapshot:
the driver's sequence and Python copy timestamp are not native acquisition stamps.
Per-channel native registration validity is also not exported. Finite zero does
not prove a channel was successfully acquired. DLL/ABI and supported-channel
checks are required before using these values for live authorization.

Engine support/extended blocks are separately sequenced and can be older than main
telemetry. No new thermal or ground-state merging is performed. Fuel-flow values
can be synthesized by load_base(), so fuel flow is deliberately not mapped.
No missing value is substituted. A source fault raises TelemetryUnavailable; the
manager fails and requests cleanup through its separately supplied lifecycle adapter.

StabilityWindow requires explicit duration, minimum distinct samples, maximum gap
and per-channel range limits. It checks units/channels, circular heading ranges,
and resets on missing samples, gaps, repeated frames, disconnection or excessive
spread. Combine with manager target checks; this gate is not wired to runtime.
Do not interpret the manager's sample-count rule alone as a duration guarantee.

Next: validate read-only snapshots against simulator instruments and existing logs,
confirm ABI/channel validity, pause/ground/flaps and clock provenance, then define
reviewed stability windows and connect a feedback provider in an offline adapter.
Universal initialization remains disabled; no live preparation adapter is provided.
