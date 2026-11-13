# Stage 4A: weather command versus measured atmosphere

No weather-setting behavior, placement, power, hardware ownership or readiness
decision is changed. The only logger changes are best-effort diagnostic hooks.
Logs use `[Weather diagnostic]` JSON lines in the existing worker session log;
the same observations appear in session context `weather_diagnostics` for saved
run metadata. Hooks add no telemetry requests, sleeps, weather writes or connections.

## Evidence chain

The native telemetry definition reads AMBIENT TEMPERATURE (Celsius), AMBIENT
PRESSURE (millibars), AMBIENT DENSITY (slug/ft^3), AMBIENT WIND VELOCITY (knots),
AMBIENT WIND DIRECTION (degrees), and AIRCRAFT WIND X/Y/Z (knots). KOHLSMAN SETTING
MB is separately read in mbar/hPa; it is not sea-level weather pressure. Ambient
pressure is local at the aircraft, not the QNH setting. Local aircraft-axis wind
components differ from ambient wind, especially after teleport/airflow recovery.

`_manual_recording_loop` captures the first fresh post-unpause sample in
manual_t0_snapshot.telemetry. CSV and the matching initialization sidecar preserve
that recording-start weather. It may differ from weather later at throttle onset.
`start_auto_test_only` loads the selected sidecar. `_restore_reference_weather`
calls reference_weather_metar(snapshot, current global METAR), then the existing
set_global_weather_metar. No pre-event target replaces the selected snapshot.

The builder reads measured ambient_temp, ambient_wind_spd/dir and pressure_alt
(actually MSL feet). It changes GLOB wind and temperature/dewpoint tokens, converts
temperature-observation altitude with feet*0.3048 into &Axxxxx, and preserves the
existing pressure token. Baro setting is intentionally not used as weather QNH.
This means reference ambient pressure/density are NOT independently restored by
the builder. Wind restoration can discard the existing @@@ aloft section.

The Python weather setter forwards ASCII to SetGlobalWeatherMETAR. Native code
selects custom weather, disables dynamic updates, applies the observation, selects
global mode, applies it again and selects global mode again. Success means all
HRESULTs succeeded, not measured convergence. Observation readback is a METAR
string; the actual OAT comes separately from aircraft telemetry.

No second weather setter was found in the engine replay/state-restoration path.
Position, motion, trim, payload/fuel and engine-thermal restoration are separate
operations; they do not directly assign ambient OAT. Scenery/aircraft reload,
weather-layer interpretation/interpolation, retained weather pressure and external
weather providers remain possible contributors. Neither the existing logs nor
offline tests prove an external provider override or identify the exact P3D layer
interpretation responsible for the 22–25 C mismatch.

The observed Acceleration command encoded 16.47 C at approximately 610 m, while
actual Automatic startup OAT was 37.22 C. Manual startup was 16.47 C, with about
15.2 C later at throttle onset. Deceleration Automatic startup was 39.69 C versus
Manual 15.01 C. These are measured mismatches, not acceptable command confirmation.
The exact meteorological reason is not confirmed offline. Numeric ft-to-m conversion
is correct; SDK/runtime interpretation and deployed-DLL identity still need proof.

## Added observation checkpoints

1. Before restoration (cached sample).
2. Immediately after the existing command (acceptance and METAR logged separately).
3. After existing live airflow recovery and re-pause (existing measured sample).
4. Final initialization readiness (existing final measured sample).
5. Last recorded sample before measured throttle differs by at least 2 percentage
   points. This is explicitly labelled, not claimed to be an interpolated onset.

Comparisons always name recording_start_sidecar as the reference phase. Thus
pre-event changes caused by aircraft motion, lapse rate or delayed weather need
interpretation against the full Manual trace, not blind comparison to its first row.
No extra telemetry reads are made inside the recording loop. Invalid diagnostic
inputs and failures are isolated from execution.

MATCHED/NOT MATCHED require finite values, a recent Python cache timestamp and
advancing bridge sequence relative to the prior checkpoint. Otherwise UNVERIFIED.
Native acquisition timing is always unverified. Numeric diagnostic limits are
1 C OAT, 1 mbar pressure/setting, 5% density, 1 kt wind/components and 5 degrees
wind direction. These are diagnostic comparisons, not qualification thresholds
or new recording blocks. Missing data remains missing; no unit guessing/conversion.

## Smallest next correction

First capture these checkpoints in both existing engine workflows on the simulator
PC, alongside current global METAR and active weather-source settings. Establish
whether OAT is wrong immediately, converges late, or changes after positioning or
resume. Only then correct the shared METAR layer/pressure construction or overriding
weather source that the evidence identifies. Preserve throttle-event replay and
RPM tolerances. Do not enable universal initialization or add weather readiness
blocking until measured convergence and missing-data behavior are demonstrated.
