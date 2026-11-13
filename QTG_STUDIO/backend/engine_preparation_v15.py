"""Engine preparation only: no flight-output copying and no shaft/combustion forcing."""
import math
import time

THERMAL_KEYS = (
    'engine_oil_temperature_c_1',
    'engine_cylinder_head_temperature_c_1',
    'engine_carburetor_temperature_c_1',
)
SWITCH_KEYS = (
    'electrical_master_battery', 'engine_master_alternator_1',
    'general_eng_fuel_pump_switch_1', 'engine_fuel_valve_1',
    'general_eng_starter_1',
)
SUPPORT_KEYS = THERMAL_KEYS + SWITCH_KEYS


def finite(value):
    try:
        value = float(value)
        return value if math.isfinite(value) else None
    except (TypeError, ValueError):
        return None


def engine_mismatches(live, reference, rpm_tolerance=80.0, include_transients=True):
    """Bounded startup limit matches the logger (80 RPM); formal limit stays 35 RPM."""
    bad = []
    if finite(live.get('engine_combustion_1')) != 1.0:
        bad.append('combustion is not ON')
    starter = finite(live.get('general_eng_starter_1'))
    if starter is None or starter >= 0.5:
        bad.append('starter is missing or still ON')
    target = finite(reference.get('engine_rpm'))
    if target is None or target < 500:
        bad.append('Manual engine RPM is invalid')
    for key in ('eng_rpms', 'prop_rpms'):
        values = live.get(key) or []
        value = finite(values[0]) if values else None
        if value is None or value < 500 or (target is not None and abs(value-target) > rpm_tolerance):
            bad.append(f'{key}={value}, Manual={target}, allowed difference={rpm_tolerance:g} RPM')
    # Pressures are observed outputs. They are never forced to make this pass.
    for key, floor, fraction in (
        ('engine_fuel_pressure_psi_1', 2.0, 0.20),
        ('engine_manifold_pressure_psi_1', 0.5, 0.03),
    ):
        if key == 'engine_fuel_pressure_psi_1' and not include_transients:
            continue
        ref, value = finite(reference.get(key)), finite(live.get(key))
        if ref is not None:
            if value is None or abs(value-ref) > max(floor, abs(ref)*fraction):
                bad.append(f'{key}={value}, Manual={ref}')
    for key in SWITCH_KEYS:
        ref = finite(reference.get(key))
        if ref is not None:
            value = finite(live.get(key))
            if value is None or (value >= 0.5) != (ref >= 0.5):
                bad.append(f'{key}={value}, Manual={ref}')
    for key in (THERMAL_KEYS if include_transients else ()):
        ref, value = finite(reference.get(key)), finite(live.get(key))
        if ref is not None and (value is None or abs(value-ref) > 5.0):
            bad.append(f'{key}={value}, Manual={ref}')
    return bad


def prepare_engine(reference, sample, hold, configure, auto_start, cancelled,
                   timeout=35.0, stable_seconds=2.0, clock=time.monotonic,
                   pause=time.sleep, progress=lambda message: None, verify_transients=True):
    """Run before timer starts, with bounded startup and a sustained natural-power gate.

    hold must omit RPM, combustion, fuel pressure and thermal-output writes.
    verify_transients=False defers pressure/temperatures to the paused final restore.
    configure runs once AFTER startup. The logger defers thermal/pressure seeds
    to final paused release when verify_transients=False.
    """
    if finite(reference.get('engine_rpm')) is None or float(reference['engine_rpm']) < 500:
        return False, 'Manual t=0 has no valid running-engine RPM'
    if finite(reference.get('engine_combustion_1')) not in (None, 1.0):
        return False, 'This test requires a running-engine Manual reference'
    if finite(reference.get('general_eng_starter_1')) not in (None, 0.0):
        return False, 'Manual t=0 was captured with the starter engaged'
    deadline = clock() + timeout
    live = sample()
    need_start = (finite(live.get('engine_combustion_1')) != 1.0
                  or finite((live.get('eng_rpms') or [None])[0]) is None
                  or float((live.get('eng_rpms') or [0])[0]) < 500)
    if need_start:
        progress('Starting engine; test timer remains 0')
        if not auto_start():
            return False, 'P3D auto-start request failed'
        while True:
            if cancelled():
                return False, 'Engine preparation cancelled'
            if clock() >= deadline:
                return False, 'Engine did not start within preparation timeout'
            hold(starting=True)
            live = sample()
            rpm = finite((live.get('eng_rpms') or [None])[0])
            if finite(live.get('engine_combustion_1')) == 1.0 and rpm is not None and rpm >= 500:
                break
            pause(0.05)
    if cancelled():
        return False, 'Engine preparation cancelled'
    configure()
    progress('Checking natural engine RPM, pressures and systems; timer remains 0')
    good_since = None
    frames = 0
    last = ['no fresh sample']
    while clock() < deadline:
        if cancelled():
            return False, 'Engine preparation cancelled'
        hold(starting=False)
        live = sample()
        last = engine_mismatches(live, reference, include_transients=verify_transients)
        if last:
            good_since, frames = None, 0
        else:
            if good_since is None:
                good_since = clock()
            frames += 1
            if frames >= 3 and clock()-good_since >= stable_seconds:
                return True, 'Natural engine readiness PASS (no RPM/combustion writes)'
        pause(0.05)
    return False, 'Natural engine readiness failed: ' + '; '.join(last)


def configuration_mismatches(live, reference):
    """Measured configuration/weather checks; outputs are never edited."""
    bad=[]
    for live_key, ref_key, tolerance in (
        ('flaps_handle_percent','flaps_handle_percent',0.01),
        ('trailing_edge_flaps_left_percent','trailing_edge_flaps_left_percent',0.01),
        ('trailing_edge_flaps_right_percent','trailing_edge_flaps_right_percent',0.01),
        ('engine_mixture_lever_1','engine_mixture_lever_1',0.005),
        ('engine_propeller_lever_1','engine_propeller_lever_1',0.005),
        ('fuel_tank_selector_1','fuel_tank_selector_1',0.0),
        ('ambient_wind_spd','actual_wind_speed_kt',1.0),
    ):
        ref, value=finite(reference.get(ref_key)),finite(live.get(live_key))
        if ref is not None and (value is None or abs(value-ref)>tolerance):
            bad.append(f'{live_key}={value}, Manual={ref}')
    speed=finite(reference.get('actual_wind_speed_kt'))
    ref,value=finite(reference.get('actual_wind_dir_deg')),finite(live.get('ambient_wind_dir'))
    if speed is not None and speed>1.0 and ref is not None:
        if value is None or abs((value-ref+180)%360-180)>3.0:
            bad.append(f'wind direction={value}, Manual={ref}')
    return bad
