"""Existing worker initialization procedures, separated from the GUI."""

import importlib.util
import math
from pathlib import Path
import sys
import time

from backend.verifier import validate_conditions
from backend.registry import engine_test_policy


def apply_manual_conditions(window,values):
    """Apply settings once while paused, then optionally verify Manual airflow."""
    from backend.airborne_preparation import enabled, prepare
    if enabled(window):
        window._manual_airflow_ready = None
    cfg=validate_conditions(values);sim=window.p3d
    if not sim or not sim.is_connected():raise RuntimeError('Sim Flight Model disconnected.')
    sim.pause_sim()
    if not sim.wait_for_paused(3.):raise RuntimeError('Simulator did not confirm pause.')
    if window.stop_worker_event.is_set():raise RuntimeError('Manual initialization cancelled.')
    from replay_support import reference_weather_metar
    metar=reference_weather_metar({'altitude_msl':cfg['alt_ft'],'oat':cfg['temperature_c']},sim.get_global_weather_metar())
    if metar and not sim.set_global_weather_metar(metar):raise RuntimeError('Initial temperature rejected.')
    if not sim.queue_position_airborne(alt_ft=cfg['alt_ft'],speed_kts=cfg['ias_kt'],pitch_deg=cfg["pitch_deg"],bank_deg=-cfg["bank_deg"],hdg_deg=cfg["heading_deg"],lat=cfg['lat'],lon=cfg['lon'],on_ground=cfg['on_ground']):raise RuntimeError('Initial position rejected.')
    window._wait_for_initial_altitude(cfg['alt_ft'])
    # INITPOSITION and aircraft reload handling can reset power and flap handles.
    # Apply them after position readback, then wait for fresh telemetry; never drive it in flight.
    if not sim.set_initial_throttle(cfg['throttle']):raise RuntimeError('Initial throttle rejected.')
    try: sim.set_initial_flaps(cfg['flaps'])
    except Exception: pass
    checks=[('pressure_alt',cfg['alt_ft'],25),('plane_latitude',cfg['lat'],.001),('plane_longitude',cfg['lon'],.001),('measured_flaps_handle_index',cfg['flaps'],.1),('engine_throttle_percent',cfg['throttle'],1)]
    deadline=time.monotonic()+3.
    while True:
        if window.stop_worker_event.is_set():raise RuntimeError('Manual initialization cancelled.')
        actual=window._refresh_paused_telemetry()
        mismatches=[]
        for key,target,tolerance in checks:
            if key == 'measured_flaps_handle_index' and target == 0:
                continue
            measured=float(actual.get(key,float('nan')))
            if not math.isfinite(measured) or abs(measured-target)>tolerance:
                mismatches.append(f'{key}: requested {target:g}, actual {measured:g}')
        if not mismatches:break
        if time.monotonic()>=deadline:
            hints=['Check physical throttle position and simulator throttle-axis assignments.'] if any('engine_throttle_percent' in item for item in mismatches) else []
            if any('measured_flaps_handle_index' in item for item in mismatches): hints.append('Check physical cockpit flap switch is set to 0 (UP).')
            hint=(' ' + ' '.join(hints)) if hints else ''
            raise RuntimeError('Manual initial-condition mismatch: '+ '; '.join(mismatches)+'.'+hint)
        if any('measured_flaps_handle_index' in item for item in mismatches):
            try: sim.set_initial_flaps(cfg['flaps'])
            except Exception: pass
        window.stop_worker_event.wait(.05)
    if enabled(window):
        actual = prepare(window, cfg, actual)
    return actual


def configure_test_extension(window, reference):
    """Load only an explicitly mapped test adapter, never alter BRUNNER modules."""
    for name,original in getattr(window,'_test_extension_originals',{}).items():
        setattr(window,name,original)
    window._test_extension_originals={}
    folders={'1.f.(1)':'engine_acceleration','1.f.(2)':'engine_deceleration'}
    if reference not in folders:return
    root=(Path(sys._MEIPASS) if getattr(sys,'frozen',False) else Path(__file__).resolve().parent.parent)/'test_suites'
    def read(name,path):
        spec=importlib.util.spec_from_file_location(name,path)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        return module
    policy=read('qtg_engine_test_policy',root/folders[reference]/'procedure.py')
    if policy.TEST_REFERENCE!=reference:raise RuntimeError('Test initialization policy mismatch.')
    if policy.USE_SETTLED_REFERENCE_ALTIMETER:
        adapter=read('qtg_engine_initialization_adapter',root/'_shared/engine_initialization.py')
        window._engine_test_adapter=adapter
        adapter.install(window)


def initialize_mode(window, mode, reference=None):
    from qtg_common_engine import initialize_mode as existing_initialize_mode
    return existing_initialize_mode(window, mode, reference)
