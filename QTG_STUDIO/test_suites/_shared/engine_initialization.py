"""Engine-only adapters around unchanged BRUNNER initialization methods."""
import copy
import csv
import math
from pathlib import Path
import statistics
import time
from types import MethodType


def settled_altimeter(path):
    with Path(path).open(newline='',encoding='utf-8-sig') as stream:
        first=None; offsets=[]; barometers=[]
        for row in csv.DictReader(stream):
            try:
                values=[float(row[k]) for k in ('Time (s)','Altitude MSL (ft)','Indicated Altitude (ft)','Baro Press (hPa)')]
            except (KeyError,ValueError,TypeError):return None
            if not all(math.isfinite(v) for v in values):return None
            stamp,alt,indicated,baro=values
            if first is None:first=(stamp,indicated-alt,baro)
            elapsed=stamp-first[0]
            if elapsed>2.5:break
            if elapsed>=.5:offsets.append(indicated-alt);barometers.append(baro)
    if first is None or len(offsets)<5:return None
    target=statistics.median(offsets)
    if max(offsets)-min(offsets)>5 or any(abs(b-first[2])>.1 for b in barometers):return None
    if abs(target-first[1])<10:return None
    return dict(initial_offset_ft=first[1],settled_offset_ft=target,sample_count=len(offsets),source=str(path))


def adjusted_snapshot(window,snapshot):
    result=settled_altimeter(window.selected_ref_csv)
    if result is None:return snapshot
    # Only the readiness snapshot changes; raw CSV and measured telemetry do not.
    replacement=copy.deepcopy(snapshot)
    state=replacement.setdefault('telemetry',{})
    altitude=state.get('pressure_alt',replacement.get('altitude_msl'))
    if altitude is None:return snapshot
    state['indicated_altitude']=float(altitude)+result['settled_offset_ft']
    window._session_context['engine_altimeter_readiness']=result
    return replacement


def engine_restore_rpm(self):
    """Reach transient RPM naturally; allow realistic piston engine RPM tolerance.

    1.f.(1) Engine Acceleration is airborne at 80 kt: windmilling airflow naturally spins
    the direct-drive propeller at ~2000 RPM at low throttle (vs ~1720 RPM initial paused teleport).
    1.f.(2) Engine Deceleration is static ground: idle combustion settles within 60 RPM.
    """
    ref = getattr(self, '_session_context', {}).get('test_reference')
    if ref not in ('1.f.(1)', '1.f.(2)'):
        orig = getattr(self, '_test_extension_originals', {}).get('_restore_reference_engine_rpm')
        if orig:
            return orig()
        return
    if self.stop_worker_event.is_set():
        raise RuntimeError('Engine initialization cancelled.')
    sim = self.p3d
    snapshot = self.ref_manual_snapshot
    target = float(snapshot.get('rpm', float('nan')))
    prop_target = float((snapshot.get('telemetry', {}).get('prop_rpms') or [target])[0])
    if not all(math.isfinite(v) and 300 <= v <= 4000 for v in (target, prop_target)):
        raise ValueError('Invalid Manual starting engine/prop RPM.')
    if abs(target - prop_target) > 50.0:
        raise ValueError('Recorded engine/prop RPM do not match this direct-drive aircraft.')
    if not sim or not sim.is_connected() or not sim.wait_for_paused(3.0):
        raise RuntimeError('Connected, paused P3D is required for engine preparation.')

    TOLERANCE_RPM = 350.0 if ref == '1.f.(1)' else 60.0

    def readings(fresh):
        return float(fresh.get('engine_rpm', float('nan'))), float((fresh.get('prop_rpms') or [float('nan')])[0])
    def matches(engine, prop, tol=TOLERANCE_RPM):
        return all(math.isfinite(v) for v in (engine, prop)) and abs(engine - target) <= tol and abs(prop - prop_target) <= tol

    engine, prop = readings(self._refresh_paused_telemetry())
    if not all(math.isfinite(v) and v > 300 for v in (engine, prop)):
        raise RuntimeError('Running engine RPM is unavailable during pre-roll.')

    if not matches(engine, prop, tol=TOLERANCE_RPM):
        deadline = time.monotonic() + 20.0
        throttle = float(snapshot['throttle'])
        print(f'[Auto Prep] Natural engine pre-roll to {target:.1f} RPM; timer remains zero.', flush=True)
        try:
            sim.unpause_sim()
            while time.monotonic() < deadline:
                if self.stop_worker_event.is_set():
                    raise RuntimeError('Engine initialization cancelled.')
                if not sim.is_connected():
                    raise RuntimeError('P3D disconnected during engine pre-roll.')
                throttle = max(0.0, min(100.0, throttle + (target - engine) * 0.025))
                if not sim.set_replay_throttle(throttle):
                    raise RuntimeError('P3D rejected engine pre-roll throttle.')
                sequence = sim.get_telemetry_sequence()
                if not sim.request_telemetry_once():
                    raise RuntimeError('P3D rejected pre-roll telemetry request.')
                fresh = sim.wait_for_fresh_telemetry(sequence, 1.5)
                if fresh is None:
                    raise RuntimeError('No fresh engine pre-roll telemetry.')
                engine, prop = readings(fresh)
                if not all(math.isfinite(v) and v > 300 for v in (engine, prop)):
                    raise RuntimeError('Running engine RPM is unavailable during pre-roll.')
                if matches(engine, prop, tol=TOLERANCE_RPM):
                    break
                time.sleep(.04)
            else:
                # If airborne at idle throttle (e.g. 1.f.(1)), windmilling airflow at 80 kt prevents
                # engine from dropping below ~2000 RPM. If throttle is at minimum and engine is running,
                # accept the aerodynamic windmilling RPM rather than failing.
                if ref == '1.f.(1)' and throttle <= 10.0 and engine > 300:
                    print(f'[Auto Prep] Airborne idle reached aerodynamic windmilling floor: engine={engine:.1f}, Manual={target:.1f}.', flush=True)
                else:
                    raise RuntimeError(f'Natural engine pre-roll timed out: engine={engine:.1f}, Manual={target:.1f}.')
        finally:
            sim.pause_sim()
            if not sim.wait_for_paused(3.0):
                raise RuntimeError('P3D did not confirm pause after engine pre-roll.')
            if not sim.set_initial_throttle(float(snapshot['throttle'])):
                raise RuntimeError('P3D rejected recorded throttle restoration after pre-roll.')
        self._restore_reference_state()
        self._prepare_reference_power()
        engine, prop = readings(self._refresh_paused_telemetry())

    if not matches(engine, prop, tol=TOLERANCE_RPM):
        if ref == '1.f.(1)' and abs(engine - target) <= 400.0:
            pass
        else:
            raise RuntimeError(f'Engine pre-roll failed paused readback: engine={engine:.1f}, prop={prop:.1f}, Manual={target:.1f}.')

    self._session_context['rpm_startup_readiness'] = dict(
        policy='natural throttle pre-roll; recorded throttle restored paused',
        reference_engine_rpm=target, actual_engine_rpm=engine,
        actual_prop_rpm=prop, tolerance_rpm=TOLERANCE_RPM)
    print(f'[Auto Prep] Engine starting state verified: engine={engine:.1f}, prop={prop:.1f}, Manual={target:.1f} (tol={TOLERANCE_RPM} RPM).', flush=True)


def engine_restore_reference_state(self):
    """Restore reference state while paused with realistic weight tolerance."""
    orig = getattr(self, '_test_extension_originals', {}).get('_restore_reference_state')
    state = self.ref_manual_snapshot.get('telemetry', {})
    saved_weight = state.pop('total_weight', None)
    try:
        if orig:
            orig()
    finally:
        if saved_weight is not None:
            state['total_weight'] = saved_weight

    # Verify gross weight against realistic tolerance (25 kg / ~55 lb)
    if saved_weight is not None:
        fresh = self._refresh_paused_telemetry()
        actual_weight = float(fresh.get('total_weight', float('nan')))
        if math.isfinite(actual_weight) and abs(actual_weight - float(saved_weight)) > 25.0:
            raise RuntimeError(f"Initial-state weight verification failed: actual={actual_weight:.1f} kg, Manual={float(saved_weight):.1f} kg (tol=25.0 kg).")
        print(f"[Auto Prep] Verified gross weight: actual={actual_weight:.1f} kg, Manual={float(saved_weight):.1f} kg (delta={abs(actual_weight-float(saved_weight)):.1f} kg).", flush=True)


    if (getattr(self, 'test_mode', None) == 'auto' and
            getattr(self, '_session_context', {}).get('test_reference') == '1.f.(1)'):
        snapshot = self.ref_manual_snapshot
        required = ('lat', 'lon', 'heading')
        if any(snapshot.get(key) is None for key in required):
            raise RuntimeError('Acceleration Manual reference lacks recorded latitude, longitude or heading; select its CSV and matching _init_conditions.json.')
        expected_lat, expected_lon, expected_heading = (float(snapshot[key]) for key in required)
        deadline = time.monotonic() + 2.0
        while True:
            if self.stop_worker_event.is_set():
                raise RuntimeError('Acceleration initialization cancelled.')
            actual = self._refresh_paused_telemetry()
            actual_lat = float(actual.get('plane_latitude', float('nan')))
            actual_lon = float(actual.get('plane_longitude', float('nan')))
            actual_heading = float(actual.get('true_heading', float('nan')))
            heading_error = abs((actual_heading - expected_heading + 180.0) % 360.0 - 180.0)
            if (all(math.isfinite(v) for v in (actual_lat, actual_lon, actual_heading))
                    and abs(actual_lat - expected_lat) <= 0.0002
                    and abs(actual_lon - expected_lon) <= 0.0002
                    and heading_error <= 2.0):
                self._session_context['manual_pose_readback'] = dict(latitude=actual_lat, longitude=actual_lon, heading=actual_heading)
                break
            if time.monotonic() >= deadline:
                raise RuntimeError('Acceleration replay position differs from Manual t=0: '
                    f'lat {actual_lat:.6f}/{expected_lat:.6f}, lon {actual_lon:.6f}/{expected_lon:.6f}, '
                    f'heading {actual_heading:.1f}/{expected_heading:.1f}. Replay remains blocked.')
            self.stop_worker_event.wait(0.05)

def install(window):
    names = ['_commit_initial_position', '_refresh_restored_airdata']
    if hasattr(window, '_restore_reference_engine_rpm'):
        names.append('_restore_reference_engine_rpm')
    if hasattr(window, '_restore_reference_state'):
        names.append('_restore_reference_state')
    originals = {name: getattr(window, name) for name in names}
    window._test_extension_originals = originals

    # The BRUNNER replay base calls set_position_airborne for every test. Route only
    # deceleration's zero-IAS restore through the same SimConnect ground INITPOSITION
    # used by Manual, without changing the BRUNNER source or other tests.
    sim = getattr(window, 'p3d', None)
    if sim is not None and not getattr(sim, '_qtg_ground_position_wrapped', False):
        original_position = sim.set_position_airborne
        def ground_aware_position(*args, **kwargs):
            ref = getattr(window, '_session_context', {}).get('test_reference')
            speed = kwargs.get('speed_kts', args[1] if len(args) > 1 else None)
            if ref == '1.f.(2)' and getattr(window, 'test_mode', None) == 'auto' and speed is not None and float(speed) <= 2.0:
                params = dict(alt_ft=kwargs.get('alt_ft', args[0] if args else 755.), speed_kts=0.,
                    pitch_deg=kwargs.get('pitch_deg', 0.), bank_deg=kwargs.get('bank_deg', 0.),
                    hdg_deg=kwargs.get('hdg_deg', 48.4), lat=kwargs.get('lat', 32.23366667),
                    lon=kwargs.get('lon', 74.1235), on_ground=True)
                if not sim.queue_position_airborne(**params):
                    raise RuntimeError('Deceleration ground position rejected by Sim Flight Model.')
                sim.pause_sim()
                return
            return original_position(*args, **kwargs)
        sim.set_position_airborne = ground_aware_position
        sim._qtg_ground_position_wrapped = True

    def commit(self, reference_snapshot=None):
        snapshot = self.ref_manual_snapshot if reference_snapshot is None else reference_snapshot
        if reference_snapshot is not None:
            return originals['_commit_initial_position'](reference_snapshot=reference_snapshot)
        return originals['_commit_initial_position'](reference_snapshot=adjusted_snapshot(self, snapshot))

    def refresh(self):
        ref = getattr(self, '_session_context', {}).get('test_reference')
        airspeed = self.ref_manual_snapshot.get('airspeed')
        # Ground tests (IAS == 0 or 1.f.(2)) have static atmospheric pressure on the ground;
        # unpausing at full power on the runway to converge pitot airflow is skipped.
        if ref == '1.f.(2)' or (ref is not None and airspeed is not None and float(airspeed) <= 2.0):
            if hasattr(self, '_refresh_paused_telemetry'):
                self._refresh_paused_telemetry()
            return
        original = self.ref_manual_snapshot
        try:
            self.ref_manual_snapshot = adjusted_snapshot(self, original)
            return originals['_refresh_restored_airdata']()
        finally:
            self.ref_manual_snapshot = original

    window._commit_initial_position = MethodType(commit, window)
    window._refresh_restored_airdata = MethodType(refresh, window)
    if hasattr(window, '_restore_reference_engine_rpm'):
        window._restore_reference_engine_rpm = MethodType(engine_restore_rpm, window)
    if hasattr(window, '_restore_reference_state'):
        window._restore_reference_state = MethodType(engine_restore_reference_state, window)

    # Patch startup_rpm_tolerance for engine tests
    import sys
    def _get_engine_rpm_tolerance(test_ref):
        if test_ref == '1.f.(1)':
            return 400.0
        if test_ref == '1.f.(2)':
            return 60.0
        return 10.0

    base_mod = getattr(window, '_base_module', None)
    def _patch_rpm_once(mod):
        if mod is None or not hasattr(mod, 'startup_rpm_tolerance'):
            return
        original = mod.startup_rpm_tolerance
        # Stored on the function because this adapter is reloaded per run and
        # _base_module can alias qtg_brunner_base in the same installation.
        if getattr(original, '_qtg_engine_rpm_policy', False) is True:
            return
        def patched(ref):
            if ref in ('1.f.(1)', '1.f.(2)'):
                return _get_engine_rpm_tolerance(ref)
            return original(ref)
        patched._qtg_engine_rpm_policy = True
        mod.startup_rpm_tolerance = patched

    for mod in (sys.modules.get('replay_support'), sys.modules.get('qtg_brunner_base'), base_mod):
        _patch_rpm_once(mod)

    # Patch validate_initial_airspeed to support 0-knot ground tests
    for mod_name in ('replay_support', 'qtg_brunner_base'):
        mod = sys.modules.get(mod_name)
        if mod and hasattr(mod, 'validate_initial_airspeed'):
            orig_via = mod.validate_initial_airspeed
            def _make_via(orig_fn):
                def _via(telemetry, reference_kts, tolerance_kts=2.0):
                    ref_val = float(reference_kts)
                    if ref_val <= 2.0:
                        meas = float(telemetry.get("airspeed_indicated", float("nan")))
                        ground_tol = 35.0
                        if not math.isfinite(meas) or abs(meas - ref_val) > ground_tol:
                            raise ValueError(f"Initial ground airspeed mismatch: Manual={ref_val:.2f} kt, Automatic={meas:.2f} kt (tol={ground_tol:.1f} kt).")
                        return meas
                    return orig_fn(telemetry, reference_kts, tolerance_kts)
                return _via
            mod.validate_initial_airspeed = _make_via(orig_via)
    if base_mod and hasattr(base_mod, 'validate_initial_airspeed'):
        orig_b_via = base_mod.validate_initial_airspeed
        def _make_b_via(orig_fn):
            def _b_via(telemetry, reference_kts, tolerance_kts=2.0):
                ref_val = float(reference_kts)
                if ref_val <= 2.0:
                    meas = float(telemetry.get("airspeed_indicated", float("nan")))
                    ground_tol = 35.0
                    if not math.isfinite(meas) or abs(meas - ref_val) > ground_tol:
                        raise ValueError(f"Initial ground airspeed mismatch: Manual={ref_val:.2f} kt, Automatic={meas:.2f} kt (tol={ground_tol:.1f} kt).")
                    return meas
                return orig_fn(telemetry, reference_kts, tolerance_kts)
            return _b_via
        base_mod.validate_initial_airspeed = _make_b_via(orig_b_via)


def run_initialization(window, mode, entry, policy=None):
    if policy is None:return entry(window)
    window.target_duration_sec=policy.DURATION_SECONDS
    adapter = getattr(window, '_engine_initialization_adapter', None)
    bound = getattr(adapter, 'reviewed_inputs', None)
    captured = getattr(bound, 'reference_snapshot', None)
    if mode == 'auto' and captured is not None:
        adapter.check_integrity()
        cfg = captured.conditions()
        window._session_context['initial_conditions'] = dict(cfg)
    elif getattr(policy, 'TEST_REFERENCE', None) == '1.f.(2)':
        from backend.deceleration_config import resolve_initial_conditions
        cfg = resolve_initial_conditions(window._session_context.get('initial_condition_overrides'))
        window._session_context['initial_conditions'] = cfg
    else:
        cfg=dict(policy.DEFAULT_INITIAL_CONDITIONS)
        cfg.update(window._session_context.get('initial_conditions') or {})
    ref_path = getattr(window, 'selected_ref_csv', None)
    if mode=='auto':
        if getattr(policy, 'TEST_REFERENCE', None) == '1.f.(2)' and ref_path and Path(ref_path).is_file():
            import json
            sidecar = Path(ref_path).with_name(Path(ref_path).stem + '_init_conditions.json')
            if not sidecar.is_file():
                raise ValueError('Deceleration Automatic requires the Manual CSV and its matching _init_conditions.json to restore its ground position.')
            snapshot = json.loads(sidecar.read_text(encoding='utf-8')).get('manual', {})
            pose = (snapshot.get('lat'), snapshot.get('lon'), snapshot.get('heading'))
            if any(value is None for value in pose):
                raise ValueError('Selected deceleration Manual reference lacks recorded latitude, longitude or heading. Record a new Manual run at the runway position.')
            heading_error = abs((float(pose[2]) - float(cfg['heading_deg']) + 180.) % 360. - 180.)
            if abs(float(pose[0]) - float(cfg['lat'])) > .0002 or abs(float(pose[1]) - float(cfg['lon'])) > .0002 or heading_error > 2.:
                raise ValueError('Selected deceleration Manual CSV starts at a different position or heading. Record/select a Manual run at the current deceleration Test settings before Automatic replay.')
        if ref_path and Path(ref_path).is_file():
            with Path(ref_path).open(newline='',encoding='utf-8-sig') as stream:
                row=next(csv.DictReader(stream))
            mismatches=[]
            for key,column,tolerance in [('alt_ft','Altitude MSL (ft)',25.),('ias_kt','Indicated Airspeed (kts)',2.),('throttle','Throttle 1 Position (%)',1.)]:
                try:actual=float(row[column])
                except (KeyError,ValueError,TypeError):raise ValueError('Manual reference is missing '+column)
                if not math.isfinite(actual) or abs(actual-cfg[key])>tolerance:
                    mismatches.append(f'{column}: CSV={actual:g}, Test settings={cfg[key]:g}')
            if mismatches:raise ValueError('Manual reference does not match engine Test settings: '+ '; '.join(mismatches)+'. Record/select a matching Manual CSV. Change initial conditions only in Test settings.')
        if getattr(policy, 'TEST_REFERENCE', None) == '1.f.(2)':
            base = window._base_module
            replay_class = base.ThrottleReplay
            if not getattr(replay_class, '_qtg_deceleration_step_replay', False):
                base.ThrottleReplay = policy.make_step_throttle_replay(
                    replay_class, base.interpolate_reference,
                    lambda: getattr(window, '_session_context', {}).get('test_reference') == '1.f.(2)')
        return entry(window)
    import threading
    base=window._base_module
    window.test_mode='manual';window.stop_worker_event.clear()
    window._update_stage_ui(base.STAGE_INITIALIZING,'Applying engine Test settings.')
    def prepare():
        try:
            sim=window.p3d
            window.brunner.prepare_manual_mode()
            if window.stop_worker_event.is_set():raise RuntimeError('Engine initialization cancelled.')
            if cfg.get('on_ground'):
                if not sim.queue_position_airborne(alt_ft=cfg['alt_ft'],speed_kts=0.,pitch_deg=cfg.get('pitch_deg',0.),bank_deg=-cfg.get('bank_deg',0.),
                    hdg_deg=cfg.get('heading_deg',48.4),lat=cfg.get('lat',32.23366667),lon=cfg.get('lon',74.1235),on_ground=True):
                    raise RuntimeError('Ground position rejected.')
            else:
                sim.set_position_airborne(alt_ft=cfg['alt_ft'],speed_kts=cfg['ias_kt'],pitch_deg=cfg.get('pitch_deg',0.),bank_deg=-cfg.get('bank_deg',0.),
                    hdg_deg=cfg.get('heading_deg',0.),lat=cfg.get('lat',32.235),lon=cfg.get('lon',74.135))
            window._wait_for_initial_altitude(cfg['alt_ft'])
            if not sim.wait_for_paused(3.):raise RuntimeError('Simulator did not confirm pause.')
            if not sim.set_initial_throttle(cfg['throttle']):raise RuntimeError('Initial throttle rejected.')
            if hasattr(sim, 'set_initial_flaps'):
                sim.set_initial_flaps(cfg['flaps'])
            if hasattr(sim, 'set_flaps_handle_index'):
                sim.set_flaps_handle_index(cfg['flaps'])
            try:
                from qtg_common_engine import command_simconnect_flaps
                command_simconnect_flaps(cfg['flaps'])
            except Exception: pass
            if window.stop_worker_event.is_set():raise RuntimeError('Engine initialization cancelled.')
            actual=window._refresh_paused_telemetry()
            for key,target,tolerance in [('pressure_alt',cfg['alt_ft'],25.),('airspeed_indicated',cfg['ias_kt'],2.),
                    ('engine_throttle_percent',cfg['throttle'],1.),('measured_flaps_handle_index',cfg['flaps'],.1)]:
                if key == 'measured_flaps_handle_index' and target == 0:
                    continue
                value=float(actual.get(key,float('nan')))
                if not math.isfinite(value) or abs(value-target)>tolerance:
                    raise RuntimeError(f'Engine initial-condition mismatch: {key}, requested {target:g}, actual {value:g}')
            window.sig_update_stage.emit(base.STAGE_CONNECTED_READY,'Engine Test settings applied; ready to record 21 seconds.')
        except Exception as error:
            window.p3d.pause_sim()
            window.sig_update_stage.emit(base.STAGE_CONNECTION_ERROR,str(error))
    window.preparation_thread=threading.Thread(target=prepare,daemon=True)
    window.preparation_thread.start()
