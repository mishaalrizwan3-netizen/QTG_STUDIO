"""Opt-in Manual Acceleration airflow preparation; no control or position writes.

Freshness uses the existing P3D client's advancing receive sequence, not a
wall-clock/cache-copy comparison. Native acquisition timing remains unverified.
"""
import math
import os
import time


def activation_requested(reference, mode, control_method='replay'):
    """Process-local operator opt-in, evaluated for every GUI initialization request."""
    return (os.environ.get('QTG_ACCELERATION_MANUAL_AIRFLOW_PREPARATION') == '1'
            and reference == '1.f.(1)' and mode == 'manual' and control_method == 'replay')


def _progress(window, text):
    base = getattr(window, '_base_module', None)
    signal = getattr(window, 'sig_update_stage', None)
    if base is not None and signal is not None:
        # Use the existing queued, generation-guarded initialization signal.
        signal.emit(base.STAGE_INITIALIZING, text)


def _pause(sim):
    sim.pause_sim()
    if not sim.wait_for_paused(3.):
        raise RuntimeError('pause not confirmed')


def enabled(window):
    context = getattr(window, '_session_context', {})
    return (context.get('acceleration_manual_airflow_preparation') is True
            and context.get('test_reference') == '1.f.(1)'
            and context.get('control_method', 'replay') == 'replay'
            and getattr(window, 'test_mode', context.get('mode')) == 'manual')


def _number(sample, key):
    try:
        value = float(sample[key])
    except (KeyError, TypeError, ValueError):
        raise RuntimeError(f'Manual airflow preparation: unavailable {key}.') from None
    if not math.isfinite(value):
        raise RuntimeError(f'Manual airflow preparation: nonfinite {key}.')
    return value


def _check(sample, plan):
    ias = _number(sample, 'airspeed_indicated')
    altitude = _number(sample, 'pressure_alt')  # Existing bridge channel is MSL feet.
    for name, actual, target, tolerance, unit in (
            ('IAS', ias, plan['ias_kt'], 2., 'kt'),
            ('MSL altitude', altitude, plan['alt_ft'], 25., 'ft')):
        if abs(actual - target) > tolerance:
            raise RuntimeError(f'Manual airflow preparation: excessive {name} deviation: {name} {actual:.2f} {unit}; '
                               f'target {target:.2f} ±{tolerance:g} {unit}. '
                               'Recording blocked. A separately validated pre-recording '
                               'stabilization method may be necessary; no flight corrections applied.')
    wind = math.sqrt(sum(_number(sample, 'aircraft_wind_' + axis + '_kt') ** 2
                         for axis in 'xyz'))
    offset = _number(sample, 'indicated_altitude') - altitude
    ready = (abs(wind - plan['wind_kt']) <= max(.5, plan['wind_kt'] * .1)
             and (abs(offset) <= 10. or abs(offset - plan['offset_ft']) <= 10.))
    return ready, dict(ias_kt=ias, altitude_msl_ft=altitude,
                      indicated_altitude_ft=altitude + offset,
                      local_wind_kt=wind, altimeter_offset_ft=offset)


def prepare(window, cfg, initial, *, clock=time.monotonic):
    """Apply no conditions here: the caller has applied them exactly once."""
    window._manual_airflow_ready = None
    sim = window.p3d
    started = clock()
    consecutive = 0
    last_status = -5.
    report = dict(status='PREPARING', native_acquisition_freshness='UNVERIFIED',
                  physics_clock_progression='UNVERIFIED', running_state='UNKNOWN',
                  pause_transitions=[], unpause_requests=0)
    window._session_context['manual_airflow_preparation'] = report

    def guard():
        # Also run after blocking reads and immediately before every unpause.
        if window.stop_worker_event.is_set():
            raise RuntimeError('Manual airflow preparation cancelled; recording blocked.')
        if not sim.is_connected():
            raise RuntimeError('Manual airflow preparation: P3D disconnected.')
        if clock() - started >= 60.:
            raise RuntimeError('Manual airflow preparation timed out: airflow not recovered '
                               'or altimeter offset not settled. '
                               f'P3D {report["running_state"]}, packet {report.get("receive_sequence")}; '
                               f'IAS {report.get("ias_kt")} / target {plan["ias_kt"]} kt; '
                               f'MSL {report.get("altitude_msl_ft")} / target {plan["alt_ft"]} ft; '
                               f'indicated {report.get("indicated_altitude_ft")} ft; '
                               f'wind {report.get("local_wind_kt")} / target {plan["wind_kt"]} kt; '
                               f'offset {report.get("altimeter_offset_ft")} / target {plan["offset_ft"]} ft. '
                               'Recording blocked.')

    def running():
        guard()
        try:
            value = sim.simulation_running()
            if value is not True and value is not False:
                raise RuntimeError('Pause event did not provide a Boolean state')
        except Exception as exc:
            if report['running_state'] != 'UNKNOWN':
                report['pause_transitions'].append(dict(state='UNKNOWN', elapsed_s=clock() - started))
            report['running_state'] = 'UNKNOWN'
            report['pause_state_error'] = str(exc)
            raise RuntimeError(f'Unable to confirm P3D running state: {exc}. Recording blocked.') from exc
        guard()
        state = 'RUNNING' if value else 'PAUSED'
        if state != report['running_state']:
            report['pause_transitions'].append(dict(state=state, elapsed_s=clock() - started))
        report['running_state'] = state
        return value

    def ensure_running():
        if running():
            return False
        was_running = any(item['state'] == 'RUNNING' for item in report['pause_transitions'])
        message = ('Unexpected P3D pause detected.' if was_running
                   else 'P3D paused before airflow recovery.')
        _progress(window, message + ' Requesting unpause; recording remains blocked.')
        guard()
        report['unpause_requests'] += 1
        try:
            sim.unpause_sim()
        except Exception as exc:
            raise RuntimeError(f'Unexpected P3D pause: unpause request failed: {exc}') from exc
        # Use the existing three-second pause-confirmation bound for acknowledgement.
        # One request per pause episode: never repeatedly transmit while waiting.
        deadline = min(started + 60., clock() + 3.)
        while not running():
            if clock() >= deadline:
                raise RuntimeError('Unexpected P3D pause: unpause did not confirm RUNNING '
                                   'within 3 seconds. Recording blocked.')
            window.stop_worker_event.wait(.05)
        return True

    try:
        from replay_support import reference_altimeter_offset
        if cfg['on_ground']:
            raise RuntimeError('Manual Acceleration airflow preparation requires airborne conditions.')
        offset = reference_altimeter_offset({'telemetry': initial})
        if offset is None or not math.isfinite(offset):
            raise RuntimeError('Manual airflow preparation: unavailable altimeter reference.')
        plan = dict(cfg, wind_kt=_number(initial, 'ambient_wind_spd'), offset_ft=offset)
        if plan['wind_kt'] < 0:
            raise RuntimeError('Manual airflow preparation: invalid ambient wind speed.')
        _progress(window, f'Recovering airflow (not recording). Targets: IAS {plan["ias_kt"]:.2f} kt; '
                  f'MSL altitude {plan["alt_ft"]:.1f} ft; local wind {plan["wind_kt"]:.2f} kt.')
        while True:
            guard()
            if ensure_running():
                consecutive = 0
            sequence = sim.get_telemetry_sequence()
            if sequence < 0:
                raise RuntimeError('Manual airflow preparation: telemetry sequence unavailable.')
            if not sim.request_telemetry_once():
                raise RuntimeError('Manual airflow preparation: telemetry request rejected.')
            sample = sim.wait_for_fresh_telemetry(sequence, 1.5)
            guard()
            if sample is None or sim.get_telemetry_sequence() <= sequence:
                raise RuntimeError('Manual airflow preparation: no advancing P3D telemetry.')
            elapsed = clock() - started
            ready, measured = _check(sample, plan)
            report.update(measured, elapsed_s=elapsed, receive_sequence=sim.get_telemetry_sequence())
            if not running():
                consecutive = 0
                ensure_running()
                # A packet obtained across a pause transition cannot prove stability.
                continue
            _progress(window, f'Recovering airflow — {elapsed:.1f}s (not recording). '
                      f'P3D {report["running_state"]}; packet {report["receive_sequence"]}. '
                      f'IAS {measured["ias_kt"]:.2f} / target {plan["ias_kt"]:.2f} kt; '
                      f'MSL altitude {measured["altitude_msl_ft"]:.1f} / target {plan["alt_ft"]:.1f} ft; '
                      f'indicated altitude {measured["indicated_altitude_ft"]:.1f} ft; '
                      f'local wind {measured["local_wind_kt"]:.2f} / target {plan["wind_kt"]:.2f} kt.')
            if elapsed - last_status >= 5.:
                print(f'[Manual Airflow] {elapsed:.1f}s: {measured}; '
                      f'wind target {plan["wind_kt"]:g} kt, offset target {offset:g} ft.', flush=True)
                last_status = elapsed
            consecutive = consecutive + 1 if ready and elapsed >= 2. else 0
            if consecutive >= 3:
                break
            window.stop_worker_event.wait(.25)
        guard()
        _pause(sim)
        report['final_pause_confirmed'] = True
        final = window._refresh_paused_telemetry()
        if not _check(final, plan)[0] or window.stop_worker_event.is_set():
            raise RuntimeError('Manual airflow preparation invalidated before readiness.')
        window._manual_airflow_ready = dict(plan=plan, conditions=dict(cfg))
        report.update(status='READY', consecutive_samples=consecutive)
        print('[Manual Airflow] Prepared, paused; receive frames advanced. Native timing unverified.', flush=True)
        return final
    except Exception as exc:
        window._manual_airflow_ready = None
        window._prepared = False
        report.update(status='FAILED', reason=str(exc), elapsed_s=clock() - started)
        print('[Manual Airflow] ' + str(exc), flush=True)
        # Failure cleanup is limited to pause; BRUNNER ownership stays with the caller.
        try:
            _pause(sim)
            report['final_pause_confirmed'] = True
        except Exception as cleanup:
            report['final_pause_confirmed'] = False
            report['pause_failure'] = str(cleanup)
            raise RuntimeError(f'{exc} Safety pause failed: {cleanup}') from exc
        raise


def verify_recording_sample(window, sample):
    """Guard both pre-unpause and first recorded frame; never move the aircraft."""
    if not enabled(window):
        return
    state = getattr(window, '_manual_airflow_ready', None)
    try:
        if window.stop_worker_event.is_set() or not window.p3d.is_connected() or not state:
            raise RuntimeError('Manual airflow preparation is not ready; recording blocked.')
        from backend.verifier import validate_conditions
        if validate_conditions(window.manual_initial_conditions) != state['conditions']:
            raise RuntimeError('Manual conditions changed; repeat airflow preparation.')
        if not _check(sample, state['plan'])[0]:
            raise RuntimeError('Manual airflow/altimeter readiness lost; recording blocked.')
    except Exception as exc:
        window._manual_airflow_ready = None
        window._prepared = False
        try:
            _pause(window.p3d)
        except Exception as cleanup:
            raise RuntimeError(f'{exc} Recording blocked; safety pause failed: {cleanup}') from exc
        raise
