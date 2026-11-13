"""Nonblocking weather evidence: cached reads only, never sets or gates weather.

MATCHED means numeric agreement with a recorded target under diagnostic limits;
it is not proof of native acquisition freshness or stabilized weather.
"""
import json
import math
import time


CHANNELS = {
    'ambient_temp': ('deg C', 1.),
    'ambient_pressure_mb': ('mbar', 1.),
    'baro_pressure': ('hPa (altimeter setting)', 1.),
    'ambient_density_slug_ft3': ('slug/ft^3', None),
    'ambient_wind_spd': ('kt', 1.), 'ambient_wind_dir': ('deg', 5.),
    'aircraft_wind_x_kt': ('kt', 1.), 'aircraft_wind_y_kt': ('kt', 1.),
    'aircraft_wind_z_kt': ('kt', 1.),
}


def finite(value):
    return isinstance(value, (int,float)) and math.isfinite(value)


def compare_weather(reference, measured, *, timing_supported):
    channels = {}
    for key,(unit,limit) in CHANNELS.items():
        target,actual=reference.get(key),measured.get(key)
        tolerance = abs(target)*.05 if limit is None and finite(target) else limit
        status='UNVERIFIED'
        delta=None
        if finite(target) and finite(actual) and timing_supported:
            delta=actual-target
            if key=='ambient_wind_dir': delta=(delta+180)%360-180
            status='MATCHED' if abs(delta)<=tolerance else 'NOT MATCHED'
        channels[key]=dict(reference=target if finite(target) else None,
            measured=actual if finite(actual) else None,unit=unit,
            difference=delta,diagnostic_tolerance=tolerance,status=status)
    statuses=[item['status'] for item in channels.values()]
    overall='NOT MATCHED' if 'NOT MATCHED' in statuses else 'UNVERIFIED' if 'UNVERIFIED' in statuses else 'MATCHED'
    return dict(status=overall,channels=channels,
        interpretation='Numeric comparison only; no runtime acceptance change. Native acquisition freshness remains UNVERIFIED.')


def observe(window, phase, telemetry=None, **command_evidence):
    """Best effort hook. Every exception is isolated by the caller."""
    if getattr(window,'_session_context',{}).get('test_reference') not in ('1.f.(1)','1.f.(2)'):
        return
    if phase=='check_pre_event':
        if getattr(window,'_weather_pre_event_logged',False) or not window.p3d_tele_buf:
            return
        actual=telemetry.get('engine_throttle_percent')
        if not finite(actual) or abs(actual-float(window.ref_throt_pos[0]))<2.:
            return
        window._weather_pre_event_logged=True
        telemetry=window.p3d_tele_buf[-1]
        phase='last_recorded_sample_before_measured_throttle_change'
    if phase=='before_restoration':
        window._weather_observations=[]
        window._weather_previous_sequence=None
        window._weather_pre_event_logged=False
    measured=telemetry if telemetry is not None else window.p3d.get_telemetry()
    now=time.time()
    stamp=measured.get('last_updated')
    sequence=measured.get('bridge_telemetry_sequence')
    previous=getattr(window,'_weather_previous_sequence',None)
    timing=(finite(stamp) and 0<=now-stamp<=1. and type(sequence) is int and sequence>0
            and (previous is None or sequence>previous))
    reference=window.ref_manual_snapshot.get('telemetry',{})
    result=compare_weather(reference,measured,timing_supported=timing)
    result.update(phase=phase,observed_wall_seconds=now,cache_copy_wall_seconds=stamp if finite(stamp) else None,
        bridge_sequence=sequence if type(sequence) is int else None,
        reference_source=getattr(window,'selected_ref_csv',None),reference_phase='recording_start_sidecar',
        native_acquisition_timestamp=None,cache_progression_supported=timing,
        location={key:measured.get(key) for key in ('plane_latitude','plane_longitude','pressure_alt')},
        reference_location={key:reference.get(key) for key in ('plane_latitude','plane_longitude','pressure_alt')},
        command_evidence=command_evidence)
    if type(sequence) is int: window._weather_previous_sequence=sequence
    observations=list(getattr(window,'_weather_observations',[]))
    observations.append(result)
    window._weather_observations=observations[-32:]
    window._session_context['weather_diagnostics']=window._weather_observations
    print('[Weather diagnostic] '+json.dumps(result,allow_nan=False,default=str),flush=True)
