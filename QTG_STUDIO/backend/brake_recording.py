"""Optional recording-clock evidence and offline parking-brake alignment only.

No simulator client, actuator interface, recording command or readiness decision.
"""
import csv
import ctypes
import hashlib
import json
import math
import os
from pathlib import Path
import time

FLAG = 'QTG_DECELERATION_BRAKE_CAPTURE'


def number(value):
    try:
        result = float(value)
        return result if math.isfinite(result) else None
    except (TypeError, ValueError):
        return None


def eligible(context):
    return (context.get('test_reference') == '1.f.(2)' and context.get('mode') in ('manual', 'auto')
            and context.get('control_method', 'replay') == 'replay')


def qpc_mapping():
    """Verify Python perf_counter and native QPC share a domain on this host."""
    if os.name != 'nt':
        return dict(verified=False, reason='Windows QPC unavailable')
    kernel = ctypes.WinDLL('kernel32', use_last_error=True)
    ticks, frequency = ctypes.c_longlong(), ctypes.c_longlong()
    kernel.QueryPerformanceCounter.argtypes = [ctypes.POINTER(ctypes.c_longlong)]
    kernel.QueryPerformanceFrequency.argtypes = [ctypes.POINTER(ctypes.c_longlong)]
    before = time.perf_counter()
    ok = kernel.QueryPerformanceCounter(ctypes.byref(ticks))
    after = time.perf_counter()
    ok = ok and kernel.QueryPerformanceFrequency(ctypes.byref(frequency)) and frequency.value > 0
    native = ticks.value/frequency.value if ok else None
    epsilon = 2*math.ulp(after)
    return dict(verified=bool(ok and before-epsilon <= native <= after+epsilon),
                frequency=frequency.value, python_before=before, python_after=after,
                qpc_seconds=native, clock='Windows QPC / Python perf_counter')


def mark_start(window, t0):
    """Only publish metadata in memory; called after the existing t0 assignment."""
    context = getattr(window, '_session_context', {})
    if os.environ.get(FLAG) != '1' or not eligible(context):
        return
    window._parking_brake_recording_anchor = dict(
        recording_id=f'{os.getpid()}:{float(t0).hex()}', t0_perf_counter=t0,
        test_reference='1.f.(2)', mode=context['mode'],
        mapping=dict(getattr(window, '_parking_brake_clock_mapping', None) or qpc_mapping()),
        provenance='existing recording-loop t0; no timer replacement')
    context['parking_brake_recording_anchor'] = dict(window._parking_brake_recording_anchor)


def install_clock_marker(window):
    """Preload and check the clock before hardware startup, not during recording."""
    if os.environ.get(FLAG) == '1':
        window._parking_brake_clock_mapping = qpc_mapping()
        window._parking_brake_clock_marker = lambda t0: mark_start(window,t0)


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def samples_from_csv(path, anchor):
    """Accept only complete paired native responses; never read wheel brakes."""
    pairs, samples, issues, barriers = {}, [], [], []
    mapping = anchor.get('mapping', {})
    if not mapping.get('verified') or number(anchor.get('t0_perf_counter')) is None:
        return [], ['Recording/native QPC mapping unverified']
    previous = None
    with Path(path).open(newline='', encoding='utf-8-sig') as stream:
        for row in csv.DictReader(stream):
            if row.get('simvar') not in ('BRAKE PARKING POSITION', 'SIM TIME'):
                continue
            if row.get('status') != 'NEW_CORRELATED_RESPONSE':
                issues.append('Invalid/missing native response: '+str(row.get('status')))
                freq, ticks = number(row.get('qpc_frequency')), number(row.get('receipt_qpc_ticks'))
                if freq == mapping.get('frequency') and freq and ticks is not None:
                    barriers.append(dict(time_s=ticks/freq-anchor['t0_perf_counter'], parking_brake=None,
                                         reason='Invalid paired native measurement'))
                continue
            # Pair by exact callback identity, not nearest timestamp or cached value.
            identity = tuple(row.get(k) for k in ('pid', 'connection_generation', 'request_id',
                'channel_sequence', 'receipt_qpc_ticks', 'qpc_frequency'))
            block = pairs.setdefault(identity, {})
            if row['simvar'] in block:
                issues.append('Duplicate native channel response'); continue
            block[row['simvar']] = row
            if len(block) != 2:
                continue
            values = [number(block[key].get('value')) for key in ('BRAKE PARKING POSITION','SIM TIME')]
            freq, ticks = number(row.get('qpc_frequency')), number(row.get('receipt_qpc_ticks'))
            age = number(block['BRAKE PARKING POSITION'].get('request_to_receipt_s'))
            if (values[0] not in (0.,1.) or values[1] is None or values[1] < 0
                    or freq != mapping.get('frequency') or not freq or ticks is None
                    or age is None or not 0 <= age <= 2.):
                return [], issues+['Invalid native pair or QPC frequency; alignment unavailable']
            stamp = ticks/freq
            if previous is not None and (identity[0:2] != previous[0:2]
                    or int(identity[2]) <= int(previous[2]) or int(identity[3]) <= int(previous[3])
                    or stamp <= samples[-1]['receipt_perf_counter'] or values[1] < samples[-1]['sim_time_s']):
                issues.append('Native session/clock/sequence changed; subsequent alignment rejected')
                return [], issues
            previous = identity
            samples.append(dict(time_s=stamp-anchor['t0_perf_counter'], parking_brake=int(values[0]),
                sim_time_s=values[1], request_id=int(identity[2]), sequence=int(identity[3]),
                receipt_perf_counter=stamp, request_to_receipt_s=age,
                freshness='new correlated native response; internal acquisition time unavailable'))
    if any(len(pair)!=2 for pair in pairs.values()):
        issues.append('Incomplete paired responses present')
    cutoff = number(anchor.get('capture_end_perf_counter'))
    if cutoff is not None:
        barriers.append(dict(time_s=cutoff-anchor['t0_perf_counter'],parking_brake=None,
                             reason='Observer stopped/disconnected; subsequent brake state unavailable'))
    return sorted(samples+barriers,key=lambda s:s['time_s']), issues


def at_time(samples, time_s, max_age_s, *, strictly_before=False):
    candidates = [s for s in samples if s['time_s'] < time_s or (not strictly_before and s['time_s'] == time_s)]
    if not candidates:
        return dict(status='UNAVAILABLE', value=None, reason='No prior paired native observation')
    sample = candidates[-1]
    if sample['parking_brake'] is None:
        return dict(status='UNAVAILABLE',value=None,reason=sample['reason'])
    gap = time_s-sample['time_s']
    age_bound = gap+sample['request_to_receipt_s']
    if age_bound > max_age_s:
        return dict(status='UNAVAILABLE', value=None, gap_s=gap, response_age_bound_s=age_bound,
                    reason='Prior response/round-trip too old for diagnostic alignment')
    return dict(status='AVAILABLE_NATIVE_RESPONSE', value=sample['parking_brake'],
        sample_time_s=sample['time_s'], gap_s=gap, sim_time_s=sample['sim_time_s'],
        request_id=sample['request_id'], sequence=sample['sequence'], freshness=sample['freshness'],
        response_age_bound_s=age_bound)


def align_capture(native_csv, anchor, qtg_csv, output, *, max_age_s):
    """New informational files only; original CSV and onset calculation unchanged."""
    if number(max_age_s) is None or max_age_s <= 0:
        raise ValueError('Positive diagnostic age criterion required')
    from backend.manual_reference_preflight import analyze_reference
    metadata = json.loads(Path(qtg_csv).with_suffix('.qtg_test.json').read_text(encoding='utf-8-sig'))
    if (metadata.get('test_reference') != '1.f.(2)' or metadata.get('mode') != anchor.get('mode')
            or (metadata.get('parking_brake_recording_anchor') or {}).get('recording_id') != anchor.get('recording_id')):
        raise ValueError('QTG CSV metadata does not match this recording clock anchor')
    samples, issues = samples_from_csv(native_csv, anchor)
    event = analyze_reference(qtg_csv, mode='deceleration').get('event_time_s')
    with Path(qtg_csv).open(newline='', encoding='utf-8-sig') as stream:
        rows = list(csv.DictReader(stream))
    output = Path(output)
    # Exclusive creation prevents accidental overwrite, including repeated alignment.
    aligned = output/'parking_brake_aligned.csv'
    fields = ['Time (s)','Throttle 1 Position (%)','Ground Speed (kt)', 'parking_brake',
              'status','native_sample_time_s','gap_s','sim_time_s','request_id','phase']
    with aligned.open('x', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream,fieldnames=fields); writer.writeheader()
        for row in rows:
            stamp = number(row.get('Time (s)'))
            if stamp is None:
                issues.append('Invalid QTG recording time'); continue
            value = at_time(samples,stamp,max_age_s)
            writer.writerow(dict(zip(fields, [stamp,row.get(fields[1]),row.get(fields[2]),value['value'],
                value['status'],value.get('sample_time_s'),value.get('gap_s'),value.get('sim_time_s'),
                value.get('request_id'),'event_and_after' if event is not None and stamp>=event else 'pre_event_or_unknown'])))
    report = dict(diagnostic_only=True, qtg_csv=str(Path(qtg_csv).resolve()), qtg_sha256=digest(qtg_csv),
        native_csv=str(Path(native_csv).resolve()), native_sha256=digest(native_csv), anchor=anchor,
        max_age_s=max_age_s, source='BRAKE PARKING POSITION (bool)', wheel_brakes='NOT REQUESTED: decoding unresolved',
        source_acquisition_time=None, issues=issues,
        native_valid_pairs=sum(s['parking_brake'] is not None for s in samples), event_time_s=event,
        recording_start=at_time(samples,0.,max_age_s),
        immediately_pre_throttle=(at_time(samples,event,max_age_s,strictly_before=True)
            if event is not None else dict(status='UNAVAILABLE',value=None,reason='Measured throttle onset unavailable')),
        note='Last prior observation with explicit age; no discrete-state interpolation or exact-event claim')
    with (output/'parking_brake_alignment.json').open('x', encoding='utf-8') as stream:
        json.dump(report,stream,indent=2,allow_nan=False)
    return report
