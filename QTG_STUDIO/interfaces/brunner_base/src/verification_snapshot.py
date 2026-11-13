"""Verification-only ABI. No legacy cache fields or clocks are reinterpreted."""
import ctypes
import math
import threading
import time

CHANNELS = (('on_ground', 'SIM ON GROUND', 'bool'), ('flaps', 'FLAPS HANDLE INDEX', 'number'), ('alt_ft', 'PLANE ALTITUDE', 'feet'), ('ias_kt', 'AIRSPEED INDICATED', 'knots'), ('ground_speed', 'GROUND VELOCITY', 'knots'), ('lat', 'PLANE LATITUDE', 'degrees'), ('lon', 'PLANE LONGITUDE', 'degrees'), ('heading_deg', 'PLANE HEADING DEGREES TRUE', 'degrees'), ('pitch_deg', 'PLANE PITCH DEGREES', 'degrees'), ('bank_deg', 'PLANE BANK DEGREES', 'degrees'), ('throttle', 'GENERAL ENG THROTTLE LEVER POSITION:1', 'percent'), ('engine_rpm', 'GENERAL ENG RPM:1', 'rpm'), ('prop_rpm', 'PROP RPM:1', 'rpm'), ('temperature_c', 'AMBIENT TEMPERATURE', 'celsius'), ('wind_speed', 'AMBIENT WIND VELOCITY', 'knots'), ('wind_direction', 'AMBIENT WIND DIRECTION', 'degrees'), ('barometer', 'KOHLSMAN SETTING MB', 'millibars'), ('ambient_pressure_mb', 'AMBIENT PRESSURE', 'millibars'), ('sim_time', 'SIM TIME', 'seconds'), ('indicated_altitude', 'INDICATED ALTITUDE', 'feet'), ('manifold_pressure', 'ENG MANIFOLD PRESSURE:1', 'psi'), ('mixture', 'GENERAL ENG MIXTURE LEVER POSITION:1', 'percent'), ('propeller_lever', 'GENERAL ENG PROPELLER LEVER POSITION:1', 'percent'))

class VerificationSnapshot(ctypes.Structure):
    _pack_ = 1
    _fields_ = [(k, ctypes.c_uint32) for k in ('size', 'version', 'connected', 'request_id')] + [
        (k, ctypes.c_uint64) for k in ('generation', 'sequence', 'valid_mask', 'receipt_ticks', 'copied_ticks', 'frequency')] + [
        ('running', ctypes.c_int32), ('reserved', ctypes.c_uint32),
        ('values', ctypes.c_double * len(CHANNELS))]


def acquire(client, timeout_sec=1.5, *, clock=time.monotonic, sleep=time.sleep):
    """Read-only ONCE on the existing connection; reject unsupported old DLLs.

    Native QPC age is a duration, never an epoch/monotonic conversion. Using
    call-start minus age conservatively includes time spent crossing the ABI.
    """
    bridge = client._bridge
    try:
        request = bridge.RequestVerificationV1
        copy = bridge.GetVerificationV1
    except AttributeError:
        raise RuntimeError('Native verification snapshot V1 unsupported; rebuild bridge')
    request.argtypes, request.restype = [], ctypes.c_uint32
    copy.argtypes = [ctypes.POINTER(VerificationSnapshot), ctypes.c_uint32]
    copy.restype = ctypes.c_int
    # One reader per client, without affecting the ordinary cache/poller.
    # setdefault is atomic under CPython; no SimConnect connection is created.
    lock = client.__dict__.setdefault('_verification_lock', threading.Lock())
    with lock:
        if not client.is_connected():
            raise RuntimeError('P3D disconnected')
        token = request()
        if not token:
            raise RuntimeError('Verification request unavailable')
        deadline = clock() + timeout_sec
        generation = None
        while clock() < deadline:
            start = clock()
            raw = VerificationSnapshot()
            if not copy(ctypes.byref(raw), ctypes.sizeof(raw)):
                raise RuntimeError('Verification connection lost or ABI unsupported')
            if raw.version != 1 or raw.size != ctypes.sizeof(raw) or not raw.connected:
                raise RuntimeError('Invalid verification ABI/connection')
            if generation is None:
                generation = raw.generation
            if not generation or raw.generation != generation:
                raise RuntimeError('Verification connection generation changed')
            if raw.request_id == token and raw.sequence:
                if not raw.frequency or raw.copied_ticks < raw.receipt_ticks or not raw.receipt_ticks:
                    raise RuntimeError('Invalid native receipt clock')
                age = (raw.copied_ticks-raw.receipt_ticks)/raw.frequency
                values = {name: raw.values[i] for i, (name, _, _) in enumerate(CHANNELS)
                          if raw.valid_mask & (1 << i) and math.isfinite(raw.values[i])}
                return dict(version=1, generation=raw.generation, request_id=token,
                    sequence=raw.sequence, measured_at=start-age, values=values,
                    running={-1: None, 0: False, 1: True}.get(raw.running),
                    valid_mask=raw.valid_mask, receipt_ticks=raw.receipt_ticks,
                    copied_ticks=raw.copied_ticks, frequency=raw.frequency,
                    receipt_age_s=age, source_acquisition_time=None,
                    sources={name: {'simvar': simvar, 'unit': unit}
                             for name, simvar, unit in CHANNELS},
                    provenance='correlated native callback receipt; simulator internal acquisition time unavailable')
            sleep(.01)
        raise RuntimeError('Timed out waiting for correlated native verification response')
