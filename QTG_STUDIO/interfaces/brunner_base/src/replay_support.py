"""Pure replay-time interpolation and initial-state channel mapping."""
from bisect import bisect_right
import math


def interpolate_reference(times, values, elapsed):
    if not times or len(times) != len(values):
        raise ValueError("Replay needs matching, nonempty time and value arrays.")
    if not math.isfinite(elapsed):
        raise ValueError("Replay time must be finite.")
    if elapsed <= times[0]: return float(values[0])
    if elapsed >= times[-1]: return float(values[-1])
    index = bisect_right(times, elapsed) - 1
    fraction = (elapsed-times[index])/(times[index+1]-times[index])
    return float(values[index]) + fraction*(float(values[index+1])-float(values[index]))


def build_dynamic_state(telemetry):
    mapping = {
        "velocity_world_x_fps": "velocity_world_x",
        "velocity_world_y_fps": "velocity_world_y",
        "velocity_world_z_fps": "velocity_world_z",
        "velocity_body_x_fps": "velocity_body_x",
        "velocity_body_y_fps": "velocity_body_y",
        "velocity_body_z_fps": "velocity_body_z",
        "rotation_velocity_body_x_rad_s": "rotation_velocity_body_x",
        "rotation_velocity_body_y_rad_s": "rotation_velocity_body_y",
        "rotation_velocity_body_z_rad_s": "rotation_velocity_body_z",
    }
    state = {}
    for dest, source in mapping.items():
        value = telemetry.get(source)
        if value is None or not math.isfinite(float(value)):
            raise ValueError("Missing or invalid Manual initial-state channel: " + source)
        state[dest] = float(value)
    return state


class BrunnerPositionTracker:
    """Bounded position feedback using only measured Brunner positions.

    This corrects servo tracking, not aircraft attitude. It never changes motor
    force/speed settings. Paused offsets are an initial estimate, not a fixed bias.
    """
    def __init__(self, offsets=(0.0, 0.0, 0.0)):
        self.integral = [max(-0.05, min(0.05, float(v))) for v in offsets]
        if len(self.integral) != 3 or not all(math.isfinite(v) for v in offsets):
            raise ValueError("Three finite Brunner calibration offsets are required.")
        self.correction = list(self.integral)
        self.last_time = None

    def command(self, elapsed, reference, feedforward, measured):
        if any(len(values) != 3 for values in (reference, feedforward, measured)):
            raise ValueError("Three Brunner axes are required.")
        if not all(math.isfinite(float(v)) for values in (reference, feedforward, measured) for v in values):
            raise ValueError("Invalid Brunner position feedback; replay stopped.")
        if not math.isfinite(elapsed) or (self.last_time is not None and elapsed < self.last_time):
            raise ValueError("Invalid replay clock.")
        dt = min(0.1, max(0.0, elapsed-self.last_time)) if self.last_time is not None else 0.0
        self.last_time = elapsed
        commands = []
        for axis in range(3):
            error = reference[axis]-measured[axis]
            candidate = max(-0.05, min(0.05, self.integral[axis]+0.8*error*dt))
            desired = max(-0.05, min(0.05, candidate+0.4*error))
            step = 0.1*dt
            correction = max(self.correction[axis]-step, min(self.correction[axis]+step, desired))
            raw = feedforward[axis]+correction
            command = max(-1.0, min(1.0, raw))
            # Do not accumulate integral correction against an actuator travel limit.
            if abs(raw) <= 1.0 or (raw > 1.0 and error < 0) or (raw < -1.0 and error > 0):
                self.integral[axis] = candidate
            self.correction[axis] = correction
            commands.append(command)
        return tuple(commands)


def validate_initial_airspeed(telemetry, reference_kts, tolerance_kts=2.0):
    """Reject inconsistent startup flight state, without changing measured data."""
    measured = float(telemetry.get("airspeed_indicated", float("nan")))
    reference = float(reference_kts)
    if not math.isfinite(measured) or not math.isfinite(reference) or reference <= 0:
        raise ValueError("Initial airspeed is unavailable; replay was not started.")
    if abs(measured-reference) > tolerance_kts:
        raise ValueError(f"Initial airspeed mismatch: Manual={reference:.2f} kt, "
                         f"Automatic={measured:.2f} kt. Replay stopped; reinitialize the aircraft.")
    return measured


def validate_reference_power(telemetry, snapshot):
    """Readiness only: never move levers or force RPM/temperatures to match."""
    state = snapshot.get("telemetry", {})
    throttle = snapshot.get("throttle", state.get("engine_throttle_percent"))
    checks = [("Throttle (%)", "engine_throttle_percent", throttle, 1.0),
              ("Mixture (%)", "engine_mixture_percent", state.get("engine_mixture_percent"), 1.0),
              ("Propeller lever (%)", "engine_propeller_percent", state.get("engine_propeller_percent"), 1.0),
              ("Oil temperature (C)", "engine_oil_temperature_c_1", state.get("engine_oil_temperature_c_1"), 10.0),
              ("Cylinder-head temperature (C)", "engine_cylinder_head_temperature_c_1", state.get("engine_cylinder_head_temperature_c_1"), 20.0)]
    problems = []
    for label, key, target, tolerance in checks:
        if target is None or not math.isfinite(float(target)):
            continue  # Legacy reference did not measure this channel.
        measured = telemetry.get(key)
        if measured is None or not math.isfinite(float(measured)):
            problems.append(label + " is unavailable")
        elif abs(float(measured)-float(target)) > tolerance:
            problems.append(f"{label}: Manual={float(target):.1f}, current={float(measured):.1f}")
    if problems:
        raise ValueError("Initial power conditions do not match. " + "; ".join(problems)
                         + ". Automatic preparation could not verify the recorded engine setup. "
                         "Check the listed mixture/propeller settings, hardware overrides and engine temperatures; replay has not started.")


def estimate_replay_lead(reference_times, reference_values, measured_times, measured_values, previous_lead=0.08):
    """Estimate residual servo lag from a completed run, never from aircraft attitude."""
    import numpy as np
    if not math.isfinite(float(previous_lead)) or not 0 <= float(previous_lead) <= 0.5:
        raise ValueError("Previous command lead must be between zero and 0.5 seconds.")
    rt, rv, mt, mv = (np.asarray(v, dtype=float) for v in
                      (reference_times, reference_values, measured_times, measured_values))
    if any(v.ndim != 1 for v in (rt, rv, mt, mv)) or len(rt) != len(rv) or len(mt) != len(mv):
        raise ValueError("Tracking calibration needs paired time and position arrays.")
    if len(rt) < 100 or len(mt) < 100 or any(not np.isfinite(v).all() for v in (rt,rv,mt,mv)):
        return float(previous_lead)
    if np.any(np.diff(rt) <= 0) or np.any(np.diff(mt) <= 0):
        return float(previous_lead)
    grid = np.arange(max(rt[0],mt[0])+0.6, min(rt[-1],mt[-1])-0.6, 0.02)
    if len(grid) < 100:
        return float(previous_lead)
    actual = np.interp(grid, mt, mv)
    if np.std(actual) < 0.005:
        return float(previous_lead)
    shifts = np.arange(-0.5,0.501,0.01)
    errors = [float(np.mean((actual-np.interp(grid-shift,rt,rv))**2)) for shift in shifts]
    best = int(np.argmin(errors))
    baseline = errors[50]
    if errors[best] >= baseline*0.9 or best in (0,len(shifts)-1):
        return float(previous_lead)
    return max(0.0,min(0.5,float(previous_lead)+float(shifts[best])))


def replay_feedforward(times, axis_values, elapsed, leads):
    """Introduce calibrated lookahead smoothly while preserving the starting position."""
    blend = min(1.0,max(0.0,elapsed))
    return tuple(interpolate_reference(times,values,elapsed+blend*lead)
                 for values,lead in zip(axis_values,leads))


def reference_altimeter_offset(snapshot):
    """Use an expected settled offset; reject the signature of a teleport-lagged altimeter."""
    state=snapshot.get("telemetry",{})
    indicated=state.get("indicated_altitude",snapshot.get("altitude"))
    physical=state.get("pressure_alt",snapshot.get("altitude_msl"))
    if indicated is None or physical is None:
        return None
    offset=float(indicated)-float(physical)
    baro=float(state.get("baro_pressure",float("nan")))
    if math.isfinite(baro) and abs(baro-1013.25)<3.0 and abs(offset)>10.0:
        return 0.0
    if abs(offset)>50.0:
        return 0.0
    return offset


class ThrottleReplay:
    """Timestamped throttle replay, independent of Brunner phase lead."""
    def __init__(self,times,values):
        if not times or len(times)!=len(values):
            raise ValueError("Throttle replay requires matching reference times and values.")
        if any(not math.isfinite(float(v)) for v in (*times,*values)):
            raise ValueError("Throttle reference contains unavailable values.")
        if any(not 0 <= float(v) <= 100 for v in values):
            raise ValueError("Throttle reference must be between zero and 100 percent.")
        if any(b<=a for a,b in zip(times,times[1:])):
            raise ValueError("Throttle reference times must increase.")
        self.times=times;self.values=values
        self.last_elapsed=None;self.last_sent=None;self.last_command=None
    def command(self,elapsed):
        if not math.isfinite(elapsed) or (self.last_elapsed is not None and elapsed<self.last_elapsed):
            raise ValueError("Throttle replay clock must be finite and monotonic.")
        self.last_elapsed=elapsed
        target=interpolate_reference(self.times,self.values,elapsed)
        if self.last_sent is None or abs(target-self.last_command)>=0.05 or elapsed-self.last_sent>=0.2:
            self.last_sent=elapsed;self.last_command=target
            return target
        return None


def reference_weather_metar(snapshot, current_metar):
    """Restore measured wind/OAT at reference altitude, retaining other weather.

    KOHLSMAN (baro_pressure) is an instrument setting, not sea-level weather
    pressure; never substitute it for the current METAR pressure.
    """
    import re
    state=snapshot.get('telemetry',{})
    def measured(key, fallback):
        value=state.get(key,snapshot.get(fallback))
        if value is None:return None
        value=float(value)
        if not math.isfinite(value):raise ValueError('Invalid recorded weather: '+key)
        return value
    wind=measured('ambient_wind_spd','wind_spd')
    direction=measured('ambient_wind_dir','wind_dir')
    temperature=measured('ambient_temp','oat')
    altitude=measured('pressure_alt','altitude_msl')
    if altitude is None:altitude=measured('indicated_altitude','altitude')
    if wind is None and temperature is None:return None
    if wind is not None and not 0 <= wind <= 400:raise ValueError('Invalid recorded wind speed')
    if wind is not None and wind>0 and direction is None:raise ValueError('Recorded wind direction is missing')
    if temperature is not None and altitude is None:raise ValueError('Recorded weather altitude is missing')
    if not isinstance(current_metar,str) or not current_metar.strip():
        raise ValueError('Current weather observation is unavailable; cannot restore reference weather')
    body,separator,aloft=current_metar.partition('@@@')
    tokens=body.split()
    tokens[0]='GLOB'
    wind_pattern=r'(?:[0-9]{3}|VRB)[0-9]+(?:G[0-9]+)?(?:KT|MPS|KMH).*'
    temp_pattern=r'(?:M|-)?[0-9]+(?:\.[0-9]+)?/(?:M|-)?[0-9]+(?:\.[0-9]+)?.*'
    def temp_token(value):
        degrees = round(value)
        return ('-' if degrees<0 else '')+f'{abs(degrees):02d}'
    result=[]
    wind_done=temp_done=False
    for token in tokens:
        if wind is not None and re.fullmatch(wind_pattern,token):
            if not wind_done:result.append(f'{round(direction or 0)%360:03d}{round(wind):02d}KT&D9999NG')
            wind_done=True
        elif temperature is not None and re.fullmatch(temp_pattern,token):
            if not temp_done:
                dewpoint=float(token.split('/',1)[1].split('&',1)[0].replace('M','-'))
                result.append(f'{temp_token(temperature)}/{temp_token(min(dewpoint,temperature))}&A{round(altitude*0.3048):05d}')
            temp_done=True
        else:result.append(token)
    if wind is not None and not wind_done:raise ValueError('Current weather has no supported wind observation')
    if temperature is not None and not temp_done:raise ValueError('Current weather has no supported temperature observation')
    if wind is None and separator:result.extend(['@@@',aloft])
    return ' '.join(result)


def startup_rpm_tolerance(test_reference):
    """Startup allowance only; never a tolerance for formal QTG results.

    Normal climb can begin during a power transient. Requiring its first
    instantaneous RPM to be a two-second equilibrium prevents replay at the
    correctly restored throttle. Keep engine-response and unknown tests strict.
    """
    return 100.0 if test_reference == '1.c.(1)' else 10.0
