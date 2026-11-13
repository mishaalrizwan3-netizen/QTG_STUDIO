"""Engine-specific additions to the working climb logger; no hardware dependencies."""
import math

class EngineTiming:
    def __init__(self,kind,post_seconds):
        self.kind=kind;self.post_seconds=float(post_seconds)
        self.baseline=None;self.event=None;self.cue=False;self.last_elapsed=0
    def update(self,elapsed,throttle):
        self.last_elapsed=float(elapsed)
        if self.baseline is None:self.baseline=float(throttle)
        direction=1 if self.kind=='acceleration' else -1
        if self.event is None and direction*(float(throttle)-self.baseline)>=2:
            self.event=float(elapsed)
        if elapsed>=5:self.cue=True
        return (math.ceil(self.event+self.post_seconds) if self.event is not None else 21)
    def metadata(self):
        return dict(kind=self.kind,post_seconds=self.post_seconds,event_time_s=self.event,
            baseline_throttle_pct=self.baseline,trigger='2 percentage-point directional throttle change',
            status='complete' if self.event is not None and self.event>=4.8 and self.last_elapsed>=self.event+self.post_seconds else 'incomplete',
            qualification_status='Not assessed: aircraft validation data required')

def frame_values(record):
    d=record['p3d'];c=record.get('cls_cached',{})
    def first(key):
        values=d.get(key) or []
        return values[0] if values else None
    mp=d.get('engine_manifold_pressure_psi_1')
    return dict(time_s=record['observation_elapsed_s'],
        ground_speed_kt=d.get('ground_speed'),
        ias_kt=d.get('airspeed_indicated'),altitude_msl_ft=d.get('pressure_alt'),
        height_agl_ft=d.get('radar_alt'),throttle_pct=first('throttles'),
        engine_rpm=first('eng_rpms'),prop_rpm=first('prop_rpms'),
        manifold_inhg=mp*2.0360206576 if mp is not None else None,
        fuel_flow_pph=d.get('fuelFlow'),
        pitch_position_norm=c.get('cls_pitch_norm'),roll_position_norm=c.get('cls_roll_norm'),
        yaw_position_norm=c.get('cls_yaw_norm'),
        elevator_input_norm=d.get('elevator_pos'),aileron_input_norm=d.get('aileron_pos'),
        rudder_input_norm=d.get('rudder_pos'))
