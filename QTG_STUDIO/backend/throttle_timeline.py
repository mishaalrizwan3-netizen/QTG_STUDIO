"""Replay observed throttle samples without inventing intermediate ramps."""
import bisect
import csv
import json
import math
import statistics
from pathlib import Path

class ThrottleTimeline:
    def __init__(self, samples):
        points = {}
        for t, value in samples:
            t, value = float(t), float(value)
            if not math.isfinite(t) or not math.isfinite(value) or t < 0 or not 0 <= value <= 100.01:
                raise ValueError('Invalid measured throttle sample')
            points[t] = max(0.0, min(100.0, value))
        self.times = sorted(points)
        self.values = [points[t] for t in self.times]
        gaps = [b-a for a,b in zip(self.times,self.times[1:])]
        if not gaps or statistics.median(gaps) > .15 or max(gaps) > .5:
            raise ValueError('High-rate Manual throttle data are missing or contain a gap over 0.5 s. Record a new Manual.')
    def at(self, elapsed):
        # Last actual observation, never a future sample or a 1-Hz interpolation.
        index = max(0, bisect.bisect_right(self.times, float(elapsed))-1)
        return self.values[index]

def load_manual_throttle(path):
    p = Path(path)
    audit = p.with_name('Measured_'+p.stem).with_suffix('.jsonl')
    samples = []
    if audit.exists():
        previous = None
        for line in audit.read_text().splitlines():
            record = json.loads(line)
            data = record.get('p3d') or {}
            sequence = data.get('bridge_telemetry_sequence')
            if sequence is None or sequence == previous:
                continue
            previous = sequence
            throttle = (data.get('throttles') or [None])[0]
            if throttle is not None:
                samples.append((record['observation_elapsed_s'], throttle))
    else:
        channels = p.with_name(p.stem+'_engine_channels').with_suffix('.csv')
        if not channels.exists():
            raise ValueError('Selected Manual has no high-rate throttle companion. Record a new Manual.')
        with channels.open(newline='') as f:
            samples = [(r['time_s'],r['throttle_pct']) for r in csv.DictReader(f) if r.get('throttle_pct')]
    return ThrottleTimeline(samples)
