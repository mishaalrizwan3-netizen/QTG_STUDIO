"""Engine-only adapters around unchanged BRUNNER initialization methods."""
import copy
import csv
import math
from pathlib import Path
import statistics
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


def install(window):
    originals={name:getattr(window,name) for name in ('_commit_initial_position','_refresh_restored_airdata')}
    window._test_extension_originals=originals
    def commit(self,reference_snapshot=None):
        snapshot=self.ref_manual_snapshot if reference_snapshot is None else reference_snapshot
        if reference_snapshot is not None:return originals['_commit_initial_position'](reference_snapshot=reference_snapshot)
        return originals['_commit_initial_position'](reference_snapshot=adjusted_snapshot(self,snapshot))
    def refresh(self):
        original=self.ref_manual_snapshot
        try:
            self.ref_manual_snapshot=adjusted_snapshot(self,original)
            return originals['_refresh_restored_airdata']()
        finally:self.ref_manual_snapshot=original
    window._commit_initial_position=MethodType(commit,window)
    window._refresh_restored_airdata=MethodType(refresh,window)
