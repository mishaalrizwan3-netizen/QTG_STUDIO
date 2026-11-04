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

def run_initialization(window, mode, entry, policy=None):
    if policy is None:return entry(window)
    window.target_duration_sec=policy.DURATION_SECONDS
    cfg=dict(policy.DEFAULT_INITIAL_CONDITIONS)
    cfg.update(window._session_context.get('initial_conditions') or {})
    if mode=='auto':
        with Path(window.selected_ref_csv).open(newline='',encoding='utf-8-sig') as stream:
            row=next(csv.DictReader(stream))
        mismatches=[]
        for key,column,tolerance in [('alt_ft','Altitude MSL (ft)',25.),('ias_kt','Indicated Airspeed (kts)',2.),('throttle','Throttle 1 Position (%)',1.)]:
            try:actual=float(row[column])
            except (KeyError,ValueError,TypeError):raise ValueError('Manual reference is missing '+column)
            if not math.isfinite(actual) or abs(actual-cfg[key])>tolerance:
                mismatches.append(f'{column}: CSV={actual:g}, Test settings={cfg[key]:g}')
        if mismatches:raise ValueError('Manual reference does not match engine Test settings: '+ '; '.join(mismatches)+'. Record/select a matching Manual CSV. Change initial conditions only in Test settings.')
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
            sim.set_position_airborne(alt_ft=cfg['alt_ft'],speed_kts=cfg['ias_kt'],pitch_deg=0.,bank_deg=0.,
                hdg_deg=0.,lat=cfg.get('lat',32.235),lon=cfg.get('lon',74.135))
            window._wait_for_initial_altitude(cfg['alt_ft'])
            if not sim.wait_for_paused(3.):raise RuntimeError('Simulator did not confirm pause.')
            if not sim.set_flaps_handle_index(cfg['flaps']):raise RuntimeError('Initial flaps rejected.')
            if not sim.set_initial_throttle(cfg['throttle']):raise RuntimeError('Initial throttle rejected.')
            if window.stop_worker_event.is_set():raise RuntimeError('Engine initialization cancelled.')
            actual=window._refresh_paused_telemetry()
            for key,target,tolerance in [('pressure_alt',cfg['alt_ft'],25.),('airspeed_indicated',cfg['ias_kt'],2.),
                    ('engine_throttle_percent',cfg['throttle'],1.),('measured_flaps_handle_index',cfg['flaps'],.1)]:
                value=float(actual.get(key,float('nan')))
                if not math.isfinite(value) or abs(value-target)>tolerance:
                    raise RuntimeError(f'Engine initial-condition mismatch: {key}, requested {target:g}, actual {value:g}')
            window.sig_update_stage.emit(base.STAGE_CONNECTED_READY,'Engine Test settings applied; ready to record 21 seconds.')
        except Exception as error:
            window.p3d.pause_sim()
            window.sig_update_stage.emit(base.STAGE_CONNECTION_ERROR,str(error))
    window.preparation_thread=threading.Thread(target=prepare,daemon=True)
    window.preparation_thread.start()
