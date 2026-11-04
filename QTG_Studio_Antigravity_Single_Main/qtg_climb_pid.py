"""Climb-only adapter for the supplied V4.1 IAS/pitch controller; no second UI/socket."""
import ast
import csv
import json
import math
from pathlib import Path
import struct
import socket
import sys
import threading
import time

SOURCE = 'SMK_IAS_PID_BRUNNER_ONLY_V4_1.py'
CLIMB_PROFILE = {
    'pitch':dict(kp=.030,ki=.004,kd=.045,invert=False),
    'ias':dict(kp=.24,ki=.008,kd=.32,min_pitch=-15.,max_pitch=15.),
    'slew':dict(max_rate=.28,max_accel=.90),
}
VALIDATION_COLUMNS=['Time (s)','Indicated Airspeed (kts)','Rate of Climb/Descent (fpm)']

def validate_dataset(path):
    import pandas as pd
    import numpy as np
    frame=pd.read_csv(path)
    if any(key not in frame for key in VALIDATION_COLUMNS):
        raise ValueError('Validation CSV requires: '+', '.join(VALIDATION_COLUMNS))
    values=frame[VALIDATION_COLUMNS].to_numpy(float)
    if len(values)<2 or not np.isfinite(values).all() or not np.all(np.diff(values[:,0])>0):
        raise ValueError('Validation samples must be finite with increasing times.')
    return frame

def controller_types():
    root=Path(sys._MEIPASS)/'pid_source' if getattr(sys,'frozen',False) else Path(__file__).resolve().parent.parent/'PID Controller'
    path=root/SOURCE
    tree=ast.parse(path.read_text(encoding='utf-8-sig'))
    names={'PitchPID','AirspeedController','slew_position'}
    nodes=[node for node in tree.body if isinstance(node,(ast.ClassDef,ast.FunctionDef)) and node.name in names]
    if {node.name for node in nodes}!=names: raise RuntimeError('Supplied PID implementation is incomplete.')
    namespace={'math':math}
    exec(compile(ast.Module(body=nodes,type_ignores=[]),str(path),'exec'),namespace)
    return namespace['PitchPID'],namespace['AirspeedController'],namespace['slew_position']

def original_pitch_transport(device):
    """Use the supplied working CLS2Sim methods on the already-open QTG socket.

    __init__ is deliberately skipped: no second connection or GUI is created.
    """
    root=Path(sys._MEIPASS)/'pid_source' if getattr(sys,'frozen',False) else Path(__file__).resolve().parent.parent/'PID Controller'
    path=root/SOURCE
    tree=ast.parse(path.read_text(encoding='utf-8-sig'))
    names={'pack_read','pack_override','pack_u16','pack_u8','pack_float','pack_watchdog','reply_status','reply_normalized','CLS2Sim'}
    nodes=[node for node in tree.body if isinstance(node,(ast.FunctionDef,ast.ClassDef)) and node.name in names]
    if {node.name for node in nodes}!=names:raise RuntimeError('Original PID transport source is incomplete.')
    namespace=dict(math=math,struct=struct,socket=socket,threading=threading,time=time,
        HOST=device.host,PORT=device.port,AX_PITCH=1,CMD_READ=0xD0,CMD_OVERRIDE=0xD1,CMD_SETTINGS=0xCE,CMD_REMOTE_CONFIG=0xD8,
        POS_NORM=0x11,OVR_AP_ENABLE=3,OVR_AP_POS=4,OVR_POSITION_TO_SIM=6,
        SID_AP_FORCE=0x50,SID_AP_SPEED=0x60,SID_AP_ENABLE=0x70,SID_AP_POSITION=0x80,
        AP_FORCE=2400,AP_SPEED=80,WATCHDOG_MS=1000)
    exec(compile(ast.Module(body=nodes,type_ignores=[]),str(path),'exec'),namespace)
    transport=object.__new__(namespace['CLS2Sim'])
    transport.sock=device.sock;transport.io_lock=device._io_lock
    transport.latest_position=0.;transport.latest_time=0.;transport.owns_overrides=False
    transport.watchdog_enabled=False;transport.last_watchdog=0.;transport.connected=True
    device.pid_transport_trace=[]
    exchange=transport.exchange
    def traced_exchange(packet):
        response=exchange(packet)
        trace=device.pid_transport_trace
        if len(trace)>=2048:trace.pop(32)
        trace.append({'time_s':time.monotonic(),'request':packet.hex(),'response':response.hex()})
        return response
    transport.exchange=traced_exchange
    return transport


def options(values=None):
    result=dict(alt_ft=3000.,ias_kt=80.,lat=32.235,lon=74.135,flaps=0,throttle=100.,temperature_c=15.,target_ias=80.)
    result.update(values or {})
    limits={'alt_ft':(100,25000),'ias_kt':(40,220),'target_ias':(40,220),'lat':(-90,90),'lon':(-180,180),'flaps':(0,10),'throttle':(0,100),'temperature_c':(-100,60)}
    for key,(lo,hi) in limits.items():
        value=float(result[key])
        if not math.isfinite(value) or not lo<=value<=hi: raise ValueError('Invalid climb PID '+key)
        result[key]=value
    if result['flaps']!=int(result['flaps']): raise ValueError('Flaps must be a handle index.')
    return result

def watchdog(device, milliseconds=1000):
    transport=getattr(device,'_pid_transport',None)
    if transport is not None:
        now=time.monotonic()
        if milliseconds!=1000 or now-transport.last_watchdog>.40:
            transport.write(struct.pack('<IIH',0xD8,0,milliseconds))
            transport.last_watchdog=now
        return
    with device._io_lock:
        if not device.sock: raise RuntimeError('CLS2sim disconnected.')
        device.sock.send(struct.pack('<IIH',0xD8,0,milliseconds))
        response=device.sock.recv(1024)
        if len(response)<3 or response[2]!=0: raise RuntimeError('CLS2sim watchdog rejected.')

def fresh(window, sequence):
    if not window.p3d.is_connected(): raise RuntimeError('Sim Flight Model disconnected.')
    if not window.p3d.request_telemetry_once(): raise RuntimeError('Telemetry request rejected.')
    data=window.p3d.wait_for_fresh_telemetry(sequence,.35)
    if data is None: raise RuntimeError('Sim Flight Model feedback stale.')
    for key in ('airspeed_indicated','pitch_angle','bank_angle','true_heading'):
        if not math.isfinite(float(data.get(key,float('nan')))): raise RuntimeError('Invalid feedback: '+key)
    return data

def check_cancel(window):
    if window.stop_worker_event.is_set(): raise RuntimeError('PID preparation cancelled.')

def discover_axes(device):
    from brunner_interface import AxisUnavailableError
    available=[]
    for axis in (device.AXIS_PITCH,device.AXIS_ROLL,device.AXIS_RUDDER):
        try: device.read_axis(axis)
        except AxisUnavailableError:
            if axis==device.AXIS_PITCH: raise RuntimeError('PID requires valid pitch-loader position feedback.')
            continue
        available.append(axis)
    return tuple(available)

def move_controls(window,pitch):
    # PID owns Pitch only. Other physical axes remain under operator control.
    window.brunner.move_actuators(pitch,None,None)


def measured_axes(window):
    device=window.brunner
    available=getattr(window,'pid_axes',(device.AXIS_PITCH,device.AXIS_ROLL,device.AXIS_RUDDER))
    return [device.read_axis(axis) if axis in available else (float('nan'),)*3 for axis in (device.AXIS_PITCH,device.AXIS_ROLL,device.AXIS_RUDDER)]


def verify_forwarding(window):
    """Small bounded physical probe with live simulator feedback; require surface response."""
    device=window.brunner
    start=device.read_axis(device.AXIS_PITCH)[0]
    if abs(start)>.65: raise RuntimeError('Pitch loader is outside PID starting range.')
    measurements=[]
    reached=[]
    try:
        for offset in (-.025,.025):
            target=max(-.55,min(.65,start+offset))
            position=device.read_axis(device.AXIS_PITCH)[0]
            for step in range(1,7):
                check_cancel(window)
                watchdog(device)
                with device._io_lock:
                    check_cancel(window)
                    move_controls(window,position+(target-position)*step/6)
                if window.stop_worker_event.wait(.04): check_cancel(window)
            # The working V4.1 log shows ~1 s loader startup latency. Keep
            # sending the bounded target until measured feedback catches up;
            # do not reverse direction after a fixed sub-second sleep.
            deadline=time.monotonic()+2.5
            while True:
                check_cancel(window)
                watchdog(device)
                with device._io_lock:
                    check_cancel(window)
                    move_controls(window,target)
                actual=device.read_axis(device.AXIS_PITCH)[0]
                data=fresh(window,window.p3d.get_telemetry_sequence())
                if abs(float(data['bank_angle']))>20 or float(data['airspeed_indicated'])<10:raise RuntimeError('Aircraft outside PID envelope during forwarding check.')
                arrived=abs(actual-target)<=.005
                if arrived or time.monotonic()>=deadline:break
                if window.stop_worker_event.wait(.04):check_cancel(window)
            reached.append(arrived)
            surface=float(data.get('surface_elevator_rad',float('nan')))
            if not math.isfinite(surface): raise RuntimeError('Elevator feedback unavailable for forwarding check.')
            measurements.append((actual,surface))
        position_delta=abs(measurements[1][0]-measurements[0][0])
        surface_delta=abs(measurements[1][1]-measurements[0][1])
        verified=all(reached) and position_delta>=.015 and surface_delta>=.0001
        window.pid_forwarding_check={'verified':verified,'samples':measurements,'targets_reached':reached,'endpoint_timeout_s':2.5,'position_delta':position_delta,'surface_delta_rad':surface_delta,'simulation_running':True,'path':'Physical Pitch -> CLS2sim -> simulator elevator surface'}
        if not verified:
            reason='Pitch loader did not reach the probe positions within 2.5 seconds' if not all(reached) or position_delta<.015 else 'Pitch loader moved, but simulator elevator did not respond'
            raise RuntimeError(f'Physical position-to-simulator forwarding was not verified: {reason}. Position change={position_delta:.5f}, elevator change={surface_delta:.6f} rad. Check CLS2sim profile and simulator axis assignments.')
    finally:
        if not window.stop_worker_event.is_set(): move_controls(window,start)

def safe_release(window):
    errors=[]
    try: window.p3d.pause_sim()
    except Exception as exc: errors.append(str(exc))
    window.brunner.strict_io=False
    window.brunner.pid_position_nodes=False
    try: window.brunner.release_controls()
    except Exception as exc: errors.append(str(exc))
    try: watchdog(window.brunner,0)
    except Exception as exc: errors.append(str(exc))
    return errors


def resume_initialization(window):
    """Bounded startup-only resume; never resume an interrupted active PID test."""
    check_cancel(window)
    if getattr(window,'pid_phase','starting') in ('holding','stabilizing','recording'):
        raise RuntimeError('Simulator paused during active PID; test incomplete.')
    attempts=getattr(window,'pid_initial_resume_count',0)
    if attempts>=2:raise RuntimeError('Simulator repeatedly paused during initialization; check simulator pause sources.')
    window.pid_initial_resume_count=attempts+1
    check_cancel(window)
    window.p3d.unpause_sim()
    deadline=time.monotonic()+3.;running_since=None
    while True:
        check_cancel(window)
        if not window.p3d.is_connected():raise RuntimeError('Sim Flight Model disconnected during initialization.')
        now=time.monotonic()
        if window.p3d.simulation_running():
            if running_since is None:running_since=now
            if now-running_since>=.25:
                getattr(window,'_session_context',{})['pid_initial_resume_count']=window.pid_initial_resume_count
                return
        else:running_since=None
        if now>=deadline:raise RuntimeError('Simulator did not confirm a steady running state during initialization.')
        window.stop_worker_event.wait(.04)


def verify_live_temperature(window,timeout_s=15.):
    """Weather interpolation needs a running simulator; retain actual-OAT verification."""
    target=window.pid_conditions['temperature_c']
    deadline=time.monotonic()+timeout_s
    actual=float('nan')
    while True:
        check_cancel(window)
        if not window.p3d.simulation_running():
            resume_initialization(window)
            continue
        data=fresh(window,window.p3d.get_telemetry_sequence())
        if abs(float(data['bank_angle']))>20 or float(data['airspeed_indicated'])<10:
            raise RuntimeError('Aircraft outside PID envelope during initial weather verification.')
        actual=float(data.get('ambient_temp',float('nan')))
        if math.isfinite(actual) and abs(actual-target)<=2.:
            getattr(window,'_session_context',{})['pid_temperature_check']={'verified':True,'requested_c':target,'actual_c':actual}
            return data
        if time.monotonic()>=deadline:
            getattr(window,'_session_context',{})['pid_temperature_check']={'verified':False,'requested_c':target,'actual_c':actual}
            raise RuntimeError(f'Initial temperature did not settle: requested {target:g} C, actual {actual:.2f} C. Check simulator weather or external weather overrides.')
        window.stop_worker_event.wait(.1)


def wait_for_aircraft_settled(window,timeout_s=20.):
    """Require two seconds of valid, steady engine/configuration feedback before PID."""
    cfg=window.pid_conditions;start=time.monotonic();samples=[]
    while True:
        check_cancel(window)
        data=fresh(window,window.p3d.get_telemetry_sequence())
        if not window.p3d.simulation_running():
            samples=[]
            resume_initialization(window)
            continue
        now=time.monotonic()
        expected=(('engine_throttle_percent',cfg['throttle'],1.),('ambient_temp',cfg['temperature_c'],2.),('measured_flaps_handle_index',cfg['flaps'],.1))
        valid=all(math.isfinite(float(data.get(key,float('nan')))) and abs(float(data[key])-target)<=tol for key,target,tol in expected)
        rpm=float(data.get('engine_rpm',float('nan')))
        altitude=float(data.get('pressure_alt',float('nan')))
        valid=valid and math.isfinite(rpm) and rpm>300 and math.isfinite(altitude) and abs(altitude-cfg['alt_ft'])<=100 and abs(float(data['bank_angle']))<=5 and abs(float(data['pitch_angle']))<=20
        if valid:
            samples.append((now,rpm));samples=[sample for sample in samples if now-sample[0]<=2.2]
            if samples and now-samples[0][0]>=2. and max(value for _,value in samples)-min(value for _,value in samples)<=50:
                getattr(window,'_session_context',{})['pid_aircraft_settling']={'verified':True,'elapsed_s':now-start,'steady_s':now-samples[0][0],'readback':dict(data)}
                return data
        else:samples=[]
        if now-start>=timeout_s:raise RuntimeError('Aircraft did not settle before PID startup: check altitude, bank, flaps, throttle, temperature and engine RPM.')
        window.stop_worker_event.wait(.1)

class PIDResponseVerifier:
    """Verify the live Pitch path from real PID motion, without a separate probe."""
    def __init__(self):self.samples=[];self.verified=False
    def update(self,command,actual,surface):
        if not all(math.isfinite(float(value)) for value in (command,actual,surface)):
            raise RuntimeError('Invalid Pitch position/elevator feedback during PID path verification.')
        self.samples.append((command,actual,surface));self.samples=self.samples[-500:]
        if not self.verified and abs(command-actual)<=.05:
            self.verified=any(abs(command-old[0])>=.015 and abs(actual-old[1])>=.015 and (command-old[0])*(actual-old[1])>0 and abs(surface-old[2])>=.0001 for old in self.samples)
        return {'verified':self.verified,'method':'Measured response to live PID commands','samples':self.samples[-8:]}


def prepare(window,base,context):
    if context['test_reference']!='1.c.(1)': raise ValueError('PID is supported only for normal climb.')
    cfg=options(dict(context.get('pid_conditions') or {},target_ias=80.))
    window.pid_automatic_workflow=True
    window._external_control_active=True
    window.pid_ready=False
    window.pid_controller_active=False
    window.pid_initial_resume_count=0
    context['pid_conditions']=cfg
    context['pid_workflow']=dict(target_ias=80.,band_kt=1.,hold_s=5.,timeout_s=45.,duration_s=window.target_duration_sec)
    window.test_mode='auto'
    window.active_manual_ref_path=None
    window.selected_ref_csv=None
    window.stop_worker_event.clear()
    window.sig_update_stage.emit(base.STAGE_INITIALIZING,'PID climb: initializing selected conditions; bank/yaw 0 degrees.')
    def worker():
        try:
            # This is the same ownership and condition-application path used by
            # Manual; no second PID-specific teleport or final-pose write follows.
            from qtg_common_engine import apply_manual_conditions,validate_conditions
            initial=validate_conditions(context.get('initial_conditions') or
                {key:cfg[key] for key in ('alt_ft','ias_kt','lat','lon','flaps','throttle','temperature_c')})
            if initial['on_ground']:raise RuntimeError('Normal climb PID requires airborne initial conditions.')
            cfg.update({key:initial[key] for key in ('alt_ft','ias_kt','lat','lon','flaps','throttle','temperature_c')})
            check_cancel(window)
            window.brunner.prepare_manual_mode()
            # No automatic motor ownership during initialization. A live watchdog
            # here would expire while weather/aircraft settling waits run.
            watchdog(window.brunner,0)
            check_cancel(window)
            actual=apply_manual_conditions(window,initial)
            check_cancel(window)
            context['pid_initialization_method']='Shared Manual initialization'
            window.pid_paused=True
            window.pid_ready=True
            window.pid_controller_active=False
            window.pid_record_requested=False
            window.pid_forwarding_check={'verified':False,'status':'Pending measured response to real PID commands after aircraft settling.'}
            context["pid_profile"]=CLIMB_PROFILE
            window.pid_conditions=cfg
            window.pid_fault=None
            window.pid_diagnostics=[]
            window.pid_phase='starting'
            window.sig_update_stage.emit(base.STAGE_INITIALIZING,'Completing live aircraft initialization before starting PID.')
            window.pid_profile=CLIMB_PROFILE
            window.is_recording=False
            for name in ('rec_time','p_pos_buf','r_pos_buf','y_pos_buf','p_force_buf','r_force_buf','y_force_buf','p3d_tele_buf','applied_force_buf','virtual_force_buf','replay_command_buf'):
                getattr(window,name).clear()
            context['pid_initial_readback']=dict(actual)
            context['pid_temperature_check']={'verified':False,'status':'Pending live weather interpolation after unpause.'}
            window.pid_controller_active=True
            window.worker_thread=threading.current_thread()
            loop(window,base)
            context.update(pid_fault=window.pid_fault,pid_complete=not bool(window.pid_fault),pid_stabilization=getattr(window,'pid_stabilization',{}),pid_forwarding_check=getattr(window,'pid_forwarding_check',{}))
        except Exception as exc:
            window.pid_fault=str(exc)
            window.pid_ready=False
            window.pid_controller_active=False
            window._external_control_active=False
            context['pid_fault']=str(exc)
            context['pid_complete']=False
            safe_release(window)
            window.sig_update_stage.emit(base.STAGE_AUTO_BLOCKED,str(exc))
    window.preparation_thread=threading.Thread(target=worker,daemon=True)
    window.preparation_thread.start()

class StabilizationGate:
    """Monotonic-time state: every out-of-band sample restarts the hold."""
    def __init__(self): self.since=None
    def update(self,now,ias,started):
        if now-started>45.: raise RuntimeError('Airspeed stabilization failed: 45-second timeout.')
        if not math.isfinite(ias): raise RuntimeError('Invalid IAS feedback.')
        if abs(ias-80.)>1.: self.since=None; return False
        if self.since is None:self.since=now
        return now-self.since>=5.


def loop(window,base):
    device=window.brunner
    failed=None
    try:
        PitchPID,AirspeedController,slew=controller_types()
        cfg=window.pid_conditions
        device.strict_io=True
        device.pid_position_nodes=True
        check_cancel(window)
        window.pid_axes=discover_axes(device)
        automatic=getattr(window,'pid_automatic_workflow',False)
        if automatic:
            check_cancel(window)
            resume_initialization(window)
            check_cancel(window)
            window.sig_update_stage.emit(base.STAGE_INITIALIZING,'Waiting for live initial temperature readback.')
            verify_live_temperature(window)
            check_cancel(window)
            window.sig_update_stage.emit(base.STAGE_INITIALIZING,'Waiting for aircraft and engine readings to settle before PID startup.')
            wait_for_aircraft_settled(window)
            check_cancel(window)
            # Engage after all live initialization and settling steps finish.
            with device._io_lock:
                check_cancel(window)
                device.configure_pid_mode(device.read_axis(device.AXIS_PITCH)[0])
            getattr(window,'_session_context',{})['pid_engaged_after_live_initialization']=True
            window.pid_forwarding_check={'verified':False,'method':'Pending measured response to live PID commands'}
            check_cancel(window)
        else:
            with device._io_lock:
                check_cancel(window)
                device.configure_pid_mode(device.read_axis(device.AXIS_PITCH)[0])
        current=device.read_axis(device.AXIS_PITCH)[0]
        data=fresh(window,window.p3d.get_telemetry_sequence())
        profile=getattr(window,'pid_profile',CLIMB_PROFILE)
        pid=PitchPID(**profile['pitch']); outer=AirspeedController(**profile['ias'])
        pid.reset(current);outer.reset(-float(data['pitch_angle']))
        velocity=0.; mismatch=None
        recording_started=None
        window.pid_phase='stabilizing' if automatic else 'holding'
        gate=StabilizationGate()
        response=PIDResponseVerifier()
        window.pid_stabilization={'achieved':False,'band_kt':1.,'hold_s':5.,'timeout_s':45.}
        if automatic: window.sig_update_stage.emit('Stabilizing Airspeed','Holding 80 kt within +/-1 kt for 5 continuous seconds.')
        started=last=time.monotonic()
        tick=started
        while window.pid_controller_active and not window.stop_worker_event.is_set():
            now=time.monotonic();dt=now-last
            if recording_started is not None and now-recording_started>=window.target_duration_sec: break
            if dt>.35: raise RuntimeError('PID scheduling watchdog expired.')
            data=fresh(window,window.p3d.get_telemetry_sequence())
            window.pid_paused=not window.p3d.simulation_running()
            if automatic and recording_started is None and now-started>45.: raise RuntimeError('PID startup failed at 45-second timeout: '+('physical Pitch-to-simulator response was not verified' if not response.verified else 'airspeed did not stabilize')+'.')
            if window.pid_paused:
                if automatic: raise RuntimeError('Simulator paused during PID Automatic; test incomplete.')
                if recording_started is not None: raise RuntimeError('Simulator paused during recording; run interrupted.')
                watchdog(device)
                last=time.monotonic();tick=last
                window.stop_worker_event.wait(.04)
                continue
            watchdog(device)
            axes=measured_axes(window)
            now=time.monotonic();dt=max(.001,now-last);last=now
            if dt>.35: raise RuntimeError('PID feedback/command cycle exceeded watchdog limit.')
            if float(data['airspeed_indicated'])<10: raise RuntimeError('IAS below PID operating range.')
            if abs(float(data['bank_angle']))>20: raise RuntimeError('Bank exceeded climb PID envelope.')
            cfg=window.pid_conditions
            profile=getattr(window,'pid_profile',CLIMB_PROFILE)
            for key in ('kp','ki','kd'):
                setattr(pid,key,profile['pitch'][key]);setattr(outer,key,profile['ias'][key])
            if pid.invert!=profile['pitch']['invert']: pid.integral=0.
            pid.invert=profile['pitch']['invert']
            pitch=-float(data['pitch_angle'])
            target=outer.update(cfg['target_ias'],float(data['airspeed_indicated']),dt,pitch)
            desired=pid.update(target,pitch,dt)
            current,velocity=slew(current,velocity,desired,dt,**CLIMB_PROFILE['slew'])
            if window.stop_worker_event.is_set(): break
            with device._io_lock:
                if window.stop_worker_event.is_set(): break
                move_controls(window,current)
            if abs(current-axes[0][0])>.25:
                mismatch=now if mismatch is None else mismatch
                if now-mismatch>2.5: raise RuntimeError('CLS2sim pitch position did not track PID command.')
            else: mismatch=None
            control_elapsed=now-started
            if automatic:
                window.pid_forwarding_check=response.update(current,axes[0][0],float(data.get('surface_elevator_rad',float('nan'))))
            stable=gate.update(now,float(data['airspeed_indicated']),started) if automatic and recording_started is None else False
            if recording_started is None and (stable and response.verified if automatic else getattr(window,'pid_record_requested',False)):
                recording_started=now
                window.is_recording=True
                window.pid_phase='recording'
                window.pid_stabilization.update(achieved=True,elapsed_s=now-started,stable_s=5. if automatic else 0.,ias=float(data['airspeed_indicated']))
                window.sig_update_stage.emit(base.STAGE_RECORDING,'Airspeed stabilized; recording with PID active.' if automatic else 'Automatic recording started by operator; PID continues IAS hold.')
            elapsed=0. if recording_started is None else now-recording_started
            row=dict(time_s=control_elapsed,recording_time_s=elapsed,phase=window.pid_phase,target_ias=cfg['target_ias'],ias=data['airspeed_indicated'],target_pitch=target,pitch=pitch,command=current,actual_position=axes[0][0],ias_error=cfg['target_ias']-float(data['airspeed_indicated']),tracking_error=current-axes[0][0],pitch_error=target-pitch,bank_deg=data['bank_angle'],heading_deg=data['true_heading'],applied_force=axes[0][1],virtual_force=axes[0][2],pitch_kp=pid.kp,pitch_ki=pid.ki,pitch_kd=pid.kd,ias_kp=outer.kp,ias_ki=outer.ki,ias_kd=outer.kd,dt=dt)
            window.pid_live=row
            window.pid_diagnostics.append(row)
            if recording_started is None:
                tick=max(tick+.04,time.monotonic())
                window.stop_worker_event.wait(max(0.,tick-time.monotonic()))
                continue
            window.rec_time.append(elapsed)
            for values,pos,force in zip(axes,(window.p_pos_buf,window.r_pos_buf,window.y_pos_buf),(window.p_force_buf,window.r_force_buf,window.y_force_buf)):
                pos.append(values[0]);force.append(values[2])
            window.p3d_tele_buf.append(data)
            window.applied_force_buf.append(tuple(a[1] for a in axes));window.virtual_force_buf.append(tuple(a[2] for a in axes))
            window.replay_command_buf.append((elapsed,current,0.,0.,cfg['throttle']))

            if len(window.rec_time)==1: window.auto_t0_snapshot={'telemetry':dict(data),'control_method':'pid','bank_target_deg':0.,'yaw_initial_deg':0.}
            window.sig_record_progress.emit(int(elapsed))
            tick=max(tick+.04,time.monotonic())
            window.stop_worker_event.wait(max(0.,tick-time.monotonic()))
    except Exception as exc:
        failed=str(exc);window.pid_fault=failed
    finally:
        window.pid_controller_active=False
        window._external_control_active=False
        if window.stop_worker_event.is_set():
            failed=failed or "Stopped by user"
            window.pid_fault=failed
        errors=safe_release(window)
        if errors:
            failed=failed or '; '.join(errors)
            window.pid_fault=failed
        window.pid_fault=failed
        window.pid_ready=False
        window._prepared=False
        window._preparing=False
        window.is_recording=False
        window.pid_phase='failed' if failed else 'completed'
        context=getattr(window,'_session_context',{})
        context.update(pid_complete=not bool(failed),pid_fault=failed,pid_stabilization=getattr(window,'pid_stabilization',{}),pid_forwarding_check=getattr(window,'pid_forwarding_check',{}))
        context['pid_complete']=not bool(failed)
        context['pid_transport_source']='Original V4.1 CLS2Sim methods; shared QTG socket'
        context['pid_transport_trace']=list(getattr(device,'pid_transport_trace',[]))
        if context.get('status_file'):
            try:
                Path(context['status_file']).with_name(Path(context['status_file']).stem+'_pid_run.json').write_text(json.dumps(context,indent=2),encoding='utf-8')
            except OSError as exc:
                failed=failed or 'PID metadata save failed: '+str(exc);window.pid_fault=failed
        if window.rec_time: window.sig_recording_finished.emit()
        elif not failed: window.sig_update_stage.emit(base.STAGE_STOPPED_BY_USER,"PID stopped; no test recording was requested.")
        if failed: window.sig_update_stage.emit(base.STAGE_CONNECTION_ERROR,failed)

def report(csv_path,validation_path=None,run_fault=None,selected_graphs=None,target_ias=80):
    """PID validation is independent of replay references. Missing data never passes."""
    import pandas as pd
    import numpy as np
    path=Path(csv_path);frame=pd.read_csv(path)
    required=VALIDATION_COLUMNS
    outcome='NOT EVALUATED';details=[];reference=None
    if validation_path:
        reference=validate_dataset(validation_path)
        t=reference[required[0]].to_numpy(float)
        actual_t=frame[required[0]].to_numpy(float)
        if not len(actual_t) or actual_t[0]<t[0] or actual_t[-1]>t[-1]: raise ValueError('Validation dataset does not cover the recorded run.')
        passed=True
        for key in required[1:]:
            expected=np.interp(actual_t,t,reference[key].to_numpy(float))
            measured=frame[key].to_numpy(float)
            tolerance=3. if key==required[1] else np.maximum(abs(expected)*.05,100.)
            good=bool(np.all(np.isfinite(measured)&(abs(measured-expected)<=tolerance)))
            passed &= good;details.append(key+': '+('PASS' if good else 'FAIL'))
        outcome='PASS' if passed else 'FAIL'
    if run_fault:
        outcome='INTERRUPTED / NOT VALIDATED';details.append(str(run_fault))
    output=path.with_name(path.stem+'_graphs.pdf')
    # Reuse the existing report generator's portrait layout and graph groups.
    import qtg_common_engine
    renderer=qtg_common_engine.load_base().plot_graphs
    renderer._pid_report(path,output,reference,outcome,target_ias,selected_graphs)
    path.with_name(path.stem+'_evaluation.json').write_text(json.dumps({'control_method':'pid','validation_csv':validation_path,'result':outcome,'details':details},indent=2))
    return str(output)
