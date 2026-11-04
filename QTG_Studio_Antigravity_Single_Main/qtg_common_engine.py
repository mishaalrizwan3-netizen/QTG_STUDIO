"""One QTG launch path; initialization/recording/replay stay in the brunner codebase.

The Qt runtime uses a separate process because QTG Studio uses Tk. Explicit
module loading prevents its newer P3D ABI from mixing with the legacy suite.
"""
import csv
import importlib.util
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time


RESULTS_ROOT = Path("D:/QTG_Test_Results")


def brunner_root():
    if getattr(sys, 'frozen', False):
        root = Path(sys._MEIPASS)/'brunner_base'
    else:
        root = Path(__file__).resolve().parent.parent/'brunner'
    if not (root/'Super_Mushak_Logger_GUI.py').is_file():
        raise FileNotFoundError('The shared brunner test code was not found: ' + str(root))
    return root


def load_base():
    if 'qtg_brunner_base' in sys.modules:
        return sys.modules['qtg_brunner_base']
    root = brunner_root()
    def load(name, path):
        spec = importlib.util.spec_from_file_location(name, path)
        module = importlib.util.module_from_spec(spec)
        sys.modules[name] = module
        spec.loader.exec_module(module)
        return module
    for name in ['replay_support','brunner_interface','p3d_simconnect_v15','plot_graphs']:
        load(name, root/'src'/(name+'.py'))
    base = load('qtg_brunner_base', root/'Super_Mushak_Logger_GUI.py')
    base.RESOURCE_DIR = str(root)
    base.SRC_DIR = str(root/'src')
    return base


def validate_reference(path):
    path = Path(path).resolve()
    aliases = [('Pitch_Manual_Pos','Pitch_Man_Pos','Pitch_Pos','Joystick Pitch Position (mm)'),
               ('Roll_Manual_Pos','Roll_Man_Pos','Roll_Pos','Joystick Roll Position (mm)'),
               ('Rudder_Manual_Pos','Rudder_Man_Pos','Rudder_Pos','Rudder Pedal Position (mm)'),
               ('Throttle 1 Position (%)','Throttle_Manual','Throttle')]
    with path.open(newline='', encoding='utf-8-sig') as stream:
        reader=csv.DictReader(stream)
        names=set(reader.fieldnames or [])
        if 'Time (s)' not in names or any(not names.intersection(group) for group in aliases):
            raise ValueError('Choose a Manual recording from the BRUNNER base, containing time, pitch, roll, rudder and throttle measurements.')
        previous=None
        count=0
        for row in reader:
            stamp=float(row['Time (s)'])
            values=[float(next(row[key] for key in group if key in row and row[key].strip())) for group in aliases]
            if not all(math.isfinite(value) for value in [stamp]+values) or (previous is not None and stamp < previous):
                raise ValueError('Manual reference contains invalid measurements or backwards timestamps.')
            if not 0 <= values[-1] <= 100:
                raise ValueError('Manual reference throttle must be between 0 and 100 percent.')
            previous=stamp
            count+=1
    if count<2: raise ValueError('Manual reference needs at least two recorded samples.')
    return str(path)


def test_directory(root, reference, title):
    import re
    name=re.sub(r'[^A-Za-z0-9]+','_',reference+' '+title).strip('_')
    return Path(root).resolve()/name


def named_result(filename, reference, title):
    import re
    label=test_directory(Path('.'),reference,title).name
    return re.sub(r'^Super_Mushak_(manual|auto)_takeoff_data_',lambda m:label+'_'+m.group(1)+'_',filename)


def validate_conditions(values):
    if values is None:return None
    result=dict(values)
    limits={'alt_ft':(-1500,60000),'ias_kt':(0,400),'lat':(-90,90),'lon':(-180,180),'flaps':(0,10),'throttle':(0,100),'temperature_c':(-100,60)}
    for key,(lo,hi) in limits.items():
        value=float(result[key])
        if not math.isfinite(value) or not lo<=value<=hi:raise ValueError('Invalid initial condition: '+key)
        result[key]=value
    if result['flaps']!=int(result['flaps']):raise ValueError('Flaps must be a whole-number handle index.')
    result['on_ground']=bool(result.get('on_ground',False))
    return result

def apply_manual_conditions(window,values):
    """Apply operator settings once while paused; do not warm up or reposition in flight."""
    cfg=validate_conditions(values);sim=window.p3d
    if not sim or not sim.is_connected():raise RuntimeError('Sim Flight Model disconnected.')
    sim.pause_sim()
    if not sim.wait_for_paused(3.):raise RuntimeError('Simulator did not confirm pause.')
    if window.stop_worker_event.is_set():raise RuntimeError('Manual initialization cancelled.')
    if not sim.set_initial_flaps(cfg['flaps']):raise RuntimeError('Initial flaps rejected.')
    from replay_support import reference_weather_metar
    metar=reference_weather_metar({'altitude_msl':cfg['alt_ft'],'oat':cfg['temperature_c']},sim.get_global_weather_metar())
    if metar and not sim.set_global_weather_metar(metar):raise RuntimeError('Initial temperature rejected.')
    if not sim.queue_position_airborne(alt_ft=cfg['alt_ft'],speed_kts=cfg['ias_kt'],pitch_deg=0.,bank_deg=0.,hdg_deg=0.,lat=cfg['lat'],lon=cfg['lon'],on_ground=cfg['on_ground']):raise RuntimeError('Initial position rejected.')
    window._wait_for_initial_altitude(cfg['alt_ft'])
    # INITPOSITION and aircraft reload handling can reset power. Apply it after
    # position readback, then wait for fresh telemetry; never drive it in flight.
    if not sim.set_initial_throttle(cfg['throttle']):raise RuntimeError('Initial throttle rejected.')
    checks=[('pressure_alt',cfg['alt_ft'],25),('plane_latitude',cfg['lat'],.001),('plane_longitude',cfg['lon'],.001),('measured_flaps_handle_index',cfg['flaps'],.1),('engine_throttle_percent',cfg['throttle'],1)]
    deadline=time.monotonic()+3.
    while True:
        if window.stop_worker_event.is_set():raise RuntimeError('Manual initialization cancelled.')
        actual=window._refresh_paused_telemetry()
        mismatches=[]
        for key,target,tolerance in checks:
            measured=float(actual.get(key,float('nan')))
            if not math.isfinite(measured) or abs(measured-target)>tolerance:
                mismatches.append(f'{key}: requested {target:g}, actual {measured:g}')
        if not mismatches:break
        if time.monotonic()>=deadline:
            hint=' Check the physical throttle position and simulator throttle-axis assignments.' if any('engine_throttle_percent' in item for item in mismatches) else ''
            raise RuntimeError('Manual initial-condition mismatch: '+ '; '.join(mismatches)+'.'+hint)
        window.stop_worker_event.wait(.05)
    return actual


def launch(mode, spec, output_dir, reference=None, duration_s=None, control_method="replay", pid_conditions=None, validation_csv=None, initial_conditions=None, selected_graphs=None):
    if mode not in ('manual','auto'): raise ValueError('Unknown test mode')
    duration_s=validate_duration(75 if duration_s is None else duration_s)
    initial_conditions=validate_conditions(initial_conditions)
    brunner_root()
    if control_method not in ('replay','pid'): raise ValueError('Unknown Automatic control method')
    if control_method=='pid' and spec.reference!='1.c.(1)': raise ValueError('PID is supported only for normal climb.')
    if mode=='auto' and control_method=='replay': reference=validate_reference(reference)
    if control_method=='pid':
        from qtg_climb_pid import options
        pid_conditions=options(pid_conditions)
        if validation_csv:
            from qtg_climb_pid import validate_dataset
            validate_dataset(validation_csv)
    output=Path(output_dir).resolve()
    output.mkdir(parents=True,exist_ok=True)
    token=f'{time.time_ns()}'
    context={'test_reference':spec.reference,'title':spec.title,'mode':mode,
             'initial_conditions':initial_conditions,'selected_graphs':selected_graphs,'reference_csv':reference, 'control_method':control_method, 'pid_conditions':pid_conditions, 'validation_csv':validation_csv, 'duration_s':duration_s, 'command_directory':str(output/(token+'_commands')), 'status_file':str(output/(token+'_status.json')),
             'stop_file':str(output/(token+'_stop')), 'results_root':str(output.parent), 'result_directory':str(test_directory(output.parent,spec.reference,spec.title)),
             'initialization_policy':'operator settings for Manual/PID; selected Manual t0 for Recorded Replay'}
    context_path=output/(token+'_context.json')
    context_path.write_text(json.dumps(context,indent=2),encoding='utf-8')
    args=[sys.executable]
    if not getattr(sys,'frozen',False): args.append(str(Path(__file__).resolve().parent/'main.py'))
    args+=['--brunner-test',str(context_path)]
    env=os.environ.copy()
    env['PYINSTALLER_RESET_ENVIRONMENT']='1'
    env['QT_QPA_PLATFORM']='offscreen'
    process=subprocess.Popen(args,cwd=str(output.parent),env=env,creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
    return process,context


def configure_test_extension(window, reference):
    """Load only an explicitly mapped test adapter, never alter BRUNNER modules."""
    for name,original in getattr(window,'_test_extension_originals',{}).items():
        setattr(window,name,original)
    window._test_extension_originals={}
    folders={'1.f.(1)':'1_f_1_Engine_acceleration','1.f.(2)':'1_f_2_Engine_deceleration'}
    if reference not in folders:return
    root=(Path(sys._MEIPASS) if getattr(sys,'frozen',False) else Path(__file__).resolve().parent.parent)/'test_suites'
    def read(name,path):
        spec=importlib.util.spec_from_file_location(name,path)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        return module
    policy=read('qtg_engine_test_policy',root/folders[reference]/'execution.py')
    if policy.TEST_REFERENCE!=reference:raise RuntimeError('Test initialization policy mismatch.')
    if policy.USE_SETTLED_REFERENCE_ALTIMETER:
        read('qtg_engine_initialization_adapter',root/'_shared/engine_initialization.py').install(window)


def initialize_mode(window, mode, reference=None):
    """Dispatch only after both connections are available. No duplicate prep code."""
    if not window.p3d or not window.p3d.is_connected() or not window.brunner.sock:
        raise RuntimeError('P3D and BRUNNER must both be connected before initialization.')
    window._prepared=False
    window._preparing=True
    context=getattr(window,'_session_context',{})
    configure_test_extension(window,context.get('test_reference'))
    if context.get('test_reference') in ('1.f.(1)','1.f.(2)'):
        # Engine tests use the user's BRUNNER initialization verbatim. Keep
        # application-specific PID/condition preparation out of this pathway.
        base=window._base_module
        if mode=='manual':
            entry=base.SuperMushakLoggerApp.start_manual_test
        elif mode=='auto':
            if getattr(window,'_control_method','replay')!='replay':
                raise ValueError('Engine tests support Recorded Replay only.')
            window.selected_ref_csv=validate_reference(reference)
            entry=base.SuperMushakLoggerApp.start_auto_test_only
        else:
            raise ValueError('Unknown test mode')
        context['initialization_source']={
            'codebase':'brunner', 'entry_point':entry.__name__,
            'module':str(Path(base.__file__).resolve())}
        entry(window)
        return
    if mode=='manual': window.start_manual_test()
    elif mode=='auto' and getattr(window,'_control_method','replay')=='pid':
        import qtg_climb_pid
        qtg_climb_pid.prepare(window,window._base_module,window._session_context)
    elif mode=='auto':
        window.selected_ref_csv=validate_reference(reference)
        window.start_auto_test_only()
    else: raise ValueError('Unknown test mode')


def validate_duration(value):
    number=float(value)
    if not math.isfinite(number) or number != int(number) or not 1 <= number <= 3600:
        raise ValueError('Duration must be a whole number from 1 to 3600 seconds.')
    return int(number)


def send_command(context, action, **values):
    if action not in ('initialize','record','stop','shutdown','pid_engage','pid_tune'):
        raise ValueError('Unknown engine command')
    directory=Path(context['command_directory'])
    directory.mkdir(parents=True,exist_ok=True)
    target=directory/f'{time.time_ns():020d}.json'
    temporary=target.with_suffix('.tmp')
    temporary.write_text(json.dumps(dict(values,action=action)),encoding='utf-8')
    temporary.replace(target)


def snapshot(window, context):
    def label(name):
        widget=getattr(window,name,None)
        return widget.text() if widget is not None else ''
    telemetry={}
    for key,value in getattr(window,'latest_telemetry',{}).items():
        if isinstance(value,(int,float)):
            telemetry[key]=value if math.isfinite(value) else None
    return dict(context,stage=getattr(window,'current_stage','Connecting'),
        details=getattr(window,'_stage_details',''),updated=time.time(),
        connected=bool(window.p3d and window.p3d.is_connected()),
        brunner_connected=bool(window.brunner.sock),
        connection_text=label('lbl_conn_status'),throttle_text=label('lbl_throttle_val'),
        engine_text=label('lbl_engine_rpm'),flight_text=label('lbl_ias_alt'),
        attitude_text=label('lbl_pitch_bank'),controls_text=label('lbl_brunner_sticks'),
        recording_text=label('lbl_rec_status'),progress_text=label('lbl_rec_progress'),
        elapsed=window.rec_time[-1] if window.rec_time else 0,
        samples=len(window.rec_time),duration_s=window.target_duration_sec,
        pid_ready=getattr(window,'pid_ready',False),pid_paused=getattr(window,'pid_paused',True),pid_phase=getattr(window,'pid_phase',''),pid_live=getattr(window,'pid_live',{}),
        controlling=bool(window.is_recording or getattr(window,'pid_controller_active',False)),recording=bool(window.is_recording and (getattr(window,'_control_method','replay')!='pid' or getattr(window,'pid_phase','')=='recording')),can_record=bool(getattr(window,'_control_method','replay')!='pid' and getattr(window,'_prepared',False)
            and window.current_stage=='Connected & Ready to Record' and not window.is_recording
            and window.p3d and window.p3d.is_connected() and window.brunner.sock) or bool(not getattr(window,'pid_automatic_workflow',False) and getattr(window,'pid_controller_active',False) and not window.is_recording and not getattr(window,'pid_paused',True)),
        data_directory=str(window.data_dir),outputs=list(getattr(window,'_outputs',[])),telemetry=telemetry)


def worker_class(base,context,publish):
    """Adapt the existing base methods to an invisible runtime and an external UI."""
    class BackgroundEngine(base.SuperMushakLoggerApp):
        def __init__(self):
            self._base_module=base
            self._session_context=context
            self._control_method=context.get('control_method','replay')
            self.control_method=self._control_method
            self.validation_csv=context.get('validation_csv')
            self.manual_initial_conditions=context.get('initial_conditions')
            self.report_graphs=context.get('selected_graphs')
            self.report_initial_conditions=context.get('initial_conditions')
            self._stage_details=''
            self._outputs=[]
            self._prepared=False
            self._preparing=False
            self._shutting_down=False
            super().__init__()
            self.target_duration_sec=validate_duration(context.get('duration_s',75))
            self.hide()
            if context.get("result_directory"):
                self.data_dir=Path(context["result_directory"])
                self.data_dir.mkdir(parents=True,exist_ok=True)

        def _publish(self): publish(snapshot(self,context))

        def _update_stage_ui(self,stage,details=''):
            if self._control_method=='pid' and stage==base.STAGE_CONNECTED_READY and self.stop_worker_event.is_set(): return
            self._stage_details=details
            if stage==base.STAGE_CONNECTED_READY and self._preparing:
                self._prepared=True
                self._preparing=False
            elif stage in (base.STAGE_AUTO_BLOCKED,base.STAGE_CONNECTION_ERROR,base.STAGE_STOPPED_BY_USER):
                self._prepared=False
                self._preparing=False
            super()._update_stage_ui(stage,details)
            self._publish()

        def handle_command(self,command):
            action=command['action']
            if action in ('pid_engage','pid_tune') and getattr(self,'pid_automatic_workflow',False):
                raise RuntimeError('PID Automatic is managed by the climb execution engine.')
            if action in ('pid_engage','pid_tune'):
                if self._control_method!='pid' or not getattr(self,'pid_ready',False): raise RuntimeError('Prepare PID Automatic first.')
                if action=='pid_engage' and getattr(self,'pid_controller_active',False): return
                worker=getattr(self,'worker_thread',None)
                if action=='pid_engage' and worker is not None and worker.is_alive(): raise RuntimeError('Wait for the previous controller to stop.')
                from qtg_climb_pid import options,CLIMB_PROFILE
                import copy,threading
                cfg=options(dict(self.pid_conditions,target_ias=float(command.get('target_ias',80))))
                profile=copy.deepcopy(CLIMB_PROFILE)
                for group in ('pitch','ias'):
                    for key in ('kp','ki','kd'):
                        value=float(command.get(group,{}).get(key,profile[group][key]))
                        if not math.isfinite(value) or not 0<=value<=5: raise ValueError('Invalid PID gain')
                        profile[group][key]=value
                profile['pitch']['invert']=bool(command.get('invert',False))
                self.pid_conditions=cfg;self.pid_profile=profile
                context["pid_profile"]=profile;context["pid_conditions"]=cfg
                if action=='pid_tune': return
                self.pid_controller_active=True;self._external_control_active=True
                self.pid_record_requested=False;self.is_recording=False;self.pid_paused=True
                self.pid_diagnostics=[];self.pid_fault=None;self.pid_phase='holding'
                for name in ('rec_time','p_pos_buf','r_pos_buf','y_pos_buf','p_force_buf','r_force_buf','y_force_buf','p3d_tele_buf','applied_force_buf','virtual_force_buf','replay_command_buf'):getattr(self,name).clear()
                self.stop_worker_event.clear()
                self._update_stage_ui('PID IAS hold','Operator controls unpause and recording. PID is active.')
                self.worker_thread=threading.Thread(target=self._auto_replay_loop,daemon=True);self.worker_thread.start()
                return
            if action=='record' and self._control_method=='pid' and getattr(self,'pid_automatic_workflow',False):
                self._stage_details='Recording starts automatically after airspeed stabilization.';self._publish();return
            if action=='record' and self._control_method=='pid':
                if not getattr(self,'pid_controller_active',False) or getattr(self,'pid_paused',True):
                    self._stage_details='Unpause the simulator and engage PID before recording.';self._publish();return
                if abs(float(getattr(self,'pid_live',{}).get('ias',float('inf')))-self.pid_conditions['target_ias'])>3:
                    self._stage_details='Maintain target IAS within 3 kt before starting Automatic Recording.';self._publish();return
                self.pid_record_requested=True
                return
            if action=='record':
                # The outer UI may request recording only when the base is ready.
                # All execution, including arm_recording, belongs to the base class.
                if not self._prepared or self.current_stage!=base.STAGE_CONNECTED_READY or self.is_recording:
                    self._stage_details='Recording remains blocked until initialization is verified.'
                    self._publish()
                    return
                if not self.p3d or not self.p3d.is_connected() or not self.brunner.sock:
                    self._update_stage_ui(base.STAGE_CONNECTION_ERROR,'P3D or BRUNNER disconnected.')
                    return
                self._prepared=False
                self.arm_recording()
            elif action in ('stop','shutdown'):
                self._prepared=False
                self.stop_test()
                if self._control_method=='pid':
                    from qtg_climb_pid import watchdog
                    try: watchdog(self.brunner,0)
                    except Exception: pass
                    self.brunner.strict_io=False
                if action=='shutdown': self._shutting_down=True
            elif action=='initialize':
                prep=getattr(self,'preparation_thread',None)
                worker=getattr(self,'worker_thread',None)
                if getattr(self,'current_stage',None)=='Generating reports' or self.is_recording or getattr(self,'pid_controller_active',False) or (prep is not None and prep.is_alive()) or (worker is not None and worker.is_alive()):
                    self._stage_details='Stop the active test and wait for preparation to finish before initializing again.'
                    self._publish()
                    return
                mode=command['mode']
                method=command.get('control_method','replay')
                if method not in ('replay','pid'): raise ValueError('Unknown Automatic control method')
                if method=='pid' and command.get('test_reference',context['test_reference'])!='1.c.(1)': raise ValueError('PID is supported only for normal climb.')
                reference=validate_reference(command['reference_csv']) if mode=='auto' and method=='replay' else None
                duration=validate_duration(command.get('duration_s',75))
                if mode not in ('manual','auto'): raise ValueError('Unknown test mode')
                if method=='pid':
                    from qtg_climb_pid import options,validate_dataset
                    options(command.get('pid_conditions'))
                    if command.get('validation_csv'): validate_dataset(command['validation_csv'])
                conditions=validate_conditions(command.get('initial_conditions'))
                # Set Conditions already prepares Manual mode. Selecting Manual again
                # must not teleport the aircraft or repeat hardware preparation.
                reuse_manual=(mode=='manual' and context.get('mode')=='manual'
                    and command.get('test_reference',context['test_reference'])==context['test_reference']
                    and conditions is not None and conditions==getattr(self,'manual_initial_conditions',None)
                    and getattr(self,'_prepared',False)
                    and getattr(self,'current_stage',None)==getattr(base,'STAGE_CONNECTED_READY',None)
                    and self.p3d is not None and self.p3d.is_connected() and bool(self.brunner.sock)
                    and not command.get('force_initialize',False))
                self.manual_initial_conditions=conditions;self.report_initial_conditions=conditions
                self.report_graphs=command.get('selected_graphs')
                context.update(initial_conditions=conditions,selected_graphs=self.report_graphs)
                self._control_method=method
                self.control_method=method
                self.validation_csv=command.get('validation_csv')
                context.update(control_method=method,pid_conditions=command.get('pid_conditions'),validation_csv=command.get('validation_csv'),mode=mode,reference_csv=reference,test_reference=command.get('test_reference',context['test_reference']),
                               title=command.get('title',context['title']),duration_s=duration)
                context["result_directory"]=str(test_directory(context["results_root"],context["test_reference"],context["title"]))
                self.data_dir=Path(context["result_directory"])
                self.data_dir.mkdir(parents=True,exist_ok=True)
                self.target_duration_sec=duration
                self._outputs=[]
                if reuse_manual:
                    self._stage_details='Using initial conditions already applied. Ready for Manual recording; aircraft was not repositioned.'
                    self._publish()
                else:
                    initialize_mode(self,mode,reference)

        def _auto_replay_loop(self):
            if self._control_method=='pid':
                from qtg_climb_pid import loop
                return loop(self,base)
            return super()._auto_replay_loop()

        def _on_ui_timer_tick(self):
            # Keep the existing status refresh, but do not hide PID feedback failures.
            try:
                if self._control_method=='pid' and not getattr(self,'pid_automatic_workflow',False) and self._prepared and not self.is_recording and getattr(self,'pid_controller_active',False):
                    from qtg_climb_pid import watchdog
                    watchdog(self.brunner)
                super()._on_ui_timer_tick()
            except Exception as exc:
                if self._control_method=='pid':
                    self.stop_test()
                    self._update_stage_ui(base.STAGE_CONNECTION_ERROR,str(exc))
                else: raise
            self._publish()

        def _save_csv_and_generate_reports(self):
            before={p for p in self.data_dir.iterdir() if p.is_file()}
            self.current_stage='Generating reports'
            self.lbl_rec_status.setText('Generating CSV and PDF reports...')
            self._publish()
            # Reports are opened explicitly by QTG Studio, never by the invisible engine.
            original_open=getattr(os,'startfile',None)
            if original_open is not None: os.startfile=lambda *args,**kwargs: None
            try: super()._save_csv_and_generate_reports()
            finally:
                if original_open is not None: os.startfile=original_open
            if getattr(self,'_control_method','replay')=='pid':
                diagnostics=self.data_dir/(test_directory(self.data_dir,context['test_reference'],context['title']).name+'_pid_diagnostics_'+str(time.time_ns())+'.csv')
                import csv
                rows=getattr(self,'pid_diagnostics',[])
                if rows:
                    with diagnostics.open('w',newline='') as stream:
                        writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
                context['pid_forwarding_check']=getattr(self,'pid_forwarding_check',{})
                context['pid_fault']=getattr(self,'pid_fault',None)
                context['pid_stabilization']=getattr(self,'pid_stabilization',{})
                context['pid_complete']=not bool(context['pid_fault'])
                # Preserve existing columns and append PID-specific measurements.
                recording_rows=[row for row in rows if row.get('phase')=='recording']
                for path in self.data_dir.glob('*.csv'):
                    if path in before or path==diagnostics:continue
                    with path.open(newline='',encoding='utf-8-sig') as stream:
                        reader=csv.DictReader(stream);fields=list(reader.fieldnames or []);samples=list(reader)
                    extra={'PID Target IAS (kts)':'target_ias','PID Command (-)':'command','PID Tracking Error (-)':'tracking_error','BRUNNER Pitch Applied Force (raw)':'applied_force'}
                    if len(samples)!=len(recording_rows):raise RuntimeError('PID diagnostic/recording sample alignment failed.')
                    for sample,row in zip(samples,recording_rows):
                        sample.update({label:row[key] for label,key in extra.items()})
                    with path.open('w',newline='',encoding='utf-8') as stream:
                        writer=csv.DictWriter(stream,fieldnames=fields+list(extra));writer.writeheader();writer.writerows(samples)
            import shutil
            for path in list(self.data_dir.iterdir()):
                if path.is_file() and path not in before and path.suffix in ('.csv','.json','.pdf'):
                    target=Path(context['result_directory'])/named_result(path.name,context['test_reference'],context['title'])
                    if getattr(self,'_control_method','replay')=='pid':
                        target=target.with_name(target.name.replace('_auto_','_pid_auto_',1))
                    if target.resolve()!=path.resolve():
                        if target.exists(): raise FileExistsError('Result already exists: '+str(target))
                        path.rename(target)
                    self._outputs.append(str(target))
                    if target.suffix=='.csv':
                        target.with_suffix('.qtg_test.json').write_text(json.dumps(context,indent=2),encoding='utf-8')
            if getattr(self,'_control_method','replay')=='pid':
                evaluations=[Path(path) for path in self._outputs if path.endswith('_evaluation.json')]
                if evaluations:
                    context['evaluation_result']=json.loads(evaluations[-1].read_text()).get('result')
                else:
                    context['evaluation_result']='NOT EVALUATED: report generation failed'
                if not any(path.endswith('.pdf') for path in self._outputs):
                    self.current_stage=base.STAGE_CONNECTION_ERROR
                    self._stage_details='PID data saved, but report generation failed. Check validation CSV and diagnostics.'
            if self.current_stage=='Generating reports': self.current_stage=base.STAGE_COMPLETED
            if getattr(self,'_control_method','replay')=='pid' and getattr(self,'pid_fault',None):
                self.current_stage=base.STAGE_CONNECTION_ERROR
                self._stage_details='Incomplete PID test: '+self.pid_fault
            if any(path.lower().endswith('.pdf') for path in self._outputs) and not (getattr(self,'_control_method','replay')=='pid' and getattr(self,'pid_fault',None)):
                self.lbl_rec_status.setText('COMPLETED - CSV and PDF reports saved')
            self._publish()

        def finish_shutdown(self):
            self.stop_worker_event.set()
            for thread in (self.worker_thread,getattr(self,'preparation_thread',None)):
                if thread and thread.is_alive(): return False
            self.ui_timer.stop()
            if self.p3d: self.p3d.stop()
            self.brunner.close()
            self.current_stage='Closed'
            self._stage_details='P3D paused and BRUNNER controls released.'
            self._publish()
            return True
    return BackgroundEngine


def run(context_path):
    # Force all Qt rendering off-screen before constructing QApplication.
    os.environ['QT_QPA_PLATFORM']='offscreen'
    context=json.loads(Path(context_path).read_text(encoding='utf-8'))
    status_path=Path(context['status_file'])
    def publish(payload):
        temporary=status_path.with_suffix('.tmp')
        temporary.write_text(json.dumps(payload,allow_nan=False),encoding='utf-8')
        try: temporary.replace(status_path)
        except PermissionError:
            # The Tk reader may briefly hold the file on Windows. Publish next tick.
            temporary.unlink(missing_ok=True)
    stream=open(status_path.with_suffix('.log'),'a',encoding='utf-8',buffering=1)
    sys.stdout=sys.stderr=stream
    base=load_base()
    from PySide6.QtCore import QTimer
    from PySide6.QtWidgets import QApplication
    app=QApplication(sys.argv)
    app.setQuitOnLastWindowClosed(False)
    class SilentProgress:
        def __init__(self,*args): pass
        def __getattr__(self,name): return lambda *args,**kwargs: None
    class InlineMessages:
        @staticmethod
        def warning(parent,title,message): parent._update_stage_ui(base.STAGE_AUTO_BLOCKED,message)
        @staticmethod
        def critical(parent,title,message): parent._update_stage_ui(base.STAGE_CONNECTION_ERROR,message)
    base.QProgressDialog=SilentProgress
    base.QMessageBox=InlineMessages
    window=worker_class(base,context,publish)()
    started=time.monotonic()
    pending={'initial':True}
    timer=QTimer(window)
    def tick():
        try:
            if Path(context['stop_file']).exists():
                Path(context['stop_file']).unlink(missing_ok=True)
                pending['initial']=False
                window.handle_command({'action':'stop'})
            directory=Path(context['command_directory'])
            for path in sorted(directory.glob('*.json')) if directory.exists() else []:
                command=json.loads(path.read_text(encoding='utf-8'))
                path.unlink()
                pending['initial']=False
                window.handle_command(command)
            if window._shutting_down:
                if window.finish_shutdown(): timer.stop(); app.quit()
                return
            if pending['initial']:
                if window.p3d and window.p3d.is_connected() and window.brunner.sock:
                    pending['initial']=False
                    initialize_mode(window,context['mode'],context['reference_csv'])
                elif time.monotonic()-started>10:
                    pending['initial']=False
                    window._update_stage_ui(base.STAGE_CONNECTION_ERROR,'Could not connect to both P3D and BRUNNER. Check Prepar3D and CLS2Sim.')
        except Exception as exc:
            if getattr(window,'pid_controller_active',False):
                window.stop_test()
                window.pid_controller_active=False
            window._update_stage_ui(base.STAGE_CONNECTION_ERROR,str(exc))
    timer.timeout.connect(tick)
    timer.start(100)
    return app.exec()


def self_test(destination):
    """Verify the actual invisible worker, driver and commands without hardware."""
    import traceback
    from types import SimpleNamespace
    result={'ok':False}
    try:
        os.environ['QT_QPA_PLATFORM']='offscreen'
        base=load_base()
        from PySide6.QtWidgets import QApplication
        app=QApplication.instance() or QApplication([])
        context={'mode':'manual','test_reference':'1.c.(1)','title':'Offline runtime check','duration_s':75}
        states=[]
        cls=worker_class(base,context,states.append)
        execution_methods=['start_manual_test','start_auto_test_only','arm_recording',
            '_manual_recording_loop','_restore_reference_state',
            '_prepare_reference_power','_wait_for_reference_rpm','_refresh_restored_airdata']
        for name in execution_methods:
            if getattr(cls,name) is not getattr(base.SuperMushakLoggerApp,name):
                raise RuntimeError('BRUNNER execution method was overridden: '+name)
        originals={name:getattr(base.SuperMushakLoggerApp,name) for name in ['_start_hardware_connections','_play_beep','arm_recording','stop_test']}
        base.SuperMushakLoggerApp._start_hardware_connections=lambda self:None
        base.SuperMushakLoggerApp._play_beep=lambda *args:None
        try:
            window=cls()
            window.ui_timer.stop()
            if window.isVisible() or any(widget.isVisible() for widget in QApplication.topLevelWidgets()):
                raise RuntimeError('The background runtime displayed a window')
            calls=[]
            window.p3d=SimpleNamespace(is_connected=lambda:True,stop=lambda:None)
            window.brunner=SimpleNamespace(sock=object(),close=lambda:None)
            window._prepared=True
            window.current_stage=base.STAGE_CONNECTED_READY
            base.SuperMushakLoggerApp.arm_recording=lambda self:calls.append('record')
            window.handle_command({'action':'record'})
            if calls!=['record']:raise RuntimeError('Record command was not dispatched')
            window._prepared=False
            window.current_stage=base.STAGE_AUTO_BLOCKED
            window.handle_command({'action':'record'})
            if calls!=['record']:raise RuntimeError('Blocked recording was allowed')
            base.SuperMushakLoggerApp.stop_test=lambda self:calls.append('stop')
            window.handle_command({'action':'stop'})
            window.handle_command({'action':'shutdown'})
            if calls!=['record','stop','stop'] or not window._shutting_down:
                raise RuntimeError('Stop/shutdown command dispatch failed')
            window.p3d=None
            window.brunner=SimpleNamespace(sock=None,close=lambda:None)
            if not window.finish_shutdown(): raise RuntimeError('Worker did not cleanly shut down')
            window.close()
        finally:
            for name,value in originals.items():setattr(base.SuperMushakLoggerApp,name,value)
        client=base.SimConnectClient()
        if client.bridge_version<1515:raise RuntimeError('Wrong BRUNNER initialization bridge')
        if getattr(sys,'frozen',False) and not Path(client._dll_path).is_relative_to(Path(sys._MEIPASS)):
            raise RuntimeError('BRUNNER bridge loaded outside bundle')
        result.update(ok=True,bridge_version=client.bridge_version,bridge=client._dll_path,
            base=base.__file__,p3d_driver=sys.modules['p3d_simconnect_v15'].__file__,
            hardware_connected=False,execution_methods_identical=execution_methods,visible_windows=0,record_command=True,blocked_recording=True,
            stop_command=True,shutdown_command=True,live_status=bool(states))
    except Exception:result['error']=traceback.format_exc()
    Path(destination).write_text(json.dumps(result,indent=2),encoding='utf-8')
    return 0 if result['ok'] else 1
