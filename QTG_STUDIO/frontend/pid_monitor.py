"""Original supplied PID interface, with its hardware worker replaced by shared-engine commands."""
import ast
import collections
import csv
from datetime import datetime
import math
import os
from pathlib import Path
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk
from types import SimpleNamespace


def original_ui_type():
    from qtg_climb_pid import SOURCE,controller_types
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    folder=Path(sys._MEIPASS)/'pid_source' if getattr(sys,'frozen',False) else Path(__file__).resolve().parent.parent/'test_suites'/'normal_climb'/'pid_source'
    path=folder/SOURCE;tree=ast.parse(path.read_text(encoding='utf-8-sig'))
    cls=next(node for node in tree.body if isinstance(node,ast.ClassDef) and node.name=='PitchPIDApp')
    init=next(node for node in cls.body if isinstance(node,ast.FunctionDef) and node.name=='__init__')
    # Do not create its independent socket/native bridge or control thread.
    def worker_statement(node):
        text=ast.unparse(node)
        return text.startswith('self.worker_thread =') or text.startswith('self.worker_thread.start(')
    init.body=[node for node in init.body if not worker_statement(node)]
    inner,outer,slew=controller_types()
    namespace=dict(tk=tk,ttk=ttk,collections=collections,csv=csv,datetime=datetime,math=math,os=os,sys=sys,threading=threading,time=time,plt=plt,FigureCanvasTkAgg=FigureCanvasTkAgg,PitchPID=inner,AirspeedController=outer,slew_position=slew,SimConnectBridge=lambda:SimpleNamespace(),CLS2Sim=lambda:SimpleNamespace(),HOST='127.0.0.1',PORT=15090)
    exec(compile(ast.Module(body=[cls],type_ignores=[]),str(path),'exec'),namespace)
    return namespace['PitchPIDApp']

class PidMonitor:
    def __init__(self,parent,stop,visible=True,send=None):
        self.stop=stop;self.send=send or (lambda action,**values:None)
        self.window=tk.Toplevel(parent)
        if not visible:self.window.withdraw()
        original=original_ui_type();owner=self
        self.ready=False;self.recording=False
        class SharedInterface(original):
            def toggle_pid(self):
                if not owner.ready:
                    self.last_fault="Prepare PID Automatic before engaging Airspeed Hold."
                    return
                if self.pid_enabled:
                    owner.stop();self.pid_enabled=False
                else:
                    owner.send('pid_engage',**owner.parameters())
                    self.pid_enabled=True
            def gui_tick(self):
                super().gui_tick()
                if self.running:
                    self.log_badge.configure(text='QTG CSV: recording' if owner.recording else 'QTG CSV: operator starts recording')
            def on_close(self):owner.close()
            def open_logs_folder(self):
                if owner.data_directory:os.startfile(owner.data_directory)
        style=ttk.Style(parent);old_theme=style.theme_use()
        import matplotlib.pyplot as plt
        with plt.rc_context():self.app=SharedInterface(self.window)
        style.theme_use(old_theme)
        self.app._shared_ready=True
        self.active=False;self.data_directory=''
        self.window.title('Original PID V4.1 - Shared QTG Engine')
        self.window.protocol('WM_DELETE_WINDOW',self.close)
        self._last_parameters=None
        self._sync_id=self.window.after(100,self.sync_parameters)

    def parameters(self):
        app=self.app
        return dict(target_ias=app.target_ias,pitch=dict(kp=app.pid.kp,ki=app.pid.ki,kd=app.pid.kd),ias=dict(kp=app.ias_ctrl.kp,ki=app.ias_ctrl.ki,kd=app.ias_ctrl.kd),invert=app.pid.invert)

    def sync_parameters(self):
        if not self.window.winfo_exists():return
        values=self.parameters()
        if self.app.pid_enabled and values!=self._last_parameters:
            self.send('pid_tune',**values)
        self._last_parameters=values
        self._sync_id=self.window.after(100,self.sync_parameters)

    def close(self):
        if self.active or self.app.pid_enabled:self.stop()
        self.app.running=False
        for token in (self.app.after_id,self._sync_id):
            if token:
                try:self.window.after_cancel(token)
                except tk.TclError:pass
        import matplotlib.pyplot as plt
        plt.close(self.app.fig)
        self.window.destroy()

    def update(self,state):
        if not self.window.winfo_exists():return
        app=self.app;live=state.get('pid_live',{});tele=state.get('telemetry',{})
        self.ready=bool(state.get('pid_ready'))
        app.engage_btn.configure(state='normal' if self.ready else 'disabled')
        self.active=bool(state.get('controlling')) and state.get('control_method')=='pid'
        if state.get('stage') in ('Automatic blocked','Connection error','Stopped by user','Completed','Closed'):app.pid_enabled=False
        self.recording=bool(state.get('recording'))
        self.data_directory=state.get('data_directory','')
        with app.lock:
            app.snap_connected=bool(state.get('connected'))
            app.snap_brunner_connected=bool(state.get('brunner_connected'))
            app.snap_paused=bool(state.get('pid_paused',True))
            app.snap_telemetry_age=0
            app.snap_measured_hz=0. if not live.get('dt') else 1./live['dt']
            app.snap_pitch_actual=float(live.get('pitch',-float(tele.get('pitch_angle',0))))
            app.snap_pitch_target=float(live.get('target_pitch',0))
            app.snap_pitch_error=app.snap_pitch_target-app.snap_pitch_actual
            app.snap_ias_actual=float(live.get('ias',tele.get('airspeed_indicated',0)))
            app.snap_ias_target=float(live.get('target_ias',app.target_ias))
            app.snap_ias_error=app.snap_ias_target-app.snap_ias_actual
            app.snap_cmd_pitch=app.snap_pitch_target
            app.snap_brunner_pos=float(live.get('actual_position',0))
            app.brunner_command=float(live.get('command',0))
            app.snap_throttle=float(tele.get('engine_throttle_percent',0))
            app.snap_weight=float(tele.get('total_weight',float('nan')))*.45359237
            app.snap_temp=float(tele.get('ambient_temp',float('nan')))
            app.last_fault=state.get('details','')
        app.log_badge.configure(text='QTG CSV: recording' if state.get('recording') else 'QTG CSV: operator starts recording')
