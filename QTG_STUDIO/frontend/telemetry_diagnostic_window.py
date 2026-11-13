"""Operator presentation only. Reads existing diagnostic files; no telemetry math."""
import json
import math
import os
from pathlib import Path
import time
import tkinter as tk
from tkinter import ttk, messagebox


GROUPS = {
    'Flight Information': [('ias_kt','Airspeed','kt'), ('ground_speed','Ground speed','kt'),
        ('alt_ft','Altitude MSL','ft'), ('heading_deg','Heading','°'), ('pitch_deg','Pitch','°'),
        ('bank_deg','Bank','°'), ('vertical_speed','Vertical speed','ft/min')],
    'Engine Information': [('engine_rpm','Engine speed','RPM'), ('prop_rpm','Propeller speed','RPM'),
        ('throttle','Throttle','%'), ('mixture','Mixture','%'), ('propeller_lever','Propeller lever','%'),
        ('manifold_pressure','Manifold pressure','psi')],
    'Aircraft Configuration': [('flaps','Flaps','handle index'), ('elevator_trim','Elevator trim','rad'),
        ('aileron_trim','Aileron trim','%'), ('rudder_trim','Rudder trim','%'),
        ('weight','Aircraft weight','kg'), ('total_fuel_quantity','Fuel quantity','kg'), ('cg','Centre of gravity','% MAC')],
    'Weather': [('temperature_c','Outside air temperature','°C'),
        ('ambient_pressure_mb','Atmospheric pressure','mbar'), ('barometer','Altimeter setting','hPa'),
        ('wind_speed','Wind speed','kt'), ('wind_direction','Wind direction','°')],
    'BRUNNER Controls': [('brunner.pitch','Physical pitch position','normalized'),
        ('brunner.roll','Physical roll position','normalized'), ('brunner.rudder','Physical rudder position','normalized')],
    'Additional Indications': [('pressure_altitude','Pressure altitude','ft'),
        ('indicated_altitude','Indicated altitude','ft'), ('sim_paused','Simulator paused','bool'),
        ('on_ground','Ground contact','bool'), ('lat','Latitude','°'), ('lon','Longitude','°')],
}
HELP = {'alt_ft': 'Height above mean sea level; not pressure altitude.',
        'manifold_pressure': 'Pressure in the engine intake manifold, in pounds per square inch.',
        'cg': 'Centre of gravity as a percentage of mean aerodynamic chord.',
        'elevator_trim': 'Elevator trim angle in radians, as supplied by the diagnostic.',
        'vertical_speed': 'Climb or descent rate. Positive values indicate a climb.',
        'pressure_altitude': 'Altitude referenced to standard pressure. Not substituted with MSL altitude.'}


def read_document(path):
    try:
        data = json.loads(Path(path).read_text(encoding='utf-8'))
        return data if isinstance(data, dict) else {}
    except (OSError, ValueError, TypeError):
        return {}


def presentation(data, *, connected=False, stale=False):
    """Display interpretation only; does not certify source validity or freshness."""
    rows = {row['channel']: row for row in data.get('measurements', [])
            if isinstance(row, dict) and isinstance(row.get('channel'), str)}
    running = data.get('active') is True and not stale
    available = any(isinstance(r.get('value'), (float, int)) and math.isfinite(r['value'])
                    and not k.startswith('brunner.') for k, r in rows.items())
    quality = 'Unverified' if available and connected and not stale else 'Unavailable'
    shown = {}
    for entries in GROUPS.values():
        for key, label, unit in entries:
            row = rows.get(key, {})
            value = row.get('value')
            usable = isinstance(value, (float, int)) and math.isfinite(value)
            # Retain historical samples with explicit labels; never present them as live.
            text = f'{value:,.3f}'.rstrip('0').rstrip('.') if usable else 'Not available'
            note = ('Rounded • may be stale' if key.startswith('brunner.') else
                    'Timing unverified') if usable else 'No validated reading'
            if usable and (stale or not connected or row.get('freshness') == 'STALE'):
                note = ('Rounded • may be stale • ' if key.startswith('brunner.') else '') + 'Last sample • not live'
            elif usable and not running:
                note = ('Rounded • may be stale • ' if key.startswith('brunner.') else '') + 'Saved sample • monitoring stopped'
            shown[key] = (text, row.get('unit') or unit, note, row)
    if stale:
        message = 'Monitoring updates have stopped arriving. Readings below are not live.'
    elif running and connected:
        message = 'Connected to P3D. Monitoring aircraft readings.'
    elif running:
        message = 'Monitoring is running. Waiting for aircraft data.'
    elif data:
        message = 'Monitoring stopped. Check the saved files below.'
    elif connected:
        message = 'Connected to P3D. Select Start Monitoring.'
    else:
        message = 'Connect to Simulator to begin. No QTG test will start.'
    if data.get('end_reason') == 'worker_shutdown':
        message = 'QTG worker shut down. Diagnostic monitoring stopped; earlier samples remain in the saved session.'
    timing_state = data.get('source_state', '')
    if running and connected:
        if timing_state == 'STALE_CACHE':
            message = 'Connected, but aircraft data has stopped updating. Values shown are older samples.'
            quality = 'Unavailable'
        elif timing_state == 'UNCHANGED_FRESHNESS_UNVERIFIED':
            message = 'Connected. Readings are temporarily unchanged; the simulator may be paused. Freshness is unverified.'
        elif 'CLOCK' in timing_state or 'TIMESTAMP' in timing_state:
            message = 'Connected. Aircraft readings are available, but their timing cannot be confirmed.'
    missing = sum(item[0] == 'Not available' for item in shown.values())
    return dict(running=running, quality=quality, readings=shown, message=message,
                warning=f'{missing} measurements are unavailable. Native acquisition timing is unverified.',
                issues=data.get('issues', []))


def confirmed_outputs(data):
    """A stopped status alone is not evidence of successful saving."""
    outputs = data.get('outputs') or {}
    if data.get('active') is not False or not outputs.get('csv') or not outputs.get('summary'):
        return None
    csv_path, summary = Path(outputs['csv']), Path(outputs['summary'])
    saved = read_document(summary)
    if (csv_path.is_file() and summary.is_file() and saved.get('active') is False
            and saved.get('outputs') == outputs):
        return csv_path, summary
    return None


class Tooltip:
    def __init__(self, widget, text):
        self.widget, self.text, self.popup = widget, text, None
        widget.bind('<Enter>', self.show)
        widget.bind('<Leave>', self.hide)
        widget.bind('<Destroy>', self.hide)

    def show(self, event=None):
        if self.popup: return
        self.popup = tk.Toplevel(self.widget)
        self.popup.overrideredirect(True)
        self.popup.geometry(f'+{self.widget.winfo_rootx()+12}+{self.widget.winfo_rooty()+25}')
        tk.Label(self.popup, text=self.text, wraplength=330, justify='left',
                 bg='#FFFBEB', fg='#0F172A', padx=10, pady=8).pack()

    def hide(self, event=None):
        if self.popup:
            self.popup.destroy()
            self.popup = None


class TelemetryDiagnosticWindow:
    def __init__(self, parent, session, *, theme, worker_state=lambda: {}):
        self.session, self.worker_state, self.theme = session, worker_state, theme
        self.pending = None
        self.saved = None
        self.timer = None
        self.closed = False
        self.window = tk.Toplevel(parent)
        self.window.title('QTG Studio | Aircraft Telemetry')
        self.window.geometry('1120x660')
        self.window.minsize(860, 540)
        self.window.configure(bg='#F1F5F9')
        top = tk.Frame(self.window, bg=theme['blue'], padx=20, pady=12)
        top.pack(fill='x')
        tk.Label(top, text='Aircraft Telemetry', font=('Segoe UI', 19, 'bold'),
                 fg='white', bg=theme['blue']).pack(anchor='w')
        tk.Label(top, text='Read-only diagnostic • Does not start a test or change aircraft controls',
                 font=('Segoe UI', 10), fg='white', bg=theme['blue']).pack(anchor='w')
        steps = ttk.Frame(self.window, padding=(16,10))
        steps.pack(fill='x')
        self.buttons = []
        for index, (label, action) in enumerate((('1  Connect to Simulator','connect'),
                ('2  Start Monitoring','start'), ('3  Stop & Save Data','stop'))):
            steps.columnconfigure(index, weight=1)
            button = ttk.Button(steps, text=label, command=lambda a=action:self.action(a))
            button.grid(row=0, column=index, sticky='ew', padx=4)
            self.buttons.append(button)
        self.step = tk.StringVar()
        ttk.Label(steps, textvariable=self.step, font=('Segoe UI', 10, 'bold')).grid(row=1,column=0,columnspan=3,sticky='w',pady=(7,0))
        status = tk.Frame(self.window, bg='white', padx=16, pady=8)
        status.pack(fill='x', padx=16)
        self.badges = {}
        for i, title in enumerate(('P3D Connection','Data Monitoring','Data Quality','Connection source')):
            status.columnconfigure(i, weight=1)
            cell = tk.Frame(status,bg='white')
            cell.grid(row=0,column=i,sticky='w',padx=6)
            tk.Label(cell,text=title,bg='white',fg=theme['dim'],font=('Segoe UI',9)).pack(anchor='w')
            badge = tk.Label(cell,bg='white',font=('Segoe UI',11,'bold'))
            badge.pack(anchor='w')
            self.badges[title] = badge
        self.message, self.warning = tk.StringVar(), tk.StringVar()
        ttk.Label(self.window,textvariable=self.message,font=('Segoe UI',11,'bold'),wraplength=820).pack(anchor='w',padx=22,pady=(9,2))
        ttk.Label(self.window,textvariable=self.warning,foreground=theme['amber'],wraplength=820).pack(anchor='w',padx=22,pady=(0,8))
        body = ttk.Frame(self.window)
        body.pack(fill='both',expand=True,padx=16,pady=(0,10))
        self.canvas = tk.Canvas(body,bg='#F1F5F9',highlightthickness=0)
        scroll = ttk.Scrollbar(body,orient='vertical',command=self.canvas.yview)
        scroll.pack(side='right',fill='y')
        self.canvas.pack(side='left',fill='both',expand=True)
        self.canvas.configure(yscrollcommand=scroll.set)
        content = tk.Frame(self.canvas,bg='#F1F5F9')
        item = self.canvas.create_window((0,0),window=content,anchor='nw')
        self.canvas.bind('<Configure>',lambda e:self.canvas.itemconfigure(item,width=e.width))
        content.bind('<Configure>',lambda e:self.canvas.configure(scrollregion=self.canvas.bbox('all')))
        self.window.bind('<MouseWheel>',lambda e:self.canvas.yview_scroll(int(-e.delta/120),'units'))
        content.columnconfigure((0,1),weight=1,uniform='cards')
        self.values, self.notes = {}, {}
        self.unit_labels = {}
        for index,(title, entries) in enumerate(GROUPS.items()):
            card = tk.LabelFrame(content,text=title,bg='white',fg=theme['blue'],
                                 font=('Segoe UI',11,'bold'),padx=12,pady=8)
            card.grid(row=index//2,column=index%2,sticky='nsew',padx=5,pady=5)
            card.columnconfigure(1,weight=1)
            for row,(key,label,unit) in enumerate(entries):
                caption=tk.Label(card,text=label,bg='white',fg=theme['text'],font=('Segoe UI',10))
                caption.grid(row=row*2,column=0,sticky='w',padx=(0,8))
                value=tk.StringVar(value='Not available')
                tk.Label(card,textvariable=value,bg='white',fg=theme['text'],font=('Segoe UI',16,'bold')).grid(row=row*2,column=1,sticky='e')
                unit_label = tk.Label(card,text=unit,bg='white',fg=theme['dim'],font=('Segoe UI',9))
                unit_label.grid(row=row*2,column=2,sticky='w',padx=(6,0))
                self.unit_labels[key] = unit_label
                note=tk.StringVar()
                tk.Label(card,textvariable=note,bg='white',fg=theme['dim'],font=('Segoe UI',8)).grid(row=row*2+1,column=0,columnspan=3,sticky='w',pady=(0,6))
                self.values[key], self.notes[key] = value,note
                Tooltip(caption, HELP.get(key,'Value supplied by the existing diagnostic. Connection does not prove freshness.'))
        files=ttk.LabelFrame(content,text='Saved diagnostic data',padding=12)
        files.grid(row=3,column=0,columnspan=2,sticky='ew',padx=5,pady=8)
        files.columnconfigure(1,weight=1)
        self.paths = {}
        for row,(key,label) in enumerate((('csv','CSV — recorded telemetry'),('summary','JSON — diagnostic summary'))):
            ttk.Label(files,text=label).grid(row=row,column=0,sticky='w',padx=(0,8),pady=4)
            variable=tk.StringVar(value='No saved file confirmed')
            ttk.Entry(files,textvariable=variable,state='readonly').grid(row=row,column=1,sticky='ew')
            button=ttk.Button(files,text='Copy Path',command=lambda k=key:self.copy_path(k),state='disabled')
            button.grid(row=row,column=2,padx=6)
            self.paths[key]=(variable,button)
        self.folder=ttk.Button(files,text='Open Data Folder',command=self.open_folder,state='disabled')
        self.folder.grid(row=2,column=0,sticky='w',pady=5)
        ttk.Button(content,text='Show / hide source details and all warnings',command=self.toggle_details).grid(row=4,column=0,columnspan=2,sticky='w',padx=5)
        self.details= tk.Text(content,height=14,wrap='word',font=('Segoe UI',9),state='disabled')
        self.details_text=''
        self.window.protocol('WM_DELETE_WINDOW',self.close)
        self.window.bind('<Destroy>',self._destroyed)
        self.refresh()

    @staticmethod
    def set_text(variable, value):
        if variable.get() != value: variable.set(value)

    def action(self, action):
        try:
            if action == 'connect': self.session.connect()
            else: self.session.command(action)
            self.pending=action
        except Exception as exc:
            messagebox.showerror('Telemetry diagnostic',str(exc),parent=self.window)
        self.refresh(reschedule=False)

    def refresh(self, reschedule=True):
        if self.closed: return
        data=read_document(self.session.status_file)
        transport=read_document(self.session.directory/'transport.json')
        stale=False
        if data:
            try: stale=time.time()-self.session.status_file.stat().st_mtime > 3
            except OSError: stale=True
        kind=self.session.kind
        connected=False
        source='Not connected'
        if kind == 'borrowed QTG worker':
            source='Existing Worker'
            state=self.worker_state()
            connected=bool(state.get('connected'))
        elif kind.startswith('borrowed dashboard'):
            source='Dashboard'
            client=self.session.dashboard
            snap=client.snapshot() if client.running else None
            connected=bool(snap is not None and snap.connected)
        elif kind != 'not connected':
            source='Diagnostic Connection'
            connected=bool(transport.get('connected')) and self.session.process.poll() is None
        model=presentation(data,connected=connected,stale=stale)
        running=model['running']
        pending=self.pending
        if pending == 'start' and running: self.pending=None
        if pending == 'stop' and data.get('active') is False: self.pending=None
        if pending == 'connect' and connected: self.pending=None
        self.buttons[0].configure(state='normal' if kind=='not connected' and pending is None else 'disabled')
        self.buttons[1].configure(state='normal' if connected and not running and self.pending not in ('start','stop') else 'disabled')
        self.buttons[2].configure(state='normal' if running or self.pending=='start' else 'disabled')
        step='Step 1 of 3 • Connect to Simulator' if not connected else ('Step 3 of 3 • Monitoring — stop when ready to save' if running else 'Step 2 of 3 • Start Monitoring')
        if self.pending=='stop': step='Step 3 of 3 • Stopping and saving…'
        self.set_text(self.step,step)
        for name,text,color in (
            ('P3D Connection','Connected' if connected else 'Disconnected','green' if connected else 'amber'),
            ('Data Monitoring','Running' if running else 'Stopped','green' if running else 'dim'),
            ('Data Quality',model['quality'],'amber'), ('Connection source',source,'blue')):
            self.badges[name].configure(text=text,fg=self.theme[color])
        failure=transport.get('error') or (stale and data.get('active') is True)
        if failure: self.badges['P3D Connection'].configure(fg=self.theme['red'])
        self.set_text(self.message,model['message'])
        self.set_text(self.warning,model['warning'])
        for key,(value,unit,note,row) in model['readings'].items():
            self.set_text(self.values[key],value)
            self.set_text(self.notes[key],note)
            self.unit_labels[key].configure(text=unit)
        self.saved=confirmed_outputs(data)
        for key,(variable,button) in self.paths.items():
            path=data.get('outputs',{}).get(key,'No saved file confirmed')
            self.set_text(variable,path)
            button.configure(state='normal' if self.saved else 'disabled')
        self.folder.configure(state='normal' if self.saved else 'disabled')
        if self.saved and not running and self.pending is None:
            self.set_text(self.message, ('QTG worker shut down. ' if data.get('end_reason') == 'worker_shutdown' else '') + 'Monitoring stopped. CSV and JSON files were created successfully.')
        elif data.get('active') is False and data.get('outputs') and not self.saved:
            self.set_text(self.message,'Monitoring stopped. Saving is not yet confirmed; check the file paths and warnings.')
        details=['Connection does not verify telemetry freshness. Native acquisition timestamps are unavailable.']
        if transport.get('error'): details.append(str(transport['error']))
        details += [str(issue) for issue in model['issues']]
        for key,(_,_,_,row) in model['readings'].items():
            if row: details.append(f"{key}: {row.get('source')} | {row.get('status')} | cache time={row.get('cache_copy_monotonic')} | native time={row.get('native_acquired_at')}")
        text='\n'.join(details)
        if text != self.details_text:
            self.details_text=text
            self.details.configure(state='normal'); self.details.delete('1.0','end')
            self.details.insert('1.0',text); self.details.configure(state='disabled')
        if reschedule: self.timer=self.window.after(500,self.refresh)

    def toggle_details(self):
        if self.details.winfo_manager(): self.details.grid_remove()
        else: self.details.grid(row=5,column=0,columnspan=2,sticky='ew',padx=5,pady=8)

    def copy_path(self, key):
        if not self.saved: return
        self.window.clipboard_clear()
        self.window.clipboard_append(str(self.saved[0 if key=='csv' else 1]))

    def open_folder(self):
        if not self.saved: return
        try: os.startfile(str(self.saved[0].parent))
        except OSError as exc: messagebox.showerror('Open Data Folder',str(exc),parent=self.window)

    def _destroyed(self,event):
        if event.widget is self.window: self.close(destroy=False)

    def close(self,destroy=True):
        if self.closed: return
        self.closed=True
        if self.timer: self.window.after_cancel(self.timer)
        try: self.session.command('close')
        finally:
            if destroy: self.window.destroy()
