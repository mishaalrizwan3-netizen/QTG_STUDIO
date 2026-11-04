"""Offline packaged-runtime checks; does not send simulator or BRUNNER commands."""
import ctypes
import hashlib
import importlib
import json
import os
from pathlib import Path
import tempfile
import traceback


def require(condition):
    if not condition:
        raise RuntimeError("Packaged runtime verification failed")


def run_self_test(app_module):
    result = {"ok": False, "checks": [], "frozen": bool(getattr(__import__('sys'), 'frozen', False))}
    app = None
    output = app_module.BASE / 'portable_self_test.json'
    try:
        for module in ['numpy', 'pandas', 'matplotlib', 'PIL', 'tkinter', 'PySide6.QtWidgets',
                       'SMK_Engine_From_Climb', 'climb_brunner_v15125', 'engine_report_v2', 'plot_graphs']:
            importlib.import_module(module)
            result['checks'].append('import: ' + module)
        from PySide6.QtWidgets import QApplication, QLabel
        qt_app = QApplication.instance() or QApplication([])
        qt_label = QLabel('QTG runtime check')
        require(not qt_label.grab().isNull())
        qt_label.close()
        result['checks'].append('Qt platform plugin and widget rendering')
        from qtg_position_setter import find_simconnect_dll, SIMCONNECT_DATA_INITPOSITION, SIMCONNECT_DATATYPE_INITPOSITION
        require(SIMCONNECT_DATATYPE_INITPOSITION == 12 and ctypes.sizeof(SIMCONNECT_DATA_INITPOSITION) == 56)
        client = ctypes.CDLL(str(find_simconnect_dll()))
        for name in ['Open', 'Close', 'SetDataOnSimObject', 'WeatherSetObservation', 'CallDispatch']:
            require(hasattr(client, 'SimConnect_' + name))
        require(hasattr(ctypes.CDLL(str(app_module.DLL_PATH)), 'QTG_GetState'))
        from p3d_simconnect_v15 import SimConnectClient
        require((app_module.MEIPASS / 'p3d_bridge.dll').is_file())
        result['checks'].append('native libraries and position ABI')
        import qtg_climb_pid
        inner,outer,slew=qtg_climb_pid.controller_types()
        require(inner().kp==.030 and outer().kp==.24)
        result['checks'].append('supplied V4.1 PID algorithms available without standalone GUI/bridge')
        app = app_module.App()
        app.withdraw()
        commands=[]
        launch=app._launch_common_test
        app._launch_common_test=lambda mode,spec: commands.append((mode,spec.reference))
        for index in range(len(app_module.TESTS)):
            if app_module.TESTS[index].reference=="1.c.(2)": continue
            app.select_test(index)
            require(not app.settings_panel.winfo_manager())
            app.btn_subtab_manual.command()
            app.btn_subtab_auto.command()
            require(commands[-2:]==[('manual',app_module.TESTS[index].reference),('auto',app_module.TESTS[index].reference)])
        app._launch_common_test=launch
        require(len(commands)==58)
        result['checks'].append('all 58 SEP GUI mode buttons route to the common BRUNNER engine')
        spec = app_module.TESTS[0]
        app.select_test(0)
        require(app._automatic_methods[spec.reference].get()=='Recorded Replay')
        app._automatic_methods[spec.reference].set('PID Automatic')
        require(app._pid_targets[spec.reference].get()=='80')
        app._automatic_methods[spec.reference].set('Recorded Replay')
        app._settings_unlocked = {spec.reference}
        app._build_settings_panel(spec)
        require(app.settings_panel.winfo_manager() == 'pack')
        fields, graphs = app._test_settings[spec.reference]
        require('Latitude[deg]' in fields and 'Heading[deg]' not in fields)
        require(len(graphs) == 25)
        fields['Alt[ft]'].set('')
        require(app._apply_test_settings(spec))
        require(fields['Alt[ft]'].get() == '3000')
        app._lock_test_settings(spec)
        require(not app.settings_panel.winfo_manager())
        result['checks'].append('all 29 SEP test screens, password panel, graph choices, defaults')
        state={'test_reference':spec.reference,'mode':'auto','stage':'Automatic blocked',
               'details':'Restored air data did not converge; replay remains paused.',
               'connected':True,'brunner_connected':True,'can_record':False,
               'duration_s':75,'elapsed':0,'throttle_text':'Throttle: 90.0 %',
               'engine_text':'Engine RPM: 2700','flight_text':'IAS: 101.6 kt | Altitude: 2951 ft',
               'attitude_text':'Pitch: -12.7 | Bank: -5.2 | Heading: 33.9',
               'controls_text':'Pitch: -0.04 | Roll: -0.03 | Rudder: 0.00',
               'recording_text':'Restored air data did not converge; replay remains paused.',
               'progress_text':'Preparing - no new samples recorded'}
        app._render_common_state(state)
        require('Restored air data' in app.common_status.cget('text'))
        require(app._common_view_vars['throttle'].get()=='Throttle: 90.0 %')
        require(str(app.common_record_button.cget('state'))=='disabled')
        state.update(stage='Connected & Ready to Record',details='',can_record=True)
        app._render_common_state(state)
        require(str(app.common_record_button.cget('state'))=='normal')
        require('Sim Flight Model Connected' in app._common_view_vars['connection'].get())
        app._common_states[spec.reference]=dict(state,elapsed=23,progress_text='23 seconds / 1000 samples')
        other_index=next(i for i,test in enumerate(app_module.TESTS) if test.reference not in (spec.reference,'1.c.(2)'))
        app.select_test(other_index)
        app._render_common_state(state)
        require(spec.reference not in app.common_status.cget('text'))
        require(float(app.common_progress['value'])==0)
        app.select_test(0)
        require(float(app.common_progress['value'])==23)
        result['checks'].append('per-test progress isolation and connection labels')
        from qtg_pid_monitor import PidMonitor
        monitor=PidMonitor(app,lambda:None,visible=False)
        monitor.update(dict(control_method='pid',stage='Acquiring target airspeed',controlling=True,pid_phase='acquiring',pid_live=dict(time_s=0.,target_ias=80.,ias=100.,target_pitch=5.,pitch=4.,command=.1,actual_position=.09,settled_s=0.)))
        require(monitor.app.snap_ias_actual==100.)
        require(hasattr(monitor.app,'engage_btn') and hasattr(monitor.app,'ias_kp_entry'))
        require(not hasattr(monitor.app,'worker_thread'))
        monitor.active=False;monitor.app.pid_enabled=False;monitor.close()
        result['checks'].append('original PID GUI loads gains and graphs without its independent hardware worker')
        result['checks'].append('embedded status/telemetry/progress and blocked/ready recording controls')
        original_base = app_module.BASE
        original_results = app_module.RESULTS_ROOT
        with tempfile.TemporaryDirectory() as temp:
            try:
                app_module.BASE = Path(temp)
                app_module.RESULTS_ROOT = Path(temp)
                spec.pdf_graphs = [('Airspeed (kt)', 'ias_kt')]
                spec.graph_csv_data = None
                path = app_module.generate_test_pdf(spec, 'Portable runtime check', [(0,80,3000,0,0),(1,81,3001,0,0)],1)
                require(path.read_bytes().startswith(b'%PDF') and path.stat().st_size > 5000)
                pid_csv=Path(temp)/'pid_auto.csv'
                pid_csv.write_text('Time (s),Indicated Airspeed (kts),Rate of Climb/Descent (fpm)\n0,80,500\n1,80,500\n')
                pid_pdf=qtg_climb_pid.report(pid_csv)
                require(Path(pid_pdf).read_bytes().startswith(b'%PDF'))
                require(json.loads(pid_csv.with_name('pid_auto_evaluation.json').read_text())['result']=='NOT EVALUATED')
                result['checks'].append('PID report without Manual recording; missing validation never passes')
            finally:
                app_module.BASE = original_base
                app_module.RESULTS_ROOT = original_results
        result['checks'].append('PDF generation from bundled runtime')
        result['ok'] = True
    except BaseException:
        result['error'] = traceback.format_exc()
    finally:
        if app is not None:
            app._on_close()
        output.write_text(json.dumps(result, indent=2), encoding='utf-8')
    return 0 if result['ok'] else 1
