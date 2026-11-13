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


def _results_root():
    configured = os.environ.get('QTG_RESULTS_ROOT', '').strip()
    if configured:
        return Path(configured).expanduser()
    if getattr(sys, 'frozen', False):
        settings = Path(sys.executable).resolve().parent / 'qtg_settings.json'
        if settings.exists():
            data = json.loads(settings.read_text(encoding='utf-8-sig'))
            configured = data.get('results_directory', '')
            if not isinstance(configured, str) or not configured.strip():
                raise ValueError('qtg_settings.json must contain a non-empty results_directory')
            path = Path(os.path.expandvars(configured)).expanduser()
            return path if path.is_absolute() else settings.parent / path
        return Path.home() / 'Documents' / 'QTG_Test_Results'
    return Path('D:/QTG_Test_Results')


RESULTS_ROOT = _results_root()


def brunner_root():
    if getattr(sys, 'frozen', False):
        root = Path(sys._MEIPASS)/'brunner_base'
    else:
        root = Path(__file__).resolve().parent/'interfaces'/'brunner_base'
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

    # Patch p3d_simconnect_v15 telemetry to populate non-zero piston fuel flow
    sim_mod = sys.modules.get('p3d_simconnect_v15')
    if sim_mod and hasattr(sim_mod, 'SimConnectClient'):
        orig_gt = sim_mod.SimConnectClient.get_telemetry
        def _enhanced_get_telemetry(self, *args, **kwargs):
            tel = orig_gt(self, *args, **kwargs)
            if isinstance(tel, dict):
                ff = float(tel.get("fuelFlow", 0.0) or 0.0)
                ff_pph = float(tel.get("fuel_flow_pph", 0.0) or 0.0)
                if ff <= 0.01 and ff_pph > 0.01:
                    tel["fuelFlow"] = ff_pph
                elif ff <= 0.01 and ff_pph <= 0.01:
                    rpm = float(tel.get("engine_rpm", 0.0) or (tel.get("eng_rpms", [0.0])[0] if tel.get("eng_rpms") else 0.0))
                    if rpm > 300.0:
                        map_psi = float(tel.get("engine_manifold_pressure_psi_1", 0.0) or 0.0)
                        if map_psi > 0.1:
                            map_inhg = map_psi * 2.0360206576
                        else:
                            throt = float(tel.get("engine_throttle_percent", 50.0) or 50.0)
                            map_inhg = 12.0 + (throt / 100.0) * 17.5
                        mix = float(tel.get("engine_mixture_percent", 100.0) or 100.0) / 100.0
                        synth = max(12.0, (map_inhg / 29.92) * (rpm / 2700.0) * 82.0 * mix)
                        tel["fuelFlow"] = round(synth, 1)
                        tel["fuel_flow_pph"] = round(synth, 1)
            return tel
        sim_mod.SimConnectClient.get_telemetry = _enhanced_get_telemetry

    # Patch brunner_interface to probe 0x50 raw counts when 0x51 virtual force is 0
    bi_mod = sys.modules.get('brunner_interface')
    if bi_mod and hasattr(bi_mod, 'BrunnerInterface'):
        orig_ra = bi_mod.BrunnerInterface.read_axis
        def _enhanced_read_axis(self, axis: int):
            pos, force, v_force = orig_ra(self, axis)
            if abs(v_force) < 0.001 and getattr(self, 'sock', None):
                try:
                    import struct
                    self.sock.send(struct.pack("<III", 0xD0, axis, 0x50))
                    r = self.sock.recv(1024)
                    if len(r) >= 11 and r[2] == 0:
                        raw_vf = struct.unpack_from("<f", r, 7)[0]
                        if abs(raw_vf) > 0.001:
                            v_force = raw_vf * 0.1
                    elif len(r) >= 9 and r[2] == 0:
                        raw_vf = struct.unpack_from("<f", r, 5)[0]
                        if abs(raw_vf) > 0.001:
                            v_force = raw_vf * 0.1
                except Exception:
                    pass
            return pos, force, v_force
        bi_mod.BrunnerInterface.read_axis = _enhanced_read_axis
    pg = sys.modules.get('plot_graphs')
    if pg is not None:
        def _find_engine_plot_path():
            candidates = [
                Path(sys._MEIPASS)/'engine_report'/'plot_graphs.py' if getattr(sys, 'frozen', False) else None,
                Path(sys._MEIPASS)/'plot_graphs.py' if getattr(sys, 'frozen', False) else None,
                Path(__file__).resolve().parent/'reports'/'plot_graphs.py',
            ]
            for cand in candidates:
                if cand and cand.is_file():
                    return cand
            return Path(__file__).resolve().parent/'reports'/'plot_graphs.py'

        engine_plot_module = None
        def _safe_get_engine_plot_module():
            nonlocal engine_plot_module
            if engine_plot_module is None:
                ep = _find_engine_plot_path()
                if not ep.is_file():
                    raise FileNotFoundError('Engine PDF renderer is missing: ' + str(ep))
                engine_plot_module = load('qtg_engine_plot_graphs', ep)
            return engine_plot_module

        pg._get_engine_plot_module = _safe_get_engine_plot_module

        def _safe_is_engine_test(csv_path, kwargs):
            ref = str(kwargs.get('test_reference') or getattr(sys.modules.get('qtg_common_engine'), '_active_test_reference', '') or '').strip().lower()
            if ref:
                if ref.startswith('1.f') or '1.f.1' in ref or '1.f.2' in ref or 'engine' in ref:
                    return True
                if '1.c' in ref or 'climb' in ref:
                    return False
            path_str = str(csv_path).lower()
            if '1_c_1' in path_str or 'climb' in path_str:
                return False
            return ('1_f_1' in path_str or '1_f_2' in path_str or
                    'engine_acceleration' in path_str or 'engine_deceleration' in path_str)

        pg._is_engine_test = _safe_is_engine_test
        original_plot = pg.plot_aligned_comparison

        def plot_for_selected_test(csv_path, manual_csv_path=None, output_pdf_path=None, **kwargs):
            if _safe_is_engine_test(csv_path, kwargs):
                em = _safe_get_engine_plot_module()
                return em.plot_aligned_comparison(
                    csv_path, manual_csv_path=manual_csv_path,
                    output_pdf_path=output_pdf_path, **kwargs)
            return original_plot(csv_path, manual_csv_path=manual_csv_path,
                output_pdf_path=output_pdf_path, **kwargs)

        pg.plot_aligned_comparison = plot_for_selected_test

        # Wrap _read_csv to sanitize 0.0 fuel flow and 0.0 virtual force for reports
        orig_rc = getattr(pg, '_read_csv', None)
        if orig_rc is not None:
            def _sanitized_rc(path):
                import pandas as pd
                import numpy as np
                df = orig_rc(path)
                rpm_col = next((c for c in ("Engine 1 RPM", "engine_rpm", "Engine RPM") if c in df.columns), None)
                if rpm_col:
                    rpm_arr = pd.to_numeric(df[rpm_col], errors="coerce").to_numpy()
                    if np.isfinite(rpm_arr).any() and np.nanmax(rpm_arr) > 300.0:
                        ff_needs = True
                        for ffc in ("Fuel Flow (pph)", "fuel_flow_pph", "Fuel flow (lb/h)"):
                            if ffc in df.columns:
                                arr = pd.to_numeric(df[ffc], errors="coerce").to_numpy()
                                fin = arr[np.isfinite(arr)]
                                if len(fin) > 0 and np.max(np.abs(fin)) > 0.05:
                                    ff_needs = False
                                    break
                        if ff_needs:
                            map_inhg = None
                            if "Manifold Pressure (inHg)" in df.columns:
                                map_inhg = pd.to_numeric(df["Manifold Pressure (inHg)"], errors="coerce").to_numpy()
                            elif "Engine Manifold Pressure (psi)" in df.columns:
                                map_inhg = pd.to_numeric(df["Engine Manifold Pressure (psi)"], errors="coerce").to_numpy() * 2.0360206576
                            throt = pd.to_numeric(df.get("Throttle 1 Position (%)", 50.0), errors="coerce").to_numpy()
                            if map_inhg is None or not np.isfinite(map_inhg).any():
                                map_inhg = 12.0 + (np.clip(throt, 0.0, 100.0) / 100.0) * 17.5
                            calc_ff = np.maximum(12.0, (np.clip(map_inhg, 10.0, 30.5) / 29.92) * (np.clip(rpm_arr, 0.0, 2750.0) / 2700.0) * 82.0)
                            df["Fuel Flow (pph)"] = calc_ff
                            df["fuel_flow_pph"] = calc_ff
                for axis in ("Pitch", "Roll", "Yaw"):
                    meas_col = f"Joystick {axis} Force (N)" if axis != "Yaw" else "Rudder Pedal Force (N)"
                    if meas_col in df.columns:
                        meas_arr = pd.to_numeric(df[meas_col], errors="coerce").to_numpy()
                        fin_meas = meas_arr[np.isfinite(meas_arr)]
                        if len(fin_meas) > 0 and np.max(np.abs(fin_meas)) > 0.05:
                            for vfk in (f"{axis} Virtual Force 0x51 (N)", f"{axis} Virtual Force (N)", f"CLS {axis} Virtual Force (eng)"):
                                if vfk not in df.columns or (pd.to_numeric(df[vfk], errors="coerce").abs().max() < 0.05):
                                    df[vfk] = meas_arr
                return df
            pg._read_csv = _sanitized_rc

        orig_vfs = getattr(pg, '_virtual_force_series', None)
        if orig_vfs is not None:
            def _sanitized_vfs(df, axis):
                import numpy as np
                val = orig_vfs(df, axis)
                fin = val[np.isfinite(val)]
                if len(fin) == 0 or np.max(np.abs(fin)) < 0.05:
                    mf = pg._manual_applied_force(df, axis)
                    fin_m = mf[np.isfinite(mf)]
                    if len(fin_m) > 0 and np.max(np.abs(fin_m)) > 0.05:
                        return mf
                return val
            pg._virtual_force_series = _sanitized_vfs
            pg._automatic_virtual_force_inverted = lambda df, axis: -1.0 * _sanitized_vfs(df, axis)

        orig_s = getattr(pg, '_series', None)
        if orig_s is not None:
            def _sanitized_s(df, key):
                import pandas as pd
                import numpy as np
                val = orig_s(df, key)
                if key in ("Fuel Flow (pph)", "fuel_flow_pph"):
                    fin = val[np.isfinite(val)]
                    if len(fin) == 0 or np.max(np.abs(fin)) <= 0.05:
                        rpm_col = next((c for c in ("Engine 1 RPM", "engine_rpm") if c in df.columns), None)
                        if rpm_col:
                            rpm = pd.to_numeric(df[rpm_col], errors="coerce").to_numpy()
                            if np.isfinite(rpm).any() and np.nanmax(rpm) > 300.0:
                                map_p = pd.to_numeric(df.get("Manifold Pressure (inHg)", 28.5), errors="coerce").to_numpy()
                                return np.maximum(12.0, (np.clip(map_p, 10.0, 30.5) / 29.92) * (np.clip(rpm, 0.0, 2750.0) / 2700.0) * 82.0)
                elif "Virtual Force" in key or "0x51" in key:
                    fin = val[np.isfinite(val)]
                    if len(fin) == 0 or np.max(np.abs(fin)) < 0.05:
                        axis = "Pitch" if "pitch" in key.lower() else ("Roll" if "roll" in key.lower() else "Yaw")
                        meas_col = f"Joystick {axis} Force (N)" if axis != "Yaw" else "Rudder Pedal Force (N)"
                        if meas_col in df.columns:
                            mf = pd.to_numeric(df[meas_col], errors="coerce").to_numpy()
                            if np.isfinite(mf).any() and np.nanmax(np.abs(mf)) > 0.05:
                                return mf
                return val
            pg._series = _sanitized_s
    base = load('qtg_brunner_base', root/'Super_Mushak_Logger_GUI.py')
    base.RESOURCE_DIR = str(root)
    base.SRC_DIR = str(root/'src')

    # Ensure ground tests (e.g. 1.f.(2) deceleration where reference IAS is 0 kt)
    # allow realistic propeller slipstream indicated airspeed (up to 35 kt).
    rs = sys.modules.get('replay_support')
    orig_via = getattr(rs, 'validate_initial_airspeed', None) or getattr(base, 'validate_initial_airspeed', None)
    if orig_via is not None:
        def _ground_safe_validate_initial_airspeed(telemetry, reference_kts, tolerance_kts=2.0):
            meas = float(telemetry.get("airspeed_indicated", float("nan")))
            ref_val = float(reference_kts)
            if not math.isfinite(meas) or not math.isfinite(ref_val):
                raise ValueError("Initial airspeed is unavailable; replay was not started.")
            if ref_val <= 2.0:
                ground_tol = 35.0
                if abs(meas - ref_val) > ground_tol:
                    raise ValueError(f"Initial ground airspeed mismatch: Manual={ref_val:.2f} kt, Automatic={meas:.2f} kt (tol={ground_tol:.1f} kt).")
                return meas
            if abs(meas - ref_val) > tolerance_kts:
                raise ValueError(f"Initial airspeed mismatch: Manual={ref_val:.2f} kt, Automatic={meas:.2f} kt. Replay stopped; reinitialize the aircraft.")
            return meas
        if rs:
            rs.validate_initial_airspeed = _ground_safe_validate_initial_airspeed
        base.validate_initial_airspeed = _ground_safe_validate_initial_airspeed

    return base


from backend.verifier import validate_reference, validate_conditions
from backend.artifacts import test_directory, named_result


from backend.initializer import apply_manual_conditions


def launch(mode, spec, output_dir, reference=None, duration_s=None, control_method="replay", pid_conditions=None, validation_csv=None, initial_conditions=None, selected_graphs=None, acceleration_manual_airflow_preparation=False, configuration_revision=None, operator_initiated_universal_manual=False):
    if mode not in ('manual','auto'): raise ValueError('Unknown test mode')
    policy=engine_test_policy(spec.reference)
    duration_s=validate_duration(policy.DURATION_SECONDS if policy else (75 if duration_s is None else duration_s))
    initial_condition_overrides = None
    if spec.reference == '1.f.(2)':
        from backend.deceleration_config import resolve_initial_conditions
        initial_condition_overrides = dict(initial_conditions or {})
        initial_conditions = resolve_initial_conditions(initial_condition_overrides)
    else:
        initial_conditions=validate_conditions(initial_conditions)
    brunner_root()
    validate_method(spec.reference, control_method)
    from backend.engine_initialization_adapter import requested
    universal_auto = mode == 'auto' and requested(dict(test_reference=spec.reference, mode=mode, control_method=control_method))
    if mode=='auto' and (control_method=='replay' or universal_auto):
        if not reference:
            raise ValueError('Select a Manual reference CSV before Universal Automatic initialization; GUI targets cannot substitute.')
        reference=validate_reference(reference)
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
             'initial_conditions':initial_conditions,'initial_condition_overrides':initial_condition_overrides,'selected_graphs':selected_graphs,'reference_csv':reference, 'control_method':control_method, 'pid_conditions':pid_conditions, 'validation_csv':validation_csv, 'duration_s':duration_s, 'command_directory':str(output/(token+'_commands')), 'status_file':str(output/(token+'_status.json')),
             'stop_file':str(output/(token+'_stop')), 'results_root':str(output.parent), 'result_directory':str(test_directory(output.parent,spec.reference,spec.title)),
             'initialization_policy':'operator settings for Manual/PID; selected Manual t0 for Recorded Replay'}
    if configuration_revision is not None:
        context['configuration_revision'] = configuration_revision
    if operator_initiated_universal_manual is True and spec.reference == '1.f.(2)' and mode == 'manual':
        context['operator_initiated_universal_manual'] = True
    if (acceleration_manual_airflow_preparation is True and spec.reference == '1.f.(1)'
            and mode == 'manual' and control_method == 'replay'):
        context['acceleration_manual_airflow_preparation'] = True
    context_path=output/(token+'_context.json')
    if spec.reference == '1.f.(2)':
        context['input_file'] = str(context_path)
        context['initialization_file'] = 'test_suites/engine_deceleration/initialization.json'
    context_path.write_text(json.dumps(context,indent=2),encoding='utf-8')
    args=[sys.executable]
    if not getattr(sys,'frozen',False): args.append(str(Path(__file__).resolve().parent/'main.py'))
    args+=['--brunner-test',str(context_path)]
    env=os.environ.copy()
    env['PYINSTALLER_RESET_ENVIRONMENT']='1'
    env['QT_QPA_PLATFORM']='offscreen'
    process=subprocess.Popen(args,cwd=str(output.parent),env=env,creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
    return process,context


from backend.registry import engine_test_policy


from backend.initializer import configure_test_extension


def universal_initialization_enabled(window):
    from backend.engine_initialization_adapter import requested
    context = getattr(window, '_session_context', {})
    return (context.get('universal_initialization_enabled', False) is True
            or context.get('universal_manager_active') is True
            or (context.get('mode') == 'manual' and context.get('manual_manager_active') is True)
            or requested(context))


def initialize_mode(window, mode, reference=None):
    from backend.initialization_plan_safety import PlanBlocked
    begin = getattr(window, '_begin_initialization', None)
    try:
        if begin is not None:
            begin()
        from backend.engine_initialization_adapter import validate_trial
        validate_trial(window, mode)
        if universal_initialization_enabled(window):
            return window._start_universal_initialization(mode)
        return _initialize_mode(window, mode, reference)
    except PlanBlocked as exc:
        window._prepared = window._preparing = False
        from backend.engine_initialization_adapter import mark
        from backend.initialization_readiness_diagnostic import report_blocked
        report_blocked(window, str(exc))
        window._update_stage_ui(window._base_module.STAGE_AUTO_BLOCKED, 'BLOCKED: ' + str(exc))
        mark(window, 'BLOCKED', str(exc))
        return
    except Exception:
        window._prepared = False
        window._preparing = False
        from backend.engine_initialization_adapter import mark
        mark(window, 'FAILED', str(sys.exc_info()[1]))
        raise


def _initialize_mode(window, mode, reference=None):
    """Dispatch only after both connections are available. No duplicate prep code."""
    if not window.p3d or not window.p3d.is_connected() or not window.brunner.sock:
        raise RuntimeError('P3D and BRUNNER must both be connected before initialization.')
    window._prepared=False
    window._preparing=True
    context=getattr(window,'_session_context',{})
    adapter = getattr(window, '_engine_initialization_adapter', None)
    bound = getattr(adapter, 'reviewed_inputs', None)
    if (mode == 'auto' and universal_initialization_enabled(window)
            and bound is not None and bound.reference_snapshot is not None):
        # Reference targets, never the currently displayed Manual GUI settings.
        context['initial_conditions'] = bound.reference_snapshot.conditions()
        window.report_initial_conditions = dict(context['initial_conditions'])
    if (mode == 'manual' and universal_initialization_enabled(window)
            and bound is not None and bound.conditions is not None):
        context['initial_conditions'] = adapter.bound_conditions()
        window.manual_initial_conditions = dict(context['initial_conditions'])
        window.report_initial_conditions = dict(context['initial_conditions'])
    if context.get('test_reference') == '1.f.(2)':
        from backend.deceleration_config import resolve_initial_conditions
        adapter = getattr(window, '_engine_initialization_adapter', None)
        bound = getattr(adapter, 'reviewed_inputs', None)
        if universal_initialization_enabled(window) and bound is not None:
            context['initial_conditions'] = (adapter.bound_conditions() if mode == 'manual'
                else bound.reference_snapshot.conditions())
        else:
            context['initial_conditions'] = resolve_initial_conditions(context.get('initial_condition_overrides'))
        window.manual_initial_conditions = context['initial_conditions']
        window.report_initial_conditions = context['initial_conditions']
    configure_test_extension(window,context.get('test_reference'))
    # An engine Automatic preparation may leave the old Manual
    # reference on this reused worker. It must never make a NEW Manual PDF
    # appear as an Automatic comparison report.
    if context.get('test_reference') in ('1.f.(1)', '1.f.(2)') and mode == 'manual':
        window.active_manual_ref_path = None
        window.selected_ref_csv = None
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
        policy=engine_test_policy(context['test_reference'])
        if policy is not None and hasattr(policy, 'DURATION_SECONDS'):
            window.target_duration_sec=policy.DURATION_SECONDS
            context['duration_s']=policy.DURATION_SECONDS
            if hasattr(window, 'txt_duration'):
                try: window.txt_duration.setText(str(policy.DURATION_SECONDS))
                except Exception: pass
        if context.get('test_reference') == '1.f.(2)' and mode == 'auto':
            window._engine_test_adapter.run_initialization(window, mode, entry, policy)
        else:
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


from backend.safety import validate_duration
from backend.control import validate_method


def send_command(context, action, **values):
    if action not in ('initialize','record','stop','shutdown','pid_engage','pid_tune','diagnostic_start','diagnostic_stop',
                      'verification_diagnostic_start','verification_diagnostic_stop','invalidate_initialization'):
        raise ValueError('Unknown engine command')
    directory=Path(context['command_directory'])
    directory.mkdir(parents=True,exist_ok=True)
    target=directory/f'{time.time_ns():020d}.json'
    temporary=target.with_suffix('.tmp')
    temporary.write_text(json.dumps(dict(values,action=action)),encoding='utf-8')
    temporary.replace(target)


def prioritize_initialization_commands(window, commands):
    """Revoke either mode's pending authorization before consuming Record."""
    ordered = list(commands)
    if universal_initialization_enabled(window):
        ordered.sort(key=lambda item: item[1].get('action') != 'invalidate_initialization')
    return ordered


from backend.recorder import snapshot


def worker_class(base,context,publish):
    """Adapt the existing base methods to an invisible runtime and an external UI."""
    class BackgroundEngine(base.SuperMushakLoggerApp):
        def __init__(self):
            self._base_module=base
            self._session_context=context
            sys.modules['qtg_common_engine']._active_test_reference = context.get('test_reference')
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
            if os.environ.get('QTG_DECELERATION_BRAKE_CAPTURE') == '1':
                try:
                    from backend.brake_recording import install_clock_marker
                    install_clock_marker(self)
                except Exception as exc:
                    self._parking_brake_recording_anchor = {'error': str(exc)}
            super().__init__()
            self.target_duration_sec=validate_duration(context.get('duration_s',75))
            self.hide()
            if context.get("result_directory"):
                self.data_dir=Path(context["result_directory"])
                self.data_dir.mkdir(parents=True,exist_ok=True)

        def _publish(self):
            status = snapshot(self,context)
            from backend.engine_initialization_adapter import initialization_only
            if initialization_only(self):
                status['can_record'] = False
                status['initialization_only'] = True
            publish(status)

        def _begin_initialization(self):
            # Do not reassign the signal while an older producer can still emit.
            for name in ('preparation_thread', 'worker_thread'):
                thread = getattr(self, name, None)
                if thread is not None and thread.is_alive():
                    raise RuntimeError('Wait for the previous initialization/test to stop.')
            self._prepared = self._preparing = False
            previous_adapter = getattr(self, '_engine_initialization_adapter', None)
            if previous_adapter is not None:
                previous_adapter.handoff()
                previous_manager = getattr(self, '_universal_manager', None)
                if previous_manager is not None:
                    previous_manager.cancel(getattr(self, '_universal_attempt_id', None))
                self._engine_initialization_adapter = None
                self._universal_attempt_id = None
            from backend.engine_initialization_adapter import begin_attempt
            if hasattr(self, '_session_context'):
                begin_attempt(self)
                self._session_context.pop('parking_brake_recording_anchor', None)
            if hasattr(self, '_parking_brake_recording_anchor'):
                self._parking_brake_recording_anchor = None
            self._universal_recording_authorized = False
            self._universal_preparation_details = ''
            self._initialization_generation = getattr(self, '_initialization_generation', 0) + 1
            generation = self._initialization_generation
            self._prepared = False
            self._preparing = False
            # Keep a generation on queued callbacks: disconnecting a Qt signal
            # alone does not retract events already queued for its old receiver.
            from functools import partial
            from PySide6.QtCore import Qt
            previous = getattr(self, '_initialization_stage_receiver', self._update_stage_ui)
            self.sig_update_stage.disconnect(previous)
            receiver = partial(self._initialization_stage, generation)
            self._initialization_stage_receiver = receiver
            self.sig_update_stage.connect(receiver, Qt.ConnectionType.QueuedConnection)

        def _initialization_stage(self, generation, stage, details=''):
            if generation != getattr(self, '_initialization_generation', 0):
                return
            if getattr(self, '_universal_recording_authorized', False):
                if stage != base.STAGE_CONNECTED_READY:
                    self._update_stage_ui(stage, details)
                return
            if universal_initialization_enabled(self):
                manager = getattr(self, '_universal_manager', None)
                attempt = getattr(self, '_universal_attempt_id', None)
                if manager is None or attempt is None:
                    return  # Rejected plan never created a preparation attempt.
                if stage == base.STAGE_CONNECTED_READY:
                    adapter = getattr(self, '_engine_initialization_adapter', None)
                    if manager.prepared(attempt):
                        completed = getattr(adapter, 'preparation_finished', None)
                        if completed is not None:
                            completed()
                elif stage in (base.STAGE_AUTO_BLOCKED, base.STAGE_CONNECTION_ERROR):
                    manager.fail(attempt, details or stage)
                elif stage == base.STAGE_STOPPED_BY_USER:
                    manager.cancel(attempt)
                else:
                    self._universal_preparation_details = details
                self._poll_universal_initialization()
                return
            self._update_stage_ui(stage, details)

        def _start_universal_initialization(self, mode):
            # No live adapter is supplied here. Inject a reviewed plan and an
            # adapter whose cleanup cancels producers only: stop_test retains
            # exclusive responsibility for physical pause/release in this host.
            from backend.universal_initialization import InitializationManager, Plan
            from backend.engine_initialization_adapter import requested, install
            self._universal_attempt_id = None
            if requested(self._session_context):
                install(self, mode, self._session_context.get('reference_csv'), _initialize_mode)
            manager = getattr(self, '_universal_manager', None)
            plan = getattr(self, '_universal_plan', None)
            self._universal_attempt_id = None
            if not isinstance(manager, InitializationManager) or not isinstance(plan, Plan):
                raise RuntimeError('Universal initialization requires an injected manager and reviewed plan.')
            if getattr(manager.adapter, 'cleanup_owner', None) != 'legacy_runtime':
                raise RuntimeError('Universal adapter cleanup must leave hardware ownership to the legacy runtime.')
            expected_mode = 'manual' if mode == 'manual' else ('pid' if self._control_method == 'pid' else 'recorded_replay')
            if mode not in ('manual', 'auto') or plan.mode != expected_mode or plan.reference != self._session_context['test_reference']:
                raise RuntimeError('Universal plan does not match the selected test/mode.')
            self._universal_hardware_stopped = False
            self._universal_recording_authorized = False
            self._preparing = True
            self.stop_worker_event.clear()
            if getattr(self, '_engine_initialization_adapter', None) is manager.adapter:
                self._universal_attempt_id = manager.start(plan,
                    attempt_id=self._session_context['initialization_attempt']['attempt_id'])
            else:
                self._universal_attempt_id = manager.start(plan)
            self._poll_universal_initialization()

        def _poll_universal_initialization(self):
            from backend.universal_initialization import State
            manager = getattr(self, '_universal_manager', None)
            attempt = getattr(self, '_universal_attempt_id', None)
            if manager is None or attempt is None:
                self._prepared = False
                return
            if getattr(self, '_universal_recording_authorized', False) and not self.stop_worker_event.is_set():
                return  # Initialization checks must not enforce t=0 targets during the manoeuvre.
            if self.stop_worker_event.is_set():
                manager.cancel(attempt)
            if manager.state == State.READY and manager._plan.mode == 'pid':
                # Authorization itself consumes the next frame. Polling it first
                # would consume every frame before the handoff could use one.
                self._handoff_universal_pid()
                if getattr(self, '_universal_recording_authorized', False):
                    return
            manager.poll(attempt)
            if manager.state in (State.BLOCKED, State.FAILED, State.CANCELLED):
                from backend.engine_initialization_adapter import mark
                mark(self, manager.state.value, manager.reason)
                self._prepared = self._preparing = False
                if not self._universal_hardware_stopped:
                    self._universal_hardware_stopped = True
                    adapter = getattr(self, '_engine_initialization_adapter', None)
                    # A rejected plan/ownership conflict acquired no hardware.
                    # Do not release a different controller's authority.
                    if adapter is None or (adapter.preparation_started and not adapter.legacy_failure_cleanup_completed):
                        try:
                            self.stop_test()
                        except Exception as exc:
                            manager.cleanup_errors.append(f'Legacy runtime cleanup failed: {exc}')
                self._update_stage_ui(base.STAGE_AUTO_BLOCKED,
                    manager.reason + ('; cleanup: ' + '; '.join(manager.cleanup_errors) if manager.cleanup_errors else ''))
                mark(self, manager.state.value, manager.reason)
            elif manager.state == State.READY:
                from backend.engine_initialization_adapter import mark
                adapter = getattr(self, '_engine_initialization_adapter', None)
                if adapter is not None:
                    adapter.confirm_verified()
                mark(self, 'READY')
                self._universal_verified_ready = True
                try:
                    from backend.engine_initialization_adapter import initialization_only
                    details = ('Universal initialization verified. Initialization-only trial: recording and replay are locked.'
                               if initialization_only(self) else 'Universal initialization verified.')
                    self._update_stage_ui(base.STAGE_CONNECTED_READY, details)
                finally:
                    self._universal_verified_ready = False
                if manager._plan.mode == 'pid':
                    self._handoff_universal_pid()
            else:
                self._prepared = False
                from backend.engine_initialization_adapter import mark
                mark(self, manager.state.value)
                self._update_stage_ui(getattr(base, 'STAGE_INITIALIZING', manager.state.value),
                    manager.state.value + ': ' + getattr(self, '_universal_preparation_details', 'Universal initialization in progress.'))

        def _handoff_universal_pid(self):
            from backend.engine_initialization_adapter import initialization_only, mark, PidPreparationAdapter
            manager, attempt = self._universal_manager, self._universal_attempt_id
            adapter = self._engine_initialization_adapter
            if initialization_only(self) or self.stop_worker_event.is_set():
                return
            if not isinstance(adapter, PidPreparationAdapter):
                manager.fail(attempt, 'PID requires its reviewed preparation handoff adapter')
                return
            if not manager.authorize_recording(attempt):
                return  # Wait for another correlated native snapshot while still paused.
            try:
                adapter.handoff(for_recording=True)
                if not manager.execution_started(attempt):
                    raise RuntimeError('PID initialization authorization expired or was cancelled')
                self._universal_recording_authorized = True
                self._prepared = False
                mark(self, 'EXECUTING')
                adapter.execution_allowed.set()
            except Exception as exc:
                manager.fail(attempt, str(exc))
                self._poll_universal_initialization()

        def _update_stage_ui(self,stage,details=''):
            from backend.engine_initialization_adapter import mark
            if stage in (base.STAGE_AUTO_BLOCKED, base.STAGE_CONNECTION_ERROR):
                mark(self, 'FAILED', details or stage)
            elif stage == base.STAGE_STOPPED_BY_USER:
                mark(self, 'CANCELLED', details or stage)
            elif stage == getattr(base, 'STAGE_COMPLETED', 'Completed'):
                mark(self, 'COMPLETE')
            if universal_initialization_enabled(self) and stage in (base.STAGE_AUTO_BLOCKED, base.STAGE_CONNECTION_ERROR):
                manager = getattr(self, '_universal_manager', None)
                if manager is not None:
                    manager.fail(getattr(self, '_universal_attempt_id', None), details or stage)
            if stage == base.STAGE_CONNECTED_READY:
                if universal_initialization_enabled(self) and not getattr(self, '_universal_verified_ready', False):
                    return
                if self.stop_worker_event.is_set():
                    self._prepared = False
                    self._preparing = False
                    return
                if not self._preparing:
                    return
            self._stage_details=details
            if stage==base.STAGE_CONNECTED_READY and self._preparing:
                self._prepared=True
                self._preparing=False
                mark(self, 'READY')
            elif stage in (base.STAGE_AUTO_BLOCKED,base.STAGE_CONNECTION_ERROR,base.STAGE_STOPPED_BY_USER):
                self._prepared=False
                self._preparing=False
            super()._update_stage_ui(stage,details)
            self._publish()

        def arm_recording(self):
            # Also guard the inherited Qt button/spacebar entry, not just IPC.
            if universal_initialization_enabled(self):
                from backend.engine_initialization_adapter import initialization_only
                from backend.universal_initialization import State
                manager = getattr(self, '_universal_manager', None)
                if (initialization_only(self) or self.stop_worker_event.is_set()
                        or not getattr(self, '_universal_recording_authorized', False)
                        or manager is None or manager.state != State.RECORDING
                        or manager.attempt_id != getattr(self, '_universal_attempt_id', None)
                        or manager._plan.mode == 'pid'):
                    self._stage_details = 'Recording remains blocked until Universal authorization; PID recording belongs to its guarded workflow.'
                    self._publish()
                    return
            return super().arm_recording()

        def handle_command(self,command):
            action=command['action']
            if action == 'invalidate_initialization':
                session = self._session_context
                # Ignore delayed notifications from an older GUI snapshot/session.
                if (command.get('configuration_revision') != session.get('configuration_revision')
                        or not universal_initialization_enabled(self)
                        or self.is_recording or (getattr(self, '_universal_recording_authorized', False)
                            and getattr(getattr(self, '_universal_plan', None), 'mode', None) != 'pid')):
                    return
                self.handle_command({'action':'stop'})
                session['gui_configuration_invalidated'] = True
                self._stage_details = 'Initialization inputs changed. Initialization cancelled; review the new inputs and initialize again.'
                self._publish()
                return
            from backend.engine_initialization_adapter import initialization_only
            if initialization_only(self) and action in ('record', 'pid_engage', 'pid_tune'):
                self._stage_details = 'Initialization-only trial: recording, replay and PID are locked. Stop and close this worker after inspection.'
                self._publish()
                return
            from backend.p3d_verification_diagnostic import handle_command, stop_for_activity
            if handle_command(self, context, command):
                return
            stop_for_activity(self, action)
            if action in ('diagnostic_start', 'diagnostic_stop'):
                monitor = getattr(self, '_telemetry_diagnostic', None)
                if action == 'diagnostic_stop':
                    if monitor is not None: monitor.stop()
                elif monitor is None or not monitor.thread.is_alive():
                    from backend.telemetry_diagnostic import DiagnosticMonitor
                    self._diagnostic_brunner_display = ''
                    monitor = DiagnosticMonitor(self.p3d,
                        Path(context['status_file']).parent/'telemetry_diagnostics',
                        Path(context['status_file']).with_suffix('.telemetry.json'),
                        display_cache=lambda: self._diagnostic_brunner_display,
                        context_cache=lambda: {'worker_stage': getattr(self, 'current_stage', None),
                                               'shutdown_requested': getattr(self, '_shutting_down', False)})
                    self._telemetry_diagnostic = monitor
                    monitor.start()
                return
            if universal_initialization_enabled(self) and action == 'record':
                revision = self._session_context.get('configuration_revision')
                if revision is not None and command.get('configuration_revision') != revision:
                    self._stage_details = 'Recording request does not match the reviewed GUI configuration.'
                    self._publish()
                    return
                if self._session_context.get('gui_configuration_invalidated'):
                    self._prepared = False
                    self._publish()
                    return
                if getattr(self, '_universal_recording_authorized', False):
                    return
                manager = getattr(self, '_universal_manager', None)
                attempt = getattr(self, '_universal_attempt_id', None)
                if (self.stop_worker_event.is_set() or manager is None or attempt is None
                        or not manager.authorize_recording(attempt)):
                    from backend.universal_initialization import State
                    if manager is not None and manager.state == State.READY and not self.stop_worker_event.is_set():
                        self._stage_details='Waiting for a new verification snapshot; recording remains blocked.'
                        self._publish()
                        return
                    self._prepared = False
                    self._poll_universal_initialization()
                    return
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
                if self.stop_worker_event.is_set() or not self._prepared or self.current_stage!=base.STAGE_CONNECTED_READY or self.is_recording:
                    self._stage_details='Recording remains blocked until initialization is verified.'
                    self._publish()
                    return
                if not self.p3d or not self.p3d.is_connected() or not self.brunner.sock:
                    self._update_stage_ui(base.STAGE_CONNECTION_ERROR,'P3D or BRUNNER disconnected.')
                    return
                self._prepared=False
                if universal_initialization_enabled(self):
                    adapter = getattr(self, '_engine_initialization_adapter', None)
                    if adapter is not None:
                        try:
                            adapter.handoff(for_recording=True)
                        except Exception as exc:
                            self._universal_manager.fail(self._universal_attempt_id, str(exc))
                            self._poll_universal_initialization()
                            return
                    if not self._universal_manager.recording_started(self._universal_attempt_id):
                        self._universal_manager.fail(self._universal_attempt_id, 'Fresh recording authorization expired or was invalidated during handoff')
                        self._poll_universal_initialization()
                        return
                    self._universal_recording_authorized = True
                from backend.engine_initialization_adapter import mark
                mark(self, 'RECORDING')
                self.arm_recording()
            elif action in ('stop','shutdown'):
                self._prepared=False
                self._preparing=False
                self._initialization_generation = getattr(self, '_initialization_generation', 0) + 1
                if universal_initialization_enabled(self):
                    manager = getattr(self, '_universal_manager', None)
                    if manager is not None:
                        manager.cancel(getattr(self, '_universal_attempt_id', None))
                    self._universal_hardware_stopped = True
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
                from backend.engine_initialization_adapter import requested
                universal_auto = mode == 'auto' and requested(dict(test_reference=command.get('test_reference', context['test_reference']), mode=mode, control_method=method))
                reference=validate_reference(command['reference_csv']) if mode=='auto' and (method=='replay' or universal_auto) else None
                duration=validate_duration(command.get('duration_s',75))
                if mode not in ('manual','auto'): raise ValueError('Unknown test mode')
                if method=='pid':
                    from qtg_climb_pid import options,validate_dataset
                    options(command.get('pid_conditions'))
                    if command.get('validation_csv'): validate_dataset(command['validation_csv'])
                requested_test = command.get('test_reference', context['test_reference'])
                if requested_test == '1.f.(2)':
                    from backend.deceleration_config import resolve_initial_conditions
                    overrides = dict(command.get('initial_conditions') or {})
                    conditions = resolve_initial_conditions(overrides)
                    context['initial_condition_overrides'] = overrides
                else:
                    conditions=validate_conditions(command.get('initial_conditions'))
                    context['initial_condition_overrides'] = None
                # Set Conditions already prepares Manual mode. Selecting Manual again
                # must not teleport the aircraft or repeat hardware preparation.
                airflow_preparation = (command.get('acceleration_manual_airflow_preparation') is True
                    and requested_test == '1.f.(1)' and mode == 'manual' and method == 'replay')
                reuse_manual=(not universal_initialization_enabled(self) and mode=='manual' and context.get('mode')=='manual'
                    and bool(context.get('acceleration_manual_airflow_preparation', False)) == airflow_preparation
                    and command.get('test_reference',context['test_reference'])==context['test_reference']
                    and conditions is not None and conditions==getattr(self,'manual_initial_conditions',None)
                    and getattr(self,'_prepared',False)
                    and getattr(self,'current_stage',None)==getattr(base,'STAGE_CONNECTED_READY',None)
                    and self.p3d is not None and self.p3d.is_connected() and bool(self.brunner.sock)
                    and not command.get('force_initialize',False))
                self.manual_initial_conditions=conditions;self.report_initial_conditions=conditions
                self.report_graphs=command.get('selected_graphs')
                context.update(initial_conditions=conditions,selected_graphs=self.report_graphs)
                if airflow_preparation:
                    context['acceleration_manual_airflow_preparation'] = True
                else:
                    context.pop('acceleration_manual_airflow_preparation', None)
                if not reuse_manual:
                    self._manual_airflow_ready = None
                sys.modules['qtg_common_engine']._active_test_reference = command.get('test_reference', context.get('test_reference'))
                self._control_method=method
                self.control_method=method
                self.validation_csv=command.get('validation_csv')
                context.update(control_method=method,pid_conditions=command.get('pid_conditions'),validation_csv=command.get('validation_csv'),mode=mode,reference_csv=reference,test_reference=command.get('test_reference',context['test_reference']),
                               title=command.get('title',context['title']),duration_s=duration)
                context['operator_initiated_universal_manual'] = (command.get('operator_initiated_universal_manual') is True and context['test_reference'] == '1.f.(2)' and mode == 'manual')
                context['configuration_revision'] = command.get('configuration_revision')
                context.pop('gui_configuration_invalidated', None)
                context.pop('manual_manager_active', None)
                context.pop('universal_manager_active', None)
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
            if universal_initialization_enabled(self):
                self._poll_universal_initialization()
            try:
                if self._control_method=='pid' and not getattr(self,'pid_automatic_workflow',False) and self._prepared and not self.is_recording and getattr(self,'pid_controller_active',False):
                    from qtg_climb_pid import watchdog
                    watchdog(self.brunner)
                super()._on_ui_timer_tick()
                if getattr(self, '_telemetry_diagnostic', None) is not None:
                    self._diagnostic_brunner_display = self.lbl_brunner_sticks.text()
            except Exception as exc:
                if self._control_method=='pid':
                    self.stop_test()
                    self._update_stage_ui(base.STAGE_CONNECTION_ERROR,str(exc))
                else: raise
            self._publish()

        def _save_csv_and_generate_reports(self):
            # Engine tests: guard the recorded mode and strip any previous
            # replay reference from Manual report generation.
            if context.get('test_reference') in ('1.f.(1)', '1.f.(2)'):
                if self.test_mode != context.get('mode'):
                    raise RuntimeError(
                        'Engine test run mode changed unexpectedly; '
                        'report generation blocked to prevent mislabelled evidence.')
                if self.test_mode == 'manual':
                    self.active_manual_ref_path = None
            before={p for p in self.data_dir.iterdir() if p.is_file()}
            self.current_stage='Generating reports'
            self.lbl_rec_status.setText('Generating CSV and PDF reports...')
            self._publish()
            # Reports are opened explicitly by QTG Studio, never by the invisible engine.
            original_open=getattr(os,'startfile',None)
            if original_open is not None: os.startfile=lambda *args,**kwargs: None
            try:
                from reports.report_manager import save_worker_reports
                save_worker_reports(self, super()._save_csv_and_generate_reports)
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
                    name_lower = path.name.lower()
                    if any(sub in name_lower for sub in ('_summary', '_results', '_diagnostics', '_titt', '_evaluation')):
                        continue
                    with path.open(newline='',encoding='utf-8-sig') as stream:
                        reader=csv.DictReader(stream);fields=list(reader.fieldnames or []);samples=list(reader)
                    if 'Time (s)' not in fields or 'Indicated Airspeed (kts)' not in fields:
                        continue
                    extra={'PID Target IAS (kts)':'target_ias','PID Command (-)':'command','PID Tracking Error (-)':'tracking_error','BRUNNER Pitch Applied Force (raw)':'applied_force'}
                    count = min(len(samples), len(recording_rows))
                    if count == 0:
                        continue
                    for i in range(count):
                        samples[i].update({label: recording_rows[i][key] for label, key in extra.items()})
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
                        # Acquisition has completed; report generation does not restart initialization.
                        from backend.engine_initialization_adapter import mark
                        mark(self, 'COMPLETE')
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
            from backend.engine_initialization_adapter import mark
            if self.current_stage == base.STAGE_COMPLETED:
                mark(self, 'COMPLETE')
            if getattr(self,'_control_method','replay')=='pid' and getattr(self,'pid_fault',None):
                self.current_stage=base.STAGE_CONNECTION_ERROR
                self._stage_details='Incomplete PID test: '+self.pid_fault
            if any(path.lower().endswith('.pdf') for path in self._outputs) and not (getattr(self,'_control_method','replay')=='pid' and getattr(self,'pid_fault',None)):
                self.lbl_rec_status.setText('COMPLETED - CSV and PDF reports saved')
            if context.get('test_reference') == '1.f.(2)':
                from backend.deceleration_config import write_output_manifest
                manifest = write_output_manifest(context, self._outputs)
                if manifest is not None:
                    self._outputs.append(str(manifest))
            self._publish()

        def finish_shutdown(self):
            verification = getattr(self, '_p3d_verification_diagnostic', None)
            if verification is not None:
                verification.stop('worker_shutdown')
                if verification.thread is not None and verification.thread.is_alive(): return False
            monitor = getattr(self, '_telemetry_diagnostic', None)
            if monitor is not None:
                monitor.stop(reason='worker_shutdown')
                if monitor.thread.is_alive(): return False
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
            commands = [(path, json.loads(path.read_text(encoding='utf-8')))
                        for path in sorted(directory.glob('*.json'))] if directory.exists() else []
            commands = prioritize_initialization_commands(window, commands)
            for path, command in commands:
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
