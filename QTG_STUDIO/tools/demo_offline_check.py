"""Frozen demo checks. Hardware startup is replaced before any window is created."""
import ctypes
import importlib
import json
import os
from pathlib import Path
import sys
import traceback
from types import SimpleNamespace


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def worker(context_path):
    import qtg_common_engine as engine
    base = engine.load_base()
    from PySide6.QtCore import QTimer
    base.SuperMushakLoggerApp._start_hardware_connections = lambda self: None
    base.SuperMushakLoggerApp._play_beep = lambda *args: None
    factory = engine.worker_class

    def offline_class(*args):
        cls = factory(*args)
        original = cls.__init__

        def initialize(self):
            original(self)
            self.ui_timer.stop()
            self.p3d = SimpleNamespace(is_connected=lambda: False, stop=lambda: None)
            self.brunner = SimpleNamespace(sock=None, close=lambda: None)
            QTimer.singleShot(250, lambda: setattr(self, '_shutting_down', True))
        cls.__init__ = initialize
        return cls

    engine.worker_class = offline_class
    return engine.run(context_path)


def run(root):
    os.environ['QT_QPA_PLATFORM'] = 'offscreen'
    result = {'ok': False, 'frozen': bool(getattr(sys, 'frozen', False)), 'hardware_connected': False, 'checks': []}
    app = None
    children = []
    try:
        import frontend.main as gui
        import qtg_common_engine as engine
        from backend.registry import get_definition, engine_test_policy
        gui.P3DClient.start = lambda self: False
        gui.App._poll_p3d = lambda self: None
        gui.App._auto_sync_timer = lambda self: None
        evidence = root / 'offline_verification'
        evidence.mkdir(exist_ok=True)
        gui.RESULTS_ROOT = evidence
        gui.LOGS_FILE = evidence / 'test_logs.json'
        gui.TEST_NOTES_FILE = evidence / 'test_notes.json'
        for name in ['numpy', 'pandas', 'matplotlib', 'PIL.ImageTk', 'psutil', 'PySide6.QtWidgets',
                     'backend.engine_initialization_adapter', 'backend.telemetry_verification', 'qtg_backend.worker']:
            importlib.import_module(name)
        result['checks'].append('runtime dependencies and dynamic backend imports')
        app = gui.App()
        app.withdraw()
        calls = []
        app._launch_common_test = lambda mode, spec: calls.append((mode, spec.reference))
        for reference in ['1.c.(1)', '1.f.(1)', '1.f.(2)']:
            index = next(i for i, spec in enumerate(gui.TESTS) if spec.reference == reference)
            app.select_test(index)
            app.btn_subtab_manual.command()
            app.btn_subtab_auto.command()
            require(calls[-2:] == [('manual', reference), ('auto', reference)], reference + ': incorrect GUI routing')
            require(get_definition(reference), reference + ': missing definition')
            if reference.startswith('1.f'):
                require(engine_test_policy(reference), reference + ': missing engine policy')
            app.update_idletasks()
            result['checks'].append(reference + ': panel, Manual/Automatic routing and configuration')
        from PySide6.QtWidgets import QApplication, QLabel
        qt = QApplication.instance() or QApplication([])
        label = QLabel('Offline render check')
        require(not label.grab().isNull(), 'Qt rendering failed')
        label.close()
        base = engine.load_base()
        client = base.SimConnectClient()  # Loads exports only; start/connect is never called.
        require(Path(client._dll_path).is_relative_to(Path(sys._MEIPASS)), 'Worker DLL outside bundle')
        require(client.bridge_version >= 1517, 'Wrong worker bridge version')
        from qtg_position_setter import find_simconnect_dll
        native = ctypes.CDLL(str(find_simconnect_dll()))
        for name in ['Open', 'Close', 'CallDispatch', 'SetDataOnSimObject', 'WeatherSetObservation']:
            require(hasattr(native, 'SimConnect_' + name), 'Missing SimConnect export ' + name)
        require(hasattr(ctypes.CDLL(str(gui.DLL_PATH)), 'QTG_GetState'), 'Missing dashboard export')
        result['checks'].append('Qt rendering; bundled native DLL load and SimConnect exports, no connection')
        import qtg_climb_pid
        require(qtg_climb_pid.controller_types()[0]().kp == .030, 'PID source could not be loaded')
        result['checks'].append('existing Climb PID source loads')
        import pandas as pd
        data = pd.DataFrame({'Time (s)': [0., 1., 2.], 'Indicated Airspeed (kts)': [80., 81., 82.],
                             'Indicated Altitude (ft)': [3000., 3010., 3020.], 'Rate of Climb/Descent (fpm)': [600., 600., 600.]})
        sample = evidence / 'offline_manual_sample.csv'
        data.to_csv(sample, index=False)
        pdf = Path(sys.modules['plot_graphs'].plot_aligned_comparison(sample))
        require(pdf.read_bytes().startswith(b'%PDF'), 'Legacy graph generation failed')
        engine_plot = sys.modules['plot_graphs']._get_engine_plot_module()
        engine_pdf = Path(engine_plot.plot_aligned_comparison(sample, output_pdf_path=evidence / 'offline_engine_graphs.pdf'))
        require(engine_pdf.read_bytes().startswith(b'%PDF'), 'Engine graph generation failed')
        require(Path(qtg_climb_pid.report(sample)).read_bytes().startswith(b'%PDF'), 'PID graph generation failed')
        result['checks'].append('CSV, legacy graph renderer, engine graph renderer and Climb PID PDF generation')
        os.environ['QTG_DEMO_OFFLINE_TEST'] = '1'
        for reference in ['1.c.(1)', '1.f.(1)', '1.f.(2)']:
            spec = next(s for s in gui.TESTS if s.reference == reference)
            child, context = engine.launch('manual', spec, evidence / reference.replace('.', '_'))
            children.append(child)
            require(Path(child.args[0]).resolve() == Path(sys.executable).resolve(), 'Worker requires external interpreter')
            require(child.wait(timeout=35) == 0, 'Worker did not exit cleanly')
            require(json.loads(Path(context['status_file']).read_text())['stage'] == 'Closed', 'Worker shutdown incomplete')
            result['checks'].append(reference + ': same-EXE worker launch and clean shutdown (mock hardware)')
        app._on_close()
        app = None
        app = gui.App()
        app.withdraw()
        app._on_close()
        app = None
        result['checks'].append('GUI opens and closes repeatedly; all launched workers exited')
        result['ok'] = True
    except BaseException:
        result['error'] = traceback.format_exc()
    finally:
        for child in children:
            if child.poll() is None:
                child.kill()
                child.wait(timeout=10)
        if app is not None:
            app._on_close()
        (root / 'demo_offline_results.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
    return 0 if result['ok'] else 1
