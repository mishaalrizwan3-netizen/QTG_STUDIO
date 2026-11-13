"""Diagnostic connection ownership. Standalone transport runs in its own process."""
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import threading
import time
from uuid import uuid4

from backend.telemetry_diagnostic import DiagnosticMonitor


class DashboardCache:
    """Read-only view of a borrowed dashboard client; no start/stop ownership.

    Bridge age is receive-age, not an acquisition timestamp. Sequence here is a
    diagnostic observation counter, not a native packet sequence.
    """
    diagnostic_provenance = 'Dashboard subset: receive-age-derived time; observation counter, not native packet sequence. Freshness UNVERIFIED.'
    def __init__(self, client):
        self.client, self.counter, self.data = client, 0, {}

    def is_connected(self):
        state = self.client.snapshot() if self.client.running else None
        connected = bool(state is not None and state.connected)
        if state is not None:
            self.counter += 1
            self.data = {'bridge_telemetry_sequence': self.counter,
                         'last_updated': time.time()-state.telemetry_age_ms/1000.}
            if state.telemetry_valid:
                for source, target in (('pitch_deg','pitch_angle'), ('bank_deg','bank_angle'),
                    ('ias_kt','airspeed_indicated'), ('altitude_ft','pressure_alt'),
                    ('throttle_percent','engine_throttle_percent'), ('weight_kg','total_weight'),
                    ('temperature_c','ambient_temp')):
                    self.data[target] = getattr(state, source)
                self.data['eng_rpms'] = [state.engine_rpm]
                self.data['prop_rpms'] = [state.prop_rpm]
        return connected

    def get_telemetry_sequence(self): return self.data.get('bridge_telemetry_sequence', 0)
    def get_telemetry(self): return dict(self.data)


class DiagnosticSession:
    """Borrow existing transports or spawn an isolated diagnostic-only reader.

    Closing a borrowed session stops only its monitor. The helper alone owns its
    independent SimConnect client's stop(); never load a second singleton DLL
    client into the dashboard or existing QTG worker process.
    """
    def __init__(self, directory, *, dashboard=None, worker=None, worker_context=None,
                 popen=subprocess.Popen):
        self.directory = Path(directory)
        self.dashboard, self.worker, self.worker_context = dashboard, worker, worker_context
        self.popen = popen
        self.monitor = self.process = None
        self.kind = 'not connected'
        self.closed = False
        self.status_file = self.directory/'status.json'

    def connect(self):
        if self.closed: return
        if self.kind != 'not connected': return
        if self.worker is not None and self.worker.poll() is None and self.worker_context:
            self.kind = 'borrowed QTG worker'
            self.status_file = Path(self.worker_context['status_file']).with_suffix('.telemetry.json')
            return
        if self.dashboard is not None and self.dashboard.running:
            view = DashboardCache(self.dashboard)
            if view.is_connected():
                self.kind = 'borrowed dashboard (limited channels)'
                self.monitor = DiagnosticMonitor(view, self.directory, self.status_file)
                return
        self.directory.mkdir(parents=True, exist_ok=True)
        args = [sys.executable]
        if not getattr(sys, 'frozen', False):
            args.append(str(Path(__file__).resolve().parents[1]/'main.py'))
        args += ['--diagnostic-connection', str(self.directory)]
        self.process = self.popen(args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
        self.kind = 'isolated diagnostic transport (handshake pending)'

    def command(self, action):
        if action not in ('start', 'stop', 'close'): raise ValueError('Invalid diagnostic action')
        if self.closed: return
        if action == 'close': self.closed = True
        if self.kind == 'not connected': return
        if self.kind == 'borrowed QTG worker':
            from qtg_common_engine import send_command
            if self.worker.poll() is None:
                send_command(self.worker_context, 'diagnostic_start' if action == 'start' else 'diagnostic_stop')
        elif self.monitor is not None:
            if action == 'start':
                if self.monitor.thread is None:
                    self.monitor.start()
                elif not self.monitor.thread.is_alive():
                    self.monitor = DiagnosticMonitor(self.monitor.reader.client, self.directory, self.status_file)
                    self.monitor.start()
            else: self.monitor.stop()
        else:
            self.directory.mkdir(parents=True, exist_ok=True)
            # Unique ordered commands avoid start/stop overwriting each other.
            (self.directory/(str(time.time_ns())+'_'+action+'.command')).write_text(action, encoding='utf-8')


def run_standalone(directory, factory=None):
    """CLI child only. Explicitly no placement callbacks, base loader or BRUNNER."""
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    client = None
    monitor = thread = None
    try:
        if factory is None:
            path = (Path(sys._MEIPASS)/'brunner_base' if getattr(sys, 'frozen', False)
                    else Path(__file__).resolve().parents[1]/'interfaces/brunner_base')/'src/p3d_simconnect_v15.py'
            spec = importlib.util.spec_from_file_location('diagnostic_p3d_readonly', path)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            factory = module.SimConnectClient
        client = factory()
        thread = threading.Thread(target=client.connect_and_run,
            kwargs={'place_at_rahwali': False, 'place_airborne': False}, daemon=True)
        thread.start()
        while True:
            (directory/'transport.json').write_text(json.dumps({
                'connected': bool(client.is_connected()), 'qtg_worker': False,
                'freshness': 'UNVERIFIED', 'error': getattr(client, 'last_error', '')}), encoding='utf-8')
            closing = False
            for path in sorted(directory.glob('*.command')):
                action = path.read_text(encoding='utf-8')
                path.unlink()
                if action == 'close': closing = True; break
                if action == 'stop' and monitor: monitor.stop()
                if action == 'start' and (monitor is None or not monitor.thread.is_alive()):
                    monitor = DiagnosticMonitor(client, directory, directory/'status.json')
                    monitor.start()
            if closing: break
            time.sleep(.2)
    finally:
        if monitor:
            monitor.stop()
            monitor.thread.join(3.)
        if client: client.stop()  # Exclusively the child-owned connection.
        if thread: thread.join(3.)
