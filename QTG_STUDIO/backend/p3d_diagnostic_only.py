"""Explicit foreground P3D-only session. No GUI, test engine or hardware owner.

Uses the existing SimConnectClient poller and Stage 5D snapshot diagnostic. Only
this process owns its connection; no subprocess or QTG test worker is launched.
"""
import ctypes
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import threading
import time
from types import SimpleNamespace
from uuid import uuid4

from backend.p3d_verification_diagnostic import (
    START, STOP, SWITCHES, finite, handle_command, publish, status_path)

KIND = 'p3d_verification_only'
CLOSE = 'verification_session_close'
INSTALLED_SHA256 = 'd6ffc5f25d3931558572f81c6184678f2d48cca6e7e605a2e4cf160de238fc7c'
ROOT = Path(__file__).resolve().parents[1]
INSTALLED_DLL = ROOT/'interfaces/brunner_base/native/bin/p3d_bridge_engine_init_v1517.dll'

# Exposing a mapped event/setter during library binding does not invoke it.
# Once the client is constructed, this gate rejects all control exports.
READ_EXPORTS = frozenset({
    'ConnectSim', 'DisconnectSim', 'ProcessMessages', 'IsSimConnected',
    'GetBridgeVersion', 'GetTelemetryDataV1507', 'GetTelemetrySequence',
    'GetExtendedCausalStateV1495', 'GetExtendedCausalSequenceV1495',
    'GetEngineSupportStateV1496', 'GetEngineSupportSequenceV1496',
    'RequestVerificationV1', 'GetVerificationV1',
    'GetSimulationRunningV1509', 'RequestSimulationRunningV1509',
})


class _ReadCall:
    def __init__(self, gate, name, function):
        self.gate, self.name, self.function = gate, name, function

    @property
    def argtypes(self): return self.function.argtypes
    @argtypes.setter
    def argtypes(self, value): self.function.argtypes = value
    @property
    def restype(self): return self.function.restype
    @restype.setter
    def restype(self, value): self.function.restype = value

    def __call__(self, *args):
        with self.gate.lock:
            if self.name == 'ConnectSim':
                if self.gate.open_attempted:
                    # The legacy poller retries before its first handshake.
                    # Reuse its first result; never open another native handle.
                    self.gate.suppressed_reopens += 1
                    return self.gate.open_result
                self.gate.open_attempted = True
            self.gate.calls[self.name] = self.gate.calls.get(self.name, 0)+1
        result = self.function(*args)
        if self.name == 'ConnectSim': self.gate.open_result = result
        return result


class ReadOnlyBridge:
    def __init__(self, bridge):
        self.bridge = bridge
        self.lock = threading.Lock()
        self.calls, self.violations, self.wrapped = {}, [], {}
        self.open_attempted, self.open_result, self.suppressed_reopens = False, 0, 0

    def __getattr__(self, name):
        if name not in READ_EXPORTS:
            with self.lock: self.violations.append(name)
            raise PermissionError('Diagnostic-only mode forbids native export: '+name)
        with self.lock:
            if name not in self.wrapped:
                self.wrapped[name] = _ReadCall(self, name, getattr(self.bridge, name))
            return self.wrapped[name]


class SingleSession:
    """OS-released mutex prevents duplicate diagnostic-only owners per project."""
    def __init__(self):
        self.handle = None

    def __enter__(self):
        if os.name != 'nt':
            raise RuntimeError('Diagnostic-only P3D connection requires Windows x64')
        self.kernel = ctypes.WinDLL('kernel32', use_last_error=True)
        self.kernel.CreateMutexW.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_wchar_p]
        self.kernel.CreateMutexW.restype = ctypes.c_void_p
        self.kernel.ReleaseMutex.argtypes = [ctypes.c_void_p]
        self.kernel.CloseHandle.argtypes = [ctypes.c_void_p]
        name = 'Local\\QTG_P3D_Verification_'+hashlib.sha256(str(ROOT).lower().encode()).hexdigest()[:20]
        ctypes.set_last_error(0)
        self.handle = self.kernel.CreateMutexW(None, 1, name)
        if not self.handle: raise OSError('Could not acquire diagnostic-only session mutex')
        if ctypes.get_last_error() == 183:
            self.kernel.CloseHandle(self.handle)
            self.handle = None
            raise RuntimeError('A diagnostic-only session for this project is already running')
        return self

    def __exit__(self, *args):
        if self.handle:
            self.kernel.ReleaseMutex(self.handle)
            self.kernel.CloseHandle(self.handle)
            self.handle = None


def validate_library(path=INSTALLED_DLL):
    path = Path(path).resolve()
    if hashlib.sha256(path.read_bytes()).hexdigest() != INSTALLED_SHA256:
        raise RuntimeError('Installed DLL is not the reviewed Stage 5C binary; connection refused')
    return path


def enqueue(context, action, **values):
    """Diagnostic IPC only; deliberately avoids importing qtg_common_engine."""
    if context.get('session_kind') != KIND or action not in (START, STOP, CLOSE):
        raise ValueError('Not an allowed diagnostic-only command')
    directory = Path(context['command_directory'])
    if not directory.is_dir(): raise RuntimeError('Diagnostic session command directory is missing')
    path = directory/(f'{time.time_ns():020d}_'+uuid4().hex+'.json')
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(dict(values, action=action, expires_at=time.time()+5.)), encoding='utf-8')
    temporary.replace(path)


def dispatch(window, context, command):
    """Return close request; never forward unknown commands to a QTG handler."""
    action = command.get('action')
    expires = command.get('expires_at')
    if not finite(expires) or time.time() > expires:
        raise ValueError('Diagnostic command expired or lacks a deadline')
    if action == CLOSE:
        return True
    if action not in (START, STOP):
        raise ValueError('Diagnostic-only session rejects command: '+str(action))
    if action == START and not window.p3d.is_connected():
        raise ValueError('P3D handshake not complete; wait for IDLE_CONNECTED')
    handle_command(window, context, command)
    return False


def run(session_root, *, connection_timeout_s=15., session_timeout_s=300.,
        factory=None, lease_factory=SingleSession, clock=time.monotonic,
        sleep=time.sleep, output=print, cancelled=None):
    """Foreground bounded session; this function must run in its own CLI process.

    Factory/clock/lease injection is for offline tests only. No factory is invoked
    until the installed DLL hash and ownership lease have been checked.
    """
    if not all(finite(v) and v > 0 for v in (connection_timeout_s, session_timeout_s)):
        raise ValueError('Positive finite connection/session deadlines required')
    if connection_timeout_s > session_timeout_s:
        raise ValueError('Connection deadline must not exceed session lifetime')
    if any(os.environ.get(key) == '1' for key in SWITCHES):
        raise RuntimeError('Disable both Universal Initialization switches before connecting')
    for key in SWITCHES: os.environ[key] = '0'  # This diagnostic process only.
    installed = validate_library()
    cancelled = cancelled if cancelled is not None else threading.Event()
    with lease_factory():
        folder = Path(session_root)/(
            datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_'+uuid4().hex[:8])
        folder.mkdir(parents=True, exist_ok=False)
        commands = folder/'commands'
        commands.mkdir()
        context = dict(session_kind=KIND, session_id=uuid4().hex, owner_pid=os.getpid(),
            command_directory=str(commands.resolve()), status_file=str((folder/'worker_status.json').resolve()),
            context_file=str((folder/'context.json').resolve()), universal_initialization_enabled=False,
            installed_dll=str(installed), installed_sha256=INSTALLED_SHA256)
        Path(context['context_file']).write_text(json.dumps(context, indent=2), encoding='utf-8')
        output('Diagnostic-only context: '+context['context_file'])
        output('No QTG initialization, BRUNNER, recording or replay. Ctrl+C closes this session.')
        client = gate = thread = window = None
        started = clock()
        phase, reason, seen, result = 'CONNECTING', None, False, 0
        pause_note = None

        def heartbeat():
            payload = dict(context, phase=phase, reason=reason, updated=time.time(),
                connected=bool(client is not None and client.is_connected()),
                recording=False, controlling=False, can_record=False,
                pause_observation_note=pause_note)
            if gate is not None:
                payload.update(native_calls=dict(gate.calls), forbidden_exports=list(gate.violations),
                    suppressed_reopens=gate.suppressed_reopens)
            publish(context['status_file'], payload)

        try:
            heartbeat()
            if factory is None:
                from interfaces.brunner_base.src.p3d_simconnect_v15 import SimConnectClient
                factory = SimConnectClient
            client = factory()
            if Path(client._dll_path).resolve() != installed:
                raise RuntimeError('Client selected a different DLL; connection refused')
            gate = ReadOnlyBridge(client._bridge)
            client._bridge = gate
            window = SimpleNamespace(p3d=client, _session_context=context,
                _preparing=False, is_recording=False, _shutting_down=False)
            thread = threading.Thread(target=client.connect_and_run, kwargs=dict(
                on_connected=None, on_error=None, place_at_rahwali=False, place_airborne=False), daemon=True)
            thread.start()
            while not cancelled.is_set():
                connected = client.is_connected()
                if gate.violations: raise RuntimeError('Forbidden export blocked: '+', '.join(gate.violations))
                if connected and not seen:
                    seen, phase = True, 'IDLE_CONNECTED'
                    # Read-only Pause subscription, no pause/unpause command.
                    try: client.simulation_running()
                    except Exception as exc: pause_note = 'UNVERIFIED: '+str(exc)
                    output('IDLE_CONNECTED. Use the printed context with the existing verification caller.')
                elif seen and not connected:
                    raise RuntimeError('P3D disconnected; no automatic reconnect or test initialization attempted')
                if not thread.is_alive():
                    raise RuntimeError('Connection poller exited: '+str(getattr(client, 'last_error', '')))
                if not seen and clock()-started >= connection_timeout_s:
                    raise TimeoutError('P3D connection/handshake deadline expired')
                if clock()-started >= session_timeout_s:
                    reason = 'diagnostic_session_deadline'
                    break
                heartbeat()
                closing = False
                for path in sorted(commands.glob('*.json')):
                    command = {}
                    try:
                        command = json.loads(path.read_text(encoding='utf-8'))
                        closing = dispatch(window, context, command)
                    except (ValueError, KeyError, TypeError) as exc:
                        publish(status_path(context), dict(active=False, state='REJECTED',
                            reason=str(exc), session_id=command.get('session_id'), measurements=[]))
                    finally:
                        path.unlink(missing_ok=True)
                    if closing: break
                if closing:
                    reason = 'operator_close'
                    break
                sleep(.1)
            if cancelled.is_set(): reason = 'operator_cancelled'
        except KeyboardInterrupt:
            reason = 'operator_cancelled'
        except Exception as exc:
            phase, reason, result = 'FAILED', str(exc), 1
            output('Diagnostic-only session stopped: '+reason)
        finally:
            failure_phase = phase == 'FAILED'
            phase = 'CLOSING'
            if window is not None:
                window._shutting_down = True
                monitor = getattr(window, '_p3d_verification_diagnostic', None)
                if monitor is not None:
                    monitor.stop(reason or 'session_closed')
                    if monitor.thread is not None:
                        monitor.thread.join(3.)
                        if monitor.thread.is_alive():
                            reason, result = 'Verification reader did not finish before shutdown deadline', 1
            if client is not None: client.stop()  # Own connection only; never aircraft pause/release.
            if thread is not None:
                thread.join(3.)
                if thread.is_alive():
                    reason, result = 'Owned connection poller did not finish before shutdown deadline', 1
            phase = 'FAILED' if failure_phase or result else 'CLOSED'
            try: heartbeat()
            except OSError as exc:
                output('Final diagnostic status save failed: '+str(exc))
                result = 1
            output(phase+': '+str(reason or 'session_closed'))
        return result
