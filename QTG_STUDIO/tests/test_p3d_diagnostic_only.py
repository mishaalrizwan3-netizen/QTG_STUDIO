"""Offline cold-start tests. All connection calls are fakes, never a real DLL."""
import ast
from contextlib import nullcontext
import ctypes
import importlib.util
import json
import os
from pathlib import Path
import re
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from backend import p3d_diagnostic_only as mode
from backend import p3d_verification_diagnostic as diagnostic
from interfaces.brunner_base.src import p3d_simconnect_v15 as api

spec = importlib.util.spec_from_file_location('diagnostic_only_console',
    Path(__file__).parents[1]/'tools/p3d_verification_diagnostic.py')
console = importlib.util.module_from_spec(spec)
spec.loader.exec_module(console)


class FakeClient:
    def __init__(self, *, never_connect=False, violate=False):
        self._dll_path = str(mode.INSTALLED_DLL)
        self._bridge = SimpleNamespace(**{name:Mock(return_value=1) for name in mode.READ_EXPORTS})
        self.connected = False
        self.stop_event = threading.Event()
        self.kwargs = None
        self.last_error = ''
        self.never_connect, self.violate = never_connect, violate
        self.close_count = 0

    def is_connected(self): return self.connected
    def stop(self): self.stop_event.set()
    def simulation_running(self): return False

    def connect_and_run(self, **kwargs):
        self.kwargs = kwargs
        try:
            self._bridge.ConnectSim()
            if self.violate: self._bridge.SetGlobalWeatherMETAR(b'FORBIDDEN')
            self.connected = not self.never_connect
            self.stop_event.wait(2.)
        except PermissionError as exc:
            self.last_error = str(exc)
        finally:
            self._bridge.DisconnectSim()
            self.close_count += 1
            self.connected = False


class DiagnosticOnlyTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.now = 0.
        self.context = None
        self.lines = []
        self.environ = patch.dict(os.environ, {key:'0' for key in diagnostic.SWITCHES})
        self.environ.start()
        self.addCleanup(self.environ.stop)

    def output(self, text):
        self.lines.append(text)
        if text.startswith('Diagnostic-only context: '):
            self.context = json.loads(Path(text.split(': ',1)[1]).read_text())

    def sleep(self, seconds):
        self.now += seconds
        time.sleep(.001)  # Yield only to a fake client's thread.

    def run_session(self, client, **options):
        return mode.run(self.root, factory=lambda:client, lease_factory=nullcontext,
            connection_timeout_s=.3, session_timeout_s=1., clock=lambda:self.now,
            sleep=self.sleep, output=options.pop('output',self.output), **options)

    def close_when_ready(self, text):
        self.output(text)
        if text.startswith('IDLE_CONNECTED'):
            mode.enqueue(self.context,mode.CLOSE)

    def test_connection_never_receives_placement_or_test_callbacks(self):
        client = FakeClient()
        self.assertEqual(self.run_session(client,output=self.close_when_ready),0)
        self.assertEqual(client.kwargs,dict(on_connected=None,on_error=None,
            place_at_rahwali=False,place_airborne=False))
        self.assertEqual(client.close_count,1)
        self.assertTrue(client.stop_event.is_set())
        status = json.loads(Path(self.context['status_file']).read_text())
        self.assertEqual(status['phase'],'CLOSED')
        self.assertFalse(status['can_record'])
        self.assertFalse(status['recording'])
        self.assertEqual(status['native_calls']['ConnectSim'],1)
        self.assertFalse(status['forbidden_exports'])

    def test_no_gui_brunner_pid_or_test_engine_imports_in_startup(self):
        tree = ast.parse(Path(mode.__file__).read_text())
        imported = [n.module for n in ast.walk(tree) if isinstance(n,ast.ImportFrom)]
        forbidden = ('frontend','qtg_common_engine','brunner_interface','qtg_climb_pid',
                     'backend.initializer','test_suites')
        self.assertFalse(any(name and name.startswith(forbidden) for name in imported))

    def test_mutating_exports_are_blocked_before_native_invocation(self):
        bridge = SimpleNamespace()
        gate = mode.ReadOnlyBridge(bridge)
        for name in ('SetUserPosition','SetGlobalWeatherMETAR','PauseSim','UnpauseSim',
                     'SetThrottle','SetInitialDynamicState','SetEngineSupportValueV1496',
                     'SetInitialTrimV1516','SendEnginePreparationEventV1496'):
            native = Mock()
            setattr(bridge,name,native)
            with self.subTest(name=name),self.assertRaises(PermissionError): getattr(gate,name)()
            native.assert_not_called()

    def test_second_open_is_suppressed_instead_of_creating_another_handle(self):
        native = Mock(return_value=1)
        gate = mode.ReadOnlyBridge(SimpleNamespace(ConnectSim=native))
        self.assertEqual(gate.ConnectSim(),1)
        self.assertEqual(gate.ConnectSim(),1)
        native.assert_called_once()
        self.assertEqual(gate.suppressed_reopens,1)

    def test_signature_binding_is_forwarded_for_snapshot_api(self):
        native = Mock(return_value=7)
        gate = mode.ReadOnlyBridge(SimpleNamespace(GetVerificationV1=native))
        gate.GetVerificationV1.argtypes = [ctypes.c_void_p,ctypes.c_uint32]
        gate.GetVerificationV1.restype = ctypes.c_int
        self.assertEqual(native.argtypes,[ctypes.c_void_p,ctypes.c_uint32])
        self.assertEqual(native.restype,ctypes.c_int)
        self.assertEqual(gate.GetVerificationV1(None,256),7)

    def test_existing_snapshot_acquisition_works_through_restricted_bridge(self):
        from test_verification_snapshot import SnapshotTests
        fixture = SnapshotTests()
        fixture.setUp()
        gate = mode.ReadOnlyBridge(fixture.client._bridge)
        fixture.client._bridge = gate
        raw = fixture.read()
        self.assertEqual(raw['request_id'],1001)
        self.assertEqual(raw['values']['flaps'],0.)
        self.assertEqual(gate.calls,dict(RequestVerificationV1=1,GetVerificationV1=1))
        self.assertFalse(gate.violations)

    def test_unexpected_control_attempt_fails_session_and_closes_owned_connection(self):
        client = FakeClient(violate=True)
        self.assertEqual(self.run_session(client),1)
        self.assertEqual(client.close_count,1)
        status = json.loads(Path(self.context['status_file']).read_text())
        self.assertEqual(status['phase'],'FAILED')
        self.assertIn('SetGlobalWeatherMETAR',status['forbidden_exports'])

    def test_handshake_timeout_closes_connection_and_poller(self):
        client = FakeClient(never_connect=True)
        self.assertEqual(self.run_session(client),1)
        self.assertEqual(client.close_count,1)
        status = json.loads(Path(self.context['status_file']).read_text())
        self.assertIn('deadline',status['reason'])

    def test_session_deadline_closes_idle_session(self):
        client = FakeClient()
        self.assertEqual(self.run_session(client),0)
        status = json.loads(Path(self.context['status_file']).read_text())
        self.assertEqual(status['reason'],'diagnostic_session_deadline')
        self.assertEqual(client.close_count,1)

    def test_cancel_during_handshake_does_not_pause_aircraft(self):
        client = FakeClient(never_connect=True)
        cancel = threading.Event()
        def sleep(seconds):
            self.sleep(seconds)
            cancel.set()
        self.assertEqual(mode.run(self.root,factory=lambda:client,lease_factory=nullcontext,
            clock=lambda:self.now,sleep=sleep,output=self.output,cancelled=cancel),0)
        self.assertEqual(client.close_count,1)
        self.assertFalse(any(name.startswith('Pause') for name in client._bridge.calls))

    def test_connection_loss_stops_without_reopening_or_preparing(self):
        client = FakeClient()
        def output(text):
            self.output(text)
            if text.startswith('IDLE_CONNECTED'): client.connected = False
        self.assertEqual(self.run_session(client,output=output),1)
        status = json.loads(Path(self.context['status_file']).read_text())
        self.assertIn('disconnected',status['reason'])
        self.assertEqual(status['native_calls']['ConnectSim'],1)

    def test_unknown_pause_observation_is_explicit(self):
        client = FakeClient()
        client.simulation_running = Mock(side_effect=RuntimeError('Pause event not received'))
        self.assertEqual(self.run_session(client,output=self.close_when_ready),0)
        status = json.loads(Path(self.context['status_file']).read_text())
        self.assertIn('UNVERIFIED',status['pause_observation_note'])

    def test_record_initialize_replay_pid_and_shutdown_commands_cannot_dispatch(self):
        context = dict(session_kind=mode.KIND,command_directory=str(self.root),
            status_file=str(self.root/'status.json'))
        client = FakeClient()
        window = SimpleNamespace(p3d=client,_session_context=context)
        for action in ('initialize','record','stop','shutdown','pid_engage','pid_tune','replay'):
            with self.subTest(action=action),self.assertRaises(ValueError):
                mode.dispatch(window,context,dict(action=action,expires_at=time.time()+5))

    def test_expired_command_rejected_before_snapshot_start(self):
        with self.assertRaisesRegex(ValueError,'expired'):
            mode.dispatch(SimpleNamespace(),{},dict(action=diagnostic.START,expires_at=0))

    def test_existing_caller_uses_diagnostic_context_without_qtg_engine_sender(self):
        context = dict(session_kind=mode.KIND,command_directory=str(self.root))
        with patch.object(mode,'enqueue') as send:
            console.send(context,diagnostic.START,max_age_s=1.)
            console.send(context,diagnostic.STOP)
            console.send(context,mode.CLOSE)
            self.assertEqual([c.args[1] for c in send.call_args_list],
                [diagnostic.START,diagnostic.STOP,mode.CLOSE])

    def test_cli_explicit_mode_calls_only_diagnostic_runner(self):
        with patch.object(mode,'run',return_value=0) as run:
            self.assertEqual(console.main(['diagnostic-only','--session-root',str(self.root)]),0)
            run.assert_called_once_with(str(self.root),connection_timeout_s=15.,session_timeout_s=300.)

    def test_close_refuses_normal_qtg_worker(self):
        with patch.object(console,'load_context',return_value={}),patch.object(console,'send') as send:
            self.assertEqual(console.main(['--context','unused','close']),1)
            send.assert_not_called()

    def test_close_waits_for_owned_session_shutdown_confirmation(self):
        path = self.root/'status.json'
        path.write_text(json.dumps(dict(phase='CLOSED',reason='operator_close')))
        context = dict(session_kind=mode.KIND,status_file=str(path))
        with patch.object(console,'load_context',return_value=context),patch.object(console,'send') as send:
            self.assertEqual(console.main(['--context','unused','close']),0)
            send.assert_called_once_with(context,mode.CLOSE)

    def test_changed_installed_library_is_rejected_before_client_creation(self):
        wrong = self.root/'wrong.dll'
        wrong.write_bytes(b'not the reviewed DLL')
        with self.assertRaisesRegex(RuntimeError,'reviewed'): mode.validate_library(wrong)

    def test_client_cannot_select_a_different_dll(self):
        client = FakeClient()
        native = client._bridge
        client._dll_path = str(self.root/'other.dll')
        self.assertEqual(self.run_session(client),1)
        native.ConnectSim.assert_not_called()

    def test_shutdown_timeout_is_visible_and_thread_is_daemon(self):
        client = FakeClient()
        fake_thread = Mock(is_alive=Mock(return_value=True))
        with patch.object(mode.threading,'Thread',return_value=fake_thread) as factory:
            self.assertEqual(self.run_session(client),1)
            self.assertTrue(factory.call_args.kwargs['daemon'])
        fake_thread.join.assert_called_once_with(3.)
        status = json.loads(Path(self.context['status_file']).read_text())
        self.assertEqual(status['phase'],'FAILED')
        self.assertIn('shutdown deadline',status['reason'])

    @unittest.skipUnless(os.name == 'nt','Windows session ownership mutex')
    def test_real_session_mutex_rejects_duplicate_owner_without_connection(self):
        with mode.SingleSession():
            with self.assertRaisesRegex(RuntimeError,'already running'):
                with mode.SingleSession(): self.fail('Duplicate mutex accepted')

    def test_enabled_manager_rejected_before_client_creation(self):
        factory = Mock()
        for flag in diagnostic.SWITCHES:
            with patch.dict(os.environ,{flag:'1'}),self.assertRaisesRegex(RuntimeError,'Disable'):
                mode.run(self.root,factory=factory,lease_factory=nullcontext)
        factory.assert_not_called()

    def test_existing_session_lease_prevents_second_client(self):
        factory = Mock()
        with self.assertRaisesRegex(RuntimeError,'already running'):
            mode.run(self.root,factory=factory,
                lease_factory=Mock(side_effect=RuntimeError('already running')))
        factory.assert_not_called()

    def test_contexts_are_unique_across_repeated_sessions(self):
        self.assertEqual(self.run_session(FakeClient(),output=self.close_when_ready),0)
        previous = self.context['context_file']
        self.now = 0.
        self.assertEqual(self.run_session(FakeClient(),output=self.close_when_ready),0)
        self.assertNotEqual(self.context['context_file'],previous)
        self.assertTrue(Path(previous).is_file())

    def test_actual_existing_python_poller_with_false_placement_flags(self):
        client = object.__new__(api.SimConnectClient)
        native = SimpleNamespace(ConnectSim=Mock(return_value=1),ProcessMessages=Mock(),
            IsSimConnected=Mock(return_value=1),GetTelemetryDataV1507=Mock(),DisconnectSim=Mock())
        client._bridge = mode.ReadOnlyBridge(native)
        client._lock = threading.Lock()
        client._stop_requested = threading.Event()
        client._connected = False
        client.last_error = ''
        client._update_latest_data = Mock(side_effect=lambda _:client.stop())
        client.set_position_rahwali = Mock(side_effect=AssertionError('No placement'))
        client.set_position_airborne = Mock(side_effect=AssertionError('No placement'))
        client.connect_and_run(on_connected=None,on_error=None,
            place_at_rahwali=False,place_airborne=False)
        native.ConnectSim.assert_called_once()
        native.DisconnectSim.assert_called_once()
        client.set_position_rahwali.assert_not_called()
        client.set_position_airborne.assert_not_called()

    def test_native_connection_and_dispatch_have_no_simulator_write_calls(self):
        root = Path(mode.__file__).parents[1]/'interfaces/brunner_base/native/src'
        source = (root/'p3d_bridge_v15_full_causal_state.cpp').read_text()
        header = (root/'verification_snapshot_v1.h').read_text()
        connect = re.search(r'int ConnectSim\(\) \{(.*?)\n    \}',source,re.S).group(1)
        callback = source.split('void CALLBACK MyDispatchProc',1)[1].split('\nextern "C"',1)[0]
        close = re.search(r'void DisconnectSim\(\) \{(.*?)\n    \}',source,re.S).group(1)
        self.assertEqual(set(re.findall(r'\b(SimConnect_\w+)\(',connect)),
            {'SimConnect_Open','SimConnect_MapClientEventToSimEvent'})
        self.assertEqual(set(re.findall(r'\b(SimConnect_\w+)\(',callback)),
            {'SimConnect_RequestDataOnSimObject'})
        self.assertEqual(set(re.findall(r'\b(SimConnect_\w+)\(',close)),{'SimConnect_Close'})
        tracked = source.split('static HRESULT TrackedAddToDataDefinition',1)[1].split('\n}',1)[0]
        self.assertEqual(set(re.findall(r'\b(SimConnect_\w+)\(',tracked)),
            {'SimConnect_AddToDataDefinition','SimConnect_GetLastSentPacketID'})
        observer = re.search(r'int RequestSimulationRunningV1509\(\) \{(.*?)\n    \}',source,re.S).group(1)
        self.assertEqual(set(re.findall(r'\b(SimConnect_\w+)\(',observer)),
            {'SimConnect_SubscribeToSystemEvent','SimConnect_UnsubscribeFromSystemEvent'})
        for forbidden in ('SimConnect_SetDataOnSimObject','SimConnect_TransmitClientEvent',
                          'SimConnect_WeatherSetObservation'):
            self.assertNotIn(forbidden,header)


if __name__ == '__main__': unittest.main()
