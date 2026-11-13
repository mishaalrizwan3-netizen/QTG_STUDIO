"""Offline diagnostic/IPC checks. No native DLL load or real hardware clients."""
import ctypes
import importlib.util
import json
import hashlib
import os
from pathlib import Path
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import qtg_common_engine as engine
from backend import p3d_verification_diagnostic as diagnostic
from interfaces.brunner_base.src.verification_snapshot import CHANNELS, VerificationSnapshot

spec = importlib.util.spec_from_file_location('verification_console',
    Path(__file__).parents[1]/'tools/p3d_verification_diagnostic.py')
cli = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cli)


class Function:
    def __init__(self, function): self.function = function
    def __call__(self, *args): return self.function(*args)


class DiagnosticTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.now = 100.
        self.raw = dict(version=1, generation=1, request_id=1001, sequence=1,
            receipt_ticks=9000, copied_ticks=9010, frequency=1000,
            measured_at=99.99, valid_mask=(1 << 23)-1, running=False,
            values={key: 0. for key,_,_ in CHANNELS}, provenance='Synthetic native response')
        self.raw['values'].update(on_ground=1., flaps=0., sim_time=50., engine_rpm=983., prop_rpm=984.)
        self.client = SimpleNamespace(is_connected=Mock(return_value=True),
            verification_snapshot=Mock(side_effect=lambda: self.raw))
        for name in ('connect', 'connect_and_run', 'stop', 'pause_sim', 'unpause_sim', 'set_throttle'):
            setattr(self.client, name, Mock(side_effect=AssertionError('Forbidden '+name)))
        self.context = dict(status_file=str(self.root/'worker.json'),
                            command_directory=str(self.root/'commands'))
        self.window = SimpleNamespace(_session_context=self.context, p3d=self.client,
            _preparing=False, is_recording=False, _shutting_down=False)
        self.environment = patch.dict(os.environ, {key:'0' for key in diagnostic.SWITCHES})
        self.environment.start()
        self.addCleanup(self.environment.stop)

    def monitor(self, **options):
        return diagnostic.SnapshotDiagnostic(self.client, self.root/'new',
            diagnostic.status_path(self.context), max_age_s=1., clock=lambda:self.now, **options)

    def advance(self):
        self.now += .1
        self.raw.update(request_id=self.raw['request_id']+1, sequence=self.raw['sequence']+1,
            receipt_ticks=self.raw['receipt_ticks']+100, copied_ticks=self.raw['copied_ticks']+100,
            measured_at=self.now-.01)

    def native_bridge(self):
        state = VerificationSnapshot()
        state.size, state.version, state.connected = ctypes.sizeof(state), 1, 1
        state.generation, state.sequence, state.request_id = 1, 1, 1001
        state.receipt_ticks, state.copied_ticks, state.frequency = 9000, 9010, 1000
        state.valid_mask, state.running = (1 << 23)-1, 0
        state.values[0] = 1.
        requests = []
        def request():
            token = 1002+len(requests)
            requests.append(token)
            state.request_id = token
            state.sequence += 1
            state.receipt_ticks += 10
            state.copied_ticks += 10
            return token
        def copy(pointer, size):
            ctypes.memmove(pointer, ctypes.byref(state), size)
            return 1
        self.client._bridge = SimpleNamespace(RequestVerificationV1=Function(request),
            GetVerificationV1=Function(copy))
        return state, requests

    def test_all_23_sources_units_and_separate_engine_prop_ground_flaps(self):
        frame = self.monitor().sample()
        self.assertEqual(frame['state'], 'FRESH_NATIVE_RESPONSE')
        self.assertEqual(len(frame['measurements']), 23)
        values = {item['channel']:item for item in frame['measurements']}
        self.assertEqual(values['on_ground']['value'], 1)
        self.assertEqual(values['flaps']['value'], 0)
        self.assertEqual(values['flaps']['source'], 'FLAPS HANDLE INDEX')
        self.assertEqual(values['engine_rpm']['value'], 983)
        self.assertEqual(values['prop_rpm']['value'], 984)
        self.assertEqual([(i['channel'],i['source'],i['unit']) for i in frame['measurements']], list(CHANNELS))
        self.assertFalse(frame['recording_authorization'])
        self.assertIsNone(frame['native_acquired_at'])

    def test_fresh_unchanged_paused_values_and_running_progress(self):
        monitor = self.monitor()
        monitor.sample()
        self.advance()
        frame = monitor.sample()
        self.assertEqual(frame['state'], 'FRESH_NATIVE_RESPONSE')
        self.assertEqual(frame['simulator_time_progression'], 'UNCHANGED')
        self.raw['running'] = True
        self.advance()
        frame = monitor.sample()
        self.assertIn('not demonstrated', frame['reason'])
        self.raw['values']['sim_time'] += .2
        self.advance()
        self.assertEqual(monitor.sample()['simulator_time_progression'], 'ADVANCED')

    def test_unknown_running_state_is_not_assumed_running(self):
        self.raw['running'] = None
        self.assertIn('unknown', self.monitor().sample()['reason'])

    def test_cached_and_superseded_identity_not_fresh(self):
        monitor = self.monitor()
        monitor.sample()
        self.assertEqual(monitor.sample()['state'], 'CACHED_OR_SUPERSEDED_RESPONSE')
        self.raw['request_id'] -= 1
        self.assertEqual(monitor.sample()['state'], 'CACHED_OR_SUPERSEDED_RESPONSE')

    def test_advancing_sequence_without_new_receipt_is_not_fresh(self):
        monitor = self.monitor()
        monitor.sample()
        self.raw['request_id'] += 1
        self.raw['sequence'] += 1
        self.assertEqual(monitor.sample()['state'], 'CACHED_OR_SUPERSEDED_RESPONSE')

    def test_stale_native_receipt(self):
        self.raw['copied_ticks'] += 2000
        frame = self.monitor().sample()
        self.assertEqual(frame['state'], 'STALE_NATIVE_RESPONSE')
        self.assertFalse(frame['receipt_within_diagnostic_age'])

    def test_disconnect_reconnect_and_old_connection_callback(self):
        monitor = self.monitor()
        monitor.sample()
        self.client.is_connected.return_value = False
        self.assertEqual(monitor.sample()['state'], 'DISCONNECTED')
        self.client.is_connected.return_value = True
        self.advance()
        self.raw['generation'] += 1
        self.raw['sequence'] = 1
        self.assertEqual(monitor.sample()['connection_transition'], 'RECONNECTED')
        self.raw['generation'] -= 1
        self.assertEqual(monitor.sample()['state'], 'CACHED_OR_SUPERSEDED_RESPONSE')

    def test_nonfinite_missing_invalid_ground_flap_not_filled(self):
        self.raw['values'].update(engine_rpm=float('nan'), on_ground=2., flaps=.5)
        self.raw['valid_mask'] &= ~(1 << 13)
        frame = self.monitor().sample()
        values = {i['channel']:i for i in frame['measurements']}
        for key in ('engine_rpm','on_ground','flaps','temperature_c'):
            self.assertEqual(values[key]['display'], 'Not available')
        json.dumps(frame, allow_nan=False)

    def test_malformed_clock_mask_version_and_payload(self):
        for key, value in [('frequency',0), ('measured_at',101.), ('valid_mask',-1),
                           ('version',2), ('request_id',0), ('values',None)]:
            old = self.raw[key]
            self.raw[key] = value
            with self.subTest(key=key): self.assertEqual(self.monitor().sample()['state'], 'UNAVAILABLE')
            self.raw[key] = old

    def test_unsupported_dll_and_cancellation(self):
        self.client.verification_snapshot.side_effect = RuntimeError('Native verification V1 unsupported')
        self.assertIn('unsupported', self.monitor().sample()['reason'])
        monitor = self.monitor()
        monitor.stop()
        count = self.client.verification_snapshot.call_count
        monitor.sample()
        self.assertEqual(self.client.verification_snapshot.call_count, count)

    def test_cancellation_during_read_does_not_publish_fresh(self):
        monitor = self.monitor()
        def read():
            monitor.stop()
            return self.raw
        self.client.verification_snapshot.side_effect = read
        self.assertEqual(monitor.sample()['state'], 'UNAVAILABLE')

    def test_only_borrowed_read_interface_called(self):
        monitor = self.monitor()
        monitor.sample()
        monitor.stop()
        for name in ('connect','connect_and_run','stop','pause_sim','unpause_sim','set_throttle'):
            getattr(self.client,name).assert_not_called()

    def test_cache_probe_issues_one_seed_request_only(self):
        state, requests = self.native_bridge()
        monitor = self.monitor(probe='cache')
        monitor.sample()
        self.now += 2.
        state.copied_ticks += 2000
        frame = monitor.sample()
        self.assertEqual(frame['state'], 'CACHED_COPY_NOT_REQUESTED')
        self.assertFalse(frame['receipt_within_diagnostic_age'])
        self.client.verification_snapshot.assert_called_once()
        self.assertEqual(requests, [])

    def test_supersession_probe_accepts_newest_request_only(self):
        _, requests = self.native_bridge()
        frame = self.monitor(probe='superseded').sample()
        self.assertEqual(requests, [1002,1003])
        self.assertEqual(frame['request_id'], 1003)
        self.assertEqual(frame['state'], 'FRESH_NATIVE_RESPONSE')
        self.assertIn('Not observable', frame['supersession_probe']['obsolete_callback_delivery'])
        self.client.verification_snapshot.assert_not_called()

    def test_supersession_probe_ignores_old_export_copy(self):
        state, requests = self.native_bridge()
        original = self.client._bridge.GetVerificationV1.function
        copies = [0]
        def copy(pointer, size):
            copies[0] += 1
            current = state.request_id
            if copies[0] == 2: state.request_id = requests[0]
            result = original(pointer,size)
            state.request_id = current
            return result
        self.client._bridge.GetVerificationV1.function = copy
        frame = self.monitor(probe='superseded').sample()
        self.assertEqual(frame['request_id'],1003)
        self.assertEqual(frame['supersession_probe']['ignored_old_export_copies'],1)

    def test_cache_probe_never_claims_correlation_for_another_readers_packet(self):
        state, _ = self.native_bridge()
        monitor = self.monitor(probe='cache')
        monitor.sample()
        self.advance()
        state.request_id, state.sequence, state.receipt_ticks, state.copied_ticks = 2000,20,9100,9110
        self.assertEqual(monitor.sample()['state'],'CACHED_COPY_NOT_REQUESTED')

    def test_busy_workers_and_both_manager_flags_refuse_start(self):
        for name in ('_preparing','is_recording','pid_controller_active','_shutting_down'):
            setattr(self.window,name,True)
            with self.subTest(name=name): self.assertIn('busy', diagnostic.idle_reason(self.window))
            setattr(self.window,name,False)
        self.window.worker_thread = Mock(is_alive=Mock(return_value=True))
        self.assertIn('worker_thread', diagnostic.idle_reason(self.window))
        self.window.worker_thread = None
        for flag in diagnostic.SWITCHES:
            with patch.dict(os.environ,{flag:'1'}): self.assertIn('disabled', diagnostic.idle_reason(self.window))

    def test_command_rejection_is_private_and_cannot_change_readiness(self):
        self.window._prepared = True
        self.window.is_recording = True
        diagnostic.handle_command(self.window,self.context,dict(action=diagnostic.START,max_age_s=1.,session_id='a'))
        result = json.loads(diagnostic.status_path(self.context).read_text())
        self.assertEqual(result['state'],'REJECTED')
        self.assertTrue(self.window._prepared)
        self.assertFalse(Path(self.context['status_file']).exists())
        self.client.verification_snapshot.assert_not_called()

    def test_private_status_permission_failure_does_not_escape(self):
        self.window.is_recording = True
        with patch.object(diagnostic,'publish',side_effect=PermissionError('locked')):
            self.assertTrue(diagnostic.handle_command(self.window,self.context,
                dict(action=diagnostic.START,max_age_s=1.)))

    def test_expired_command_performs_no_simulator_read(self):
        diagnostic.handle_command(self.window,self.context,
            dict(action=diagnostic.START,max_age_s=1.,expires_at=0.))
        self.assertIn('expired',json.loads(diagnostic.status_path(self.context).read_text())['reason'])
        self.client.verification_snapshot.assert_not_called()

    def test_repeated_runs_log_independently_and_stop_has_no_cleanup_authority(self):
        for index in range(2):
            monitor = diagnostic.SnapshotDiagnostic(self.client,self.root/str(index),
                diagnostic.status_path(self.context),max_age_s=1.,duration_s=.02,interval_s=.1)
            monitor.start()
            monitor.thread.join(2.)
            self.assertFalse(monitor.thread.is_alive())
            summary = json.loads(Path(monitor.outputs['summary']).read_text())
            self.assertFalse(summary['active'])
            self.assertEqual(summary['samples'],1)
            self.assertTrue(Path(monitor.outputs['csv']).is_file())
            self.assertTrue(Path(monitor.outputs['jsonl']).is_file())
            self.assertEqual(len(summary['channels']),23)
            self.assertEqual(len(summary['connection_transitions']),1)
            self.client.stop.assert_not_called()

    def test_old_stop_cannot_cancel_new_session(self):
        monitor = self.monitor()
        self.window._p3d_verification_diagnostic = monitor
        diagnostic.handle_command(self.window,self.context,dict(action=diagnostic.STOP,session_id='old'))
        self.assertFalse(monitor.cancelled.is_set())
        diagnostic.handle_command(self.window,self.context,dict(action=diagnostic.STOP,session_id=monitor.session_id))
        self.assertTrue(monitor.cancelled.is_set())

    def test_ordinary_qtg_commands_only_stop_diagnostic_event(self):
        monitor = self.monitor()
        self.window._p3d_verification_diagnostic = monitor
        diagnostic.stop_for_activity(self.window,'record')
        self.assertTrue(monitor.cancelled.is_set())
        self.client.stop.assert_not_called()

    def test_console_only_queues_two_diagnostic_actions(self):
        with patch.object(engine,'send_command') as send:
            cli.send(self.context,diagnostic.START,max_age_s=1.)
            cli.send(self.context,diagnostic.STOP)
            with self.assertRaises(ValueError): cli.send(self.context,'initialize')
            self.assertEqual([c.args[1] for c in send.call_args_list],[diagnostic.START,diagnostic.STOP])

    def test_console_refuses_stale_worker_context(self):
        Path(self.context['command_directory']).mkdir()
        Path(self.context['status_file']).write_text(json.dumps(dict(self.context,updated=90.)))
        context_path = self.root/'context.json'
        context_path.write_text(json.dumps(self.context))
        with self.assertRaisesRegex(ValueError,'heartbeat'): cli.load_context(context_path,wall_clock=lambda:100.)

    def test_console_missing_values_and_unconfirmed_saved_files(self):
        frame = self.monitor().sample()
        frame['measurements'][0]['display']='Not available'
        frame.update(active=False,outputs={'summary':str(self.root/'absent.json')})
        text = cli.render(frame)
        self.assertIn('Not available',text)
        self.assertIn('NOT CONFIRMED SAVED',text)
        self.assertIn('FLAPS HANDLE INDEX',text)
        self.assertIn('historical',text)
        self.assertIn('Last observation UTC',text)

    def test_worker_routes_start_without_initialization_or_recording(self):
        base = SimpleNamespace(SuperMushakLoggerApp=object)
        worker = object.__new__(engine.worker_class(base,self.context,lambda _:None))
        worker.__dict__.update(self.window.__dict__)
        worker._prepared = True
        worker.rec_time = [0.]
        worker.arm_recording = Mock(side_effect=AssertionError('No recording'))
        with patch.object(diagnostic,'SnapshotDiagnostic') as factory:
            factory.return_value.thread = Mock(is_alive=Mock(return_value=False))
            worker.handle_command(dict(action=diagnostic.START,max_age_s=1.,session_id='test'))
            factory.assert_called_once()
            self.assertIs(factory.call_args.args[0],self.client)
            factory.return_value.start.assert_called_once()
            self.assertTrue(worker._prepared)
            self.assertEqual(worker.rec_time,[0.])
            worker.arm_recording.assert_not_called()

    def test_worker_shutdown_waits_for_reader_without_closing_shared_client(self):
        base = SimpleNamespace(SuperMushakLoggerApp=object)
        worker = object.__new__(engine.worker_class(base,self.context,lambda _:None))
        worker.__dict__.update(self.window.__dict__)
        monitor = Mock(thread=Mock(is_alive=Mock(return_value=True)))
        worker._p3d_verification_diagnostic = monitor
        self.assertFalse(worker.finish_shutdown())
        monitor.stop.assert_called_once_with('worker_shutdown')
        self.client.stop.assert_not_called()

    def test_console_wait_ignores_old_status_and_stops_only_its_session(self):
        diagnostic.status_path(self.context).write_text(json.dumps(dict(session_id='old',active=False)))
        now = [0.]
        def sleep(_): now[0] += 10.
        with patch.object(cli,'send') as send:
            with self.assertRaises(RuntimeError):
                cli.follow(self.context,'new',1.,clock=lambda:now[0],sleep=sleep,output=Mock())
            send.assert_called_once_with(self.context,diagnostic.STOP,session_id='new')

    def test_source_command_queue_accepts_only_diagnostic_action_without_qtg_start(self):
        engine.send_command(self.context,diagnostic.START,max_age_s=1.)
        files = list(Path(self.context['command_directory']).glob('*.json'))
        self.assertEqual(len(files),1)
        self.assertEqual(json.loads(files[0].read_text())['action'],diagnostic.START)

    def test_active_universal_sampler_is_rejected_even_if_switches_off(self):
        self.window._engine_initialization_adapter = Mock(thread=Mock(is_alive=Mock(return_value=True)))
        self.assertIn('still active',diagnostic.idle_reason(self.window))

    def test_library_provenance_is_read_only_and_not_claimed_as_memory_hash(self):
        path = self.root/'fake.dll'
        path.write_bytes(b'offline fixture, not a DLL')
        self.client._dll_path = str(path)
        frame = self.monitor().sample()
        self.assertEqual(frame['library']['file_sha256'],hashlib.sha256(path.read_bytes()).hexdigest())
        self.assertIn('not a hash of loaded memory',frame['library']['note'])


if __name__ == '__main__': unittest.main()
