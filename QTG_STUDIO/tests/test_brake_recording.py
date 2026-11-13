"""Offline recording-sidecar tests. No simulator or native executable is started."""
import csv
import ast
import importlib.util
import json
import os
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from backend import brake_recording as brake
from backend.recorder import snapshot

spec=importlib.util.spec_from_file_location('brake_capture_console',Path(__file__).parents[1]/'tools/deceleration_brake_capture.py')
console=importlib.util.module_from_spec(spec);spec.loader.exec_module(console)


class BrakeRecordingTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name)
        self.anchor=dict(recording_id='run1',mode='manual',test_reference='1.f.(2)',t0_perf_counter=100.,
                         mapping=dict(verified=True,frequency=1000))
        self.context=dict(test_reference='1.f.(2)',mode='manual',control_method='replay',
            status_file=str(self.root/'worker.json'),command_directory=str(self.root/'commands'),result_directory=str(self.root))
        self.context_path=self.root/'context.json';self.context_path.write_text(json.dumps(self.context))
        self.csv=self.root/'run.csv'
        self.csv.write_text('Time (s),Throttle 1 Position (%),Ground Speed (kt)\n'+
            ''.join(f'{i},{100 if i<4 else 5},{i}\n' for i in range(8)))
        self.csv.with_suffix('.qtg_test.json').write_text(json.dumps(dict(self.context,parking_brake_recording_anchor=self.anchor)))
        self.native=self.root/'brake_samples.csv'
        self.fields=['pid','connection_generation','request_id','channel_sequence','receipt_qpc_ticks',
            'qpc_frequency','request_to_receipt_s','simvar','value','status']
        self.write_native([(-.1,0),(.9,1),(1.9,1),(2.9,0),(3.9,1),(4.9,1),(5.9,0),(6.9,0)])

    def write_native(self,values):
        rows=[]
        for seq,(stamp,value) in enumerate(values,1):
            base=dict(pid=42,connection_generation=1,request_id=100+seq,channel_sequence=seq,
                receipt_qpc_ticks=round((100+stamp)*1000),qpc_frequency=1000,request_to_receipt_s=.02,
                status='NEW_CORRELATED_RESPONSE')
            rows += [dict(base,simvar='BRAKE PARKING POSITION',value=value),dict(base,simvar='SIM TIME',value=1000+stamp)]
        with self.native.open('w',newline='') as stream:
            writer=csv.DictWriter(stream,fieldnames=self.fields);writer.writeheader();writer.writerows(rows)

    def test_flag_off_does_not_touch_window_or_clock(self):
        window=SimpleNamespace(_session_context=self.context)
        with patch.dict(os.environ,{brake.FLAG:'0'}),patch.object(brake,'qpc_mapping') as clock:
            brake.mark_start(window,100.)
        clock.assert_not_called();self.assertFalse(hasattr(window,'_parking_brake_recording_anchor'))

    def test_manual_and_replay_use_exact_existing_timer_origin(self):
        for mode in ('manual','auto'):
            window=SimpleNamespace(_session_context=dict(self.context,mode=mode))
            with patch.dict(os.environ,{brake.FLAG:'1'}),patch.object(brake,'qpc_mapping',return_value=self.anchor['mapping']):
                brake.install_clock_marker(window)
                window._parking_brake_clock_marker(123.456)
            self.assertEqual(window._parking_brake_recording_anchor['t0_perf_counter'],123.456)
            self.assertEqual(window._session_context['parking_brake_recording_anchor']['mode'],mode)

    def test_acceleration_pid_and_unrelated_tests_have_no_marker(self):
        for changes in (dict(test_reference='1.f.(1)'),dict(control_method='pid'),dict(mode='pid')):
            window=SimpleNamespace(_session_context=dict(self.context,**changes))
            with patch.dict(os.environ,{brake.FLAG:'1'}):brake.mark_start(window,100.)
            self.assertFalse(hasattr(window,'_parking_brake_recording_anchor'))

    def test_status_exposes_marker_only_when_present(self):
        window=SimpleNamespace(p3d=Mock(is_connected=Mock(return_value=True)),brunner=SimpleNamespace(sock=True),
            rec_time=[],target_duration_sec=21,is_recording=False,data_dir=self.root,current_stage='Ready')
        self.assertNotIn('parking_brake_recording_anchor',snapshot(window,self.context))
        window._parking_brake_recording_anchor=self.anchor
        self.context['parking_brake_recording_anchor']=self.anchor
        self.assertEqual(snapshot(window,self.context)['parking_brake_recording_anchor'],self.anchor)

    def test_zero_is_off_and_missing_or_stale_is_not_off(self):
        samples,issues=brake.samples_from_csv(self.native,self.anchor)
        self.assertFalse(issues)
        self.assertEqual(brake.at_time(samples,0.,.5)['value'],0)
        self.assertIsNone(brake.at_time(samples,.7,.5)['value'])
        self.assertIsNone(brake.at_time([],0.,1.)['value'])

    def test_recent_receipt_with_delayed_request_is_not_claimed_fresh(self):
        samples,_=brake.samples_from_csv(self.native,self.anchor)
        samples[0]['request_to_receipt_s']=1.5
        self.assertIsNone(brake.at_time(samples,0.,.5)['value'])

    def test_qpc_domain_or_frequency_mismatch_cannot_align(self):
        anchor=dict(self.anchor,mapping=dict(verified=False,frequency=1000))
        self.assertFalse(brake.samples_from_csv(self.native,anchor)[0])
        anchor=dict(self.anchor,mapping=dict(verified=True,frequency=999))
        self.assertFalse(brake.samples_from_csv(self.native,anchor)[0])

    def test_cached_and_reordered_responses_are_not_new_samples(self):
        samples,_=brake.samples_from_csv(self.native,self.anchor)
        with self.native.open() as stream:rows=list(csv.DictReader(stream))
        with self.native.open('a',newline='') as stream:
            csv.DictWriter(stream,fieldnames=self.fields).writerows(rows[:2])
        updated,issues=brake.samples_from_csv(self.native,self.anchor)
        self.assertEqual(len(updated),len(samples));self.assertTrue(issues)

    def test_invalid_reading_blocks_reuse_of_previous_off(self):
        with self.native.open(newline='') as stream:rows=list(csv.DictReader(stream))[:2]
        invalid=dict(rows[0],status='INVALID',value='nan',receipt_qpc_ticks=100100,request_id=102,channel_sequence=2)
        with self.native.open('w',newline='') as stream:
            writer=csv.DictWriter(stream,fieldnames=self.fields);writer.writeheader();writer.writerows(rows+[invalid])
        samples,_=brake.samples_from_csv(self.native,self.anchor)
        self.assertIsNone(brake.at_time(samples,.2,1.)['value'])

    def test_observer_stop_invalidates_later_state(self):
        samples,_=brake.samples_from_csv(self.native,dict(self.anchor,capture_end_perf_counter=107.))
        self.assertIsNone(brake.at_time(samples,7.1,1.)['value'])

    def test_alignment_uses_measured_onset_and_preserves_original_csv(self):
        before=self.csv.read_bytes();folder=self.root/'aligned';folder.mkdir()
        report=brake.align_capture(self.native,self.anchor,self.csv,folder,max_age_s=1.)
        self.assertEqual(self.csv.read_bytes(),before)
        self.assertEqual(report['recording_start']['value'],0)
        self.assertIsNotNone(report['event_time_s'])
        self.assertEqual(report['immediately_pre_throttle']['value'],0)
        self.assertLess(report['immediately_pre_throttle']['sample_time_s'],report['event_time_s'])
        self.assertIn('NOT REQUESTED',report['wheel_brakes'])
        with self.assertRaises(FileExistsError):brake.align_capture(self.native,self.anchor,self.csv,folder,max_age_s=1.)

    def test_wrong_run_csv_is_rejected(self):
        metadata=self.csv.with_suffix('.qtg_test.json')
        metadata.write_text(json.dumps(dict(self.context,parking_brake_recording_anchor=dict(self.anchor,recording_id='old'))))
        with self.assertRaisesRegex(ValueError,'does not match'):
            brake.align_capture(self.native,self.anchor,self.csv,self.root,max_age_s=1.)
        self.assertIsNone(console.choose_csv(dict(outputs=[str(self.csv)]),'run1'))

    def test_native_owner_requests_only_pair_and_closes_only_owned_process(self):
        process=Mock();factory=Mock(return_value=process)
        owner=console.NativeOwner('mock.exe',self.root,30,popen=factory)
        command=factory.call_args.args[0]
        self.assertIn('--parking-only',command);self.assertIn('--owner-pid',command)
        owner.stop();process.wait.assert_called_once_with(timeout=5.)
        self.assertTrue(owner.stop_file.exists())

    def fake_capture(self, statuses, *, launch_failure=False, process_exit=None):
        exe=self.root/'mock.exe';exe.write_bytes(b'never executed')
        exe.with_name('manifest.json').write_text(json.dumps(dict(parking_pair_protocol=1,executable_sha256=brake.digest(exe))))
        owner=Mock();owner.process.poll.return_value=process_exit
        def factory(executable,folder,duration):
            if launch_failure:raise OSError('mock connection process unavailable')
            native=folder/'native/brakes_fake';native.mkdir(parents=True)
            (native/'brake_samples.csv').write_bytes(self.native.read_bytes())
            return owner
        reader=Mock(side_effect=statuses)
        saved_states=iter(statuses[1:])
        def saved():
            state=next(saved_states,{})
            if state.get('outputs'):
                return self.csv,json.loads(self.csv.with_suffix('.qtg_test.json').read_text())
            return None
        with patch.dict(os.environ,{brake.FLAG:'1','QTG_UNIVERSAL_ENGINE_ACCELERATION':'0','QTG_UNIVERSAL_ENGINE_DECELERATION':'0'}),patch.object(console.time,'perf_counter',side_effect=[99.,108.]):
            result=console._capture(self.context_path,exe,self.root/'captures',duration_s=30,max_age_s=1.,
                owner_factory=factory,status_reader=reader,sleep=lambda _:None,metadata_reader=saved)
        self.status_reader=reader
        return result,owner

    def test_attach_recording_start_stop_and_saved_csv_binding(self):
        idle=dict(self.context,connected=True,recording=False)
        active=dict(idle,recording=True,parking_brake_recording_anchor=self.anchor)
        ended=dict(active,recording=False,outputs=[str(self.csv)])
        result,owner=self.fake_capture([idle,active,ended])
        self.assertEqual(result['alignment'],'SAVED_DIAGNOSTIC_ONLY');owner.stop.assert_called_once()
        self.assertEqual(result['anchor']['recording_id'],'run1')

    def test_connection_process_failure_does_not_start_or_stop_qtg(self):
        idle=dict(self.context,connected=True,recording=False)
        result,owner=self.fake_capture([idle,idle],process_exit=1)
        self.assertIsNone(result['anchor']);self.assertNotIn('alignment',result)
        self.assertIn('Observer exited',result['reason']);owner.stop.assert_called_once()

    def test_launch_failure_is_visible_without_changing_worker(self):
        result,_=self.fake_capture([dict(self.context,connected=True,recording=False)],launch_failure=True)
        self.assertIn('mock connection process unavailable',result['reason'])
        self.assertIsNone(result['anchor'])

    def test_attach_after_recording_started_is_refused_before_child(self):
        with patch.dict(os.environ,{brake.FLAG:'1'}),self.assertRaisesRegex(ValueError,'Attach before'):
            console._capture(self.context_path,'never.exe',self.root,
                status_reader=lambda _:dict(self.context,connected=True,recording=True))

    def test_stale_worker_heartbeat_is_not_live(self):
        status=dict(self.context,updated=94.)
        Path(self.context['status_file']).write_text(json.dumps(status))
        with patch.object(console.time,'time',return_value=100.),self.assertRaisesRegex(ValueError,'stale'):
            console.read_status(self.context)

    def test_repeated_run_ignores_previous_clock_marker(self):
        idle=dict(self.context,connected=True,recording=False,
            parking_brake_recording_anchor=dict(self.anchor,recording_id='old_run'))
        active=dict(idle,recording=True,parking_brake_recording_anchor=self.anchor)
        ended=dict(active,recording=False,outputs=[str(self.csv)])
        result,owner=self.fake_capture([idle,idle,active,ended])
        self.assertEqual(result['anchor']['recording_id'],'run1');owner.stop.assert_called_once()

    def test_only_owned_child_is_terminated_if_it_ignores_stop(self):
        import subprocess
        process=Mock();process.wait.side_effect=[subprocess.TimeoutExpired('mock.exe',5),None]
        owner=console.NativeOwner('mock.exe',self.root,30,popen=Mock(return_value=process))
        owner.stop();process.terminate.assert_called_once_with()
        self.assertEqual(process.wait.call_count,2)

    def test_real_host_qpc_mapping_without_simulator(self):
        if os.name!='nt':self.skipTest('Windows QPC')
        mapping=brake.qpc_mapping()
        self.assertTrue(mapping['verified']);self.assertGreater(mapping['frequency'],0)

    def test_existing_loop_hook_calls_marker_and_cannot_raise_into_recording(self):
        path=Path(__file__).parents[1]/'interfaces/brunner_base/Super_Mushak_Logger_GUI.py'
        tree=ast.parse(path.read_text(encoding='utf-8'))
        node=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='_observe_brake_recording_clock')
        scope={};exec(compile(ast.Module(body=[node],type_ignores=[]),str(path),'exec'),scope)
        hook=scope[node.name]
        window=SimpleNamespace();hook(window,100.)
        self.assertFalse(hasattr(window,'_parking_brake_recording_anchor'))
        window._parking_brake_clock_marker=Mock()
        hook(window,123.);window._parking_brake_clock_marker.assert_called_once_with(123.)
        window._parking_brake_clock_marker.side_effect=RuntimeError('clock evidence failed')
        hook(window,124.)
        self.assertIn('clock evidence failed',window._parking_brake_recording_anchor['error'])

    def test_duplicate_capture_lease_refused_without_opening_simconnect(self):
        if os.name!='nt':self.skipTest('Windows ownership mutex')
        with console.CaptureLease(self.context['status_file']):
            with self.assertRaisesRegex(ValueError,'already owns'):
                with console.CaptureLease(self.context['status_file']):self.fail('duplicate owner')

    def test_next_initialization_clears_previous_brake_clock_evidence(self):
        import qtg_common_engine as engine
        window=object.__new__(engine.worker_class(SimpleNamespace(SuperMushakLoggerApp=object),self.context,Mock()))
        window._session_context=dict(self.context,parking_brake_recording_anchor=self.anchor)
        window._parking_brake_recording_anchor=self.anchor
        window.sig_update_stage=Mock()
        window._begin_initialization()
        self.assertIsNone(window._parking_brake_recording_anchor)
        self.assertNotIn('parking_brake_recording_anchor',window._session_context)

    def test_active_capture_never_reopens_live_worker_status(self):
        idle=dict(self.context,connected=True,recording=False)
        active=dict(idle,recording=True,parking_brake_recording_anchor=self.anchor)
        ended=dict(active,recording=False,outputs=[str(self.csv)])
        result,_=self.fake_capture([idle,active,ended])
        self.assertEqual(result['alignment'],'SAVED_DIAGNOSTIC_ONLY')
        self.status_reader.assert_called_once()

    def test_completed_metadata_watcher_excludes_old_and_unrelated_runs(self):
        watcher=console.SavedRecordings(self.context)
        self.assertIsNone(watcher()) # existing recording is not a new run
        other=self.root/'new.csv';other.write_bytes(self.csv.read_bytes())
        meta=dict(self.context,command_directory='other_worker',parking_brake_recording_anchor=self.anchor)
        other.with_suffix('.qtg_test.json').write_text(json.dumps(meta))
        self.assertIsNone(watcher())
        meta['command_directory']=self.context['command_directory']
        other.with_suffix('.qtg_test.json').write_text(json.dumps(meta))
        self.assertEqual(watcher()[0],other)


if __name__=='__main__':unittest.main()
