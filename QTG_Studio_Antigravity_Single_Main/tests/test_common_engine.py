import csv
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import qtg_common_engine as engine


class CommonEngineTests(unittest.TestCase):
    def test_test_folders_are_distinct_and_named(self):
        with tempfile.TemporaryDirectory() as root:
            climb=engine.test_directory(root,'1.c.(1)','Normal climb -- all engines operating')
            acceleration=engine.test_directory(root,'1.f.(1)','Engine acceleration')
            self.assertNotEqual(climb,acceleration)
            self.assertEqual(climb.parent,Path(root).resolve())
            self.assertIn('Normal_climb',climb.name)

    def test_launch_routes_outputs_to_selected_test(self):
        with tempfile.TemporaryDirectory() as root:
            spec=SimpleNamespace(reference='1.c.(1)',title='Normal climb')
            with patch.object(engine.subprocess,'Popen'):
                _,context=engine.launch('manual',spec,Path(root)/'sessions')
            self.assertEqual(Path(context['result_directory']),engine.test_directory(root,spec.reference,spec.title))

    def test_output_filename_includes_test_name_and_preserves_companion_stem(self):
        prefix='Super_Mushak_manual_takeoff_data_2026-11-03_14-17-24'
        expected='1_c_1_Normal_climb_manual_2026-11-03_14-17-24'
        for suffix in ['.csv','_init_conditions.json','_graphs.pdf']:
            self.assertEqual(engine.named_result(prefix+suffix,'1.c.(1)','Normal climb'),expected+suffix)
        self.assertEqual(str(engine.RESULTS_ROOT).replace('\\','/'),'D:/QTG_Test_Results')

    def test_report_save_renames_csv_json_pdf_together(self):
        with tempfile.TemporaryDirectory() as root:
            folder=Path(root)
            context={'result_directory':root,'test_reference':'1.c.(1)','title':'Normal climb','mode':'manual'}
            class Base:
                def _save_csv_and_generate_reports(self):
                    for suffix in ['.csv','_init_conditions.json','_graphs.pdf']:
                        (self.data_dir/('Super_Mushak_manual_takeoff_data_2026-11-03_14-17-24'+suffix)).write_text('sample')
            module=SimpleNamespace(SuperMushakLoggerApp=Base,STAGE_COMPLETED='Completed')
            cls=engine.worker_class(module,context,lambda value:None)
            window=object.__new__(cls)
            window.data_dir=folder;window.lbl_rec_status=Mock();window._publish=Mock();window._outputs=[]
            window._save_csv_and_generate_reports()
            self.assertEqual(len(window._outputs),3)
            self.assertTrue(all(Path(p).name.startswith('1_c_1_Normal_climb_manual_') for p in window._outputs))
            self.assertTrue(list(folder.glob('*manual*.qtg_test.json')))
            self.assertFalse(list(folder.glob('Super_Mushak*')))

    def test_pid_save_preserves_csv_columns_and_adds_control_channels_and_metadata(self):
        with tempfile.TemporaryDirectory() as root:
            context=dict(test_reference='1.c.(1)',title='Normal climb',mode='auto',result_directory=root,duration_s=10)
            class Base:
                def _save_csv_and_generate_reports(self):
                    stem='Super_Mushak_auto_takeoff_data_2026-11-04_09-00-00'
                    (self.data_dir/(stem+'.csv')).write_text('Time (s),Indicated Airspeed (kts)\n0,80\n')
                    (self.data_dir/(stem+'_graphs.pdf')).write_bytes(b'%PDF')
                    (self.data_dir/(stem+'_evaluation.json')).write_text('{"result":"NOT EVALUATED"}')
            module=SimpleNamespace(SuperMushakLoggerApp=Base,STAGE_COMPLETED='Completed',STAGE_CONNECTION_ERROR='Failed')
            window=object.__new__(engine.worker_class(module,context,lambda state:None))
            window.data_dir=Path(root);window._control_method='pid';window._outputs=[];window._publish=Mock();window.lbl_rec_status=Mock()
            window.pid_fault=None;window.pid_stabilization={'achieved':True}
            window.pid_diagnostics=[dict(phase='recording',target_ias=80,command=.1,tracking_error=.02,applied_force=2)]
            window._save_csv_and_generate_reports()
            path=next(path for path in map(Path,window._outputs) if path.suffix=='.csv' and 'diagnostics' not in path.name)
            with path.open(newline='') as stream: row=next(csv.DictReader(stream))
            self.assertEqual(row['Time (s)'],'0')
            self.assertEqual(row['Indicated Airspeed (kts)'],'80')
            self.assertEqual(row['PID Target IAS (kts)'],'80')
            self.assertEqual(row['PID Tracking Error (-)'],'0.02')
            metadata=json.loads(path.with_suffix('.qtg_test.json').read_text())
            self.assertTrue(metadata['pid_stabilization']['achieved'])
            self.assertTrue(metadata['pid_complete'])
            self.assertEqual(metadata['duration_s'],10)

    def test_dashboard_uses_metadata_not_takeoff_filename(self):
        import main
        with tempfile.TemporaryDirectory() as root:
            folder=Path(root)
            (folder/'takeoff.qtg_test.json').write_text(json.dumps({'test_reference':'1.c.(1)','title':'Normal climb','mode':'manual'}))
            (folder/'takeoff_graphs.pdf').write_bytes(b'%PDF')
            with patch.object(main,'RESULTS_ROOT',folder),patch.object(main,'LOGS_FILE',folder/'test_logs.json'):
                main.sync_smk_generated_reports()
                logs=main.get_test_logs()
            self.assertEqual(logs[0]['reference'],'1.c.(1)')
            self.assertEqual(logs[0]['title'],'Normal climb')

    def test_manual_settings_are_applied_without_default_reposition_or_unpause(self):
        import threading
        cfg=dict(alt_ft=4500,ias_kt=95,lat=31.5,lon=73.25,flaps=1,throttle=65,temperature_c=25)
        actual=dict(pressure_alt=4500,plane_latitude=31.5,plane_longitude=73.25,measured_flaps_handle_index=1,engine_throttle_percent=65)
        sim=Mock();sim.is_connected.return_value=True;sim.wait_for_paused.return_value=True
        window=SimpleNamespace(p3d=sim,stop_worker_event=threading.Event(),_wait_for_initial_altitude=Mock(),_refresh_paused_telemetry=Mock(return_value=actual))
        engine.load_base()
        with patch('replay_support.reference_weather_metar',return_value=None):engine.apply_manual_conditions(window,cfg)
        sim.queue_position_airborne.assert_called_once_with(alt_ft=4500.,speed_kts=95.,pitch_deg=0.,bank_deg=0.,hdg_deg=0.,lat=31.5,lon=73.25,on_ground=False)
        sim.set_initial_throttle.assert_called_once_with(65.)
        sim.set_initial_flaps.assert_called_once_with(1.)
        sim.unpause_sim.assert_not_called()

    def test_manual_waits_for_throttle_readback_and_reports_persistent_override(self):
        import threading
        cfg=dict(alt_ft=4500,ias_kt=95,lat=31.5,lon=73.25,flaps=0,throttle=0,temperature_c=25)
        good=dict(pressure_alt=4500,plane_latitude=31.5,plane_longitude=73.25,measured_flaps_handle_index=0,engine_throttle_percent=0)
        stale=dict(good,engine_throttle_percent=5)
        sim=Mock();sim.is_connected.return_value=True;sim.wait_for_paused.return_value=True
        window=SimpleNamespace(p3d=sim,stop_worker_event=threading.Event(),_wait_for_initial_altitude=Mock(),_refresh_paused_telemetry=Mock(side_effect=[stale,good]))
        engine.load_base()
        with patch('replay_support.reference_weather_metar',return_value=None):
            self.assertEqual(engine.apply_manual_conditions(window,cfg),good)
        sim.set_initial_throttle.assert_called_once_with(0.)
        sim.unpause_sim.assert_not_called()
        window._refresh_paused_telemetry=Mock(return_value=stale)
        with patch('replay_support.reference_weather_metar',return_value=None),patch.object(engine.time,'monotonic',side_effect=[0,4]):
            with self.assertRaisesRegex(RuntimeError,'requested 0, actual 5'):
                engine.apply_manual_conditions(window,cfg)

    def test_manual_entry_uses_custom_settings_and_never_standalone_defaults(self):
        import threading
        base=engine.load_base()
        cfg=dict(alt_ft=5000,ias_kt=95,lat=31.5,lon=73.25,flaps=1,throttle=65,temperature_c=25)
        window=SimpleNamespace(manual_initial_conditions=cfg,stop_worker_event=threading.Event(),
            _update_stage_ui=Mock(),sig_update_stage=Mock(),brunner=Mock(),p3d=Mock())
        with patch.object(engine,'apply_manual_conditions',return_value={'pressure_alt':5000}) as apply:
            base.SuperMushakLoggerApp.start_manual_test(window)
            window.preparation_thread.join(timeout=2)
        apply.assert_called_once_with(window,cfg)
        window.p3d.set_position_airborne.assert_not_called()
        self.assertIn('5000',window.sig_update_stage.emit.call_args.args[1])

    def test_live_altitude_displays_position_even_when_paused_instrument_is_stale(self):
        base=engine.load_base()
        window=SimpleNamespace(p3d=Mock(),is_recording=False,current_stage=base.STAGE_INITIALIZING,
            brunner=None,lbl_throttle_val=Mock(),throttle_bar=Mock(),lbl_engine_rpm=Mock(),
            lbl_ias_alt=Mock(),lbl_pitch_bank=Mock())
        window.p3d.get_telemetry.return_value={'pressure_alt':5000,'indicated_altitude':2855,'airspeed_indicated':80}
        base.SuperMushakLoggerApp._on_ui_timer_tick(window)
        text=window.lbl_ias_alt.setText.call_args.args[0]
        self.assertIn('5000',text)
        self.assertIn('MSL',text)
        self.assertNotIn('2855',text)

    def test_manual_reuses_prepared_conditions_but_reapplies_changes_and_explicit_set(self):
        cfg=dict(alt_ft=5000,ias_kt=95,lat=31.5,lon=73.25,flaps=1,throttle=65,temperature_c=25)
        for changed,force,ready,connected,expected in [(False,False,True,True,0),(True,False,True,True,1),
                (False,True,True,True,1),(False,False,False,True,1),(False,False,True,False,1)]:
            with self.subTest(changed=changed,force=force,ready=ready,connected=connected),tempfile.TemporaryDirectory() as folder:
                context=dict(test_reference='1.c.(1)',title='Normal climb',mode='manual',results_root=folder)
                base=SimpleNamespace(SuperMushakLoggerApp=object,STAGE_CONNECTED_READY='Ready')
                cls=engine.worker_class(base,context,lambda value:None)
                window=object.__new__(cls)
                window.is_recording=False;window._prepared=ready;window.current_stage='Ready'
                window.manual_initial_conditions=engine.validate_conditions(cfg)
                window.p3d=Mock(is_connected=lambda:connected);window.brunner=SimpleNamespace(sock=object())
                window._publish=Mock()
                command=dict(action='initialize',mode='manual',initial_conditions=dict(cfg,alt_ft=5500) if changed else cfg,
                    force_initialize=force,selected_graphs=['pitch_force'],duration_s=90)
                with patch.object(engine,'initialize_mode') as initialize:
                    window.handle_command(command)
                self.assertEqual(initialize.call_count,expected)
                self.assertEqual(window.target_duration_sec,90)
                self.assertEqual(window.report_graphs,['pitch_force'])
                if not expected:
                    window._publish.assert_called_once()
                    self.assertIn('not repositioned',window._stage_details)

    def test_launch_and_reinitialize_preserve_settings_and_graph_choices(self):
        cfg=dict(alt_ft=4500,ias_kt=95,lat=31.5,lon=73.25,flaps=1,throttle=65,temperature_c=25)
        with tempfile.TemporaryDirectory() as folder:
            spec=SimpleNamespace(reference='1.c.(1)',title='Normal climb')
            with patch.object(engine.subprocess,'Popen'):
                _,context=engine.launch('manual',spec,Path(folder)/'sessions',initial_conditions=cfg,selected_graphs=['pitch_force'])
            self.assertEqual(context['initial_conditions']['alt_ft'],4500)
            self.assertEqual(context['selected_graphs'],['pitch_force'])
            cls=engine.worker_class(SimpleNamespace(SuperMushakLoggerApp=object),context,lambda value:None)
            window=object.__new__(cls)
            window.is_recording=False;window.p3d=Mock(is_connected=lambda:True);window.brunner=SimpleNamespace(sock=object());window.start_manual_test=Mock()
            window.handle_command(dict(action='initialize',mode='manual',initial_conditions=dict(cfg,alt_ft=5000),selected_graphs=[]))
            self.assertEqual(window.manual_initial_conditions['alt_ft'],5000)
            self.assertEqual(window.report_graphs,[])
            window.start_manual_test.assert_called_once()

    def reference(self, root):
        path=Path(root)/'manual.csv'
        path.write_text('Time (s),Pitch_Manual_Pos,Roll_Manual_Pos,Rudder_Manual_Pos,Throttle 1 Position (%)\n0,0,0,0,0\n1,0.1,0.2,0.3,80\n')
        return path

    def test_manual_calls_original_manual_entry_once(self):
        window=SimpleNamespace(p3d=Mock(),brunner=SimpleNamespace(sock=object()),start_manual_test=Mock(),start_auto_test_only=Mock())
        window.p3d.is_connected.return_value=True
        engine.initialize_mode(window,'manual')
        window.start_manual_test.assert_called_once_with()
        window.start_auto_test_only.assert_not_called()

    def test_automatic_calls_original_auto_entry_with_selected_reference(self):
        with tempfile.TemporaryDirectory() as folder:
            ref=self.reference(folder)
            window=SimpleNamespace(p3d=Mock(),brunner=SimpleNamespace(sock=object()),start_manual_test=Mock(),start_auto_test_only=Mock())
            window.p3d.is_connected.return_value=True
            engine.initialize_mode(window,'auto',ref)
            self.assertEqual(window.selected_ref_csv,str(ref.resolve()))
            window.start_auto_test_only.assert_called_once_with()
            window.start_manual_test.assert_not_called()

    def test_engine_tests_call_updated_brunner_initialization_directly(self):
        for reference in ('1.f.(1)','1.f.(2)'):
            for mode in ('manual','auto'):
                manual=Mock();manual.__name__='start_manual_test'
                auto=Mock();auto.__name__='start_auto_test_only'
                base=SimpleNamespace(__file__=str(engine.brunner_root()/'Super_Mushak_Logger_GUI.py'),
                    SuperMushakLoggerApp=SimpleNamespace(start_manual_test=manual,start_auto_test_only=auto))
                window=SimpleNamespace(p3d=Mock(is_connected=lambda:True),brunner=SimpleNamespace(sock=object()),
                    _session_context={'test_reference':reference},_base_module=base,_control_method='replay',
                    _commit_initial_position=Mock(),_refresh_restored_airdata=Mock(),
                    start_manual_test=Mock(),start_auto_test_only=Mock())
                with patch.object(engine,'validate_reference',return_value='selected_manual.csv'),patch.object(engine,'apply_manual_conditions') as extra:
                    engine.initialize_mode(window,mode,'selected_manual.csv')
                (manual if mode=='manual' else auto).assert_called_once_with(window)
                (auto if mode=='manual' else manual).assert_not_called()
                window.start_manual_test.assert_not_called()
                window.start_auto_test_only.assert_not_called()
                extra.assert_not_called()
                self.assertEqual(window._session_context['initialization_source']['codebase'],'brunner')
                if mode=='auto':self.assertEqual(window.selected_ref_csv,'selected_manual.csv')

    def test_missing_connections_prevent_initialization(self):
        for p3d,sock in [(None,object()),(Mock(is_connected=lambda:False),object()),(Mock(is_connected=lambda:True),None)]:
            window=SimpleNamespace(p3d=p3d,brunner=SimpleNamespace(sock=sock),start_manual_test=Mock())
            with self.assertRaises(RuntimeError):engine.initialize_mode(window,'manual')
            window.start_manual_test.assert_not_called()

    def test_bad_automatic_reference_never_starts_worker(self):
        with tempfile.TemporaryDirectory() as folder:
            invalid=Path(folder)/'invalid.csv';invalid.write_text('Time (s),Airspeed\n0,80\n')
            spec=SimpleNamespace(reference='1.c.(1)',title='Normal climb')
            with patch.object(engine.subprocess,'Popen') as spawn:
                with self.assertRaises(ValueError):engine.launch('auto',spec,folder,invalid)
                spawn.assert_not_called()

    def test_all_tests_share_same_process_entry_in_both_modes(self):
        import main
        with tempfile.TemporaryDirectory() as folder:
            reference=self.reference(folder)
            with patch.object(engine.subprocess,'Popen') as spawn:
                for spec in main.TESTS:
                    for mode in ['manual','auto']:
                        _,context=engine.launch(mode,spec,Path(folder)/'sessions',reference if mode=='auto' else None)
                        self.assertEqual(context['test_reference'],spec.reference)
                        self.assertEqual(context['mode'],mode)
                        args=spawn.call_args.args[0]
                        self.assertIn('--brunner-test',args)
                        self.assertNotIn('--engine-suite',args)
                self.assertEqual(spawn.call_count,60)

    def test_frozen_launch_uses_bundled_exe(self):
        with tempfile.TemporaryDirectory() as folder:
            spec=SimpleNamespace(reference='2.a.(1)',title='Pitch force')
            with patch.object(engine,'brunner_root'),patch.object(engine.sys,'frozen',True,create=True),patch.object(engine.subprocess,'Popen') as spawn:
                engine.launch('manual',spec,folder)
                args=spawn.call_args.args[0]
                self.assertEqual(args[0],sys.executable)
                self.assertEqual(args[1],'--brunner-test')
                self.assertEqual(len(args),3)
                self.assertEqual(spawn.call_args.kwargs['env']['PYINSTALLER_RESET_ENVIRONMENT'],'1')

    def test_background_process_has_no_window_and_offscreen_qt(self):
        with tempfile.TemporaryDirectory() as folder:
            spec=SimpleNamespace(reference='1.c.(1)',title='Climb')
            with patch.object(engine.subprocess,'Popen') as spawn:
                engine.launch('manual',spec,folder)
                self.assertEqual(spawn.call_args.kwargs['env']['QT_QPA_PLATFORM'],'offscreen')
                self.assertIn('creationflags',spawn.call_args.kwargs)

    def test_command_queue_is_atomic_and_ordered(self):
        with tempfile.TemporaryDirectory() as folder:
            context={'command_directory':str(Path(folder)/'commands')}
            engine.send_command(context,'record')
            engine.send_command(context,'stop')
            files=sorted((Path(folder)/'commands').glob('*.json'))
            self.assertEqual([json.loads(p.read_text())['action'] for p in files],['record','stop'])
            self.assertFalse(list((Path(folder)/'commands').glob('*.tmp')))
            with self.assertRaises(ValueError):engine.send_command(context,'invalid')

    def test_invalid_duration_rejected_before_start(self):
        with tempfile.TemporaryDirectory() as folder:
            spec=SimpleNamespace(reference='1.c.(1)',title='Climb')
            with patch.object(engine.subprocess,'Popen') as spawn:
                for duration in [0,3601,float('nan'),1.5]:
                    with self.assertRaises(ValueError):engine.launch('manual',spec,folder,duration_s=duration)
                spawn.assert_not_called()

if __name__=='__main__':unittest.main()
