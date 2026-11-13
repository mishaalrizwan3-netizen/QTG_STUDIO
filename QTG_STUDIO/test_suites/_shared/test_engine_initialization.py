import importlib.util
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

spec=importlib.util.spec_from_file_location('engine_adapter',Path(__file__).with_name('engine_initialization.py'))
adapter=importlib.util.module_from_spec(spec);spec.loader.exec_module(adapter)

class EngineAdapterTests(unittest.TestCase):
    def make_csv(self,folder,changing_baro=False,stable=False):
        path=Path(folder)/'manual.csv'
        path.write_text('Time (s),Altitude MSL (ft),Indicated Altitude (ft),Baro Press (hPa)\n'
            +'0,2000,1985,1013\n'+''.join(f'{t},2000,{1985 if stable else 2001},{1014 if changing_baro else 1013}\n' for t in (.5,.7,1.,1.5,2.)))
        return path
    def test_stale_initial_instrument_uses_settled_same_baro_samples(self):
        with tempfile.TemporaryDirectory() as folder:
            result=adapter.settled_altimeter(self.make_csv(folder))
            self.assertEqual(result['initial_offset_ft'],-15)
            self.assertEqual(result['settled_offset_ft'],1)
    def test_real_offset_and_changed_barometer_are_preserved(self):
        with tempfile.TemporaryDirectory() as folder:
            self.assertIsNone(adapter.settled_altimeter(self.make_csv(folder,stable=True)))
            self.assertIsNone(adapter.settled_altimeter(self.make_csv(folder,changing_baro=True)))
    def test_refresh_restores_original_snapshot_even_on_failure(self):
        with tempfile.TemporaryDirectory() as folder:
            original={'telemetry':{'pressure_alt':2000.,'indicated_altitude':1985.}}
            host=SimpleNamespace(selected_ref_csv=self.make_csv(folder),ref_manual_snapshot=original,_session_context={})
            host._commit_initial_position=Mock()
            def refresh():
                self.assertEqual(host.ref_manual_snapshot['telemetry']['indicated_altitude'],2001.)
                raise RuntimeError('cancelled')
            host._refresh_restored_airdata=refresh
            adapter.install(host)
            host._commit_initial_position()
            self.assertIs(host.ref_manual_snapshot,original)
            with self.assertRaisesRegex(RuntimeError,'cancelled'):host._refresh_restored_airdata()
            self.assertIs(host.ref_manual_snapshot,original)
            self.assertEqual(original['telemetry']['indicated_altitude'],1985.)
    def test_manual_uses_test_settings_instead_of_standalone_defaults(self):
        import threading
        for settings in ({},{'alt_ft':2500.,'ias_kt':90.,'throttle':12.,'flaps':1.}):
            cfg=dict(alt_ft=2000.,ias_kt=80.,throttle=5.,flaps=0.);cfg.update(settings)
            sim=Mock();sim.wait_for_paused.return_value=True
            host=SimpleNamespace(p3d=sim,brunner=Mock(),stop_worker_event=threading.Event(),
                _session_context={'initial_conditions':settings},_base_module=SimpleNamespace(
                    STAGE_INITIALIZING='Initializing',STAGE_CONNECTED_READY='Ready',STAGE_CONNECTION_ERROR='Failed'),
                _update_stage_ui=Mock(),_wait_for_initial_altitude=Mock(),sig_update_stage=Mock(),
                _refresh_paused_telemetry=Mock(return_value=dict(pressure_alt=cfg['alt_ft'],airspeed_indicated=cfg['ias_kt'],
                    engine_throttle_percent=cfg['throttle'],measured_flaps_handle_index=cfg['flaps'])))
            policy=SimpleNamespace(DEFAULT_INITIAL_CONDITIONS=dict(alt_ft=2000.,ias_kt=80.,throttle=5.,flaps=0.),DURATION_SECONDS=21)
            entry=Mock();adapter.run_initialization(host,'manual',entry,policy)
            host.preparation_thread.join(2)
            self.assertFalse(host.preparation_thread.is_alive())
            self.assertEqual(sim.set_position_airborne.call_args.kwargs['alt_ft'],cfg['alt_ft'])
            sim.set_initial_throttle.assert_called_once_with(cfg['throttle'])
            sim.set_flaps_handle_index.assert_called_once_with(cfg['flaps'])
            self.assertEqual(host.target_duration_sec,21)
            self.assertEqual(host.sig_update_stage.emit.call_args.args[0],'Ready')
            entry.assert_not_called()

    def test_incompatible_reference_is_rejected_before_base_initialization(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'manual.csv'
            path.write_text('Altitude MSL (ft),Indicated Airspeed (kts),Throttle 1 Position (%)\n3000,80,81\n')
            host=SimpleNamespace(selected_ref_csv=path,_session_context={})
            policy=SimpleNamespace(DEFAULT_INITIAL_CONDITIONS=dict(alt_ft=2000.,ias_kt=80.,throttle=5.,flaps=0.),DURATION_SECONDS=21)
            entry=Mock()
            with self.assertRaisesRegex(ValueError,'does not match'):adapter.run_initialization(host,'auto',entry,policy)
            entry.assert_not_called()
            path.write_text('Altitude MSL (ft),Indicated Airspeed (kts),Throttle 1 Position (%)\n2000,80,5\n')
            adapter.run_initialization(host,'auto',entry,policy)
            entry.assert_called_once_with(host)

if __name__=='__main__':unittest.main()
