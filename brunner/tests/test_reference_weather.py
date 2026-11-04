import json
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock,patch
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'src'))
from replay_support import reference_weather_metar
import Super_Mushak_Logger_GUI as gui

SNAPSHOT={'altitude_msl':2852.668584046899,'telemetry':{'ambient_wind_spd':7.766309191907695,
    'ambient_wind_dir':324.9589538574219,'ambient_temp':10.87021255493164,'pressure_alt':2852.668584046899,
    'baro_pressure':1013.1}}
CALM='GLOB&A0 000000Z 00000KT&D985NG 27020KT&A2231NG 9999 BKN020 15/05 Q1007 @@@ 73 15 270 20 |'

class ReferenceWeatherTests(unittest.TestCase):
    def test_reported_reference_restores_wind_and_temperature(self):
        metar=reference_weather_metar(SNAPSHOT,CALM)
        self.assertIn('32508KT&D9999NG',metar)
        self.assertIn('10.87/05&A00869',metar)
        self.assertIn('Q1007',metar)
        self.assertIn('BKN020',metar)
        self.assertNotIn('27020KT',metar)
        self.assertNotIn('@@@',metar)
        self.assertTrue(metar.startswith('GLOB '))

    def test_calm_reference_is_applied_and_legacy_unknown_is_skipped(self):
        self.assertIsNone(reference_weather_metar({},None))
        self.assertIn('00000KT',reference_weather_metar({'wind_spd':0},CALM))

    def test_unavailable_or_invalid_weather_is_rejected(self):
        with self.assertRaises(ValueError):reference_weather_metar(SNAPSHOT,None)
        with self.assertRaises(ValueError):reference_weather_metar({'wind_spd':8},CALM)
        with self.assertRaises(ValueError):reference_weather_metar({'wind_spd':float('nan')},CALM)

    def test_restore_requires_pause_and_applies_metar(self):
        sim=Mock();sim.is_connected.return_value=True;sim.wait_for_paused.return_value=True
        sim.get_global_weather_metar.return_value=CALM;sim.set_global_weather_metar.return_value=True
        fake=SimpleNamespace(p3d=sim,ref_manual_snapshot=SNAPSHOT,stop_worker_event=threading.Event())
        gui.SuperMushakLoggerApp._restore_reference_weather(fake)
        sim.wait_for_paused.assert_called_once_with(3.)
        self.assertIn('32508KT',sim.set_global_weather_metar.call_args.args[0])
        sim.wait_for_paused.return_value=False
        sim.set_global_weather_metar.reset_mock()
        with self.assertRaisesRegex(RuntimeError,'confirm pause'):gui.SuperMushakLoggerApp._restore_reference_weather(fake)
        sim.set_global_weather_metar.assert_not_called()

    def test_rejected_weather_does_not_continue(self):
        sim=Mock();sim.get_global_weather_metar.return_value=CALM;sim.set_global_weather_metar.return_value=False
        fake=SimpleNamespace(p3d=sim,ref_manual_snapshot=SNAPSHOT,stop_worker_event=threading.Event())
        with self.assertRaisesRegex(RuntimeError,'rejected'):gui.SuperMushakLoggerApp._restore_reference_weather(fake)

    def test_weather_is_restored_before_power_and_control_movement(self):
        source=(ROOT/'Super_Mushak_Logger_GUI.py').read_text(encoding='utf-8')
        body=source.split('def start_auto_test_only',1)[1].split('def arm_recording',1)[0]
        self.assertLess(body.index('_restore_reference_weather(self)'),body.index('self._prepare_reference_power()'))
        self.assertLess(body.index('_restore_reference_weather(self)'),body.index('self.brunner.configure_replay_mode()'))

    def test_reported_calm_wind_failure_recovers_after_reference_weather_restore(self):
        reference={'altitude_msl':2852.668584046899,'telemetry':dict(SNAPSHOT['telemetry'],
            aircraft_wind_x_kt=7.6487,aircraft_wind_y_kt=0.,aircraft_wind_z_kt=0.,indicated_altitude=2852.6123)}
        live={'aircraft_wind_x_kt':0.,'aircraft_wind_y_kt':0.,'aircraft_wind_z_kt':0.,
              'ambient_wind_spd':0.,'indicated_altitude':2852.6123,'pressure_alt':2852.668584046899}
        sim=Mock();sim.get_global_weather_metar.return_value=CALM
        sim.wait_for_fresh_telemetry.side_effect=lambda *args:dict(live)
        def restore(metar):
            live.update(aircraft_wind_x_kt=8.,ambient_wind_spd=8.)
            return True
        sim.set_global_weather_metar.side_effect=restore
        fake=SimpleNamespace(p3d=sim,ref_manual_snapshot=reference,stop_worker_event=threading.Event(),_refresh_paused_telemetry=Mock())
        gui.SuperMushakLoggerApp._restore_reference_weather(fake)
        with patch.object(gui.time,'sleep'),patch.object(gui.time,'monotonic',side_effect=[0.,2.,2.3,2.6]):
            gui.SuperMushakLoggerApp._commit_initial_position(fake)
        self.assertEqual(sim.request_telemetry_once.call_count,3)
        fake._refresh_paused_telemetry.assert_called_once()
        sim.pause_sim.assert_called_once()

    def test_transient_reference_does_not_accept_current_calm_weather(self):
        sim=Mock();sim.get_telemetry_sequence.return_value=1
        sim.wait_for_fresh_telemetry.return_value={'aircraft_wind_x_kt':0.,'aircraft_wind_y_kt':0.,'aircraft_wind_z_kt':0.,'ambient_wind_spd':0.}
        fake=SimpleNamespace(p3d=sim,ref_manual_snapshot={'telemetry':{'aircraft_wind_x_kt':0.,'aircraft_wind_y_kt':0.,'aircraft_wind_z_kt':0.,'ambient_wind_spd':8.}},stop_worker_event=threading.Event(),_refresh_paused_telemetry=Mock())
        with patch.object(gui.time,'sleep'),patch.object(gui.time,'monotonic',side_effect=[0.,2.,3.,60.]):
            with self.assertRaisesRegex(RuntimeError,'wind 0.00'):gui.SuperMushakLoggerApp._commit_initial_position(fake)
        sim.pause_sim.assert_called_once()

if __name__=='__main__':unittest.main()
