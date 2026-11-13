import ast
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import threading
import unittest
from unittest.mock import Mock, patch

from backend.weather_verification import CHANNELS, compare_weather, observe


ROOT=Path(__file__).resolve().parents[1]


class WeatherVerificationTests(unittest.TestCase):
    def setUp(self):
        self.reference={key:0. for key in CHANNELS}
        self.reference.update(ambient_temp=15.,ambient_pressure_mb=986.,baro_pressure=1013.2,
                              ambient_density_slug_ft3=.0023,ambient_wind_spd=8.,ambient_wind_dir=325.)
        self.window=SimpleNamespace(_session_context={'test_reference':'1.f.(2)'},
            ref_manual_snapshot={'telemetry':self.reference},selected_ref_csv='selected.csv',
            p3d=Mock(),stop_worker_event=threading.Event())

    def sample(self,sequence=1,temperature=15.):
        return dict(self.reference,bridge_telemetry_sequence=sequence,last_updated=100.,ambient_temp=temperature)

    def observe(self,phase,sample,**evidence):
        with patch('backend.weather_verification.time.time',return_value=100.1), patch('builtins.print'):
            observe(self.window,phase,sample,**evidence)
        return self.window._weather_observations[-1]

    def test_correct_measured_weather_and_wrong_temperature(self):
        self.assertEqual(compare_weather(self.reference,self.reference,timing_supported=True)['status'],'MATCHED')
        result=compare_weather(self.reference,dict(self.reference,ambient_temp=39.7),timing_supported=True)
        self.assertEqual(result['status'],'NOT MATCHED')
        self.assertAlmostEqual(result['channels']['ambient_temp']['difference'],24.7)

    def test_delayed_update_and_later_override_remain_visible(self):
        self.observe('before_restoration',self.sample(1,39.7))
        immediate=self.observe('after_weather_command',self.sample(1,39.7),command_accepted=True)
        self.assertEqual(immediate['status'],'UNVERIFIED')
        self.assertTrue(immediate['command_evidence']['command_accepted'])
        self.assertEqual(self.observe('after_resume_airflow_recovery',self.sample(2))['status'],'MATCHED')
        self.assertEqual(self.observe('final_initialization_readiness',self.sample(3,39.7))['status'],'NOT MATCHED')
        self.assertEqual(len(self.window._session_context['weather_diagnostics']),4)

    def test_missing_stale_and_nonfinite_readings(self):
        result=compare_weather(self.reference,{},timing_supported=True)
        self.assertEqual(result['status'],'UNVERIFIED')
        sample=self.sample();sample['last_updated']=90.
        self.assertEqual(self.observe('before_restoration',sample)['status'],'UNVERIFIED')
        sample=self.sample();sample['ambient_temp']=float('nan')
        self.assertEqual(self.observe('before_restoration',sample)['channels']['ambient_temp']['status'],'UNVERIFIED')

    def test_no_silent_pressure_temperature_or_density_conversion(self):
        result=compare_weather(self.reference,dict(self.reference,ambient_pressure_mb=98600.,
            ambient_temp=59.,ambient_density_slug_ft3=1.2),timing_supported=True)
        for key in ('ambient_pressure_mb','ambient_temp','ambient_density_slug_ft3'):
            self.assertEqual(result['channels'][key]['status'],'NOT MATCHED')
        self.assertEqual(result['channels']['ambient_pressure_mb']['measured'],98600.)
        self.assertNotEqual(result['channels']['baro_pressure']['unit'],result['channels']['ambient_pressure_mb']['unit'])

    def test_existing_command_builder_altitude_conversion_and_pressure_preservation(self):
        spec=importlib.util.spec_from_file_location('weather_replay_support',ROOT/'interfaces/brunner_base/src/replay_support.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        snapshot={'telemetry':dict(self.reference,pressure_alt=2000.)}
        metar=module.reference_weather_metar(snapshot,'GLOB 000000Z 32508KT 15/12&A00000 Q1013')
        self.assertIn('15/12&A00610',metar)
        self.assertIn('Q1013',metar)
        self.assertIn('32508KT',metar)

    def test_weather_hook_does_not_add_setters_or_gate_failed_comparison(self):
        path=ROOT/'interfaces/brunner_base/Super_Mushak_Logger_GUI.py'
        tree=ast.parse(path.read_text(encoding='utf-8'))
        helper=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='_observe_weather')
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='SuperMushakLoggerApp')
        method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='_restore_reference_weather')
        builder=Mock(return_value='unchanged commanded METAR')
        scope={'reference_weather_metar':builder}
        exec(compile(ast.Module(body=[helper,method],type_ignores=[]),str(path),'exec'),scope)
        self.window.p3d.get_telemetry.side_effect=[self.sample(1,39.7),self.sample(2,39.7)]
        with patch('backend.weather_verification.time.time',return_value=100.1),patch('builtins.print'):
            scope['_restore_reference_weather'](self.window)
        self.window.p3d.set_global_weather_metar.assert_called_once_with('unchanged commanded METAR')
        self.window.p3d.unpause_sim.assert_not_called()
        self.window.p3d.queue_position_airborne.assert_not_called()
        self.assertEqual(self.window._weather_observations[-1]['status'],'NOT MATCHED')


if __name__=='__main__': unittest.main()
