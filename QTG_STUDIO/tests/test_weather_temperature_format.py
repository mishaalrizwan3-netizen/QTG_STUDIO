"""Canonical METAR formatting only; no simulator or reference-data writes."""
import copy
import importlib.util
from pathlib import Path
import unittest

path=Path(__file__).resolve().parents[1]/'interfaces/brunner_base/src/replay_support.py'
spec=importlib.util.spec_from_file_location('weather_format_replay_support',path)
replay=importlib.util.module_from_spec(spec)
spec.loader.exec_module(replay)


class WeatherTemperatureFormatTests(unittest.TestCase):
    def metar(self,temperature,dewpoint='12',altitude=749.4888437517968):
        reference={'telemetry':{'ambient_temp':temperature,'pressure_alt':altitude,
                               'ambient_wind_spd':5.9841329521,'ambient_wind_dir':325.}}
        before=copy.deepcopy(reference)
        result=replay.reference_weather_metar(reference,
            f'GLOB 000000Z 32506KT&D9999NG 64KM&B-213&D3048 2CU065&CU000FMMN000N 15/{dewpoint}&A00228 Q1013')
        self.assertEqual(reference,before)
        return result

    def test_fractional_reference_temperature(self):
        self.assertIn('15/12&A00228',self.metar(15.0129))
        self.assertIn('16/12&A00610',self.metar(16.467,altitude=2000.))

    def test_whole_degrees_and_zero_padding(self):
        for temperature,token in ((15.,'15'),(0.,'00'),(5.,'05'),(-5.,'-05'),(-15.,'-15')):
            with self.subTest(temperature=temperature):
                self.assertIn(f' {token}/',self.metar(temperature))

    def test_negative_fractional_values_and_no_negative_zero(self):
        self.assertIn(' -05/-12&A00228',self.metar(-5.2,dewpoint='M12.3'))
        self.assertIn(' -06/-12&A00228',self.metar(-5.8,dewpoint='-12.3'))
        self.assertIn(' 00/-01&A00228',self.metar(-.2,dewpoint='-1'))

    def test_dewpoint_rounding_and_existing_temperature_cap(self):
        self.assertIn('15/12&A00228',self.metar(15.0129,dewpoint='12.4'))
        self.assertIn('15/13&A00228',self.metar(15.0129,dewpoint='12.6'))
        self.assertIn('05/05&A00228',self.metar(5.2,dewpoint='12.6'))

    def test_non_temperature_sections_identical(self):
        self.assertEqual(self.metar(15.0129),
            'GLOB 000000Z 32506KT&D9999NG 64KM&B-213&D3048 2CU065&CU000FMMN000N 15/12&A00228 Q1013')


if __name__=='__main__': unittest.main()
