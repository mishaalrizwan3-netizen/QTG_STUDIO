"""Offline SDK compilation and actual callback tests; no simulator connection."""
import os
import json
import csv
from pathlib import Path
import re
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
SDK = Path('C:/Program Files/Lockheed Martin/Prepar3D v4 SDK 4.5.14.34698')
VCVARS = Path('C:/Program Files/Microsoft Visual Studio/2022/Community/VC/Auxiliary/Build/vcvars64.bat')


class ParkingBrakeDiagnosticTests(unittest.TestCase):
    def test_native_callback_validation_and_logging(self):
        if os.name != 'nt' or not VCVARS.is_file() or not (SDK/'inc/SimConnect/SimConnect.h').is_file():
            self.skipTest('MSVC x64/P3D SDK unavailable')
        with tempfile.TemporaryDirectory() as temp:
            folder = Path(temp)
            script = folder/'compile.cmd'
            script.write_text(f'''@echo off
call "{VCVARS}" >nul
if errorlevel 1 exit /b 1
cl /nologo /MD /EHsc /std:c++17 /I"{SDK / 'inc/SimConnect'}" "{ROOT / 'tests/native_parking_brake.cpp'}" /Fe:mock.exe /link /LIBPATH:"{SDK / 'lib/SimConnect'}" SimConnect.lib user32.lib advapi32.lib ole32.lib shell32.lib ws2_32.lib
if errorlevel 1 exit /b 1
mock.exe "{folder}"
exit /b %ERRORLEVEL%
''', encoding='utf-8')
            result = subprocess.run(['cmd','/c',str(script)],cwd=folder,capture_output=True,text=True,timeout=90)
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)
            text = (folder/'mock.csv').read_text()
            self.assertIn('BRAKE PARKING POSITION,bool,0,NEW_CORRELATED_RESPONSE',text)
            self.assertIn('BRAKE PARKING POSITION,bool,1,NEW_CORRELATED_RESPONSE',text)
            self.assertIn('BRAKE LEFT POSITION,position,0.5,NEW_CORRELATED_RESPONSE',text)
            self.assertIn('BRAKE RIGHT POSITION,position,0.75,NEW_CORRELATED_RESPONSE',text)
            self.assertIn('LATE_REJECTED',text)
            self.assertIn('INVALID',text)
            events = [json.loads(line) for line in (folder/'events.jsonl').read_text().splitlines()]
            matched = [event for event in events if event['event']=='MATCHED_DATA']
            self.assertEqual(matched[0]['returned_object_id'],1)
            reasons = {event['reason'] for event in events if event['event']=='REJECTED_DATA'}
            self.assertTrue({'user_object_changed','incomplete_double_payload','unexpected_datum_count',
                'unexpected_request_flags','no_outstanding_request_duplicate_or_cached',
                'request_id_mismatch_or_superseded'} <= reasons)
            error = next(event for event in map(json.loads,(folder/'exception.jsonl').read_text().splitlines())
                         if event['event']=='SDK_EXCEPTION')
            self.assertEqual((error['send_id'],error['request_id'],error['definition_id'],error['index']),(77,101,1,1))
            self.assertEqual(error['operation'],'RequestDataOnSimObject')
            self.assertGreater(error['exception_code'],0)
            with (folder/'pair.csv').open() as stream:
                pair=list(csv.DictReader(stream))
            self.assertEqual([r['simvar'] for r in pair],['BRAKE PARKING POSITION','SIM TIME']*2)
            for a,b in zip(pair[::2],pair[1::2]):
                self.assertEqual(a['receipt_qpc_ticks'],b['receipt_qpc_ticks'])
                self.assertEqual(a['request_id'],b['request_id'])

    def test_sdk_call_surface_is_read_only(self):
        source = (ROOT/'tools/parking_brake_diagnostic.cpp').read_text()
        self.assertEqual(set(re.findall(r'\b(SimConnect_\w+)\(',source)),{
            'SimConnect_Open','SimConnect_Close','SimConnect_AddToDataDefinition',
            'SimConnect_SubscribeToSystemEvent','SimConnect_CallDispatch','SimConnect_RequestDataOnSimObject',
            'SimConnect_GetLastSentPacketID'})

    def test_no_legacy_channels_or_worker_connection_used(self):
        source = (ROOT/'tools/parking_brake_diagnostic.cpp').read_text()
        self.assertNotIn('LoadLibrary',source)
        self.assertNotIn('GetTelemetry',source)
        self.assertNotIn('SIM ON GROUND',source)
        self.assertNotIn('BRUNNER',source.replace('// Close only this observer\'s handle; no worker, brake, pause or BRUNNER command.',''))

    def test_explicit_opt_in_and_bounded_session(self):
        source = (ROOT/'tools/parking_brake_diagnostic.cpp').read_text()
        self.assertLess(source.index('std::string(argv[1]) != "--connect-read-only"'),source.index('SimConnect_Open('))
        for required in ('elapsed >= 15.','duration > 600','elapsed >= duration','Parking-brake responses unavailable',
                         'QTG_UNIVERSAL_ENGINE_ACCELERATION','QTG_UNIVERSAL_ENGINE_DECELERATION'):
            self.assertIn(required,source)


if __name__ == '__main__': unittest.main()
