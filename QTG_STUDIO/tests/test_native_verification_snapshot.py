"""Build/run mock-only native harness; never load the simulator DLL."""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest


class NativeSnapshotTests(unittest.TestCase):
    def test_production_bridge_compiles_against_installed_sdk(self):
        vcvars = Path('C:/Program Files/Microsoft Visual Studio/2022/Community/VC/Auxiliary/Build/vcvars64.bat')
        sdk = Path('C:/Program Files/Lockheed Martin/Prepar3D v4 SDK 4.5.14.34698/inc/SimConnect')
        if os.name != 'nt' or not vcvars.is_file() or not (sdk/'SimConnect.h').is_file():
            self.skipTest('MSVC/P3D SDK unavailable')
        source = Path(__file__).resolve().parents[1]/'interfaces/brunner_base/native/src/p3d_bridge_v15_full_causal_state.cpp'
        with tempfile.TemporaryDirectory() as folder:
            command = Path(folder)/'check.cmd'
            command.write_text(f'@echo off\ncall "{vcvars}" >nul\ncl /nologo /Zs /EHsc /std:c++17 /I"{sdk}" "{source}"\n', encoding='utf-8')
            result = subprocess.run(['cmd', '/c', str(command)], cwd=folder,
                                    capture_output=True, text=True, timeout=90)
            self.assertEqual(result.returncode, 0, result.stdout+result.stderr)

    def test_native_callback_correlation_and_atomic_copy(self):
        vcvars = Path('C:/Program Files/Microsoft Visual Studio/2022/Community/VC/Auxiliary/Build/vcvars64.bat')
        if os.name != 'nt' or not vcvars.is_file():
            self.skipTest('MSVC x64 tools unavailable')
        source = Path(__file__).with_name('native_verification_snapshot.cpp').resolve()
        with tempfile.TemporaryDirectory() as folder:
            command = Path(folder)/'build.cmd'
            command.write_text(f'@echo off\ncall "{vcvars}" >nul\ncl /nologo /EHsc /std:c++17 "{source}" /Fe:harness.exe\nif errorlevel 1 exit /b 1\nharness.exe\n', encoding='utf-8')
            result = subprocess.run(['cmd', '/c', str(command)], cwd=folder,
                                    capture_output=True, text=True, timeout=90)
            self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
