"""Build a separate read-only SDK diagnostic; never deploy/load the worker DLL."""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess
from uuid import uuid4

ROOT = Path(__file__).resolve().parents[1]
SDK = Path('C:/Program Files/Lockheed Martin/Prepar3D v4 SDK 4.5.14.34698')
VCVARS = Path('C:/Program Files/Microsoft Visual Studio/2022/Community/VC/Auxiliary/Build/vcvars64.bat')


def build():
    installed = ROOT/'interfaces/brunner_base/native/bin/p3d_bridge_engine_init_v1517.dll'
    before = hashlib.sha256(installed.read_bytes()).hexdigest()
    folder = ROOT/'build'/('stage5h_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_'+uuid4().hex[:8])
    folder.mkdir(parents=True, exist_ok=False)
    source = ROOT/'tools/parking_brake_diagnostic.cpp'
    executable = folder/'parking_brake_diagnostic.exe'
    command = folder/'build.cmd'
    command.write_text(f'''@echo off
call "{VCVARS}" >nul
if errorlevel 1 exit /b 1
cl /nologo /MD /EHsc /O2 /std:c++17 /I"{SDK / 'inc/SimConnect'}" "{source}" /Fo"{folder / 'diagnostic.obj'}" /Fe"{executable}" /link /LIBPATH:"{SDK / 'lib/SimConnect'}" SimConnect.lib user32.lib advapi32.lib ole32.lib shell32.lib ws2_32.lib
exit /b %ERRORLEVEL%
''', encoding='utf-8')
    result = subprocess.run(['cmd', '/c', str(command)], cwd=folder, capture_output=True, text=True, timeout=90)
    (folder/'build.log').write_text(result.stdout+result.stderr, encoding='utf-8')
    after = hashlib.sha256(installed.read_bytes()).hexdigest()
    if before != after:
        raise RuntimeError('Installed DLL changed unexpectedly during build')
    if result.returncode:
        raise RuntimeError('Diagnostic build failed; inspect '+str(folder/'build.log'))
    help_result = subprocess.run([str(executable), '--help'], capture_output=True, text=True, timeout=10)
    if help_result.returncode:
        raise RuntimeError('Offline --help check failed')
    manifest = dict(executable=str(executable), executable_sha256=hashlib.sha256(executable.read_bytes()).hexdigest(),
        installed_dll_sha256_before=before, installed_dll_sha256_after=after,
        source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        help_output=help_result.stdout, live_connection_opened=False, parking_pair_protocol=1)
    (folder/'manifest.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8')
    print(json.dumps(manifest, indent=2))
    return executable


if __name__ == '__main__':
    build()
