"""Additive one-folder demonstration build; existing releases/DLLs are untouched."""
from pathlib import Path
import datetime
import hashlib
import json
import os
import shutil
import subprocess
import sys
import zipfile

ROOT = Path(__file__).resolve().parents[1]
FLAGS = ['QTG_UNIVERSAL_ENGINE_ACCELERATION', 'QTG_UNIVERSAL_ENGINE_DECELERATION',
         'QTG_UNIVERSAL_INITIALIZATION_ONLY', 'QTG_DECELERATION_BRAKE_CAPTURE',
         'QTG_ACCELERATION_MANUAL_AIRFLOW_PREPARATION']


def main():
    stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    resume = '--resume' in sys.argv
    staging = (Path(sys.argv[sys.argv.index('--resume') + 1]).resolve() if resume
               else ROOT / 'build' / ('demo_' + stamp))
    if not staging.is_relative_to(ROOT / 'build'):
        raise ValueError('Staging must remain under the application build directory')
    staging.mkdir(parents=True, exist_ok=resume)
    distribution = staging / 'distribution'
    release = distribution / 'QTG_Studio_Demo'
    spec = (ROOT / 'QTG_Studio_FNPTII.spec').read_text()
    spec = spec.replace('root = Path(SPECPATH)', 'root = Path(' + repr(str(ROOT)) + ')')
    spec = spec.replace("str(root/'main.py')", "str(root/'tools'/'demo_entry.py')")
    spec = spec.replace("name='QTG_Studio_FNPTII'", "name='QTG_Studio_Demo'", 1)
    # The executable has the requested short name; the collection has the folder name.
    spec = spec.replace("name='QTG_Studio_Demo'", "name='QTG_Studio'", 1)
    spec = spec.replace("name='QTG_Studio_FNPTII'", "name='QTG_Studio_Demo'")
    spec = spec.replace("hiddenimports=[", "hiddenimports=['tools.demo_offline_check','psutil','backend.engine_initialization_adapter',")
    # Data-loaded policies are also included as modules so their dependencies are analyzed.
    spec = spec.replace("'numpy', 'pandas', 'PIL.ImageTk'", "'numpy', 'pandas', 'PIL.ImageTk', 'test_suites._shared.engine_initialization', 'test_suites.engine_acceleration.procedure', 'test_suites.engine_deceleration.procedure'")
    # Preserve both the historical resource locations and the base GUI's fallback config path.
    spec = spec.replace('binaries = [', "datas += [(str(brunner/'config/brunner_timing_calibration.json'), 'config')]\n\nbinaries = [")
    spec_path = staging / 'demo.spec'
    spec_path.write_text(spec, encoding='utf-8')
    production = ROOT / 'interfaces/brunner_base/native/bin/p3d_bridge_engine_init_v1517.dll'
    original_hash = hashlib.sha256(production.read_bytes()).hexdigest()
    if not resume:
        with (staging / 'build.log').open('w', encoding='utf-8') as log:
            subprocess.run([sys.executable, '-m', 'PyInstaller', '--noconfirm', '--distpath', str(distribution),
                            '--workpath', str(staging / 'work'), str(spec_path)], cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, check=True)
    # The registry reads this source with AST; bytecode alone is insufficient.
    shutil.copy2(ROOT / 'test_suites/normal_climb/procedure.py', release / '_internal/test_suites/normal_climb/procedure.py')
    # App-local Microsoft C++ runtime: avoid depending on the development PC's PATH.
    import PySide6
    runtime_candidates = [Path(PySide6.__file__).parent / 'MSVCP140.dll',
                          Path(os.environ.get('WINDIR', 'C:/Windows')) / 'System32/MSVCP140.dll']
    runtime = next((p for p in runtime_candidates if p.is_file()), None)
    if runtime is None:
        raise FileNotFoundError('Microsoft C++ x64 runtime MSVCP140.dll is required')
    shutil.copy2(runtime, release / '_internal/MSVCP140.dll')
    (release / 'logs').mkdir(exist_ok=True)
    launcher = '@echo off\nsetlocal\ncd /d "%~dp0"\n'
    launcher += ''.join('set "' + flag + '=0"\n' for flag in FLAGS)
    launcher += '''set "QTG_DEMO_OFFLINE_TEST="
if not exist "logs" mkdir "logs"
set "MPLCONFIGDIR=%~dp0logs\\matplotlib"
if not exist "%MPLCONFIGDIR%" mkdir "%MPLCONFIGDIR%"
if not exist "QTG_Studio.exe" (
 echo ERROR: QTG_Studio.exe is missing. Extract the complete ZIP.
 pause
 exit /b 2
)
if not exist "_internal\\python313.dll" (
 echo ERROR: The bundled runtime is missing. Extract the complete ZIP including _internal.
 pause
 exit /b 2
)
if not exist "D:\\" (
 echo ERROR: D: drive is required by the existing QTG results configuration.
 echo Prepare D:\\QTG_Test_Results on the simulator PC before launching.
 pause
 exit /b 3
)
if not exist "D:\\QTG_Test_Results" mkdir "D:\\QTG_Test_Results"
if not exist "D:\\QTG_Test_Results" (
 echo ERROR: Cannot create D:\\QTG_Test_Results. Check drive permissions.
 pause
 exit /b 3
)
echo %date% %time% Starting QTG Studio >> "logs\\launcher.log"
start "QTG Studio" /wait "%~dp0QTG_Studio.exe" >> "logs\\launcher.log" 2>&1
set "QTG_EXIT=%ERRORLEVEL%"
echo %date% %time% Exit code %QTG_EXIT% >> "logs\\launcher.log"
if not "%QTG_EXIT%"=="0" (
 echo ERROR: QTG Studio exited with code %QTG_EXIT%. See logs and startup_error.log.
 pause
)
exit /b %QTG_EXIT%
'''
    (release / 'Launch_QTG_Studio.bat').write_bytes(launcher.replace('\n', '\r\n').encode('ascii'))
    reference_files = [
        ('1_f_1_Engine_acceleration', '1_f_1_Engine_acceleration_manual_2026-11-10_16-01-46'),
        ('1_f_2_Engine_deceleration', '1_f_2_Engine_deceleration_manual_2026-11-10_13-52-11')]
    for folder, stem in reference_files:
        target = release / 'References' / folder
        target.mkdir(parents=True, exist_ok=True)
        for suffix in ['.csv', '_init_conditions.json']:
            source = Path('D:/QTG_Test_Results') / folder / (stem + suffix)
            if not source.is_file():
                raise FileNotFoundError('Required demonstrated reference: ' + str(source))
            shutil.copy2(source, target / source.name)
    (release / 'README_OFFLINE.txt').write_text('''QTG Studio - three-test offline demonstration package

Copy/extract the ENTIRE QTG_Studio_Demo folder to a writable local location.
Do not run inside the ZIP. Keep QTG_Studio.exe and _internal together.
No Python, internet, Visual Studio or development environment is required.

Simulator prerequisites: Windows x64, Prepar3D v5 with its working SimConnect
installation and Super Mushshak aircraft/scenery, CLS2Sim, BRUNNER drivers and
the established working control profile (CLS2Sim remote interface port 15090).
Microsoft Visual C++ x64 runtime must be available, normally installed with
P3D/CLS2Sim. Native DLL load errors are recorded in startup logs/test status.
This package does not install or replace anything in P3D or CLS2Sim.

Start P3D and CLS2Sim, then double-click Launch_QTG_Studio.bat.
The launcher disables Universal Initialization, initialization-only mode,
optional brake capture and experimental Manual airflow preparation.
Results retain the existing D:\\QTG_Test_Results location. D: must exist and
be writable; the launcher reports a missing drive/directory explicitly.

Select Normal Climb 1.c.(1), Engine Acceleration 1.f.(1), or Deceleration 1.f.(2).
Use the existing Manual and Automatic procedures. Climb PID Automatic performs
stabilization and starts recording automatically; engine replay waits for
readiness and Start Recording. Opening a test can initialize the aircraft.
Check selected-worker P3D and BRUNNER connections, not just dashboard status.

References contains copies of the demonstrated engine Manual CSVs and matching
sidecars. Browse CSV can import a reference into its normal test result folder.
Keep the original CSV filename and sidecar together. Never overwrite an existing
reference. If already present, select that existing reference in the combo box.
Verify the selected filename after Refresh (Deceleration may select the newest).
Climb PID Automatic does not require an engine Manual reference.
Source-session GUI edits are not transferred as a saved user profile: check the
packaged initial-condition fields and weather against the proven setup.

Stop & Save / Emergency Stop use the existing worker stop path. Verify P3D pause
and physical control release. Wait for reports before switching tests/closing.
Verify each new CSV/PDF timestamp; do not mistake an older PDF for a new report.

Diagnostics: logs\\launcher.log, logs\\startup_*.log, startup_error.log,
and ordinary worker status/log files under D:\\QTG_Test_Results.
Offline verification evidence: demo_offline_results.json and offline_verification.
To rerun hardware-free packaged checks: QTG_Studio.exe --demo-self-test

Offline checks do not establish live simulator/hardware compatibility. Perform
one supervised startup and each test's existing workflow on the simulator PC
before the demonstration. No live simulator testing was performed for this build.
''', encoding='utf-8')
    env = os.environ.copy()
    env['PATH'] = str(Path(os.environ.get('WINDIR', 'C:/Windows')) / 'System32')
    for key in ['PYTHONPATH', 'PYTHONHOME', 'QT_PLUGIN_PATH', 'QML2_IMPORT_PATH']:
        env.pop(key, None)
    env.update({flag: '0' for flag in FLAGS})
    env['MPLCONFIGDIR'] = str(release / 'logs/matplotlib')
    Path(env['MPLCONFIGDIR']).mkdir(exist_ok=True)
    subprocess.run([str(release / 'QTG_Studio.exe'), '--demo-self-test'], cwd=release, env=env, check=True, timeout=180)
    assert hashlib.sha256(production.read_bytes()).hexdigest() == original_hash
    sources = {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
               for folder in ['backend', 'frontend', 'interfaces', 'qtg_backend', 'reports', 'test_suites', 'tools']
               for path in (ROOT / folder).rglob('*.py') if '__pycache__' not in path.parts}
    manifest = {'built_at': stamp, 'entry': 'tools/demo_entry.py -> frontend.main.cli (same current application)',
                'python': sys.version, 'production_worker_dll_sha256': original_hash, 'source_hashes': sources,
                'packaged_files': {str(p.relative_to(release)): hashlib.sha256(p.read_bytes()).hexdigest()
                                   for p in release.rglob('*') if p.is_file()}}
    (release / 'build_manifest.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8')
    final = ROOT / 'dist' / 'QTG_Studio_Demo'
    if final.exists():
        raise FileExistsError('Existing demo preserved; new verified build remains at ' + str(release))
    shutil.copytree(release, final)
    archive = ROOT / 'dist' / ('QTG_Studio_Demo_' + stamp + '.zip')
    with zipfile.ZipFile(archive, 'x', zipfile.ZIP_DEFLATED) as package:
        for path in final.rglob('*'):
            if path.is_file():
                package.write(path, Path(final.name) / path.relative_to(final))
    print(json.dumps({'folder': str(final), 'zip': str(archive), 'build_log': str(staging / 'build.log'),
                      'offline': json.loads((final / 'demo_offline_results.json').read_text())}, indent=2))


if __name__ == '__main__':
    main()
