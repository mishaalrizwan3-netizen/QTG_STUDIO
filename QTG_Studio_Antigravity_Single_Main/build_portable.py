"""Build to staging, preserve user results, and replace dist only after verification."""
from pathlib import Path
import datetime
import hashlib
import json
import os
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parent
STAGING = ROOT/'build/portable_dist'
RELEASE = STAGING/'QTG_Studio_FNPTII'
DIST = ROOT/'dist/QTG_Studio_FNPTII'

def main():
    for relative in ['bridge/bin/qtg_simconnect.dll','bridge/bin/qtg_p3d_bridge.dll','p3d_bridge.dll']:
        if not (ROOT/relative).is_file(): raise FileNotFoundError(relative)
    subprocess.run([sys.executable,'-m','PyInstaller','--noconfirm',
                    '--distpath',str(STAGING),'--workpath',str(ROOT/'build/portable_work'),
                    str(ROOT/'QTG_Studio_FNPTII.spec')],cwd=ROOT,check=True)
    (RELEASE/'START_QTG.bat').write_bytes(b'@echo off\r\ncd /d "%~dp0"\r\nstart "" "%~dp0QTG_Studio_FNPTII.exe"\r\n')
    (RELEASE/'README.txt').write_text('QTG Studio - standalone Windows x64 release\n\n'
        'Run START_QTG.bat or QTG_Studio_FNPTII.exe.\n'
        'Copy this ENTIRE folder to the destination PC, including _internal.\n'
        'Python, pip, Visual Studio and C++ build tools are not required.\n'
        'Prepar3D and the BRUNNER/CLS2Sim software and device drivers are still required for hardware tests.\n'
        'Keep the folder in a writable location. Recorded reports are in recorded_tests.\n'
        'The engine suite uses the same EXE and bundled runtime.\n',encoding='utf-8')
    (RELEASE/'recorded_tests').mkdir(exist_ok=True)
    for relative in ['recorded_tests','test_notes.json']:
        source=DIST/relative
        if source.is_dir(): shutil.copytree(source,RELEASE/relative,dirs_exist_ok=True)
        elif source.is_file(): shutil.copy2(source,RELEASE/relative)
    manifest={'built_at':datetime.datetime.now().isoformat(), 'python':sys.version,
              'sources':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in ROOT.glob('*.py')}}
    (RELEASE/'build_manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
    env=os.environ.copy()
    env['PATH']=str(Path(os.environ.get('WINDIR','C:/Windows'))/'System32')
    for key in ['PYTHONPATH','PYTHONHOME','QT_PLUGIN_PATH','QML2_IMPORT_PATH']:env.pop(key,None)
    subprocess.run([str(RELEASE/'QTG_Studio_FNPTII.exe'),'--portable-self-test'],
                   cwd=RELEASE,env=env,check=True,timeout=120)
    subprocess.run([str(RELEASE/'QTG_Studio_FNPTII.exe'),'--brunner-self-test',str(RELEASE/'brunner_self_test.json')],cwd=RELEASE,env=env,check=True,timeout=60)
    result=json.loads((RELEASE/'portable_self_test.json').read_text())
    if not result.get('ok'): raise RuntimeError(result)
    # Only the explicitly named release directory is moved, and only within ROOT.
    backup=ROOT/'release_backups'/datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    for target in [DIST, RELEASE, backup]:
        if not target.resolve().is_relative_to(ROOT):raise RuntimeError('Unsafe release path')
    if DIST.exists():
        backup.parent.mkdir(parents=True,exist_ok=True)
        shutil.move(str(DIST),str(backup))
    DIST.parent.mkdir(exist_ok=True)
    shutil.move(str(RELEASE),str(DIST))
    print('Release ready:',DIST)
    print('Offline packaged checks:',result)

if __name__=='__main__':main()
