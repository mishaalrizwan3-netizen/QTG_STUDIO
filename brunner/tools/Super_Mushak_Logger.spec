from pathlib import Path
import os
root=Path(SPECPATH).parent
bridge=root/'native/bin/p3d_bridge_engine_init_v1515.dll'
binaries=[(str(bridge),'native/bin')]
for name in ['MSVCP140.dll','VCRUNTIME140.dll','VCRUNTIME140_1.dll']:
    path=Path(os.environ['WINDIR'])/'System32'/name
    if path.exists():binaries.append((str(path),'.'))
a=Analysis([str(root/'Super_Mushak_Logger_GUI.py')],pathex=[str(root),str(root/'src')],
    binaries=binaries,datas=[(str(root/'config/brunner_timing_calibration.json'),'config')],
    hiddenimports=['tools.packaging_check'],hookspath=[],runtime_hooks=[],
    excludes=['PyQt5','PyQt6','PySide2','tkinter'],hooksconfig={'matplotlib':{'backends':['Agg']}},noarchive=False)
pyz=PYZ(a.pure)
exe=EXE(pyz,a.scripts,[],exclude_binaries=True,name='Super_Mushak_Logger',debug=False,
    bootloader_ignore_signals=False,strip=False,upx=False,console=False)
coll=COLLECT(exe,a.binaries,a.datas,strip=False,upx=False,name='Super_Mushak_Logger')
