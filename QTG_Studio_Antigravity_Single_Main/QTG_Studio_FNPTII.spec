# Build a single shared runtime for the Tk workspace and Qt engine suite.
from pathlib import Path
from PyInstaller.utils.hooks import collect_dynamic_libs
root = Path(SPECPATH)
brunner = root.parent/'brunner'
datas = [(str(root.parent/'PID Controller'/'SMK_IAS_PID_BRUNNER_ONLY_V4_1.py'), 'pid_source'),
         (str(brunner/'Super_Mushak_Logger_GUI.py'), 'brunner_base'),
         (str(brunner/'config/brunner_timing_calibration.json'), 'brunner_base/config'),
         (str(root/'easa_references'), 'easa_references'),
         (str(root/'test_catalog.csv'), '.')]
datas += [(str(brunner/'src'/name), 'brunner_base/src') for name in ['replay_support.py','brunner_interface.py','p3d_simconnect_v15.py','plot_graphs.py']]
datas += [(str(root.parent/'test_suites'/relative), 'test_suites/'+str(Path(relative).parent)) for relative in ['_shared/engine_initialization.py','1_f_1_Engine_acceleration/execution.py','1_f_2_Engine_deceleration/execution.py']]

binaries = [(str(brunner/'native/bin/p3d_bridge_engine_init_v1517.dll'), 'brunner_base/native/bin'),
            (str(root/'bridge/bin/qtg_p3d_bridge.dll'), 'bridge/bin'),
            (str(root/'bridge/bin/qtg_simconnect.dll'), 'bridge/bin'),
            (str(root/'p3d_bridge.dll'), '.')]
binaries += collect_dynamic_libs('numpy')
binaries += collect_dynamic_libs('pandas')
a = Analysis([str(root/'main.py')], pathex=[str(root)], binaries=binaries, datas=datas,
    hiddenimports=['qtg_pid_monitor','qtg_climb_pid','winsound', 'qtg_common_engine', 'PySide6.QtGui', 'PySide6.QtCore', 'SMK_Engine_From_Climb', 'portable_self_test', 'engine_report_v2',
                   'plot_graphs', 'SMK_Engine_TiTt_V2_1_3_Addon', 'qtg_position_setter',
                   'numpy', 'pandas', 'PIL.ImageTk'],
    hooksconfig={'matplotlib': {'backends': ['Agg']}},
    hookspath=[], runtime_hooks=[], excludes=['PyQt5','PyQt6','PySide2','IPython','pytest','unittest','numpy.tests','pandas.tests','matplotlib.tests'],
    noarchive=False, optimize=1)
pyz = PYZ(a.pure)
exe = EXE(pyz, a.scripts, [], exclude_binaries=True, name='QTG_Studio_FNPTII',
    debug=False, strip=False, upx=False, console=False, disable_windowed_traceback=False)
coll = COLLECT(exe, a.binaries, a.datas, strip=False, upx=False, name='QTG_Studio_FNPTII')
