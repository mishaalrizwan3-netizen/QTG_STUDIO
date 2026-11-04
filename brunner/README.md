# Super Mushshak Logger

Run the source GUI with `RUN_GUI.bat` or `.venv\Scripts\python.exe Super_Mushak_Logger_GUI.py`.
The launch script prefers the project virtual environment.

## Project groups

- `Super_Mushak_Logger_GUI.py`: GUI entry point.
- `src/`: simulator driver, Brunner interface, replay logic, plotting and standalone tools.
- `native/src/`: C++ bridge source.
- `native/bin/`: active bridge DLL (version 1515).
- `config/`: measured Brunner timing calibration.
- `tools/`: executable/native build scripts and packaging specifications.
- `tests/`: regression tests.
- `launchers/`: standalone tool launch scripts; `RUN_3AXIS.bat` remains a root shortcut.
- `Output/reports/`: existing PDF reports and plots.
- `Output/recordings/`: existing local CSV/JSON recordings and the latest local telemetry copy.
- `Output/diagnostics/`: existing diagnostic results.
- `docs/`: legacy documentation.
- `build/`: generated build and smoke-test artifacts.
- `dist/Super_Mushak_Logger/`: portable GUI executable and supporting libraries.
- `dist/standalone/`: older standalone 3-axis executable.
- `_archive/`: recovery ZIP from the earlier cleanup.

## Build and check

Run `tools\build_exe.cmd` to build the GUI with PyInstaller in `.venv`.
Run `tools\build_bridge.cmd` to rebuild/install the native DLL; close the GUI first.
The native build requires Visual Studio 2022 C++ tools and the installed compatible
Prepar3D v4.5 SDK client library. The portable executable needs no Python or SDK.

Run `.venv\Scripts\python.exe -B -m unittest discover -s tests -p "test_*.py"` for regression checks.

## Portable GUI

Launch `dist\Super_Mushak_Logger\Super_Mushak_Logger.exe`.
Copy the **entire Super_Mushak_Logger folder**, including `_internal`, to another
Windows x64 PC. Prepar3D and CLS2Sim must be installed and running for flight tests.
Logs are saved in `logs/application.log` beside the executable. Calibration changes
are saved in `config/` beside the executable, using the bundled calibration initially.

Flight recordings continue to use `D:\Super_Mushak_Logger_Data` when D: exists,
or `Documents\Super_Mushak_Logger_Data` otherwise. Existing recordings are retained.
Pitch, roll and rudder commands use Brunner; P3D throttle replay is enabled.

Automatic initialization restores recorded mixture and propeller levers only while paused, then verifies measured readback. Runtime flight axes remain Brunner-only.
