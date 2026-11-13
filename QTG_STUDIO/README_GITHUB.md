# QTG Studio

QTG Studio provides an operator interface for flight-simulator qualification test preparation, telemetry recording, and graph reports. Its existing workflows include Normal Climb, Engine Acceleration and Engine Deceleration. Universal Initialization is disabled by default and is not approved as an operational workflow.

The experimental engine throttle sequence is under development. Its offline model is not integrated into the application and must not be treated as a working simulator test.

## Development setup

Use 64-bit Windows and Python 3.10 or later with Tkinter. Create a virtual environment and install the project dependencies, including PySide6, NumPy, pandas, matplotlib, Pillow and psutil. PyInstaller is needed only when building a deployment folder. The exact dependency versions should be validated and pinned by the team for a reproducible build.

```powershell
py -3 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install PySide6 numpy pandas matplotlib Pillow psutil pyinstaller
```

The full application additionally requires a compatible Prepar3D installation and SimConnect runtime, CLS2Sim, BRUNNER hardware and separately supplied, approved native bridge files. The repository includes the SDK resources, reference images and project-built bridge binaries authorized by the project owner for public redistribution. Local simulator recordings and generated deployment packages are excluded. Verify simulator and bridge compatibility before live use.

For the existing deployment spec, the required bridge files are `p3d_bridge.dll`, `bridge/bin/qtg_p3d_bridge.dll`, `bridge/bin/qtg_simconnect.dll` and `interfaces/brunner_base/native/bin/p3d_bridge_engine_init_v1517.dll`. Verify their hashes and compatibility before deployment. Respect the applicable licensing terms for all simulator and reference materials.

For a sanitized collaboration snapshot, set `QTG_TEST_ADMIN_PASSWORD` locally before starting the application. Do not commit the value or put it in a shared script. This applies to this public copy; set the variable in your own environment and never commit its value.

## Run and test

From the application directory, run `python main.py`. Normal startup may connect to the simulator; do not start a live session without supervision.

Focused offline tests can be run with `python -B -m unittest tests.test_initialization_compatibility tests.test_initial_condition_settings`. Some other tests require local data or platform-specific dependencies; review test setup before running them. Do not use offline tests as evidence of simulator or hardware validation.

To build a one-folder Windows deployment after installing PyInstaller and obtaining the authorized native resources, run `python -m PyInstaller --noconfirm QTG_Studio_SimPC.spec`. Copy the entire resulting folder, including `_internal`; the EXE alone is insufficient. Results can be configured with `QTG_RESULTS_ROOT` or a `qtg_settings.json` file beside the EXE.

## Team workflow

Create a feature branch for each change. Submit a pull request to `main`, document offline and supervised testing, and obtain review before merging. Do not force-push or commit recordings, credentials or generated packages. The incomplete throttle sequence belongs on its separate feature branch.
