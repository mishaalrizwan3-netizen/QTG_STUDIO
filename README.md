# QTG Studio workspace

The source application is `QTG_Studio_Antigravity_Single_Main/main.py`.
Run `python QTG_Studio_Antigravity_Single_Main/main.py` from this folder to use current source changes.
`RUN_QTG.bat` prefers an existing EXE, which may be older than the source.
No executable was rebuilt during this organization.

## Required components

| Location | Purpose |
| --- | --- |
| `QTG_Studio_Antigravity_Single_Main/` | Main GUI, shared execution engine, climb PID integration, existing engine modules and report generators |
| `brunner/Super_Mushak_Logger_GUI.py` | Existing initialization and recording implementation used by the common engine |
| `brunner/src/` | Shared hardware, simulator, replay and report modules |
| `brunner/native/bin/` and application `bridge/bin/` | Native runtime bridges; keep DLLs at existing paths |
| `PID Controller/SMK_IAS_PID_BRUNNER_ONLY_V4_1.py` | Original PID algorithm and transport source loaded by the climb integration |
| Application `easa_references/`, `test_catalog.csv`, `p3d_sdk/` | Reference assets and native build dependencies |
| Both projects' `tests/` and root test scripts | Regression and standalone diagnostic checks |
| `docs/` | Workspace inventory and dependency/cleanup notes |
| `_archive/organization_2026-11-04/` | Original documentation and loose generated artifacts, with a restoration manifest |

## Recording and control paths

QTG results use `D:\QTG_Test_Results` in per-test folders; source-local `recorded_tests/` contains historical data.
The independent BRUNNER application has its own historical output paths. Do not merge or delete recordings based on similar names.
Normal Climb PID runs through the common engine without opening the PID GUI. It uses the existing shared hardware connection, targets 80 kt, and records after IAS remains within +/-1 kt for 5 seconds and control-path verification passes.
The PID duration default is 75 seconds. Recorded Replay uses the selected test's Manual reference CSV.
Manual and Replay initialization/control behavior was not changed by workspace organization.

## Development and release

Keep the three component directories beside one another: their source and packaging paths depend on this layout.
The application `MAKE_EXE.bat`, `build_portable.py` and `QTG_Studio_FNPTII.spec` define the release build.
Copy the entire `dist/QTG_Studio_FNPTII/` folder for a portable release, including `_internal`.
Do not rebuild unless requested. Never run hardware diagnostics as part of automatic cleanup.

See [organization notes](docs/WORKSPACE_ORGANIZATION.md) for consolidation decisions and the machine-readable file inventory.

## Browse by test

Open [test_suites/README.md](test_suites/README.md) for one folder per catalog test, plus shared implementation links. Climb report previews are now grouped under its test folder.
