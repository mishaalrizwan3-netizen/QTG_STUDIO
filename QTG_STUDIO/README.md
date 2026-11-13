# QTG Studio source layout

Run `python main.py` from this directory, or `python QTG_STUDIO/main.py` from the workspace root. `RUN_QTG.bat` may prefer an existing packaged EXE; use the Python command to run current source changes. No EXE was rebuilt in this reorganization.

| Path | Role |
| --- | --- |
| `main.py` | Source and packaged entry point; exposes the existing `main` import. |
| `frontend/main.py` | Tkinter GUI and test settings. The two older standalone Qt GUIs are also here. |
| `frontend/pid_monitor.py` | Existing PID monitor UI code. |
| `qtg_common_engine.py` | Existing worker entry and hardware state machine. |
| `backend/` | Test catalog, input contracts, registry, initialization, validation, control, recording, safety, engine helpers, and artifact naming. |
| `interfaces/p3d.py` | Dashboard P3D bridge; legacy SimConnect and climb-controller transport modules are also here. |
| `interfaces/brunner.py` | Adapter to the existing BRUNNER base at `interfaces/brunner_base/`. |
| `reports/` | Report routing, graph generation, engine report generation, timing rules, and Ti/Tt analysis. |
| `tools/` | Portable build implementation and standalone diagnostics. Root batch files remain launch/build entry points. |
| `test_suites/normal_climb/` | Climb definition, live PID procedure, validation, and original PID source. |
| `test_suites/engine_acceleration/` and `test_suites/engine_deceleration/` | Engine definitions, procedures, and validation adapters. |
| `bridge/`, `p3d_sdk/`, `easa_references/` | Native runtime/build dependencies and reference assets. |
| `tests/` | Offline regression tests. |
| `build/`, `dist/`, `release_backups/` | Generated build files and preserved releases. |

The root compatibility modules keep existing imports and commands working; their implementations now live under the role folders above. Active test code uses one BRUNNER base and the existing common worker. Runtime results remain in `D:\QTG_Test_Results`, grouped by test name; source-local historical recordings and built releases were not moved into runtime results.

See [application architecture](docs/APP_ARCHITECTURE.md) and [test suites](test_suites/README.md).
