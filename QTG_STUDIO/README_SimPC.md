# QTG Studio simulator PC

Extract the complete `QTG_Studio` folder, including `_internal`, to a writable location on the simulator PC. Double-click `QTG_Studio.exe`; Python and VS Code are not required. Keep this candidate separate from the existing demonstration installation.

Prepar3D, its compatible SimConnect runtime, CLS2Sim and the configured BRUNNER hardware are external prerequisites. Connect and check them using the established simulator procedures before running a test. The packaged bridge is a copy of the existing worker DLL; the simulator installation is not modified by extracting this folder.

Results and per-test settings default to `Documents\QTG_Test_Results` for the current Windows user. To choose another folder, create `qtg_settings.json` beside the EXE, for example:

```json
{"results_directory": "D:\\QTG_Test_Results"}
```

An absolute path or a path relative to the EXE folder is accepted. Alternatively, set the `QTG_RESULTS_ROOT` environment variable before launching; it takes precedence. Ensure the selected location is writable. Initial-condition edits are saved when a field loses focus or Enter is pressed.

Universal Initialization is disabled in this deployment. No live simulator or BRUNNER validation was performed as part of the build. Before operational use, perform the established supervised simulator acceptance checks and retain the existing demonstration package as a rollback option.
