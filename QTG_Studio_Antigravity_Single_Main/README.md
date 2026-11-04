# QTG Studio — Antigravity (single main.py edition)

**Everything you asked to edit in Python is visible in `main.py`.** It contains the 26 draft EASA FNPT II test definitions, the complete Tkinter GUI, the Python read-only P3D telemetry wrapper, and the `if __name__ == "__main__"` entry point. No `qtg_studio/` Python subfolder is needed.

## Open in Antigravity (Windows PC)

1. **Extract** this ZIP (do not open the code directly inside the ZIP).
2. Open **Antigravity → Open Folder** → `QTG_Studio_Antigravity_Single_Main`.
3. Open **main.py**. The complete code is visible.
4. Open Antigravity **Terminal** and enter:

   ```powershell
   py -3 main.py
   ```

   Or double-click **RUN_QTG.bat**. Python 3.10+ with Tkinter is required. No pip setup is needed for the GUI.

5. Choose any of the 26 tests in the left-hand test tab list, review its EASA reference and save your notes.

**No P3D is needed just to see the GUI**. The P3D indicator stays offline until its DLL is compiled and P3D is running. The CLS2Sim indicator is a placeholder, not connected.

## To show live P3D data

The **GUI itself is Python**, but connecting P3D with this included C++ bridge requires one local Windows compilation:

1. Install **Visual Studio Build Tools** with *Desktop development with C++* (if not already installed).
2. In Antigravity terminal, run `./BUILD_P3D_BRIDGE.bat` (PowerShell) or double-click it in File Explorer. It attempts to find and initialize the x64 compiler automatically.
3. It should create `bridge\bin\qtg_p3d_bridge.dll`. If compilation fails, read the error; the GUI will still run.
4. Start Prepar3D and load the aircraft, open the GUI and click **Connect P3D**. You can monitor IAS, altitude, pitch, etc.

The supplied SDK (`p3d_sdk/SimConnect.h`, `p3d_sdk/SimConnect.lib`) was provided by you. **Check the installed Prepar3D version and matching SDK** before relying on a live build. The legacy `SimConnect.dll` is 32-bit, and is kept isolated in `p3d_sdk/legacy_x86_do_not_load/`—it must NOT be used by the new 64-bit bridge.

## File structure

```text
QTG_Studio_Antigravity_Single_Main/
  main.py                       ALL Python GUI, test definitions, P3D wrapper
  RUN_QTG.bat                   Double-click: run GUI
  BUILD_P3D_BRIDGE.bat          Compile native x64 C++ bridge on Windows
  MAKE_EXE.bat                  Optional PyInstaller EXE builder
  test_catalog.csv              Editable EASA index / reference list
  bridge/
     qtg_p3d_bridge.cpp        New read-only SimConnect C++ source
     qtg_p3d_bridge.h          C++ / Python structure declaration
     bin/                      Compiled DLL appears here after build
  p3d_sdk/
     SimConnect.h              User-supplied SDK
     SimConnect.lib            User-supplied x64 library
     legacy_x86_do_not_load/   User-supplied x86 DLL (not run)
  reference_previous_bridge/
     SimBridge_BrunnerOnly_V4_1.cpp
     smk_simbridge_v4.dll       Old bridge for reference only; NOT loaded
```

## Antigravity instructions you can paste

> Open `main.py` and make all future GUI modifications in this file so I can see the full GUI code directly. Maintain exactly 26 test entries in the sidebar. Keep the existing EASA references and test notes. Do not issue writes to P3D or BRUNNER until I give explicit test procedures. The `bridge/` code is a read-only x64 SimConnect client; preserve its ABI if changing it. Do not load the previous bridge DLL.

## Important current limits

- This is a **clickable GUI scaffold**, not functioning QTG tests. Test procedures, manual/automatic control, CLS2Sim protocol integration, measurement and reporting will be added as requirements arrive.
- The provisional list has **26 tests for a single-engine baseline**. Three separately marked engine-inoperative entries are excluded for now. Some entries depend on fitted equipment (trim, spoiler, landing gear, visual etc.). Confirm qualification applicability; the tabs do **not** establish EASA compliance.
- P3D telemetry path is **read-only**, no automatic throttle, no pitch writes, no automatic BRUNNER movement, no pauses.
- The new DLL must be compiled and simulator-tested on your Windows PC. It is **not** the same as your old `smk_simbridge_v4.dll`.
- Quick integrity check: `py -3 main.py --check`.
