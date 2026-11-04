# Shared BRUNNER execution in the QTG workspace

QTG Studio is the only visible interface. Every selected test uses the same
qtg_common_engine.py dispatcher and the existing ../brunner source code.

1. Select a test. For Automatic, select its Manual CSV in the workspace dropdown
   or use Browse CSV. Refresh updates the recording list.
2. Set recording duration, then click Manual or Automatic to connect and initialize.
   Manual uses the BRUNNER base defaults. Automatic restores the selected Manual
   reference and performs all existing power, RPM, control and air-data checks.
3. Read live connection, engine, flight, control and recording status in the workspace.
4. Once initialization is verified, Start Recording becomes enabled.
5. Stop & Save and Emergency Stop pause P3D, release controls and end recording.
   The original recording completion/report functions save captured data.
6. Open Data Folder and Open Latest PDF use the generated outputs.

An invisible Qt process runs the original BRUNNER initialization, synchronized
recording/replay and reporting functions. It uses the offscreen Qt platform and
has no window or console. QTG commands and live snapshots are exchanged through
atomic files under recorded_tests/sessions. Report dialogs and automatic PDF
opening are replaced with inline status and the Open Latest PDF control.
Closing QTG requests a clean worker shutdown after cancellation and cleanup.

Selecting a different test shares initialization/replay; it does not create a new
test-specific maneuver or qualification evaluation. Init failures still block
recording/replay. New recordings and reports are copied into recorded_tests, with
selected-test sidecars; the original BRUNNER output location is retained.

MAKE_EXE.bat bundles the same BRUNNER source, timing calibration and native bridge.
Copy the entire dist/QTG_Studio_FNPTII folder to another Windows x64 PC. No Python
or compiler is required. Rebuild the release after modifying the base source.
The build verifies the workspace and invisible worker without connecting hardware.
