SMK BRUNNER-ONLY V4 — SOURCE PACKAGE (NOT HARDWARE-VALIDATED)

IMPORTANT: This is a prototype for simulator bench testing, not an approved flight control system.

1. Build SimBridge_BrunnerOnly_V4.cpp as a 64-bit DLL named smk_simbridge.dll using the Prepar3D SimConnect SDK, Windows C++ toolchain, and matching SimConnect libraries. Put DLL beside the Python script.
2. Install Python dependencies: matplotlib (tkinter comes with many Windows Python installs).
3. Ensure CLS2Sim forwards physical pitch position to P3D, and no other input mapper fights it.
4. Run the Python script on the simulator PC. Choose AIRSPEED mode; BRUNNER is the only selectable output.
5. First test with simulator paused / hardware motion area clear; confirm pitch direction and physical movement at small commands. Then test in a safe simulated flight envelope with manual stop accessible.

P3D WRITES: ONLY PLANE BANK DEGREES = 0 when engaged. No elevator, yoke or axis command writes in the C++ bridge.
BRUNNER WRITES: autopilot enable, speed, force, target position, override settings. CLS2Sim must transmit physical stick position to P3D.
WEIGHT 1050 kg AND TEMPERATURE 20 C: NOT automatically configured. P3D TOTAL WEIGHT and AMBIENT TEMPERATURE are not directly writable via these SimVars. Configure fuel/payload and weather separately and verify in P3D before each run.
THROTTLE: manual (not written).

LIMITATIONS: Not compiled/tested against your actual SimConnect SDK or BRUNNER hardware. Verify override 0x06 release semantics with your CLS2Sim version. The original gains and GUI are retained for continuity; do not assume IAS accuracy. Stop immediately for wrong-direction response, large pitch excursions, or stale telemetry.
