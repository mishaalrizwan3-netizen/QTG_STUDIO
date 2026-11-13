Super Mushshak V15.12.5 — P3D code (close source references)

Files:
1. p3d_simconnect_v15.py — Python SimConnect wrapper. Requires p3d_bridge.dll compiled for Windows x64 and matched to the Python ctypes structures.
2. p3d_bridge_v15_full_causal_state.cpp — corresponding C++ source implementing SimConnect communication and aircraft telemetry/state setup (bridge version 1498 in this related source copy).
3. P3D_DirectAxisBridge.py — reference-only automatic-mode adapter using the already-connected SimConnectClient. NOT standalone. Contains direct elevator/aileron/rudder writes to P3D.

Provenance: The user-provided V15.12.5 EXE was statically inspected. Its executable contains Python 3.14 compiled bytecode, not the original Python and C++ sources. These readable sources are closely related project copies, NOT guaranteed byte-for-byte matches to the EXE or its running DLL.

Integration: Keep the existing working V15.12.5 EXE and DLL unchanged. Do not run automatic direct axis commands in parallel with a native CLS2Sim P3D control path unless double-input conflicts have been designed out. Read-only telemetry is separate from aircraft-initialization and direct control writes.
