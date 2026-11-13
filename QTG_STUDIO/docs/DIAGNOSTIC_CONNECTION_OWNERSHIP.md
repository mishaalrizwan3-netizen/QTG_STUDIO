# Diagnostic connection correction

Global P3D indicator: frontend App.p3d is interfaces.p3d.P3DClient, using
QTG_BridgeStart/QTG_GetState in bridge/qtg_p3d_bridge.cpp. It is a limited,
independent dashboard transport. Its running flag means bridge thread started;
connected/telemetry_valid/age in snapshot provide separate evidence.

Global CLS2Sim indicator can reflect process discovery or a manually toggled
boolean, not a validated loader connection. The common test panel instead shows
backend.recorder.snapshot of BackgroundEngine.p3d and BackgroundEngine.brunner.sock
in the separate qtg_common_engine worker process. No worker means no test-panel
connection. A running CLS2Sim process is not physical telemetry verification.

Open Telemetry Diagnostic > Connect for Diagnostics > Start Monitoring. No test
needs to start. A connected active worker is preferred; otherwise the connected
dashboard is borrowed read-only. Dashboard subset includes independent engine and
prop RPM, IAS, MSL altitude, pitch/bank, throttle, weight and OAT. Missing richer
channels stay missing. Its sequence is an observation counter, and time is derived
from bridge receive-age; these are labelled unverified, not native acquisition.

If neither is usable, a separate --diagnostic-connection process loads the existing
brunner_base/src/p3d_simconnect_v15.py directly (not load_base or worker_class).
connect_and_run explicitly receives both placement flags False, with no callbacks.
Native ConnectSim opens/registers telemetry and maps events but does not transmit
aircraft-control events. Polling copies telemetry; no BRUNNER import or connection
occurs. Separate processes isolate the DLL's singleton handle, locks and caches;
same-process duplicate SimConnectClient instances would not be safe ownership.
Source inspection supports this isolation; actual simultaneous-client behavior
and deployed DLL/source identity still require supervised simulator confirmation.

Stop Monitoring saves logs and leaves transport ownership intact. Closing the
window stops its borrowed monitor only, or asks its standalone child to close its
own connection. Shared dashboard/worker connections are never stopped. A single
window per GUI prevents accidental duplicate diagnostic sessions.

Follow SUPERVISED_TELEMETRY_CHECK.md, starting with stationary aircraft. Connecting
does not initialize, pause/unpause, change power, install wrappers or authorize
recording. Check transport, active-worker state and measurement validity separately.
Dashboard subset freshness/clock metadata and cached BRUNNER text remain unverified.
The standalone session does not expose BRUNNER positions. Universal remains disabled.
