# Supervised diagnostic checklist

This diagnostic is prepared for operator use; no live testing has been performed.
Universal initialization stays disabled. Do not enable it for this check.

1. Use an already-running QTG worker with existing P3D/CLS2Sim connections.
   The diagnostic cannot create a session. Starting a normal Manual/Automatic
   session can reposition the aircraft; that action is outside this checklist.
2. Establish a stationary aircraft under normal operator procedures. Confirm safe
   controls, clear loaders and no active QTG recording, replay or PID manoeuvre.
3. In the common test panel select **Telemetry Diagnostic**, then **Start monitoring**.
   Observe without moving controls or changing power for the first comparison.
4. Compare independent engine/prop RPM, levers, MAP, MSL altitude, IAS, ground speed,
   weather, weight/fuel/CG and trim against instruments and existing logs. Units and
   source names appear in each row. Native channel registration still requires
   validation; finite values alone do not establish correct source/scale.
5. Treat freshness as UNVERIFIED: only cache-copy timestamps are available. A frozen
   cache is reported as unavailable; timestamps are not native acquisition times.
   Pressure altitude, flaps, pause and ground contact remain missing/unvalidated.
6. BRUNNER positions are separate rounded UI-cache values with no native timestamp,
   sequence or reliable connection-loss evidence. They may persist after disconnect.
   Do not infer physical alignment or fresh hardware feedback from this display.
7. Select **Stop monitoring and save**; wait for Monitoring=False. CSV and JSON
   summary paths appear in the window under the session's telemetry_diagnostics
   directory. Closing the window also requests stop. Stop does not pause P3D,
   release loaders, start/stop QTG recording, or issue control commands.
8. Inspect saved errors, missing channels, sequences, cache times and actual observed
   update intervals before considering a separate approved live validation step.

The existing runtime continues its normal telemetry queries. The diagnostic adds
only cache reads and output writes, never additional read_axis calls or telemetry
packet requests. File writes run on a background thread. Storage failures can leave
the status file stale; the window flags status older than three seconds. A blocked
existing client cache read cannot be forcibly interrupted; stop completes when that
read returns. Multiple diagnostic windows address the same worker monitor.
