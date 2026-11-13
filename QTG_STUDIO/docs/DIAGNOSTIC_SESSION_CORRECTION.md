# November 10 diagnostic investigation

Read-only evidence: 20261110T075149Z_a138ce34.csv and matching summary under
D:/QTG_Test_Results/sessions/telemetry_diagnostics; the 12:52:15 Deceleration
run context and its session log (1794297045299382000_status.log).

63 observed snapshots contain 54 with P3D values. Engine RPM's last available
snapshot is 2026-11-10T07:52:16.435091+00:00, sequence 4701. Nine subsequent
snapshots through 07:52:21.138542 have no P3D values. The saved issue is
`p3d: repeated/out-of-order snapshot`, not a confirmed transport disconnection.
ReadOnlyTelemetry rejects repeated sequence/time; DiagnosticMonitor.sample catches
that rejection and leaves measurements missing. It must not fill them from history.

The worker log shows Deceleration completion, simulator pause and synchronous report
generation, then later Acceleration activity using the same worker. Main telemetry
is frame-driven; paused telemetry refresh requests normally come from the UI timer.
Synchronous report generation can delay that timer. This is a code-supported
explanation for a repeated sequence; the log has no per-line acquisition timestamps
to prove the exact blocked interval. Worker shutdown is not established by this CSV.
The worker remains usable afterward, so do not label these nine rows as shutdown.
Actual shutdown now records an explicit worker_shutdown end reason before stopping
the diagnostic and before stopping the simulator client. Worker stage is sampled
separately in availability transitions without changing telemetry.

The old JSON simply retained `measurements` from the last loop iteration. It now
keeps that honest final snapshot plus a `session` aggregate: channel availability
counts, first/last finite readings with original metadata, contiguous missing
intervals and source-state/worker-stage transitions. Finite is not verified:
verified_sample_count remains zero and BRUNNER display values stay potentially stale.

WinError 5 occurred on replacement of the diagnostic status JSON. Windows can deny
replacement while a GUI/other reader holds the destination without delete sharing.
The evidence does not identify the locking process or rule out ACL/security tools.
The ordinary worker status filename and its temporary filename differ from the
diagnostic filenames. No collision with ordinary status is demonstrated.
Publication now uses per-monitor private temporary files and three bounded replace
attempts; errors/counters are retained. CSV and summary saving are independent of
status publication. These writes and retries occur only in the monitor thread;
they do not acquire worker/hardware locks or change ordinary status publication.

Original supplied artifacts are unchanged. No values, clock stamps, provenance or
validation rules are rewritten to suppress the missing tail. Native acquisition
timing and physical BRUNNER freshness remain unverified. No live testing performed.
