# Diagnostic timestamp correction

| Evidence | Clock / units | Meaning |
|---|---|---|
| Native acquisition timestamp | Unavailable | Never reconstructed or claimed verified |
| brunner-base last_updated | time.time(), Unix seconds | Python cache-copy time, even when native packet sequence has not progressed |
| bridge_telemetry_sequence | Native counter | Packet progression, not a timestamp or simulation time |
| Dashboard telemetry_age_ms | GetTickCount64 receive age, milliseconds | Host bridge receive-age, not aircraft acquisition time |
| Dashboard diagnostic last_updated | time.time() minus age/1000 | Receive-age estimate in Unix seconds; counter is only an observation counter |
| Diagnostic observed_utc | UTC datetime | Time the diagnostic observed the cache |
| Diagnostic monotonic observation | time.monotonic(), seconds | Local interval measurement; never compared directly to Unix epoch |
| Converted cache_copy_monotonic | monotonic observation minus same-domain wall age | Diagnostic estimate only; absent when clock metadata is uncertain |

The strict adapter anchors wall-minus-monotonic once, then subtracts that offset
from epoch timestamps and accepts only 1e-9 seconds of negative age. Sampling order
and epoch floating-point cancellation can exceed that margin without genuinely
future data. A focused test reproduces rejection with a microsecond difference.
The same exception also conflated old samples, future dates and clock mismatch,
and DiagnosticMonitor previously discarded all otherwise valid measurements.
The prior error text alone cannot prove the precise live trigger: raw clock inputs
were not logged. Frozen sequences are a separate, previously demonstrated issue.

DiagnosticMonitor now exclusively calls diagnostic_feedback(). It compares epoch
timestamps to wall time in the same domain, logs raw source time and its meaning,
and retains finite values without claiming freshness. Missing timestamps, uncertain
domains, future dates and host-clock changes are separately labelled unverified.
Repeated sequences are temporarily unchanged, possibly paused; sustained sequence
freezing or old cache metadata are explicitly STALE_CACHE. No pause is inferred as
fact. Copy timestamp changes alone do not disguise a frozen native packet counter.
Dashboard's observation counter cannot prove native progression; receive age is
still used as additional evidence and remains explicitly unverified.

The strict feedback() path, manager authorization, stale limits, sources, values,
clock provenance and QTG execution are unchanged. The display-only path can return
no usable monotonic stamp; it is not an authorization adapter. CSV records retain
raw source timestamps rather than inventing acquisition timestamps. Native/physical
BRUNNER freshness remains unverified. No hardware testing or feature activation.

On the simulator PC compare source_timestamp, source_clock, sequence, observed UTC,
and cache monotonic estimates during running and paused conditions for each source.
Check Windows clock adjustment and resume behavior. No live root-cause claim should
be made without that capture. Universal initialization must remain disabled.
