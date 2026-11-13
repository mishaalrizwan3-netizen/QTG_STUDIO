# Supervised Manual Acceleration airflow experiment

This option is OFF by default. It applies only to Manual Engine Acceleration
`1.f.(1)`, including Set Conditions. Automatic Replay, PID, Deceleration and
Universal Initialization are not enabled or changed by this switch.

## Activate for one supervised application session

1. Close QTG Studio and wait for its test worker to exit. Do not interrupt an active run.
2. On the simulator PC, open PowerShell and launch the source application:

   ```powershell
   Set-Location '<PROJECT_ROOT>'
   $env:QTG_ACCELERATION_MANUAL_AIRFLOW_PREPARATION = '1'
   python main.py
   ```

   For an updated packaged build, launch that build from the same PowerShell
   session instead. An older executable will not contain these changes.
   This environment variable applies to this process tree; it is not a machine setting.

3. Select **Engine Acceleration 1.f.(1)** and review the existing Manual settings.
   Select Manual / Set Conditions once. This will unpause the aircraft during
   preparation. Keep the usual hands-off BRUNNER procedure; supervise the simulator.
4. Observe **Recovering airflow** and current/target IAS, MSL altitude and local
   wind. Preparation is not recording. Do not request recording until
   **Ready for Manual Recording** is displayed and the simulator is paused.
5. If preparation fails, inspect the measured reason. Recording stays blocked.
   Confirm the simulator is actually paused. If pause confirmation failed or
   the connection was lost, the application cannot guarantee a safe simulator
   state; use the operator's simulator controls to secure it.
6. Only after successful preparation, explicitly click Start Recording for the
   supervised Manual experiment. Confirm the first recorded sample has recovered
   airflow. Recording-start remains recording-start; no pre-event substitution occurs.

## Disable again

Stop the experiment and close QTG Studio and its worker. In the same PowerShell:

```powershell
Remove-Item Env:QTG_ACCELERATION_MANUAL_AIRFLOW_PREPARATION -ErrorAction SilentlyContinue
python main.py
```

Closing that PowerShell session also removes its process-local opt-in. Do not
use `setx` or edit production configuration files to enable this experiment.

## Acceptance and limitations

- IAS must remain within ±2 kt and MSL altitude within ±25 ft of the selected
  Manual settings. Local wind uses the existing ±max(0.5 kt, 10% target) criterion;
  altimeter offset uses the existing 10 ft criterion.
- Three consecutive matching, advancing bridge frames are required after at least
  two seconds; recovery times out at 60 seconds. Native acquisition timing remains
  unverified even when bridge packet sequences advance.
- Cancellation, missing frames, mismatches and timeouts invalidate readiness and
  request a confirmed pause. Pause failure is reported rather than hidden.
- Preparation now checks the existing P3D Pause-event running-state interface
  before and after receiving telemetry. A confirmed pause triggers one unpause
  request per episode, with up to three seconds for acknowledgement inside the
  existing 60-second recovery limit. Readiness counting restarts after a pause.
  Unknown pause state blocks preparation; packet arrival does not establish that
  physics is advancing. State observations, packet sequences and indicated altitude
  accompany the existing recovery measurements. The final safety pause is confirmed
  separately from the last observed recovery running state.
- At 5% throttle, hands-off flight may descend or accelerate outside the permitted
  envelope before wind recovers. A failed attempt is useful evidence. Do not widen
  tolerances or retry by repeatedly teleporting; a separately validated preparation
  method may be needed.
- After readiness, this path performs no position, weather or velocity restoration.
  Applying conditions again invalidates readiness before any setting commands.
  Recording rechecks the paused state and first live sample. External simulator or
  weather changes are not intercepted; reinitialize after any such change.

No simulator or hardware experiment has been performed by the offline tests.
