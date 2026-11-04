# Operator-controlled PID climb

Only Normal climb (1.c.(1)) supports PID Automatic. No EXE rebuild is required when running the source app.

1. Choose PID Automatic and click Automatic. The aircraft is placed at the configured initial conditions (defaults 3000 ft and 80 kt), with bank and heading zero, and left paused.
2. The original V4.1 PID interface opens with its original IAS controls, gains and graphs. Its independent hardware/native worker is not started; commands go through the common QTG engine.
3. Unpause the simulator yourself and click START AIRSPEED HOLD in the PID interface. Adjust the original IAS/gain controls as needed to maintain 80 kt.
4. Click Start Recording in QTG Studio when the airspeed is maintained. Recording requires active PID, an unpaused simulator and IAS within 3 kt of target. No automatic start or five-second gate remains.
5. PID continues while the selected test duration is recorded. Stop pauses the simulator and releases the hardware; closing an active PID interface also stops PID.

There is no paused physical-motion probe. The existing CLS2sim physical position forwarding is enabled when you engage PID, and actual controller positions, forces and aircraft response are read continuously. Missing mandatory pitch feedback or failed commands stop the controller. Absent optional axes are recorded as unavailable. No continuous simulator attitude or flight-dynamic writes are introduced.

Validation CSV is independent of Manual replay recordings and needs Time (s), Indicated Airspeed (kts), Rate of Climb/Descent (fpm), with finite values and increasing times covering the test. Missing validation is NOT EVALUATED. Results remain under D:/QTG_Test_Results in the named climb folder. Recorded Replay is unchanged.
