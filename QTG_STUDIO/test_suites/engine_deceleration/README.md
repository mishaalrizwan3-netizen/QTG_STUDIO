# 1.f.(2) — Engine deceleration

Official EASA CS-FSTD(A) Issue 2 performance validation test for piston engine throttle chop response.

## Initial Conditions (Default)

| Parameter | Default Value | Notes |
| :--- | :--- | :--- |
| **Flight Condition** | Ground | Ground run / static trim on runway |
| **Altitude MSL** | 755 ft | Runway ground elevation |
| **Indicated Airspeed (IAS)** | 0 kts | Stationary before throttle chop |
| **Flaps** | 0 | |
| **Throttle** | 100% | Full power before chop |
| **Duration** | 21 seconds | 5.0 s baseline + 16.0 s response |

> [!NOTE]
> These are the default initial conditions. They can only be modified via the password-protected **Test Settings** side panel in the Studio interface.

`initialization.json` is the single source of default altitude, IAS, flaps, runway latitude/longitude, heading, pitch, bank, throttle, temperature, and ground state. The backend reads it for every Manual or Automatic initialization. Test Settings supplies only fields the operator changed. Automatic replay still checks the chosen Manual CSV and its initial-condition sidecar before starting.

`input.json` defines the 21-second replay input contract. Each run also saves its resolved input context as a `*_context.json` file under `D:\QTG_Test_Results\sessions`. `output.json` describes the measured artifacts; each completed Deceleration run writes a `*_output.json` manifest alongside its measured CSV and PDF in the Deceleration results folder.

## Official 5-Page Report Format

All engine deceleration runs generate the standard 5-page QTG report:
- **Page 1**: Official Document Header (Doc Number `161310G00000-00`, Rev `008`), Test Date/Time, and **Initial Recorded Conditions** table (Engine Parameters, Ground Parameters, Environment Parameters, Sim Systems).
- **Page 2 (Group 1)**: Measured Ground Speed (kt).
- **Page 3 (Group 2)**: Throttle (%), Engine Speed (RPM), and Propeller Speed (RPM).
- **Page 4 (Group 3)**: Engine Power Parameters — Manifold Pressure (inHg) and Fuel Flow (lb/h).
- **Page 5**: **$T_t$ Response Time Assessment** (EASA CS-FSTD(A) Issue 2, 1.f supplemental engineering timing analysis with $90\%$ $T_t$ tolerance comparison table and throttle-onset overlay graphs).

## Implementation Modules

- [main.py](../../main.py)
- [definition.json](definition.json)
- [initialization.json](initialization.json)
- [input.json](input.json)
- [output.json](output.json)
- [procedure.py](procedure.py)
- [plot_graphs.py](../../plot_graphs.py)
- [engine_report_v2.py](../../engine_report_v2.py)
- [engine_rules_v2.py](../../engine_rules_v2.py)
- [SMK_Engine_TiTt_V2_1_3_Addon.py](../../SMK_Engine_TiTt_V2_1_3_Addon.py)

## Results Directory

Runtime output: `D:\QTG_Test_Results\1_f_2_Engine_deceleration`.
