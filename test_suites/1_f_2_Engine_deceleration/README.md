# 1.f.(2) — Engine deceleration

Official EASA CS-FSTD(A) Issue 2 performance validation test for piston engine throttle chop response.

## Initial Conditions (Default)

| Parameter | Default Value | Notes |
| :--- | :--- | :--- |
| **Flight Condition** | Ground | Ground run / static trim |
| **Altitude MSL** | 2000 ft | |
| **Indicated Airspeed (IAS)** | 80 kts | |
| **Flaps** | 0 | |
| **Throttle** | 100% | Full power before chop |
| **Duration** | 21 seconds | 5.0 s baseline + 16.0 s response |

> [!NOTE]
> These are the default initial conditions. They can only be modified via the password-protected **Test Settings** side panel in the Studio interface.

## Official 5-Page Report Format

All engine deceleration runs generate the standard 5-page QTG report:
- **Page 1**: Official Document Header (Doc Number `161310G00000-00`, Rev `008`), Test Date/Time, and **Initial Recorded Conditions** table (Engine Parameters, Ground Parameters, Environment Parameters, Sim Systems).
- **Page 2 (Group 1)**: Measured Ground Speed (kt).
- **Page 3 (Group 2)**: Throttle (%), Engine Speed (RPM), and Propeller Speed (RPM).
- **Page 4 (Group 3)**: Engine Power Parameters — Manifold Pressure (inHg) and Fuel Flow (lb/h).
- **Page 5**: **$T_t$ Response Time Assessment** (EASA CS-FSTD(A) Issue 2, 1.f supplemental engineering timing analysis with $90\%$ $T_t$ tolerance comparison table and throttle-onset overlay graphs).

## Implementation Modules

- [main.py](../../QTG_Studio_Antigravity_Single_Main/main.py)
- [test_definition.json](test_definition.json)
- [execution.py](execution.py)
- [plot_graphs.py](../../QTG_Studio_Antigravity_Single_Main/plot_graphs.py)
- [engine_report_v2.py](../../QTG_Studio_Antigravity_Single_Main/engine_report_v2.py)
- [engine_rules_v2.py](../../QTG_Studio_Antigravity_Single_Main/engine_rules_v2.py)
- [SMK_Engine_TiTt_V2_1_3_Addon.py](../../QTG_Studio_Antigravity_Single_Main/SMK_Engine_TiTt_V2_1_3_Addon.py)

## Results Directory

Runtime output: `D:\QTG_Test_Results\1_f_2_Engine_deceleration`.
