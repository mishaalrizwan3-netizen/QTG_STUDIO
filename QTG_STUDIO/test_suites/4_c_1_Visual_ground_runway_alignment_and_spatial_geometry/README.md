# 4.c.(1) ? Visual ground / runway alignment and spatial geometry

Uses the existing shared QTG execution pathway. A catalog entry does not imply a dedicated automatic maneuver is implemented.

## Definition

See [test_definition.json](test_definition.json). Conditions shown there are catalog defaults; custom GUI settings remain authoritative.

## Existing implementation

- [main.py](../../QTG_Studio_Antigravity_Single_Main/main.py)
- [qtg_common_engine.py](../../QTG_Studio_Antigravity_Single_Main/qtg_common_engine.py)
- [qtg_position_setter.py](../../QTG_Studio_Antigravity_Single_Main/qtg_position_setter.py)

## Shared interfaces

- [Initialization and recording](../../brunner/Super_Mushak_Logger_GUI.py)
- [Hardware interface](../../brunner/src/brunner_interface.py)
- [Simulator interface](../../brunner/src/p3d_simconnect_v15.py)
- [Shared report generator](../../brunner/src/plot_graphs.py)

## Results

Current runtime output: `D:\QTG_Test_Results\4_c_1_Visual_ground_runway_alignment_and_spatial_geometry`. Historical files remain at their recorded locations.

[Reference page](../../QTG_Studio_Antigravity_Single_Main/easa_references/page_90.png)
