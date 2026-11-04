# 1.f.(2) ? Engine deceleration

Uses the existing shared QTG execution pathway. A catalog entry does not imply a dedicated automatic maneuver is implemented.

## Definition

See [test_definition.json](test_definition.json). Conditions shown there are catalog defaults; custom GUI settings remain authoritative.

## Existing implementation

- [main.py](../../QTG_Studio_Antigravity_Single_Main/main.py)
- [qtg_common_engine.py](../../QTG_Studio_Antigravity_Single_Main/qtg_common_engine.py)
- [qtg_position_setter.py](../../QTG_Studio_Antigravity_Single_Main/qtg_position_setter.py)
- [SMK_Engine_From_Climb.py](../../QTG_Studio_Antigravity_Single_Main/SMK_Engine_From_Climb.py)
- [engine_report_v2.py](../../QTG_Studio_Antigravity_Single_Main/engine_report_v2.py)
- [engine_rules_v2.py](../../QTG_Studio_Antigravity_Single_Main/engine_rules_v2.py)
- [engine_preparation_v15.py](../../QTG_Studio_Antigravity_Single_Main/engine_preparation_v15.py)
- [throttle_timeline.py](../../QTG_Studio_Antigravity_Single_Main/throttle_timeline.py)
- [SMK_Engine_TiTt_V2_1_3_Addon.py](../../QTG_Studio_Antigravity_Single_Main/SMK_Engine_TiTt_V2_1_3_Addon.py)
- [plot_graphs.py](../../QTG_Studio_Antigravity_Single_Main/plot_graphs.py)

## Shared interfaces

- [Initialization and recording](../../brunner/Super_Mushak_Logger_GUI.py)
- [Hardware interface](../../brunner/src/brunner_interface.py)
- [Simulator interface](../../brunner/src/p3d_simconnect_v15.py)
- [Shared report generator](../../brunner/src/plot_graphs.py)

## Results

Current runtime output: `D:\QTG_Test_Results\1_f_2_Engine_deceleration`. Historical files remain at their recorded locations.

[Reference page](../../QTG_Studio_Antigravity_Single_Main/easa_references/page_61.png)
