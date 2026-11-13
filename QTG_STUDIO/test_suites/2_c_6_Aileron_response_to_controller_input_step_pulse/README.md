# 2.c.(6) ? Aileron response to controller input (step/pulse)

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

Current runtime output: `D:\QTG_Test_Results\2_c_6_Aileron_response_to_controller_input_step_pulse`. Historical files remain at their recorded locations.

[Reference page](../../QTG_Studio_Antigravity_Single_Main/easa_references/page_77.png)
