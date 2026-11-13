# 1.c.(1) ? Normal climb -- all engines operating

Uses the existing shared QTG execution pathway. A catalog entry does not imply a dedicated automatic maneuver is implemented.

## Definition

See [definition.json](definition.json). Conditions shown there are catalog defaults; custom GUI settings remain authoritative.

## Existing implementation

- [main.py](../../main.py)
- [qtg_common_engine.py](../../qtg_common_engine.py)
- [qtg_position_setter.py](../../qtg_position_setter.py)
- [qtg_climb_pid.py](../../qtg_climb_pid.py)
- [tests/test_climb_pid.py](../../tests/test_climb_pid.py)

## Shared interfaces

- [Initialization and recording](../../interfaces/brunner_base/Super_Mushak_Logger_GUI.py)
- [Hardware interface](../../interfaces/brunner_base/src/brunner_interface.py)
- [Simulator interface](../../interfaces/brunner_base/src/p3d_simconnect_v15.py)
- [Shared report generator](../../interfaces/brunner_base/src/plot_graphs.py)

## Results

Current runtime output: `D:\QTG_Test_Results\1_c_1_Normal_climb_all_engines_operating`. Historical files remain at their recorded locations.

[Reference page](../../easa_references/page_58.png)

[Climb report previews](report_previews/)
