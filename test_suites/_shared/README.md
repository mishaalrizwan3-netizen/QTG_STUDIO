# Shared execution code

All test folders reference the existing common implementation; no hardware module is duplicated per test.

- [GUI and test settings](../../QTG_Studio_Antigravity_Single_Main/main.py)
- [Execution dispatcher](../../QTG_Studio_Antigravity_Single_Main/qtg_common_engine.py)
- [Initialization and recorder](../../brunner/Super_Mushak_Logger_GUI.py)
- [Hardware transport](../../brunner/src/brunner_interface.py)
- [Simulator transport](../../brunner/src/p3d_simconnect_v15.py)
- [Replay support](../../brunner/src/replay_support.py)
- [Shared regression tests](../../QTG_Studio_Antigravity_Single_Main/tests/)
- [BRUNNER regression tests](../../brunner/tests/)

Source paths are retained because dynamic loading, standalone launchers and packaging depend on them. Runtime code was not moved or combined.
