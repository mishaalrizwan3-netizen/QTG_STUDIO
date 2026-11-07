# Offline simulation mode

This mode replaces the P3D/SimConnect and Brunner/CLS2Sim adapters with local
software mocks. It is intended for UI, initialization, recording-flow and
fault-handling development only.

**SIMULATION ONLY.** Data produced in this mode is not aircraft validation
data, must not be used for QTG qualification, and does not demonstrate
Prepar3D, CLS2Sim or Brunner hardware performance.

## Enable on a source development PC

From the repository root in Windows Command Prompt:

```bat
cd QTG_Studio_Antigravity_Single_Main
set QTG_SIMULATION_MODE=1
python main.py
```

Then use QTG Studio normally. The child test process receives the explicit
simulation-mode setting through its test context. If the variable is absent
or is not exactly `1`, QTG continues to select the real hardware adapters.
A failed live connection never triggers an automatic mock fallback.

To return to live mode, close QTG Studio and start it without setting
`QTG_SIMULATION_MODE` (or run `set QTG_SIMULATION_MODE=` first).

## Offline adapter smoke tests

From the repository root:

```bat
python -m unittest tests.simulation.test_mock_adapters -v
```

The mock aircraft model is deliberately simplified. It is useful for
checking software flow and injecting faults such as ground creep; it is not
a validated Super Mushshak aerodynamic or engine model.

## Packaging status

The source-development mode is wired into the QTG child worker. A frozen
portable build must not be used for simulation mode until the packaging
configuration explicitly includes the `simulator_mock` package and the
release is checked. Keep the Sim PC's production package in live mode.
