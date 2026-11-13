"""Typed contracts for the existing GUI/worker JSON boundary.

The runtime still serializes plain dictionaries for compatibility with old
recordings and the packaged worker. These types document their shape.
"""

from typing import Any, Literal, NotRequired, TypedDict


class InitialConditions(TypedDict):
    alt_ft: float
    ias_kt: float
    lat: float
    lon: float
    heading_deg: float
    pitch_deg: float
    bank_deg: float
    flaps: float
    throttle: float
    temperature_c: float
    on_ground: bool


class RunContext(TypedDict):
    test_reference: str
    title: str
    mode: Literal["manual", "auto"]
    control_method: Literal["replay", "pid"]
    duration_s: int
    initial_conditions: InitialConditions | None
    selected_graphs: list[str] | None
    reference_csv: str | None
    command_directory: str
    status_file: str
    stop_file: str
    results_root: str
    result_directory: str
    pid_conditions: NotRequired[dict[str, Any] | None]
    validation_csv: NotRequired[str | None]
