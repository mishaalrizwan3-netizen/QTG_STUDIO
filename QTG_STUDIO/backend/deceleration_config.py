"""Deceleration's on-disk inputs, loaded afresh for each initialization."""

import json
import hashlib
from pathlib import Path

from backend.registry import suite_root
from backend.verifier import validate_conditions


REFERENCE = "1.f.(2)"
REQUIRED = (
    "flight_condition", "alt_ft", "ias_kt", "flaps", "lat", "lon",
    "heading_deg", "pitch_deg", "bank_deg", "throttle", "temperature_c", "on_ground",
)
DISPLAY_KEYS = {
    "flight_condition": "Flt cond", "alt_ft": "Alt[ft]", "ias_kt": "IAS[kts]",
    "flaps": "Flaps", "lat": "Latitude[deg]", "lon": "Longitude[deg]",
    "heading_deg": "Heading[deg]", "pitch_deg": "Pitch[deg]",
    "bank_deg": "Bank[deg]", "throttle": "Throttle[%]",
    "temperature_c": "Temperature[C]",
}


def _folder():
    return suite_root() / "engine_deceleration"


def _read(name):
    document = json.loads((_folder() / name).read_text(encoding="utf-8"))
    if document.get("test_reference") != REFERENCE:
        raise ValueError(f"Deceleration {name} has the wrong test reference.")
    return document


def load_initialization():
    document = _read("initialization.json")
    missing = set(REQUIRED) - document.keys()
    if missing:
        raise ValueError("Deceleration initialization is missing: " + ", ".join(sorted(missing)))
    conditions = validate_conditions({key: document[key] for key in REQUIRED if key != "flight_condition"})
    if ("ground" in str(document["flight_condition"]).lower()) != conditions["on_ground"]:
        raise ValueError("Deceleration flight condition and on_ground disagree.")
    return conditions


def display_defaults():
    document = _read("initialization.json")
    load_initialization()
    return {label: (str(int(document[key])) if isinstance(document[key], (int, float))
                    and float(document[key]).is_integer() else str(document[key]))
            for key, label in DISPLAY_KEYS.items()}


def load_input():
    document = _read("input.json")
    if document.get("initialization_file") != "initialization.json":
        raise ValueError("Deceleration input must use initialization.json.")
    return document


def load_output():
    return _read("output.json")


def resolve_initial_conditions(overrides=None):
    """Use current file contents, then apply only explicit Test settings changes."""
    conditions = load_initialization()
    conditions.update(overrides or {})
    return validate_conditions(conditions)


def resolve_reviewed_configuration(context):
    """Read-only review snapshot using the unchanged existing resolver."""
    path = _folder() / 'initialization.json'
    before = path.read_bytes()
    raw = json.loads(before.decode('utf-8-sig'))
    if type(raw.get('on_ground')) is not bool:
        raise ValueError('on_ground: configured target must be a boolean')
    conditions = resolve_initial_conditions(context.get('initial_condition_overrides'))
    from backend.initialization_requirements import FIELDS
    for key in FIELDS:
        if key not in conditions and key in raw:
            conditions[key] = raw[key]
    if path.read_bytes() != before:
        raise ValueError('Deceleration configuration changed during resolution')
    origins = {key: ('GUI override' if key in (context.get('initial_condition_overrides') or {})
                    else 'test_suites/engine_deceleration/initialization.json') for key in conditions}
    return conditions, hashlib.sha256(before).hexdigest(), origins


def display_overrides(values, original_defaults):
    """Translate changed GUI fields to the backend contract."""
    result = {}
    for key, label in DISPLAY_KEYS.items():
        if key == "flight_condition":
            if str(values[label]).strip() != str(original_defaults[label]).strip():
                result["on_ground"] = "ground" in str(values[label]).lower()
        elif str(values[label]).strip() != str(original_defaults[label]).strip():
            result[key] = values[label]
    return result


def write_output_manifest(context, outputs):
    """Index the measured artifacts of one completed run; never synthesize measurements."""
    template = load_output()
    csv_files = [Path(path) for path in outputs if str(path).lower().endswith(".csv")]
    if not csv_files:
        return None
    target = csv_files[0].with_name(csv_files[0].stem + "_output.json")
    payload = {
        "test_reference": REFERENCE,
        "mode": context["mode"],
        "initial_conditions": context["initial_conditions"],
        "initialization_file": str(_folder() / "initialization.json"),
        "manual_reference_csv": context.get("reference_csv"),
        "duration_seconds": context["duration_s"],
        "measured_channels": template["measured_channels"],
        "artifacts": [str(path) for path in outputs],
    }
    target.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    return target
