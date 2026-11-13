"""Locate the existing test-specific execution policies."""

import importlib.util
import importlib
import json
from pathlib import Path
import sys

from backend.catalog import TESTS, TestSpec


def get_test(reference):
    return next((spec for spec in TESTS if spec.reference == reference), None)


ENGINE_FOLDERS = {
    "1.f.(1)": "engine_acceleration",
    "1.f.(2)": "engine_deceleration",
}

TEST_FOLDERS = {"1.c.(1)": "normal_climb", **ENGINE_FOLDERS}

# File-backed configuration belongs to its existing test module, not the manager.
MANUAL_CONFIGURATION_RESOLVERS = {
    '1.f.(2)': ('backend.deceleration_config', 'resolve_reviewed_configuration'),
}


def manual_configuration_resolver(reference):
    if get_test(reference) is None or reference == '1.c.(2)':
        raise ValueError('Selected test is unknown or not applicable in the current GUI')
    entry = MANUAL_CONFIGURATION_RESOLVERS.get(reference)
    return getattr(importlib.import_module(entry[0]), entry[1]) if entry else None


def suite_root():
    base = Path(sys._MEIPASS) if getattr(sys, "frozen", False) else Path(__file__).resolve().parents[1]
    return base / "test_suites"


def get_definition(reference):
    """Read catalog metadata; GUI overrides remain authoritative at runtime."""
    folder = TEST_FOLDERS.get(reference)
    if folder is None:
        return None
    return json.loads((suite_root() / folder / "definition.json").read_text(encoding="utf-8"))


def engine_test_policy(reference):
    folder = ENGINE_FOLDERS.get(reference)
    if folder is None:
        return None
    path = suite_root() / folder / "procedure.py"
    spec = importlib.util.spec_from_file_location("qtg_engine_policy", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module
