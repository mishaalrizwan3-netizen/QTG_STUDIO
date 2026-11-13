"""Stable per-test output names; existing files and folders are unchanged."""

from pathlib import Path
import re


def test_directory(root, reference, title):
    name = re.sub(r"[^A-Za-z0-9]+", "_", reference + " " + title).strip("_")
    return Path(root).resolve() / name


def named_result(filename, reference, title):
    label = test_directory(Path("."), reference, title).name
    return re.sub(
        r"^Super_Mushak_(manual|auto)_takeoff_data_",
        lambda match: label + "_" + match.group(1) + "_",
        filename,
    )
