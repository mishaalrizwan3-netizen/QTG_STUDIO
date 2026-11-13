"""Control-method ownership policy used before starting a run."""


def validate_method(reference, method):
    if method not in ("replay", "pid"):
        raise ValueError("Unknown Automatic control method")
    if method == "pid" and reference != "1.c.(1)":
        raise ValueError("PID is supported only for normal climb.")
    return method
