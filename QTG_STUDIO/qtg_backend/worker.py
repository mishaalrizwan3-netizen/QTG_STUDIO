"""Entry point for the hidden QTG execution process."""


def run(context_path):
    from backend.runner import run as run_engine

    return run_engine(context_path)
