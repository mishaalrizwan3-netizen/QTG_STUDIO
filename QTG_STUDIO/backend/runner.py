"""Backend execution entry, preserving the established worker implementation."""


def run(context_path):
    from qtg_common_engine import run as run_existing_worker

    return run_existing_worker(context_path)


def worker_class(base, context, publish):
    from qtg_common_engine import worker_class as existing_worker_class

    return existing_worker_class(base, context, publish)
