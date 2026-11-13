"""Access the unmodified BRUNNER base through the established worker loader."""


def load_base():
    from qtg_common_engine import load_base as existing_load_base

    return existing_load_base()
