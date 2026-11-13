"""Engine renderer access with unchanged PDF implementation."""


def plot_aligned_comparison(*args, **kwargs):
    from interfaces.brunner import load_base

    return load_base().plot_graphs.plot_aligned_comparison(*args, **kwargs)
