"""Engine-only diagnostic PDF: no Pitch/Roll/Yaw or physical CLS plots.

Original climb logger is intentionally unchanged. The engine app delegates
its failed-start PDF here while retaining raw diagnostic CSV/telemetry.
"""
import math
from pathlib import Path


def plot_engine_failed_start(app, timestamp, reason):
    rows = list(getattr(app, 'p3d_state_diag_rows', []) or [])
    if not rows:
        return None
    try:
        from matplotlib.figure import Figure
        from matplotlib.backends.backend_agg import FigureCanvasAgg
        from matplotlib.backends.backend_pdf import PdfPages

        def finite(v):
            try:
                n = float(v)
                return n if math.isfinite(n) else float('nan')
            except (TypeError, ValueError, OverflowError):
                return float('nan')

        t0 = finite(rows[0].get('diag_elapsed_s'))
        times = [finite(r.get('diag_elapsed_s')) - t0 for r in rows]
        groups = [
            ('Air-relative state', 'deg / kt', [
                ('Sideslip', 'p3d_sideslip', 'ref_sideslip_deg'),
                ('AoA', 'p3d_aoa', 'ref_aoa_deg'),
                ('IAS', 'p3d_ias_kts', None),
            ]),
            ('Angular rates', 'deg/s', [
                ('Roll rate', 'p3d_roll_rate_deg_s', 'ref_roll_rate_deg_s'),
                ('Yaw rate', 'p3d_yaw_rate_deg_s', 'ref_yaw_rate_deg_s'),
            ]),
            ('Engine', 'rpm', [
                ('Engine RPM', 'p3d_engine_rpm_1', 'ref_engine_rpm_1'),
            ]),
            ('Actual wind', 'kt', [
                ('Wind speed', 'p3d_actual_wind_speed_kt', 'ref_actual_wind_speed_kt'),
            ]),
        ]
        path = Path(app.data_dir) / f'Super_Mushak_FAILED_AUTO_START_{timestamp}_graphs.pdf'
        with PdfPages(path) as pdf:
            for page in range(0, len(groups), 2):
                fig = Figure(figsize=(11.7, 8.3), constrained_layout=True)
                FigureCanvasAgg(fig)
                axes = fig.subplots(2, 1)
                fig.suptitle('BLOCKED ENGINE START - measured conditions; no Automatic replay', fontsize=12)
                for ax, (label, ylabel, traces) in zip(axes, groups[page:page+2]):
                    for name, actual_key, ref_key in traces:
                        vals = [finite(r.get(actual_key)) for r in rows]
                        if any(math.isfinite(v) for v in vals):
                            line, = ax.plot(times, vals, label=name, linewidth=1.5)
                            if ref_key:
                                ref = [finite(r.get(ref_key)) for r in rows]
                                if any(math.isfinite(v) for v in ref):
                                    ax.plot(times, ref, '--', color=line.get_color(), alpha=.7,
                                            label=name + ' reference')
                    ax.set(title=label, xlabel='Seconds since first startup frame', ylabel=ylabel)
                    ax.grid(alpha=.3)
                    if ax.lines:
                        ax.legend(loc='best', fontsize=8)
                pdf.savefig(fig)
        print(f'[ENGINE FAILED START] saved {path} ({len(rows)} samples)', flush=True)
        return path
    except Exception as exc:
        print(f'[ENGINE FAILED START] no PDF; diagnostic CSV retained: {exc}', flush=True)
        return None
