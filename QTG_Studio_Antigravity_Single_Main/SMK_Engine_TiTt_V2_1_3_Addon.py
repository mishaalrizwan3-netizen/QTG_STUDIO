#!/usr/bin/env python3
"""Super Mushshak V2.1.3 engine-test Ti/Tt post-processor (no simulator writes).

EASA CS-FSTD(A) Issue 2, Objective tests 1.f(1)/1.f(2).
Ti = first sustained 10% critical engine parameter response since throttle onset.
Tt = first sustained 90% critical engine parameter response since throttle onset.
This analyses exported CSVs; does NOT alter measured/replayed throttle timing.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Optional


def median(a):
    return statistics.median(a)


ALIASES = {
    'time': ['time_s', 'time (s)', 'time_sec', 'elapsed_time_s', 'elapsed_s', 'elapsed time (s)',
             'elapsed time', 'time', 't', 'simtime', 'simulation time', 'timestamp_s'],
    'throttle': ['throttle_pct', 'throttle (%)', 'throttle_percent', 'throttle percent',
                 'throttle 1 position (%)', 'throttle 1 position', 'throttle_1_position',
                 'throttle', 'general eng throttle lever position:1', 'eng throttle lever position',
                 'throttle lever position', 'throttle position', 'throttle_input'],
    'map': ['map_inhg', 'map', 'manifold_pressure_inhg', 'manifold pressure', 'manifold_inhg',
            'general eng manifold pressure:1', 'eng manifold pressure', 'manifold pressure (inhg)',
            'engine_manifold_pressure_psi_1'],
    'power': ['engine_power_hp', 'shaft_power_hp', 'brake_horsepower', 'power_hp',
              'horsepower', 'engine power', 'power', 'general eng power available:1'],
    'rpm': ['engine_rpm', 'eng_rpm', 'rpm', 'engine 1 rpm', 'engine_1_rpm', 'general eng rpm:1',
            'engine rpm', 'prop_rpm', 'propeller 1 rpm', 'propeller rpm'],
}


def norm_key(x: str) -> str:
    return ''.join(ch for ch in str(x).casefold() if ch.isalnum())


def find_column(names, kind: str, explicit: Optional[str]):
    if explicit:
        for n in names:
            if norm_key(n) == norm_key(explicit):
                return n
        raise ValueError(f"Cannot find requested {kind} column {explicit!r}; available: {names}")
    byname = {norm_key(n): n for n in names}
    for possibility in ALIASES[kind]:
        if norm_key(possibility) in byname:
            return byname[norm_key(possibility)]
    return None


def numeric(v):
    try:
        n = float(str(v).strip())
        return n if math.isfinite(n) else None
    except (ValueError, TypeError):
        return None


@dataclass
class Measurement:
    name: str
    mode: str
    parameter: str
    t_start: Optional[float]
    t_10: Optional[float]
    t_90: Optional[float]
    Ti_s: Optional[float]
    Tt_s: Optional[float]
    initial_parameter: Optional[float]
    final_parameter: Optional[float]
    throttle_initial_pct: Optional[float]
    throttle_final_pct: Optional[float]
    status: str
    warnings: list[str]
    columns: dict


def sustained_crossing(t, response, target, start_idx, hold_s=0.08):
    """First linearly interpolated crossing that persists over hold_s.

    Uses time rather than a fixed number of samples; no smoothing time lag.
    """
    for i in range(max(1, start_idx), len(t)):
        if response[i] < target:
            continue
        if response[i - 1] >= target:
            tc = t[i - 1]
        else:
            dv = response[i] - response[i - 1]
            tc = t[i - 1] + (t[i] - t[i - 1]) * ((target - response[i - 1]) / dv) if dv else t[i]
        # At least hold_s beyond threshold (allow last sample if exact endpoint)
        k = i
        while k < len(t) and t[k] - tc < hold_s:
            if response[k] < target:
                break
            k += 1
        else:
            if k < len(t) or t[-1] - tc >= hold_s:
                return tc
    return None


def parse_trace(path: Path, *, time_col=None, throttle_col=None, parameter_col=None,
                parameter_kind='map', throttle_scale=None, time_scale=1.0):
    path = Path(path)
    # FIRST use time-stamped high-rate measurements, not the 1-Hz plotted CSV.
    # The 1-Hz plot files cannot establish an accurate sub-second Ti/Tt.
    base = (path.with_name(path.stem[:-12] + '.csv')
            if path.stem.endswith('_legacy_plot') else path)
    if path.stem.endswith('_engine_channels'):
        candidates = [path]
    else:
        companion = base.with_name(base.stem + '_engine_channels.csv')
        candidates = ([companion] if companion.exists() else []) + [base]
        if path != base:
            candidates.append(path)

    last_error = None
    for cand in candidates:
        try:
            with cand.open('r', newline='', encoding='utf-8-sig') as f:
                reader = csv.DictReader(f)
                names = reader.fieldnames or []
                tc = find_column(names, 'time', time_col)
                hc = find_column(names, 'throttle', throttle_col)
                pc = find_column(names, parameter_kind, parameter_col)
                if not tc or not hc or not pc:
                    raise ValueError(f'{cand.name}: missing time/throttle/{parameter_kind} column; found {names}.')
                raw = []
                for r in reader:
                    vals = [numeric(r.get(c)) for c in (tc, hc, pc)]
                    if all(v is not None for v in vals):
                        raw.append(vals)
            if len(raw) < 8:
                raise ValueError(f'{cand.name}: at least eight complete data rows are needed')
            t = [r[0] * time_scale for r in raw]
            h = [r[1] for r in raw]
            p = [r[2] for r in raw]
            if any(b <= a for a, b in zip(t, t[1:])):
                raise ValueError(f'{cand.name}: timestamps must be strictly increasing')
            if throttle_scale is None:
                peak = max(abs(x) for x in h)
                if peak <= 1.2:
                    throttle_scale = 100.0  # SimConnect 0..1 lever position
                elif peak <= 105.0:
                    throttle_scale = 1.0    # Already percentage
                elif peak <= 17000.0:
                    throttle_scale = 100.0 / 16383.0  # P3D signed/unsigned axis magnitude
                else:
                    raise ValueError(f'{cand.name}: throttle raw range unclear; provide --throttle-scale')
            h = [x * throttle_scale for x in h]
            return t, h, p, {'time': tc, 'throttle': hc, 'critical_parameter': pc,
                              'source_file': str(cand), 'median_interval_s': statistics.median(
                                  b-a for a, b in zip(t, t[1:]))}
        except Exception as e:
            last_error = e
            continue

    raise last_error or ValueError(f'Could not parse trace from {path}')



def measure(path: Path, name: str, mode: str, *, time_col=None, throttle_col=None,
            parameter_col=None, parameter_kind='map', throttle_scale=None, time_scale=1.0,
            hold_s=0.08):
    t, h, p, cols = parse_trace(path, time_col=time_col, throttle_col=throttle_col,
                                 parameter_col=parameter_col, parameter_kind=parameter_kind,
                                 throttle_scale=throttle_scale, time_scale=time_scale)
    warnings = []
    n = len(t)
    step_max = max(b-a for a,b in zip(t,t[1:]))
    # Signal quality is an assessment gate: never mark a 1-Hz response as valid.
    too_sparse = step_max > .25 or cols['median_interval_s'] > .15
    if too_sparse:
        warnings.append(f'INSUFFICIENT TEMPORAL RESOLUTION: largest gap {step_max:.3f}s, '
                        f'median interval {cols["median_interval_s"]:.3f}s. '
                        'Use the high-rate *_engine_channels.csv export.')
    first = max(2, min(5, n // 50))
    last = max(3, min(n // 5, n // 8))
    h0 = median(h[:first]); hf = median(h[-last:])
    expected_sign = 1 if mode == 'acceleration' else -1
    throttle_change = expected_sign * (hf - h0)
    throttle_ok = throttle_change >= 20.0  # reveal poor/partial lever sweeps
    if not throttle_ok:
        warnings.append(f'Throttle sweep too small/wrong direction: {throttle_change:.2f} percentage points')
    if min(h) < -3 or max(h) > 103:
        warnings.append('Throttle normalization outside expected 0–100%; verify source units')
    threshold = max(1.0, 0.02 * abs(hf - h0))
    onset_i = next((i for i in range(1, n) if expected_sign * (h[i] - h0) >= threshold), None)
    if onset_i is None:
        warnings.append('No throttle movement onset detectable')
        return Measurement(name, mode, parameter_kind, None, None, None, None, None,
                           None, None, h0, hf, 'INVALID', warnings, cols), (t, h, p)
    # Interpolate crossing from pre-movement lever to threshold. The first trace timestamp is
    # NOT silently treated as throttle movement: measured throttle is the sole trigger.
    h_prev, h_curr = expected_sign * (h[onset_i-1] - h0), expected_sign * (h[onset_i] - h0)
    if h_curr == h_prev:
        t0 = t[onset_i]
    else:
        t0 = t[onset_i - 1] + (t[onset_i] - t[onset_i-1]) * (threshold - h_prev) / (h_curr-h_prev)
    power_first = [p[i] for i in range(onset_i) if t[i] <= t0]
    if len(power_first) < 1:
        power_first = p[:first]
        warnings.append('No clearly measured pre-onset engine baseline; check recording pre-roll')
    pi = median(power_first)
    pf = median(p[-last:])
    total = expected_sign * (pf - pi)
    scale = max(1.0, abs(pi), abs(pf))
    if total <= 0.01 * scale:
        warnings.append(f'Critical parameter total response is negligible or reversed (change {total:.5g})')
        return Measurement(name, mode, parameter_kind, t0, None, None, None, None,
                           pi, pf, h0, hf, 'INVALID', warnings, cols), (t, h, p)
    # Look for high transient activity in final 12.5% (unsettled endpoint biases both thresholds).
    final_spread = max(p[-last:]) - min(p[-last:])
    if final_spread > 0.10 * total:
        warnings.append('Final engine parameter not settled: endpoint/90% time is provisional')
    before_noise = (max(power_first) - min(power_first)) if len(power_first) > 1 else 0.0
    if before_noise > 0.05 * total:
        warnings.append('Unsteady engine parameter before throttle onset; verify starting condition')
    resp = [expected_sign * (x - pi) / total for x in p]
    t10 = sustained_crossing(t, resp, 0.10, onset_i, hold_s)
    t90 = sustained_crossing(t, resp, 0.90, onset_i, hold_s)
    if t10 is None or t90 is None:
        warnings.append('Unable to identify sustained 10% and/or 90% response crossings')
    if t10 is not None and t10 < t0 - 1e-3:
        warnings.append('Engine response precedes throttle onset: clock/data alignment problem')
    ti = max(0.0, t10-t0) if t10 is not None else None
    tt = max(0.0, t90-t0) if t90 is not None else None
    if ti is not None and tt is not None and tt < ti:
        warnings.append('90% response precedes 10% response: invalid signal')
    hard_fail = (too_sparse or (not throttle_ok) or t10 is None or t90 is None
                 or (tt is not None and ti is not None and tt < ti))
    status = 'INVALID' if hard_fail else 'PROVISIONAL' if warnings else 'MEASURED'
    return Measurement(name, mode, parameter_kind, t0, t10, t90, ti, tt, pi, pf,
                       h0, hf, status, warnings, cols), (t, h, p)


def compare(candidate: Measurement, reference: Measurement | dict, label: str, *, formal=False):
    ti_ref = reference.Ti_s if isinstance(reference, Measurement) else reference.get('Ti_s')
    tt_ref = reference.Tt_s if isinstance(reference, Measurement) else reference.get('Tt_s')
    ti = candidate.Ti_s; tt = candidate.Tt_s
    metrics = {}
    for key, observed, target, allowance in [
        ('Ti', ti, ti_ref, max(0.10 * abs(ti_ref), .25) if ti_ref is not None else None),
        ('Tt', tt, tt_ref, 0.10 * abs(tt_ref) if tt_ref is not None else None)]:
        if observed is None or target is None or allowance is None:
            metrics[key] = {'status': 'NOT_ASSESSED'}
        else:
            delta = observed-target
            metrics[key] = {'observed_s': observed, 'reference_s': target,
                            'difference_s': delta, 'allowed_s': allowance,
                            'status': 'WITHIN_TOLERANCE' if abs(delta) <= allowance+1e-9 else 'OUTSIDE_TOLERANCE'}
    valid = candidate.status != 'INVALID' and (not isinstance(reference, Measurement) or reference.status != 'INVALID')
    status = ('INVALID' if not valid else
              'NOT_ASSESSED' if any(m['status'] == 'NOT_ASSESSED' for m in metrics.values()) else
              'WITHIN_TOLERANCE' if all(m['status'] == 'WITHIN_TOLERANCE' for m in metrics.values()) else
              'OUTSIDE_TOLERANCE')
    return {'label': label, 'assessment_type': 'aircraft_reference' if formal else 'manual_vs_automatic_engineering',
            'metrics': metrics, 'status': status}


def plot_overlay(results, traces, output: Path, parameter: str):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        return 'matplotlib unavailable: PNG graph omitted'
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    for name, m in results.items():
        t, h, p = traces[name]
        if m.t_start is None:
            continue
        x = [v - m.t_start for v in t]
        ax1.plot(x, h, label=name)
        ax2.plot(x, p, label=name)
        if m.t_10 is not None:
            ax2.axvline(m.Ti_s, alpha=.6, linestyle='--', linewidth=1.0)
        if m.t_90 is not None:
            ax2.axvline(m.Tt_s, alpha=.6, linestyle=':', linewidth=1.3)
        if m.initial_parameter is not None and m.final_parameter is not None:
            dp = m.final_parameter - m.initial_parameter
            ax2.plot([m.Ti_s] if m.Ti_s is not None else [],
                     [m.initial_parameter+.1*dp] if m.Ti_s is not None else [], 'o')
            ax2.plot([m.Tt_s] if m.Tt_s is not None else [],
                     [m.initial_parameter+.9*dp] if m.Tt_s is not None else [], 's')
    ax1.set_ylabel('Throttle (%)'); ax1.set_title('Throttle onset aligned to t = 0')
    ax2.set_ylabel(parameter); ax2.set_xlabel('Time since measured throttle onset (s)')
    ax2.set_title('Critical engine parameter: circles=Ti (10%), squares=Tt (90%)')
    ax1.grid(True, alpha=.2); ax2.grid(True, alpha=.2)
    ax1.legend(); ax2.legend()
    fig.tight_layout()
    fig.savefig(output, dpi=160)
    plt.close(fig)
    return None


def plot_titt_page(fig, measurements, traces, mode, parameter_name, comparison=None):
    """Draw a dedicated Ti/Tt Assessment Page on a matplotlib figure for PDF reporting."""
    fig.clear()
    fig.suptitle(f"Super Mushshak Engine {str(mode).capitalize()} - Ti / Tt Response Analysis\n"
                 f"Engineering timing check - CS-FSTD(A) Issue 2, 1.f method",
                 fontsize=13, fontweight='bold', y=0.97)

    gs = fig.add_gridspec(3, 1, height_ratios=[1.1, 1.2, 1.5], top=0.91, bottom=0.07, hspace=0.38,
                          left=0.12, right=0.94)

    ax_info = fig.add_subplot(gs[0])
    ax_info.axis('off')

    table_data = [
        ['Metric', 'Manual', 'Automatic', 'Difference', 'Compare band', 'Result']
    ]

    m_man = measurements.get('Manual')
    m_auto = measurements.get('Automatic')

    def fmt_s(val):
        return f"{val:.3f} s" if val is not None else "N/A"

    ti_man = m_man.Ti_s if m_man else None
    ti_auto = m_auto.Ti_s if m_auto else None
    tt_man = m_man.Tt_s if m_man else None
    tt_auto = m_auto.Tt_s if m_auto else None

    ti_diff = (ti_auto - ti_man) if (ti_auto is not None and ti_man is not None) else None
    ti_allow = max(0.10 * abs(ti_man), 0.25) if ti_man is not None else 0.25
    ti_status = "WITHIN_TOLERANCE" if (ti_diff is not None and abs(ti_diff) <= ti_allow + 1e-9) else ("N/A" if ti_diff is None else "OUTSIDE_TOLERANCE")

    tt_diff = (tt_auto - tt_man) if (tt_auto is not None and tt_man is not None) else None
    tt_allow = 0.10 * abs(tt_man) if tt_man is not None else 0.50
    tt_status = "WITHIN_TOLERANCE" if (tt_diff is not None and abs(tt_diff) <= tt_allow + 1e-9) else ("N/A" if tt_diff is None else "OUTSIDE_TOLERANCE")

    if comparison:
        assessment = comparison[0]
        overall_status = assessment['status']
        ti_status = assessment['metrics']['Ti']['status']
        tt_status = assessment['metrics']['Tt']['status']
    elif m_man and not m_auto:
        overall_status = m_man.status
        ti_status = tt_status = "MANUAL_MEASUREMENT"
    else:
        overall_status = "NOT_ASSESSED"
    if any(m.status == 'INVALID' for m in measurements.values()):
        overall_status = 'INVALID'

    table_data.append([
        "Ti (10% response)",
        fmt_s(ti_man), fmt_s(ti_auto),
        f"{ti_diff:+.3f} s" if ti_diff is not None else "N/A",
        f"+/-{ti_allow:.2f} s",
        ti_status
    ])
    table_data.append([
        "Tt (90% response)",
        fmt_s(tt_man), fmt_s(tt_auto),
        f"{tt_diff:+.3f} s" if tt_diff is not None else "N/A",
        f"+/-{tt_allow:.2f} s",
        tt_status
    ])

    result_labels = {'WITHIN_TOLERANCE': 'Within', 'OUTSIDE_TOLERANCE': 'Outside',
                     'MANUAL_MEASUREMENT': 'Manual only', 'NOT_ASSESSED': 'N/A',
                     'INVALID': 'Invalid', 'PROVISIONAL': 'Provisional'}
    for row in table_data[1:]:
        row[-1] = result_labels.get(row[-1], row[-1])
    tb = ax_info.table(cellText=table_data, loc='center', cellLoc='center',
                       colWidths=[.21, .14, .14, .14, .19, .18],
                       bbox=[0.0, 0.22, 1.0, 0.70])
    tb.auto_set_font_size(False)
    tb.set_fontsize(7.9)
    for (row, col), cell in tb.get_celld().items():
        if row == 0:
            cell.set_facecolor('#dce4ec')
            cell.set_text_props(weight='bold')
        elif col == 5:
            txt = cell.get_text().get_text()
            if txt == 'Within':
                cell.set_facecolor('#d4edda')
                cell.set_text_props(color='#155724', weight='bold')
            elif txt == 'Outside':
                cell.set_facecolor('#f8d7da')
                cell.set_text_props(color='#721c24', weight='bold')

    status_color = '#155724' if overall_status in ('WITHIN_TOLERANCE', 'MEASURED') else '#721c24'
    ax_info.text(0.0, 0.055, f"Timing/repeatability: {overall_status}", fontsize=9,
                 weight='bold', color=status_color)
    friendly_param = {'manifold_inhg': 'Manifold pressure (inHg)',
                      'engine_rpm': 'Engine RPM (rpm)', 'prop_rpm': 'Propeller RPM (rpm)'}
    ax_info.text(1.0, 0.055, f"Channel: {friendly_param.get(parameter_name, parameter_name)}",
                 fontsize=7.6, ha='right', style='italic')
    ax_info.text(0, -0.11, 'Comparison bands: Ti = max(10% of Manual Ti, 0.25 s); '
                 'Tt = 10% of Manual Tt (engineering comparison only).',
                 fontsize=7.0, color='#555555')

    ax1 = fig.add_subplot(gs[1])
    ax2 = fig.add_subplot(gs[2], sharex=ax1)

    colors = {'Manual': 'green', 'Automatic': '#0066cc'}
    styles = {'Manual': '-', 'Automatic': '--'}

    for name in ['Manual', 'Automatic']:
        if name not in measurements or name not in traces:
            continue
        m = measurements[name]
        t, h, p = traces[name]
        if m.t_start is None:
            continue
        x = [v - m.t_start for v in t]
        c = colors.get(name, 'black')
        ls = styles.get(name, '-')
        ax1.plot(x, h, label=f"{name} ({m.mode})", color=c, linestyle=ls, linewidth=1.8)
        ax2.plot(x, p, label=f"{name} (Ti={fmt_s(m.Ti_s)}, Tt={fmt_s(m.Tt_s)})", color=c, linestyle=ls, linewidth=1.8)

        if m.Ti_s is not None:
            ax2.axvline(m.Ti_s, color=c, linestyle='--', alpha=0.7, linewidth=1.2,
                        label=f"{name} Ti: {m.Ti_s:.3f}s" if name == 'Automatic' else None)
        if m.Tt_s is not None:
            ax2.axvline(m.Tt_s, color=c, linestyle=':', alpha=0.7, linewidth=1.4,
                        label=f"{name} Tt: {m.Tt_s:.3f}s" if name == 'Automatic' else None)
        if m.initial_parameter is not None and m.final_parameter is not None:
            dp = m.final_parameter - m.initial_parameter
            if m.Ti_s is not None:
                ax2.plot([m.Ti_s], [m.initial_parameter + 0.1 * dp], 'o', color=c, markersize=5)
            if m.Tt_s is not None:
                ax2.plot([m.Tt_s], [m.initial_parameter + 0.9 * dp], 's', color=c, markersize=5)

    ax1.set_ylabel('Throttle Position (%)', fontweight='bold')
    ax1.set_title('Throttle Onset Aligned to t = 0 s', fontsize=10, fontweight='bold')
    ax1.grid(True, which='major', alpha=0.45, linewidth=0.7)
    ax1.minorticks_on()
    ax1.grid(True, which='minor', alpha=0.20, linewidth=0.45)
    ax1.legend(loc='best', fontsize=8, framealpha=0.95)

    ax2.set_ylabel(parameter_name, fontweight='bold')
    ax2.set_xlabel('Time Since Measured Throttle Onset (seconds)', fontweight='bold')
    ax2.set_title(f'Engine Response ({parameter_name}): Circles = 10% (Ti), Squares = 90% (Tt)', fontsize=10, fontweight='bold')
    ax2.grid(True, which='major', alpha=0.45, linewidth=0.7)
    ax2.minorticks_on()
    ax2.grid(True, which='minor', alpha=0.20, linewidth=0.45)
    ax2.legend(loc='best', fontsize=8, framealpha=0.95)


def evaluate_engine_run(automatic_path: Path, manual_path: Optional[Path] = None,
                        mode: str = 'acceleration', parameter_kind: str = 'map',
                        out_dir: Optional[Path] = None, **kwargs) -> dict:
    """Programmatic entry point to assess Ti/Tt from logged files."""
    automatic_path = Path(automatic_path)
    mode = str(mode).lower()
    if out_dir is None:
        out_dir = automatic_path.parent
    else:
        out_dir = Path(out_dir)
        out_dir.mkdir(parents=True, exist_ok=True)

    measurements = {}
    traces = {}
    # A manual-only report must not present its measurements as 'Automatic'.
    targets = ([('Manual', Path(manual_path)), ('Automatic', automatic_path)]
               if manual_path else [('Manual', automatic_path)])

    for label, path in targets:
        try:
            m, tr = measure(path, label, mode, parameter_kind=parameter_kind, **kwargs)
            measurements[label] = m
            traces[label] = tr
        except Exception as e:
            print(f"[TiTt] Measure {label} ({path.name}) failed: {e}", flush=True)

    comparisons = []
    if 'Automatic' in measurements and 'Manual' in measurements:
        comparisons.append(compare(measurements['Automatic'], measurements['Manual'], 'Automatic vs Manual'))

    primary_param = parameter_kind
    if 'Automatic' in measurements and measurements['Automatic'].columns.get('critical_parameter'):
        primary_param = measurements['Automatic'].columns['critical_parameter']
    elif 'Manual' in measurements and measurements['Manual'].columns.get('critical_parameter'):
        primary_param = measurements['Manual'].columns['critical_parameter']

    report = {
        'software': 'SMK Engine V2.1.3 Ti/Tt supplemental assessment',
        'basis': 'EASA CS-FSTD(A) Issue 2, 1.f(1)/(2)',
        'mode': mode,
        'parameter_kind': parameter_kind,
        'critical_parameter': primary_param,
        'measurements': {k: asdict(v) for k, v in measurements.items()},
        'comparisons': comparisons,
        'status': comparisons[0]['status'] if comparisons else (
            measurements['Manual'].status if 'Manual' in measurements else 'NOT_ASSESSED'),
        'qualification_status': 'NOT_ASSESSED',
        'qualification_note': ('Manual-vs-Automatic time agreement is an engineering repeatability '
                               'check, not EASA qualification. An accepted independent aircraft '
                               'reference, matched conditions, and justified power parameter are required.'),
        'reference_type': 'recorded Manual repeatability baseline' if manual_path else 'Manual data only'

    }

    try:
        json_path = out_dir / (automatic_path.stem + '_TiTt_results.json')
        json_path.write_text(json.dumps(report, indent=2), encoding='utf-8')
        csv_path = out_dir / (automatic_path.stem + '_TiTt_summary.csv')
        with csv_path.open('w', newline='', encoding='utf-8') as f:
            w = csv.writer(f)
            w.writerow(['Run', 'Mode', 'Parameter', 'Throttle_onset_s', 'Ti_s', 'Tt_s', 'Status', 'Warnings'])
            for key, val in measurements.items():
                w.writerow([key, mode, parameter_kind, val.t_start, val.Ti_s, val.Tt_s, val.status, '; '.join(val.warnings)])
        overlay_path = out_dir / (automatic_path.stem + '_TiTt_overlay.png')
        plot_warning = plot_overlay(measurements, traces, overlay_path, primary_param)
        if plot_warning:
            report['plot_warning'] = plot_warning
        elif overlay_path.exists():
            report['overlay_path'] = str(overlay_path)
    except Exception as io_err:
        print(f"[TiTt EXPORT WARNING] {io_err}", flush=True)

    report['raw_measurements'] = measurements
    report['raw_traces'] = traces
    return report


def main(argv=None):
    ap = argparse.ArgumentParser(description='Ti/Tt supplement for SMK Engine V2.1.3 CSV output')
    ap.add_argument('--manual', type=Path, required=True)
    ap.add_argument('--automatic', type=Path, required=True)
    ap.add_argument('--mode', choices=['acceleration', 'deceleration'], required=True)
    ap.add_argument('--out', type=Path, default=Path('TiTt_Report'))
    ap.add_argument('--time-col', help='Exact logged CSV time header (if not autodetected)')
    ap.add_argument('--throttle-col', help='Exact logged CSV throttle header (if not autodetected)')
    ap.add_argument('--parameter-col', help='Exact critical engine parameter CSV header')
    ap.add_argument('--parameter-kind', choices=['map', 'power', 'rpm'], default='map',
                    help='MAP/RPM initial engineering diagnostics; power preferred when justified by aircraft data')
    ap.add_argument('--time-scale', type=float, default=1.0, help='Time unit→seconds (e.g. .001 if logged ms)')
    ap.add_argument('--throttle-scale', type=float, help='Throttle raw unit→percent (otherwise auto detected)')
    ap.add_argument('--validation-csv', type=Path, help='Independent aircraft time-history; SAME critical parameter/conditions')
    ap.add_argument('--reference-ti', type=float, help='Independent accepted aircraft Ti (seconds)')
    ap.add_argument('--reference-tt', type=float, help='Independent accepted aircraft Tt (seconds)')
    ap.add_argument('--validation-accepted', action='store_true', help='Review acknowledgement, not authority approval')
    ap.add_argument('--parameter-approved', action='store_true', help='Power parameter justified and agreed for qualification')
    ap.add_argument('--conditions-verified', action='store_true', help='Flight/ground and configuration matching verified')
    ap.add_argument('--hold-ms', type=float, default=80.0, help='Persist-crossing guard against isolated spikes')
    args = ap.parse_args(argv)
    if (args.reference_ti is None) != (args.reference_tt is None):
        ap.error('Provide both --reference-ti and --reference-tt, or neither')
    args.out.mkdir(parents=True, exist_ok=True)
    kw = dict(time_col=args.time_col, throttle_col=args.throttle_col,
              parameter_col=args.parameter_col, parameter_kind=args.parameter_kind,
              throttle_scale=args.throttle_scale, time_scale=args.time_scale,
              hold_s=args.hold_ms/1000)
    measurements = {}; traces = {}
    for key, file in [('Manual', args.manual), ('Automatic', args.automatic)]:
        measurements[key], traces[key] = measure(file, key, args.mode, **kw)
    comparisons = [compare(measurements['Automatic'], measurements['Manual'], 'Automatic vs Manual')]
    if args.validation_csv:
        measurements['Aircraft validation'], traces['Aircraft validation'] = measure(
            args.validation_csv, 'Aircraft validation', args.mode, **kw)
        comparisons.append(compare(measurements['Automatic'], measurements['Aircraft validation'],
                                   'Automatic vs independent aircraft', formal=True))
    elif args.reference_ti is not None:
        comparisons.append(compare(measurements['Automatic'],
                                   {'Ti_s': args.reference_ti, 'Tt_s': args.reference_tt},
                                   'Automatic vs independent aircraft', formal=True))
    report = {
        'software': 'SMK Engine V2.1.3 Ti/Tt supplemental assessment',
        'basis': 'EASA CS-FSTD(A) Issue 2, 1.f(1)/(2) (applicable before 30 April 2028)',
        'mode': args.mode, 'parameter_kind': args.parameter_kind,
        'critical_parameter': measurements['Manual'].columns['critical_parameter'],
        'units': 'logged units',
        'Ti_definition': 'elapsed time from first measured throttle movement to 10% response',
        'Tt_definition': 'elapsed time from first measured throttle movement to 90% response',
        'accel_decel_details': 'Responses are relative to measured start/end parameter values, not throttle percentage',
        'measurements': {k: asdict(v) for k, v in measurements.items()},
        'comparisons': comparisons,
        'formal_validation_evidence': {
            'aircraft_reference_supplied': args.validation_csv is not None or args.reference_ti is not None,
            'validation_accepted_acknowledged': args.validation_accepted,
            'parameter_approved_acknowledged': args.parameter_approved,
            'test_conditions_verified_acknowledged': args.conditions_verified,
            'formal_qualification_claim': 'NOT_CLAIMED',
        },
        'disclaimer': ('Manual vs Automatic reflects replay agreement only, not qualification. '
                       'MAP/RPM may be useful diagnostics; formal engine power parameter needs justification. '
                       'A real accepted aircraft reference and regulator/authority evaluation remain required.'),
    }
    (args.out / 'TiTt_results.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    with (args.out / 'TiTt_summary.csv').open('w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(['Run', 'Mode', 'Parameter', 'Throttle_onset_s', 'Ti_s', 'Tt_s', 'Status', 'Warnings'])
        for key, val in measurements.items():
            w.writerow([key, args.mode, args.parameter_kind, val.t_start, val.Ti_s,
                        val.Tt_s, val.status, '; '.join(val.warnings)])
    graph_warning = plot_overlay(measurements, traces, args.out / 'TiTt_overlay.png',
                                 measurements['Manual'].columns['critical_parameter'])
    if graph_warning:
        print(graph_warning)
    print('Mode:', args.mode, '| power surrogate:', args.parameter_kind)
    for key, m in measurements.items():
        def fmt(x): return f'{x:.3f}s' if x is not None else 'n/a'
        print(f' {key:20} Ti={fmt(m.Ti_s)}  Tt={fmt(m.Tt_s)}  {m.status}')
        for warning in m.warnings:
            print('    WARNING:', warning)
    for c in comparisons:
        print(' ', c['label'], c['status'], '|',
              ', '.join(f"{x}: {m['status']}" for x, m in c['metrics'].items()))
    print('Engineering assessment only. No EASA PASS claim.')
    print('Reports:', args.out.resolve())
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
