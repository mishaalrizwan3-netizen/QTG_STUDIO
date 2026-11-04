"""Read-only analysis of saved BRUNNER commissioning CSV, no hardware access.

Usage on Windows:
    py -3 CHECK_BRUNNER_REPLAY.py "C:\\...\\Super_Mushak_Brunner_Commissioning_auto_....csv"

Shows actual motor feedback versus the native climb replay target, and checks
AP enable and CLS2Sim output state. Does not infer EASA qualification.
"""
import csv
import math
import statistics
import sys
from pathlib import Path


def number(value):
    try:
        x = float(value)
        return x if math.isfinite(x) else None
    except (ValueError, TypeError):
        return None


def percentile(values, percentile_rank):
    if not values:
        return None
    values = sorted(values)
    k = (len(values) - 1) * percentile_rank / 100.0
    floor = int(k)
    ceiling = min(floor + 1, len(values)-1)
    return values[floor] * (ceiling - k) + values[ceiling] * (k-floor)


def summarize(path):
    path = Path(path)
    with path.open('r', newline='', encoding='utf-8-sig') as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise ValueError('Empty commissioning CSV')
    recording = [row for row in rows if str(row.get('test_mode','')).lower() == 'auto' and str(row.get('is_recording','')).strip() == '1']
    selected = recording if recording else [row for row in rows if str(row.get('test_mode','')).lower() == 'auto']
    if not selected:
        raise ValueError('File contains no Automatic BRUNNER commissioning rows')

    lines = [f'File: {path.name}', f'Samples: {len(selected)} ({"during recording" if recording else "setup only"})',
             'Diagnostic only: this is NOT a qualification pass.', '']
    forward = [number(row.get('cls_to_p3d_feedback_enabled')) for row in selected]
    known_forward = [x for x in forward if x is not None]
    lines.append('CLS2Sim -> P3D forwarding: ' +
                 (f'ON {sum(x == 1 for x in known_forward)}/{len(known_forward)} reported samples'
                  if known_forward else 'NOT RECORDED'))
    observed_any = False
    for name in ('pitch','roll','yaw'):
        prefix = name + '_'
        target_col = prefix+'position_target_norm'
        actual_col = prefix+'position_actual_norm'
        if target_col not in selected[0] or actual_col not in selected[0]:
            continue
        samples = []
        for row in selected:
            target = number(row.get(target_col))
            actual = number(row.get(actual_col))
            same_time = number(row.get(prefix+'position_reference_now_norm'))
            if target is not None and actual is not None:
                samples.append((target,actual,same_time,row.get(prefix+'ap_enabled')))
        if not samples:
            continue
        observed_any = True
        targets = [x[0] for x in samples]
        actuals = [x[1] for x in samples]
        sent_err = [abs(t-a) for t,a,_,_ in samples]
        reference_err = [abs(ref-a) for _,a,ref,_ in samples if ref is not None]
        ap_known = [number(ap) for *_,ap in samples]
        ap_known = [x for x in ap_known if x is not None]
        target_span = max(targets)-min(targets)
        actual_span = max(actuals)-min(actuals)
        lines += [f'\n{name.upper()}',
                  f'  Native AP command range: {min(targets):+.5f} .. {max(targets):+.5f} (span {target_span:.5f})',
                  f'  Actual physical range: {min(actuals):+.5f} .. {max(actuals):+.5f} (span {actual_span:.5f})',
                  f'  Command vs physical median/p95: {statistics.median(sent_err):.5f} / {percentile(sent_err,95):.5f} norm',
                  '  Current-time Manual vs physical median/p95: ' +
                  (f'{statistics.median(reference_err):.5f} / {percentile(reference_err,95):.5f} norm' if reference_err else 'no current-time reference in CSV'),
                  '  AP enabled: ' +
                  (f'{sum(x==1 for x in ap_known)}/{len(ap_known)} reported samples' if ap_known else 'status unavailable')]
        if target_span < 0.008:
            lines.append('  RESULT: STATIC/SMALL reference. Dynamic motor tracking cannot be proven from this run.')
        elif actual_span < min(0.008,0.15*target_span):
            lines.append('  RESULT: PHYSICAL LOADER BARELY MOVED despite a changing command. Check AP/profile/limits.')
        elif percentile(reference_err or sent_err,95) > 0.02:
            lines.append('  RESULT: Significant tracking error. Compare rise/delay with working climb commissioning.')
        else:
            lines.append('  RESULT: Physical movement observed; verify sign, response and P3D input separately.')
        if ap_known and any(x != 1 for x in ap_known):
            lines.append('  WARNING: AP status disabled for at least one sample.')
    if not observed_any:
        lines.append('\nNo BRUNNER target/actual pairs found. Check file version and collection.')
    if known_forward and any(x != 1 for x in known_forward) and recording:
        lines.append('\nWARNING: CLS2Sim position output not enabled throughout recording.')
    lines.append('\nImportant: A moving BRUNNER axis does not prove the corresponding P3D elevator, aileron or rudder moved. Check the separately saved P3D State Diagnostic CSV.')
    return '\n'.join(lines) + '\n'


if __name__ == '__main__':
    if len(sys.argv) != 2:
        sys.exit('Usage: py -3 CHECK_BRUNNER_REPLAY.py "..._Brunner_Commissioning_auto_....csv"')
    try:
        result = summarize(sys.argv[1])
        print(result)
        output = Path(sys.argv[1]).with_name(Path(sys.argv[1]).stem + '_tracking_check.txt')
        output.write_text(result, encoding='utf-8')
        print('Report saved:', output)
    except Exception as exc:
        sys.exit('Unable to analyze CSV: ' + str(exc))
