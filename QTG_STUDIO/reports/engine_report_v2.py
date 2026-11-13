"""Engine reports from actual measurement-audit observations, bound to selected Manual."""
import csv
import json
from pathlib import Path
from engine_rules_v2 import frame_values
CHANNELS={
 'ground_speed_kt':('Ground Speed','kt'),
 'ias_kt':('Airspeed','kt'),'altitude_msl_ft':('Altitude MSL','ft'),
 'height_agl_ft':('Height AGL','ft'),'throttle_pct':('Throttle','%'),
 'engine_rpm':('Engine RPM','rpm'),'prop_rpm':('Propeller RPM','rpm'),
 'manifold_inhg':('Manifold pressure','inHg'),'fuel_flow_pph':('Fuel flow','lb/h'),
 'pitch_position_norm':('BRUNNER pitch','normalised'),'roll_position_norm':('BRUNNER roll','normalised'),
 'yaw_position_norm':('BRUNNER yaw','normalised'),'elevator_input_norm':('P3D elevator input','normalised'),
 'aileron_input_norm':('P3D aileron input','normalised'),'rudder_input_norm':('P3D rudder input','normalised')}

def load_measurements(csv_path):
    p=Path(csv_path);p=p.with_name('Measured_'+p.stem).with_suffix('.jsonl')
    rows=[];previous=None
    for line in p.read_text().splitlines():
        record=json.loads(line);seq=record['p3d'].get('bridge_telemetry_sequence')
        if seq is None or seq==previous:continue
        previous=seq;rows.append(frame_values(record))
    if not rows:raise ValueError('No fresh measured observations for engine report')
    return rows

def _write_channel_quality_note(base, manual_path):
    """Readable RPM/ground-speed audit; explicitly show zero-vs-missing cases."""
    from plot_graphs import _load_engine_dataframe, _read_csv, _series, _rpm_signal_diagnostics
    import numpy as np
    paths = ([('Manual', Path(manual_path)), ('Automatic', Path(base))]
             if manual_path else [('Manual', Path(base))])
    lines = ["V2.1.3 ENGINE CHANNEL QUALITY CHECK - diagnostic only",
             "Actual P3D source measurements. No RPM is guessed or manufactured.", ""]
    for label, p in paths:
        if not p.exists():
            lines.append(f"{label}: CSV missing: {p}")
            continue
        try:
            high = _load_engine_dataframe(p)
            legacy = _read_csv(p)
            lines.append(f"{label}: {p.name}")
            for key in ('engine_rpm', 'prop_rpm'):
                info = _rpm_signal_diagnostics(high, legacy, key)
                lines.append(f"  {key}: {info['state']} | high-rate samples={info['high_count']}, "
                             f"min={info['high_min']}, max={info['high_max']} | "
                             f"legacy samples={info['legacy_count']}, "
                             f"min={info['legacy_min']}, max={info['legacy_max']}")
                if info['state'].endswith('legacy_disagrees'):
                    lines.append("    SOURCE CONFLICT: high-rate RPM is zero/missing but recorded 1-Hz "
                                 "P3D RPM is nonzero. Check P3D bridge timing/data; "
                                 "the report displays both sources without substituting either.")
            clock = _series(high, 'time_s')
            ct = clock[np.isfinite(clock)]
            has_high = len(ct) >= 3 and np.median(np.diff(ct)) < 0.25
            gs = _series(high, 'ground_speed_kt') if has_high else np.array([])
            vals = gs[np.isfinite(gs)]
            lines.append(f"  high-rate ground_speed_kt: samples={len(vals)}, "
                         f"min={float(vals.min()) if len(vals) else 'unavailable'}, "
                         f"max={float(vals.max()) if len(vals) else 'unavailable'}")
            legacy_gs = _series(legacy, 'ground_speed_kt')
            lv = legacy_gs[np.isfinite(legacy_gs)]
            lines.append(f"  native 1-Hz Ground Speed (kt): samples={len(lv)}, "
                         f"min={float(lv.min()) if len(lv) else 'unavailable'}, "
                         f"max={float(lv.max()) if len(lv) else 'unavailable'}")
            lines.append("")
        except Exception as e:
            lines.append(f"  Unable to inspect channel data: {e}")
    target=Path(base).with_name(Path(base).stem + '_RPM_GroundSpeed_Diagnostic.txt')
    target.write_text('\n'.join(lines), encoding='utf-8')
    return target


def create_report(csv_path, manual_path, metadata):
    from plot_graphs import plot_aligned_comparison
    p = Path(csv_path)
    base_p = p
    if p.stem.endswith('_legacy_plot'):
        orig = p.with_name(p.stem[:-12] + '.csv')
        if orig.exists():
            base_p = orig

    try:
        rows = load_measurements(base_p)
        manifest = dict(metadata or {}, source_csv=base_p.name, selected_manual=str(manual_path) if manual_path else None)
        base_p.with_name(base_p.stem + '_engine_test').with_suffix('.json').write_text(json.dumps(manifest, indent=2))
        with base_p.with_name(base_p.stem + '_engine_channels').with_suffix('.csv').open('w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0]))
            w.writeheader()
            w.writerows(rows)
    except Exception as exc:
        print(f"[ENGINE ARTIFACTS EXPORT] {exc}", flush=True)

    m_path = Path(manual_path) if manual_path else None
    if m_path and not m_path.stem.endswith('_legacy_plot'):
        m_legacy = m_path.with_name(m_path.stem + '_legacy_plot.csv')
        if m_legacy.exists():
            m_path = m_legacy

    titt_res = None
    try:
        from SMK_Engine_TiTt_V2_1_3_Addon import evaluate_engine_run
        mode = (metadata or {}).get('kind', 'acceleration')
        # High-rate *_engine_channels.csv is selected by the Ti/Tt reader.
        # Never infer EASA qualification from a Manual-vs-Automatic agreement.
        parameter_kind = (metadata or {}).get('critical_parameter_kind', 'map')
        titt_res = evaluate_engine_run(base_p, manual_path=manual_path,
                                        mode=mode, parameter_kind=parameter_kind)
        print(f"[TiTt] Assessment status: {titt_res.get('status')}", flush=True)
    except Exception as e:
        print(f"[TiTt RUN WARNING] {e}", flush=True)

    pdf = plot_aligned_comparison(str(p), manual_csv_path=str(m_path) if m_path else None, titt_results=titt_res, metadata=metadata)
    try:
        note = _write_channel_quality_note(base_p, m_path)
        print(f"[ENGINE QUALITY] Saved measured RPM / ground-speed details: {note}", flush=True)
    except Exception as e:
        print(f"[ENGINE QUALITY WARNING] Could not export channel note: {e}", flush=True)
    return Path(pdf)

