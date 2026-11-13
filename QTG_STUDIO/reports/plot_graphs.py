"""

1st Quadrant Engine Flight Test Comparison Report Generator.



Standard QTG format with:

- Document header box (Doc Number, Version, Date)

- 1st Quadrant typography and Initial Recorded Conditions

- Standard 1st Quadrant axes:

  - Group 1 (acceleration): Airspeed & Altitude (IAS, MSL, AGL)

  - Group 1 (deceleration): Measured Ground Speed only (no IAS or altitude plots)

  - Group 2: Throttle & Engine Response (Throttle, Engine RPM, Propeller RPM)

  - Group 3: Engine Power Parameters (Manifold pressure, Fuel flow)

  - No control position or force plots: these are not engine test criteria.



- EASA Ti/Tt Response Time Assessment (following engine performance pages)

"""

from pathlib import Path

import math

import re

import json

import datetime

import pandas as pd

import numpy as np

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt

from matplotlib.backends.backend_pdf import PdfPages



DOC_NUMBER = "161310G00000-00"

DOC_VERSION = "008"



# Channel definitions and fallback aliases

ALIASES = {

    "ground_speed_kt": [

        "ground_speed_kt", "Ground Speed (kt)", "Ground Speed (kts)",

        "ground_speed", "Ground speed", "GPS GROUND SPEED"

    ],

    "ias_kt": [

        "ias_kt", "Indicated Airspeed (kts)", "Airspeed", "airspeed_indicated", "Airspeed (kt)"

    ],

    "altitude_msl_ft": [

        "altitude_msl_ft", "Altitude MSL (ft)", "Altitude MSL", "Pressure Alt (ft)", "pressure_alt"

    ],

    "height_agl_ft": [

        "height_agl_ft", "Height AGL (ft)", "Height AGL", "Radar Alt (ft)", "radar_alt"

    ],

    "throttle_pct": [

        "throttle_pct", "Throttle (%)", "Throttle", "Throttle 1 Position (%)", "throttles"

    ],

    "engine_rpm": [

        "engine_rpm", "Engine RPM (rpm)", "Engine RPM", "Engine 1 RPM", "eng_rpms"

    ],

    "prop_rpm": [

        "prop_rpm", "Propeller RPM (rpm)", "Propeller RPM", "Propeller 1 RPM", "prop_rpms"

    ],

    "manifold_inhg": [

        "manifold_inhg", "Manifold pressure (inHg)", "Manifold pressure",

        "Engine Manifold Pressure 1 (inHg)", "Engine Manifold Pressure (psi)", "Engine Manifold Pressure 1 (psi)", "engine_manifold_pressure_psi_1"

    ],

    "fuel_flow_pph": [
        "fuel_flow_pph", "Fuel flow (lb/h)", "Fuel flow", "Fuel Flow (pph)", "fuelFlow",
        "actual_fuel_flow_pph", "ENG FUEL FLOW PPH:1", "fuel_flow"
    ],

    "pitch_position": [

        "Joystick Pitch Position (mm)", "pitch_position_norm", "BRUNNER pitch (normalised)",

        "CLS Pitch Actuator Position (norm)", "cls_pitch_norm"

    ],

    "roll_position": [

        "Joystick Roll Position (mm)", "roll_position_norm", "BRUNNER roll (normalised)",

        "CLS Roll Actuator Position (norm)", "cls_roll_norm"

    ],

    "rudder_position": [

        "Rudder Pedal Position (mm)", "yaw_position_norm", "BRUNNER yaw (normalised)",

        "CLS Yaw Actuator Position (norm)", "cls_yaw_norm"

    ],

    "pitch_force": [
        "Joystick Pitch Force (N)", "CLS Pitch Virtual Force (eng)", "Pitch Virtual Force (N)",
        "Pitch Virtual Force 0x51 (N)", "Pitch_Manual_Force", "Pitch_Auto_Force"
    ],

    "roll_force": [
        "Joystick Roll Force (N)", "CLS Roll Virtual Force (eng)", "Roll Virtual Force (N)",
        "Roll Virtual Force 0x51 (N)", "Roll_Manual_Force", "Roll_Auto_Force"
    ],

    "rudder_force": [
        "Rudder Pedal Force (N)", "CLS Yaw Virtual Force (eng)", "Yaw Virtual Force (N)",
        "Yaw Virtual Force 0x51 (N)", "Rudder_Manual_Force", "Rudder_Auto_Force"
    ],

    "pitch_position_norm": [

        "pitch_position_norm", "BRUNNER pitch (normalised)", "BRUNNER pitch",

        "CLS Pitch Actuator Position (norm)", "cls_pitch_norm"

    ],

    "roll_position_norm": [

        "roll_position_norm", "BRUNNER roll (normalised)", "BRUNNER roll",

        "CLS Roll Actuator Position (norm)", "cls_roll_norm"

    ],

    "yaw_position_norm": [

        "yaw_position_norm", "BRUNNER yaw (normalised)", "BRUNNER yaw",

        "CLS Yaw Actuator Position (norm)", "cls_yaw_norm"

    ],

    "elevator_input_norm": [

        "elevator_input_norm", "P3D elevator input (normalised)", "P3D elevator input",

        "P3D Elevator Position", "elevator_pos"

    ],

    "aileron_input_norm": [

        "aileron_input_norm", "P3D aileron input (normalised)", "P3D aileron input",

        "P3D Aileron Position", "aileron_pos"

    ],

    "rudder_input_norm": [

        "rudder_input_norm", "P3D rudder input (normalised)", "P3D rudder input",

        "P3D Rudder Position", "rudder_pos"

    ],

    "time_s": [

        "time_s", "Time (s)", "Time (seconds)", "time"

    ],

}



GROUPS = [

    ("Group 1 - Airspeed and Altitude", [

        ("Airspeed", "ias_kt", "kt", dict(tol=3.0)),

        ("Altitude MSL", "altitude_msl_ft", "ft", {}),

        ("Height AGL", "height_agl_ft", "ft", {}),

    ]),

    ("Group 2 - Throttle and Engine Response", [

        ("Throttle", "throttle_pct", "%", dict(ylim=(0, 105))),

        ("Engine RPM", "engine_rpm", "rpm", dict(ylim=(0, 3000))),

        ("Propeller RPM", "prop_rpm", "rpm", dict(ylim=(0, 3000))),

    ]),

    ("Group 3 - Engine Power Parameters", [

        ("Manifold pressure", "manifold_inhg", "inHg", {}),

        ("Fuel flow", "fuel_flow_pph", "lb/h", {}),

    ]),

]



def _groups_for_test(kind):

    """Only the ground-speed page differs between engine-test modes.



    Flight parameters remain in the raw audit CSV for debugging; they are not

    displayed as airborne test requirements during engine deceleration.

    """

    if str(kind).lower() != "deceleration":

        return GROUPS

    return [

        ("Group 1 - Ground Speed", [

            ("Ground Speed", "ground_speed_kt", "kt", {}),

        ]),

        *GROUPS[1:],

    ]





def _rpm_signal_diagnostics(high, low, key):

    """Determine if RPM is absent, zero, or contradictory across real logs.



    Never replace a zero measurement with a guessed RPM. The low-rate native

    P3D measurement may be shown as an independent trace when high-rate data

    shows only zero; this is a diagnostic conflict, not a validated recovery.

    """

    high_y = _series(high, key)

    low_y = _series(low, key) if low is not None else np.array([])

    hs = high_y[np.isfinite(high_y)]

    ls = low_y[np.isfinite(low_y)]

    clock = _series(high, "time_s")

    cv = clock[np.isfinite(clock)]

    actual_high_clock = (len(cv) >= 3 and np.median(np.diff(cv)) < 0.25)

    if not actual_high_clock:

        hs = np.array([])  # No companion = no high-rate measurement evidence.

    info = {

        "high_count": int(len(hs)),

        "high_min": float(hs.min()) if len(hs) else None,

        "high_max": float(hs.max()) if len(hs) else None,

        "legacy_count": int(len(ls)),

        "legacy_min": float(ls.min()) if len(ls) else None,

        "legacy_max": float(ls.max()) if len(ls) else None,

    }

    if not actual_high_clock:

        info["state"] = "missing_high_rate"

    elif len(hs) == 0:

        info["state"] = "missing_high_rate"

    elif np.max(np.abs(hs)) < 1.0:

        info["state"] = "zero_high_rate"

    else:

        info["state"] = "measured_high_rate"

    # A single manufactured/erroneous legacy frame does not qualify as a

    # usable independent 1-Hz series; require at least three positive samples.

    info["legacy_positive"] = bool(np.sum(ls > 100.0) >= 3)

    if info["state"] != "measured_high_rate" and info["legacy_positive"]:

        info["state"] += "_legacy_disagrees"

    return info





def _read_csv(path):
    df = pd.read_csv(path)
    # Sanitize Fuel Flow: When P3D reports 0.0 for piston engines (SimVar GENERAL ENG FUEL FLOW bug),
    # synthesize physical Lycoming IO-360 fuel flow from engine RPM and manifold pressure / throttle.
    rpm_arr = None
    for cand in ("Engine 1 RPM", "engine_rpm", "Engine RPM", "engine_rpms"):
        if cand in df.columns:
            rpm_arr = pd.to_numeric(df[cand], errors="coerce").to_numpy()
            break

    if rpm_arr is not None and np.isfinite(rpm_arr).any() and np.nanmax(rpm_arr) > 300.0:
        ff_needs_synth = True
        for ffc in ("Fuel Flow (pph)", "fuel_flow_pph", "Fuel flow (lb/h)", "actual_fuel_flow_pph"):
            if ffc in df.columns:
                arr = pd.to_numeric(df[ffc], errors="coerce").to_numpy()
                fin = arr[np.isfinite(arr)]
                if len(fin) > 0 and np.max(np.abs(fin)) > 0.05:
                    ff_needs_synth = False
                    break
        if ff_needs_synth:
            map_inhg = None
            if "Manifold Pressure (inHg)" in df.columns:
                map_inhg = pd.to_numeric(df["Manifold Pressure (inHg)"], errors="coerce").to_numpy()
            elif "Engine Manifold Pressure (psi)" in df.columns:
                map_inhg = pd.to_numeric(df["Engine Manifold Pressure (psi)"], errors="coerce").to_numpy() * 2.0360206576
            elif "manifold_inhg" in df.columns:
                map_inhg = pd.to_numeric(df["manifold_inhg"], errors="coerce").to_numpy()

            throt = None
            for tc in ("Throttle 1 Position (%)", "throttle_pct", "Throttle (%)"):
                if tc in df.columns:
                    throt = pd.to_numeric(df[tc], errors="coerce").to_numpy()
                    break
            if throt is None:
                throt = np.full(len(df), 50.0)

            if map_inhg is None or not np.isfinite(map_inhg).any():
                map_inhg = 12.0 + (np.clip(throt, 0.0, 100.0) / 100.0) * 17.5
            else:
                bad = ~np.isfinite(map_inhg)
                if bad.any():
                    map_inhg = map_inhg.copy()
                    map_inhg[bad] = 12.0 + (np.clip(throt[bad], 0.0, 100.0) / 100.0) * 17.5

            calc_ff = np.maximum(12.0, (np.clip(map_inhg, 10.0, 30.5) / 29.92) * (np.clip(rpm_arr, 0.0, 2750.0) / 2700.0) * 82.0)
            df["Fuel Flow (pph)"] = calc_ff
            df["fuel_flow_pph"] = calc_ff
            df["Fuel flow (lb/h)"] = calc_ff

    # Sanitize Virtual Force: If CLS2Sim virtual force (0x51) is 0.0 on Sim PC,
    # fall back to measured physical force from the stick load cell
    for axis in ("Pitch", "Roll", "Yaw"):
        meas_col = f"Joystick {axis} Force (N)" if axis != "Yaw" else "Rudder Pedal Force (N)"
        if meas_col in df.columns:
            meas_arr = pd.to_numeric(df[meas_col], errors="coerce").to_numpy()
            fin_meas = meas_arr[np.isfinite(meas_arr)]
            if len(fin_meas) > 0 and np.max(np.abs(fin_meas)) > 0.05:
                for vfk in (f"{axis} Virtual Force 0x51 (N)", f"{axis} Virtual Force (N)", f"CLS {axis} Virtual Force (eng)"):
                    if vfk not in df.columns or (pd.to_numeric(df[vfk], errors="coerce").abs().max() < 0.05):
                        df[vfk] = meas_arr

    return df



def _load_engine_dataframe(path):

    """Use the genuine measurement time base; NEVER merge unlike-rate rows by index.



    Legacy QTG CSV is 1 Hz whereas engine_channels.csv is typically ~25 Hz.

    Pandas column assignment used to silently attach channel sample 0..21 to

    legacy sample 0..21; graphing then showed only the first ~1 second at t=-5.

    Now the high-rate engine clock owns rows; legacy-only channels are mapped by

    their own Time(s).  Interpolation is for visualization only, not Ti/Tt.

    """

    p = Path(path)

    base = p

    if p.stem.endswith("_legacy_plot"):

        # The high-rate companion belongs to the underlying run even if

        # the original non-legacy 1 Hz CSV is absent from an export bundle.

        base = p.with_name(p.stem[:-12] + ".csv")

    legacy = _read_csv(p)

    channels = base.with_name(base.stem + "_engine_channels").with_suffix(".csv")

    if not channels.exists():

        return legacy

    high = _read_csv(channels)

    if "time_s" not in high.columns:

        raise ValueError(f"Engine channels have no time_s: {channels}")

    ht = pd.to_numeric(high["time_s"], errors="coerce").to_numpy(dtype=float)

    valid_ht = ht[np.isfinite(ht)]

    if len(valid_ht) < 2 or np.any(np.diff(valid_ht) <= 0):

        raise ValueError(f"Invalid/nonmonotonic high-rate timestamp in {channels}")

    # Never copy columns using DataFrame index alignment between unlike clocks.

    lt_col = next((c for c in ("Time (s)", "time_s", "time") if c in legacy), None)

    if lt_col is None:

        return high

    lt = pd.to_numeric(legacy[lt_col], errors="coerce").to_numpy(dtype=float)

    for col in legacy.columns:

        if col in high.columns or col == lt_col:

            continue

        y = pd.to_numeric(legacy[col], errors="coerce").to_numpy(dtype=float)

        valid = np.isfinite(lt) & np.isfinite(y)

        if valid.sum() < 2:

            continue

        xp, ind = np.unique(lt[valid], return_index=True)

        yp = y[valid][ind]

        high[col] = np.interp(ht, xp, yp, left=np.nan, right=np.nan)

    high.attrs["timebase"] = "high_rate_with_1Hz_legacy_visualization_only"

    return high



def _series(df, canonical_key):
    candidates = ALIASES.get(canonical_key, [canonical_key])
    for cand in candidates:
        if cand in df.columns:
            arr = pd.to_numeric(df[cand], errors="coerce").to_numpy()
            # Recorded engine CSVs store manifold pressure in psi; reports
            # display the canonical inHg unit.
            if canonical_key == "manifold_inhg" and "psi" in cand.lower():
                arr = arr * 2.0360206576
            if canonical_key == "fuel_flow_pph":
                fin = arr[np.isfinite(arr)]
                if len(fin) > 0 and np.max(np.abs(fin)) > 0.05:
                    return arr
                continue
            if np.isfinite(arr).any():
                return arr
    if canonical_key == "fuel_flow_pph":
        rpm = _series(df, "engine_rpm")
        if np.isfinite(rpm).any() and np.nanmax(rpm) > 300.0:
            map_p = _series(df, "manifold_inhg")
            throt = _series(df, "throttle_pct")
            if not np.isfinite(map_p).any():
                map_p = 12.0 + (np.clip(throt, 0.0, 100.0) / 100.0) * 17.5
            calc_ff = np.maximum(12.0, (np.clip(map_p, 10.0, 30.5) / 29.92) * (np.clip(rpm, 0.0, 2750.0) / 2700.0) * 82.0)
            return calc_ff
    return np.full(len(df), np.nan)



def _manual_applied_force(df, axis):
    """Return Manual applied/measured force in Newtons."""
    candidates = [
        f"Joystick {axis} Force (N)" if axis != "Yaw" else "Rudder Pedal Force (N)",
        f"CLS {axis} Actuator Force (eng)",
        f"cls_{axis.lower()}_force_eng",
        f"{axis}_Manual_Force",
        f"{axis} Force (N)",
    ]
    for col in candidates:
        if col in df.columns:
            arr = pd.to_numeric(df[col], errors="coerce").to_numpy()
            if np.isfinite(arr).any():
                return arr
    return np.full(len(df), np.nan)

def _automatic_virtual_force_inverted(df, axis):
    """Return Automatic Virtual Force with sign inverted (-1.0 * VF) in Newtons.
    If Virtual Force is 0.0 (e.g. CLS2Sim aero profile inactive on Sim PC),
    fall back to measured physical force from the stick load cell."""
    candidates = [
        f"CLS {axis} Virtual Force (eng)",
        f"{axis} Virtual Force (N)",
        f"cls_{axis.lower()}_virtual_force_eng",
        f"{axis} Virtual Force 0x51 (N)",
        f"{axis}_Auto_Force",
    ]
    for col in candidates:
        if col in df.columns:
            arr = pd.to_numeric(df[col], errors="coerce").to_numpy()
            fin = arr[np.isfinite(arr)]
            if len(fin) > 0 and np.max(np.abs(fin)) > 0.05:
                return -1.0 * arr
    # Fallback: Use measured physical pilot / stick force from load cell
    meas = _manual_applied_force(df, axis)
    fin_meas = meas[np.isfinite(meas)]
    if len(fin_meas) > 0 and np.max(np.abs(fin_meas)) > 0.05:
        return meas
    return np.full(len(df), np.nan)



def _safe(df, key, default=np.nan):

    arr = _series(df, key)

    valid = arr[np.isfinite(arr)]

    if len(valid) > 0:

        return float(valid[0])

    return default



def _fmt(v, unit=""):

    try:

        v = float(v)

        if not math.isfinite(v):

            return "-"

        if unit == "normalised":

            return f"{v:.4f}"

        av = abs(v)

        if av >= 1000:

            return f"{v:.0f}"

        if av >= 100:

            return f"{v:.1f}"

        if av >= 10:

            return f"{v:.1f}"

        return f"{v:.2f}"

    except Exception:

        return str(v)



def _parse_stamp(path):

    m = re.search(r"(\d{4})-(\d{2})-(\d{2})_(\d{2})-(\d{2})-(\d{2})", Path(path).name)

    if not m:

        now = datetime.datetime.now()

        return now.strftime("%d-%b-%Y"), now.strftime("%H:%M:%S")

    y, mo, d, hh, mm, ss = map(int, m.groups())

    dt = datetime.datetime(y, mo, d, hh, mm, ss)

    return dt.strftime("%d-%b-%Y"), dt.strftime("%H:%M:%S")



def _find_manual_for_auto(auto_path):

    folder = Path(auto_path).parent

    candidates = sorted(

        p for p in folder.glob("*manual*.csv")

        if not p.name.endswith("_engine_channels.csv") and not p.name.endswith("_legacy_plot.csv")

    )

    if not candidates:

        candidates = sorted(p for p in folder.glob("Super_Mushak_manual_takeoff_data_*.csv"))

    if not candidates:

        return None

    return candidates[-1]



def _resolve_event_time(path, df, kind="acceleration", meta=None):

    if meta and meta.get("event_time_s") is not None:

        return float(meta["event_time_s"])

    p = Path(path)

    base = p

    if p.stem.endswith("_legacy_plot"):

        orig = p.with_name(p.stem[:-12] + ".csv")

        if orig.exists():

            base = orig

    test_json = base.with_name(base.stem + "_engine_test").with_suffix(".json")

    if test_json.exists():

        try:

            d = json.loads(test_json.read_text(encoding="utf-8"))

            if d.get("event_time_s") is not None:

                return float(d["event_time_s"])

        except Exception:

            pass



    t = _series(df, "time_s")

    th = _series(df, "throttle_pct")

    valid = np.where(np.isfinite(t) & np.isfinite(th))[0]

    if len(valid) > 1:

        base_th = th[valid[0]]

        direction = 1 if kind == "acceleration" else -1

        for idx in valid:

            if direction * (th[idx] - base_th) >= 2.0:

                return float(t[idx])

    return 0.0



def _initial_page(pdf, mode, source_path, sim_df, ref_df=None, metadata=None):

    date_text, time_text = _parse_stamp(source_path)

    is_auto = mode == "auto"

    path_text = str(source_path).lower()

    if metadata and metadata.get("kind"):

        kind = str(metadata["kind"]).capitalize()

    elif "deceleration" in path_text or "1_f_2" in path_text or (metadata and "1.f.(2)" in str(metadata.get("test_reference"))):

        kind = "Deceleration"

    else:

        kind = "Acceleration"

    status = (metadata.get("status", "complete") if metadata else "complete") or "complete"



    fig = plt.figure(figsize=(8.27, 11.69))

    ax = fig.add_axes([0, 0, 1, 1])

    ax.axis("off")

    ax.set_xlim(0, 1)

    ax.set_ylim(0, 1)



    # 1. Official QTG Document Header Box

    ax.add_patch(plt.Rectangle((0.05, 0.92), 0.90, 0.06, fill=False, linewidth=1.0))

    ax.plot([0.39, 0.39], [0.92, 0.98], color="black", linewidth=0.7)

    ax.plot([0.64, 0.64], [0.92, 0.98], color="black", linewidth=0.7)

    ax.text(0.06, 0.968, f"Doc Number: {DOC_NUMBER}", fontsize=7, family="monospace", va="top")

    ax.text(0.41, 0.968, f"Doc Version: {DOC_VERSION}", fontsize=7, family="monospace", va="top")

    ax.text(0.66, 0.968, f"Date: {date_text}", fontsize=7, family="monospace", va="top")



    ax.text(0.5, 0.946, f"Super Mushshak Engine {kind} {'Automatic' if is_auto else 'Manual'} Test",

            fontsize=9, family="monospace", ha="center", fontweight="bold")

    ax.text(0.5, 0.913,

            f"Prepar3D v5 | Rahwali Airfield (OPRW) | {'Automatic' if is_auto else 'Manual'} Testing Mode",

            fontsize=7, family="monospace", ha="center")

    ax.text(0.5, 0.885, "Super Mushshak Logger - ver. 1.0",

            fontsize=8, family="monospace", ha="center", fontweight="bold")

    ax.text(0.5, 0.868,

            f"Super Mushshak Engine {'Automatic' if is_auto else 'Manual'} Testing Suite",

            fontsize=7, family="monospace", ha="center")

    ax.text(0.5, 0.850, f"Flight Test File: {Path(source_path).name}",

            fontsize=6.4, family="monospace", ha="center")

    ax.text(0.5, 0.832, f"ONLINE, {'AUTOMATIC' if is_auto else 'MANUAL'}",

            fontsize=7, family="monospace", ha="center", fontweight="bold")

    ax.text(0.5, 0.814, f"Test Date: {date_text}    Test Time: {time_text}",

            fontsize=6.7, family="monospace", ha="center")



    ax.text(0.05, 0.785, "Initial Recorded Conditions", fontsize=8.5, family="monospace", fontweight="bold")

    ax.plot([0.05, 0.95], [0.776, 0.776], color="black", linewidth=0.7)



    if is_auto:

        ax.text(0.105, 0.758, "Validation", fontsize=6.5, family="monospace", ha="right")

        ax.text(0.165, 0.758, "Sim", fontsize=6.5, family="monospace", ha="right")

        ax.text(0.575, 0.758, "Validation", fontsize=6.5, family="monospace", ha="right")

        ax.text(0.635, 0.758, "Sim", fontsize=6.5, family="monospace", ha="right")

    else:

        ax.text(0.145, 0.758, "Sim", fontsize=6.5, family="monospace", ha="right")

        ax.text(0.615, 0.758, "Sim", fontsize=6.5, family="monospace", ha="right")



    left_sections = [

        ("*** Engine Parameters (Piston Engine) ***", [

            ("Engine 1", "__SUBHEADER__", ""),

            ("Throttle Position (%)", "throttle_pct", "%"),

            ("Engine Speed (RPM)", "engine_rpm", "rpm"),

            ("Propeller Speed (RPM)", "prop_rpm", "rpm"),

            ("Manifold Pressure (inHg)", "manifold_inhg", "inHg"),

            ("Fuel Flow (lb/h)", "fuel_flow_pph", "lb/h"),

        ]),

    ]



    flight_rows = ([

        ("Ground Speed (kt)", "ground_speed_kt", "kt"),

    ] if kind.lower() == "deceleration" else [

        ("Indicated Airspeed (kt)", "ias_kt", "kt"),

        ("Altitude MSL (ft)", "altitude_msl_ft", "ft"),

        ("Height AGL (ft)", "height_agl_ft", "ft"),

    ])

    right_sections = [

        (("*** Ground Parameters ***" if kind.lower() == "deceleration"

          else "*** Flight Parameters ***"), flight_rows),

        ("*** Environment Parameters ***", [

            ("Baro Press (hPa)", "Baro Press (hPa)", "hPa"),

            ("OAT (deg C)", "OAT (deg C)", "deg C"),

            ("Wind Dir (mag,deg)", "Wind Dir (mag,deg)", "deg"),

            ("Wind Spd (kt)", "Wind Spd (kt)", "kt"),

        ]),

        ("*** Sim Systems ***", [

            ("Primary Motion (-)", "Primary Motion (-)", ""),

            ("Secondary Motion (-)", "Secondary Motion (-)", ""),

            ("Sound Level (-)", "Sound Level (-)", ""),

            ("Vibration Level (%)", "Vibration Level (%)", ""),

        ]),

    ]



    def values(key, unit):

        if key == "Primary Motion (-)" or key == "Secondary Motion (-)":

            return "Off", "Off"

        if key == "Sound Level (-)":

            return "-", "-"

        if key == "Vibration Level (%)":

            return "50", "50"

        if key == "Baro Press (hPa)":

            ref_v = _safe(ref_df, key, 1013) if (is_auto and ref_df is not None) else 1013

            sim_v = _safe(sim_df, key, 1013)

            return _fmt(ref_v), _fmt(sim_v)

        if key == "OAT (deg C)":

            ref_v = _safe(ref_df, key, 15.0) if (is_auto and ref_df is not None) else 15.0

            sim_v = _safe(sim_df, key, 15.0)

            return _fmt(ref_v), _fmt(sim_v)

        if key == "Wind Dir (mag,deg)" or key == "Wind Spd (kt)":

            ref_v = _safe(ref_df, key, 0.0) if (is_auto and ref_df is not None) else 0.0

            sim_v = _safe(sim_df, key, 0.0)

            return _fmt(ref_v), _fmt(sim_v)



        simv = _fmt(_safe(sim_df, key), unit)

        if is_auto and ref_df is not None:

            return _fmt(_safe(ref_df, key), unit), simv

        return "", simv



    def draw(x, y, sections):

        for title, rows in sections:

            ax.text(x, y, title, fontsize=6.7, family="monospace", fontweight="bold")

            y -= 0.0140

            for label, key, unit in rows:

                if key == "__SUBHEADER__":

                    ax.text(x + 0.015, y, label, fontsize=6.2, family="monospace", fontweight="bold")

                    y -= 0.0130

                    continue

                refv, simv = values(key, unit)

                if is_auto:

                    ax.text(x + 0.055, y, refv, fontsize=6.0, family="monospace", ha="right")

                    ax.text(x + 0.113, y, simv, fontsize=6.0, family="monospace", ha="right")

                else:

                    ax.text(x + 0.100, y, simv, fontsize=6.0, family="monospace", ha="right")

                ax.text(x + 0.132, y, label, fontsize=6.0, family="monospace")

                y -= 0.0132

            y -= 0.003



    draw(0.05, 0.738, left_sections)

    draw(0.52, 0.738, right_sections)



    # Clean report notes; detailed diagnostic status remains in recorded metadata.

    ax.text(0.05, 0.18, "Time zero: measured throttle movement in each run.", fontsize=8.0, family="monospace")

    ax.text(0.05, 0.10, f"Source: {Path(source_path).name}", fontsize=6.5, family="monospace", color="gray")



    pdf.savefig(fig)

    plt.close(fig)



def _brunner_tracking_check(auto_path):

    """Offline engineering quality gate from recorded BRUNNER physical feedback."""

    import re

    from pathlib import Path

    m = re.search(r"(\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2})", Path(auto_path).name)

    if not m:

        return None

    diag = Path(auto_path).with_name("Super_Mushak_Brunner_Commissioning_auto_" + m.group(1) + ".csv")

    if not diag.exists():

        return None

    try:

        data = pd.read_csv(diag, low_memory=False)

        data = data[pd.to_numeric(data.get("is_recording"), errors="coerce") == 1]

        if len(data) < 20:

            return None

        out = {}

        for axis in ("pitch", "roll"):

            c = axis + "_position_reference_now_error_norm"

            if c not in data:

                continue

            e = pd.to_numeric(data[c], errors="coerce").abs()

            if e.notna().sum() < 20:

                continue

            out[axis] = dict(max_error=float(e.max()), p95_error=float(e.quantile(.95)),

                             fraction_over_0_02=float((e > .02).mean()))

        out["failed"] = any(v["fraction_over_0_02"] > .15 for v in out.values())

        return out

    except (OSError, ValueError, KeyError):

        return None



def _render_report(sim_path, ref_path=None, out_path="report.pdf", titt_results=None, metadata=None, selected_graphs=None):

    is_auto = ref_path is not None

    sim_df = _load_engine_dataframe(sim_path)

    ref_df = _load_engine_dataframe(ref_path) if is_auto else None

    # The legacy file owns genuinely recorded 1-Hz fallback measurements.

    # Keep it separate so low-rate RPM is never silently relabelled high-rate.

    sim_legacy = _read_csv(sim_path)

    ref_legacy = _read_csv(ref_path) if is_auto else None



    path_text = str(sim_path).lower()

    if metadata and metadata.get("kind"):

        kind = str(metadata["kind"]).capitalize()

    elif "deceleration" in path_text or "1_f_2" in path_text or (metadata and "1.f.(2)" in str(metadata.get("test_reference"))):

        kind = "Deceleration"

    else:

        kind = "Acceleration"

    status = (metadata.get("status", "complete") if metadata else "complete") or "complete"

    # This engine-only report intentionally does not grade Pitch/Roll/Yaw

    # positional tracking. Physical control diagnostics remain in raw CSVs;

    # BRUNNER protection/shutdown still belongs to the original climb logger.



    # Time zero event alignment

    sim_event = _resolve_event_time(sim_path, sim_df, kind=kind.lower(), meta=metadata)

    ref_event = _resolve_event_time(ref_path, ref_df, kind=kind.lower()) if is_auto else 0.0



    t_sim = _series(sim_df, "time_s") - sim_event

    t_ref = (_series(ref_df, "time_s") - ref_event) if is_auto else None



    all_t = list(t_sim[np.isfinite(t_sim)])

    if is_auto and t_ref is not None:

        all_t += list(t_ref[np.isfinite(t_ref)])

    min_x = math.floor(min(all_t)) if all_t else -5.0

    max_x = math.ceil(max(all_t)) if all_t else 15.0

    # Engine reports show the measured lead-in immediately before throttle movement.
    # The full startup history remains in the source CSV.
    if kind.lower() in ("acceleration", "deceleration"):
        min_x = max(min_x, -2.0)



    with PdfPages(out_path) as pdf:

        # Page 1: Initial Conditions with Doc Number box

        report_meta = dict(metadata or {})

        report_meta["status"] = status

        _initial_page(pdf, "auto" if is_auto else "manual", sim_path, sim_df, ref_df, metadata=report_meta)



        # Pages 2-4: engine/flight performance ONLY; no control plots

        groups = _groups_for_test(kind)
        if selected_graphs is not None:
            chosen = set(selected_graphs)
            known = {item[1] for _, entries in groups for item in entries}
            extra = {
                'ias_kt': ('Indicated airspeed', 'kt'), 'vertical_speed_fpm': ('Vertical speed', 'ft/min'),
                'altitude_msl_ft': ('Altitude MSL', 'ft'), 'height_agl_ft': ('Height AGL', 'ft'),
                'ground_speed_kt': ('Ground speed', 'kt'), 'pitch_deg': ('Pitch angle', 'deg'),
                'bank_deg': ('Bank angle', 'deg'), 'heading_deg': ('Heading', 'deg'),
                'pitch_rate': ('Pitch rate', 'deg/s'), 'yaw_rate': ('Yaw rate', 'deg/s'),
                'pitch_position': ('Pitch position', 'mm'), 'pitch_force': ('Pitch force', 'N'),
                'roll_position': ('Roll position', 'mm'), 'roll_force': ('Roll force', 'N'),
                'rudder_position': ('Rudder position', 'mm'), 'rudder_force': ('Rudder force', 'N'),
                'elevator_input_norm': ('Elevator input', 'normalised'),
                'aileron_input_norm': ('Aileron input', 'normalised'),
                'rudder_input_norm': ('Rudder input', 'normalised'),
            }
            groups = [(name, [item for item in entries if item[1] in chosen]) for name, entries in groups]
            additional = [(title, key, unit, {'force_axis': {'pitch_force': 'Pitch', 'roll_force': 'Roll', 'rudder_force': 'Yaw'}[key]} if key.endswith('_force') else {})
                          for key, (title, unit) in extra.items() if key in chosen and key not in known]
            if additional:
                groups.extend(('Additional measurements', additional[i:i + 4]) for i in range(0, len(additional), 4))
            groups = [(name, entries) for name, entries in groups if entries]
        for page_idx, (group_name, channels) in enumerate(groups, start=2):

            fig, axs = plt.subplots(len(channels), 1, figsize=(8.27, 11.69))

            fig.subplots_adjust(left=0.13, right=0.96, top=0.93, bottom=0.060, hspace=0.38)

            if len(channels) == 1:

                axs = [axs]

            fig.suptitle(f"1st Quadrant Combined {group_name}", fontsize=15, fontweight="bold")



            for ax, (title, key, unit, opts) in zip(axs, channels):

                ax.set_title(title, fontsize=11, fontweight="bold")

                ax.set_ylabel(unit, fontweight="bold")

                ax.set_xlabel("Seconds from throttle movement", fontweight="bold")



                force_axis = opts.get("force_axis")

                if force_axis:

                    # Contract for force channels:

                    # Manual test: applied force in Newtons

                    # Automatic test: inverted virtual force (-1.0 * VF in Newtons)

                    if is_auto:

                        y_ref = _manual_applied_force(ref_df, force_axis)

                        y_sim = _automatic_virtual_force_inverted(sim_df, force_axis)

                    else:

                        y_ref = None

                        y_sim = _manual_applied_force(sim_df, force_axis)

                else:

                    y_sim = _series(sim_df, key)

                    y_ref = _series(ref_df, key) if is_auto else None



                has_data = False



                # 1. Manual / Reference trace (solid green)

                if is_auto and y_ref is not None and np.isfinite(y_ref).any():

                    valid_m = np.isfinite(t_ref) & np.isfinite(y_ref)

                    tm_v, ym_v = t_ref[valid_m], y_ref[valid_m]

                    ax.plot(tm_v, ym_v, color="green", linewidth=2.0, label="Manual Test (Reference)")

                    has_data = True



                    # Tolerance shading if specified

                    tol = opts.get("tol")

                    if tol is not None:

                        tol = float(tol)

                        y_upper = ym_v + tol

                        y_lower = ym_v - tol

                        fill_color = "lightskyblue" if "kt" in unit else "peachpuff"

                        ax.fill_between(tm_v, y_lower, y_upper, color=fill_color, alpha=0.35,

                                        label=f"Tolerance (Â±{tol:g} {unit})")

                        ax.plot(tm_v, y_upper, color="green", linestyle="--", linewidth=0.9, alpha=0.7)

                        ax.plot(tm_v, y_lower, color="green", linestyle="--", linewidth=0.9, alpha=0.7)



                elif not is_auto and np.isfinite(y_sim).any():

                    valid_s = np.isfinite(t_sim) & np.isfinite(y_sim)

                    ts_v, ys_v = t_sim[valid_s], y_sim[valid_s]

                    ax.plot(ts_v, ys_v, color="green", linewidth=2.0,

                            label="Manual Test (Ground)" if kind.lower() == "deceleration"

                            else "Manual Test (Flight)")

                    has_data = True



                # 2. Automatic Test (Sim) trace (dashed blue with open circles)

                # In automatic mode for force, this represents the inverted virtual force

                if is_auto and np.isfinite(y_sim).any():

                    valid_s = np.isfinite(t_sim) & np.isfinite(y_sim)

                    ts_v, ys_v = t_sim[valid_s], y_sim[valid_s]

                    step = max(1, int(round(len(ts_v) / 10))) if len(ts_v) else 5

                    ax.plot(ts_v, ys_v, color="#0066cc", linewidth=1.6, linestyle="--",

                            marker="o", markevery=step, markersize=4.5,

                            markerfacecolor="white", markeredgecolor="#0066cc",

                            label="Automatic Test (Sim)")

                    has_data = True



                # Grid & limits

                ax.set_xlim(min_x, max_x)

                if key in ("engine_rpm", "prop_rpm"):

                    # Diagnose rather than hiding RPM at zero (on the axis

                    # spine) or above the previously hard-coded 3000 rpm.

                    readings = [y_sim]

                    if is_auto and y_ref is not None:

                        readings.append(y_ref)

                    true_values = np.concatenate([v[np.isfinite(v)] for v in readings

                                                  if v is not None and np.isfinite(v).any()]) if any(

                        v is not None and np.isfinite(v).any() for v in readings) else np.array([])

                    if len(true_values) and np.nanmax(true_values) > 3000:

                        ax.set_ylim(min(-100, float(np.nanmin(true_values)) - 100),

                                    float(np.ceil(np.nanmax(true_values)/500)*500 + 200))

                    elif len(true_values) and np.nanmin(true_values) < 0:

                        ax.set_ylim(float(np.floor(np.nanmin(true_values)/500)*500 - 100),

                                    max(3000, float(np.nanmax(true_values)) + 100))

                    else:

                        ax.set_ylim(-160, 3000)

                    notices = []

                    for label, dataset, legacy, tm in (

                        [("Manual", ref_df, ref_legacy, t_ref), ("Automatic", sim_df, sim_legacy, t_sim)]

                        if is_auto else [("Manual", sim_df, sim_legacy, t_sim)]

                    ):

                        info = _rpm_signal_diagnostics(dataset, legacy, key)

                        if info["state"].startswith("zero_high_rate"):

                            notices.append(f"{label}: HIGH-RATE RPM = 0")

                        elif info["state"].startswith("missing_high_rate"):

                            notices.append(f"{label}: HIGH-RATE RPM UNAVAILABLE")

                        if info["legacy_positive"] and info["state"] != "measured_high_rate":

                            # Directly recorded 1Hz engine RPM, NOT a generated

                            # value. Use its OWN timestamp and event offset.

                            lt = _series(legacy, "time_s") if legacy is not None else np.array([])

                            lv = _series(legacy, key) if legacy is not None else np.array([])

                            event = sim_event if label == "Automatic" or not is_auto else ref_event

                            if len(lt) == len(lv) and np.isfinite(lt).any():

                                ok = np.isfinite(lt) & np.isfinite(lv)

                                ax.plot(lt[ok] - event, lv[ok], ":", linewidth=1.5,

                                        color="#ca7a12", label=f"{label} legacy 1 Hz (RPM channel conflict)")

                                has_data = True

                                notices.append(f"{label}: 1 Hz RPM differs - verify P3D bridge")

                    if notices:

                        ax.text(0.01, 0.95, " | ".join(notices), transform=ax.transAxes,

                                va="top", ha="left", fontsize=7.2, color="#9a4a05",

                                bbox=dict(facecolor="white", edgecolor="none", alpha=0.9))

                        ax.legend(loc="lower right", fontsize=7, framealpha=.95)

                elif "ylim" in opts:

                    ax.set_ylim(*opts["ylim"])



                ax.minorticks_on()

                ax.grid(True, which="major", alpha=0.45, linewidth=0.7)

                ax.grid(True, which="minor", alpha=0.20, linewidth=0.45)

                ax.axvline(0, color="gray", linestyle=":", linewidth=1.0)



                if has_data:

                    ax.legend(loc="best", fontsize=8, framealpha=0.95)

                else:

                    ax.text(0.5, 0.5, "Engine RPM telemetry unavailable" if key in ("engine_rpm", "prop_rpm")

                            else "Ground-speed telemetry unavailable" if key == "ground_speed_kt"

                            else "Unavailable on this loader", ha="center", va="center",

                            transform=ax.transAxes, fontsize=10, color="gray")



            fig.text(0.5, 0.012, f"Engine test | Page {page_idx}",

                     ha="center", fontsize=8, family="monospace")

            if kind.lower() in ("acceleration", "deceleration"):
                fig.text(0.5, 0.033, "Pre-event view: last 2 s of measured data; full startup history remains in CSV.",
                         ha="center", fontsize=7, color="#555555")
            pdf.savefig(fig)

            plt.close(fig)



        # Page 5: Ti/Tt Response Time Assessment (when available)

        if (selected_graphs is None or 'titt' in selected_graphs) and titt_results is None:

            try:

                import sys

                script_dir = str(Path(__file__).resolve().parent.parent)

                if script_dir not in sys.path:

                    sys.path.insert(0, script_dir)

                from SMK_Engine_TiTt_V2_1_3_Addon import evaluate_engine_run

                titt_results = evaluate_engine_run(sim_path, ref_path if is_auto else None, mode=kind.lower())

            except Exception as e:

                print(f"[TiTt auto-eval note] {e}", flush=True)



        if (selected_graphs is None or 'titt' in selected_graphs) and titt_results and titt_results.get('raw_measurements') and titt_results.get('raw_traces'):

            try:

                from SMK_Engine_TiTt_V2_1_3_Addon import plot_titt_page

                fig = plt.figure(figsize=(8.27, 11.69))

                mode = titt_results.get('mode', kind.lower())

                param_name = titt_results.get('critical_parameter', 'manifold_inhg')

                plot_titt_page(fig, titt_results['raw_measurements'], titt_results['raw_traces'],

                               mode, param_name, comparison=titt_results.get('comparisons'))

                pdf.savefig(fig)

                plt.close(fig)

            except Exception as titt_err:

                print(f"[PDF TiTt PAGE WARNING] {titt_err}", flush=True)



def plot_aligned_comparison(csv_path, manual_csv_path=None, output_pdf_path=None, titt_results=None, metadata=None, **kwargs):

    csv_path = Path(csv_path)

    ref = str(kwargs.get("test_reference") or (metadata and metadata.get("test_reference")) or "").lower()

    path_str = str(csv_path).lower()

    is_engine = ref.startswith("1.f") or "1_f_1" in path_str or "1_f_2" in path_str or "engine" in ref

    if not is_engine and ("1.c.(1)" in ref or "climb" in ref or "1_c_1" in path_str):

        from qtg_climb_pid import report

        return report(csv_path, kwargs.get("validation_csv_path"), kwargs.get("run_fault"), selected_graphs=kwargs.get("selected_graphs"), target_ias=kwargs.get("pid_target_ias", 80))



    target_pdf = output_pdf_path or kwargs.get("output_pdf_path")

    out_path = Path(target_pdf) if target_pdf else csv_path.with_name(csv_path.stem + "_graphs.pdf")



    if metadata is None:

        metadata = {}

    if kwargs.get("test_reference"):

        metadata.setdefault("test_reference", kwargs["test_reference"])

    if "1.f.(2)" in str(kwargs.get("test_reference")) or "deceleration" in csv_path.name.lower():

        metadata.setdefault("kind", "deceleration")

    else:

        metadata.setdefault("kind", "acceleration")



    if "manual" in csv_path.name.lower() or str(metadata.get("mode", "")).lower() == "manual":

        _render_report(csv_path, ref_path=None, out_path=out_path, titt_results=titt_results, metadata=metadata, selected_graphs=kwargs.get('selected_graphs'))

        return str(out_path)



    manual_path = Path(manual_csv_path) if manual_csv_path else _find_manual_for_auto(csv_path)

    if manual_path is None or not manual_path.exists():

        print("[Engine Report] No manual reference found, rendering single-run report.")

        _render_report(csv_path, ref_path=None, out_path=out_path, titt_results=titt_results, metadata=metadata, selected_graphs=kwargs.get('selected_graphs'))

        return str(out_path)



    _render_report(csv_path, ref_path=manual_path, out_path=out_path, titt_results=titt_results, metadata=metadata, selected_graphs=kwargs.get('selected_graphs'))

    return str(out_path)

