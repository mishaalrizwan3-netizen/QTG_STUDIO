from pathlib import Path

import math

import re

import datetime

import pandas as pd

import numpy as np

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt

from matplotlib.backends.backend_pdf import PdfPages



DOC_NUMBER = "161310G00000-00"

DOC_VERSION = "008"



def _read_csv(path):

    return pd.read_csv(path)



def _safe(df, key, default=np.nan):

    if key not in df.columns or df.empty:

        return default

    try:

        return float(df.iloc[0][key])

    except Exception:

        return default



def _fmt(v):

    try:

        v = float(v)

        if not math.isfinite(v):

            return "-"

        av = abs(v)

        if av >= 1000: return f"{v:.0f}"

        if av >= 100: return f"{v:.1f}"

        if av >= 10: return f"{v:.1f}"

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

    auto_path = Path(auto_path)

    folder = auto_path.parent

    # 1. Check if companion json references the exact manual file

    for suffix in ["_init_conditions.json", "_initial_state.json", "_manifest.json"]:

        cand_json = auto_path.with_name(auto_path.stem + suffix)

        if cand_json.exists():

            try:

                import json

                with open(cand_json, "r", encoding="utf-8") as jf:

                    data = json.load(jf)

                    ref_f = data.get("manual_ref_csv") or data.get("reference_csv") or data.get("manual_file")

                    if ref_f and Path(ref_f).exists():

                        return Path(ref_f)

            except Exception:

                pass



    # 2. Search folder for valid manual flight test CSVs sorted by mtime

    def is_valid_flight(p):

        name_l = p.name.lower()

        if any(k in name_l for k in ["diagnostic", "commissioning", "legacy_plot", "test_qtg"]):

            return False

        return "manual" in name_l and p.suffix == ".csv"



    candidates = [p for p in folder.glob("*.csv") if is_valid_flight(p)]

    candidates.sort(key=lambda p: p.stat().st_mtime, reverse=True)

    if candidates:

        return candidates[0]

    return None



def _series(df, key):

    if key not in df.columns:

        return np.full(len(df), np.nan)

    return pd.to_numeric(df[key], errors="coerce").to_numpy()



def _virtual_force_series(df, axis):

    """

    Return native BRUNNER Virtual Force in engineering units (Newtons).



    Automatic bound-report staging uses the AUTHORITATIVE Manual CSV, while

    the Automatic side is the compatibility *_legacy_plot.csv.  The former

    stores the native channel as:

        CLS <Axis> Virtual Force (eng)

    and the latter stores the report alias as:

        <Axis> Virtual Force (N)



    Accept both names so Manual and Automatic are always plotted.

    If the logged virtual force is a normalized ratio (-1.0 to +1.0),

    it is scaled to full-scale Newtons (75N Pitch, 40N Roll, 50N Yaw)

    so both Manual Applied Force and Automatic Virtual Force display on

    the same physical Newton scale.

    """

    candidates = (

        f"{axis} Virtual Force 0x51 (N)",

        f"{axis} Virtual Force (N)",

        f"CLS {axis} Virtual Force (eng)",

        f"{axis}_Auto_Force",

    )

    for key in candidates:

        if key in df.columns:

            values = pd.to_numeric(df[key], errors="coerce").to_numpy()

            finite = values[np.isfinite(values)]

            if len(finite) > 0:

                max_abs = float(np.max(np.abs(finite)))

                if 0.001 < max_abs <= 1.5:

                    scale = {"Pitch": 75.0, "Roll": 40.0, "Yaw": 50.0}.get(axis, 50.0)

                    return values * scale

                return values

    return np.full(len(df), np.nan)



def _manual_applied_force(df, axis):

    """Return Manual applied/measured force in Newtons."""

    mapping = {

        "Pitch": "Joystick Pitch Force (N)",

        "Roll": "Joystick Roll Force (N)",

        "Yaw": "Rudder Pedal Force (N)",

    }

    # These channels are already measured Newtons. Amplitude cannot identify units.
    return _series(df, mapping[axis])



def _automatic_virtual_force_inverted(df, axis):

    """Return Automatic BRUNNER Virtual Force 0x51 with sign inverted."""

    return -1.0 * _virtual_force_series(df, axis)



def _unwrap_heading(values):

    vals = np.asarray(values, dtype=float)

    out = np.array(vals, copy=True)

    last_raw = None

    last_unwrapped = None

    for i, raw in enumerate(vals):

        if not np.isfinite(raw):

            continue

        if last_raw is None:

            last_raw = raw

            last_unwrapped = raw

            out[i] = raw

        else:

            last_unwrapped = last_unwrapped + ((raw-last_raw+180.0)%360.0 - 180.0)

            out[i] = last_unwrapped

            last_raw = raw

    return out



def _style_axis(ax, ylabel):

    ax.grid(True, which="major", alpha=0.45, linewidth=0.7)

    ax.minorticks_on()

    ax.grid(True, which="minor", alpha=0.20, linewidth=0.45)

    ax.set_xlim(0, 75)

    ax.set_xticks(np.arange(0, 76, 5))

    ax.set_xlabel("Time (s)", fontweight="bold")

    ax.set_ylabel(ylabel, fontweight="bold")



def _plot_manual(ax, t, y, ylabel, target=None, tol=None, altitude=False):

    ax.plot(t, y, color="green", linewidth=2.2, label="Manual Test (Flight)")

    if target is not None:

        target = float(target)

        ax.axhline(target, color="gray", linestyle=":", linewidth=1.3,

                   label=("POH Speed (80 kt)" if "Airspeed" in ylabel else "POH ROC (820 fpm)"))

    if tol is not None:

        # The band follows the recorded flight, independently of the fixed guide.

        tol = float(tol)

        reference = np.asarray(y, dtype=float)

        band_color = "lightskyblue" if "Airspeed" in ylabel else "peachpuff"

        ax.fill_between(t, reference-tol, reference+tol,

                        color=band_color, alpha=0.45, zorder=1,

                        label=f"Tolerance (±{tol:g} {'kt' if 'Airspeed' in ylabel else 'fpm'})")

        ax.plot(t, reference-tol, color="gray", linestyle="--", linewidth=0.8)

        ax.plot(t, reference+tol, color="gray", linestyle="--", linewidth=0.8)

    if altitude:

        ax.fill_between(t, y-1000, y, color="mediumseagreen", alpha=0.22,

                        label="Tolerance (-1000 ft)")

        ax.plot(t, y-1000, color="seagreen", linestyle="--", linewidth=1.0)

    _style_axis(ax, ylabel)

    ax.legend(loc="best", fontsize=8, framealpha=0.95)



def _plot_auto(ax, t_m, y_m, t_a, y_a, ylabel,

               target=None, target_label=None, tol=None, altitude=False,

               manual_label="Manual Test (Reference)", auto_label="Automatic Test (Sim)"):

    ax.plot(t_m, y_m, color="green", linewidth=2.2, label=manual_label)

    valid_a = np.asarray(y_a, dtype=float)

    if np.count_nonzero(np.isfinite(valid_a)) > 1:

        ax.plot(t_a, y_a, color="#0066ff", linewidth=1.8, linestyle="--",

                marker="o", markevery=0.065, markersize=3.0, markerfacecolor="white",

                label=auto_label)

    else:

        ax.plot(t_a, y_a, color="#0066ff", linewidth=1.8, linestyle="--",

                label=auto_label)

    if target is not None:

        target = float(target)

        ax.axhline(target, color="gray", linestyle=":", linewidth=1.3,

                   label=target_label or "Target")

    if tol is not None:

        # Use Manual times and values: never center the band on Automatic or

        # on the fixed POH guide, and never modify either recorded curve.

        tol = float(tol)

        reference = np.asarray(y_m, dtype=float)

        band_color = "lightskyblue" if "Airspeed" in ylabel else "peachpuff"

        ax.fill_between(t_m, reference-tol, reference+tol,

                        color=band_color, alpha=0.45, zorder=1,

                        label=f"Tolerance (±{tol:g} {'kt' if 'Airspeed' in ylabel else 'fpm'})")

        ax.plot(t_m, reference-tol, color="gray", linestyle="--", linewidth=0.8)

        ax.plot(t_m, reference+tol, color="gray", linestyle="--", linewidth=0.8)

    if altitude:

        ax.fill_between(t_m, y_m-1000, y_m, color="mediumseagreen", alpha=0.22,

                        label="Tolerance (-1000 ft)")

        ax.plot(t_m, y_m-1000, color="seagreen", linestyle="--", linewidth=1.0)

    _style_axis(ax, ylabel)

    ax.legend(loc="best", fontsize=8, framealpha=0.95)



def _initial_page(pdf, mode, source_path, sim_df, ref_df=None):

    date_text, time_text = _parse_stamp(source_path)

    fig = plt.figure(figsize=(8.27, 11.69))

    ax = fig.add_axes((0.0, 0.0, 1.0, 1.0))

    ax.axis("off"); ax.set_xlim(0,1); ax.set_ylim(0,1)



    ax.add_patch(plt.Rectangle((0.05,0.92),0.90,0.06,fill=False,linewidth=1.0))

    ax.plot([0.39,0.39],[0.92,0.98], color="black", linewidth=0.7)

    ax.plot([0.64,0.64],[0.92,0.98], color="black", linewidth=0.7)

    ax.text(0.06,0.968,f"Doc Number: {DOC_NUMBER}",fontsize=7,family="monospace",va="top")

    ax.text(0.41,0.968,f"Doc Version: {DOC_VERSION}",fontsize=7,family="monospace",va="top")

    ax.text(0.66,0.968,f"Date: {date_text}",fontsize=7,family="monospace",va="top")



    is_auto = mode == "auto"

    ax.text(0.5,0.946,f"Super Mushshak {'Automatic' if is_auto else 'Manual'} Flight Test",

            fontsize=9,family="monospace",ha="center",fontweight="bold")

    ax.text(0.5,0.913,

            f"Prepar3D v5 | Rahwali Airfield (OPRW) | {'Automatic' if is_auto else 'Manual'} Testing Mode",

            fontsize=7,family="monospace",ha="center")

    ax.text(0.5,0.885,"Super Mushshak Logger - ver. 1.0",

            fontsize=8,family="monospace",ha="center",fontweight="bold")

    ax.text(0.5,0.868,

            f"Super Mushshak {'Airborne Automatic' if is_auto else 'Ground/Airborne Manual'} Testing Suite",

            fontsize=7,family="monospace",ha="center")

    ax.text(0.5,0.850,f"Flight Test File: {Path(source_path).name}",

            fontsize=6.4,family="monospace",ha="center")

    ax.text(0.5,0.832,f"ONLINE, {'AUTOMATIC' if is_auto else 'MANUAL'}",

            fontsize=7,family="monospace",ha="center",fontweight="bold")

    ax.text(0.5,0.814,f"Test Date: {date_text}    Test Time: {time_text}",

            fontsize=6.7,family="monospace",ha="center")

    ax.text(0.05,0.785,"Initial Test Conditions",fontsize=8.5,family="monospace",fontweight="bold")

    ax.plot([0.05,0.95],[0.776,0.776],color="black",linewidth=0.7)



    if is_auto:

        ax.text(0.105,0.758,"Validation",fontsize=6.5,family="monospace",ha="right")

        ax.text(0.165,0.758,"Sim",fontsize=6.5,family="monospace",ha="right")

        ax.text(0.575,0.758,"Validation",fontsize=6.5,family="monospace",ha="right")

        ax.text(0.635,0.758,"Sim",fontsize=6.5,family="monospace",ha="right")

    else:

        ax.text(0.145,0.758,"Sim",fontsize=6.5,family="monospace",ha="right")

        ax.text(0.615,0.758,"Sim",fontsize=6.5,family="monospace",ha="right")



    left_sections = [

        ("*** Weight and Balance ***", [

            ("Gross Wt (kg)","Gross Wt (kg)"),

            ("Total Fuel Quantity (kg)","Total Fuel Quantity (kg)"),

            ("CG Pos in X Axis (mm)","CG Pos in X Axis (mm)"),

            ("CG Pos in Y Axis (mm)","CG Pos in Y Axis (mm)")]),

        ("*** Control Parameters ***", [

            ("Pitch Control Position (raw)","Joystick Pitch Position (mm)"),

            ("Roll Control Position (raw)","Joystick Roll Position (mm)"),

            ("Rudder Pedal Position (raw)","Rudder Pedal Position (mm)"),

            ("Throttle Position (%)","Throttle 1 Position (%)"),

            ("Pitch Force: Applied/Virtual (N)" if is_auto else "Pitch Applied Force (N)","Joystick Pitch Force (N)"),

            ("Roll Force: Applied/Virtual (N)" if is_auto else "Roll Applied Force (N)","Joystick Roll Force (N)"),

            ("Yaw Force: Applied/Virtual (N)" if is_auto else "Yaw Applied Force (N)","Rudder Pedal Force (N)")]),

        ("*** Environment Parameters ***", [

            ("Baro Press (hPa)","Baro Press (hPa)"),

            ("OAT (deg C)","OAT (deg C)"),

            ("Wind Dir (mag,deg)","Wind Dir (mag,deg)"),

            ("Wind Spd (kt)","Wind Spd (kt)")]),

        ("*** Engine Parameters (Piston Engine) ***", [

            ("Engine Speed (RPM)","Engine 1 RPM"),

            ("Throttle Position (%)","Throttle 1 Position (%)"),

            ("Fuel Flow (pph)","Fuel Flow (pph)"),

            ("Exhaust Gas Temp (deg C)","EGT (deg C)")]),

    ]

    right_sections = [

        ("*** Flight Parameters ***", [

            ("Indicated Airspeed (kt)","Indicated Airspeed (kts)"),

            ("Ground Speed (kt)","Ground Speed (kt)"),

            ("Mach Number (-)","Mach Number (-)"),

            ("Pressure Alt (ft)","Pressure Alt (ft)"),

            ("Radar Alt (ft)","Radar Alt (ft)"),

            ("Height of Terrain (ft)","Height of Terrain (ft)"),

            ("Vertical Velocity (ft/min)","Rate of Climb/Descent (fpm)"),

            ("Pitch Angle (deg)","Pitch Angle (deg)"),

            ("Bank Angle (deg)","Bank Angle (deg)"),

            ("Heading (deg)","Magnetic Heading (deg)"),

            ("Aircraft Angle of Attack (deg)","Aircraft Angle of Attack (deg)"),

            ("Sideslip Angle (deg)","Sideslip Angle (deg)"),

            ("Track Angle (deg)","Track Angle (deg)"),

            ("Roll Rate (deg/s)","Roll Rate (deg/s)"),

            ("Pitch Rate (deg/s)","Pitch Rate (deg/s)"),

            ("Yaw Rate (deg/s)","Yaw Rate (deg/s)"),

            ("X Body Acceleration (g)","X Body Acceleration (g)"),

            ("Y Body Acceleration (g)","Y Body Acceleration (g)"),

            ("Z Body Acceleration (g)","Z Body Acceleration (g)")]),

    ]



    def values(key):

        simv = _fmt(_safe(sim_df,key))

        force_axes = {"Joystick Pitch Force (N)": "Pitch", "Joystick Roll Force (N)": "Roll", "Rudder Pedal Force (N)": "Yaw"}

        if is_auto and key in force_axes:

            force = _automatic_virtual_force_inverted(sim_df, force_axes[key])

            simv = _fmt(force[0]) if len(force) else "-"

            if ref_df is not None:

                ref_force = _manual_applied_force(ref_df, force_axes[key])

                refv = _fmt(ref_force[0]) if len(ref_force) else "-"

                return refv, simv

        if is_auto and ref_df is not None:

            return _fmt(_safe(ref_df,key)), simv

        return "", simv



    def draw(x, y, sections):

        for title, rows in sections:

            ax.text(x,y,title,fontsize=6.9,family="monospace",fontweight="bold")

            y -= 0.0155

            for label,key in rows:

                refv,simv = values(key)

                if is_auto:

                    ax.text(x+0.055,y,refv,fontsize=6.15,family="monospace",ha="right")

                    ax.text(x+0.113,y,simv,fontsize=6.15,family="monospace",ha="right")

                else:

                    ax.text(x+0.100,y,simv,fontsize=6.15,family="monospace",ha="right")

                ax.text(x+0.132,y,label,fontsize=6.15,family="monospace")

                y -= 0.0147

            y -= 0.004

    draw(0.05,0.738,left_sections)

    draw(0.52,0.738,right_sections)



    pdf.savefig(fig)

    plt.close(fig)




BRUNNER_CONTROL_GRAPH_KEYS = {
    "pitch_position", "pitch_force",
    "roll_position", "roll_force",
    "rudder_position", "rudder_force",
}


def _append_control_pages(pdf, sim_df, sim_time, selected_graphs=None,
                          manual_df=None, manual_time=None):
    """Render selected BRUNNER graphs; recording itself is never filtered."""
    selected = (
        set(BRUNNER_CONTROL_GRAPH_KEYS)
        if selected_graphs is None
        else {str(item) for item in selected_graphs} & BRUNNER_CONTROL_GRAPH_KEYS
    )
    groups = [
        ("Group 4 (BRUNNER Pitch and Roll)", [
            ("pitch_position", "Joystick Pitch Position (mm)", "position", "Pitch"),
            ("pitch_force", "Pitch Applied / Virtual Force (N)", "force", "Pitch"),
            ("roll_position", "Joystick Roll Position (mm)", "position", "Roll"),
            ("roll_force", "Roll Applied / Virtual Force (N)", "force", "Roll"),
        ]),
        ("Group 5 (BRUNNER Rudder)", [
            ("rudder_position", "Rudder Pedal Position (mm)", "position", "Yaw"),
            ("rudder_force", "Yaw Applied / Virtual Force (N)", "force", "Yaw"),
        ]),
    ]

    for title, definitions in groups:
        chosen = [item for item in definitions if item[0] in selected]
        if not chosen:
            continue
        fig, axes = plt.subplots(
            len(chosen), 1, figsize=(8.27, 11.69), constrained_layout=True
        )
        axes = np.atleast_1d(axes).ravel()
        fig.suptitle(title, fontsize=15, fontweight="bold")

        for ax, (key, label, kind, axis_name) in zip(axes, chosen):
            if kind == "position":
                column = "Rudder Pedal Position (mm)" if axis_name == "Yaw" else f"Joystick {axis_name} Position (mm)"
                y_sim = _series(sim_df, column)
                y_ref = _series(manual_df, column) if manual_df is not None else None
            else:
                y_sim = (
                    _automatic_virtual_force_inverted(sim_df, axis_name)
                    if manual_df is not None
                    else _manual_applied_force(sim_df, axis_name)
                )
                y_ref = _manual_applied_force(manual_df, axis_name) if manual_df is not None else None

            if manual_df is not None and y_ref is not None:
                _plot_auto(
                    ax, manual_time, y_ref, sim_time, y_sim, label,
                    manual_label="Manual Test (Reference)",
                    auto_label="Automatic Test (Sim)",
                )
            else:
                _plot_manual(ax, sim_time, y_sim, label)
        pdf.savefig(fig)
        plt.close(fig)


def _manual_report(csv_path, out_path, selected_graphs=None):

    df = _read_csv(csv_path)

    t = _series(df,"Time (s)")

    with PdfPages(out_path) as pdf:

        _initial_page(pdf,"manual",csv_path,df)



        fig,axes=plt.subplots(3,1,figsize=(8.27,11.69),constrained_layout=True)

        fig.suptitle("1st Quadrant Combined Group 1 (Airspeed, Climb, Altitude)",fontsize=16,fontweight="bold")

        _plot_manual(axes[0],t,_series(df,"Indicated Airspeed (kts)"),"Indicated Airspeed\n(kts)",target=80,tol=3)

        _plot_manual(axes[1],t,_series(df,"Rate of Climb/Descent (fpm)"),"Rate of Climb/Descent\n(fpm)",target=820,tol=100)

        _plot_manual(axes[2],t,_series(df,"Indicated Altitude (ft)"),"Indicated Altitude\n(ft)",altitude=True)

        axes[2].set_xlabel("Time (seconds)",fontweight="bold")

        pdf.savefig(fig); plt.close(fig)



        fig,axes=plt.subplots(5,1,figsize=(8.27,11.69),constrained_layout=True)

        fig.suptitle("1st Quadrant Combined Group 2 (Pitch, Bank, Yaw, Heading)",fontsize=16,fontweight="bold")

        for ax,key,label in [

            (axes[0],"Pitch Angle (deg)","Pitch Angle\n(deg)"),

            (axes[1],"Bank Angle (deg)","Bank Angle\n(deg)"),

            (axes[2],"Magnetic Heading (deg)","Magnetic Heading\n(deg)"),

            (axes[3],"Pitch Rate (deg/s)","Pitch Rate\n(deg/s)"),

            (axes[4],"Yaw Rate (deg/s)","Yaw Rate\n(deg/s)")]:

            y=_series(df,key)

            if key=="Magnetic Heading (deg)": y=_unwrap_heading(y)

            _plot_manual(ax,t,y,label)

        pdf.savefig(fig); plt.close(fig)



        fig,axes=plt.subplots(3,1,figsize=(8.27,11.69),constrained_layout=True)

        fig.suptitle("1st Quadrant Combined Group 3 (Propeller, Engine, Throttle)",fontsize=16,fontweight="bold")

        _plot_manual(axes[0],t,_series(df,"Propeller 1 RPM"),"Propeller 1 RPM")

        _plot_manual(axes[1],t,_series(df,"Engine 1 RPM"),"Engine 1 RPM")

        _plot_manual(axes[2],t,_series(df,"Throttle 1 Position (%)"),"Throttle 1 Position\n(%)")

        pdf.savefig(fig); plt.close(fig)



        _append_control_pages(pdf, df, t, selected_graphs=selected_graphs)


def _auto_report(auto_path, manual_path, out_path, selected_graphs=None):

    a=_read_csv(auto_path); m=_read_csv(manual_path)

    ta=_series(a,"Time (s)"); tm=_series(m,"Time (s)")

    with PdfPages(out_path) as pdf:

        _initial_page(pdf,"auto",auto_path,a,m)



        fig,axes=plt.subplots(3,1,figsize=(8.27,11.69),constrained_layout=True)

        fig.suptitle("1st Quadrant Combined Group 1 (Airspeed, Climb, Altitude)",fontsize=16,fontweight="bold")

        _plot_auto(axes[0],tm,_series(m,"Indicated Airspeed (kts)"),ta,_series(a,"Indicated Airspeed (kts)"),

                   "Indicated Airspeed\n(kts)",target=80,target_label="POH Speed (80 kt)",tol=3)

        _plot_auto(axes[1],tm,_series(m,"Rate of Climb/Descent (fpm)"),ta,_series(a,"Rate of Climb/Descent (fpm)"),

                   "Rate of Climb/Descent\n(fpm)",target=820,target_label="POH ROC (820 fpm)",tol=100)

        _plot_auto(axes[2],tm,_series(m,"Indicated Altitude (ft)"),ta,_series(a,"Indicated Altitude (ft)"),

                   "Indicated Altitude\n(ft)",altitude=True)

        axes[2].set_xlabel("Time (seconds)",fontweight="bold")

        pdf.savefig(fig); plt.close(fig)



        fig,axes=plt.subplots(5,1,figsize=(8.27,11.69),constrained_layout=True)

        fig.suptitle("1st Quadrant Combined Group 2 (Pitch, Bank, Yaw, Heading)",fontsize=16,fontweight="bold")

        for ax,key,label in [

            (axes[0],"Pitch Angle (deg)","Pitch Angle\n(deg)"),

            (axes[1],"Bank Angle (deg)","Bank Angle\n(deg)"),

            (axes[2],"Magnetic Heading (deg)","Magnetic Heading\n(deg)"),

            (axes[3],"Pitch Rate (deg/s)","Pitch Rate\n(deg/s)"),

            (axes[4],"Yaw Rate (deg/s)","Yaw Rate\n(deg/s)")]:

            ym=_series(m,key); ya=_series(a,key)

            if key=="Magnetic Heading (deg)":

                ym=_unwrap_heading(ym); ya=_unwrap_heading(ya)

            _plot_auto(ax,tm,ym,ta,ya,label)

        pdf.savefig(fig); plt.close(fig)



        fig,axes=plt.subplots(3,1,figsize=(8.27,11.69),constrained_layout=True)

        fig.suptitle("1st Quadrant Combined Group 3 (Propeller, Engine, Throttle)",fontsize=16,fontweight="bold")

        _plot_auto(axes[0],tm,_series(m,"Propeller 1 RPM"),ta,_series(a,"Propeller 1 RPM"),"Propeller 1 RPM")

        _plot_auto(axes[1],tm,_series(m,"Engine 1 RPM"),ta,_series(a,"Engine 1 RPM"),"Engine 1 RPM")

        _plot_auto(axes[2],tm,_series(m,"Throttle 1 Position (%)"),ta,_series(a,"Throttle 1 Position (%)"),"Throttle 1 Position\n(%)")

        pdf.savefig(fig); plt.close(fig)



        _append_control_pages(pdf, a, ta, selected_graphs=selected_graphs, manual_df=m, manual_time=tm)


def _is_engine_test(csv_path, kwargs):
    test_ref = str(kwargs.get('test_reference', '')).lower()
    path_str = str(csv_path).lower()
    return ('1.f.(1)' in test_ref or '1.f.(2)' in test_ref or '1.f.1' in test_ref or '1.f.2' in test_ref or
            'engine' in test_ref or 'engine' in path_str or '1_f_1' in path_str or '1_f_2' in path_str)

def _get_engine_plot_module():
    import importlib.util
    engine_plot_path = Path(__file__).resolve().parent.parent / "QTG_Studio_Antigravity_Single_Main" / "plot_graphs.py"
    if not engine_plot_path.exists():
        engine_plot_path = Path(r"C:\Users\Qasim 007\Desktop\QTG_Studio\QTG_Studio_Antigravity_Single_Main\plot_graphs.py")
    spec = importlib.util.spec_from_file_location("engine_plot_graphs", str(engine_plot_path))
    engine_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(engine_module)
    return engine_module

def plot_aligned_comparison(csv_path, manual_csv_path=None, output_pdf_path=None, **kwargs):
    if _is_engine_test(csv_path, kwargs):
        try:
            engine_mod = _get_engine_plot_module()
            return engine_mod.plot_aligned_comparison(csv_path, manual_csv_path=manual_csv_path, output_pdf_path=output_pdf_path, **kwargs)
        except Exception as e:
            print(f"[plot_graphs Engine delegation note] {e}", flush=True)


    csv_path = Path(csv_path)

    if output_pdf_path:

        out_path = Path(output_pdf_path)

    else:

        out_path = csv_path.with_name(csv_path.stem + "_graphs.pdf")

    if "manual" in csv_path.name.lower():

        _manual_report(csv_path, out_path, selected_graphs=kwargs.get('selected_graphs'))

        return str(out_path)

    manual_path = Path(manual_csv_path) if manual_csv_path else _find_manual_for_auto(csv_path)

    if manual_path is None or not manual_path.exists():

        manual_path = _find_manual_for_auto(csv_path)

    if manual_path is None or not manual_path.exists():

        raise FileNotFoundError("No Manual reference CSV found for Automatic report.")

    _auto_report(csv_path, manual_path, out_path, selected_graphs=kwargs.get('selected_graphs'))

    return str(out_path)

