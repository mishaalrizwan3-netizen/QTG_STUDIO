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



def _initial_page(pdf, mode, source_path, sim_df, ref_df=None, evaluation_result=None, virtual_forces=False):

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



    is_auto = mode in ("auto","pid")
    display_mode = "PID Automatic" if mode=="pid" else ("Automatic" if is_auto else "Manual")

    ax.text(0.5,0.946,f"Super Mushshak {display_mode} Flight Test",

            fontsize=9,family="monospace",ha="center",fontweight="bold")

    ax.text(0.5,0.913,

            f"Prepar3D v5 | Rahwali Airfield (OPRW) | {display_mode} Testing Mode",

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



    if virtual_forces or mode=="pid":
        force_keys={'Joystick Pitch Force (N)':'Pitch Virtual Force 0x51 (N)','Joystick Roll Force (N)':'Roll Virtual Force 0x51 (N)','Rudder Pedal Force (N)':'Yaw Virtual Force 0x51 (N)'}
        left_sections=[(title,[(label.replace('Applied Force','Virtual Force'),force_keys.get(key,key)) for label,key in rows]) for title,rows in left_sections]

    def values(key):

        simv = _fmt(_safe(sim_df,key))

        force_axes = {"Joystick Pitch Force (N)": "Pitch", "Joystick Roll Force (N)": "Roll", "Rudder Pedal Force (N)": "Yaw"}

        if mode=="auto" and key in force_axes:

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



    if evaluation_result:
        ax.text(.05,.025,"Validation: "+evaluation_result,fontsize=7,family="monospace",fontweight="bold")
    pdf.savefig(fig)

    plt.close(fig)



def _manual_report(csv_path, out_path):

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



        fig,axes=plt.subplots(4,1,figsize=(8.27,11.69),constrained_layout=True)

        fig.suptitle("1st Quadrant Combined Group 4 (Joystick Position & Applied Force)",fontsize=15,fontweight="bold")

        _plot_manual(axes[0],t,_series(df,"Joystick Pitch Position (mm)"),"Joystick Pitch Position\n(mm)")

        _plot_manual(axes[1],t,_manual_applied_force(df,"Pitch"),"Pitch Applied Force 0x21\n(N)")

        _plot_manual(axes[2],t,_series(df,"Joystick Roll Position (mm)"),"Joystick Roll Position\n(mm)")

        _plot_manual(axes[3],t,_manual_applied_force(df,"Roll"),"Roll Applied Force 0x21\n(N)")

        pdf.savefig(fig); plt.close(fig)



        fig,axes=plt.subplots(2,1,figsize=(8.27,11.69),constrained_layout=True)

        fig.suptitle("1st Quadrant Combined Group 5 (Rudder Position & Applied Force)",fontsize=15,fontweight="bold")

        _plot_manual(axes[0],t,_series(df,"Rudder Pedal Position (mm)"),"Rudder Pedal Position\n(mm)")

        _plot_manual(axes[1],t,_manual_applied_force(df,"Yaw"),"Yaw Applied Force 0x21\n(N)")

        pdf.savefig(fig); plt.close(fig)



def _draw_climb_airspeed_target(ax,target_ias,tolerance=3.):
    lower,upper=target_ias-tolerance,target_ias+tolerance
    ax.axhspan(lower,upper,color='lightskyblue',alpha=.22,
        label=f'IAS tolerance +/-{tolerance:g} kt ({lower:g}-{upper:g} kt)')
    ax.axhline(lower,color='steelblue',linestyle='--',linewidth=1.)
    ax.axhline(upper,color='steelblue',linestyle='--',linewidth=1.)
    ax.axhline(target_ias,color='gray',linestyle=':',linewidth=1.3,label=f'Target IAS ({target_ias:g} kt)')


def _format_climb_airspeed_axis(ax):
    from matplotlib.ticker import MaxNLocator, StrMethodFormatter
    import math
    low,high=ax.get_ylim()
    ax.set_ylim(math.floor(low),max(math.floor(low)+1,math.ceil(high)))
    ax.yaxis.set_major_locator(MaxNLocator(nbins=6,integer=True))
    ax.yaxis.set_major_formatter(StrMethodFormatter('{x:.0f}'))


def _pid_report(csv_path, out_path, validation_df=None, evaluation_result="NOT EVALUATED", target_ias=80, selected_graphs=None, mode="pid"):
    """Same six-page portrait graph groups as the supplied Manual report."""
    df=_read_csv(csv_path);t=_series(df,"Time (s)")
    if "Manifold Pressure (inHg)" not in df and "Engine Manifold Pressure (psi)" in df:
        df["Manifold Pressure (inHg)"]=pd.to_numeric(df["Engine Manifold Pressure (psi)"],errors="coerce")*2.0360206576
    groups=[
        ("1st Quadrant Combined Group 1 (Airspeed, Climb, Altitude)",[
            ("Indicated Airspeed (kts)","Indicated Airspeed\n(kts)"),
            ("Rate of Climb/Descent (fpm)","Rate of Climb/Descent\n(fpm)"),
            ("Indicated Altitude (ft)","Indicated Altitude\n(ft)")]),
        ("1st Quadrant Combined Group 2 (Pitch, Bank, Yaw, Heading)",[
            ("Pitch Angle (deg)","Pitch Angle\n(deg)"),("Bank Angle (deg)","Bank Angle\n(deg)"),
            ("Magnetic Heading (deg)","Magnetic Heading\n(deg)"),("Pitch Rate (deg/s)","Pitch Rate\n(deg/s)"),("Yaw Rate (deg/s)","Yaw Rate\n(deg/s)")]),
        ("1st Quadrant Combined Group 3 (Propeller, Engine, Throttle)",[
            ("Propeller 1 RPM","Propeller 1 RPM"),("Engine 1 RPM","Engine 1 RPM"),("Throttle 1 Position (%)","Throttle 1 Position\n(%)")]),
        ("1st Quadrant Combined Group 4 (Joystick Position & Virtual Force)",[
            ("Joystick Pitch Position (mm)","Joystick Pitch Position\n(mm)"),("Pitch Virtual Force 0x51 (N)","Pitch Virtual Force 0x51\n(N)"),
            ("Joystick Roll Position (mm)","Joystick Roll Position\n(mm)"),("Roll Virtual Force 0x51 (N)","Roll Virtual Force 0x51\n(N)")]),
        ("1st Quadrant Combined Group 5 (Rudder Position & Virtual Force)",[
            ("Rudder Pedal Position (mm)","Rudder Pedal Position\n(mm)"),("Yaw Virtual Force 0x51 (N)","Yaw Virtual Force 0x51\n(N)")])]
    mapping={
        'ias_kt':'Indicated Airspeed (kts)','vertical_speed_fpm':'Rate of Climb/Descent (fpm)',
        'altitude_msl_ft':'Indicated Altitude (ft)','height_agl_ft':'Radar Alt (ft)','ground_speed_kt':'Ground Speed (kt)',
        'pitch_deg':'Pitch Angle (deg)','bank_deg':'Bank Angle (deg)','heading_deg':'Magnetic Heading (deg)',
        'pitch_rate':'Pitch Rate (deg/s)','yaw_rate':'Yaw Rate (deg/s)','prop_rpm':'Propeller 1 RPM','engine_rpm':'Engine 1 RPM','throttle_pct':'Throttle 1 Position (%)',
        'manifold_inhg':'Manifold Pressure (inHg)','fuel_flow_pph':'Fuel Flow (pph)',
        'pitch_position':'Joystick Pitch Position (mm)','pitch_force':'Pitch Virtual Force 0x51 (N)',
        'roll_position':'Joystick Roll Position (mm)','roll_force':'Roll Virtual Force 0x51 (N)',
        'rudder_position':'Rudder Pedal Position (mm)','rudder_force':'Yaw Virtual Force 0x51 (N)',
        'elevator_input_norm':'Measured Elevator Input (normalised)','aileron_input_norm':'Measured Aileron Input (normalised)','rudder_input_norm':'Measured Rudder Input (normalised)',
        'titt':'Ti/Tt response analysis'}
    if selected_graphs is not None:
        chosen={mapping[key] for key in selected_graphs if key in mapping}
        groups.extend([
            ('Additional Flight and Engine Measurements',[(mapping[key],mapping[key]) for key in ('height_agl_ft','ground_speed_kt','manifold_inhg','fuel_flow_pph')]),
            ('Measured Simulator Control Inputs',[(mapping[key],mapping[key]) for key in ('elevator_input_norm','aileron_input_norm','rudder_input_norm')]),
            ('Ti/Tt Response Analysis',[(mapping['titt'],mapping['titt'])])])
        groups=[(title,[(key,label) for key,label in channels if key in chosen]) for title,channels in groups]
        groups=[(title,channels) for title,channels in groups if channels]
    measured_label={'pid':'PID Automatic Test (Sim)','manual':'Manual Test (Flight)','auto':'Recorded Replay Test (Sim)'}[mode]
    duration=max(75.,float(np.nanmax(t))) if len(t) else 75.
    with plt.rc_context({'font.family':'DejaVu Sans','axes.facecolor':'white','figure.facecolor':'white','text.color':'black','axes.labelcolor':'black','xtick.color':'black','ytick.color':'black'}),PdfPages(out_path) as pdf:
        _initial_page(pdf,mode,csv_path,df,validation_df,evaluation_result,virtual_forces=True)
        for title,channels in groups:
            fig,axes=plt.subplots(len(channels),1,figsize=(8.27,11.69),constrained_layout=True,squeeze=False)
            fig.suptitle(title,fontsize=15 if len(channels)==4 else 16,fontweight='bold')
            for ax,(key,label) in zip(axes[:,0],channels):
                values=_series(df,key)
                if key=="Magnetic Heading (deg)":values=_unwrap_heading(values)
                ax.plot(t,values,color='green',linewidth=2.2,label=measured_label)
                if not np.isfinite(np.asarray(values,dtype=float)).any():
                    ax.text(.5,.5,'Measurement not available',transform=ax.transAxes,ha='center',va='center',color='gray')
                if key=='Indicated Airspeed (kts)':
                    _draw_climb_airspeed_target(ax,target_ias)
                if validation_df is not None and key in validation_df and (mode=='auto' or key in ('Indicated Airspeed (kts)','Rate of Climb/Descent (fpm)')):
                    rt=_series(validation_df,'Time (s)');reference=_series(validation_df,key)
                    tolerance=3. if key=='Indicated Airspeed (kts)' else np.maximum(abs(reference)*.05,100.) if key=='Rate of Climb/Descent (fpm)' else None
                    color='lightskyblue' if key=='Indicated Airspeed (kts)' else 'peachpuff'
                    ax.plot(rt,reference,color='gray',linestyle=':',linewidth=1.3,label='Manual reference' if mode=='auto' else 'Validation reference')
                    if tolerance is not None:
                        ax.fill_between(rt,reference-tolerance,reference+tolerance,color=color,alpha=.45,label='Validation tolerance')
                        ax.plot(rt,reference-tolerance,color='gray',linestyle='--',linewidth=.8)
                        ax.plot(rt,reference+tolerance,color='gray',linestyle='--',linewidth=.8)
                _style_axis(ax,label)
                if key=='Indicated Airspeed (kts)':_format_climb_airspeed_axis(ax)
                ax.set_xlim(0,duration)
                ax.set_xticks(np.arange(0,duration+1,5 if duration<=120 else 10))
                ax.legend(loc='best',fontsize=8,framealpha=.95)
            fig.text(.99,.002,measured_label+' | '+evaluation_result,ha='right',fontsize=6)
            pdf.savefig(fig);plt.close(fig)
    return str(out_path)


def _auto_report(auto_path, manual_path, out_path):

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



        fig,axes=plt.subplots(4,1,figsize=(8.27,11.69),constrained_layout=True)

        fig.suptitle("1st Quadrant Combined Group 4 (Position + Manual Applied Force vs Inverted Automatic Virtual Force)",fontsize=13.5,fontweight="bold")

        _plot_auto(

            axes[0],tm,_series(m,"Joystick Pitch Position (mm)"),

            ta,_series(a,"Joystick Pitch Position (mm)"),

            "Joystick Pitch Position\n(mm)"

        )

        _plot_auto(

            axes[1],tm,_manual_applied_force(m,"Pitch"),

            ta,_automatic_virtual_force_inverted(a,"Pitch"),

            "Pitch: Manual Applied Force\nvs Automatic Virtual Force (inverted) (N)",

            manual_label="Manual Applied Force (Reference)",

            auto_label="Automatic Virtual Force (Sim)"

        )

        _plot_auto(

            axes[2],tm,_series(m,"Joystick Roll Position (mm)"),

            ta,_series(a,"Joystick Roll Position (mm)"),

            "Joystick Roll Position\n(mm)"

        )

        _plot_auto(

            axes[3],tm,_manual_applied_force(m,"Roll"),

            ta,_automatic_virtual_force_inverted(a,"Roll"),

            "Roll: Manual Applied Force\nvs Automatic Virtual Force (inverted) (N)",

            manual_label="Manual Applied Force (Reference)",

            auto_label="Automatic Virtual Force (Sim)"

        )

        pdf.savefig(fig); plt.close(fig)



        fig,axes=plt.subplots(2,1,figsize=(8.27,11.69),constrained_layout=True)

        fig.suptitle("1st Quadrant Combined Group 5 (Rudder Position + Manual Applied Force vs Inverted Automatic Virtual Force)",fontsize=13.5,fontweight="bold")

        _plot_auto(

            axes[0],tm,_series(m,"Rudder Pedal Position (mm)"),

            ta,_series(a,"Rudder Pedal Position (mm)"),

            "Rudder Pedal Position\n(mm)"

        )

        _plot_auto(

            axes[1],tm,_manual_applied_force(m,"Yaw"),

            ta,_automatic_virtual_force_inverted(a,"Yaw"),

            "Yaw: Manual Applied Force\nvs Automatic Virtual Force (inverted) (N)",

            manual_label="Manual Applied Force (Reference)",

            auto_label="Automatic Virtual Force (Sim)"

        )

        pdf.savefig(fig); plt.close(fig)




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

    if kwargs.get('control_method')=='pid':
        from qtg_climb_pid import report
        return report(csv_path,kwargs.get('validation_csv_path'),kwargs.get('run_fault'),selected_graphs=kwargs.get('selected_graphs'),target_ias=kwargs.get('pid_target_ias',80))


    csv_path = Path(csv_path)

    if output_pdf_path:

        out_path = Path(output_pdf_path)

    else:

        out_path = csv_path.with_name(csv_path.stem + "_graphs.pdf")

    if kwargs.get('test_reference')=='1.c.(1)' and kwargs.get('selected_graphs') is not None:
        mode='manual' if 'manual' in csv_path.name.lower() else 'auto'
        reference=_read_csv(manual_csv_path) if mode=='auto' and manual_csv_path else None
        conditions=kwargs.get('initial_conditions') or {}
        target=float(conditions.get('ias_kt',80))
        return _pid_report(csv_path,out_path,reference,'NOT EVALUATED',target,kwargs['selected_graphs'],mode)

    if "manual" in csv_path.name.lower():

        _manual_report(csv_path, out_path)

        return str(out_path)

    manual_path = Path(manual_csv_path) if manual_csv_path else _find_manual_for_auto(csv_path)

    if manual_path is None or not manual_path.exists():

        manual_path = _find_manual_for_auto(csv_path)

    if manual_path is None or not manual_path.exists():

        raise FileNotFoundError("No Manual reference CSV found for Automatic report.")

    _auto_report(csv_path, manual_path, out_path)

    return str(out_path)

