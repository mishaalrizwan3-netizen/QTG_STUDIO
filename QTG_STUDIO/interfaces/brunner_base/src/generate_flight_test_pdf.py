#!/usr/bin/env python3
"""
generate_flight_test_pdf.py - Controls PDF Report Generator (Group 4 & Group 5)
==============================================================================
Generates official Controls PDF Report:
  - Page 1: Combined Group 4 (Pitch & Roll Position + Force Tracking)
  - Page 2: Combined Group 5 (Rudder Pedal Position + Yaw Force Tracking)
Zero P3D / SimConnect dependencies.
"""

import sys
import os
import pandas as pd
import numpy as np

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_pdf import PdfPages
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False


def map_force_to_overlay(man_force, auto_vf, pos=None):
    if pos is not None and len(pos) > 0:
        neutral_mask = np.abs(pos) < 0.02
        if np.sum(neutral_mask) > 10:
            v0 = float(np.median(auto_vf[neutral_mask]))
            m0 = float(np.median(man_force[neutral_mask]))
        else:
            v0 = float(np.median(auto_vf))
            m0 = float(np.median(man_force))
    else:
        v0 = float(np.median(auto_vf))
        m0 = float(np.median(man_force))

    mf = man_force - m0
    vf = auto_vf - v0

    if pos is not None and len(pos) > 0:
        motion_indices = np.where(np.abs(pos) >= 0.015)[0]
        if len(motion_indices) > 0:
            first_motion = motion_indices[0]
            vf[:first_motion] = 0.0

    corr = np.corrcoef(mf, vf)[0, 1] if (np.std(mf) > 1e-6 and np.std(vf) > 1e-6) else 1.0
    vf_mapped = -1.0 * vf if corr < 0 else 1.0 * vf

    max_v = np.max(np.abs(vf_mapped))
    max_m = np.max(np.abs(mf))
    if max_m > 1e-5 and max_v > 1e-5:
        scale_factor = max_v / max_m
        mf_scaled = mf * scale_factor
    else:
        mf_scaled = mf

    return mf_scaled, vf_mapped


def generate_flight_test_pdf(csv_path="pitch_roll_rudder_telemetry.csv", output_pdf=None, auto_open=True, **kwargs):
    if not HAS_MATPLOTLIB:
        print("[ERROR] 'matplotlib' is required to generate the flight test PDF.")
        return None

    if not os.path.exists(csv_path):
        print(f"[ERROR] Telemetry CSV not found: {csv_path}")
        return None

    if output_pdf is None:
        base_name = os.path.splitext(os.path.basename(csv_path))[0]
        output_pdf = f"Super_Mushshak_Controls_Report_{base_name}.pdf"

    df = pd.read_csv(csv_path)

    # Time series
    if "Time_s" in df:
        t = df["Time_s"].values
    elif "Man_Time_s" in df:
        t = df["Man_Time_s"].values
    else:
        t = np.arange(len(df)) * 0.02
    mark_step = max(1, len(t) // 30)

    def get_col(candidates, default_val=0.0):
        if isinstance(candidates, str):
            candidates = [candidates]
        for name in candidates:
            if name in df:
                return df[name].values
        return np.full(len(t), default_val)

    # Pitch
    p_man_pos = get_col(["Pitch_Man_Pos", "Pitch_Pos"])
    p_auto_pos = get_col(["Pitch_Auto_Pos", "Auto_Pitch_Pos"])
    p_man_force = get_col(["Pitch_Man_Force", "Pitch_Force"])
    p_auto_vf = get_col(["Pitch_Auto_VF", "Pitch_Virtual_Force"])

    # Roll
    r_man_pos = get_col(["Roll_Man_Pos", "Roll_Pos"])
    r_auto_pos = get_col(["Roll_Auto_Pos", "Auto_Roll_Pos"])
    r_man_force = get_col(["Roll_Man_Force", "Roll_Force"])
    r_auto_vf = get_col(["Roll_Auto_VF", "Roll_Virtual_Force"])

    # Rudder
    y_man_pos = get_col(["Rudder_Man_Pos", "Man_Pos"])
    y_auto_pos = get_col(["Rudder_Auto_Pos", "Auto_Pos"])
    y_man_force = get_col(["Rudder_Man_Force", "Man_Force"])
    y_auto_vf = get_col(["Rudder_Auto_VF", "Auto_Virtual_Force"])

    p_mf_plot, p_vf_plot = map_force_to_overlay(p_man_force, p_auto_vf, p_man_pos)
    r_mf_plot, r_vf_plot = map_force_to_overlay(r_man_force, r_auto_vf, r_man_pos)
    y_mf_plot, y_vf_plot = map_force_to_overlay(y_man_force, y_auto_vf, y_man_pos)

    has_pitch_roll = (np.std(p_man_pos) > 1e-4 or np.std(r_man_pos) > 1e-4 or
                      np.std(p_auto_pos) > 1e-4 or np.std(r_auto_pos) > 1e-4 or
                      "Pitch_Man_Pos" in df or "Roll_Man_Pos" in df)

    a4_size = (8.27, 11.69)
    print(f"[INFO] Generating Controls-Only PDF: {output_pdf}")

    with PdfPages(output_pdf) as pdf:

        # PAGE 1: COMBINED GROUP 4 (PITCH & ROLL)
        if has_pitch_roll:
            fig1, axes1 = plt.subplots(4, 1, figsize=a4_size)
            fig1.subplots_adjust(top=0.94, bottom=0.06, left=0.15, right=0.94, hspace=0.44)
            fig1.suptitle("1st Quadrant Combined Group 4 (Position + Manual Applied Force vs Inverted Automatic Virtual Force)",
                          fontsize=10.5, fontweight="bold", y=0.975)

            # 1. Pitch Position
            axes1[0].plot(t, p_man_pos, color="green", lw=1.8, label="Manual Test (Reference)")
            axes1[0].plot(t, p_auto_pos, color="#0066ff", lw=1.5, ls="--", marker="o", markersize=3.2, markevery=mark_step, label="Automatic Test (Sim)")
            axes1[0].set_ylabel("Joystick Pitch Position\n(Norm)", fontsize=8.5, fontweight="bold")
            axes1[0].set_xlabel("Time (s)", fontsize=8, fontweight="bold")
            axes1[0].tick_params(axis="both", labelsize=8)
            axes1[0].grid(True, linestyle="--", alpha=0.5)
            axes1[0].legend(loc="upper right", fontsize=7.5, framealpha=0.9)

            # 2. Pitch Force
            axes1[1].plot(t, p_mf_plot, color="green", lw=1.8, label="Manual Test (Reference)")
            axes1[1].plot(t, p_vf_plot, color="#0066ff", lw=1.5, ls="--", marker="o", markersize=3.2, markevery=mark_step, label="Automatic Test (Sim)")
            axes1[1].set_ylabel("Pitch: Manual Applied Force\nvs Automatic Virtual Force (inverted) (N)", fontsize=8, fontweight="bold")
            axes1[1].set_xlabel("Time (s)", fontsize=8, fontweight="bold")
            axes1[1].tick_params(axis="both", labelsize=8)
            axes1[1].grid(True, linestyle="--", alpha=0.5)
            axes1[1].legend(loc="upper right", fontsize=7.5, framealpha=0.9)

            # 3. Roll Position
            axes1[2].plot(t, r_man_pos, color="green", lw=1.8, label="Manual Test (Reference)")
            axes1[2].plot(t, r_auto_pos, color="#0066ff", lw=1.5, ls="--", marker="o", markersize=3.2, markevery=mark_step, label="Automatic Test (Sim)")
            axes1[2].set_ylabel("Joystick Roll Position\n(Norm)", fontsize=8.5, fontweight="bold")
            axes1[2].set_xlabel("Time (s)", fontsize=8, fontweight="bold")
            axes1[2].tick_params(axis="both", labelsize=8)
            axes1[2].grid(True, linestyle="--", alpha=0.5)
            axes1[2].legend(loc="upper right", fontsize=7.5, framealpha=0.9)

            # 4. Roll Force
            axes1[3].plot(t, r_mf_plot, color="green", lw=1.8, label="Manual Test (Reference)")
            axes1[3].plot(t, r_vf_plot, color="#0066ff", lw=1.5, ls="--", marker="o", markersize=3.2, markevery=mark_step, label="Automatic Test (Sim)")
            axes1[3].set_ylabel("Roll: Manual Applied Force\nvs Automatic Virtual Force (inverted) (N)", fontsize=8, fontweight="bold")
            axes1[3].set_xlabel("Time (s)", fontsize=8, fontweight="bold")
            axes1[3].tick_params(axis="both", labelsize=8)
            axes1[3].grid(True, linestyle="--", alpha=0.5)
            axes1[3].legend(loc="upper right", fontsize=7.5, framealpha=0.9)

            pdf.savefig(fig1)
            plt.close(fig1)

        # PAGE 2: COMBINED GROUP 5 (RUDDER)
        fig2, axes2 = plt.subplots(2, 1, figsize=a4_size)
        fig2.subplots_adjust(top=0.93, bottom=0.08, left=0.15, right=0.94, hspace=0.34)
        fig2.suptitle("1st Quadrant Combined Group 5 (Rudder Position + Manual Applied Force vs Inverted Automatic Virtual Force)",
                      fontsize=10.5, fontweight="bold", y=0.97)

        # 1. Rudder Position
        axes2[0].plot(t, y_man_pos, color="green", lw=1.8, label="Manual Test (Reference)")
        axes2[0].plot(t, y_auto_pos, color="#0066ff", lw=1.5, ls="--", marker="o", markersize=3.2, markevery=mark_step, label="Automatic Test (Sim)")
        axes2[0].set_ylabel("Rudder Pedal Position\n(Norm)", fontsize=9, fontweight="bold")
        axes2[0].set_xlabel("Time (s)", fontsize=8.5, fontweight="bold")
        axes2[0].tick_params(axis="both", labelsize=8.5)
        axes2[0].grid(True, linestyle="--", alpha=0.5)
        axes2[0].legend(loc="upper right", fontsize=8, framealpha=0.9)

        # 2. Rudder Force
        axes2[1].plot(t, y_mf_plot, color="green", lw=1.8, label="Manual Test (Reference)")
        axes2[1].plot(t, y_vf_plot, color="#0066ff", lw=1.5, ls="--", marker="o", markersize=3.2, markevery=mark_step, label="Automatic Test (Sim)")
        axes2[1].set_ylabel("Yaw: Manual Applied Force\nvs Automatic Virtual Force (inverted) (N)", fontsize=9, fontweight="bold")
        axes2[1].set_xlabel("Time (s)", fontsize=8.5, fontweight="bold")
        axes2[1].tick_params(axis="both", labelsize=8.5)
        axes2[1].grid(True, linestyle="--", alpha=0.5)
        axes2[1].legend(loc="upper right", fontsize=8, framealpha=0.9)

        pdf.savefig(fig2)
        plt.close(fig2)

    print(f"[SUCCESS] Controls PDF Report generated: {output_pdf}")

    if auto_open and os.path.exists(output_pdf):
        try:
            os.startfile(output_pdf)
        except Exception:
            pass

    return output_pdf


if __name__ == "__main__":
    target_csv = sys.argv[1] if len(sys.argv) > 1 else "pitch_roll_rudder_telemetry.csv"
    generate_flight_test_pdf(target_csv, auto_open=True)
