#!/usr/bin/env python3
"""
record_replay_pitch_roll_rudder.py - Synchronized Brunner 3-Axis Record & Replay Suite
====================================================================================
Architecture & Decoupling:
  1. Flight Controls (100% Brunner Exclusive):
     - All stick and rudder control forces and positions are strictly read from and written to
       Brunner CLS2Sim hardware over direct UDP (127.0.0.1:15090).
     - ZERO control commands are sent to Prepar3D (no SimConnect axis fighting).

  2. Flight Simulation Telemetry & Initial Conditions (Prepar3D v5):
     - Prepar3D is connected in read-only telemetry mode via p3d_simconnect_v15.py.
     - Before Manual Recording starts, the script verifies active simulation frames.
     - Manual t=0 captures the exact flight pose (Lat, Lon, Alt, Heading, Speed, Pitch, Bank).
     - Automatic Replay teleports P3D while paused to the exact Manual t=0 flight state,
       pre-positions Brunner actuators to (P0, R0, Y0), settles trim forces, and unpauses
       in perfect lockstep.

  3. Standalone Safety:
     - If Prepar3D is not running, the system runs seamlessly in pure Brunner hardware mode.
"""

import os
import sys
import time
import csv
import json
import threading
from typing import Optional, Dict, Any

# Ensure local directories are on import path
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(CURRENT_DIR, ".."))
if CURRENT_DIR not in sys.path:
    sys.path.insert(0, CURRENT_DIR)
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from brunner_interface import BrunnerInterface

# Optional P3D SimConnect integration
try:
    from p3d_simconnect_v15 import SimConnectClient
    HAS_P3D = True
except Exception:
    SimConnectClient = None
    HAS_P3D = False

# Optional Matplotlib
try:
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
except ImportError:
    plt = None
    HAS_MATPLOTLIB = False

# =====================================================================
# CONFIGURATION
# =====================================================================
SAMPLE_RATE_HZ   = 50
INTERVAL_SEC     = 1.0 / SAMPLE_RATE_HZ
LEAD_MS          = 80
LEAD_SAMPLES     = round((LEAD_MS / 1000.0) * SAMPLE_RATE_HZ)
GAIN             = 1.00
SETTLING_TIME_S  = 2.5
CSV_FILENAME     = "pitch_roll_rudder_telemetry.csv"
JSON_FILENAME    = "pitch_roll_rudder_init_conditions.json"


def init_p3d_client() -> Optional[Any]:
    """Connect to Prepar3D v5 in a background daemon thread for read-only telemetry."""
    if not HAS_P3D or SimConnectClient is None:
        print("[P3D] SimConnect client module not available. Running pure Brunner controls.")
        return None

    try:
        p3d = SimConnectClient(app_name="Brunner 3-Axis Suite")
        worker = threading.Thread(target=p3d.connect_and_run, daemon=True)
        worker.start()

        # Wait up to 3.0s for P3D connection handshake
        timeout = 3.0
        start_t = time.time()
        while time.time() - start_t < timeout:
            if p3d.is_connected():
                break
            time.sleep(0.15)

        if p3d.is_connected():
            print(f"[P3D] Successfully connected to Prepar3D v5 (Bridge v{p3d.bridge_version}).")
            return p3d
        else:
            print("[P3D] Prepar3D v5 not detected on localhost. Running pure Brunner controls.")
            p3d.stop()
            return None
    except Exception as e:
        print(f"[P3D] Connection bypass note: {e}")
        return None


def wait_for_valid_flight_state(p3d: Any, timeout_sec: float = 4.0) -> Dict[str, Any]:
    """Ensure Prepar3D telemetry has delivered at least one live flight packet."""
    if not p3d or not p3d.is_connected():
        return {}

    deadline = time.time() + timeout_sec
    while time.time() < deadline:
        tele = p3d.get_telemetry()
        # Check if real flight telemetry has arrived
        if tele.get("airspeed_indicated", 0.0) > 5.0 or p3d.get_telemetry_sequence() > 0:
            return tele
        time.sleep(0.05)
    return p3d.get_telemetry()


def print_initial_conditions_table(manual_snap: Dict[str, Any], auto_snap: Dict[str, Any]):
    """Print clean verification table comparing Manual vs Auto initial conditions."""
    print("\n" + "=" * 78)
    print("INITIAL CONDITIONS VERIFICATION (t = 0.00s)")
    print("=" * 78)
    print(f"{'Parameter':<26} | {'Manual (t=0)':<14} | {'Auto (t=0)':<14} | {'Difference':<12} | {'Status'}")
    print("-" * 78)

    checks = [
        ("Pitch Control (Stick)", "pitch_pos", 0.05),
        ("Roll Control (Stick)",  "roll_pos",  0.05),
        ("Rudder Pedal Position", "rudder_pos", 0.05),
        ("Airspeed (IAS kt)",     "airspeed", 2.0),
        ("Altitude (ft MSL)",     "altitude", 50.0),
        ("Pitch Angle (deg)",     "pitch_ang", 2.0),
        ("Bank Angle (deg)",      "bank_ang", 2.0),
        ("Heading (deg)",         "heading", 2.0),
        ("Engine RPM",            "rpm", 100.0),
        ("Throttle (%)",          "throttle", 5.0),
    ]

    for label, key, tol in checks:
        if key in manual_snap and key in auto_snap:
            m_val = float(manual_snap[key])
            a_val = float(auto_snap[key])
            diff = abs(m_val - a_val)
            status = "[MATCH]" if diff <= tol else "[MISMATCH]"
            print(f"{label:<26} | {m_val:>14.2f} | {a_val:>14.2f} | {diff:>12.2f} | {status}")

    print("-" * 78 + "\n")


def main():
    print("=" * 70)
    print("  Brunner 3-Axis Record & Replay Suite (Pitch, Roll & Rudder)")
    print("  Controls: Pure Brunner UDP (15090) | Telemetry: Prepar3D v5")
    print("=" * 70)

    # 1. Connect to Brunner Hardware via UDP (15090)
    brunner = BrunnerInterface()
    if not brunner.connect():
        print("\n[ERROR] Could not connect to Brunner CLS2Sim on 127.0.0.1:15090.")
        print("Please ensure CLS2Sim is running and its UDP interface is active.\n")
        return

    # 2. Connect to Prepar3D (Read-Only Telemetry & Positioning)
    p3d = init_p3d_client()

    try:
        # User input for recording duration
        sec_input = input("\nEnter recording duration in seconds (default 5): ").strip()
        duration = float(sec_input) if sec_input else 5.0

        # Pre-verify P3D telemetry state before prompting the user
        if p3d:
            print("[INFO] Verifying live Prepar3D flight state...", end="", flush=True)
            p3d_ready = wait_for_valid_flight_state(p3d, timeout_sec=2.5)
            ias = p3d_ready.get("airspeed_indicated", 0.0)
            alt = p3d_ready.get("indicated_altitude", 0.0)
            print(f" Ready! (P3D IAS: {ias:.1f} kt, Alt: {alt:.0f} ft)")

        # Prompt user to begin manual recording
        input("\n>>> Place hands on the stick and feet on the pedals, then press ENTER to START RECORDING... <<<")
        print(">>> RECORDING NOW! Move Stick in Pitch/Roll & Pedals in Rudder! <<<")

        # Storage arrays
        man_time = []
        p_man_pos, p_man_force = [], []
        r_man_pos, r_man_force = [], []
        y_man_pos, y_man_force = [], []
        man_p3d_tele = []

        # =====================================================================
        # MANUAL RECORDING LOOP (50 Hz)
        # =====================================================================
        t0 = time.time()
        next_tick = 0.0
        manual_t0_snapshot: Dict[str, Any] = {}

        while time.time() - t0 < duration:
            t = time.time() - t0

            # 1. Read physical controls from Brunner CLS2Sim
            pp, pf, _ = brunner.read_axis(brunner.AXIS_PITCH)
            rp, rf, _ = brunner.read_axis(brunner.AXIS_ROLL)
            yp, yf, _ = brunner.read_axis(brunner.AXIS_RUDDER)

            # 2. Read live flight dynamics from Prepar3D
            tele = p3d.get_telemetry() if p3d else {}

            man_time.append(t)
            p_man_pos.append(pp); p_man_force.append(pf)
            r_man_pos.append(rp); r_man_force.append(rf)
            y_man_pos.append(yp); y_man_force.append(yf)
            man_p3d_tele.append(tele)

            # Capture exact t=0 snapshot on the very first sample
            if len(man_time) == 1:
                manual_t0_snapshot = {
                    "pitch_pos": pp,
                    "roll_pos": rp,
                    "rudder_pos": yp,
                    "airspeed": tele.get("airspeed_indicated", 0.0),
                    "altitude": tele.get("indicated_altitude", 0.0),
                    "pitch_ang": tele.get("pitch_angle", 0.0),
                    "bank_ang": tele.get("bank_angle", 0.0),
                    "heading": tele.get("true_heading", 0.0),
                    "rpm": tele.get("engine_rpm", 0.0) or (tele.get("eng_rpms", [0])[0]),
                    "throttle": tele.get("max_throttle", 80.0),
                    "lat": tele.get("plane_latitude", 32.233833),
                    "lon": tele.get("plane_longitude", 74.123667),
                    "full_telemetry": dict(tele),
                }

            # Console progress display
            if len(man_time) % 10 == 0:
                print(f"\r  Recording: {t:4.1f}s | Pitch: {pp:+.2f} | Roll: {rp:+.2f} | Rudder: {yp:+.2f}", end="", flush=True)

            next_tick += INTERVAL_SEC
            dt = next_tick - (time.time() - t0)
            if dt > 0:
                time.sleep(dt)

        print(f"\nRecording complete! ({len(man_time)} samples captured across all 3 axes)")
        input("\n>>> Press ENTER to automatically replay from the EXACT SAME initial conditions... <<<")

        # =====================================================================
        # AUTOMATIC REPLAY PREPARATION (Exact t=0 Matching)
        # =====================================================================
        print("\n[PREPARATION] Synchronizing initial conditions for Automatic Replay...")

        # 1. Reposition Prepar3D to exact Manual t=0 state
        if p3d and manual_t0_snapshot:
            print("  [P3D] Freezing simulation (pause_sim)...")
            p3d.pause_sim()

            m_lat = float(manual_t0_snapshot.get("lat", 32.233833))
            m_lon = float(manual_t0_snapshot.get("lon", 74.123667))
            m_alt = float(manual_t0_snapshot.get("altitude", 3000.0))
            m_spd = float(manual_t0_snapshot.get("airspeed", 80.0))
            m_pitch = float(manual_t0_snapshot.get("pitch_ang", 0.0))
            m_bank = float(manual_t0_snapshot.get("bank_ang", 0.0))
            m_hdg = float(manual_t0_snapshot.get("heading", 48.4))
            m_throttle = float(manual_t0_snapshot.get("throttle", 80.0))

            print(f"  [P3D] Teleporting to Manual t=0: Lat={m_lat:.5f}, Lon={m_lon:.5f}, Alt={m_alt:.0f}ft, IAS={m_spd:.1f}kt...")
            p3d.queue_position_airborne(
                alt_ft=m_alt,
                speed_kts=m_spd,
                pitch_deg=m_pitch,
                bank_deg=m_bank,
                hdg_deg=m_hdg,
                lat=m_lat,
                lon=m_lon,
                throttle=m_throttle,
            )

        # 2. Configure Brunner Autopilot and pre-position actuators to exact Manual t=0
        brunner.configure_replay_mode()
        if p_man_pos:
            brunner.move_actuators(p_man_pos[0], r_man_pos[0], y_man_pos[0])

        print(f"  [STABILIZING] Waiting {SETTLING_TIME_S:.1f}s for P3D physics & Brunner trim forces to settle...")
        time.sleep(SETTLING_TIME_S)

        # 3. Unfreeze P3D in lockstep with replay start
        if p3d:
            p3d.unpause_sim()
            print("  [P3D] Simulation unpaused.")

        print("\n>>> REPLAYING NOW! (Hands off the stick, Feet off the pedals) <<<")

        # =====================================================================
        # AUTOMATIC REPLAY LOOP (50 Hz)
        # =====================================================================
        auto_time = []
        p_auto_pos, p_auto_vf = [], []
        r_auto_pos, r_auto_vf = [], []
        y_auto_pos, y_auto_vf = [], []
        auto_p3d_tele = []
        auto_t0_snapshot: Dict[str, Any] = {}

        t_play = time.time()
        total = len(man_time)

        for i in range(total):
            t_tick = t_play + (i * INTERVAL_SEC)
            lead_idx = min(total - 1, i + LEAD_SAMPLES)

            # Command physical Brunner hardware ONLY (Zero P3D control writes)
            p_cmd = max(-1.0, min(1.0, p_man_pos[lead_idx] * GAIN))
            r_cmd = max(-1.0, min(1.0, r_man_pos[lead_idx] * GAIN))
            y_cmd = max(-1.0, min(1.0, y_man_pos[lead_idx] * GAIN))

            brunner.move_actuators(p_cmd, r_cmd, y_cmd)

            # Read back physical Brunner motor positions & dynamic virtual forces
            pp, _, pvf = brunner.read_axis(brunner.AXIS_PITCH)
            rp, _, rvf = brunner.read_axis(brunner.AXIS_ROLL)
            yp, _, yvf = brunner.read_axis(brunner.AXIS_RUDDER)

            # Read live flight dynamics from Prepar3D
            tele = p3d.get_telemetry() if p3d else {}

            auto_time.append(time.time() - t_play)
            p_auto_pos.append(pp); p_auto_vf.append(pvf)
            r_auto_pos.append(rp); r_auto_vf.append(rvf)
            y_auto_pos.append(yp); y_auto_vf.append(yvf)
            auto_p3d_tele.append(tele)

            # Capture exact t=0 snapshot on the very first sample
            if len(auto_time) == 1:
                auto_t0_snapshot = {
                    "pitch_pos": pp,
                    "roll_pos": rp,
                    "rudder_pos": yp,
                    "airspeed": tele.get("airspeed_indicated", 0.0),
                    "altitude": tele.get("indicated_altitude", 0.0),
                    "pitch_ang": tele.get("pitch_angle", 0.0),
                    "bank_ang": tele.get("bank_angle", 0.0),
                    "heading": tele.get("true_heading", 0.0),
                    "rpm": tele.get("engine_rpm", 0.0) or (tele.get("eng_rpms", [0])[0]),
                    "throttle": tele.get("max_throttle", 80.0),
                    "lat": tele.get("plane_latitude", 32.233833),
                    "lon": tele.get("plane_longitude", 74.123667),
                    "full_telemetry": dict(tele),
                }

            if i % 10 == 0 or i == total - 1:
                print(f"\r  Replaying: {i+1}/{total} | Pitch: {pp:+.2f} | Roll: {rp:+.2f} | Rudder: {yp:+.2f}", end="", flush=True)

            dt = (t_tick + INTERVAL_SEC) - time.time()
            if dt > 0:
                time.sleep(dt)

        print("\nPlayback complete!")

    finally:
        brunner.close()
        if p3d:
            p3d.stop()

    # =====================================================================
    # VERIFY INITIAL CONDITIONS TABLE
    # =====================================================================
    if manual_t0_snapshot and auto_t0_snapshot:
        print_initial_conditions_table(manual_t0_snapshot, auto_t0_snapshot)

    # =====================================================================
    # SAVE CSV TELEMETRY
    # =====================================================================
    csv_file = os.path.join(PROJECT_ROOT, CSV_FILENAME)
    try:
        with open(csv_file, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "Index", "Time_s",
                "Pitch_Man_Pos", "Pitch_Man_Force", "Pitch_Auto_Pos", "Pitch_Auto_VF",
                "Roll_Man_Pos", "Roll_Man_Force", "Roll_Auto_Pos", "Roll_Auto_VF",
                "Rudder_Man_Pos", "Rudder_Man_Force", "Rudder_Auto_Pos", "Rudder_Auto_VF",
                "Airspeed_Man", "Airspeed_Auto",
                "Vertical_Speed_Man", "Vertical_Speed_Auto",
                "Altitude_Man", "Altitude_Auto",
                "Pitch_Angle_Man", "Pitch_Angle_Auto",
                "Bank_Angle_Man", "Bank_Angle_Auto",
                "Heading_Man", "Heading_Auto",
                "Pitch_Rate_Man", "Pitch_Rate_Auto",
                "Yaw_Rate_Man", "Yaw_Rate_Auto",
                "Roll_Rate_Man", "Roll_Rate_Auto",
                "Prop_RPM_Man", "Prop_RPM_Auto",
                "Engine_RPM_Man", "Engine_RPM_Auto",
                "Throttle_Man", "Throttle_Auto"
            ])

            for i in range(len(man_time)):
                auto_idx = i if i < len(p_auto_pos) else -1
                m_tel = man_p3d_tele[i] if i < len(man_p3d_tele) else {}
                a_tel = auto_p3d_tele[auto_idx] if (auto_idx >= 0 and auto_idx < len(auto_p3d_tele)) else {}

                m_rpm = m_tel.get("engine_rpm", 0.0) or (m_tel.get("eng_rpms", [0])[0])
                a_rpm = a_tel.get("engine_rpm", 0.0) or (a_tel.get("eng_rpms", [0])[0])
                m_prop = m_tel.get("prop_rpms", [0.0])[0] if m_tel.get("prop_rpms") else m_rpm
                a_prop = a_tel.get("prop_rpms", [0.0])[0] if a_tel.get("prop_rpms") else a_rpm

                writer.writerow([
                    i, f"{man_time[i]:.4f}",
                    f"{p_man_pos[i]:.4f}", f"{p_man_force[i]:.4f}",
                    f"{p_auto_pos[auto_idx] if auto_idx >= 0 else 0.0:.4f}",
                    f"{p_auto_vf[auto_idx] if auto_idx >= 0 else 0.0:.4f}",
                    f"{r_man_pos[i]:.4f}", f"{r_man_force[i]:.4f}",
                    f"{r_auto_pos[auto_idx] if auto_idx >= 0 else 0.0:.4f}",
                    f"{r_auto_vf[auto_idx] if auto_idx >= 0 else 0.0:.4f}",
                    f"{y_man_pos[i]:.4f}", f"{y_man_force[i]:.4f}",
                    f"{y_auto_pos[auto_idx] if auto_idx >= 0 else 0.0:.4f}",
                    f"{y_auto_vf[auto_idx] if auto_idx >= 0 else 0.0:.4f}",
                    f"{m_tel.get('airspeed_indicated', 0.0):.2f}",
                    f"{a_tel.get('airspeed_indicated', 0.0):.2f}",
                    f"{m_tel.get('vertical_speed', 0.0):.1f}",
                    f"{a_tel.get('vertical_speed', 0.0):.1f}",
                    f"{m_tel.get('indicated_altitude', 0.0):.1f}",
                    f"{a_tel.get('indicated_altitude', 0.0):.1f}",
                    f"{m_tel.get('pitch_angle', 0.0):.2f}",
                    f"{a_tel.get('pitch_angle', 0.0):.2f}",
                    f"{m_tel.get('bank_angle', 0.0):.2f}",
                    f"{a_tel.get('bank_angle', 0.0):.2f}",
                    f"{m_tel.get('true_heading', 0.0):.2f}",
                    f"{a_tel.get('true_heading', 0.0):.2f}",
                    f"{m_tel.get('pitch_rate', 0.0):.2f}",
                    f"{a_tel.get('pitch_rate', 0.0):.2f}",
                    f"{m_tel.get('yaw_rate', 0.0):.2f}",
                    f"{a_tel.get('yaw_rate', 0.0):.2f}",
                    f"{m_tel.get('roll_rate', 0.0):.2f}",
                    f"{a_tel.get('roll_rate', 0.0):.2f}",
                    f"{m_prop:.0f}",
                    f"{a_prop:.0f}",
                    f"{m_rpm:.0f}",
                    f"{a_rpm:.0f}",
                    f"{m_tel.get('max_throttle', 0.0):.1f}",
                    f"{a_tel.get('max_throttle', 0.0):.1f}",
                ])
        print(f"\n[SAVED] CSV Telemetry: {csv_file}")
    except Exception as e:
        print(f"Warning: Could not save CSV: {e}")

    # =====================================================================
    # SAVE INITIAL CONDITIONS JSON
    # =====================================================================
    json_file = os.path.join(PROJECT_ROOT, JSON_FILENAME)
    try:
        init_data = {
            "manual": manual_t0_snapshot,
            "auto": auto_t0_snapshot,
        }
        with open(json_file, "w") as jf:
            json.dump(init_data, jf, indent=2, default=str)
        print(f"[SAVED] Initial Conditions JSON: {json_file}")
    except Exception as e:
        print(f"Warning: Could not save JSON: {e}")

    # =====================================================================
    # GENERATE CONTROLS PDF REPORT (GROUP 4 & GROUP 5)
    # =====================================================================
    try:
        from generate_flight_test_pdf import generate_flight_test_pdf
        pdf_out = generate_flight_test_pdf(csv_file, auto_open=True)
        if pdf_out:
            print(f"[SAVED] Controls PDF Report: {pdf_out}")
    except Exception as e:
        print(f"[INFO] PDF generation note: {e}")

    # =====================================================================
    # OPTIONAL MATPLOTLIB CHARTS
    # =====================================================================
    if HAS_MATPLOTLIB and plt is not None:
        try:
            print("\n[GENERATING GRAPHS] Rendering 3-axis flight control telemetry...")
            fig, axes = plt.subplots(2, 3, figsize=(18, 9), sharex=True)
            (ax_p1, ax_r1, ax_y1), (ax_p2, ax_r2, ax_y2) = axes

            # Pitch
            ax_p1.plot(man_time, p_man_pos, label="Manual Pitch (Stick)", color="#1f77b4", linewidth=2.5)
            ax_p1.plot(auto_time, p_auto_pos, label="Auto Pitch (Motor)", color="#d62728", linestyle="--", linewidth=2.0)
            ax_p1.set_title(f"Pitch Tracking (@ {SAMPLE_RATE_HZ}Hz, Lead: {LEAD_MS}ms)", fontsize=11, fontweight="bold")
            ax_p1.set_ylabel("Pitch Position (Norm)")
            ax_p1.set_ylim(-1.05, 1.05)
            ax_p1.grid(True, linestyle="--", alpha=0.6)
            ax_p1.legend(loc="upper right", fontsize=9)

            ax_p2.plot(man_time, p_man_force, label="Manual Pitch Force", color="#2ca02c", linewidth=2.0)
            ax_p2.set_ylabel("Manual Force (Norm)", color="#2ca02c")
            ax_p2.tick_params(axis="y", labelcolor="#2ca02c")
            ax_p2.grid(True, linestyle="--", alpha=0.6)
            ax_p2_right = ax_p2.twinx()
            ax_p2_right.plot(auto_time, p_auto_vf, label="Auto Pitch VF (Nm)", color="#ff7f0e", linestyle="--", linewidth=2.0)
            ax_p2_right.set_ylabel("Auto VF (Nm)", color="#ff7f0e")
            ax_p2_right.tick_params(axis="y", labelcolor="#ff7f0e")

            # Roll
            ax_r1.plot(man_time, r_man_pos, label="Manual Roll (Stick)", color="#1f77b4", linewidth=2.5)
            ax_r1.plot(auto_time, r_auto_pos, label="Auto Roll (Motor)", color="#d62728", linestyle="--", linewidth=2.0)
            ax_r1.set_title(f"Roll Tracking (@ {SAMPLE_RATE_HZ}Hz, Lead: {LEAD_MS}ms)", fontsize=11, fontweight="bold")
            ax_r1.set_ylabel("Roll Position (Norm)")
            ax_r1.set_ylim(-1.05, 1.05)
            ax_r1.grid(True, linestyle="--", alpha=0.6)
            ax_r1.legend(loc="upper right", fontsize=9)

            ax_r2.plot(man_time, r_man_force, label="Manual Roll Force", color="#2ca02c", linewidth=2.0)
            ax_r2.set_ylabel("Manual Force (Norm)", color="#2ca02c")
            ax_r2.tick_params(axis="y", labelcolor="#2ca02c")
            ax_r2.grid(True, linestyle="--", alpha=0.6)
            ax_r2_right = ax_r2.twinx()
            ax_r2_right.plot(auto_time, r_auto_vf, label="Auto Roll VF (Nm)", color="#ff7f0e", linestyle="--", linewidth=2.0)
            ax_r2_right.set_ylabel("Auto VF (Nm)", color="#ff7f0e")
            ax_r2_right.tick_params(axis="y", labelcolor="#ff7f0e")

            # Rudder
            ax_y1.plot(man_time, y_man_pos, label="Manual Rudder (Pedals)", color="#1f77b4", linewidth=2.5)
            ax_y1.plot(auto_time, y_auto_pos, label="Auto Rudder (Motor)", color="#d62728", linestyle="--", linewidth=2.0)
            ax_y1.set_title(f"Rudder Tracking (@ {SAMPLE_RATE_HZ}Hz, Lead: {LEAD_MS}ms)", fontsize=11, fontweight="bold")
            ax_y1.set_ylabel("Rudder Position (Norm)")
            ax_y1.set_ylim(-1.05, 1.05)
            ax_y1.grid(True, linestyle="--", alpha=0.6)
            ax_y1.legend(loc="upper right", fontsize=9)

            ax_y2.plot(man_time, y_man_force, label="Manual Rudder Force", color="#2ca02c", linewidth=2.0)
            ax_y2.set_ylabel("Manual Force (Norm)", color="#2ca02c")
            ax_y2.tick_params(axis="y", labelcolor="#2ca02c")
            ax_y2.grid(True, linestyle="--", alpha=0.6)
            ax_y2_right = ax_y2.twinx()
            ax_y2_right.plot(auto_time, y_auto_vf, label="Auto Rudder VF (Nm)", color="#ff7f0e", linestyle="--", linewidth=2.0)
            ax_y2_right.set_ylabel("Auto VF (Nm)", color="#ff7f0e")
            ax_y2_right.tick_params(axis="y", labelcolor="#ff7f0e")

            plt.tight_layout()
            chart_png = os.path.join(PROJECT_ROOT, "pitch_roll_rudder_comparison.png")
            plt.savefig(chart_png, dpi=150)
            print(f"[SAVED] Telemetry Graphs: {chart_png}")
            plt.close()
        except Exception as e:
            print(f"[INFO] Chart rendering note: {e}")


if __name__ == "__main__":
    main()
