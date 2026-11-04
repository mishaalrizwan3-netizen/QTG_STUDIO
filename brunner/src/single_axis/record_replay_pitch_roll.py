import socket
import struct
import time
import csv

# Matplotlib for generating graphs at the end
try:
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
except ImportError:
    plt = None
    HAS_MATPLOTLIB = False

# =====================================================================
# 1. SETUP UDP CONNECTION TO CLS2SIM (PORT 15090) - 3 AXES
# =====================================================================
AXIS_PITCH  = 0x01  # Elevator / Pitch
AXIS_ROLL   = 0x02  # Aileron / Roll
AXIS_RUDDER = 0x04  # Rudder / Yaw
ALL_AXES    = AXIS_PITCH | AXIS_ROLL | AXIS_RUDDER  # 0x07 (Combined bitmask for all 3 axes)

# Sampling & Timing Configuration (50 Hz / 20 ms)
SAMPLE_RATE_HZ = 50                                             # 50 Hz update rate
INTERVAL_SEC   = 1.0 / SAMPLE_RATE_HZ                          # 0.020s (20 ms)
LEAD_MS        = 80                                            # 80 ms lead compensation
LEAD_SAMPLES   = round((LEAD_MS / 1000.0) * SAMPLE_RATE_HZ)      # 4 samples at 50 Hz
GAIN           = 1.00                                          # 1:1 Unity gain

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.settimeout(2.0)
s.connect(("127.0.0.1", 15090))

def read_axis(axis):
    """Reads Position (0x11), Applied Force (0x21), and Virtual Force (0x51)"""
    pos, force, v_force = 0.0, 0.0, 0.0
    try:
        # 1. Position (-1.0 to +1.0)
        s.send(struct.pack("<III", 0xD0, axis, 0x11))
        r = s.recv(1024)
        if len(r) >= 9:
            pos = struct.unpack_from("<f", r, 5)[0]

        # 2. Applied Hand/Foot Force (Load cell)
        s.send(struct.pack("<III", 0xD0, axis, 0x21))
        r = s.recv(1024)
        if len(r) >= 9:
            force = struct.unpack_from("<f", r, 5)[0]

        # 3. Virtual Aerodynamic Force (in Nm)
        s.send(struct.pack("<III", 0xD0, axis, 0x51))
        r = s.recv(1024)
        if len(r) >= 11:
            v_force = struct.unpack_from("<f", r, 7)[0]
    except Exception:
        pass

    return pos, force, v_force

def write_setting(axis, setting_id, fmt, val):
    try:
        s.send(struct.pack(f"<III{fmt}", 0xCE, axis, setting_id, val))
        s.recv(1024)
    except Exception:
        pass

def set_overrides(axis_mask, enable):
    """
    Enables (True) or releases (False) Autopilot overrides across axis_mask (0x07 for all 3 axes).
    """
    try:
        val = 1 if enable else 0
        s.send(struct.pack("<III", 0xD1, axis_mask, 0x01 if val else 0x00))
        s.recv(1024)  # Force scale
        s.send(struct.pack("<III", 0xD1, axis_mask, 0x03 if val else 0x00))
        s.recv(1024)  # AP enable
        s.send(struct.pack("<III", 0xD1, axis_mask, 0x04 if val else 0x00))
        s.recv(1024)  # AP position
    except Exception:
        pass

# =====================================================================
# 2. RECORD MANUAL PITCH, ROLL & RUDDER -> REPLAY AUTOMATICALLY
# =====================================================================
try:
    sec = input("Enter recording duration in seconds (default 5): ").strip()
    duration = float(sec) if sec else 5.0

    # Countdown
    print(f"\nRecording Pitch + Roll + Rudder for {duration:.1f}s.")
    print("Place hands on the stick and feet on the rudder pedals...")
    for c in [3, 2, 1]:
        print(f"  Starting in {c}...", end="\r", flush=True)
        time.sleep(1.0)
    print(">>> RECORDING NOW! Move Stick in Pitch/Roll & Pedals in Rudder! <<<          ")

    # Storage arrays
    man_time = []
    p_man_pos, p_man_force = [], []
    r_man_pos, r_man_force = [], []
    y_man_pos, y_man_force = [], []

    t0 = time.time()
    next_tick = 0.0
    while time.time() - t0 < duration:
        t = time.time() - t0

        # Read Pitch (0x01), Roll (0x02), and Rudder (0x04)
        pp, pf, _ = read_axis(AXIS_PITCH)
        rp, rf, _ = read_axis(AXIS_ROLL)
        yp, yf, _ = read_axis(AXIS_RUDDER)

        man_time.append(t)
        p_man_pos.append(pp); p_man_force.append(pf)
        r_man_pos.append(rp); r_man_force.append(rf)
        y_man_pos.append(yp); y_man_force.append(yf)

        next_tick += INTERVAL_SEC
        sleep_dur = next_tick - (time.time() - t0)
        if sleep_dur > 0:
            time.sleep(sleep_dur)

    print(f"\nRecording complete! ({len(man_time)} samples captured across all 3 axes)")
    input("Press ENTER to automatically replay ALL 3 AXES on physical controls...")

    # Configure Autopilot for ALL 3 AXES SIMULTANEOUSLY (Bitmask 0x07)
    set_overrides(ALL_AXES, True)
    write_setting(ALL_AXES, 0x20, "H", 100)   # Force Scale 100%
    write_setting(ALL_AXES, 0xA0, "B", 1)     # Virtual Force Enable (ON)
    write_setting(ALL_AXES, 0x50, "H", 6000)  # Strong motor torque
    write_setting(ALL_AXES, 0x60, "H", 6000)  # Max transit speed
    write_setting(ALL_AXES, 0x70, "B", 1)     # Autopilot ON

    # Move smoothly to start position on all 3 axes
    if p_man_pos:
        write_setting(AXIS_PITCH,  0x80, "f", p_man_pos[0])
    if r_man_pos:
        write_setting(AXIS_ROLL,   0x80, "f", r_man_pos[0])
    if y_man_pos:
        write_setting(AXIS_RUDDER, 0x80, "f", y_man_pos[0])
    time.sleep(0.5)

    print("\n>>> REPLAYING NOW! (Hands off the stick, Feet off the pedals) <<<")
    auto_time = []
    p_auto_pos, p_auto_vf = [], []
    r_auto_pos, r_auto_vf = [], []
    y_auto_pos, y_auto_vf = [], []

    t_play = time.time()
    total = len(man_time)

    # Replay loop with 4-sample lookahead (80ms) & 1.00 gain across all axes at 50 Hz
    for i in range(total):
        t_tick = t_play + (i * INTERVAL_SEC)
        lead_idx = min(total - 1, i + LEAD_SAMPLES)

        # Commands with lead compensation
        p_cmd = max(-1.0, min(1.0, p_man_pos[lead_idx] * GAIN))
        r_cmd = max(-1.0, min(1.0, r_man_pos[lead_idx] * GAIN))
        y_cmd = max(-1.0, min(1.0, y_man_pos[lead_idx] * GAIN))

        write_setting(AXIS_PITCH,  0x80, "f", p_cmd)
        write_setting(AXIS_ROLL,   0x80, "f", r_cmd)
        write_setting(AXIS_RUDDER, 0x80, "f", y_cmd)

        # Read back actual motor positions and dynamic virtual forces
        pp, _, pvf = read_axis(AXIS_PITCH)
        rp, _, rvf = read_axis(AXIS_ROLL)
        yp, _, yvf = read_axis(AXIS_RUDDER)

        auto_time.append(time.time() - t_play)
        p_auto_pos.append(pp); p_auto_vf.append(pvf)
        r_auto_pos.append(rp); r_auto_vf.append(rvf)
        y_auto_pos.append(yp); y_auto_vf.append(yvf)

        # Live visual terminal indicator
        if i % 10 == 0 or i == total - 1:
            print(f"\r  Replaying: {i+1}/{total} | Pitch: {pp:+.2f} | Roll: {rp:+.2f} | Rudder: {yp:+.2f}", end="", flush=True)

        # Precise timing to prevent drift
        dt = (t_tick + INTERVAL_SEC) - time.time()
        if dt > 0:
            time.sleep(dt)

    print("\nPlayback complete!")

finally:
    # Always release overrides on ALL 3 axes safely
    try:
        write_setting(AXIS_PITCH,  0x80, "f", 0.0)
        write_setting(AXIS_ROLL,   0x80, "f", 0.0)
        write_setting(AXIS_RUDDER, 0x80, "f", 0.0)
        time.sleep(0.3)
        write_setting(ALL_AXES, 0x70, "B", 0)
        write_setting(ALL_AXES, 0xA0, "B", 0)
        set_overrides(ALL_AXES, False)
    except Exception:
        pass
    s.close()
    print("Overrides safely released on Pitch, Roll, and Rudder.")

# =====================================================================
# 3. SAVE TELEMETRY TO CSV (ZERO DEPENDENCIES)
# =====================================================================
csv_file = "pitch_roll_rudder_telemetry.csv"
try:
    with open(csv_file, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "Index", "Time_s",
            "Pitch_Man_Pos", "Pitch_Man_Force", "Pitch_Auto_Pos", "Pitch_Auto_VF",
            "Roll_Man_Pos", "Roll_Man_Force", "Roll_Auto_Pos", "Roll_Auto_VF",
            "Rudder_Man_Pos", "Rudder_Man_Force", "Rudder_Auto_Pos", "Rudder_Auto_VF"
        ])
        for i in range(len(man_time)):
            writer.writerow([
                i, f"{man_time[i]:.4f}",
                f"{p_man_pos[i]:.4f}", f"{p_man_force[i]:.4f}",
                f"{p_auto_pos[i] if i < len(p_auto_pos) else 0.0:.4f}",
                f"{p_auto_vf[i] if i < len(p_auto_vf) else 0.0:.4f}",
                f"{r_man_pos[i]:.4f}", f"{r_man_force[i]:.4f}",
                f"{r_auto_pos[i] if i < len(r_auto_pos) else 0.0:.4f}",
                f"{r_auto_vf[i] if i < len(r_auto_vf) else 0.0:.4f}",
                f"{y_man_pos[i]:.4f}", f"{y_man_force[i]:.4f}",
                f"{y_auto_pos[i] if i < len(y_auto_pos) else 0.0:.4f}",
                f"{y_auto_vf[i] if i < len(y_auto_vf) else 0.0:.4f}",
            ])
    print(f"\n[SAVED] CSV Telemetry: {csv_file}")
except Exception as e:
    print(f"Warning: Could not save CSV: {e}")

# =====================================================================
# 4. GENERATE SUPER MUSHSHAK OFFICIAL FLIGHT TEST PDF REPORT
# =====================================================================
try:
    from generate_flight_test_pdf import generate_flight_test_pdf
    pdf_out = generate_flight_test_pdf(csv_file, auto_open=True)
    if pdf_out:
        print(f"[SAVED] Controls PDF Report: {pdf_out}")
except Exception as e:
    print(f"[INFO] Controls PDF skipped: {e}")

# =====================================================================
# 5. AUTOMATIC GRAPH GENERATION (MATPLOTLIB)
# =====================================================================
if HAS_MATPLOTLIB and plt is not None:
    try:
        fig, axes = plt.subplots(2, 3, figsize=(18, 9), sharex=True)
        (ax_p1, ax_r1, ax_y1), (ax_p2, ax_r2, ax_y2) = axes

        # --- COLUMN 1: PITCH ---
        ax_p1.plot(man_time, p_man_pos, label="Manual Pitch (Stick)", color="#1f77b4", linewidth=2.5)
        ax_p1.plot(auto_time, p_auto_pos, label="Auto Pitch (Motor)", color="#d62728", linestyle="--", linewidth=2.0)
        ax_p1.set_title(f"Pitch Tracking (@ {SAMPLE_RATE_HZ}Hz, Lead: {LEAD_MS}ms)", fontsize=11, fontweight="bold")
        ax_p1.set_ylabel("Pitch Position (Norm)")
        ax_p1.set_ylim(-1.05, 1.05)
        ax_p1.grid(True, linestyle="--", alpha=0.6)
        ax_p1.legend(loc="upper right", fontsize=9)

        line_p1 = ax_p2.plot(man_time, p_man_force, label="Manual Pitch Force (0x21)", color="#2ca02c", linewidth=2.0)
        ax_p2.set_ylabel("Manual Force (Norm)", color="#2ca02c")
        ax_p2.tick_params(axis="y", labelcolor="#2ca02c")
        ax_p2.grid(True, linestyle="--", alpha=0.6)
        ax_p2_right = ax_p2.twinx()
        line_p2 = ax_p2_right.plot(auto_time, p_auto_vf, label="Auto Pitch VF (Nm, 0x51)", color="#ff7f0e", linestyle="--", linewidth=2.0)
        ax_p2_right.set_ylabel("Auto Virtual Force (Nm)", color="#ff7f0e")
        ax_p2_right.tick_params(axis="y", labelcolor="#ff7f0e")
        lines_p = line_p1 + line_p2
        ax_p2.legend(lines_p, [l.get_label() for l in lines_p], loc="upper right", fontsize=9)
        ax_p2.set_xlabel("Time (seconds)")

        # --- COLUMN 2: ROLL ---
        ax_r1.plot(man_time, r_man_pos, label="Manual Roll (Stick)", color="#1f77b4", linewidth=2.5)
        ax_r1.plot(auto_time, r_auto_pos, label="Auto Roll (Motor)", color="#d62728", linestyle="--", linewidth=2.0)
        ax_r1.set_title(f"Roll Tracking (@ {SAMPLE_RATE_HZ}Hz, Lead: {LEAD_MS}ms)", fontsize=11, fontweight="bold")
        ax_r1.set_ylabel("Roll Position (Norm)")
        ax_r1.set_ylim(-1.05, 1.05)
        ax_r1.grid(True, linestyle="--", alpha=0.6)
        ax_r1.legend(loc="upper right", fontsize=9)

        line_r1 = ax_r2.plot(man_time, r_man_force, label="Manual Roll Force (0x21)", color="#2ca02c", linewidth=2.0)
        ax_r2.set_ylabel("Manual Force (Norm)", color="#2ca02c")
        ax_r2.tick_params(axis="y", labelcolor="#2ca02c")
        ax_r2.grid(True, linestyle="--", alpha=0.6)
        ax_r2_right = ax_r2.twinx()
        line_r2 = ax_r2_right.plot(auto_time, r_auto_vf, label="Auto Roll VF (Nm, 0x51)", color="#ff7f0e", linestyle="--", linewidth=2.0)
        ax_r2_right.set_ylabel("Auto Virtual Force (Nm)", color="#ff7f0e")
        ax_r2_right.tick_params(axis="y", labelcolor="#ff7f0e")
        lines_r = line_r1 + line_r2
        ax_r2.legend(lines_r, [l.get_label() for l in lines_r], loc="upper right", fontsize=9)
        ax_r2.set_xlabel("Time (seconds)")

        # --- COLUMN 3: RUDDER ---
        ax_y1.plot(man_time, y_man_pos, label="Manual Rudder (Pedals)", color="#1f77b4", linewidth=2.5)
        ax_y1.plot(auto_time, y_auto_pos, label="Auto Rudder (Motor)", color="#d62728", linestyle="--", linewidth=2.0)
        ax_y1.set_title(f"Rudder Tracking (@ {SAMPLE_RATE_HZ}Hz, Lead: {LEAD_MS}ms)", fontsize=11, fontweight="bold")
        ax_y1.set_ylabel("Rudder Position (Norm)")
        ax_y1.set_ylim(-1.05, 1.05)
        ax_y1.grid(True, linestyle="--", alpha=0.6)
        ax_y1.legend(loc="upper right", fontsize=9)

        line_y1 = ax_y2.plot(man_time, y_man_force, label="Manual Rudder Force (0x21)", color="#2ca02c", linewidth=2.0)
        ax_y2.set_ylabel("Manual Force (Norm)", color="#2ca02c")
        ax_y2.tick_params(axis="y", labelcolor="#2ca02c")
        ax_y2.grid(True, linestyle="--", alpha=0.6)
        ax_y2_right = ax_y2.twinx()
        line_y2 = ax_y2_right.plot(auto_time, y_auto_vf, label="Auto Rudder VF (Nm, 0x51)", color="#ff7f0e", linestyle="--", linewidth=2.0)
        ax_y2_right.set_ylabel("Auto Virtual Force (Nm)", color="#ff7f0e")
        ax_y2_right.tick_params(axis="y", labelcolor="#ff7f0e")
        lines_y = line_y1 + line_y2
        ax_y2.legend(lines_y, [l.get_label() for l in lines_y], loc="upper right", fontsize=9)
        ax_y2.set_xlabel("Time (seconds)")

        plt.suptitle(f"Brunner CLS2Sim: 3-Axis Control Loading (Pitch, Roll & Rudder @ {SAMPLE_RATE_HZ}Hz)", fontsize=14, fontweight="bold")
        plt.tight_layout()
        plt.savefig("pitch_roll_rudder_comparison.png", dpi=120)
        print(f"[SAVED] PNG Graph: pitch_roll_rudder_comparison.png")
        print("Opening graph window... (close window to finish)")
        plt.show()
    except Exception as e:
        print(f"Warning: Could not display matplotlib graph: {e}")
else:
    print("\n[INFO] 'matplotlib' is not installed on this PC.")
    print("       (To enable PNG charts, run: pip install matplotlib)")
