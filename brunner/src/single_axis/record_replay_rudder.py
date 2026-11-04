import socket
import struct
import time
import csv
import os

# Optional matplotlib import (prevents ModuleNotFoundError on test PCs)
try:
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
except ImportError:
    plt = None
    HAS_MATPLOTLIB = False

def generate_recording_graph(man_time, man_pos, man_force, filename="recorded_rudder.png"):
    """Automatically generates and displays graph immediately when recording finishes."""
    if not man_time or not HAS_MATPLOTLIB or plt is None:
        return None
    try:
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
        ax1.plot(man_time, man_pos, color="#0284c7", linewidth=2.2, label="Manual Rudder Position")
        ax1.set_title(f"Recorded Rudder Movement ({len(man_time)} samples @ {SAMPLE_RATE_HZ}Hz)", fontsize=12, fontweight="bold")
        ax1.set_ylabel("Position (Norm -1 to +1)")
        ax1.set_ylim(-1.05, 1.05)
        ax1.grid(True, linestyle="--", alpha=0.5)
        ax1.legend(loc="upper right")

        ax2.plot(man_time, man_force, color="#16a34a", linewidth=1.8, label="Pedal Applied Force (0x21)")
        ax2.set_ylabel("Applied Force (Norm)")
        ax2.set_xlabel("Time (seconds)")
        ax2.grid(True, linestyle="--", alpha=0.5)
        ax2.legend(loc="upper right")

        plt.tight_layout()
        plt.savefig(filename, dpi=120)
        plt.close(fig)
        print(f"[SAVED] Recording Graph: {filename}")
        try:
            os.startfile(filename)
            print(f"[OPENED] Automatically opened {filename}")
        except Exception:
            pass
        return filename
    except Exception as e:
        print(f"Warning: Could not plot recording: {e}")
        return None

# =====================================================================
# 1. SETUP UDP CONNECTION TO CLS2SIM (PORT 15090) - RUDDER AXIS (0x04)
# =====================================================================
AXIS_RUDDER = 0x04  # Rudder / Yaw Axis

# Sampling & Timing Configuration (50 Hz / 20 ms)
SAMPLE_RATE_HZ = 50                                             # 50 Hz update rate
INTERVAL_SEC   = 1.0 / SAMPLE_RATE_HZ                          # 0.020s (20 ms)
LEAD_MS        = 80                                            # 80 ms lead compensation
LEAD_SAMPLES   = round((LEAD_MS / 1000.0) * SAMPLE_RATE_HZ)      # 4 samples at 50 Hz
GAIN           = 1.00                                          # 1:1 Unity gain

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.settimeout(2.0)
s.connect(("127.0.0.1", 15090))

def read_rudder():
    """Reads Rudder Position (0x11), Pedal Applied Force (0x21), and Virtual Force (0x51)"""
    pos, force, v_force = 0.0, 0.0, 0.0
    try:
        # 1. Position (-1.0 to +1.0)
        s.send(struct.pack("<III", 0xD0, AXIS_RUDDER, 0x11))
        r = s.recv(1024)
        if len(r) >= 9:
            pos = struct.unpack_from("<f", r, 5)[0]

        # 2. Applied Pedal Force (Load cell)
        s.send(struct.pack("<III", 0xD0, AXIS_RUDDER, 0x21))
        r = s.recv(1024)
        if len(r) >= 9:
            force = struct.unpack_from("<f", r, 5)[0]

        # 3. Virtual Aerodynamic Force (in Nm)
        s.send(struct.pack("<III", 0xD0, AXIS_RUDDER, 0x51))
        r = s.recv(1024)
        if len(r) >= 11:
            v_force = struct.unpack_from("<f", r, 7)[0]
    except Exception:
        pass

    return pos, force, v_force

def write_setting(setting_id, fmt, val):
    try:
        s.send(struct.pack(f"<III{fmt}", 0xCE, AXIS_RUDDER, setting_id, val))
        s.recv(1024)
    except Exception:
        pass

def set_overrides(enable):
    """Enables (True) or releases (False) Autopilot overrides for Rudder"""
    try:
        val = 1 if enable else 0
        s.send(struct.pack("<III", 0xD1, AXIS_RUDDER, 0x01 if val else 0x00))
        s.recv(1024)  # Force scale
        s.send(struct.pack("<III", 0xD1, AXIS_RUDDER, 0x03 if val else 0x00))
        s.recv(1024)  # AP enable
        s.send(struct.pack("<III", 0xD1, AXIS_RUDDER, 0x04 if val else 0x00))
        s.recv(1024)  # AP position
    except Exception:
        pass

# =====================================================================
# 2. RECORD MANUAL RUDDER & REPLAY AUTOMATICALLY
# =====================================================================
try:
    sec = input("Enter recording duration in seconds (default 5): ").strip()
    duration = float(sec) if sec else 5.0

    # Countdown
    print(f"\nRecording Rudder for {duration:.1f} seconds. Place feet on pedals...")
    for c in [3, 2, 1]:
        print(f"  Starting in {c}...", end="\r", flush=True)
        time.sleep(1.0)
    print(">>> RECORDING NOW! Move the Rudder pedals! <<<          ")

    # Record manual movement at 50 Hz
    man_time, man_pos, man_force = [], [], []
    t0 = time.time()
    next_tick = 0.0
    while time.time() - t0 < duration:
        t = time.time() - t0
        p, f, _ = read_rudder()
        man_time.append(t)
        man_pos.append(p)
        man_force.append(f)

        next_tick += INTERVAL_SEC
        sleep_dur = next_tick - (time.time() - t0)
        if sleep_dur > 0:
            time.sleep(sleep_dur)

    print(f"\nRecording complete! ({len(man_pos)} points captured)")
    generate_recording_graph(man_time, man_pos, man_force, filename="recorded_rudder.png")
    input("\nPress ENTER to automatically move the physical pedals... (or Ctrl+C to exit)")

    # Configure Autopilot: High speed & high torque to eliminate lag
    set_overrides(True)
    write_setting(0x20, "H", 100)   # Force Scale 100%
    write_setting(0xA0, "B", 1)     # Virtual Force Enable (ON)
    write_setting(0x50, "H", 6000)  # Strong motor torque
    write_setting(0x60, "H", 6000)  # Max transit speed
    write_setting(0x70, "B", 1)     # Autopilot ON

    # Move smoothly to start position
    write_setting(0x80, "f", man_pos[0])
    print("Stabilizing CLS2Sim rudder forces (waiting 3.0s for trim settling)...")
    time.sleep(3.0)

    print("\n>>> REPLAYING NOW! (Feet off the pedals) <<<")
    auto_time, auto_pos, auto_vf = [], [], []
    t_play = time.time()
    total = len(man_pos)

    # Lead compensation: LEAD_SAMPLES samples (LEAD_MS ms) + GAIN gain at SAMPLE_RATE_HZ Hz
    for i in range(total):
        t_tick = t_play + (i * INTERVAL_SEC)

        lead_idx = min(total - 1, i + LEAD_SAMPLES)
        cmd = max(-1.0, min(1.0, man_pos[lead_idx] * GAIN))
        write_setting(0x80, "f", cmd)

        p, _, vf = read_rudder()
        auto_time.append(time.time() - t_play)
        auto_pos.append(p)
        auto_vf.append(vf)

        # Precise timing to prevent timer drift
        dt = (t_tick + INTERVAL_SEC) - time.time()
        if dt > 0:
            time.sleep(dt)

    print("Playback complete!")

finally:
    # Always release overrides back to flight simulator
    try:
        write_setting(0x80, "f", 0.0)
        time.sleep(0.3)
        write_setting(0x70, "B", 0)
        write_setting(0xA0, "B", 0)
    except Exception:
        pass
    set_overrides(False)
    s.close()
    print("Overrides released back to normal.")

# =====================================================================
# 3. SAVE TELEMETRY TO CSV (ZERO DEPENDENCIES)
# =====================================================================
csv_file = "rudder_telemetry.csv"
try:
    with open(csv_file, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["Index", "Man_Time_s", "Man_Pos", "Man_Force", "Auto_Time_s", "Auto_Pos", "Auto_Virtual_Force"])
        for i in range(len(man_time)):
            writer.writerow([
                i,
                f"{man_time[i]:.4f}",
                f"{man_pos[i]:.4f}",
                f"{man_force[i]:.4f}",
                f"{auto_time[i] if i < len(auto_time) else 0.0:.4f}",
                f"{auto_pos[i] if i < len(auto_pos) else 0.0:.4f}",
                f"{auto_vf[i] if i < len(auto_vf) else 0.0:.4f}"
            ])
    print(f"\n[SAVED] CSV Telemetry: {csv_file}")
except Exception as e:
    print(f"Warning: Could not save CSV: {e}")

# =====================================================================
# 3b. GENERATE SUPER MUSHSHAK OFFICIAL FLIGHT TEST CONTROLS PDF REPORT
# =====================================================================
try:
    from generate_flight_test_pdf import generate_flight_test_pdf
    pdf_out = generate_flight_test_pdf(csv_file, auto_open=True)
    if pdf_out:
        print(f"[SAVED] Controls PDF Report: {pdf_out}")
except Exception as e:
    print(f"[INFO] Controls PDF skipped: {e}")

# =====================================================================
# 4. PLOT COMPARISON GRAPH (SKIPPED IF MATPLOTLIB IS MISSING)
# =====================================================================
if HAS_MATPLOTLIB and plt is not None:
    try:
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)

        # Subplot 1: Rudder Position Comparison
        ax1.plot(man_time, man_pos, label="Manual Rudder (Pedal Input)", color="#1f77b4", linewidth=2.5)
        ax1.plot(auto_time, auto_pos, label="Automatic Rudder (Motor Replay)", color="#d62728", linestyle="--", linewidth=2.0)
        ax1.set_title(f"Brunner CLS2Sim: Rudder Tracking (UDP @ {SAMPLE_RATE_HZ}Hz, Lead: {LEAD_MS}ms)", fontsize=13, fontweight="bold")
        ax1.set_ylabel("Rudder Position (Norm)", fontsize=11)
        ax1.set_ylim(-1.05, 1.05)
        ax1.grid(True, linestyle="--", alpha=0.6)
        ax1.legend(loc="upper right")

        # Subplot 2: Manual Applied Force vs Inverted Automatic Virtual Force
        line1 = ax2.plot(man_time, man_force, label="Manual Pedal Force (Load Cell, 0x21)", color="#2ca02c", linewidth=2.0)
        ax2.set_ylabel("Manual Pedal Force (Norm)", color="#2ca02c", fontsize=11)
        ax2.tick_params(axis="y", labelcolor="#2ca02c")
        ax2.grid(True, linestyle="--", alpha=0.6)

        # Right Axis for Virtual Force (in Nm)
        ax2_right = ax2.twinx()
        line2 = ax2_right.plot(auto_time, auto_vf, label="Auto Virtual Force (Nm, 0x51)", color="#ff7f0e", linestyle="--", linewidth=2.0)
        ax2_right.set_ylabel("Auto Virtual Force (Nm)", color="#ff7f0e", fontsize=11)
        ax2_right.tick_params(axis="y", labelcolor="#ff7f0e")

        # Combined legend on bottom plot
        lines = line1 + line2
        labels = [l.get_label() for l in lines]
        ax2.legend(lines, labels, loc="upper right")
        ax2.set_xlabel("Time (seconds)", fontsize=11)

        plt.tight_layout()
        plt.savefig("rudder_comparison.png", dpi=120)
        print("Graph saved to: rudder_comparison.png")
        print("Opening graph window... (close window to finish)")
        plt.show()
    except Exception as e:
        print(f"Warning: Could not show plot: {e}")
else:
    print("\n[INFO] 'matplotlib' is not installed on this PC.")
    print(f"       CSV Telemetry was saved to: {csv_file}")
    print("       (To enable PNG graphs, run: pip install matplotlib)")

