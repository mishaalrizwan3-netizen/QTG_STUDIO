import socket
import struct
import time
try:
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
except ImportError:
    plt = None
    HAS_MATPLOTLIB = False

# =====================================================================
# 1. SETUP UDP CONNECTION TO CLS2SIM (PORT 15090) - ROLL AXIS (0x02)
# =====================================================================
AXIS_ROLL = 0x02  # Roll / Aileron Axis

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.settimeout(2.0)
s.connect(("127.0.0.1", 15090))

def read_roll():
    """Reads Roll Position (0x11), Hand Applied Force (0x21), and Virtual Force (0x51)"""
    # 1. Position (-1.0 to +1.0)
    s.send(struct.pack("<III", 0xD0, AXIS_ROLL, 0x11))
    pos = struct.unpack_from("<f", s.recv(1024), 5)[0]

    # 2. Applied Hand Force (Load cell)
    s.send(struct.pack("<III", 0xD0, AXIS_ROLL, 0x21))
    force = struct.unpack_from("<f", s.recv(1024), 5)[0]

    # 3. Virtual Aerodynamic Force (in Nm) - Inverted to match applied force direction
    s.send(struct.pack("<III", 0xD0, AXIS_ROLL, 0x51))
    raw_vf = struct.unpack_from("<f", s.recv(1024), 7)[0]
    v_force = -raw_vf  # Inverted

    return pos, force, v_force

def write_setting(setting_id, fmt, val):
    s.send(struct.pack(f"<III{fmt}", 0xCE, AXIS_ROLL, setting_id, val))
    s.recv(1024)

def set_overrides(enable):
    """Enables (True) or releases (False) Autopilot overrides for Roll"""
    val = 1 if enable else 0
    s.send(struct.pack("<III", 0xD1, AXIS_ROLL, 0x01 if val else 0x00)); s.recv(1024) # Force scale
    s.send(struct.pack("<III", 0xD1, AXIS_ROLL, 0x03 if val else 0x00)); s.recv(1024) # AP enable
    s.send(struct.pack("<III", 0xD1, AXIS_ROLL, 0x04 if val else 0x00)); s.recv(1024) # AP position

# =====================================================================
# 2. RECORD MANUAL ROLL & REPLAY AUTOMATICALLY
# =====================================================================
try:
    sec = input("Enter recording duration in seconds (default 5): ").strip()
    duration = float(sec) if sec else 5.0

    # Countdown
    print(f"\nRecording Roll for {duration:.1f} seconds. Place hand on stick...")
    for c in [3, 2, 1]:
        print(f"  Starting in {c}...", end="\r", flush=True)
        time.sleep(1.0)
    print(">>> RECORDING NOW! Move the stick in Roll (left/right)! <<<          ")

    # Record manual movement at 25 Hz
    man_time, man_pos, man_force = [], [], []
    t0 = time.time()
    while time.time() - t0 < duration:
        t = time.time() - t0
        p, f, _ = read_roll()
        man_time.append(t)
        man_pos.append(p)
        man_force.append(f)
        time.sleep(0.04)

    print(f"\nRecording complete! ({len(man_pos)} points captured)")
    input("Press ENTER to automatically move the physical stick...")

    # Configure Autopilot: High speed & high torque to eliminate lag
    set_overrides(True)
    write_setting(0x20, "H", 100)   # Force Scale 100%
    write_setting(0x50, "H", 6000)  # Strong motor torque
    write_setting(0x60, "H", 6000)  # Max transit speed
    write_setting(0x70, "B", 1)     # Autopilot ON

    # Move smoothly to start position
    write_setting(0x80, "f", man_pos[0])
    time.sleep(0.5)

    print("\n>>> REPLAYING NOW! (Hands off the stick) <<<")
    auto_time, auto_pos, auto_vf = [], [], []
    t_play = time.time()
    total = len(man_pos)

    # Lead compensation: 6 samples (240ms) + 1.12 gain to cancel firmware damping & phase lag
    for i in range(total):
        t_tick = t_play + (i * 0.04)

        lead_idx = min(total - 1, i + 6)
        cmd = max(-1.0, min(1.0, man_pos[lead_idx] * 1.12))
        write_setting(0x80, "f", cmd)

        p, _, vf = read_roll()
        auto_time.append(time.time() - t_play)
        auto_pos.append(p)
        auto_vf.append(vf)

        # Precise timing to prevent timer drift
        dt = (t_tick + 0.04) - time.time()
        if dt > 0:
            time.sleep(dt)

    print("Playback complete!")

finally:
    # Always release overrides back to flight simulator
    set_overrides(False)
    s.close()
    print("Overrides released back to normal.")

# =====================================================================
# 3. PLOT COMPARISON GRAPH (MANUAL VS AUTOMATIC ROLL)
# =====================================================================
if HAS_MATPLOTLIB and plt is not None:
    try:
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)

        # Subplot 1: Roll Position Comparison
        ax1.plot(man_time, man_pos, label="Manual Roll (Hand Input)", color="#1f77b4", linewidth=2.5)
        ax1.plot(auto_time, auto_pos, label="Automatic Roll (Motor Replay)", color="#d62728", linestyle="--", linewidth=2.0)
        ax1.set_title("Brunner CLS2Sim: Roll Position & Force Tracking (UDP)", fontsize=13, fontweight="bold")
        ax1.set_ylabel("Roll Position (Norm)", fontsize=11)
        ax1.set_ylim(-1.05, 1.05)
        ax1.grid(True, linestyle="--", alpha=0.6)
        ax1.legend(loc="upper right")

        # Subplot 2: Manual Applied Force vs Inverted Automatic Virtual Force
        line1 = ax2.plot(man_time, man_force, label="Manual Applied Force (Hand Load Cell, 0x21)", color="#2ca02c", linewidth=2.0)
        ax2.set_ylabel("Manual Applied Force (Norm)", color="#2ca02c", fontsize=11)
        ax2.tick_params(axis="y", labelcolor="#2ca02c")
        ax2.grid(True, linestyle="--", alpha=0.6)

        # Right Axis for Inverted Virtual Force (in Nm)
        ax2_right = ax2.twinx()
        line2 = ax2_right.plot(auto_time, auto_vf, label="Auto Virtual Force (Inverted, 0x51)", color="#ff7f0e", linestyle="--", linewidth=2.0)
        ax2_right.set_ylabel("Auto Virtual Force (Inverted Nm)", color="#ff7f0e", fontsize=11)
        ax2_right.tick_params(axis="y", labelcolor="#ff7f0e")

        # Combined legend on bottom plot
        lines = line1 + line2
        labels = [l.get_label() for l in lines]
        ax2.legend(lines, labels, loc="upper right")
        ax2.set_xlabel("Time (seconds)", fontsize=11)

        plt.tight_layout()
        plt.savefig("roll_comparison.png", dpi=120)
        print("\nGraph saved to: roll_comparison.png")
        print("Opening graph window... (close window to finish)")
        plt.show()
    except Exception as e:
        print(f"Warning: Could not display matplotlib graph: {e}")
else:
    print("\n[INFO] 'matplotlib' is not installed on this PC.")

