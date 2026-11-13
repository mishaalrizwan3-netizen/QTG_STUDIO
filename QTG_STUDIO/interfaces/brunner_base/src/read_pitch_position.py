import socket
import struct
import time
import sys

# =====================================================================
# AXIS DEFINITIONS (Brunner Protocol v2)
# =====================================================================
AXES = {
    "1": {"mask": 0x01, "name": "Pitch (Elevator)"},
    "2": {"mask": 0x02, "name": "Roll (Aileron)"},
    "3": {"mask": 0x04, "name": "Rudder (Yaw)"},
}

# =====================================================================
# SAFE UNPACK HELPERS (Prevents crash if a drive is offline/unplugged)
# =====================================================================
def get_float(data, offset=5):
    return struct.unpack_from("<f", data, offset)[0] if len(data) >= offset + 4 else None

def get_int(data, offset=5):
    return struct.unpack_from("<i", data, offset)[0] if len(data) >= offset + 4 else None

def get_uint(data, offset=5):
    return struct.unpack_from("<I", data, offset)[0] if len(data) >= offset + 4 else None

def get_byte(data, offset=5):
    return struct.unpack_from("<B", data, offset)[0] if len(data) >= offset + 1 else None


# =====================================================================
# INTERACTIVE MENU
# =====================================================================
print("=" * 60)
print("          Brunner CLS2Sim Multi-Axis Controller")
print("=" * 60)
print("Select Mode:")
print("  1. Pitch  (Elevator, 0x01) - Read & Move")
print("  2. Roll   (Aileron,  0x02) - Read & Move")
print("  3. Rudder (Yaw,      0x04) - Read & Move")
print("  4. Record & Replay Movement")

choice = input("\nEnter choice [1/2/3/4] (default 1): ").strip()

if choice == "4":
    try:
        from single_axis import record_replay_pitch
    except ImportError:
        import record_replay_pitch
    if hasattr(record_replay_pitch, "main"):
        record_replay_pitch.main()
    sys.exit(0)

if choice not in AXES:
    choice = "1"

axis_info = AXES[choice]
axis_name = axis_info["name"]
axis_mask = axis_info["mask"]

pos_str = input(f"\nEnter Target Position for {axis_name} (-1.00 to +1.00, default 0.00): ").strip()
try:
    target_pos = float(pos_str)
    target_pos = max(-1.0, min(1.0, target_pos))
except ValueError:
    target_pos = 0.0

speed_str = input("Enter AP Speed (5 to 9999, default 50): ").strip()
try:
    target_speed = int(speed_str)
    target_speed = max(5, min(9999, target_speed))
except ValueError:
    target_speed = 50

print("\n" + "=" * 60)
print(f"  Axis Selected   : {axis_name} (Mask: 0x{axis_mask:02X})")
print(f"  Target Position : {target_pos:+.2f}")
print(f"  AP Speed        : {target_speed}")
print("=" * 60)


# =====================================================================
# CONNECT TO CLS2Sim TCP SERVER (Port 15080)
# =====================================================================
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.settimeout(2.0)
s.connect(("127.0.0.1", 15080))

try:
    # -----------------------------------------------------------------
    # 1. READ AXIS TELEMETRY (Command 0xD0)
    # -----------------------------------------------------------------
    print(f"\n=== 1. Current {axis_name} Readings ===")

    # 1. Position Normalized (0x11, Float)
    s.sendall(struct.pack("<III", 0xD0, axis_mask, 0x11))
    pos_norm = get_float(s.recv(1024), 5)
    print(f"  Position normalized       : {f'{pos_norm:+.4f}' if pos_norm is not None else 'Drive offline/not connected'}")

    # 2. Position RAW (0x12, Int32)
    s.sendall(struct.pack("<III", 0xD0, axis_mask, 0x12))
    pos_raw = get_int(s.recv(1024), 5)
    print(f"  Position RAW              : {pos_raw if pos_raw is not None else 'N/A'}")

    # 3. Force Normalized (0x21, Float)
    s.sendall(struct.pack("<III", 0xD0, axis_mask, 0x21))
    force_norm = get_float(s.recv(1024), 5)
    print(f"  Force Normalized          : {f'{force_norm:+.4f}' if force_norm is not None else 'N/A'}")

    # 4. Force RAW (0x20, Int32)
    s.sendall(struct.pack("<III", 0xD0, axis_mask, 0x20))
    force_raw = get_int(s.recv(1024), 5)
    print(f"  Force RAW                 : {force_raw if force_raw is not None else 'N/A'}")

    # 5. Virtual Force in Eng Units (0x51: Unit at byte 5, Float at byte 7)
    s.sendall(struct.pack("<III", 0xD0, axis_mask, 0x51))
    resp = s.recv(1024)
    v_force = get_float(resp, 7)
    unit_code = struct.unpack_from("<H", resp, 5)[0] if len(resp) >= 7 else 0
    unit_str = "N" if unit_code == 1 else "Nm"
    print(f"  Virtual force in units    : {f'{v_force:+.4f} {unit_str}' if v_force is not None else 'N/A'}")

    # 6. Virtual Force RAW (0x50, UInt32)
    s.sendall(struct.pack("<III", 0xD0, axis_mask, 0x50))
    v_raw = get_uint(s.recv(1024), 5)
    print(f"  Virtual force RAW         : {v_raw if v_raw is not None else 'N/A'}")

    # 7. Autopilot Enabled (0x55, UInt8)
    s.sendall(struct.pack("<III", 0xD0, axis_mask, 0x55))
    ap_enabled = get_byte(s.recv(1024), 5)
    print(f"  AP Enabled                : {'Yes (1)' if ap_enabled == 1 else 'No (0)' if ap_enabled is not None else 'N/A'}")
    print("-" * 55)

    # -----------------------------------------------------------------
    # 2. WRITE MATCHING VARIABLES (Command 0xCE)
    # -----------------------------------------------------------------
    print(f"\n=== 2. Writing Variables to {axis_name} ===")

    # Step A: Enable Overrides so CLS2Sim accepts external writes
    s.sendall(struct.pack("<III", 0xD1, axis_mask, 0x01))  # Force scale override
    s.recv(1024)
    s.sendall(struct.pack("<III", 0xD1, axis_mask, 0x03))  # AP enable override
    s.recv(1024)
    s.sendall(struct.pack("<III", 0xD1, axis_mask, 0x04))  # AP position override
    s.recv(1024)

    # 1. Force Scale (Setting 0x20, UInt16: 100%)
    s.sendall(struct.pack("<IIIH", 0xCE, axis_mask, 0x20, 100))
    print(f"  Write Force Scale (100%)  : Status {s.recv(1024)[2]} (OK)")

    # 2. Virtual Force Enable (Setting 0xA0, UInt8: 1 = ON)
    s.sendall(struct.pack("<IIIB", 0xCE, axis_mask, 0xA0, 1))
    print(f"  Write Virtual Force (ON)  : Status {s.recv(1024)[2]} (OK)")

    # 3. AP Motor Holding Force (Setting 0x50, UInt16: 2000)
    s.sendall(struct.pack("<IIIH", 0xCE, axis_mask, 0x50, 2000))
    print(f"  Write AP Force (2000)     : Status {s.recv(1024)[2]} (OK)")

    # 4. AP Transit Speed (Setting 0x60, UInt16)
    s.sendall(struct.pack("<IIIH", 0xCE, axis_mask, 0x60, target_speed))
    print(f"  Write AP Speed ({target_speed})    : Status {s.recv(1024)[2]} (OK)")

    # 5. AP Enable (Setting 0x70, UInt8: 1 = ON)
    s.sendall(struct.pack("<IIIB", 0xCE, axis_mask, 0x70, 1))
    print(f"  Write AP Enable (ON)      : Status {s.recv(1024)[2]} (OK)")

    # 6. Target Position (Setting 0x80, Float)
    s.sendall(struct.pack("<IIIf", 0xCE, axis_mask, 0x80, target_pos))
    print(f"  Write Target Pos ({target_pos:+.2f}) : Status {s.recv(1024)[2]} (OK)")

    # -----------------------------------------------------------------
    # 3. LIVE VERIFICATION LOOP (Hold for 3 seconds)
    # -----------------------------------------------------------------
    print(f"\nMoving {axis_name} to {target_pos:+.2f} at speed {target_speed} (3 seconds hold):")
    print("    [Time]   Pos (Norm)   Force (Norm)   Virtual Force   AP Enabled")
    print("    " + "-" * 60)

    for i in range(6):
        time.sleep(0.5)

        # 1. Read Position
        s.sendall(struct.pack("<III", 0xD0, axis_mask, 0x11))
        cur_pos = get_float(s.recv(1024), 5)
        pos_txt = f"{cur_pos:+.4f}" if cur_pos is not None else "  N/A  "

        # 2. Read Force
        s.sendall(struct.pack("<III", 0xD0, axis_mask, 0x21))
        cur_force = get_float(s.recv(1024), 5)
        force_txt = f"{cur_force:+.4f}" if cur_force is not None else "  N/A  "

        # 3. Read Virtual Force
        s.sendall(struct.pack("<III", 0xD0, axis_mask, 0x51))
        cur_vf = get_float(s.recv(1024), 7)
        vf_txt = f"{cur_vf:+.4f} Nm" if cur_vf is not None else "  N/A   "

        # 4. Read AP Status
        s.sendall(struct.pack("<III", 0xD0, axis_mask, 0x55))
        cur_ap = get_byte(s.recv(1024), 5)
        ap_txt = "ON (1)" if cur_ap == 1 else "OFF (0)" if cur_ap is not None else "N/A"

        print(f"    +{i*0.5+0.5:.1f}s    {pos_txt}       {force_txt}        {vf_txt}       {ap_txt}")

finally:
    # -----------------------------------------------------------------
    # 4. RELEASE OVERRIDES AND CLOSE SOCKET
    # -----------------------------------------------------------------
    try:
        s.sendall(struct.pack("<III", 0xD1, axis_mask, 0x01))  # Release Force Scale
        s.recv(1024)
        s.sendall(struct.pack("<III", 0xD1, axis_mask, 0x03))  # Release AP Enable
        s.recv(1024)
        s.sendall(struct.pack("<III", 0xD1, axis_mask, 0x04))  # Release AP Position
        s.recv(1024)
    except Exception:
        pass

    s.close()
    print("\nDone. Overrides released back to normal.")
