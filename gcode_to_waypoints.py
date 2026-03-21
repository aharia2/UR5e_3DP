#!/usr/bin/env python3
"""
gcode_to_waypoints.py
=====================
Convert a PrusaSlicer (or similar) G-code file into a waypoints.csv that
gcode_executor.py / cartesian_mover.py can execute.

Coordinate transform
--------------------
  robot_x = (gcode_x - GCODE_CENTER_X) / 1000.0 + ROBOT_CENTER_X
  robot_y = (gcode_y - GCODE_CENTER_Y) / 1000.0 + ROBOT_CENTER_Y
  robot_z = gcode_z  / 1000.0 + ROBOT_Z_OFFSET

Tune the SIX constants in the "─── User settings ───" block below.

Usage
-----
  python3 gcode_to_waypoints.py  [input.gcode]  [output.csv]

Defaults:
  input  = Ian/UR5e-Dev/print.gcode
  output = waypoints.csv  (overwrites the current file)
"""

import sys
import os
import math

# ─── User settings ────────────────────────────────────────────────────────────
# G-code bounding-box centre (mm) — adjust to match your print's geometry.
# Tip: (min_x+max_x)/2 from the G-code; see the first LAYER comments.
GCODE_CENTER_X = 94.0        # mm  (Prusa coordinate space)
GCODE_CENTER_Y = 100.0       # mm

# Where that centre should land in the robot's base_link frame (m)
ROBOT_CENTER_X = 0.300       # m
ROBOT_CENTER_Y = 0.100       # m

# Height offset: where Klipper Z=0 maps to in robot frame (m)
# Typically your print surface height above base_link.
ROBOT_Z_OFFSET = 0.050       # m

# Fine-tune offsets added to EVERY waypoint AFTER the main transform (m).
# Use these to shift the entire print without changing the centre-point math.
X_OFFSET = 0.000             # m  positive = towards robot's +X
Y_OFFSET = 0.000             # m  positive = towards robot's +Y
Z_OFFSET = 0.03             # m  positive = up

# Orientation: tool pointing straight down (matches existing waypoints.csv)
QX, QY, QZ, QW = 0.70711, 0.00056, 0.00056, 0.70711

# Minimum segment length to keep (m). Shorter moves are skipped.
# Increase to reduce waypoint count, decrease for more detail.
MIN_SEGMENT_M = 0.002        # 2 mm

# Only keep moves where filament is actually extruded (E increases).
# Set False to include all G1 XY moves (travel + extrusion).
EXTRUSION_ONLY = True

# Velocity scaling for all waypoints (0.0–1.0, written to the CSV)
VELOCITY_SCALE = 0.3

# Extrusion amount written per waypoint (mm sent to Klipper)
# You can also write "auto" here to use the delta-E from the G-code.
EXTRUDE_MM = "auto"          # or e.g. 1.0
# ──────────────────────────────────────────────────────────────────────────────


def gcode_to_robot(gx, gy, gz):
    rx = (gx - GCODE_CENTER_X) / 1000.0 + ROBOT_CENTER_X + X_OFFSET
    ry = (gy - GCODE_CENTER_Y) / 1000.0 + ROBOT_CENTER_Y + Y_OFFSET
    rz = gz  / 1000.0 + ROBOT_Z_OFFSET + Z_OFFSET
    return rx, ry, rz


def parse_gcode(path):
    """
    Parse G-code and return a list of dicts:
        { 'x', 'y', 'z', 'e_delta', 'f' }
    Only G1 moves with an XY component are returned.
    Travel moves (EXTRUSION_ONLY=True and e_delta<=0) are dropped.
    """
    waypoints = []

    cur_x = cur_y = cur_z = 0.0
    cur_e = 0.0
    absolute_xyz = True   # G90
    absolute_e   = True   # M82

    with open(path, 'r') as fh:
        for raw in fh:
            line = raw.split(';')[0].strip().upper()  # strip comments
            if not line:
                continue

            if line == 'G90':
                absolute_xyz = True;  continue
            if line == 'G91':
                absolute_xyz = False; continue
            if line == 'M82':
                absolute_e = True;    continue
            if line == 'M83':
                absolute_e = False;   continue
            if line.startswith('G92'):
                # Reset E (or all axes) — only handle E reset
                if 'E' in line:
                    cur_e = _val(line, 'E', 0.0)
                continue

            if not (line.startswith('G0 ') or line.startswith('G1 ')
                    or line == 'G0' or line == 'G1'):
                continue

            # Parse fields
            nx = _val(line, 'X', None)
            ny = _val(line, 'Y', None)
            nz = _val(line, 'Z', None)
            ne = _val(line, 'E', None)

            # Update position
            if absolute_xyz:
                new_x = nx if nx is not None else cur_x
                new_y = ny if ny is not None else cur_y
                new_z = nz if nz is not None else cur_z
            else:
                new_x = cur_x + (nx or 0.0)
                new_y = cur_y + (ny or 0.0)
                new_z = cur_z + (nz or 0.0)

            if ne is not None:
                new_e = ne if absolute_e else cur_e + ne
            else:
                new_e = cur_e

            e_delta = new_e - cur_e

            # Only record moves that change XY
            if nx is not None or ny is not None:
                if not EXTRUSION_ONLY or e_delta > 1e-6:
                    waypoints.append({
                        'x': new_x,
                        'y': new_y,
                        'z': new_z,
                        'e_delta': max(e_delta, 0.0),
                    })

            cur_x, cur_y, cur_z, cur_e = new_x, new_y, new_z, new_e

    return waypoints


def _val(line, letter, default):
    """Extract float value for a G-code letter (e.g. X, Y, E)."""
    i = line.find(letter)
    if i < 0:
        return default
    j = i + 1
    end = j
    while end < len(line) and (line[end] in '0123456789.-+'):
        end += 1
    try:
        return float(line[j:end])
    except ValueError:
        return default


def dist(a, b):
    return math.sqrt((a['x']-b['x'])**2 + (a['y']-b['y'])**2)


def downsample(pts, min_dist_m):
    """Drop waypoints that are closer than min_dist_m to the previous kept one."""
    if not pts:
        return pts
    kept = [pts[0]]
    acc_e = pts[0]['e_delta']
    for p in pts[1:]:
        rx_prev, ry_prev, _ = gcode_to_robot(kept[-1]['x'], kept[-1]['y'], kept[-1]['z'])
        rx_cur,  ry_cur,  _ = gcode_to_robot(p['x'],        p['y'],        p['z'])
        d = math.sqrt((rx_cur-rx_prev)**2 + (ry_cur-ry_prev)**2)
        acc_e += p['e_delta']
        if d >= min_dist_m:
            kept[-1] = kept[-1].copy()  # don't mutate
            p = p.copy()
            p['e_delta'] = acc_e       # accumulate skipped extrusion
            kept.append(p)
            acc_e = 0.0
    return kept


def write_csv(waypoints, out_path):
    lines = [
        "# Cartesian waypoints — auto-generated from G-code by gcode_to_waypoints.py",
        "# Format: x, y, z, qx, qy, qz, qw [, extrude_mm [, velocity_scale]]",
        f"# Source transform: gcode_center=({GCODE_CENTER_X},{GCODE_CENTER_Y}) mm  "
        f"robot_center=({ROBOT_CENTER_X},{ROBOT_CENTER_Y}) m  z_offset={ROBOT_Z_OFFSET} m",
        f"# Fine-tune offsets: X={X_OFFSET:+.4f} m  Y={Y_OFFSET:+.4f} m  Z={Z_OFFSET:+.4f} m",
        f"# Points: {len(waypoints)}  min_segment: {MIN_SEGMENT_M*1000:.1f} mm  extrusion_only: {EXTRUSION_ONLY}",
    ]

    for p in waypoints:
        rx, ry, rz = gcode_to_robot(p['x'], p['y'], p['z'])

        if EXTRUDE_MM == "auto":
            extrude = p['e_delta']
        else:
            extrude = float(EXTRUDE_MM)

        lines.append(
            f"{rx:.6f}, {ry:.6f}, {rz:.6f}, "
            f"{QX}, {QY}, {QZ}, {QW}, "
            f"{extrude:.5f}, {VELOCITY_SCALE}"
        )

    with open(out_path, 'w') as fh:
        fh.write('\n'.join(lines) + '\n')

    print(f"Wrote {len(waypoints)} waypoints → {out_path}")


def main():
    default_gcode = os.path.join(os.path.dirname(__file__),
                                 'Ian', 'UR5e-Dev', '40x40x0.9 Box.gcode')
    default_csv   = os.path.join(os.path.dirname(__file__), 'waypoints.csv')

    in_path  = sys.argv[1] if len(sys.argv) > 1 else default_gcode
    out_path = sys.argv[2] if len(sys.argv) > 2 else default_csv

    if not os.path.exists(in_path):
        print(f"ERROR: G-code file not found: {in_path}")
        print("Usage: python3 gcode_to_waypoints.py <input.gcode> [output.csv]")
        sys.exit(1)

    print(f"Parsing: {in_path}")
    raw = parse_gcode(in_path)
    print(f"  Raw extrusion moves:  {len(raw)}")

    filtered = downsample(raw, MIN_SEGMENT_M)
    print(f"  After downsampling:   {len(filtered)}")

    # Stats
    if filtered:
        xs = [gcode_to_robot(p['x'], p['y'], p['z'])[0] for p in filtered]
        ys = [gcode_to_robot(p['x'], p['y'], p['z'])[1] for p in filtered]
        zs = [gcode_to_robot(p['x'], p['y'], p['z'])[2] for p in filtered]
        print(f"  Robot X: {min(xs):.4f} → {max(xs):.4f} m")
        print(f"  Robot Y: {min(ys):.4f} → {max(ys):.4f} m")
        print(f"  Robot Z: {min(zs):.4f} → {max(zs):.4f} m")

    write_csv(filtered, out_path)


if __name__ == '__main__':
    main()
