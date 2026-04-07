#!/usr/bin/env python3
"""
gcode_to_waypoints_continuous.py
=================================
Convert a PrusaSlicer (or similar) G-code file into a waypoints_continuous.csv
that gcode_executor_continuous.py can execute as continuous Cartesian paths.

Differences from gcode_to_waypoints.py
---------------------------------------
  - Travel moves (e_delta = 0) are always included alongside extrusion moves.
  - A trailing `is_travel` column (0 = extrusion, 1 = travel) lets the executor
    group consecutive extrusion waypoints into one multi-point Cartesian path,
    and handle travel moves as discrete repositioning moves with no extrusion.

CSV format
----------
  x, y, z, qx, qy, qz, qw, extrude_mm, velocity_scale, is_travel

Coordinate transform
--------------------
  robot_x = (gcode_x - GCODE_CENTER_X) / 1000.0 + ROBOT_CENTER_X
  robot_y = (gcode_y - GCODE_CENTER_Y) / 1000.0 + ROBOT_CENTER_Y
  robot_z = gcode_z  / 1000.0 + ROBOT_Z_OFFSET

Tune the SIX constants in the "─── User settings ───" block below.

Usage
-----
  python3 gcode_to_waypoints_continuous.py  [input.gcode]  [output.csv]

Defaults:
  input  = Ian/UR5e-Dev/print.gcode
  output = waypoints_continuous.csv
"""

import sys
import os
import math
import numpy as np

# ─── User settings ────────────────────────────────────────────────────────────
# G-code bounding-box centre (mm) — adjust to match your print's geometry.
GCODE_CENTER_X = 94.0        # mm  (Prusa coordinate space)
GCODE_CENTER_Y = 100.0       # mm

# Where that centre should land in the robot's base_link frame (m)
ROBOT_CENTER_X = 0.5      # m
ROBOT_CENTER_Y = 0.0       # m

# Height offset: where Klipper Z=0 maps to in robot frame (m)
ROBOT_Z_OFFSET = -0.7       # m

# Fine-tune offsets added to EVERY waypoint AFTER the main transform (m).
X_OFFSET = 0.000             # m
Y_OFFSET = 0.000             # m
Z_OFFSET = 0.000             # m

# Orientation: nozzle pointing down — Y axis of tool_tip = +Z base_link
QX, QY, QZ, QW = 0.5, 0.5, 0.5, 0.5

# Minimum segment length to keep (m) — applied within extrusion runs only.
# Travel moves are never downsampled.
MIN_SEGMENT_M = 0.002        # 2 mm

# Velocity scaling for all waypoints (0.0–1.0, written to the CSV)
VELOCITY_SCALE = 0.04


# Extrusion amount written per waypoint (mm sent to Klipper).
# "auto" uses the delta-E from the G-code.
EXTRUDE_MM = "auto"

# ─── Bed leveling ─────────────────────────────────────────────────────────────
BED_CORNER_FRAME = "tool_tip"         # "tool_tip" or "tool0"
FLANGE_TO_TIP    = (0.0, 0.0, -0.150)   # metres, in base_link frame (tool pointing down)

USE_BED_LEVELING = True
BED_CORNERS = [
    (0.5783,  0.0346,  0.0786),  # corner 1
    (0.5783,  0.0247,  0.0786),  # corner 2
    (0.5683,  0.0347,  0.0786),  # corner 3
    (0.5683,  0.0247,  0.0786),  # corner 4
]
# ──────────────────────────────────────────────────────────────────────────────

_bed_transform = None


def _mat_to_quat(R):
    """Convert 3x3 rotation matrix to quaternion (x,y,z,w)."""
    trace = R[0,0] + R[1,1] + R[2,2]
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2,1] - R[1,2]) * s
        y = (R[0,2] - R[2,0]) * s
        z = (R[1,0] - R[0,1]) * s
    elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
        s = 2.0 * math.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2])
        w = (R[2,1] - R[1,2]) / s
        x = 0.25 * s
        y = (R[0,1] + R[1,0]) / s
        z = (R[0,2] + R[2,0]) / s
    elif R[1,1] > R[2,2]:
        s = 2.0 * math.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2])
        w = (R[0,2] - R[2,0]) / s
        x = (R[0,1] + R[1,0]) / s
        y = 0.25 * s
        z = (R[1,2] + R[2,1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1])
        w = (R[1,0] - R[0,1]) / s
        x = (R[0,2] + R[2,0]) / s
        y = (R[1,2] + R[2,1]) / s
        z = 0.25 * s
    n = math.sqrt(x*x + y*y + z*z + w*w)
    return np.array([x/n, y/n, z/n, w/n])


def _quat_mul(q1, q2):
    """Multiply two quaternions (x,y,z,w). Result applies q1 then q2."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    ])


def _get_bed_transform():
    """
    Build and cache the bed coordinate frame from BED_CORNERS.
    Returns (origin, R, tool_quat).
    """
    global _bed_transform
    if _bed_transform is not None:
        return _bed_transform

    pts = np.array(BED_CORNERS, dtype=float)

    if BED_CORNER_FRAME == "tool0":
        tip_offset = np.array(FLANGE_TO_TIP, dtype=float)
        pts = pts + tip_offset
        print(f"[BedLevel] Applying flange→tip offset: {tip_offset} m")

    origin = pts.mean(axis=0)

    _, _, Vt = np.linalg.svd(pts - origin)
    normal = Vt[-1]
    if normal[2] < 0:
        normal = -normal

    wx     = np.array([1.0, 0.0, 0.0])
    x_axis = wx - np.dot(wx, normal) * normal
    if np.linalg.norm(x_axis) < 1e-6:
        wx     = np.array([0.0, 1.0, 0.0])
        x_axis = wx - np.dot(wx, normal) * normal
    x_axis = x_axis / np.linalg.norm(x_axis)

    y_axis = np.cross(normal, x_axis)
    y_axis = y_axis / np.linalg.norm(y_axis)

    R = np.column_stack([x_axis, y_axis, normal])

    q_flat  = np.array([QX, QY, QZ, QW])
    q_bed_R = _mat_to_quat(R)
    tq      = _quat_mul(q_bed_R, q_flat)
    tool_quat = (float(tq[0]), float(tq[1]), float(tq[2]), float(tq[3]))

    print(f"[BedLevel] Normal : {normal}")
    print(f"[BedLevel] Origin : {origin}")
    print(f"[BedLevel] Tool Q : {tool_quat}")

    _bed_transform = (origin, R, tool_quat)
    return _bed_transform


def gcode_to_robot(gx, gy, gz):
    if USE_BED_LEVELING:
        origin, R, _ = _get_bed_transform()
        local = np.array([
            (gx - GCODE_CENTER_X) / 1000.0 + X_OFFSET,
            (gy - GCODE_CENTER_Y) / 1000.0 + Y_OFFSET,
            gz / 1000.0 + Z_OFFSET,
        ])
        rp = R @ local + origin
        return float(rp[0]), float(rp[1]), float(rp[2])
    else:
        rx = (gx - GCODE_CENTER_X) / 1000.0 + ROBOT_CENTER_X + X_OFFSET
        ry = (gy - GCODE_CENTER_Y) / 1000.0 + ROBOT_CENTER_Y + Y_OFFSET
        rz =  gz / 1000.0 + ROBOT_Z_OFFSET + Z_OFFSET
        return rx, ry, rz


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


def parse_gcode(path):
    """
    Parse G-code and return a list of dicts:
        { 'x', 'y', 'z', 'e_delta', 'is_travel' }

    Both extrusion moves (e_delta > 0) and travel moves (e_delta == 0) are
    included so the executor can form continuous per-segment paths.
    Only G0/G1 moves with an XY component are returned.
    """
    waypoints = []

    cur_x = cur_y = cur_z = 0.0
    cur_e = 0.0
    absolute_xyz = True   # G90
    absolute_e   = True   # M82

    with open(path, 'r') as fh:
        for raw in fh:
            line = raw.split(';')[0].strip().upper()
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
                if 'E' in line:
                    cur_e = _val(line, 'E', 0.0)
                continue

            if not (line.startswith('G0 ') or line.startswith('G1 ')
                    or line == 'G0' or line == 'G1'):
                continue

            nx = _val(line, 'X', None)
            ny = _val(line, 'Y', None)
            nz = _val(line, 'Z', None)
            ne = _val(line, 'E', None)

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

            # Record all moves that change XY (both travel and extrusion)
            if nx is not None or ny is not None:
                is_travel = e_delta <= 1e-6
                waypoints.append({
                    'x':        new_x,
                    'y':        new_y,
                    'z':        new_z,
                    'e_delta':  max(e_delta, 0.0),
                    'is_travel': is_travel,
                })

            cur_x, cur_y, cur_z, cur_e = new_x, new_y, new_z, new_e

    return waypoints


def downsample(pts, min_dist_m):
    """
    Downsample extrusion moves using a minimum-distance filter.

    Rules:
      - Travel moves are always kept (they mark segment transitions).
      - Any type boundary (travel→extrusion or extrusion→travel) always keeps
        both the last point of the previous group and the first point of the
        next group.
      - Within a continuous extrusion run, points closer than min_dist_m to
        the previous kept point are merged (their e_delta is accumulated into
        the next kept point).
    """
    if not pts:
        return pts

    kept  = [pts[0].copy()]
    acc_e = pts[0]['e_delta']

    for p in pts[1:]:
        prev = kept[-1]

        # ── Type boundary: flush and keep ────────────────────────────────────
        if p['is_travel'] != prev['is_travel']:
            # Flush any accumulated extrusion into the last kept point
            kept[-1]['e_delta'] = acc_e
            kept.append(p.copy())
            acc_e = p['e_delta']
            continue

        # ── Travel move: always keep ──────────────────────────────────────────
        if p['is_travel']:
            kept[-1]['e_delta'] = acc_e
            kept.append(p.copy())
            acc_e = p['e_delta']
            continue

        # ── Extrusion move: apply distance filter ────────────────────────────
        rx_prev, ry_prev, _ = gcode_to_robot(prev['x'], prev['y'], prev['z'])
        rx_cur,  ry_cur,  _ = gcode_to_robot(p['x'],   p['y'],   p['z'])
        d = math.sqrt((rx_cur - rx_prev)**2 + (ry_cur - ry_prev)**2)
        acc_e += p['e_delta']

        if d >= min_dist_m:
            p_copy = p.copy()
            p_copy['e_delta'] = acc_e
            kept.append(p_copy)
            acc_e = 0.0

    # Flush any remaining accumulated extrusion into the last kept point
    kept[-1]['e_delta'] = kept[-1]['e_delta'] + acc_e if acc_e > 0 else kept[-1]['e_delta']

    return kept


def write_csv(waypoints, out_path):
    extrusion_count = sum(1 for p in waypoints if not p['is_travel'])
    travel_count    = sum(1 for p in waypoints if p['is_travel'])

    lines = [
        "# Cartesian waypoints (continuous) — auto-generated by gcode_to_waypoints_continuous.py",
        "# Format: x, y, z, qx, qy, qz, qw, extrude_mm, velocity_scale, is_travel",
        "#   is_travel: 0 = extrusion move, 1 = travel (repositioning) move",
        f"# Source transform: gcode_center=({GCODE_CENTER_X},{GCODE_CENTER_Y}) mm  "
        f"robot_center=({ROBOT_CENTER_X},{ROBOT_CENTER_Y}) m  z_offset={ROBOT_Z_OFFSET} m",
        f"# Fine-tune offsets: X={X_OFFSET:+.4f} m  Y={Y_OFFSET:+.4f} m  Z={Z_OFFSET:+.4f} m",
        f"# Total points: {len(waypoints)}  extrusion: {extrusion_count}  travel: {travel_count}  "
        f"min_segment: {MIN_SEGMENT_M*1000:.1f} mm",
    ]

    if USE_BED_LEVELING:
        _, _, (qx, qy, qz, qw) = _get_bed_transform()
    else:
        qx, qy, qz, qw = QX, QY, QZ, QW

    for p in waypoints:
        rx, ry, rz = gcode_to_robot(p['x'], p['y'], p['z'])

        if EXTRUDE_MM == "auto":
            extrude = p['e_delta']
        else:
            extrude = float(EXTRUDE_MM) if not p['is_travel'] else 0.0

        is_travel_flag = 1 if p['is_travel'] else 0

        lines.append(
            f"{rx:.6f}, {ry:.6f}, {rz:.6f}, "
            f"{qx:.6f}, {qy:.6f}, {qz:.6f}, {qw:.6f}, "
            f"{extrude:.5f}, {VELOCITY_SCALE}, {is_travel_flag}"
        )

    with open(out_path, 'w') as fh:
        fh.write('\n'.join(lines) + '\n')

    print(f"Wrote {len(waypoints)} waypoints → {out_path}")
    print(f"  Extrusion moves : {extrusion_count}")
    print(f"  Travel moves    : {travel_count}")


def main():
    default_gcode = os.path.join(os.path.dirname(__file__),
                                 'Ian', 'UR5e-Dev', '40x40x0.9 Box.gcode')
    default_csv   = os.path.join(os.path.dirname(__file__), 'waypoints_continuous.csv')

    in_path  = sys.argv[1] if len(sys.argv) > 1 else default_gcode
    out_path = sys.argv[2] if len(sys.argv) > 2 else default_csv

    if not os.path.exists(in_path):
        print(f"ERROR: G-code file not found: {in_path}")
        print("Usage: python3 gcode_to_waypoints_continuous.py <input.gcode> [output.csv]")
        sys.exit(1)

    print(f"Parsing: {in_path}")
    raw = parse_gcode(in_path)
    extrusion_raw = sum(1 for p in raw if not p['is_travel'])
    travel_raw    = sum(1 for p in raw if p['is_travel'])
    print(f"  Raw moves:  {len(raw)}  (extrusion: {extrusion_raw}, travel: {travel_raw})")

    filtered = downsample(raw, MIN_SEGMENT_M)
    extrusion_filt = sum(1 for p in filtered if not p['is_travel'])
    travel_filt    = sum(1 for p in filtered if p['is_travel'])
    print(f"  After downsampling: {len(filtered)}  (extrusion: {extrusion_filt}, travel: {travel_filt})")

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
