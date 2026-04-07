#!/usr/bin/env python3
"""
gcode_interpreter.py
====================
Parse a G-code file and output a CSV of waypoints with:
  X, Y, Z, QX, QY, QZ, QW, Toolhead_Speed_mm_per_min,
  Extrusion_Length_mm, Extrusion_Speed_mm_per_min, Move_Type, Segment_ID

Units
-----
  X, Y, Z              : millimetres
  Toolhead_Speed       : mm / min  (from F value)
  Extrusion_Length     : mm        (delta-E between consecutive points)
  Extrusion_Speed      : mm / min  (calculated from geometry + speed)

Usage
-----
  python3 gcode_interpreter.py  [input.gcode]  [output.csv]

Defaults:
  input  = (set INPUT_GCODE below, or pass as first argument)
  output = output.csv  (in the same directory as this script)
"""

import sys
import os
import math

# ─── User settings ────────────────────────────────────────────────────────────

# Default input G-code path (overridden by command-line arg 1)
INPUT_GCODE = "print.gcode"

# Default output directory — relative to this script's location in the repo.
# Assumes the script lives alongside the "gcode_interpreted files" folder.
OUTPUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "gcode_interpreted files")

# Default output CSV path (overridden by command-line arg 2)
# Output filename matches the input gcode filename with a .csv extension
OUTPUT_CSV = None  # resolved at runtime from input filename

# Tool orientation quaternion (tool pointing straight down)
QX = 0.70711
QY = 0.00056
QZ = 0.00056
QW = 0.70711

# Machine default toolhead speed (mm/min) used when F is 0 or not yet set
DEFAULT_SPEED_MM_PER_MIN = 1000.0

# ──────────────────────────────────────────────────────────────────────────────


def _val(line, letter, default=None):
    """Extract float value for a G-code letter (e.g. X, Y, E, F, Z)."""
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
    Parse G-code and return a list of waypoint dicts:
        {
            'x', 'y', 'z',
            'speed',         # toolhead speed (mm/min)
            'e_delta',       # extrusion length this segment (mm)
            'move_type',     # 'extrusion' or 'travel'
        }
    """
    waypoints = []

    cur_x = 0.0
    cur_y = 0.0
    cur_z = 0.0
    cur_e = 0.0
    cur_speed = DEFAULT_SPEED_MM_PER_MIN  # mm/min

    IGNORED_PREFIXES = ('M190', 'M104', 'M109', 'M82', 'G21', 'G90', 'G91',
                        'G28', 'G29', 'G0 ', 'G0\n', 'T0', 'T1')

    with open(path, 'r') as fh:
        for line_num, raw in enumerate(fh, start=1):
            # Strip comments and whitespace
            line = raw.split(';')[0].strip()
            if not line:
                continue

            upper = line.upper()

            # ── Ignore miscellaneous commands ──
            if any(upper.startswith(p) for p in IGNORED_PREFIXES):
                continue

            # ── G92: reset extrusion counter ──
            if upper.startswith('G92'):
                e_val = _val(upper, 'E', None)
                if e_val is not None:
                    cur_e = e_val
                continue

            # ── G1: the main move command ──
            if upper.startswith('G1') and (len(upper) == 2 or upper[2] == ' '):
                nx = _val(upper, 'X', None)
                ny = _val(upper, 'Y', None)
                nz = _val(upper, 'Z', None)
                ne = _val(upper, 'E', None)
                nf = _val(upper, 'F', None)

                # Always update speed if F is present
                if nf is not None:
                    cur_speed = nf if nf > 0 else DEFAULT_SPEED_MM_PER_MIN

                # Only Z update — not a waypoint
                if nz is not None and nx is None and ny is None:
                    cur_z = nz
                    continue

                # Only F update — not a waypoint
                if nx is None and ny is None and nz is None:
                    continue

                # XY move → waypoint
                if nx is not None or ny is not None:
                    new_x = nx if nx is not None else cur_x
                    new_y = ny if ny is not None else cur_y
                    new_z = nz if nz is not None else cur_z
                    new_e = ne if ne is not None else cur_e

                    e_delta = new_e - cur_e

                    # Move type
                    if ne is not None and e_delta > 1e-9:
                        move_type = 'extrusion'
                    else:
                        move_type = 'travel'

                    # Skip travel waypoints that are at the same position as the
                    # previous point — these are just speed-definition lines at
                    # the start of a travel move. Speed has already been updated.
                    same_pos = (
                        abs(new_x - cur_x) < 1e-6 and
                        abs(new_y - cur_y) < 1e-6 and
                        abs(new_z - cur_z) < 1e-6
                    )
                    if move_type == 'travel' and same_pos:
                        cur_e = new_e
                        continue

                    waypoints.append({
                        'x': new_x,
                        'y': new_y,
                        'z': new_z,
                        'speed': cur_speed,
                        'e_delta': max(e_delta, 0.0),
                        'move_type': move_type,
                        'gcode_line': line_num,
                    })

                    cur_x, cur_y, cur_z, cur_e = new_x, new_y, new_z, new_e

    return waypoints


def assign_segments(waypoints):
    """
    Group consecutive waypoints of the same move_type into segments.
    Returns list of waypoints with 'segment_id' added.
    """
    if not waypoints:
        return waypoints

    segment_id = 0
    prev_type = waypoints[0]['move_type']
    waypoints[0]['segment_id'] = segment_id

    for wp in waypoints[1:]:
        if wp['move_type'] != prev_type:
            segment_id += 1
            prev_type = wp['move_type']
        wp['segment_id'] = segment_id

    return waypoints


def compute_extrusion_speed(waypoints):
    """
    Calculate extrusion_speed (mm/min) for each waypoint based on:
      - 3D distance from previous waypoint
      - toolhead speed
      - extrusion length (e_delta)
    First waypoint has extrusion_speed = 0.
    """
    if not waypoints:
        return waypoints

    waypoints[0]['extrusion_speed'] = 0.0

    for i in range(1, len(waypoints)):
        prev = waypoints[i - 1]
        cur  = waypoints[i]

        dx = cur['x'] - prev['x']
        dy = cur['y'] - prev['y']
        dz = cur['z'] - prev['z']
        distance = math.sqrt(dx*dx + dy*dy + dz*dz)  # mm

        speed = cur['speed']  # mm/min

        if distance > 1e-9 and speed > 1e-9:
            segment_time_min = distance / speed        # minutes
            extrusion_speed = cur['e_delta'] / segment_time_min  # mm/min
        else:
            extrusion_speed = 0.0

        cur['extrusion_speed'] = extrusion_speed

    return waypoints


def write_csv(waypoints, out_path):
    header_lines = [
        "# G-code interpreter output — auto-generated by gcode_interpreter.py",
        "# Units: X/Y/Z in mm | Toolhead_Speed in mm/min | Extrusion_Length in mm | Extrusion_Speed in mm/min",
        "X,Y,Z,QX,QY,QZ,QW,Toolhead_Speed_mm_per_min,Extrusion_Length_mm,Extrusion_Speed_mm_per_min,Move_Type,Segment_ID,GCode_Line",
    ]

    rows = []
    for wp in waypoints:
        rows.append(
            f"{wp['x']:.4f},{wp['y']:.4f},{wp['z']:.4f},"
            f"{QX:.6f},{QY:.6f},{QZ:.6f},{QW:.6f},"
            f"{wp['speed']:.4f},"
            f"{wp['e_delta']:.6f},"
            f"{wp['extrusion_speed']:.6f},"
            f"{wp['move_type']},"
            f"{wp['segment_id']},"
            f"{wp['gcode_line']}"
        )

    with open(out_path, 'w') as fh:
        fh.write('\n'.join(header_lines) + '\n')
        fh.write('\n'.join(rows) + '\n')

    print(f"Wrote {len(waypoints)} waypoints → {out_path}")


def main():
    in_path  = sys.argv[1] if len(sys.argv) > 1 else INPUT_GCODE
    if len(sys.argv) > 2:
        out_path = sys.argv[2]
    else:
        gcode_name = os.path.splitext(os.path.basename(in_path))[0]
        out_path = os.path.join(OUTPUT_DIR, gcode_name + ".csv")

    if not os.path.exists(in_path):
        print(f"ERROR: G-code file not found: {in_path}")
        print("Usage: python3 gcode_interpreter.py <input.gcode> [output.csv]")
        sys.exit(1)

    print(f"Parsing: {in_path}")
    waypoints = parse_gcode(in_path)
    print(f"  Total waypoints: {len(waypoints)}")

    waypoints = assign_segments(waypoints)
    waypoints = compute_extrusion_speed(waypoints)

    extrusion_count = sum(1 for wp in waypoints if wp['move_type'] == 'extrusion')
    travel_count    = sum(1 for wp in waypoints if wp['move_type'] == 'travel')
    segment_count   = waypoints[-1]['segment_id'] + 1 if waypoints else 0
    print(f"  Extrusion moves: {extrusion_count}  |  Travel moves: {travel_count}")
    print(f"  Segments: {segment_count}")

    write_csv(waypoints, out_path)


if __name__ == '__main__':
    main()
