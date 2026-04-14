#!/usr/bin/env python3
"""
bed_transform.py
================
Applies an XYZ origin transform to a gcode_interpreter.py CSV output,
aligning the gcode coordinate origin with the robot bed position.

The bed origin defines where gcode (0, 0, 0) sits in the robot's base_link
frame (metres).  Every X/Y/Z point in the CSV is shifted by this offset.
All other columns (orientation, speeds, move type, etc.) are preserved as-is.

Output
------
A new CSV with the same format, saved as <input_name>.transformed.csv.
This file can be passed directly to gcode_executor.py.

Usage
-----
  python3 bed_transform.py <interpreted.csv>

Example
-------
  python3 bed_transform.py "gcode_interpreted files/40x40x40.csv"
  → outputs: "gcode_interpreted files/40x40x40.transformed.csv"
"""

import csv
import math
import os
import sys

# ─── Bed origin settings ──────────────────────────────────────────────────────
# Position of the gcode origin (0, 0, 0) in the robot's base_link frame.
# Units: metres.  Adjust these to match your physical setup.
BED_ORIGIN_X =  -0.748   # metres
BED_ORIGIN_Y =  -0.25   # metres
BED_ORIGIN_Z =  -0.5315   # metres

# ─── Nozzle orientation override ─────────────────────────────────────────────
# Set OVERRIDE_ORIENTATION = True to replace every waypoint's quaternion with
# the values below.  Leave False to keep the orientations from the interpreter.
# Units: unit quaternion (will be normalised automatically).
OVERRIDE_ORIENTATION = True

TARGET_QX =  0.5
TARGET_QY =  -0.5
TARGET_QZ =  -0.5   #change z to change the orientation abotu z axis
TARGET_QW =  0.5
# ─────────────────────────────────────────────────────────────────────────────


def transform_csv(in_path: str, out_path: str) -> int:
    """
    Read *in_path*, shift every X/Y/Z by the bed origin offset, write *out_path*.
    If OVERRIDE_ORIENTATION is True, replaces QX/QY/QZ/QW on every waypoint.
    Returns the number of waypoints transformed.
    """
    # Convert bed origin from metres to mm to match CSV units
    dx = BED_ORIGIN_X * 1000.0
    dy = BED_ORIGIN_Y * 1000.0
    dz = BED_ORIGIN_Z * 1000.0

    # Normalise the override quaternion once
    if OVERRIDE_ORIENTATION:
        mag = math.sqrt(TARGET_QX**2 + TARGET_QY**2 + TARGET_QZ**2 + TARGET_QW**2)
        if mag < 1e-9:
            raise ValueError('TARGET quaternion has zero magnitude.')
        q_str = [
            f'{TARGET_QX / mag:.6f}',
            f'{TARGET_QY / mag:.6f}',
            f'{TARGET_QZ / mag:.6f}',
            f'{TARGET_QW / mag:.6f}',
        ]

    output_rows = []
    count       = 0

    with open(in_path, 'r') as fh:
        for raw_row in csv.reader(fh):
            row = [c.strip() for c in raw_row]

            # Preserve comment and header lines unchanged
            if not row or row[0].startswith('#') or row[0].upper() == 'X':
                output_rows.append(raw_row)
                continue

            if len(row) < 13:
                output_rows.append(raw_row)
                continue

            try:
                x = float(row[0]) + dx
                y = float(row[1]) + dy
                z = float(row[2]) + dz
            except ValueError:
                output_rows.append(raw_row)
                continue

            orientation = q_str if OVERRIDE_ORIENTATION else row[3:7]

            transformed = [
                f"{x:.4f}", f"{y:.4f}", f"{z:.4f}",
                *orientation,  # QX QY QZ QW
                *row[7:]       # speeds, move_type, segment_id, gcode_line
            ]
            output_rows.append(transformed)
            count += 1

    with open(out_path, 'w', newline='') as fh:
        csv.writer(fh).writerows(output_rows)

    return count


def main():
    if len(sys.argv) < 2:
        print('Usage: python3 bed_transform.py <interpreted.csv>')
        sys.exit(1)

    in_path = sys.argv[1]
    if not os.path.isfile(in_path):
        print(f'ERROR: File not found: {in_path}')
        sys.exit(1)

    # Build output path: same directory, .transformed.csv extension
    base, _ = os.path.splitext(in_path)
    if base.endswith('.transformed'):
        base = base[: -len('.transformed')]
    out_path = base + '.transformed.csv'

    print(f'Input    : {in_path}')
    print(f'Output   : {out_path}')
    print(f'Bed origin (base_link frame):  '
          f'X={BED_ORIGIN_X:.4f} m  '
          f'Y={BED_ORIGIN_Y:.4f} m  '
          f'Z={BED_ORIGIN_Z:.4f} m')
    print(f'Offset applied:  '
          f'ΔX={BED_ORIGIN_X*1000:.2f} mm  '
          f'ΔY={BED_ORIGIN_Y*1000:.2f} mm  '
          f'ΔZ={BED_ORIGIN_Z*1000:.2f} mm')
    if OVERRIDE_ORIENTATION:
        print(f'Orientation override: QX={TARGET_QX}  QY={TARGET_QY}  '
              f'QZ={TARGET_QZ}  QW={TARGET_QW}')
    else:
        print('Orientation: preserved from interpreter CSV')

    count = transform_csv(in_path, out_path)
    print(f'Transformed {count} waypoints → {out_path}')


if __name__ == '__main__':
    main()
