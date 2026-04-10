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
import os
import sys

# ─── Bed origin settings ──────────────────────────────────────────────────────
# Position of the gcode origin (0, 0, 0) in the robot's base_link frame.
# Units: metres.  Adjust these to match your physical setup.
BED_ORIGIN_X =  0.500   # metres
BED_ORIGIN_Y =  0.000   # metres
BED_ORIGIN_Z =  0.0788   # metres

# ─── Future: bed orientation ──────────────────────────────────────────────────
# Bed tilt and nozzle-normal orientation will be added here in a future update.
# When implemented, this section will:
#   - Accept a bed normal vector or rotation (roll, pitch, yaw)
#   - Rotate every XYZ point into the tilted bed frame
#   - Update QX/QY/QZ/QW so the nozzle stays perpendicular to the bed surface
# ──────────────────────────────────────────────────────────────────────────────


def transform_csv(in_path: str, out_path: str) -> int:
    """
    Read *in_path*, shift every X/Y/Z by the bed origin offset, write *out_path*.
    Returns the number of waypoints transformed.
    """
    # Convert bed origin from metres to mm to match CSV units
    dx = BED_ORIGIN_X * 1000.0
    dy = BED_ORIGIN_Y * 1000.0
    dz = BED_ORIGIN_Z * 1000.0

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

            transformed = [
                f"{x:.4f}", f"{y:.4f}", f"{z:.4f}",
                *row[3:]   # QX QY QZ QW, speeds, move_type, segment_id, gcode_line
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

    count = transform_csv(in_path, out_path)
    print(f'Transformed {count} waypoints → {out_path}')


if __name__ == '__main__':
    main()
